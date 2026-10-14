// ============================================================
// SPACE DOCKING SYSTEM: DYNAMICS & PHYSICS ENGINE (SERVER)
// ============================================================
// Runs rigid-body physics, contact dynamics, precision microsecond timing,
// and disk logging. Communicates simulation state to the UI layer over UDP.
// ============================================================

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <algorithm>
#include <thread>
#include <atomic>
#include <fstream>
#include <sstream>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#endif

#include "common/math_types.hpp"
#include "common/protocol.hpp"
#include "common/net.hpp"

// ============================================================
// 1. RIGID BODY
// ============================================================
struct RigidBody
{
    std::string name;
    double mass;
    Vec3 inertia;
    Vec3 position;
    Vec3 velocity;
    Quat attitude;
    Vec3 angularVel;
    Vec3 size;
};

// ============================================================
// 2. LOAD CELL POSITIONS (HEXAGONAL DOCKING FIXTURE)
// ============================================================
const Vec3 g_loadCellPos[6] =
{
    Vec3(0.0,  0.50,  0.000),
    Vec3(0.0,  0.25,  0.433),
    Vec3(0.0, -0.25,  0.433),
    Vec3(0.0, -0.50,  0.000),
    Vec3(0.0, -0.25, -0.433),
    Vec3(0.0,  0.25, -0.433)
};

void DistributeContactForceToLoadCells(
    const Vec3& totalContactForce,
    const Vec3& contactPt,
    Vec3 outForces[6])
{
    Vec3 torque = contactPt.Cross(totalContactForce);

    double sumY2 = 0.0;
    double sumZ2 = 0.0;

    for (int i = 0; i < 6; ++i)
    {
        sumY2 += g_loadCellPos[i].y * g_loadCellPos[i].y;
        sumZ2 += g_loadCellPos[i].z * g_loadCellPos[i].z;
    }

    for (int i = 0; i < 6; ++i)
    {
        double fNormal =
            totalContactForce.x / 6.0
            + torque.y * g_loadCellPos[i].z / sumZ2
            - torque.z * g_loadCellPos[i].y / sumY2;

        double fShearY = totalContactForce.y / 6.0;
        double fShearZ = totalContactForce.z / 6.0;

        outForces[i] = Vec3(fNormal, fShearY, fShearZ);
    }
}

struct ContactEstimationResult
{
    Vec3 F_net;
    Vec3 tau_net;
    Vec3 Pc_est;
    Vec3 a_CM;
    Vec3 alpha;
    Vec3 delta_x;
    Vec3 delta_theta;
};

ContactEstimationResult ComputePostContactPose(
    const Vec3 forces[6],
    double mass,
    const Vec3& inertia,
    double dt)
{
    ContactEstimationResult result;

    result.F_net = Vec3(0, 0, 0);
    for (int i = 0; i < 6; ++i)
        result.F_net += forces[i];

    result.tau_net = Vec3(0, 0, 0);
    for (int i = 0; i < 6; ++i)
        result.tau_net += g_loadCellPos[i].Cross(forces[i]);

    double forceSquared = result.F_net.Dot(result.F_net);

    if (forceSquared > 1e-12)
        result.Pc_est = result.F_net.Cross(result.tau_net) * (1.0 / forceSquared);
    else
        result.Pc_est = Vec3(0, 0, 0);

    result.a_CM = result.F_net * (1.0 / mass);

    result.alpha = Vec3(
        result.tau_net.x / inertia.x,
        result.tau_net.y / inertia.y,
        result.tau_net.z / inertia.z
    );

    double dtSquaredHalf = 0.5 * dt * dt;

    result.delta_x = result.a_CM * dtSquaredHalf;
    result.delta_theta = result.alpha * dtSquaredHalf;

    return result;
}

// ============================================================
// 3. CONFIG & STATS
// ============================================================
constexpr int DEFAULT_PHYSICS_PERIOD_US = 1000;
constexpr int MIN_PHYSICS_PERIOD_US     = 50;
constexpr int MAX_PHYSICS_PERIOD_US     = 100000;
constexpr int PERIOD_HIST_BUCKETS       = 400;

struct SimConfig
{
    int    physicsPeriodUs  = DEFAULT_PHYSICS_PERIOD_US;
    double durationSeconds  = 0.0;     // 0 = run indefinitely until Ctrl+C
    bool   durationSpecified= false;
    bool   logEnabled       = true;
    bool   timingLogEnabled = true;
    int    physicsCpu       = -1;
    bool   spin             = false;
    double tolUs            = 50.0;
    int    logEvery         = 1;
    bool   realtime         = true;
    bool   spinForced       = false;
    bool   autoTune         = true;
    int    warmupMs         = 300;
    std::string uiHost      = SIM_DEFAULT_HOST;
    int    statePort        = SIM_STATE_PORT;
    int    cmdPort          = SIM_COMMAND_PORT;
};

SimConfig g_config;

struct alignas(64) TimingStats
{
    std::atomic<uint32_t> windowAvgPeriodNs{0};
    std::atomic<uint32_t> windowMaxPeriodNs{0};
    std::atomic<uint32_t> windowMaxWorkNs{0};
    std::atomic<uint32_t> windowMaxLatencyNs{0};
    std::atomic<uint64_t> latencyViolations{0};
    std::atomic<uint64_t> lateTicks{0};
    std::atomic<uint64_t> droppedLogRows{0};
    std::atomic<uint64_t> droppedTimingRows{0};
    std::atomic<uint64_t> workViolations{0};

    uint64_t runSamples = 0;
    uint64_t runSumPeriodNs = 0;
    uint32_t runMaxPeriodNs = 0;
    uint32_t runMaxWorkNs = 0;
    uint32_t runMaxLatencyNs = 0;
    std::array<uint32_t, PERIOD_HIST_BUCKETS> periodHist{};
};

TimingStats g_timing;

// ============================================================
// 4. LOCK-FREE RING BUFFER FOR LOGGING
// ============================================================
struct LogRecord
{
    double timestamp;
    Vec3 cube1Position;
    Quat cube1Orientation;
    Vec3 cube2Position;
    Quat cube2Orientation;
    uint64_t physicsStep;
};

struct TimingRecord
{
    uint64_t step;
    int64_t  timestampNs;
    int64_t  periodNs;
    int64_t  latencyNs;
};

template <typename T, size_t N>
class SpscRing
{
    static_assert((N & (N - 1)) == 0, "N must be a power of two");
public:
    bool Push(const T& v)
    {
        const size_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) == N)
            return false;
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    void Prefault()
    {
        volatile unsigned char* p = reinterpret_cast<volatile unsigned char*>(buf_.data());
        for (size_t i = 0; i < sizeof(buf_); i += 4096)
            p[i] = p[i];
    }

    bool Pop(T& out)
    {
        const size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire))
            return false;
        out = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

private:
    std::array<T, N> buf_{};
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

SpscRing<LogRecord, (1u << 15)> g_logRing;
SpscRing<TimingRecord, (1u << 16)> g_timingRing;

// ============================================================
// 5. GLOBAL SIMULATION STATE & COMMANDS
// ============================================================
enum class Scenario { HeadOn = 1, OffCenter = 2 };

Scenario g_currentScenario = Scenario::HeadOn;
RigidBody g_target;
RigidBody g_chaser;
bool g_contactOccurred = false;
bool g_paused = false;

std::atomic<bool> g_running{true};
std::atomic<bool> g_loggerStop{false};
std::atomic<int>  g_requestedScenario{1};
std::atomic<bool> g_resetRequested{false};
std::atomic<bool> g_pauseRequested{false};

void SetupScenario(Scenario scenario)
{
    g_currentScenario = scenario;
    g_contactOccurred = false;

    // Target
    g_target.name = "Target";
    g_target.mass = 50.0;
    g_target.inertia = Vec3(5.0, 5.0, 10.0);
    g_target.position = Vec3(0.0, 0.0, 0.0);
    g_target.velocity = Vec3(0.0, 0.0, 0.0);
    g_target.attitude = Quat(1.0, 0.0, 0.0, 0.0);
    g_target.angularVel = Vec3(0.0, 0.0, 0.0);
    g_target.size = Vec3(1.0, 1.0, 1.0);

    // Chaser
    g_chaser.name = "Chaser";
    g_chaser.mass = 50.0;
    g_chaser.inertia = Vec3(5.0, 5.0, 10.0);
    g_chaser.velocity = Vec3(0.5, 0.0, 0.0);
    g_chaser.attitude = Quat(1.0, 0.0, 0.0, 0.0);
    g_chaser.angularVel = Vec3(0.0, 0.0, 0.0);
    g_chaser.size = Vec3(1.0, 1.0, 1.0);

    if (scenario == Scenario::HeadOn)
    {
        g_chaser.position = Vec3(-3.0, 0.0, 0.0);
        std::cout << "[Physics] Active Scenario 1: HEAD-ON\n";
    }
    else
    {
        g_chaser.position = Vec3(-3.0, 0.35, 0.15);
        std::cout << "[Physics] Active Scenario 2: OFF-CENTER\n";
    }
}

void ProcessCommands(UdpSocket& cmdSocket)
{
    // 1. Drain any pending network commands sent from UI
    CommandPacket cmdPkt;
    while (cmdSocket.RecvFrom(&cmdPkt, sizeof(cmdPkt)) == sizeof(cmdPkt))
    {
        if (cmdPkt.magic != CMD_PACKET_MAGIC)
            continue;

        if (cmdPkt.cmd == SimCommand::Scenario1)
        {
            g_requestedScenario = 1;
            g_resetRequested = true;
        }
        else if (cmdPkt.cmd == SimCommand::Scenario2)
        {
            g_requestedScenario = 2;
            g_resetRequested = true;
        }
        else if (cmdPkt.cmd == SimCommand::Reset)
        {
            g_resetRequested = true;
        }
        else if (cmdPkt.cmd == SimCommand::TogglePause)
        {
            g_pauseRequested = true;
        }
    }

    // 2. Apply commands
    if (g_resetRequested.exchange(false))
    {
        int scenario = g_requestedScenario.load();
        SetupScenario(scenario == 1 ? Scenario::HeadOn : Scenario::OffCenter);
    }

    if (g_pauseRequested.exchange(false))
    {
        g_paused = !g_paused;
        std::cout << (g_paused ? ">>> [Physics] PAUSED\n" : ">>> [Physics] RESUMED\n");
    }
}

void UpdatePhysics(double dt)
{
    if (g_paused)
        return;

    // AABB Overlap
    double overlapX =
        (g_chaser.position.x + g_chaser.size.x * 0.5)
        - (g_target.position.x - g_target.size.x * 0.5);

    double overlapY =
        (g_chaser.size.y * 0.5 + g_target.size.y * 0.5)
        - std::abs(g_chaser.position.y - g_target.position.y);

    double overlapZ =
        (g_chaser.size.z * 0.5 + g_target.size.z * 0.5)
        - std::abs(g_chaser.position.z - g_target.position.z);

    bool isTouching = overlapX >= 0.0 && overlapY > 0.0 && overlapZ > 0.0;

    // Contact response
    if (!g_contactOccurred && isTouching)
    {
        g_contactOccurred = true;

        double contactY = (g_chaser.position.y + g_target.position.y) * 0.5;
        double contactZ = (g_chaser.position.z + g_target.position.z) * 0.5;

        Vec3 contactPoint(0.0, contactY, contactZ);
        double restitution = 0.85;
        double relativeVelocity = g_chaser.velocity.x - g_target.velocity.x;
        double reducedMass = (g_chaser.mass * g_target.mass) / (g_chaser.mass + g_target.mass);
        double impulse = (1.0 + restitution) * reducedMass * relativeVelocity;
        double contactForce = impulse / dt;

        Vec3 totalContactForce(contactForce, 0.0, 0.0);
        Vec3 loadCellForces[6];

        DistributeContactForceToLoadCells(totalContactForce, contactPoint, loadCellForces);

        ContactEstimationResult estimate =
            ComputePostContactPose(loadCellForces, g_target.mass, g_target.inertia, dt);

        g_target.velocity += estimate.a_CM * dt;
        g_target.angularVel += estimate.alpha * dt;

        g_chaser.velocity += estimate.a_CM * (-1.0 * (g_target.mass / g_chaser.mass) * dt);
        g_chaser.angularVel += estimate.alpha * (-0.5 * dt);
    }

    g_target.position += g_target.velocity * dt;
    g_chaser.position += g_chaser.velocity * dt;

    Quat targetDeltaQuat = Quat::FromAngularDisplacement(g_target.angularVel * dt);
    g_target.attitude = (targetDeltaQuat * g_target.attitude).Normalized();

    Quat chaserDeltaQuat = Quat::FromAngularDisplacement(g_chaser.angularVel * dt);
    g_chaser.attitude = (chaserDeltaQuat * g_chaser.attitude).Normalized();
}

// ============================================================
// 6. LOGGER THREAD
// ============================================================
void LoggerThread()
{
    if (!g_config.logEnabled && !g_config.timingLogEnabled)
        return;

    std::ofstream logFile;
    std::ofstream timingFile;

    if (g_config.logEnabled)
    {
        logFile.open("simulation.log");
        if (logFile.is_open())
        {
            logFile << "timestamp,cube1_x,cube1_y,cube1_z,cube1_qw,cube1_qx,cube1_qy,cube1_qz,"
                    << "cube2_x,cube2_y,cube2_z,cube2_qw,cube2_qx,cube2_qy,cube2_qz,physics_step\n";
            logFile << std::fixed << std::setprecision(9);
        }
    }

    if (g_config.timingLogEnabled)
    {
        timingFile.open("timestep.csv");
        if (timingFile.is_open())
        {
            timingFile << "step,timestamp_s,period_us,error_us,latency_us\n";
        }
    }

    constexpr int FLUSH_EVERY_N_ROWS = 2000;
    int rowsSinceFlush = 0;
    const double targetUs = static_cast<double>(g_config.physicsPeriodUs);

    LogRecord record;
    TimingRecord trec;
    char line[128];

    auto drain = [&]() -> bool
    {
        bool any = false;

        while (g_logRing.Pop(record))
        {
            any = true;
            logFile << record.timestamp << ","
                    << record.cube1Position.x << "," << record.cube1Position.y << "," << record.cube1Position.z << ","
                    << record.cube1Orientation.w << "," << record.cube1Orientation.x << "," << record.cube1Orientation.y << "," << record.cube1Orientation.z << ","
                    << record.cube2Position.x << "," << record.cube2Position.y << "," << record.cube2Position.z << ","
                    << record.cube2Orientation.w << "," << record.cube2Orientation.x << "," << record.cube2Orientation.y << "," << record.cube2Orientation.z << ","
                    << record.physicsStep << "\n";

            if (++rowsSinceFlush >= FLUSH_EVERY_N_ROWS)
            {
                logFile.flush();
                rowsSinceFlush = 0;
            }
        }

        while (g_timingRing.Pop(trec))
        {
            any = true;
            const double periodUs = static_cast<double>(trec.periodNs) / 1000.0;
            const double errorUs  = trec.periodNs == 0 ? 0.0 : periodUs - targetUs;

            const int n = std::snprintf(line, sizeof(line), "%llu,%.9f,%.3f,%.3f,%.3f\n",
                static_cast<unsigned long long>(trec.step),
                static_cast<double>(trec.timestampNs) * 1e-9,
                trec.periodNs == 0 ? targetUs : periodUs,
                errorUs,
                static_cast<double>(trec.latencyNs) / 1000.0);

            timingFile.write(line, n);
        }

        return any;
    };

    for (;;)
    {
        if (!drain())
        {
            if (g_loggerStop.load(std::memory_order_acquire))
            {
                drain();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    if (logFile.is_open())    { logFile.flush(); logFile.close(); }
    if (timingFile.is_open()) { timingFile.flush(); timingFile.close(); }
}

// ============================================================
// 7. REAL-TIME LOOP & TIMING
// ============================================================
using PhysClock = std::chrono::steady_clock;

static inline void CpuRelax()
{
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    _mm_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield");
#else
    std::this_thread::yield();
#endif
}

#if defined(_WIN32)
constexpr auto SLEEP_MARGIN = std::chrono::microseconds(1500);
#else
inline std::chrono::microseconds GetSleepMargin()
{
    const int marginUs = std::min(500, std::max(100, g_config.physicsPeriodUs / 2));
    return std::chrono::microseconds(marginUs);
}
#endif

static void WaitUntil(PhysClock::time_point target)
{
#if defined(_WIN32)
    constexpr auto sleepMargin = SLEEP_MARGIN;
#else
    const auto sleepMargin = GetSleepMargin();
#endif

    for (;;)
    {
        const auto now = PhysClock::now();
        if (now >= target)
            return;

        const auto remaining = target - now;

        if (!g_config.spin && remaining > sleepMargin)
        {
#if defined(__linux__)
            const auto wakeTime = target - sleepMargin;
            const auto d = wakeTime.time_since_epoch();
            const auto sec = std::chrono::duration_cast<std::chrono::seconds>(d);
            const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(d - sec);
            struct timespec req;
            req.tv_sec = static_cast<time_t>(sec.count());
            req.tv_nsec = static_cast<long>(nsec.count());
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &req, nullptr);
#else
            std::this_thread::sleep_for(remaining - sleepMargin);
#endif
        }
        else
        {
            for (int i = 0; i < 4; ++i)
            {
                if (PhysClock::now() >= target)
                    return;
            }
            CpuRelax();
        }
    }
}

static int OnlineCpus()
{
#if defined(_WIN32)
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return static_cast<int>(si.dwNumberOfProcessors);
#elif defined(__linux__)
    const long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? static_cast<int>(n) : 1;
#else
    return 1;
#endif
}

static void ConfigureProcessForRealtime()
{
    const int ncpu = OnlineCpus();

    if (g_config.autoTune && g_config.realtime && ncpu >= 3)
    {
        if (g_config.physicsCpu < 0)
            g_config.physicsCpu = ncpu - 1;

        if (!g_config.spinForced)
        {
            long rtRuntime = 950000;
#if defined(__linux__)
            if (FILE* f = std::fopen("/proc/sys/kernel/sched_rt_runtime_us", "r"))
            {
                if (std::fscanf(f, "%ld", &rtRuntime) != 1)
                    rtRuntime = 950000;
                std::fclose(f);
            }
#endif
            g_config.spin = (rtRuntime == -1);
        }
    }

    g_logRing.Prefault();
    g_timingRing.Prefault();

    if (!g_config.realtime)
        return;

#if defined(_WIN32)
    if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS))
        SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#elif defined(__linux__)
    rlimit lim{};
    getrlimit(RLIMIT_MEMLOCK, &lim);
#ifndef MCL_ONFAULT
#define MCL_ONFAULT 4
#endif
    if (mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) != 0)
        std::cerr << "[INFO] mlockall failed (" << std::strerror(errno) << ")\n";

    static int s_cpuDmaFd = -1;
    if (s_cpuDmaFd < 0)
    {
        s_cpuDmaFd = open("/dev/cpu_dma_latency", O_RDWR | O_CLOEXEC);
        if (s_cpuDmaFd >= 0)
        {
            int32_t lat = 0;
            write(s_cpuDmaFd, &lat, sizeof(lat));
        }
    }
#endif
}

static void ConfigurePhysicsThread()
{
    {
        volatile unsigned char stack[128 * 1024];
        for (size_t i = 0; i < sizeof(stack); i += 4096)
            stack[i] = 0;
    }

#if defined(_WIN32)
    timeBeginPeriod(1);
    if (g_config.realtime)
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    if (g_config.physicsCpu >= 0)
        SetThreadAffinityMask(GetCurrentThread(), static_cast<DWORD_PTR>(1) << g_config.physicsCpu);
#elif defined(__linux__)
    prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL);
    prctl(PR_SET_NAME, "physics", 0UL, 0UL, 0UL);
    if (g_config.physicsCpu >= 0)
    {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(g_config.physicsCpu, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    }
    if (g_config.realtime)
    {
        sched_param sp{};
        sp.sched_priority = 80;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0)
        {
            sp.sched_priority = 50;
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        }
    }
#endif
}

static void RestorePhysicsThread()
{
#if defined(_WIN32)
    timeEndPeriod(1);
#endif
}

// ============================================================
// 8. TIMING REPORT
// ============================================================
static void PrintTimingSummary()
{
    const uint64_t n = g_timing.runSamples;
    if (n == 0) return;

    auto percentileUs = [&](double p) -> int
    {
        const uint64_t target = static_cast<uint64_t>(std::ceil(p * static_cast<double>(n)));
        uint64_t cumulative = 0;
        for (int b = 0; b < PERIOD_HIST_BUCKETS; ++b)
        {
            cumulative += g_timing.periodHist[b];
            if (cumulative >= target) return (b + 1) * 10;
        }
        return PERIOD_HIST_BUCKETS * 10;
    };

    const double avgUs = static_cast<double>(g_timing.runSumPeriodNs) / static_cast<double>(n) / 1000.0;

    std::printf("\n---------------- physics timing ----------------\n");
    std::printf("target period   : %d us (%.0f Hz)\n", g_config.physicsPeriodUs, 1e6 / g_config.physicsPeriodUs);
    std::printf("ticks measured  : %llu\n", static_cast<unsigned long long>(n));
    std::printf("average period  : %.2f us\n", avgUs);
    std::printf("p99 / p99.9     : <= %d us / <= %d us  (10 us buckets)\n", percentileUs(0.99), percentileUs(0.999));
    std::printf("max period      : %.1f us\n", g_timing.runMaxPeriodNs / 1000.0);
    std::printf("max start delay : %.1f us  (tick start vs. its schedule)\n", g_timing.runMaxLatencyNs / 1000.0);
    std::printf("ticks > %.0f us late: %llu\n", g_config.tolUs, static_cast<unsigned long long>(g_timing.latencyViolations.load()));
    std::printf("max work / tick : %.2f us  (physics+publish+log push%s)\n",
                g_timing.runMaxWorkNs / 1000.0,
                g_timing.runMaxWorkNs < 1000000u ? " - STRICTLY < 1 ms" : " - EXCEEDED 1 ms");
    std::printf("late ticks      : %llu  (period > 1.5x target)\n", static_cast<unsigned long long>(g_timing.lateTicks.load()));
    std::printf("dropped log rows: %llu\n", static_cast<unsigned long long>(g_timing.droppedLogRows.load()));
    std::printf("dropped timing rows: %llu\n", static_cast<unsigned long long>(g_timing.droppedTimingRows.load()));
    if (g_config.timingLogEnabled)
        std::printf("per-tick timestamps: timestep.csv  (check with: python3 check_timing.py)\n");
    std::printf("------------------------------------------------\n");
}

static void PrintUsage(const char* exe)
{
    std::cout
        << "Usage: " << exe << " [options]\n"
        << "  --dt-us N        physics period in microseconds (default " << DEFAULT_PHYSICS_PERIOD_US << ")\n"
        << "  --scenario N     1 = head-on (default), 2 = off-center\n"
        << "  --seconds S      run time in seconds (default: unlimited, run until Ctrl+C)\n"
        << "  --no-log         do not write simulation.log\n"
        << "  --no-timing-log  do not write timestep.csv\n"
        << "  --cpu N          pin physics thread to core N\n"
        << "  --spin           busy-wait the whole period (lowest jitter)\n"
        << "  --ui-host IP     UI client host (default 127.0.0.1)\n"
        << "  --state-port P   port to send simulation state to UI (default 9870)\n"
        << "  --cmd-port P     port to receive interactive commands from UI (default 9871)\n"
        << "  -h, --help       show this help\n";
}

static int ParseArgs(int argc, char** argv)
{
    auto needVal = [&](int& i, const char* name) -> const char* {
        if (i + 1 >= argc) { std::cerr << "Missing value for " << name << "\n"; return nullptr; }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        const char* v = nullptr;

        if (a == "-h" || a == "--help") { PrintUsage(argv[0]); return 0; }
        else if (a == "--no-log")        g_config.logEnabled = false;
        else if (a == "--no-timing-log") g_config.timingLogEnabled = false;
        else if (a == "--spin")          { g_config.spin = true; g_config.spinForced = true; }
        else if (a == "--no-spin")       { g_config.spin = false; g_config.spinForced = true; }
        else if (a == "--no-pin")        g_config.autoTune = false;
        else if (a == "--dt-us")         { if (!(v = needVal(i, "--dt-us"))) return 1; g_config.physicsPeriodUs = std::atoi(v); }
        else if (a == "--scenario")      { if (!(v = needVal(i, "--scenario"))) return 1; g_requestedScenario = std::atoi(v); }
        else if (a == "--seconds")       { if (!(v = needVal(i, "--seconds"))) return 1; g_config.durationSeconds = std::atof(v); g_config.durationSpecified = true; }
        else if (a == "--cpu")           { if (!(v = needVal(i, "--cpu"))) return 1; g_config.physicsCpu = std::atoi(v); }
        else if (a == "--ui-host")       { if (!(v = needVal(i, "--ui-host"))) return 1; g_config.uiHost = v; }
        else if (a == "--state-port")    { if (!(v = needVal(i, "--state-port"))) return 1; g_config.statePort = std::atoi(v); }
        else if (a == "--cmd-port")      { if (!(v = needVal(i, "--cmd-port"))) return 1; g_config.cmdPort = std::atoi(v); }
        else { std::cerr << "Unknown option: " << a << "\n"; PrintUsage(argv[0]); return 1; }
    }
    return -1;
}

// ============================================================
// 9. MAIN PHYSICS SERVER ENTRY POINT
// ============================================================
int main(int argc, char** argv)
{
    const int pr = ParseArgs(argc, argv);
    if (pr >= 0) return pr;

    ConfigureProcessForRealtime();

    std::cout
        << "====================================================\n"
        << " SPACE DOCKING PHYSICS ENGINE (SERVER)\n"
        << "====================================================\n"
        << "Physics timestep : " << g_config.physicsPeriodUs << " us ("
        << std::fixed << std::setprecision(0) << 1e6 / g_config.physicsPeriodUs << " Hz)\n"
        << std::defaultfloat << std::setprecision(6)
        << "Streaming state  : UDP -> " << g_config.uiHost << ":" << g_config.statePort << "\n"
        << "Listening cmds   : UDP <- 127.0.0.1:" << g_config.cmdPort << "\n"
        << "Logging          : " << (g_config.logEnabled ? "asynchronous -> simulation.log" : "off") << "\n"
        << "====================================================\n";

    // Setup command listening socket (non-blocking)
    UdpSocket cmdSocket;
    if (!cmdSocket.Bind(g_config.cmdPort))
    {
        std::cerr << "[WARN] Could not bind command port " << g_config.cmdPort
                  << " (another instance may be running)\n";
    }
    cmdSocket.SetNonBlocking(true);

    // Setup state broadcasting socket
    UdpSocket stateSocket;
    stateSocket.Open();

    SetupScenario(g_requestedScenario.load() == 1 ? Scenario::HeadOn : Scenario::OffCenter);

    // Start background logger thread
    std::thread loggerThread(LoggerThread);

    // Physics Loop runs on this thread
    ConfigurePhysicsThread();

    using Clock = PhysClock;
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;

    const auto    period   = std::chrono::microseconds(g_config.physicsPeriodUs);
    const int64_t periodNs = static_cast<int64_t>(g_config.physicsPeriodUs) * 1000;
    const double  dt       = g_config.physicsPeriodUs * 1e-6;
    const int     logEvery = std::max(1, g_config.logEvery);

    // Send UDP state to UI at ~60 Hz (e.g. every 16.6 ms)
    const uint64_t ticksPerNetworkUpdate =
        std::max<uint64_t>(1, static_cast<uint64_t>(16000 / g_config.physicsPeriodUs));

    const uint64_t ticksPerWindow =
        std::max<uint64_t>(1, 1000000u / static_cast<uint64_t>(g_config.physicsPeriodUs));

    const auto startTime = Clock::now();
    auto nextTick = startTime;

    Clock::time_point prevStart{};
    bool havePrev = false;
    int64_t tickPeriodNs = 0;
    uint64_t physicsStep = 0;

    uint64_t winCount = 0;
    uint64_t winSumNs = 0;
    uint32_t winMaxPeriodNs = 0;
    uint32_t winMaxWorkNs = 0;
    uint32_t winMaxLatencyNs = 0;
    const int64_t tolNs = static_cast<int64_t>(g_config.tolUs * 1000.0);
    const auto    warmup = std::chrono::milliseconds(g_config.warmupMs);

    std::cout << "[Physics] Engine running. Waiting for / streaming to UI...\n";

    while (g_running.load(std::memory_order_relaxed))
    {
        const auto tickStart = Clock::now();

        if (g_config.durationSpecified)
        {
            double elapsed = duration_cast<std::chrono::duration<double>>(tickStart - startTime).count();
            if (elapsed >= g_config.durationSeconds)
            {
                std::cout << "\n[Physics] Reached duration limit of " << g_config.durationSeconds << " s.\n";
                break;
            }
        }

        const int64_t latencyNs =
            std::max<int64_t>(0, duration_cast<nanoseconds>(tickStart - nextTick).count());

        const bool measuring = (tickStart - startTime) >= warmup;

        {
            const uint32_t l = static_cast<uint32_t>(std::min<int64_t>(latencyNs, UINT32_MAX));
            winMaxLatencyNs = std::max(winMaxLatencyNs, l);

            if (measuring)
            {
                g_timing.runMaxLatencyNs = std::max(g_timing.runMaxLatencyNs, l);
                if (latencyNs > tolNs)
                    g_timing.latencyViolations.fetch_add(1, std::memory_order_relaxed);
            }
        }

        tickPeriodNs = 0;
        if (havePrev)
        {
            const int64_t p = duration_cast<nanoseconds>(tickStart - prevStart).count();
            tickPeriodNs = p;
            const uint32_t pNs = static_cast<uint32_t>(std::min<int64_t>(p, UINT32_MAX));

            winSumNs += pNs;
            ++winCount;
            winMaxPeriodNs = std::max(winMaxPeriodNs, pNs);

            if (measuring)
            {
                g_timing.runSumPeriodNs += pNs;
                ++g_timing.runSamples;
                g_timing.runMaxPeriodNs = std::max(g_timing.runMaxPeriodNs, pNs);

                const size_t bucket = std::min<size_t>(pNs / 10000u, PERIOD_HIST_BUCKETS - 1);
                ++g_timing.periodHist[bucket];

                if (p > periodNs + periodNs / 2)
                    g_timing.lateTicks.fetch_add(1, std::memory_order_relaxed);
            }

            if (winCount >= ticksPerWindow)
            {
                g_timing.windowAvgPeriodNs.store(static_cast<uint32_t>(winSumNs / winCount), std::memory_order_relaxed);
                g_timing.windowMaxPeriodNs.store(winMaxPeriodNs, std::memory_order_relaxed);
                g_timing.windowMaxWorkNs.store(winMaxWorkNs, std::memory_order_relaxed);
                g_timing.windowMaxLatencyNs.store(winMaxLatencyNs, std::memory_order_relaxed);
                winCount = 0;
                winSumNs = 0;
                winMaxPeriodNs = 0;
                winMaxWorkNs = 0;
                winMaxLatencyNs = 0;
            }
        }

        prevStart = tickStart;
        havePrev = true;

        if (g_config.timingLogEnabled)
        {
            const TimingRecord tr{
                physicsStep,
                duration_cast<nanoseconds>(tickStart - startTime).count(),
                tickPeriodNs,
                latencyNs
            };
            if (!g_timingRing.Push(tr))
                g_timing.droppedTimingRows.fetch_add(1, std::memory_order_relaxed);
        }

        // Process network / UI commands
        ProcessCommands(cmdSocket);

        // Advance physics equations
        UpdatePhysics(dt);

        // Send state packet to UI over UDP (at ~60 Hz)
        if (physicsStep % ticksPerNetworkUpdate == 0)
        {
            SimulationPacket pkt{};
            pkt.magic = SIM_PACKET_MAGIC;
            pkt.step = physicsStep;
            pkt.simTime = duration_cast<std::chrono::duration<double>>(tickStart - startTime).count();

            pkt.targetPos[0] = g_target.position.x;
            pkt.targetPos[1] = g_target.position.y;
            pkt.targetPos[2] = g_target.position.z;
            pkt.targetQuat[0] = g_target.attitude.w;
            pkt.targetQuat[1] = g_target.attitude.x;
            pkt.targetQuat[2] = g_target.attitude.y;
            pkt.targetQuat[3] = g_target.attitude.z;
            pkt.targetVel[0] = g_target.velocity.x;
            pkt.targetVel[1] = g_target.velocity.y;
            pkt.targetVel[2] = g_target.velocity.z;

            pkt.chaserPos[0] = g_chaser.position.x;
            pkt.chaserPos[1] = g_chaser.position.y;
            pkt.chaserPos[2] = g_chaser.position.z;
            pkt.chaserQuat[0] = g_chaser.attitude.w;
            pkt.chaserQuat[1] = g_chaser.attitude.x;
            pkt.chaserQuat[2] = g_chaser.attitude.y;
            pkt.chaserQuat[3] = g_chaser.attitude.z;
            pkt.chaserVel[0] = g_chaser.velocity.x;
            pkt.chaserVel[1] = g_chaser.velocity.y;
            pkt.chaserVel[2] = g_chaser.velocity.z;

            pkt.contactOccurred = g_contactOccurred ? 1 : 0;
            pkt.paused = g_paused ? 1 : 0;
            pkt.scenario = static_cast<uint8_t>(g_currentScenario);
            pkt.secondsSpecified = g_config.durationSpecified ? 1 : 0;

            pkt.physicsPeriodUs = g_config.physicsPeriodUs;
            pkt.avgPeriodNs = g_timing.windowAvgPeriodNs.load(std::memory_order_relaxed);
            pkt.maxPeriodNs = g_timing.windowMaxPeriodNs.load(std::memory_order_relaxed);
            pkt.maxWorkNs = g_timing.windowMaxWorkNs.load(std::memory_order_relaxed);
            pkt.maxLatencyNs = g_timing.windowMaxLatencyNs.load(std::memory_order_relaxed);
            pkt.latencyViolations = g_timing.latencyViolations.load(std::memory_order_relaxed);
            pkt.lateTicks = g_timing.lateTicks.load(std::memory_order_relaxed);
            pkt.tolUs = g_config.tolUs;
            pkt.headlessSeconds = g_config.durationSeconds;

            stateSocket.SendTo(&pkt, sizeof(pkt), g_config.uiHost.c_str(), g_config.statePort);
        }

        if (g_config.logEnabled && !g_paused && (physicsStep % logEvery) == 0)
        {
            LogRecord record;
            record.timestamp = std::chrono::duration<double>(tickStart - startTime).count();
            record.cube1Position = g_target.position;
            record.cube1Orientation = g_target.attitude;
            record.cube2Position = g_chaser.position;
            record.cube2Orientation = g_chaser.attitude;
            record.physicsStep = physicsStep;

            if (!g_logRing.Push(record))
                g_timing.droppedLogRows.fetch_add(1, std::memory_order_relaxed);
        }

        physicsStep++;

        const auto workEnd = Clock::now();
        const uint32_t workNs = static_cast<uint32_t>(
            std::min<int64_t>(duration_cast<nanoseconds>(workEnd - tickStart).count(), UINT32_MAX));

        winMaxWorkNs = std::max(winMaxWorkNs, workNs);
        if (measuring)
        {
            g_timing.runMaxWorkNs = std::max(g_timing.runMaxWorkNs, workNs);
            if (workNs >= 1000000u)
                g_timing.workViolations.fetch_add(1, std::memory_order_relaxed);
        }

        nextTick += period;
        if (workEnd - nextTick > std::chrono::milliseconds(50))
        {
            nextTick = workEnd;
            havePrev = false;
        }

        WaitUntil(nextTick);
    }

    RestorePhysicsThread();

    // Stop logger thread
    g_loggerStop.store(true, std::memory_order_release);
    if (loggerThread.joinable())
        loggerThread.join();

    PrintTimingSummary();

    std::cout << "[Physics] Engine terminated.\n";
    return 0;
}
