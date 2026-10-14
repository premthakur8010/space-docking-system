#pragma once

#include <cstdint>

// ============================================================
// NETWORK PORTS FOR CLIENT-SERVER IPC
// ============================================================
constexpr int SIM_STATE_PORT   = 9870;  // Physics -> UI (telemetry & poses)
constexpr int SIM_COMMAND_PORT = 9871;  // UI -> Physics (interactive commands)
constexpr const char* SIM_DEFAULT_HOST = "127.0.0.1";

// ============================================================
// TELEMETRY PACKET (Physics -> UI)
// ============================================================
#pragma pack(push, 1)
struct SimulationPacket
{
    uint32_t magic;         // 0x53504143 ('SPAC')
    uint64_t step;
    double   simTime;

    // Cube 1 (Target)
    double targetPos[3];
    double targetQuat[4];   // w, x, y, z
    double targetVel[3];

    // Cube 2 (Chaser)
    double chaserPos[3];
    double chaserQuat[4];   // w, x, y, z
    double chaserVel[3];

    // Flags
    uint8_t contactOccurred;
    uint8_t paused;
    uint8_t scenario;
    uint8_t secondsSpecified;

    // Timing telemetry
    uint32_t physicsPeriodUs;
    uint32_t avgPeriodNs;
    uint32_t maxPeriodNs;
    uint32_t maxWorkNs;
    uint32_t maxLatencyNs;
    uint64_t latencyViolations;
    uint64_t lateTicks;
    double   tolUs;
    double   headlessSeconds;
};

// ============================================================
// COMMAND PACKET (UI -> Physics)
// ============================================================
enum class SimCommand : uint32_t
{
    Scenario1   = 1,
    Scenario2   = 2,
    Reset       = 3,
    TogglePause = 4
};

struct CommandPacket
{
    uint32_t   magic;   // 0x434D4E44 ('CMND')
    SimCommand cmd;
};
#pragma pack(pop)

constexpr uint32_t SIM_PACKET_MAGIC = 0x53504143;
constexpr uint32_t CMD_PACKET_MAGIC = 0x434D4E44;
