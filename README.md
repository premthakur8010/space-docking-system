# Spacecraft Docking System: 3D Cube Collision & Dynamics Simulation

A real-time 3D rigid-body docking and post-contact pose estimation simulation built in C++17 with OpenGL, GLFW, GLAD, and GLM.

## Features
- **3D Visualization:** Real-time 60 Hz OpenGL rendering of Chaser and Target spacecraft cubes.
- **Orientation Kinematics:** 4D unit quaternion attitude tracking avoiding Euler gimbal lock, with conversion to Euler Roll-Pitch-Yaw angles.
- **Interactive Scenarios:**
  - `[1]` **Head-on collision**: Symmetrical straight-line rebound, zero net torque.
  - `[2]` **Off-center collision**: Asymmetric impact producing net torque, lever-arm rotation, and spin.
  - `[R]` **Reset**: Re-initialize current scenario.
  - `[SPACE]` **Pause / Resume**: Toggle simulation state.
- **Contact Reconstruction:** Multi-axis force and torque distribution across a 6-cell hexagonal load-cell array, contact point estimation, and rigid-body acceleration propagation.

## Prerequisites
- CMake 3.20+
- C++17 compliant compiler (GCC, Clang, or MSVC)
- **Ubuntu / Debian:** 
  ```bash
  sudo apt install build-essential cmake git pkg-config \
      libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
  ```

## Client-Server Architecture: 1 Physics Server + Multiple Independent UI Clients

The simulation is split into independent executables communicating via high-performance UDP Multicast:

1. **`physics_engine` (Server Process):**
   - **Role:** Computes rigid-body kinematics, collision response, hexagonal load-cell contact mechanics, and 1 ms fixed-step timing.
   - **Streaming:** Broadcasts `SimulationPacket` state snapshots over UDP Multicast (`239.255.0.1:9870`) at ~60 Hz.
   - **Command Listener:** Listens for interactive user commands (1, 2, Reset, Pause) on UDP `127.0.0.1:9871`.
   - **Zero Graphics:** Has ZERO OpenGL, GLFW, GLAD, or ImGui code. Can run with elevated RT privileges (`sudo` or real-time kernel) on an isolated CPU core without needing display server access!
   - **Logging:** Drains `simulation.log` and `timestep.csv` asynchronously.

2. **`docking_ui` (Client Process - Multiple Independent Windows):**
   - **Role:** Pure visual frontend. Connects to multicast `239.255.0.1:9870` with socket address reuse (`SO_REUSEADDR`), receiving telemetry snapshots simultaneously.
   - **Multiple Independent Shapes:** Supports 4 distinct geometric body shapes:
     - `--shape cube` (Classic Cube visualizer)
     - `--shape sphere` (Smooth UV-Sphere visualizer)
     - `--shape cone` (Slanted Cone visualizer)
     - `--shape pyramid` (4-sided Pyramid visualizer)
   - **Independent Processes:** You can open 1, 2, 3, or all 4 UI windows at the same time! Closing any window has zero effect on the other windows or the physics server.
   - **90 FPS Rendering:** High-precision software frame limiter maintaining smooth 90 FPS independently on each window.
   - **Command Sender:** Dispatches keystrokes (`1`, `2`, `R`, `SPACE`) to the physics server on UDP `127.0.0.1:9871`.
   - **Fault Tolerant:** If the physics engine crashes or is not yet started, the UI remains responsive, displays `[STATUS: WAITING FOR PHYSICS BROADCAST]`, and automatically resumes the moment physics is started.

## Build Instructions

**Ubuntu / Linux**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

**Windows (MinGW)**
```bash
cmake -B build_win -G "MinGW Makefiles"
cmake --build build_win
```

## Running the Executables

### Step 1: Start Physics Engine (Server)
```bash
# On Windows:
run_physics.bat

# On Linux (or with RT isolation):
./build/physics_engine
# or: sudo -E SIM_CPU=3 ./run_realtime.sh
```

### Step 2: Launch 4 Independent UI Visualizers @ 90 FPS
You can launch all 4 UI windows simultaneously with:
```bash
# On Windows:
run_all_4_ui.bat
```
Or launch any shape individually:
```bash
run_ui_cube.bat      # Cube visualizer
run_ui_sphere.bat    # Sphere visualizer
run_ui_cone.bat      # Cone visualizer
run_ui_pyramid.bat   # Pyramid visualizer
```
Or launch directly from the command line:
```bash
./build_win/docking_ui.exe --shape cube --fps 90
./build_win/docking_ui.exe --shape sphere --fps 90
./build_win/docking_ui.exe --shape cone --fps 90
./build_win/docking_ui.exe --shape pyramid --fps 90
```

A Release build is the default if you don't pass `CMAKE_BUILD_TYPE`.

## Physics rate

Physics runs on its own thread with a fixed timestep, **1000 us (1 ms, 1 kHz) by default**
(previously 2 ms). Change it with `--dt-us`:

```bash
./build/sim                  # 1 ms
./build/sim --dt-us 500      # 0.5 ms (2 kHz)
./build/sim --dt-us 2000     # old 2 ms behaviour
```

Other options: `--scenario 1|2`, `--no-log`, `--log-every N`, `--no-realtime`, `--help`.

### Verifying the 1 ms tick (timestep.csv)

Every run writes **`timestep.csv`** next to the executable, one row per physics tick:

| column | meaning |
|---|---|
| `step` | physics tick number |
| `timestamp_s` | time the tick started, seconds since physics started |
| `period_us` | real time since the previous tick (should be about 1000) |
| `error_us` | `period_us` minus the target |
| `latency_us` | how late the tick started versus its ideal scheduled time (the number that matters most) |

Check it with the included script (plain Python 3, no packages):

```bash
python3 check_timing.py                      # reads timestep.csv, target 1000 us
python3 check_timing.py --target-us 500      # if you ran with --dt-us 500
```

It prints the average period and rate, min/max, p50/p99/p99.9, the percentage of ticks on time,
late ticks and missing rows. Close the window normally (not Ctrl+C) so the file is fully written.
Use `--no-timing-log` to turn the file off.

### Hard real-time (no tick may be late)

The simulation *time* always advances by exactly one fixed step per tick, so the physics results
are identical no matter what. What can vary is **when on the wall clock** each tick runs. A
normal Windows or Ubuntu kernel can never *guarantee* that: anything else on the machine (kernel
work, interrupts, power management, the GPU driver) can delay a tick. You can push the worst case
down a lot, and then **measure it** - you cannot prove it from software alone.

Steps, most important first (Linux):

1. **Isolate a core at boot.** In `/etc/default/grub` add to `GRUB_CMDLINE_LINUX`:
   `isolcpus=3 nohz_full=3 rcu_nocbs=3` (use your last core), then `sudo update-grub` and reboot.
2. **Use a real-time kernel** (PREEMPT_RT). Ubuntu offers one through Ubuntu Pro
   (`sudo pro enable realtime-kernel`; see Canonical's docs for your release).
3. **BIOS:** turn off deep C-states / power saving, and ideally SMT (hyper-threading).
4. **Run with the helper script**, which turns off kernel RT throttling, sets the CPU governor to
   `performance`, pins the physics thread to the isolated core and busy-waits:
   ```bash
   sudo -E SIM_CPU=3 ./run_realtime.sh
   sudo -E SIM_CPU=3 ./run_realtime.sh --headless --seconds 3600 --no-log   # long headless soak test
   ```
   (Manual equivalent: `./build/sim --cpu 3 --spin` plus `sudo sysctl -w kernel.sched_rt_runtime_us=-1`.)
5. **Verify with a long run** - the worst tick matters, not the average:
   ```bash
   python3 check_timing.py --limit-us 50
   ```
   It reports the worst start delay and says FAIL if even one tick started more than the limit late.

Windows: run as Administrator (real-time priority class), use the *High performance* power plan,
and start with `sim.exe --cpu 3 --spin`. Windows cannot give a hard guarantee; expect a worst
case in the hundreds of microseconds or worse. If a missed tick could cause real harm
(hardware in the loop), run the 1 ms loop on a real-time controller instead.

New options: `--cpu N`, `--spin`, `--no-spin`, `--no-pin`, `--warmup-ms N`, `--tol-us N` (start-delay limit counted as a violation).

**Ubuntu quick start (recommended order).**
1. `sudo ./setup_ubuntu_rt.sh`, then log out and back in. This allows real-time priority and memory
   locking without sudo. Confirm with `ulimit -r` (should print 95).
2. Run `./build/sim`. It pins physics to the last core and uses sleep-then-spin (about 30% of that core).
3. For the tightest result, also run `sudo -E ./run_realtime.sh`: it turns off RT throttling, which is
   what allows full busy-waiting (spin) and removes the ~50 ms pause the kernel otherwise inserts every
   second into a 100%-busy real-time thread.

**Automatic tuning (default).** On machines with 3+ cores the sim pins the physics thread to the
last core. Busy-waiting (`--spin`) turns on automatically only when RT throttling is already disabled
(use `--no-pin` to turn all of this off). It also
locks memory (`mlockall`) so a page fault can never stall a tick, and the first 300 ms (thread
start-up, GL driver spin-up) are excluded from the run statistics (`--warmup-ms 0` to include them).

**Why "max period" can never be exactly <= 1000 us.** The period is the gap between two tick *starts*.
If one tick starts a few nanoseconds late and the next on time, one gap is slightly over 1000 us and
the other slightly under. So judge the loop by **max start delay** (how late a tick began versus its
schedule) and the `ticks > N us late` count, not by max period. A good result is a start delay of a
few microseconds with zero ticks over your limit.

### Measuring timing without a window

```bash
./build/sim --headless --seconds 10 --no-log
```

prints the achieved period (average, p99, p99.9, max), the worst per-tick compute time,
and the number of late ticks. The GUI shows the same live numbers in the "Physics" block.

### Getting the tightest timing

- **Linux:** the physics thread asks for `SCHED_FIFO` priority. That needs either `sudo`, or
  an rtprio limit, e.g. in `/etc/security/limits.conf`: `@yourgroup - rtprio 50`
  (log out and back in). Without it the sim still runs, just with more jitter.
- **Windows:** uses `timeBeginPeriod(1)` and `THREAD_PRIORITY_TIME_CRITICAL`. Because Windows
  sleep is ~1 ms coarse, a sub-millisecond period busy-waits, so expect one CPU core at 100%.
- Sub-millisecond pacing busy-waits for the final part of each tick on any OS, so use a
  machine with at least 4 cores and avoid running in a heavily shared VM.
