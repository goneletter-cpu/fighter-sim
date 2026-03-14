# Fighter Sim

A C++ OpenGL flight simulation prototype. The primary mode is a **JSBSim-driven scripted experiment** for testing aircraft flight dynamics across defined phases. A 2.5D arcade combat mode is implemented but disabled by default.

---

## Primary Mode: JSBSim Scripted Experiment

The sim reads a flight script (`config/flight_script.txt`) and runs the aircraft through a sequence of timed phases. Each phase specifies either:

- **Direct surface commands** — fixed elevator/aileron/rudder/throttle values (open-loop).
- **Target tracking** — desired airspeed, climb rate, and heading, closed-loop via PID controllers.

At the end of all phases, the sim exits and logs are written to `build/logs/`.

### Flight Script Format

```text
run_mode single           # single | batch
model fgdata:c172p        # JSBSim model name
initial_alt_m 0           # starting altitude (metres)
initial_speed_mps 0       # starting airspeed (m/s)

columns: duration_s, target_speed_mps, target_climb_mps, target_heading_deg, fcs/flap-cmd-norm, fcs/gear-cmd-norm

# duration  speed   climb  hdg   flap  gear
12,          30,     0,     0,    0.2,  1.0
60,          60,     3.0,   0,    0.0,  1.0
120,         70,     0.5,   0,    0.0,  1.0
```

**Column modes:**

| Column header style | Behaviour |
|---|---|
| `target_speed_mps` / `target_climb_mps` / `target_heading_deg` | PID closed-loop tracking |
| `fcs/elevator-cmd-norm` etc. | Direct open-loop surface command |
| Mixed | Any `target_*` column activates PID for the whole config |

**Batch mode** — set `run_mode batch` and list multiple models:

```text
run_mode batch
models fgdata:c172p, fgdata:c182p
```

Each model runs the full phase list independently, producing a separate log file.

### Output Logs

| File | Rate | Contents |
|---|---|---|
| `build/logs/<model>.csv` | Every physics step (120 Hz) | Full state: position, velocity, Euler angles, angular rates, AoA, airspeed, throttle. Enabled by compiling with `-DENABLE_CSV_LOG=1`. |
| `build/logs/last_flight.txt` | 1 Hz | Compact track: time, altitude, position, speed, climb, control surfaces, throttle (cmd + actual), engine RPM. Always written. |

---

## Arcade Combat Mode (disabled by default)

Set `kEnableCombat = true` in `main.cpp` to enable.

### Gameplay

- **Forward-scrolling lane** with a bounded low-altitude flight envelope (55 – 820 m).
- **Enemies** spawn radially around the player (1500 – 2900 m radius), fly toward the player, and respawn on kill or out-of-range.
- **Weapons:**
  - `Space` tap: bomb / proximity missile.
  - `Space` hold (> 0.22 s): machine gun at 18 rounds/sec, alternating left/right muzzles.
- **Ground AA batteries** fire lead-compensated shells with gravity. 10 batteries along the scroll path.
- **Explosions** scale and colour-shift over their lifetime (gold → red).
- **Carrier intro sequence** (set `intro_active = true`): catapult launch animation with deck markings, gear retraction, and flap recovery before entering the combat lane.

### Combat Controls

| Key | Action |
|---|---|
| `W / S` | Pitch up / down |
| `A / D` | Roll left / right (+ diagonal motion contribution) |
| `Q / E` | Yaw left / right |
| `Left Shift / Ctrl` | Throttle up / down |
| `Space` tap | Fire bomb / missile |
| `Space` hold | Machine gun |
| `Esc` | Quit |

### Combat HUD

- **Bottom-left**: forward-sector radar with scan sweep and enemy blips.
- **Bottom-right**: world-map overview with relative enemy positions.
- **Title bar**: live HP, score, altitude, airspeed, climb rate, AoA, throttle, active round counts.

*(Attitude gauge and quadrant overlay are implemented in `renderer.cpp` but currently disabled.)*

---

## Build

### macOS Quick Path

```bash
chmod +x build.sh
./build.sh
./build/fighter_sim
```

`build.sh` uses Homebrew to ensure `glfw` and `glm` are installed, then compiles with `clang++`.

### CMake

```bash
cmake -S . -B build-cmake
cmake --build build-cmake -j
./build-cmake/fighter_sim
```

### Enable detailed CSV logging

```bash
cmake -S . -B build-cmake -DENABLE_CSV_LOG=1
cmake --build build-cmake -j
```

---

## Requirements

- C++17 compiler
- OpenGL 3.3 Core Profile
- GLFW
- GLM
- JSBSim (linked via `jsbsim_adapter`)
- macOS frameworks: `OpenGL`, `Cocoa`, `IOKit`, `CoreVideo`

---

## Project Structure

```text
config/
  flight_script.txt        # phase definitions (auto-generated on first run if missing)

include/
  physics.h                # AircraftState, ControlInput, AttitudeCommand structs
  pid.h                    # single-axis PID controller
  flight_controller.h      # attitude controller interface
  jsbsim_adapter.h         # JSBSim integration wrapper
  renderer.h               # OpenGL wireframe renderer

src/
  main.cpp                 # main loop, script runner, JSBSim stepping, input, camera, rendering
  physics.cpp              # 6DoF dynamics + RK4 (retained for future full-physics mode)
  pid.cpp                  # PID implementation
  flight_controller.cpp    # attitude controller (pitch/roll/yaw rate)
  jsbsim_adapter.cpp       # JSBSim initialisation, step, property read/write
  renderer.cpp             # OpenGL wireframe renderer, mesh factories, HUD drawing

build/
  logs/
    <model>.csv            # per-model detailed flight log (ENABLE_CSV_LOG=1)
    last_flight.txt        # last-run 1 Hz track log

build.sh                   # one-command local build (macOS)
```

---

## Key Compile-Time Flags

| Flag | Default | Effect |
|---|---|---|
| `ENABLE_CSV_LOG` | `0` | Write per-step CSV to `build/logs/<model>.csv` |
| `kEnableCombat` | `false` | Enable enemies, weapons, AA, explosions |
| `kUseAutopilot` | `false` | Hand control to JSBSim built-in autopilot |
| `kScriptPath` | `config/flight_script.txt` | Flight script location (relative to `FIGHTER_SIM_ROOT`) |

`kEnableCombat`, `kUseAutopilot`, and `kScriptPath` are `static const` values at the top of `main.cpp`.

---

## Development Tips

### Fast rebuild on file change

```bash
brew install entr
find src include -name '*.cpp' -o -name '*.h' | entr -r ./build.sh
```

### Main tuning points

| What | Where |
|---|---|
| PID gains (speed / climb / heading) | `main()` in `src/main.cpp` — `PID speed_pid(...)`, `climb_pid(...)`, `heading_pid(...)` |
| Enemy spawn radius and speed | `respawn_enemy(...)` in `src/main.cpp` |
| Chase camera distance and lag | `update_chase_camera(...)` in `src/main.cpp` |
| HUD drawing | `draw_hud(...)`, `draw_radar(...)` in `src/renderer.cpp` |
| Wireframe mesh shapes | `make_fighter_mesh()`, `make_cloud_mesh()` etc. in `src/renderer.cpp` |

---

## Architecture Notes

**Fixed-step physics, variable-rate rendering.** Physics runs at 1/120 s via an accumulator loop. Rendering runs at the display frame rate (vsync). The two are fully decoupled.

**Three control modes** (mutually exclusive, selected by script/flags):

1. Open-loop: fixed surface deflections from the script.
2. Closed-loop: three-PID autopilot (throttle → speed, elevator → climb, rudder → heading).
3. JSBSim built-in autopilot (`kUseAutopilot = true`).

**`[[maybe_unused]]` helpers** — several functions (`apply_direct_player_attitude_control`, `keep_forward_flight`, `apply_constrained_3d_motion`, etc.) implement the original 2.5D arcade physics. They are retained for reference and future full-physics expansion but are not called in the current default path.
