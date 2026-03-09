# Fighter Sim

A lightweight C++ OpenGL air-combat prototype with a 2.5D side-scroller flight model, wireframe rendering, tactical HUD, and arcade combat loop.

## Current Gameplay

- Intro sequence: carrier takeoff with catapult deck lines, landing gear retraction, and flap recovery.
- Core mode: forward-scrolling combat lane with bounded flight area.
- Enemies: spawn in four quadrants (clustered near axes), fly toward the player, and respawn when destroyed/out of range.
- Weapons:
- `Space` tap: proximity-fuse missile (named as "bomb" in code, used as missile gameplay).
- `Space` hold: machine gun stream.
- Feedback: fireball-like explosion effect at enemy position and scale.
- Threats: ground anti-air batteries fire shells at the player.
- HUD:
- Bottom-left tactical forward-sector radar with scan sweep and target blips.
- Bottom-right attitude instrument (pitch/roll horizon gauge).
- World-space quadrant overlay with dashed boundary/axes and origin marker.

## Controls

- `W / S`: vertical control (Y axis).
- `Q / E`: lateral control (X axis, nose-yaw visual attitude).
- `A / D`: roll input and diagonal motion contribution (quadrant-dependent behavior).
- `Left Shift / Left Ctrl`: throttle up / down.
- `Space` (tap): fire missile with proximity detonation.
- `Space` (hold): fire machine gun continuously.
- `Esc`: quit.

## Build (macOS Quick Path)

```bash
chmod +x build.sh
./build.sh
./build/fighter_sim
```

`build.sh` uses Homebrew to ensure `glfw` and `glm` are installed, then compiles with `clang++`.

## Build (CMake Alternative)

```bash
cmake -S . -B build-cmake
cmake --build build-cmake -j
./build-cmake/fighter_sim
```

## Requirements

- C++17 compiler
- OpenGL 3.3 Core Profile
- GLFW
- GLM
- macOS frameworks used by the direct build script:
- `OpenGL`
- `Cocoa`
- `IOKit`
- `CoreVideo`

## Project Structure

```text
include/
  physics.h
  pid.h
  flight_controller.h
  renderer.h

src/
  main.cpp               # main loop, gameplay, input, camera, UI flow
  physics.cpp            # 6DoF dynamics + RK4 (kept for future full-physics mode)
  pid.cpp                # single-axis PID controller
  flight_controller.cpp  # attitude controller (pitch/roll/yaw)
  renderer.cpp           # OpenGL wireframe renderer + meshes + HUD drawing

build.sh                 # one-command local build for macOS
```

## Notes on Physics Mode

The current playable mode uses a side-scroller motion model for smooth arcade control.  
The real-flight physics path (dynamics/controller integration and coupling helpers) is intentionally kept in the codebase as commented/retained logic for future expansion.

## Development Tips

- Rebuild quickly while editing:

```bash
brew install entr
find src include -name '*.cpp' -o -name '*.h' | entr -r ./build.sh
```

- Main tuning points:
- `apply_side_scroller_motion(...)` in `src/main.cpp` for control feel.
- `respawn_enemy(...)` and `update_enemies(...)` for enemy pacing/placement.
- `draw_radar(...)` and `draw_attitude_gauge(...)` in `src/renderer.cpp` for HUD behavior.
