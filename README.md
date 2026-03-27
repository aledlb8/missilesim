# MissileSim

MissileSim is a real-time 3D missile engagement sandbox built with C++17, OpenGL, GLFW, ImGui, and miniaudio. It combines a physics-driven missile model with live tuning controls, target countermeasures, multiple camera views, and telemetry-focused UI panels.

## What It Does

- Simulates missile flight with gravity, drag, lift, thrust, fuel burn, and a layered atmosphere model.
- Supports guided interception with proportional navigation, seeker tracking, proximity fuse tuning, and terrain-avoidance controls.
- Spawns AI-controlled aircraft targets with missile warning behavior and flare countermeasures.
- Renders the engagement in 3D with trajectory previews, target labels, intercept visualization, explosion effects, and audio cues.
- Exposes most simulation parameters in an in-app ImGui control deck so you can tune the scenario while it is running.

## Requirements

- Windows
- Visual Studio 2022 Build Tools or Visual Studio Community with MSVC
- CMake 3.21 or newer
- Ninja
- `vcpkg`

The `build.bat` helper script will try to auto-detect both Visual Studio and `vcpkg`. If `vcpkg` is not in one of the expected locations, set `VCPKG_ROOT` before building.

## Dependencies

- Installed through `vcpkg`: `glfw3`, `glad`, `glm`
- Fetched by CMake at configure time: Dear ImGui, miniaudio

## Quick Start

Build a debug executable:

```powershell
.\build.bat
```

Build a release executable:

```powershell
.\build.bat release
```

Clean generated build output:

```powershell
.\build.bat clean
```

Run the debug build:

```powershell
.\build\ninja-debug\bin\MissileSimOpenGL.exe
```

Run the release build:

```powershell
.\build\ninja-release\bin\MissileSimOpenGL.exe
```

## Manual CMake Build

If you want to use the presets directly, run them from a Visual Studio Developer PowerShell or another shell where the MSVC environment is already loaded:

```powershell
cmake --preset ninja-debug
cmake --build --preset build-debug
```

The presets write build output to `build/<preset-name>`.

## Controls

- `Tab`: Show or hide the UI
- `Enter` or keypad `Enter`: Pause or resume the simulation
- `F`: Launch the missile
- `C`: Return to the free camera and frame the engagement
- `R`: Toggle the pre-launch seeker cue
- `W/A/S/D`: Move the free camera
- `Space`: Move the free camera up
- `Left Ctrl` or `Right Ctrl`: Move the free camera down
- `Left Shift` or `Right Shift`: Increase free-camera movement speed
- Hold right mouse button and move mouse: Rotate the active camera

The in-app HUD also lets you switch between `Free`, `Missile`, and `Fighter Jet` camera modes.

## In-App UI

The simulator exposes most of its controls through ImGui panels, including:

- mission controls for pause, launch, reset, and time scaling
- missile tuning for mass, drag, lift, thrust, fuel, guidance, fuse radius, IRCCM, and terrain avoidance
- target group tuning for count, spacing, speed profile, and AI refresh
- view controls for field of view, camera speed, trajectory detail, and overlay toggles
- telemetry panels for missile, target, and system readouts
- HUD and RWR-style widgets for flight state and threat awareness

Settings are autosaved beside the executable in `config/user_settings.ini`. ImGui window layout is stored in `imgui.ini`.

## Project Layout

- `src/application`: app lifecycle, input handling, UI, camera logic, missile launch flow
- `src/physics`: atmosphere model, physics engine, gravity, drag, lift
- `src/objects`: missile, target, flare, and shared physics-object behavior
- `src/rendering`: OpenGL renderer, scene effects, debug drawing, asset loading
- `src/audio`: miniaudio-based runtime audio system
- `assets`: runtime models and audio assets
- `tools`: utility scripts for project support tasks

## Assets

Runtime model attribution is documented in `assets/assets.md`. Audio assets in `assets/audio` are generated in-repo.

## License

Do whatever you want. This is free and unencumbered software released into the public domain.