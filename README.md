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
- Visual Studio 2022 or newer (any edition, including Build Tools) with the C++ workload
- CMake 3.25 or newer
- Ninja
- `vcpkg`

The `build.bat` helper script finds Visual Studio through `vswhere` and loads the MSVC environment for you. It uses `VCPKG_ROOT` if set, otherwise the copy of `vcpkg` bundled with Visual Studio.

## Dependencies

- Installed through `vcpkg`: `glfw3`, `glad`, `glm`, `assimp`, `nlohmann-json`, `stb`
- Fetched by CMake at configure time: Dear ImGui, miniaudio

## Quick Start

| Command | Output |
| --- | --- |
| `.\build.bat` | Debug build: `build\ninja-debug\bin\MissileSimOpenGL.exe` |
| `.\build.bat release` | Release build: `build\ninja-release\bin\MissileSimOpenGL.exe` |
| `.\build.bat dist` | Shareable zip: `build\dist\MissileSim-<version>-win64.zip` |
| `.\build.bat clean` | Deletes everything under `build\` |

The debug and release builds are for development: they link the vcpkg DLLs, need the Visual C++ runtime, and fall back to loading assets straight from the source tree.

## Sharing a Build

`.\build.bat dist` produces a zip you can send to anyone with 64-bit Windows 10/11 and an OpenGL 4.5 graphics card. They extract it and double-click `MissileSim.exe`; nothing needs to be installed.

The `dist` preset (`MISSILESIM_PORTABLE=ON`, vcpkg triplet `x64-windows-static`):

- links every dependency and the C++ runtime statically, so the exe only depends on system DLLs
- builds a windowed app with no console; log output goes to `missilesim.log` next to the exe, and startup errors (such as a GPU without OpenGL 4.5) are shown in a dialog
- always runs from the exe's own folder, so shortcuts and other launch directories still find `assets/`
- bakes no source-tree paths into the binary
- packages only the runtime assets (Blender sources and previews are excluded) plus `packaging/README.txt`, the player-facing readme

The first `dist` build compiles the static vcpkg dependencies and takes several minutes; later builds reuse them. To bump the version shown in the zip name, change `project(... VERSION ...)` in `CMakeLists.txt`.

## Manual CMake Build

If you want to use the presets directly, run them from a Visual Studio Developer PowerShell or another shell where the MSVC environment is already loaded:

```powershell
cmake --preset ninja-debug
cmake --build --preset build-debug

# configure + build + zip the shareable package
cmake --workflow --preset dist
```

The presets write build output to `build/<preset-name>`.

## Controls

- `F`: Launch the missile
- `R`: Toggle the pre-launch seeker cue
- `Enter` or keypad `Enter`: Pause or resume the simulation
- `V`: Cycle the camera: free, missile, fighter
- `C`: Return to the free camera and frame the engagement
- `W/A/S/D`: Move the free camera
- `Space` / `Ctrl`: Move the free camera up / down
- `Shift`: Increase free-camera movement speed
- Hold right mouse button and move mouse: Rotate the active camera
- `Tab`: Show or hide the control panel
- `H`: Show or hide the HUD
- `F11`: Toggle borderless fullscreen
- `Esc`: Pause menu (resume, restart, settings, main menu, quit)

## Interface

- **Title screen**: the menu sits over the live scene, with a slow cinematic orbit around the lead fighter.
- **HUD**: mission state, camera selector, target roster, a flight-data strip (speed, Mach, altitude, load, fuel, flight time), target brackets with edge arrows for off-screen targets, an event feed (launch, lock, flares, decoys), an RWR scope in the fighter camera, and a result card after each shot.
- **Control panel** (`Tab`): launch/rearm/pause actions plus every simulation parameter in four tabs: Missile, Targets, World, Telemetry. Hover a label for an explanation; double-click a slider to type an exact value.
- **Settings** (`Esc`): display mode, V-sync, interface size, HUD options, field of view, graphics, audio and the full key list.

The window opens centred at 80% of the screen and remembers its display mode. The UI scales with the Windows display scale, times the interface size setting. The look is defined once in `src/ui/Theme.h` (colours, type scale, fonts), and `src/ui/Widgets.h` provides the shared controls.

Settings are autosaved beside the executable in `config/user_settings.ini`.

## Project Layout

- `src/application`: app lifecycle, window management, input, menus, HUD, control panel, camera logic, missile launch flow
- `src/ui`: the visual language (theme tokens, fonts, DPI scaling) and the shared widgets
- `src/physics`: atmosphere model, physics engine, gravity, drag, lift
- `src/objects`: missile, target, flare, and shared physics-object behavior
- `src/rendering`: OpenGL renderer, scene effects, debug drawing, asset loading
- `src/audio`: physical audio: `engine` propagates sound through the air (speed-of-sound delay, Doppler, sonic booms, atmospheric absorption, ground reflection, terrain echoes, adaptive exposure), `synth` generates every sound procedurally from physical parameters, `AudioSystem` connects them to the simulation. miniaudio is only the output device
- `assets`: runtime models, shaders, skyboxes, fonts and config
- `tools`: utility scripts for project support tasks; `tools/audition` builds `AudioAudition`, which renders scripted scenarios (launches, flybys, explosions at several ranges) through the audio engine to WAV files

## Assets

Runtime model attribution is documented in `assets/assets.md`. There are no audio assets: all sound is synthesized at runtime.

## License

Do whatever you want. This is free and unencumbered software released into the public domain.