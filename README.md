# Missile Physics Simulator

A 3D missile physics simulator with OpenGL rendering and realistic physics.

## Features

- **Physics-Based Simulation**: 
  - Realistic gravity and aerodynamic forces (drag and lift)
  - Atmosphere model with variable air density
  - Ground collision with configurable bounciness
  - Accurate missile dynamics

- **Missile Guidance System**:
  - Proportional Navigation guidance algorithm
  - Target tracking and homing capabilities
  - Configurable guidance parameters

- **Visual Effects**:
  - 3D rendering of missile, targets, and environment
  - Explosion effects when targets are hit
  - Checkerboard ground surface
  - Camera controls for various viewing angles

- **Interactive UI**:
  - Real-time adjustment of simulation parameters
  - Score tracking for destroyed targets
  - Launch controls for missile targeting
  - Visual feedback for missile status

## Controls

- **Camera Controls**:
- WASD: Move camera position
- Space/Ctrl: Move camera up/down
- Arrow keys: Rotate camera view

- **Simulation Controls**:
- Enter: Pause/Resume simulation
  - F: Launch missile at nearest target
  - Escape: Exit application

## Building the Project

### Requirements

- CMake 3.21 or higher
- Ninja
- C++17 compatible compiler
- vcpkg package manager
- The following libraries (installed via vcpkg):
  - GLFW
  - GLAD
  - GLM
  - ImGui

### Build Instructions

1. Clone the repository
2. Set `VCPKG_ROOT` if vcpkg is not installed in the default Visual Studio or Scoop locations
3. Configure and build with Ninja:
   ```
   cmake --preset ninja-debug
   cmake --build --preset build-debug
   ```

   Or use the helper script:
   ```
   build.bat
   ```

   Release build:
   ```
   build.bat release
   ```

   Clean generated output:
   ```
   build.bat clean
   ```

## Physics Model

The simulation includes:

- Gravitational forces (configurable acceleration)
- Drag forces based on object shape and air density
- Lift forces for aerodynamic objects
- Proportional navigation for missile guidance
- Collision detection with ground and targets

## License

This project is open source, provided as-is without warranty.
