# Flight handling checks

The harness uses the game's airframe, flight controls, instructor and fighter
adapter without opening an OpenGL window. Build it with developer tools enabled
(the default for the debug preset):

```powershell
.\build.bat
ctest --test-dir build/ninja-debug --output-on-failure
```

Run `build/ninja-debug/bin/flight_harness.exe --check` for the individual handling
measurements. A failed check returns a nonzero exit code. Running without
`--check` retains the exploratory flight reports; an optional substring such as
`aim`, `stick`, or `energy` selects those scenarios. Those reports are diagnostic
output, not pass/fail tests.

The regression suite checks:

- Small vertical corrections at several speeds, including unintended bank and
  roll rate as well as final nose alignment.
- Large turns and diving maneuvers, with load and angle-of-attack limits.
- Pitch, roll and yaw release with the mouse held in world space, reacquisition
  of that aim, and immediate response to a new manual input.
- Steady aim-motion estimation from 30 through 240 FPS and clearing motion on
  pause/recenter.
- The real `Fighter` input path with independent rendering and physics rates,
  uneven frame times, and simulation-time scaling.

Input samples and physics steps intentionally have separate clocks:
`Jet::setControls(controls, sampleDeltaTime)` runs once per input sample, including
frames with no physics update. The interval is in **simulation seconds**; pass
zero to rebase the aim after a discontinuity or while paused. `Jet::step` may run
zero or several times afterward and must not resample the input. Keyboard
handoffs advance on the inner physics timestep.

The limits in these checks are game-handling regression criteria, not claims of
exact War Thunder behavior or validated F-16 performance.
