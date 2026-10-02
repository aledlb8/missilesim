# F-16C Block 50 flight model

The airplane this file describes is the one `src/flight` already integrates. The variant name on that model is a single-seat F-16C Block 50 with an F110-GE-129. The aerodynamic tables are not a Block 50 test. They are the early NASA TP-1538 F-16, a low-speed wind-tunnel model flown in that report at 20,500 lb. Say that on every card that quotes a coefficient: Block 50 is the mass and the thrust scale; the polar is 1979.

The exterior sheet is [fighters/f-16c-block-50.md](../fighters/f-16c-block-50.md). It is a mesh, not this flight model. Nothing here is a weapon envelope.

## Status

**FULL_TABLE.** The low-speed six-degree-of-freedom model is already in the repository: body-axis forces and moments, damping, actuators, and the F100 thrust deck, transcribed in `src/flight/F16Data.cpp` and integrated in `src/flight/F16Airframe.cpp`.

Mach effect on lift and moments is **NOT** in those tables. `src/flight/F16Data.h` says the data are low-speed wind-tunnel data and that there is no Mach effect on lift or moments. NASA TP-1538 says the same limit in its own words: the results can only be considered valid for Mach numbers less than about 0.6, and only the clean configuration was investigated. The code still evaluates the same tables at any Mach.

The only Mach term in the code is the Brandt zero-lift drag rise in `F16Airframe.cpp` (`machDragRise`). It is added as wind-axis drag. It does not change lift or moments.

## What the game already integrates

`Jet` (`src/flight/Jet.h`, `src/flight/Jet.cpp`) owns one `F16Airframe` built at `kCombatMassKg`, a `FlightControlSystem`, and the mouse-aim instructor. Each call steps the airframe at `kInnerStepSeconds` (1/400 s) with semi-implicit Euler: rates and velocity, then attitude and position. World axes are the simulator's, y up. Body axes are x forward, y right wing, z down. The attitude quaternion maps body vectors into world.

The player lever in `JetControls` is 0 to 1 from idle to military. Without afterburner, `Jet::step` multiplies it by 0.77 before it reaches the airframe. Afterburner forces the airframe lever to 1. The airframe then runs `throttleGearing`, the power lag, and `thrustLbf`.

Surface commands from the flight-control law pass through the TP-1538 first-order actuators (lag, rate limit, then deflection stop). Forces are thrust along body +x, the tabulated aerodynamic force, and the Brandt drag rise opposite the velocity vector. Moments use the Stevens and Lewis inertia constants `c1` through `c9`, with the engine angular momentum left at the TP-1538 value. The centre of gravity is fixed at the reference station, so the centre-of-gravity shift terms in pitch and yaw are zero.

Mass does not change. Fuel is not burned. There is no gear, no speed brake, and no leading-edge flap state. Atmosphere (density and speed of sound) is an input. Altitude for the thrust deck is world y in feet, held at or above zero.

The control law is not the report's gain schedule. `src/flight/FlightControl.h` says the report's gains and block diagrams are not reproduced. The law is nonlinear dynamic inversion of these same tables. The bandwidths in `src/flight/FlightControl.cpp` (angle of attack 3.5 /s, sideslip 2.5 /s, pitch rate 9 /s, roll rate 7 /s, yaw rate 6 /s) are code, not NASA numbers, and they are not airframe limits. A command floor of −8° angle of attack is also code, inside the tabulated −10° edge, so a pull does not ask the tables for a value they do not have.

## Point-mass card

The row the game flies is tagged **CODE**. A different print is another row. They are not averaged. Kilograms of the combat mass are the code expression `22972.0f * 0.45359237f` (10,419.92 kg as a stored float).

| Item | Value | Unit | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Empty mass in the combat-mass sum | 18,900 | lb | CODE | `src/flight/Jet.h` comment | The comment's first addend. The Wikipedia specifications section "F-16C Block 50 and 52" prints the same 18,900 lb and attributes the table to the USAF fact sheet, Frawley's *International Directory of Military Aircraft*, and the Hellenic Air Force Block 50/52+ manual GR1F-16CJ-1. Those three were not opened. The USAF fact-sheet URL returned Access Denied. |
| Empty mass, other print | 18,238 | lb | SECONDARY | f-16.net, standard Block 50/52 card | Not used. Not a second empty weight to blend with 18,900. |
| Empty mass, other print | 20,300 | lb | SECONDARY | GlobalSecurity F-16 specifications table, one column, also printed there as 9,207.9 kg | Not used. The same page is two cards run together. Do not treat it as one consistent Block 50. |
| Internal fuel in the sum | 7,000 | lb | CODE | `src/flight/Jet.h` comment | The sum uses half of this, 3,500 lb. Wikipedia's Block 50/52 specification line prints 7,000 lb internal. GlobalSecurity's fuel table also has a single-seat internal cell of 7,000 lb. |
| Internal fuel, other print | 5,920 | lb | SECONDARY | GlobalSecurity, a separate "Internal fuel" cell, also printed as 2,685.2 kg | Not used. Left beside the 7,000 lb cell. Not averaged. |
| AIM-9X mass in the sum | 186 | lb | CODE | `src/flight/Jet.h` comment; NAVAIR AIM-9X page prints launch weight 186 lb (84.37 kg) | Two rounds are 372 lb. That is mass in the constant only. No store drag is added. |
| Pilot mass in the sum | 200 | lb | CODE | `src/flight/Jet.h` comment | The project note `research_notes/Famous Fox 2 missiles/flight-model.md` calls this an estimate. No opened specification page prints 200 lb. Empty weight on those pages excludes the pilot. The game still includes 200 lb because that is how the constant was built. |
| Combat mass | 22,972 | lb | CODE | `src/flight/Jet.h` `kCombatMassKg` | 18,900 + 0.5×7,000 + 2×186 + 200 = 22,972 lb. The code stores `22972.0f * 0.45359237f` = 10,419.92 kg. Fixed for the whole flight. |
| Loaded mass, other print | 26,500 | lb | WIKI-ONLY | Wikipedia Block 50/52 specifications, gross weight | Not this sum. Not used. |
| Loaded mass, other print | 26,463 | lb | SECONDARY | f-16.net, "normal loaded (air-to-air mission)" | The page does not define it as half internal fuel plus two wingtip missiles plus a pilot. Not used. |
| Loaded mass, other print | 23,000 | lb | WIKI-ONLY | Wikipedia thrust-to-weight footnote: "Loaded weight with 50% internal fuel (23,000 lb)" divided into thrust 28,600 lb | The footnote's thrust is 28,600 lb, not 29,500. Not the combat mass. Not used. |
| Wing area S | 27.87 | m² | CODE | `src/flight/F16Data.h` `kWingAreaM2` | Comment and NASA TP-1538 Table I both print 300 ft² and 27.87 m². Coefficients use 27.87 m². Wikipedia, GlobalSecurity, and f-16.net also print 300 ft². |
| Span b, aerodynamic reference | 9.144 | m | CODE | `src/flight/F16Data.h` `kSpanM` | 30 ft. NASA TP-1538 Table I prints 9.144 m (30 ft). This is the span in the force and moment equations. It is not tip-to-tip with launchers. |
| Span, other print | 32 ft 8 in | — | WIKI-ONLY | Wikipedia Block 50/52 specifications | Overall span. Do not put it in the lift equation with S = 300 ft². That pair would be an aspect ratio of 3.56, which this model does not fly. |
| Span, other print | 31 ft 0 in | — | SECONDARY | f-16.net Block 50/52 dimensions | Not the reference span. Not used. |
| Mean aerodynamic chord cbar | 3.45 | m | CODE | `src/flight/F16Data.h` `kChordM` | NASA TP-1538 Table I prints 3.45 m (11.32 ft). Pitching moment is on this chord. It is not span/aspect ratio. |
| Aspect ratio | 3 | — | DERIVED | 30 ft and 300 ft² in Table I and in the `F16Data.h` comments | The code does not store AR. b²/S on the stored SI pair 9.144 m and 27.87 m² is 3.0001, from rounding 300 ft² to 27.87 m². Forces use S and b, not AR. |
| Cd0 at Mach 0.30 | 0.0193 | — | CODE | `src/flight/F16Airframe.cpp` `machDragRise` | Brandt "actual" F-16 polar, as the file comment names it. On wing area. The rise subtracted in flight is this value minus itself, so the added drag is 0 at and below Mach 0.30. The low-speed Cx tables already carry subsonic zero-lift drag. |
| Cd0 at Mach 0.86 | 0.0202 | — | CODE | same samples | Added drag coefficient at this Mach is 0.0202 − 0.0193 = 0.0009. |
| Cd0 at Mach 1.05 | 0.0444 | — | CODE | same samples | Added coefficient 0.0251. |
| Cd0 at Mach 1.50 | 0.0448 | — | CODE | same samples | Added coefficient 0.0255. |
| Cd0 at Mach 2.00 | 0.0458 | — | CODE | same samples | Added coefficient 0.0265. Above Mach 2.00 the added coefficient stays 0.0265. Between the samples the code interpolates linearly. The force is q·S·ΔCd0 opposite the velocity, not a body-axis Cx patch and not a moment. |
| CLmax | none | — | NOT PUBLISHED | — | The code has no CLmax constant. Lift is `cz`, positive along body +z (down), so a pull is a negative Cz. Do not replace that table with one CLmax. No combat CLmax was printed on the pages opened for this sheet. |
| Structural normal-load cap, positive | +9 | g | CODE | `src/flight/FlightControl.h` `kMaxNormalLoad` | The header calls it the published F-16C limit and attributes the limit set to NASA TP-1538 appendix A. Wikipedia's Block 50/52 line prints g limits +9 and does not print a negative limit. The code cap is constant. It is a command limit in the control law, not a structural failure. Nz is specific force, positive nose-up: `−(force_z / mass) / g`. |
| Normal-load cap, negative | −3 | g | CODE | `src/flight/FlightControl.h` `kMinNormalLoad` | The constant comment says "negative g limit." Appendix A of TP-1538 draws a negative-g schedule against dynamic pressure (figure 62(b)); the text extraction does not reduce that figure to a single −3. No opened specification line printed −3. The game flies −3 anyway. |
| Angle-of-attack cap | 25 | deg | CODE | `src/flight/FlightControl.h` `kAlphaLimitDeg` | Hard cap. TP-1538 appendix A gets about 25° in 1 g flight by a different law: commanded normal acceleration is reduced 0.322 g per degree from 15° to 20.4°, and 1.322 g per degree above 20.4°. The code does not use 0.322 or 1.322. At high dynamic pressure the +9 g cap binds first and the angle of attack stops lower than 25°. |
| Roll-rate cap, clean | 308 | deg/s | CODE | `src/flight/FlightControl.h` `kMaxRollRateDeg` | Stability-axis roll-rate command. NASA TP-1538 prints the same 308°/s maximum for the roll-rate command system. Wikipedia's specification line prints 324°/s, citing a *Code One* article that was not opened. The game flies 308. |
| Roll-rate floor | 80 | deg/s | CODE | `FlightControl.h` comment and `FlightControl.cpp` `rollRateLimit` | The report describes the command being reduced to as little as 80°/s. The code uses that floor. |
| Roll-rate schedule | 308 − 0.0115 per Pa below 10,500 Pa − 4 per deg above 15° | deg/s | CODE | `src/flight/FlightControl.cpp` `rollRateLimit` | Dynamic pressure in pascals, angle of attack in degrees, then a floor of 80. TP-1538's control system B text has the same pieces: −0.0115°/s per N/m² below 10,500 N/m² (the report also prints 219.3 lb/ft²), and 4°/s per degree of angle of attack above 15°, down from 308°/s. The report also schedules roll rate with stabilator deflection. The code does not. |
| Actuator lag | 0.0495 | s | CODE | `src/flight/F16Airframe.cpp` | First-order lag on elevator, aileron, and rudder. NASA TP-1538 appendix A prints 0.0495 s for those surfaces. The commanded rate is (error) / 0.0495, then clipped to the rate limit. |
| Elevator rate limit | 60 | deg/s | CODE | `src/flight/F16Airframe.cpp` | Matches the symmetric-tail rate in appendix A. |
| Aileron rate limit | 80 | deg/s | CODE | `src/flight/F16Airframe.cpp` | Appendix A prints 80°/s for the ailerons. |
| Rudder rate limit | 120 | deg/s | CODE | `src/flight/F16Airframe.cpp` | Appendix A prints 120°/s. |
| Elevator deflection stop | ±25 | deg | CODE | `src/flight/F16Data.h` `kElevatorLimitDeg` | Symmetric tail. Positive elevator is trailing edge down, nose down (`src/flight/F16Airframe.h`). Table I and appendix A both print ±25°. |
| Aileron deflection stop | ±21.5 | deg | CODE | `src/flight/F16Data.h` `kAileronLimitDeg` | Positive aileron rolls left in the Stevens and Lewis convention the header states. Appendix A prints ±21.5°. |
| Rudder deflection stop | ±30 | deg | CODE | `src/flight/F16Data.h` `kRudderLimitDeg` | Positive rudder yaws left. Appendix A prints ±30°. |
| F110-GE-129 military rating, scale numerator | 17,155 | lbf | CODE | `src/flight/F16Airframe.cpp` `kMilitaryScale` | Uninstalled-rating numerator. Wikipedia's Block 50/52 specification line prints 17,155 lbf dry for the Block 50 engine. f-16.net prints the same 17,155 lb dry. This is not the sea-level static thrust the deck returns. |
| F110-GE-129 afterburning rating, scale numerator | 29,500 | lbf | CODE | `src/flight/F16Airframe.cpp` `kAfterburnerScale` | The number the scale uses. Wikipedia's specification line prints 29,500 lbf with afterburner. |
| F110-GE-129 afterburning, other print | 28,984 | lbf | SECONDARY | f-16.net Block 50/52 engine line, with dry thrust still 17,155 | Not used. The same page's history paragraph also says both IPE engines are "rated at 29,000 lbs." That third print is not blended in. |
| F110-GE-129 afterburning, other print | 29,588 | lbf | WIKI-ONLY | Wikipedia propulsion section, "29,588 lbf (131.61 kN) F110-GE-129" | Same article as the 29,500 specification line. Not used. The specification line is the one that matches the code. |
| F110-GE-129 thrust class | 29,000 | lb | OFFICIAL | GE Aerospace F110-GE-129 datasheet, sea level, standard day, thrust class 29,000 lb (129 kN) | A class, not a military/afterburner split. Does not replace 17,155 or 29,500. Length 181.9 in, airflow 270 lb/s, diameter 46.5 in, and bypass ratio 0.76 on that sheet are engine hardware, not used by the flight model. |
| F100-PW-200 military rating, scale denominator | 14,690 | lbf | CODE | `src/flight/F16Airframe.cpp` | Wikipedia's F100-100/200 specification box prints military/intermediate thrust 14,690 lbf (65.3 kN) and says the −200 ratings are almost identical to the −100. The code's comment names this pair as the uninstalled ratings the deck is scaled from. |
| F100-PW-200 afterburning rating, scale denominator | 23,930 | lbf | CODE | `src/flight/F16Airframe.cpp` | Wikipedia's F100-100/200 box prints 23,930 lbf (106.4 kN) with full afterburner. The file comment also records an unused 22,600 lbf afterburning figure that would make the afterburner scale about 1.31. That page was not opened. The game divides by 23,930. |
| Military scale | 17,155 / 14,690 | — | CODE | `kMilitaryScale` | 1.1678. Applied only to the military deck, not to idle. |
| Afterburner scale | 29,500 / 23,930 | — | CODE | `kAfterburnerScale` | 1.2328. Applied only to the maximum-afterburner deck. The altitude and Mach shape stay the F100 deck's shape. |
| Sea-level static military thrust the deck returns | 14,807.7 | lbf | DERIVED | `kThrustMil` at Mach 0, 0 ft is 12,680 lbf, times 17,155/14,690 | Power command 50. Installation loss in the 1979 deck is kept, so this is not 17,155 lbf. |
| Sea-level static afterburning thrust the deck returns | 24,655.2 | lbf | DERIVED | `kThrustMax` at Mach 0, 0 ft is 20,000 lbf, times 29,500/23,930 | Power command 100. Not 29,500 lbf. |
| Sea-level static idle thrust | 1,060 | lbf | CODE | `kThrustIdle` at Mach 0, 0 ft | Not multiplied by either scale. |
| Military throttle split | 0.77 | — | CODE | `src/flight/F16Airframe.h`; `src/flight/Jet.cpp` `kMilitaryLever`; `throttleGearing` in `F16Data.cpp` | Airframe lever 0.77 is military. Gearing is 64.94×lever up to 0.77 (that product is 50, the idle-to-military boundary) and 217.38×lever − 117.38 above it (100 at lever 1). `Jet` maps a player throttle of 1, afterburner off, onto lever 0.77, and afterburner onto lever 1. |
| Test weight the inertias belong to | 9,298.6 | kg | CODE | `src/flight/F16Data.h` `kTestWeightKg` | Comment: 20,500 lb. NASA TP-1538 Table I prints 91,188 N (20,500 lb). 20,500 lb × 0.45359237 kg/lb = 9,298.64 kg; the code stores 9,298.6. This is simulation weight, not empty weight and not the combat mass. |
| Ixx at the test weight | 12,875 | kg·m² | CODE | `kIxx` | Table I prints 12,875 kg·m² (9,496 slug·ft²). |
| Iyy at the test weight | 75,674 | kg·m² | CODE | `kIyy` | Table I prints 75,674 kg·m² (55,814 slug·ft²). |
| Izz at the test weight | 85,552 | kg·m² | CODE | `kIzz` | Table I prints 85,552 kg·m² (63,100 slug·ft²). |
| Ixz at the test weight | 1,331 | kg·m² | CODE | `kIxz` | Table I prints 1,331 kg·m² (982 slug·ft²). |
| Inertia scale at the combat mass | 1.1206 | — | DERIVED | `F16Airframe` constructor, `massKg / kTestWeightKg` | Ixx, Iyy, Izz, and Ixz are each multiplied by this ratio. The store and fuel distribution was not published; the code says the scale is an assumption. Engine angular momentum is not scaled. |
| Engine angular momentum | 216.9 | kg·m²/s | CODE | `kEngineMomentum` | TP-1538 appendix B prints 216.9 kg·m²/s (160 slug·ft²/s). Held fixed. |
| Reference centre of gravity | 0.35 | fraction of cbar | CODE | `kReferenceCg`; `F16Airframe.cpp` sets the flying cg to the same value | Table I reference station. The moment equations correct to the flying cg, which is this station, so the correction is zero. The heavier combat mass does not move the cg. |

## Six-degree-of-freedom data

These tables are already transcribed in `src/flight/F16Data.cpp` and must not be retyped. `src/flight/F16Data.h` names the source: NASA TP-1538, in the reduced form tabulated by Stevens and Lewis, *Aircraft Control and Simulation*, appendix A. The header says the numbers were checked value for value between two transcriptions. This sheet does not repeat that check and does not copy the grids.

Body axes: x forward, y right wing, z down. Angles in degrees at the lookup, body rates in rad/s when the derivatives are applied. Coefficients are on S, b, and cbar. Rate terms enter as (cbar / 2V)·q and (b / 2V)·p or r. Lookups clamp angle of attack to −10°…45° and sideslip to ±30°. They do not extrapolate past those angles. The elevator grids are breakpointed at −24°…24° in 12° steps; a surface at the ±25° stop is one degree outside that grid and the lookup walks the end segment.

The full TP-1538 appendix B build-up also adds leading-edge-flap increments, speed-brake increments, and extra aileron and rudder increments. Those are not functions in this file. The reduced model below is what the game calls.

| Function | What it returns | Alpha / beta range | Source |
| --- | --- | --- | --- |
| `cx(alphaDeg, elevatorDeg)` | Body-axis X-force coefficient, elevator included | Alpha −10°…45°, step 5°. Elevator grid −24°…24°, step 12° | NASA TP-1538 via Stevens and Lewis, as `F16Data.h` states |
| `cy(betaDeg, aileronDeg, rudderDeg)` | Body-axis side force. Not a grid: −0.02·β + 0.021·(aileron/20) + 0.086·(rudder/30), β in degrees | Beta clamped to ±30° | Same reduced model |
| `cz(alphaDeg, betaDeg, elevatorDeg)` | Body-axis Z force. Base Cz(alpha) times (1 − (β/57.3)²), minus 0.19·(elevator/25). This is the lift model | Alpha −10°…45°, step 5°. Beta clamped to ±30° | Same reduced model |
| `cm(alphaDeg, elevatorDeg)` | Pitching-moment coefficient | Same alpha and elevator grids as `cx` | Same |
| `cl(alphaDeg, betaDeg)` | Rolling-moment coefficient, odd in beta | \|β\| 0°…30°, step 5°. Alpha −10°…45°, step 5° | Same |
| `cn(alphaDeg, betaDeg)` | Yawing-moment coefficient, odd in beta | Same grids as `cl` | Same |
| `dlda(alphaDeg, betaDeg)` | Rolling moment per aileron normalised by 20° | Beta −30°…30°, step 10°. Alpha −10°…45°, step 5° | Same. The airframe multiplies by aileron/20 |
| `dldr(alphaDeg, betaDeg)` | Rolling moment per rudder normalised by 30° | Same beta and alpha grids as `dlda` | Same. Multiplied by rudder/30 |
| `dnda(alphaDeg, betaDeg)` | Yawing moment per aileron normalised by 20° | Same grids as `dlda` | Same |
| `dndr(alphaDeg, betaDeg)` | Yawing moment per rudder normalised by 30° | Same grids as `dlda` | Same |
| `damping(alphaDeg)` | `cxq`, `cyr`, `cyp`, `czq`, `clr`, `clp`, `cmq`, `cnr`, `cnp` | Alpha −10°…45°, step 5° | Same |
| `throttleGearing(throttle)` | Lever 0…1 to commanded power 0…100. Military is lever 0.77 and power 50 | — | Stevens and Lewis gearing, as `Jet.cpp` states. TP-1538 figure 66 is the power logic the report drew |
| `powerRate(power, commandedPower)` | Power rate in percent per second. Inside a regime, rate factor `rtau` is 1 when the power error is ≤ 25, 0.1 when the error is ≥ 50, and 1.9 − 0.036·error between. Crossing 50 aims at an intermediate target (60 when spooling up, 40 when spooling down). Within afterburner, and when leaving it, the factor is 5 | Power 0…100. Power 0…50 is idle to military. Power 50…100 is afterburner | Same power model. Figure 66 shows the 60 / 40 targets and the factor 5 |
| `thrustLbf(power, altitudeFt, mach, milScale, maxScale)` | Pounds of thrust. Idle is unscaled. Military and maximum are scaled. Below power 50 the result blends idle to military. At and above 50 it blends military to maximum | Mach breakpoints 0, 0.2, 0.4, 0.6, 0.8, 1.0. Altitude breakpoints 0 to 50,000 ft in 10,000 ft steps. Past the last breakpoint the end segment is extended. The function refuses Mach above 2 or altitude above 70,000 ft. Those caps are not tabulated data | `F16Data.h` calls the deck TP-1538 appendix B / table VI, the F100-PW-200 installed in that airplane. The grids are in `F16Data.cpp`. Do not retype them |

`computeLoads` will not return less than −20,000 lbf before the conversion 1 lbf = 4.448221615 N.

## Feel knobs that are real limits

`src/flight/FlightControl.h` describes the limits it implements as NASA TP-1538 appendix A describes them: an angle-of-attack limiter (about 25° in 1 g) and +9 g, roll-rate command up to 308°/s scheduled down with dynamic pressure and angle of attack (the report's control system B), pilot rudder faded out between 20° and 30° angle of attack, and a spin-prevention mode above 29°.

The game's numbers are:

- **+9 g** and **−3 g.** Positive is the published cap the header names. Negative is the code constant. Either cap is turned into the angle of attack that would produce it with the current `cz`, and the pitch-rate command is held inside that angle. If the 25° cap is the tighter one, the limit is angle of attack, not g.
- **25° angle of attack.** Hard stop on the nose-up command. The report's 0.322 g/deg and 1.322 g/deg schedule is not in the code. The 25° number is.
- **308°/s roll schedule.** `rollRateLimit` is 308°/s, minus 0.0115°/s for each pascal of dynamic pressure below 10,500 Pa, minus 4°/s for each degree of angle of attack above 15°, and never below 80°/s. That is the dynamic-pressure and angle-of-attack part of control system B. It is not the report's extra cut with stabilator deflection, and it is not the report's pseudo-angle-of-attack path that feeds roll rate back into the pitch limiter.

Two more limits from the same header are in the code because the report prints them, not because they were tuned:

- Rudder-pedal command fades from full at 20° angle of attack to zero at 30°. The fade is `(30 − α) / 10`.
- Above 29° the spin-prevention flag is set, the roll-rate command and the sideslip command are zeroed, and the inversion is left to oppose a yaw-rate build-up. The report's spin-mode rudder gain of 0.75 deg per deg/s is not reproduced.

The real F-16, the header notes, is a g command at speed and blends toward pitch-rate command at low speed. This implementation takes a pitch-rate command at every speed and lets the g and angle-of-attack caps bound it. That is a control-law choice. The caps themselves are the numbers above.

## Not the Block 50

Still the 1979 model:

- The force and moment tables, including damping. Low-speed, clean, no Mach on lift or moments, valid in the report only below about Mach 0.6. A Block 50 aero test is not what was transcribed.
- The inertia tensor, measured for 20,500 lb and scaled to 22,972 lb by one mass ratio. The combat load's own inertia and cg were not published and are not in the code.
- The engine deck shape. Idle, military, and maximum tables are the F100-PW-200 installation from TP-1538 table VI. The F110-GE-129 enters only as 17,155 / 14,690 on the military table and 29,500 / 23,930 on the maximum table. Idle is the 1979 idle. Above Mach 1 and above 50,000 ft the deck is an extension of the last tabulated segment, not an F110 lapse.
- The leading-edge flap is 1979 as well, and it is missing here. The report schedules it with angle of attack and q̄/Ps, with a 0.136 s lag, a 25°/s rate limit, and a 25° stop, and appendix B adds flap increments to the coefficients. `F16Data.h` has no flap argument. The Block 50 flap is not modelled either.
- Differential tail and the speed brake are in Table I (differential tail ±5.375° per surface in the table, ±5.38° in the appendix A text, speed brake 60°) and are not states in this code. Roll control is one aileron channel at ±21.5°.
- Block 52 is the other IPE airplane, F100-PW-229. Wikipedia's specification note prints 17,800 lbf dry and 29,160 lbf afterburning for that engine. It is not the scale this file uses.

The big-mouth inlet and the enlarged tail belong to the exterior sheet. They are not in these coefficients.

## Not published

- Mach increments on lift, pitching moment, and the lateral coefficients for a Block 50, or for this 1979 model above about Mach 0.6. The Brandt samples cover zero-lift drag only, and only the five Mach numbers in the card.
- A single combat CLmax, and any corner speed that would need one. The Cz table is the model.
- Drag of the two wingtip missiles. TP-1538 was clean. The mass sum includes the missiles. The polar does not.
- Inertia and centre of gravity of the 22,972 lb build. The mass-ratio scale is an assumption written in `F16Airframe.cpp`.
- A service or manufacturer pilot weight of 200 lb. The figure is only the addend in the repository comment.
- An F110-GE-129 installed thrust deck against altitude and Mach. GE's datasheet gives a 29,000 lb thrust class, not that deck.
- A single official negative-g number. The game uses −3 g. The 1979 report draws a schedule. The Block 50/52 specification line that was opened prints +9 only.
- Block 50 actuator rates, flap schedule, and inertia. The rates in the card are the 1979 rates.

## Sources

Repository, read for this sheet:

- `src/flight/F16Data.h`
- `src/flight/F16Data.cpp` (function bodies and breakpoints only; the grids stay in that file)
- `src/flight/F16Airframe.h`
- `src/flight/F16Airframe.cpp`
- `src/flight/FlightControl.h`
- `src/flight/FlightControl.cpp`
- `src/flight/Jet.h`
- `src/flight/Jet.cpp`
- `research_notes/Famous Fox 2 missiles/flight-model.md`, the Block 50 shooter paragraphs only. Missile employment in that file is not part of this model.

Opened public sources:

- NASA TP-1538, Nguyen, Ogburn, Gilbert, Kibler, Brown, and Deal, *Simulator Study of Stall/Post-Stall Characteristics of a Fighter Airplane With Relaxed Longitudinal Static Stability*, December 1979. NTRS `19800005879`: <https://ntrs.nasa.gov/api/citations/19800005879/downloads/19800005879.pdf>. Table I, appendix A (actuators, 25° limiter, 308°/s, control system B), appendix B (equations, engine momentum, table VI pointed at by the code).
- Wikipedia, "General Dynamics F-16 Fighting Falcon," specifications section "F-16C Block 50 and 52," and the propulsion paragraph that prints 29,588 lbf. <https://en.wikipedia.org/wiki/General_Dynamics_F-16_Fighting_Falcon>. WIKI-ONLY. The USAF fact sheet it cites was not opened (Access Denied): <https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104505/f-16-fighting-falcon/>.
- Wikipedia, "Pratt & Whitney F100," F100-100/200 specification box, 14,690 lbf and 23,930 lbf. <https://en.wikipedia.org/wiki/Pratt_%26_Whitney_F100>.
- GE Aerospace, F110-GE-129 datasheet, thrust class 29,000 lb. <https://www.geaerospace.com/sites/default/files/datasheet-F110-GE-129.pdf>.
- NAVAIR, AIM-9X Sidewinder, launch weight 186 lb. <https://www.navair.navy.mil/product/AIM-9X-Sidewinder>.
- f-16.net, F-16C/D Block 50/52. <https://www.f-16.net/f-16_versions_article9.html>.
- GlobalSecurity, F-16 specifications. <https://www.globalsecurity.org/military/systems/aircraft/f-16-specs.htm>.

Stevens and Lewis is the reduction the code header names. The book itself was not opened. Where a Table I or appendix A number was read in the NASA PDF and matches the code, the code is still the number the game flies.
