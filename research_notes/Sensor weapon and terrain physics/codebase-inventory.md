# Codebase inventory: sensors, signatures, countermeasures, missiles, atmosphere, terrain

Scope is what `missilesim` computes today. Paths are under `C:\Users\alede\Documents\code\missilesim`. Line numbers are the lines read. A constant is "compiled in" unless it is loaded from `assets/config/default_simulation.json` (or the same schema via `SimulationConfig.cpp`). The Fox 2 catalog is compiled C++, not JSON.

Two missiles share one class. The custom round is the JSON airframe (player role is not Fighter). A catalog Fox 2 replaces that airframe in `Missile::configureFox2` when the player is the fighter (`ApplicationMissile.cpp` 608–622). Guidance, seeker, drag, fuze, and thrust then take the Fox 2 branch.

## What stands in for heat, lock, flares, FOV, gimbal, track rate, PN, thrust, drag, radar, and RCS?

### Takeaway

Infrared is a dimensionless "heat" score divided by range squared. There is no radiant intensity in watts, no band, no atmospheric transmission, and no radar or RCS. Seeker "FOV" is either a screen-pixel circle (custom round, pre-launch only) or an angle about the body or boresight (Fox 2). Proportional navigation is two different formulas. Thrust and drag are the parts that already use physical quantities; most of the drag and thrust numbers for catalog rounds are stand-ins, not measured curves.

### Cited Findings

- Target "heat" is the field `Target::m_heatSignature`, initialized to `1.0f` and never written again. `getHeatSignature()` only returns it (`Target.h` 128, 170). No other `m_heatSignature` assignment exists under `src`.
- A Fox 2 source is `intensity = heat * aspect`, then `irradiance = intensity / range^2`, with `range` floored at 1 m. Visible iff `intensity > 0` and, if `tailAcquisitionM > 0`, `range <= tailAcquisitionM * sqrt(intensity)` (`MissileFox2.cpp` 275–296, function `measureTarget`).
- Aspect is not a radiometric integral. `seekerIntensity` returns a rear lobe `max(0, -cosPsi)`, a rear cone, an all-aspect mix, or an afterburner nose gate (`Fox2Flight.h` 191–256). `cosPsi` is the dot of the target velocity unit vector with the direction from target to missile. Below 1 m/s of target speed, `cosPsi` is forced to `-1` (pure tail) (`MissileFox2.cpp` 281–285; `kMinimumAspectSpeed` at line 13).
- All-aspect forward fraction is capped at `kAllAspectForwardFraction = 0.2` (`Fox2Flight.h` 69, 199–203). Head-on intensity is that fraction; exact tail is 1 for the all-aspect law (`aspectIntensity`).
- AIM-9D is the only afterburner-dependent seeker (`afterburnerScalesTail`, `AspectKind::AfterburnerNoseGate`, `Fox2Catalog.cpp` 133–136). Tail lobe is multiplied by 4 when the target is "in afterburner"; a nose gate of half-angle 20° returns `kAim9dNoseIntensityFractionUnpublished = 0.05` (`Fox2Flight.h` 85–89, 241–252). "Afterburner" is `Target::getThrottle() >= kAiAfterburnerThrottle` with `kAiAfterburnerThrottle = 0.85` (`MissileFox2.cpp` 77–80; `Fox2Flight.h` 117–119). The player fighter's afterburner is not read by this test.
- Custom in-flight score is `(targetHeat / distanceSq) * angleWeight * primaryTrackBias`. Rear aspect only scales heat by `mix(0.55, 1.0, rearAspect)` when target speed exceeds 0.5 m/s (`Missile.cpp` 215–230, `updateHeatSeeker`). No `1/r^2` extinction term beyond the inverse-square score.
- Flare "effectiveness" is the same score using `Flare::getHeatSignature()` (`Missile.cpp` 275). Fox 2 compares `flareIrradiance = heat / range^2` with `kUnhardenedSeductionRatio = 2` times primary irradiance (`MissileFox2.cpp` 519–554; `Fox2Flight.h` 76–77).
- Custom pre-launch lock is not an angle. `findSeekerCueTarget` keeps the target whose projected screen point is inside `m_seekerCueRadiusPixels` (default 44 px) of the view center (`ApplicationMissile.cpp` 708–755; `SimulationConfig.h` 80; JSON `seeker_cue_radius_px` 64). In-flight custom FOV is `m_trackingAngleDegrees` (default 85°) about the velocity vector (`Missile.cpp` 189–213, 411–418).
- Fox 2 gimbal is `spec.gimbalDeg`, a half-angle cone about the body axis. Boresight is rate-limited by `trackRateDegPerS` and then clamped into that cone (`MissileFox2.cpp` 467–469). If `ifovDeg > 0`, a source is inside the seeker only when it is within `ifovDeg/2` of the boresight; if `ifovDeg` is 0, the test is body angle `<= gimbalDeg + 0.05°` and ignores the boresight (`MissileFox2.cpp` 488–500). Only AIM-9B (4°), AIM-9D (2.5°), AIM-9H (2.5°, `ifovPublished = false`), and R-3S (3.5°) set `ifovDeg` (`Fox2Catalog.cpp` 96–97, 137–138, 168–169, 307–308). Every other round leaves `ifovDeg` at 0 (`Fox2Catalog.h` 66).
- Track rate does not enter the acceleration command. Guidance aims at the target, the flare, the memory point, or a 1.5 s lead point (`MissileFox2.cpp` 447–466, 638–649). Track rate only slews `m_boresight`. It matters for lock only when `ifovDeg > 0`, because that gate is about the boresight.
- Custom PN, `applyGuidance`: `omega = (R × Vr) / (R·R)`, `Vc = -(Vr · u_LOS)`, command `N * max(|Vc|, 0.1*speed) * (omega × u_LOS)`, then stripped of the velocity component (`Missile.cpp` 421–440). `N` is `clamp(m_navigationGain, 1, 6)`.
- Fox 2 PN, `applyFox2Guidance`, only while locked or on track memory: `N * speed * (omega × u_velocity)` with `N` clamped to 3..5, plus a full-authority blend toward the collision course when heading error is between 15° and 40° (`MissileFox2.cpp` 701–753). Lock-after-launch before any lock is not PN: lateral command is the component of the line of sight normal to velocity, times `speed / 1.5 s` (`MissileFox2.cpp` 755–759; `kInertialLeadSeconds` line 12).
- Thrust magnitude is `throttle * (m_thrust + (P_exit - P_ambient) * A_exit)`, floored at 0 (`Missile.cpp` 564–571, `applyThrust`). `m_thrust` is sea-level momentum thrust. Fox 2 multiplies the axial part by `m_axialThrustScale` when thrust vectoring spends lateral thrust (lines 573–576, 771–776).
- Drag force is `q * Cd * A` opposite velocity, `q = 0.5*rho*v^2` (`Drag.cpp` 42–82). Catalog rounds replace `Cd0` with `kAim9bDragScale * fleemanBodyCd0(...)` (`MissileFox2.cpp` 216–226). Induced drag is `Cl^2 / (pi * AR * e)` (`Aerodynamics.h` 78–88).
- No radar range equation, no RCS variable, no chaff type, and no seeker that consumes a radar track. Catalog card strings mention radar or helmet as a designation source (`Fox2Catalog.cpp`, for example PL-9C card at 787 and MICA card at 609). The running code designates by geometry only (`updateFox2Prelaunch`, `MissileFox2.cpp` 300–382).

### Inferences

- `heat` and `irradiance` impersonate in-band radiant intensity and irradiance. The code never states watts or W/m². A target score of 1 and a flare score of 5.5 are a fixed ratio, not a plume model.
- The AIM-9D factor of 4 impersonates "afterburner doubles detection range" (`Fox2Catalog.cpp` 150) because range scales with `sqrt(intensity)` when a range gate exists. AIM-9D's `tailAcquisitionM` stays 0, so the factor does not open or close a range gate; it only changes the flare-to-target irradiance ratio and opens the 20° nose gate.
- `m_maxSteeringForce` (default 20,000 N) is stored, loaded, and shown as "Maximum lateral force guidance may command" (`ApplicationUI.cpp` 499; `Missile.h` 195). `applyGuidance` and `applyFox2Guidance` never read it. Lateral authority is dynamic pressure, `Cn` or `maxLiftCoefficient`, and a g cap.
- `Missile::m_liftCoefficient` (JSON 0.1) is not the maneuver coefficient. `Lift::applyTo` returns immediately for type `"Missile"` (`Lift.cpp` 17–22). Maneuver lift is `AeroProfile::maxLiftCoefficient` (JSON 20 for the custom round; catalog `resolvedCnMax` for Fox 2).

### Gaps

- `tools/fox2_kinematics/main.cpp` searches a drag scale against two handbook ranges (lines 52–71) and does not write `kAim9bDragScale`. This inventory did not run the tool, so the comment's 8,578 m and 13,122 m residuals (`Fox2Flight.h` 48–51) were not re-measured.
- Fighter exhaust pixels (`ApplicationFighter.cpp` 402–409) and target exhaust pixels (`ApplicationEffects.cpp` 272–277) were not traced into the seeker. Nothing in `measureTarget` or `updateHeatSeeker` reads those intensities.

## Named constants and the quantity each one impersonates

### Takeaway

Shared seeker, fuze, aero, and time-step constants are a few dozen named floats. Per-round mass, diameter, gimbal, IRCCM class, and fuze gates live in `Fox2Catalog.cpp` and are filled in by `resolve`. JSON covers only the custom SAM, the flat-Earth environment, the target AI, MAWS, and the flare dispenser. Viscosity is computed and unused. Several stored Fox 2 fields (`vaneDeg`, `spanM`) do not enter the integrator.

### Cited Findings

Shared seeker and countermeasure constants:

| Symbol | Value | Units as written | Where | Function | Impersonates |
| --- | --- | --- | --- | --- | --- |
| `m_heatSignature` | 1.0 | none | `Target.h` 170 | field initializer | whole-aircraft in-band intensity |
| `FlareDispenserConfig::heatSignature` | 5.5 | none | `Target.h` 35; JSON line 109 | dispenser | flare intensity at ejection |
| `heatDecayRate` | 1.3 (JSON) / 1.4 (`Flare.h` 15, 45) | 1/s implied by `exp(-rate*dt)` | `Flare.cpp` 36 | `Flare::update` | cooling time constant |
| flare death floor | 0.01 | same as heat | `Flare.cpp` 38 | `Flare::update` | extinguished flare |
| `kUnhardenedSeductionRatio` | 2.0 | irradiance ratio | `Fox2Flight.h` 77 | flare loop in `updateFox2InFlight` | unhardened reticle handover |
| `kRiseRatioIllustration` | 2.5 | irradiance ratio | `Fox2Flight.h` 72 | rise IRCCM, `MissileFox2.cpp` 541–544 | open-article rise test, not a missile spec |
| `kRiseWindowIllustrationS` | 0.040 | s | `Fox2Flight.h` 73 | same | rise window |
| `kImagingSeparationDeg` | 1.0 | deg | `Fox2Flight.h` 97 | imaging reject, `MissileFox2.cpp` 524–526 | angular split of a second source |
| `kKinematicSeparationDeg` | 2.0 | deg | `Fox2Flight.h` 101 | kinematic reject, lines 528–533 | off-target gate |
| `kKinematicFlareSpeedFraction` | 0.85 | fraction of target speed | `Fox2Flight.h` 102 | same | "flare has slowed" |
| kinematic speed floor | 30 | m/s | `MissileFox2.cpp` 531 | same | target fast enough to judge a slow flare |
| `kAllAspectForwardFraction` | 0.2 | fraction of tail intensity | `Fox2Flight.h` 69 | `aspectIntensity` | forward-quadrant cap |
| `kAim9dNoseIntensityFractionUnpublished` | 0.05 | fraction of unboosted tail | `Fox2Flight.h` 89 | nose gate | unpublished nose-on intensity |
| nose half-angle (AIM-9D) | 20 | deg | `Fox2Catalog.cpp` 135 | `seekerIntensity` | afterburner head-on gate |
| tail *4 | 4 | multiplier | `Fox2Flight.h` 245 | same | afterburner brightness |
| `kAiAfterburnerThrottle` | 0.85 | throttle fraction | `Fox2Flight.h` 119 | `targetAfterburner` | reheat, AI target only |
| `kDecoyAngularSeparationMinDegrees` | 0.15 | deg | `Missile.cpp` 11 | `updateHeatSeeker` | custom decoy still on the target line |
| `kDecoyAngularSeparationMaxDegrees` | 1.6 | deg | `Missile.cpp` 12 | same | custom decoy fully separated |
| `kMinimumVelocityCoherence` | 0.05 | fraction | `Missile.cpp` 13 | same | floor on velocity match |
| velocity mismatch smoothstep | 0.08 to 0.45 | fraction of target speed | `Missile.cpp` 288 | same | flare no longer co-speed |
| directional coherence mix | 0.1 to 1 | fraction | `Missile.cpp` 279 | same | flare still along the target LOS |
| `irccmBlend` | `0.45 + 0.55*resistance` | fraction | `Missile.cpp` 292 | same | how hard custom IRCCM is |
| switch margin | `mix(1.35, 2.5, resistance)` | score ratio | `Missile.cpp` 297 | same | flare must beat the aircraft by this |
| `m_lockRetentionBias` | 0.25 | fraction | `Missile.h` 199 | custom score, lines 228 and 306 | stickiness; not in JSON |
| `m_countermeasureResistance` | 0.65 | 0..1 | `Missile.h` 198; JSON line 60 | custom score only | not used by Fox 2 IRCCM |
| `m_trackingAngleDegrees` | 85 | deg, clamped 5..180 | `Missile.h` 196; `Missile.cpp` 75 | custom seeker and self-destruct | cone about velocity |
| rear-aspect heat mix | 0.55 to 1.0 | fraction of heat | `Missile.cpp` 222 | custom target score | dimmer off the tail |
| `m_seekerCueRadiusPixels` | 44 | px | `SimulationConfig.h` 80 | `findSeekerCueTarget` | pre-launch custom designation |
| audio lock floor | 0.66 + 0.34*pixel signal, else 0.72 | fraction | `ApplicationEffects.cpp` 163–168 | `updateAudioFrame` | tone strength, not a lock test |
| `kTrackMemorySeconds` | 3.0 | s | `MissileFox2.cpp` 18 | `fox2OnTrackMemory` | coast on last track |
| `kInertialLeadSeconds` | 1.5 | s | `MissileFox2.cpp` 12 | LOAL pursuit | lead time before lock |
| `kTurnBlendStartRad` / `Full` | 15° / 40° | rad in source | `MissileFox2.cpp` 23–24 | Fox 2 PN blend | when to add a hard turn |
| `kBodyAlignRateRadPerS` | 1.5 | rad/s | `Fox2Flight.h` 115 | `updateFox2InFlight` 411–412 | body axis chasing velocity |
| gimbal slack | 0.05 | deg | `MissileFox2.cpp` 343, 365, 499 | pre-launch and geometry | comparison epsilon |
| `kUnpublishedTrackRateDegPerS` | 180 | deg/s | `Fox2Flight.h` 93 | `resolve` if unpublished and `<= 0` | rate so high the gimbal stops the head |
| default gimbal if `<= 0` | 40 | deg | `Fox2Catalog.cpp` 1025–1028 | `resolve` | sim stop |
| navigation gain | 4, custom clamp 1..6, Fox 2 clamp 3..5, UI 1..4 | dimensionless N | `Missile.h` 194; `Missile.cpp` 431; `MissileFox2.cpp` 143, 712; `ApplicationUI.cpp` 497 | both guidance functions | PN gain |
| `kUnpublishedStructuralG` | 30 | g | `Fox2Flight.h` 112 | `resolve` when `structuralGPublished` is false | airframe cap |
| `kAim9bStructuralG` | 10 | g | `Fox2Flight.h` 42 | AIM-9B only | published cap |
| `kAim9bCnMax` | 5.49 | body CN | `Fox2Flight.h` 41 | shape-coefficient rounds | CN inverted from 4.2 g at 50,000 ft, Mach 2.3 |
| `kAim9bStructuralDynamicPressure` | 1.02e5 | Pa | `Fox2Flight.h` 337 | `cnMaxForStructuralG` | q where that CN hits 10 g |
| `kGravity` | 9.80665 | m/s² | `Fox2Flight.h` 12; also `PhysicsObject.cpp` 8 | g caps, Isp | standard gravity |
| engine gravity | 9.81 | m/s² | `PhysicsEngine.cpp` 39; JSON line 6 | `Gravity` | weight, direction `(0,-1,0)` (`Gravity.h` 23) |

Thrust, mass, drag, atmosphere:

| Symbol | Value | Units | Where | Function | Impersonates |
| --- | --- | --- | --- | --- | --- |
| custom thrust | 10000 | N | JSON 48; `Missile.h` 208 | `applyThrust` | sea-level full-burn thrust |
| custom fuel | 100 | kg | JSON 49 | `synchronizeMass` | propellant added to dry mass |
| custom mdot | 0.5 | kg/s | JSON 50 | `applyThrust` | mass flow; `Ve = thrust/mdot` (`Missile.h` 147) |
| custom `A_e` | 0.01 | m² | JSON 51 | back-pressure term | nozzle exit area |
| custom `P_e` | 101325 | Pa | JSON 52 | same | design exit pressure |
| `kSeaLevelPressure` | 101325 | Pa | `Fox2Flight.h` 27 | Fox 2 exit pressure, set in `configureFox2` line 142 | same, called out as not a Sidewinder measurement |
| `kAssumedExitAreaRatio` | 0.4 | `A_e / body area` | `Fox2Flight.h` 23 | `configureFox2` 141; `fleemanBodyCd0` 146 | nozzle area and powered base drag |
| `kAim9bDragScale` | 0.900161 | dimensionless | `Fox2Flight.h` 52 | `sampleZeroLiftDrag` | scale on Fleeman `Cd0`; every catalog round uses it |
| `kAssumedNoseFineness` | 2.4 | `l_N / d` | `Fox2Flight.h` 18 | wave drag | Fleeman baseline nose |
| friction factor | 0.053, exponent 0.2 | mixed | `Fox2Flight.h` 142 | `fleemanBodyCd0` | skin friction |
| base `Cd` | `0.12+0.13*M^2` below Mach 1, else `0.25/M`; powered times `(1-0.4)` | `Cd` | `Fox2Flight.h` 144–146 | same | base drag |
| wave | `(1.59 + 1.83/M^2) * noseAngle^1.69` | `Cd` | `Fox2Flight.h` 148–154 | same, Mach > 1 | nose wave drag |
| transonic peak | 1.25 times the higher endpoint; peak Mach 1.02; blend to 1.2 | `Cd` ratio | `Fox2Flight.h` 168–183 | same | Hoerner hump, height not a Sidewinder measurement |
| feet conversion | `* 3.280839895013123` | ft/m | `Fox2Flight.h` 140 | friction argument | Fleeman's q and length in imperial |
| psf conversion | Pa `/ 47.880258888889` | psf | `Fox2Flight.h` 141 | same | same |
| `kAssumedFinAspectRatio` | 4 | dimensionless | `Fox2Flight.h` 124 | Fox 2 induced drag | fin AR, not measured |
| `kAssumedOswaldEfficiency` | 0.8 | dimensionless | `Fox2Flight.h` 125 | same | span efficiency |
| custom AR / Oswald / `Cl_max` / load | 18 / 0.8 / 20 / 40 g | mixed | JSON 32–35; also constructor `Missile.cpp` 52–57 before config overwrites | custom guidance limit | normal-force ceiling and structure |
| default drag-rise curve | Mach samples 0, 0.8, 0.95, 1.05, 1.2, 2, 3, 5 with multipliers 1, 1.1, 1.9, 3.8, 3.5, 2.6, 2, 1.8 | `Cd0(M)/Cd0(0)` | `Aerodynamics.h` 95–107; JSON 36–45 | custom `Cd0` only | transonic rise; Fox 2 clears this curve (`MissileFox2.cpp` 131) |
| custom `Cd0` | 0.1 | `Cd` | JSON 29 | custom drag | low-Mach zero-lift drag |
| custom area | 0.1 | m² | JSON 30 | reference area | not a 5-inch body (`bodyArea` of 0.127 m is about 0.0127 m²; that figure is not in the source) |
| custom dry mass | 100 | kg | JSON 28 | mass | not a Fox 2 mass |
| ISA sea-level T, P, rho, a | 288.15 K, 101325 Pa, 1.225 kg/m³, 340.294 m/s | as named | `Atmosphere.h` 10–13; `Atmosphere.cpp` 12–13 | `sample` | ISA sea level |
| `R_specific` | 287.05287 | J/(kg·K) | `Atmosphere.cpp` 10 | pressure and density | dry air |
| `gamma` | 1.4 | dimensionless | `Atmosphere.cpp` 11 | speed of sound | `sqrt(gamma*R*T)` |
| Earth radius | 6356766 | m | `Atmosphere.cpp` 8 | geopotential | `h_gp = r*h/(r+h)` |
| g in hydrostatics | 9.80665 | m/s² | `Atmosphere.cpp` 9 | layer pressure | ISA |
| layer tops | 11000, 20000, 32000, 47000, 51000, 71000, 84852 | geopotential m | `Atmosphere.cpp` 18–26 | `sample` | ISA layer tops |
| lapse rates | -0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002 | K/m | `Atmosphere.cpp` 28–34 | `sample` | ISA |
| altitude clamp | -5000 to 86000 | geometric m | `Atmosphere.h` 19–20 | `sample` | table ends |
| Sutherland | `mu_ref=1.716e-5` at 273.15 K, `S=110.4` K | Pa·s, K | `Atmosphere.cpp` 14–16, 70–75 | `calculateDynamicViscosity` | viscosity; no caller reads it (`dynamicViscosity` appears only in `Atmosphere.h` / `Atmosphere.cpp`) |
| density scale | `rho_sl / 1.225` | dimensionless | `Atmosphere.cpp` 95–103, 158 | `sample` | multiplies pressure, hence density; temperature and sound speed stay ISA |
| fixed step | 0.01 | s | JSON line 5; `Application.h` 410 | frame accumulator | outer physics step |
| max steps per frame | 5 | count | `ApplicationLifecycle.cpp` 518 | main loop | frame clamp |
| `kMaxSubStepSeconds` | 0.0025 | s | `PhysicsEngine.cpp` 13 | `update` | 400 Hz ceiling |
| `kMaxSubStepsPerFrame` | 32 | count | `PhysicsEngine.cpp` 14 | `update` | sub-step cap |
| integrator | semi-implicit Euler | — | `PhysicsObject.cpp` 73–80 | `PhysicsObject::update` | `v += a dt`, `x += v dt` |
| Fox 2 inner step in the tail-chase helper | 0.01 | s | `Fox2Flight.h` 271 | `integrateTailChase` | checker only, not the game loop |
| fighter inner step | 1/400 | s | `Jet.h` 24 | `Jet::step` | player jet, not the missile |

Custom cold launch (`ApplicationMissile.cpp` 49–68 and `Application.h` 156–168): eject 26 m/s at 82° pitch; ground profile if within 12 m of the pad plus 1.6 m clearance; position offset 0.45 m along eject; ignition delay 0.85 s (0.05 s if already airborne); throttle at light-up 0.35; thrust ramp 0.30 s; boost multiplier 4 for 1.5 s; pitch-over 110°/s; handoff when velocity is inside 55° of the line of sight (`ApplicationMissile.cpp` 370–372); guidance backstop 1.30 s. These impersonate a SAM eject, boost, and pitch-over. They are not loaded from JSON. Fox 2 launch adds no rail delta: velocity is the fighter velocity (`ApplicationFighter.cpp` 341). Rail offsets `kRailOutboard=0.762`, `kRailBelow=0.16`, `kRailAft=0.36` are fractions of the fighter draw radius (lines 30–32), not a launcher stroke.

Fox 2 motor stand-in, `resolve` (`Fox2Catalog.cpp` 976–994): if the motor is `UnpublishedStandIn` or thrust or burn is non-positive, `resolvedThrustN = kAim9bThrustN * (massKg / kAim9bMassKg)`, `resolvedBurnS = kAim9bBurnS` (2.2 s), `propellantKg = 0`. That impersonates AIM-9B thrust-to-weight for 2.2 s. The three motors that are not this stand-in:

- AIM-9B: `160 lb * 0.45359237` kg, impulse `8440 lbf·s / 2.2 s * 4.448221615` N, length `111.5 in * 0.0254`, diameter `5 in * 0.0254`, span `22 in * 0.0254` (`Fox2Flight.h` 29–36; `Fox2Catalog.cpp` 83–86). Propellant mass 0, so mass is constant (`configureFox2` counts propellant only for `DerivedAverage`, `MissileFox2.cpp` 110–122).
- AIM-9D: `3500 lbf * 4.448221615` N for 5 s, mass `195 lb * 0.45359237` kg, length 2.87 m, diameter 0.127 m, span 0.63 m (`Fox2Flight.h` 54–58; `Fox2Catalog.cpp` 124–129).
- R-3S: thrust `38100 N·s / 2.45 s`, propellant 20.5 kg inside launch mass 75.3 kg, length 2.838 m, diameter 0.127 m, span 0.528 m (`Fox2Flight.h` 60–65; `Fox2Catalog.cpp` 296–303). This is the only round whose fuel field is real propellant (`Missile.cpp` 118–125).

`vaneDeg` is stored (`kJetVaneDegrees = 10`, `kJetTabDegreesUncertain = 15`, `Fox2Flight.h` 79–83) and is not read by `applyFox2Guidance`. TVC lateral acceleration is `thrust/mass` while burning, not `T*sin(vane)` (`MissileFox2.cpp` 679–688). The comment at 679–684 says the older sine-of-vane model was removed. `spanM` is stored on the spec and is not an input to `bodyArea`, drag, or CN. Reference area is `pi*(d/2)^2` (`Fox2Flight.h` 127–130). `diameterIsAssumption` is set only for PL-8 (`Fox2Catalog.cpp` 751) and does not change the equations.

Catalog rounds that change the seeker or the g cap (all other fields are the family defaults above). Citations are the constructor lines read:

| id | mass, length, diameter | aspect / IRCCM / homing | gimbal, cue, track | g cap | fuze / acquisition | TVC |
| --- | --- | --- | --- | --- | --- | --- |
| `aim-9b` | formula above | rear hemisphere, no IRCCM, lock before launch | 25°, IFOV 4°, track 11°/s | 10 g, CN 5.49 | inhibit 0.5 s, guidance 20 s, destruct 24 s, arm 150 m, proximity `30 ft * 0.3048` | no (`Fox2Catalog.cpp` 77–115) |
| `aim-9d` | formula above | afterburner nose gate | 40°, IFOV 2.5°, track 12°/s | shape CN, unpublished g so burn/coast cap 30 g | guidance and destruct 60 s, arm `1000 ft * 0.3048` published, proximity `17 ft * 0.3048` not published | no (118–151) |
| `aim-9h` | 186 lb, 2.87 m, 0.127 m | rear hemisphere | gimbal 40° unpublished, IFOV 2.5° unpublished, track 20°/s | shape CN, cap 30 g | `aim9dArm` inferred | no (154–177) |
| `aim-9j` | 77 kg, 3.05 m, 0.127 m, span 0.58 m | rear hemisphere | gimbal 40° unpublished, track 16.5°/s, IFOV 0 | shape CN, cap 30 g | guidance limit 40 s, inferred arm | no (180–202) |
| `aim-9l` | 86 kg, 2.85 m, 0.127 m | all-aspect, kinematic, lock before launch | gimbal 40° unpublished, track 180°/s | shape CN, cap 30 g | inferred AIM-9D arm | no (913–916) |
| `aim-9m` | same geometry | all-aspect, rise, reduced smoke | same | same | same | no (917–920) |
| `aim-9p-5` | 190 lb, 10 ft, 0.127 m, span 1.9 ft | all-aspect, rise | gimbal 40° unpublished, track 16.5°/s unpublished | shape CN, cap 30 g | inferred arm | no (232–257) |
| `aim-9l-i` | 84 kg, 2.87 m | all-aspect, kinematic | lima defaults | shape CN, cap 30 g | inferred arm | no (928–931) |
| `aim-9l-i-1` | same | all-aspect, rise | same | same | same | no (932–935) |
| `aim-9x-blk1` | 186 lb, 3.02 m, 5 in, span 17.6 in | all-aspect, imaging, lock before launch, not rear-hemisphere designation | gimbal 90° published, track 180°/s | shape CN, cap 30 g | inferred AIM-9D arm, reduced smoke | jet vane (260–287, call 922–923) |
| `aim-9x-blk2` and `blk2plus` | same body | imaging, lock after launch, `rearHemisphereDesignation` | same | same | same | jet vane (924–927) |
| `r-3s` | 75.3 kg, 2.838 m, 0.127 m | rear through the beam (forward of the beam is dark) | gimbal 28°, IFOV 3.5°, track 180°/s | shape CN, cap 30 g | inhibit 0.6 s, guidance 21 s, destruct 22 s unpublished, arm 0.4 s after burnout, proximity 9 m | no (290–323) |
| `r-13m` | 90 kg, span 0.632 m, length 2.87 m | rear hemisphere | gimbal 40° unpublished, track 180 | shape CN, cap 30 g | guidance and destruct 55 s, inferred arm 150 m | no (937–938) |
| `r-13m1` | 90.6 kg, span 0.651 m | rear hemisphere | same | same | no 55 s timer, inferred arm 150 m | no (939–940) |
| `r-60` | 43.5 kg, 2.096 m, 0.120 m, span 0.390 m | rear hemisphere | cue 12° published, gimbal 35° unpublished, track 35°/s | sized to 47 g | guidance 24 s, destruct 25 s, inferred arm 250 m | no (353–387) |
| `r-60m` | 44 kg, 2.138 m | all-aspect | cue 20°, gimbal 35° unpublished, track 180 | sized to 47 g | arm 150 m | no |
| `r-73` | 105 kg, 2.90 m, 0.170 m, span 0.510 m | all-aspect, IRCCM none, lock after launch | cue 45°, gimbal 75°, track 60°/s | shape CN, published cap 40 g | inferred arm 300 m, proximity 3.5 m, tail gate 0 | jet tab (390–437, call 943–944) |
| `r-73m` | 110 kg, same dimensions | kinematic | cue 45°, gimbal 75°, track 180 | same 40 g | arm 300 m, tail gate 0 | jet tab (945–948) |
| `rvv-md` | 106 kg, 2.92 m, 0.170 m | kinematic, lock after launch | cue 60°, gimbal 75°, track 180 | shape CN, cap 30 g because g is not published | arm 300 m, `tailAcquisitionM = 10000/sqrt(0.2)` | jet tab (949–952) |
| `r-27t` | 245 kg, 3.80 m, 0.230 m, span 0.77 m | all-aspect, lock before launch | cue 55° published, gimbal 55° unpublished, track 180 | sized to 8 g | inferred arm 500 m | no (439–474) |
| `r-27et` | 343 kg, 4.50 m, 0.260 m, span 0.80 m | all-aspect | cue 55° not published, gimbal 55° | shape CN, cap 30 g | arm 500 m | no |
| `magic-1` | 89 kg, 2.75 m, 0.157 m, span 0.66 m | rear cone 70° half-angle, no IRCCM | gimbal 30°, track 180 | sized to 35 g | arm time 1.8 s, destruct 26 s, inferred arm 300 m | no (477–518) |
| `magic-2` | same geometry | all-aspect, rise | gimbal 30° | sized to 50 g | inferred arm 150 m; 1.8 s and 26 s are not copied | no |
| `iris-t` | 87.4 kg, 2.936 m, 0.127 m, span 0.447 m | all-aspect, imaging, lock after launch, rear hemisphere | gimbal 90°, track 180 | shape CN, published cap 60 g | inferred arm 150 m | jet vane (521–549) |
| `asraam` | 88 kg, 2.90 m, 0.166 m, span 0.45 m | all-aspect, imaging, lock after launch, rear hemisphere | gimbal 90°, track 180 | sized to 50 g | inferred arm 300 m, reduced smoke | no (552–579) |
| `mica-ir` | 112 kg, 3.10 m, 0.160 m, span 0.480 m | all-aspect, imaging, lock after launch, rear hemisphere | gimbal 60° unpublished, track 180 | sized to 50 g | arm 500 m published, reduced smoke | jet vane (582–610) |
| `shafrir-2` | 94 kg, 2.60 m, 0.160 m, span 0.55 m | rear hemisphere | gimbal 10° | shape CN, cap 30 g | inferred arm 600 m | no (613–632) |
| `python-3` | 120 kg, 2.95 m, 0.160 m, span 0.86 m | all-aspect, lock before launch | cue 30°, gimbal 40° | sized to 40 g | inferred arm 500 m | no (635–657) |
| `python-4` | 105 kg, 3.10 m, 0.160 m, span 0.64 m | all-aspect, kinematic | gimbal 60° unpublished | shape CN, cap 30 g | inferred arm 150 m | no (660–683) |
| `python-5` | same geometry | imaging, lock after launch, rear hemisphere | gimbal 90° unpublished | shape CN, cap 30 g | inferred arm 150 m | no (686–712) |
| `pl-5eii` | 83 kg, 2.893 m, 0.127 m, span 0.617 m | all-aspect, kinematic | gimbal 40° unpublished | sized to 35 g | tail gate 16000 m, inferred arm 150 m | no (715–739) |
| `pl-8` | 115 kg, 2.90 m, diameter 0.160 m flagged assumed | rear hemisphere | gimbal 40° unpublished | sized to 38 g | inferred arm 150 m | no (742–761) |
| `pl-9c` | 115 kg, 2.992 m, 0.157 m, span 0.856 m | all-aspect, kinematic | cue 40°, gimbal 40° unpublished | shape CN, cap 30 g | inferred arm 150 m | no (764–788) |
| `pl-10` | 105 kg, 3.00 m, 0.160 m | all-aspect, imaging, lock after launch, not full sphere | gimbal 90° unpublished | shape CN, published cap 60 g | inferred arm 150 m | jet vane (791–816) |
| `aam-3` | 91 kg, 3.10 m, 0.127 m, span 0.64 m | all-aspect, kinematic | gimbal 40° unpublished | shape CN, cap 30 g | inferred arm 150 m | no (819–842) |
| `aam-5` | 95 kg, 3.105 m, 0.130 m, span 0.412 m | all-aspect, imaging, lock after launch | gimbal 60° unpublished | shape CN, cap 30 g | inferred arm 150 m | jet vane (845–869) |
| `a-darter` | 93 kg, 2.98 m, 0.166 m, span 0.488 m | all-aspect, imaging, lock after launch, not full sphere | gimbal 90°, track 120°/s | aero CN sized to 50 g; burn cap 100 g; coast cap 50 g (`aeroStructuralG`, `coastStructuralG`) | inferred arm 150 m, reduced smoke | jet vane (872–903) |

`cnMaxForStructuralG` (`Fox2Flight.h` 341–348) is `structuralG * mass * 9.80665 / (1.02e5 * bodyArea)`. `resolve` uses `aeroStructuralG` when it is positive, otherwise `structuralG` (`Fox2Catalog.cpp` 1005–1008). Shape-coefficient rounds (`shapeCoefficient`, lines 26–31) keep `resolvedCnMax = 5.49` even if a later line sets a published g cap. IRIS-T, PL-10, and the R-73 family do that: CN stays 5.49 and the g number is only the acceleration cap (`Fox2Catalog.cpp` 533–535, 404–406, 801–803, 1011–1022).

Flare body, not signature: mass 0.9 kg, `Cd` 1.1, area 0.018 m², life 4 s, eject 45 m/s, aft offset 1.2 m, lateral offset 0.8 m, lateral fraction 0.18, downward fraction 0.12, inventory 24, burst 2, interval 0.12 s, cooldown 0.9 s (`Target.h` 22–39; JSON 97–113). No Mach curve: `getAeroProfile` stays null, so drag is constant `Cd` (`Drag.cpp` 71–75; `PhysicsObject.h` 53–56).

Target airframe constants that feed the afterburner flag indirectly, because throttle is thrust over ceiling (`Target.cpp` 108–119, 441–446): mass 8000 kg, load limit 9 g, thrust ceiling 75000 N (constructor; the comment at line 113 says 75000, the header default `m_maxThrust` is 60000 and is overwritten), wing area 12 m², `Cd0` 0.022, AR 6, Oswald 0.85, `Cl_max` 1.4, engine response 1 s (line 441). AI speed band 180–320 m/s is both a command and a hard clamp at 0.5×min to 1.15×max (lines 449–453).

MAWS constants: range 3200 m, reaction window 6 s, closest-approach threshold 140 m (`Target.h` 14–19; JSON 91–95).

Ground and end-of-flight numbers: ground `y = 0` with no setter (`PhysicsEngine.h` 76); restitution 0.5; horizontal friction 0.8; stop if speed under 0.1 m/s (`PhysicsEngine.cpp` 176–190). Flight kill box: custom `max(5000, 5*engagementRadius)` m horizontally and `max(3000, 1.8*engagementRadius)` m altitude; Fox 2 150000 m and 30000 m (`ApplicationLifecycle.cpp` 586–595). Opening-range miss: after 0.75 s, range grew by `max(4*targetRadius, 25)` m, range rate `> 15` m/s, and velocity dot LOS `< -0.15`, custom only (lines 604–618). Dead-stick: motor off, flight `> 2` s, speed `< 15` m/s (lines 624–629). Ground impact: previous `y > 0.05` and current `y <= 0.01` (lines 580–582). Detonation hold 3 s wall clock (`Application.h` 393).

### Inferences

- `0.60 / 0.666548` equals the compiled `kAim9bDragScale` of 0.900161 to the printed digits. The comment at `Fox2Flight.h` 44–47 is consistent with "peak unscaled coast Cd was 0.666548, and k was chosen so the scaled peak is 0.60," not with k itself being 0.666548.
- RVV-MD's tail gate expression `10000/sqrt(0.2)` is the 10 km front-hemisphere figure divided by `sqrt(0.2)`, which the card calls about 22 km of tail lock (`Fox2Catalog.cpp` 951–952). The integrator uses the expression, not a rounded literal.
- A published proximity of 0 with `proximityPublished == false` becomes a fuse radius of 0 (`MissileFox2.cpp` 146). The hit sphere is then only `Target::radius`.

### Gaps

- `spanM` may be displayed in UI cards. This pass did not open every HUD string. It is unused in force and seeker math.
- Per-round `card` strings contain extra rejected numbers (brochure range, Mach claims). Those strings are not integrator inputs except where the same number is also assigned to a `Spec` field in the table above.

## What is data-driven versus compiled in?

### Takeaway

`assets/config/default_simulation.json` drives the custom SAM, the ISA sea-level density scale, gravity, the fixed step, the flat-ground bounce, target spawn and AI, MAWS, and the flare dispenser. It does not list Fox 2 rounds. Every Fox 2 number is a C++ literal in `Fox2Flight.h` and `Fox2Catalog.cpp`. Evasive-maneuver weights are header defaults; the default JSON has no `evasive` object, so they stay compiled unless some other file supplies that object.

### Cited Findings

- Loader: `loadSimulationConfig` reads environment, missile airframe, motor, guidance, target AI, spawn, MAWS, flares, and evasive blocks (`SimulationConfig.cpp` 173, 241–247, 263, 306–312). Defaults if a key is missing are the struct initializers in `SimulationConfig.h`.
- Default file contents are the custom missile (mass 100 kg, `Cd` 0.1, area 0.1 m², `Cl` 0.1, AR 18, Oswald 0.8, `Cl_max` 20, 40 g, the Mach curve, thrust 10000 N, fuel 100 kg, mdot 0.5 kg/s, `A_e` 0.01 m², `P_e` 101325 Pa, N 4, steering force 20000 N, tracking angle 85°, proximity 18 m, countermeasure resistance 0.65, terrain clearance 90 m, lookahead 6 s, cue 44 px) plus environment `dt` 0.01 s, speed 1, gravity 9.81, density 1.225, ground on, restitution 0.5, and the target/MAWS/flare block quoted above (`default_simulation.json` 1–116). No missile id, no seeker band, no atmosphere profile, no terrain mesh.
- `Missile::configureFox2` overwrites thrust, area, CN, navigation gain (forced to 4), proximity, nozzle, and drag path from the compiled spec (`MissileFox2.cpp` 106–167). Slider fields are left in memory for the custom round (`ApplicationMissile.cpp` 608–609).
- Default fighter loadout id if missing is `"aim-9x-blk2"` (`ApplicationMissile.cpp` 615).
- `user_settings.ini` can override the same custom-missile fields (`ApplicationSettings.cpp` 142–151, 278–287). That is still the custom round, not the catalog.
- `tools/fox2_kinematics` and `tools/flight_harness` are offline checkers. The game loop does not call them. `integrateTailChase` is a template in the header used by the kinematics tool (`Fox2Flight.h` 283–333).

### Inferences

- Promoting a Fox 2 number into JSON would be a new path. Today `find(id)` returns a pointer into a function-local `static` vector (`Fox2Catalog.cpp` 906–908, 1047–1060).
- The evasive struct (`SimulationConfig.h` 136–157, mirrored in `Target.h` 46–68) is tactics weights (pitch rate, altitude offsets). It does not scale heat or flare intensity.

### Gaps

- This pass did not open `config/user_settings.ini` or confirm whether a user's ini currently overrides the JSON defaults. The code path exists.
- `SimulationConfig.cpp` was not read line by line beyond the keys found by search. The default JSON was read in full, so the shipped baseline is known.

## Radar, chaff, terrain elevation, line of sight, occlusion, atmosphere attenuation

### Takeaway

None of those are flight or seeker models. The geometric "line of sight" is a unit vector used by proportional navigation. Terrain is the plane `y = 0` plus, for the custom round only, an upward acceleration if predicted altitude is low. Atmosphere attenuation of infrared is absent. Viscosity is unused. Audio and particle smoke have their own propagation and are not inputs to lock or miss.

### Cited Findings

- Searched `src` for `radar`, `chaff`, `RCS`, `rcs`, `occlusion`, `transmittance`, `extinction`, `heightmap`, `elevation`, and `terrain`. Hits are: catalog prose ("helmet or radar slave", `Fox2Catalog.cpp` 787 and similar cards); sun elevation in the renderer (`Renderer.cpp` 375–387); particle-shader transmittance for smoke pixels (`SceneEffectsShaders.cpp` 275–311); audio "terrain echoes" (`AudioEngine.cpp` 67, `Acoustics.cpp` 845); missile terrain-avoidance fields.
- No `chaff` symbol and no RCS field exist in those hits. MAWS does not use a radar equation. It keeps a missile if distance is inside 3200 m, distance is at least 0.1 m, closing speed is positive, time to closest approach is inside 6 s, and predicted miss distance is inside 140 m (`Target.cpp` 204–231, `updateThreatAssessment`). There is no angular gate and no terrain mask.
- Ground is `m_groundLevel = 0` (`PhysicsEngine.h` 76). Nothing calls a setter; the symbol `setGroundLevel` does not exist. Collision is `position.y < 0`, snap to 0, reflect vertical speed by restitution (`PhysicsEngine.cpp` 161–196).
- Custom terrain avoidance (`Missile.cpp` 442–455): if `y + vy*lookahead < clearance`, add `a_y = 2*deficit/lookahead^2`. `clearance` default 90 m, lookahead default 6 s, minimum lookahead 0.5 s. Fox 2 calls `setTerrainAvoidanceEnabled(false)` (`MissileFox2.cpp` 145). The comment in the flight UI says the catalog rounds do not use it (`ApplicationUI.cpp` 404).
- Seeker vectors are world-space differences. `projectWorldPointToScreen` (`ApplicationMissile.cpp` 680–694) can reject a target that is not in the camera frustum. That is pre-launch custom designation, not terrain occlusion. The x-ray overlay is documented as drawing through terrain (`ApplicationUI.cpp` 633; comment at `ApplicationMissile.cpp` 944).
- Infrared irradiance has no `exp(-beta * range)` factor (`MissileFox2.cpp` 292; `Missile.cpp` 230, 275).
- `Atmosphere::sample` returns temperature, pressure, density, sound speed, and both viscosities (`Atmosphere.cpp` 111–178). Drag and guidance read density and, for Mach, sound speed (`Drag.cpp` 40–42; `PhysicsEngine.cpp` 109–116). Viscosity fields have no reader outside `Atmosphere`.
- Acoustic speed of sound is a separate setting (`AudioEngine.cpp` 682; `Acoustics.h` 56). It was not traced into `Missile` or `Target`. It does not attenuate the heat score.

### Inferences

- "Line of sight" in `applyGuidance` (`Missile.cpp` 409) is the kinematic PN unit vector `R/|R|`. It is not a ray cast.
- Predicted clearance assumes the ground stays at `m_groundReferenceAltitude`, which is set from `m_groundLevel` every sub-step (`PhysicsEngine.cpp` 104). A hill cannot exist in this model.
- Visual terrain mottling (`RendererAssets.cpp` 590, from the terrain search) is a texture comment, not an elevation sample. It was not read past the search hit.

### Gaps

- `Acoustics.cpp` was not read in full. The search shows outdoor echoes and a speed of sound. Whether those echoes use a height field was not verified. They are not consulted by guidance.
- Shader transmittance (`SceneEffectsShaders.cpp`) was not read past the search hit. It is a fragment-shader smoke integral, not a seeker.

## How a missile acquires, guides, and ends

### Takeaway

Acquisition, guidance, and termination are different predicates, and the custom round and the Fox 2 do not share them. A flare does not set a "defeated" flag. It steals the aim point. A hit is a swept sphere against any active target and does not ask what the seeker is tracking. Fox 2 shots are not ended for flying past the target; custom shots are.

### Cited Findings

Custom acquisition, before launch, `updatePreLaunchSeekerLock` (`ApplicationMissile.cpp` 855–886): if the seeker cue is off or guidance is disabled, `clearTarget()`. Otherwise `findSeekerCueTarget`: prefer the already tracked target if it still projects on screen; else the on-screen target with smallest pixel distance inside 44 px, tie-broken by range (lines 708–755). Launch is refused when `getTrackedMissileTarget()` is null (`ApplicationMissile.cpp` 118–122).

Custom acquisition in flight, `updateHeatSeeker` (`Missile.cpp` 174–323), only when guidance is enabled, thrust is enabled, and `dt > 0` (lines 184–187). Seeker axis is velocity if thrust is on and speed is not tiny, else thrust direction (lines 189–191). A target is eligible when `dot(axis, direction) >= cos(trackingAngle)`. Best target wins the score. Then every active flare inside the same cone is scored. A flare replaces the target only if it is already the tracked decoy, or its score exceeds `primaryScore * switchMargin`. If no target is inside the cone, `clearTarget()` (lines 238–241), which drops the track entirely.

Custom guidance starts only after the cold-launch handoff sets `m_guidanceEnabled` (`ApplicationMissile.cpp` 343–375, 387–401). Until then `applyGuidance` returns (`Missile.cpp` 341–344). While guiding, the aim point is the live target unless `m_trackingDecoy` (`Missile.cpp` 356–362). Self-destruct is requested when the line-of-sight angle from the velocity exceeds `m_trackingAngleDegrees` (lines 411–418). Below 0.1 m/s the function only points thrust at the target (lines 392–399). Below 0.5 m range it returns without a command (lines 404–407).

Fox 2 acquisition before launch, `updateFox2Prelaunch` (`MissileFox2.cpp` 300–382): search cone is 180° if `rearHemisphereDesignation`, else the cue if lock-after-launch and `cueDeg > 0`, else the gimbal (lines 312–322). Best target is the smallest angle inside that cone, tie-broken by range (lines 348–353). Infrared lock is `signal.visible && angle <= gimbalDeg + 0.05°` (lines 363–366). Lock-after-launch may designate without that infrared lock (lines 367–370). `launchFox2FromRail` blocks if there is no designation, and blocks lock-before-launch rounds that lack `hasFox2InfraredLock` (`ApplicationFighter.cpp` 324–336). `beginFox2Flight` starts the motor, sets the inhibit timer to `inhibitS`, and sets `m_fox2Locked` from the boolean passed in (`MissileFox2.cpp` 183–213).

Fox 2 in flight, `updateFox2InFlight` (`MissileFox2.cpp` 384–614). Guidance expires when `guidanceLimitS > 0` and flight time reaches it (lines 398–401). Self-destruct is requested when `selfDestructS > 0` and flight time reaches it (lines 402–405). While not expired, the boresight slews toward the flare, the locked target, the memory point, or the designated target, at `trackRate`, then clamps to the gimbal (lines 447–469). Lock holds on a flare that passes IRCCM and `geometryHolds` and beats twice the target irradiance (lines 502–591). Otherwise lock holds on the target if `primary.visible && geometryHolds` (lines 596–604). Otherwise `m_fox2Locked = false` but the designation is kept (lines 607–613). Memory age increments every call (lines 394–397) and stays usable while `0 <= age <= 3 s` (lines 170–174).

Fox 2 commands, `applyFox2Guidance` (`MissileFox2.cpp` 616–785), are skipped when guidance has expired, guidance is disabled, or `m_fox2InhibitLeft > 0` (lines 625–628). The three aim modes are the predicate at lines 630–636: tracking (locked on target or flare), memory, or pursuit (never locked, designated target still alive). Anything else produces no lateral command. Inhibit is counted down at the start of `updateFox2InFlight` (line 393), so the first `inhibitS` seconds are unguided thrust.

Fuze arm, `isFuzeArmed` (`MissileFox2.cpp` 229–263): custom rounds return true immediately. Fox 2 returns false until flown distance exceeds `armDistanceM` if that field is positive, until `flightTime >= armTimeS` if that field is positive, and until `armAfterBurnoutS` after `m_fox2BurnoutTime` if that field is positive. Burnout time is stamped when the motor turns off (`Missile.cpp` 588–591).

Hit, `checkMissileTargetHit` (`PhysicsEngine.cpp` 213–245): if the fuze is not armed, return false. Otherwise if the segment from previous position to current position passes within `targetRadius + proximityRadius` of the target center, deactivate that target and return true. The test does not look at `m_trackingDecoy` or lock state. Application then treats any newly inactive target as an interception, adds score, and starts the detonation hold (`ApplicationLifecycle.cpp` 541–559). `beginDetonationHold` disables thrust and guidance, clears the target, zeroes velocity, and removes the missile from the engine (`ApplicationMissile.cpp` 1028–1061).

Other ends, all inside the in-flight block (`ApplicationLifecycle.cpp` 569–638), each calling the same detonation hold:

- `consumeSelfDestructRequest()` is true. Sources: custom seeker walked off the tracking cone (`Missile.cpp` 414–418); Fox 2 `selfDestructS` (`MissileFox2.cpp` 402–405). `clearTarget` on the custom path clears the request (`Missile.cpp` 164–171), so losing every target does not by itself explode the custom round.
- Ground: previous `y > 0.05` and `y <= 0.01` while ground collision is enabled (lines 579–582).
- Outside the kill box (lines 586–595).
- Custom only: opening range behind the missile (lines 604–618). The comment says Fox 2 shots may lose the target aft and keep flying (lines 614–615).
- Motor off, flight time over 2 s, speed under 15 m/s (lines 624–629).

There is no predicate named flare defeat. A decoy ends the shot only by making one of the predicates above true (the missile hits the ground, slows down, leaves the box, or, for the custom round, sees the real target fall behind). A Fox 2 that tracks a flare until the flare dies clears `m_trackingDecoy` (`MissileFox2.cpp` 438–445) and can reacquire.

### Inferences

- Custom seeker updates stop when the motor stops (`Missile.cpp` 184–187), but `applyGuidance` does not. After burnout the last decoy flag and aim point remain, while a non-decoy track still copies the live target position (lines 356–362). The 85° self-destruct test still runs.
- Fox 2 guidance expiry blanks the lock and stops commands (`MissileFox2.cpp` 416–422, 625–628) without requesting self-destruct unless `selfDestructS` is also reached. The round then coasts until the ground, the box, or the 15 m/s rule.
- Because the hit test ignores the seeker, a missile guiding on a flare can still detonate on the aircraft if the sphere test passes. Proximity is not a seeker-cone fuze.

### Gaps

- `createExplosion` was not read. Termination is the detonation hold, not a separate miss dialog.
- Whether a Fox 2 with `selfDestructS == 0` can fly until the 150 km box was inferred from the predicates, not from a logged flight. AIM-9H, AIM-9J, the Lima family, Python, PL-series, and several others leave `selfDestructS` at 0 (`Fox2Catalog.h` 89).

## How target signature is computed, and one update frame with flares

### Takeaway

The target signature does not depend on throttle, Mach, altitude, or aspect except through the seeker's own aspect function. It is the constant 1. Flare signature is the dispenser value, multiplied by `exp(-decay*dt)` after the flare has been integrated. New flares are spawned after the physics step, so the seeker sees them on the next step. Fox 2 aspect uses target velocity as the nose, and the afterburner bit uses the AI throttle.

### Cited Findings

Signature inputs on the target, each step of `measureTarget` (`MissileFox2.cpp` 275–296):

1. Position and the missile position give range and the unit vector from target to missile.
2. If target speed is at least 1 m/s, `cosPsi = dot(normalize(velocity), that unit vector)`. Velocity is the nose. There is no separate body axis on `Target`.
3. `seekerIntensity` maps `cosPsi` through the round's `AspectKind` (`Fox2Flight.h` 216–256).
4. Multiply by `getHeatSignature()`, which is 1 (`Target.h` 170).
5. Divide by `range^2`.

`Target::updateAutonomousFlight` sets `m_throttle = thrustForce / m_maxThrust` after solving the thrust that tracks the desired speed (`Target.cpp` 441–446). That throttle is not multiplied into `m_heatSignature`. It is multiplied into the exhaust sprite (`ApplicationEffects.cpp` 272–277) and, for seekers with `afterburnerScalesTail` or the nose gate, into `targetAfterburner` (`MissileFox2.cpp` 77–80). Only AIM-9D sets `afterburnerScalesTail` (`Fox2Catalog.cpp` 134).

Flare birth, `Target::updateCountermeasures` (`Target.cpp` 614–661): if MAWS is active, the dispenser is enabled, flares remain, no burst is pending, and cooldown is 0, queue `min(burstSize, remaining)` shots and set cooldown. Each shot places the flare at `position + aft*(radius+aftOffset) + lateral*lateralOffset`, with velocity `aircraftVelocity + aft*ejectSpeed + lateral*ejectSpeed*lateralFraction + down*ejectSpeed*downFraction`, and copies heat 5.5 and decay 1.3. `consumePendingFlareLaunches` only moves the queue (`Target.cpp` 255–259). `ApplicationLifecycle.cpp` 662 calls `collectPendingTargetFlares()` after the physics steps, so the new `Flare` is not in `m_flares` during the step that decided to launch it.

Flare aging, `Flare::update` (`Flare.cpp` 21–43): integrate motion, `heat *= exp(-decay*dt)`, deactivate if life is 0 or heat is at most 0.01.

One physics sub-step, `PhysicsEngine::integrateStep` (`PhysicsEngine.cpp` 64–149), for objects already in `m_objects` (missiles and flares; targets are not, `addTarget` only pushes `m_targets`, lines 303–316):

1. Skip objects whose type is `"Target"` (lines 76–79).
2. `resetForces`, gravity, drag. Drag on a Fox 2 calls `sampleZeroLiftDrag` and adds induced drag from the previous sub-step's commanded lift (`Drag.cpp` 46–55). Drag on a flare uses constant `Cd` 1.1 and area 0.018 m². Missile lift from `Lift::applyTo` is skipped (`Lift.cpp` 21–22).
3. For a missile: set ground reference to 0, sample ISA at `position.y`, set ambient pressure, call `updateHeatSeeker(targets, flares, dt)` which sees flares that already exist, then `applyGuidance` if guidance is on and `hasTarget()` (`PhysicsEngine.cpp` 101–117). Fox 2 pursuit can have `hasTarget` without a lock, so LOAL still steers.
4. `object->update`. For a missile that applies thrust and integrates (`Missile.cpp` 607–620). For a flare that integrates and decays heat. Order is the `m_objects` order. A flare earlier in the vector decays before the missile's seeker runs; a flare later in the vector decays after the seeker has already scored it. Insertion order is launch order.
5. Ground snap if `y < 0`.
6. After every rigid body: each active target gets atmosphere density and sound speed, `updateThreatAssessment`, then `Target::update` (flight, airspace clamp, countermeasure queue) (`PhysicsEngine.cpp` 132–146; `Target.cpp` 281–306).
7. `handleTargetCollisions` sweeps missile segments against target spheres (`PhysicsEngine.cpp` 199–208).

The outer step is 0.01 s, split into sub-steps of `0.01/n` with each piece at most 0.0025 s (`PhysicsEngine.cpp` 51–60). Seeker and guidance therefore run four times per 0.01 s step at the default rate. The application, after all sub-steps, may start the detonation hold and only then instantiates queued flares (`ApplicationLifecycle.cpp` 539–662).

Custom scoring in that seeker call (`Missile.cpp` 174–323): build the best aircraft score inside the 85° cone, using heat 1 scaled by the 0.55–1 rear mix and by `1/r^2`. Then for each live flare inside the cone, start from flare heat over `r^2`, multiply by the angle weight, then multiply by a mix between 1 and `directionalCoherence * separationCoherence * velocityCoherence`. Accept the flare only if it clears the resistance-dependent margin. Guidance on the same sub-step steers at the winner.

Fox 2 scoring in that call (`MissileFox2.cpp` 480–604): one designated target, not the best of all targets (`targets` is unused, line 386). Measure it. Walk flares. Reject by class: imaging if separation from the target direction exceeds 1°; kinematic if the flare is slower than 0.85 times target speed (target faster than 30 m/s) and separation exceeds `max(ifov, 2°)` ; rise if this same flare's irradiance grew by 2.5 within 0.040 s of the stored sample. Also reject if `geometryHolds` fails. Among survivors, keep the brightest that is at least twice the target irradiance. If none, lock the target if it is visible and inside the gate.

### Inferences

- A flare launched this step cannot seduce until a later seeker call. With a 0.12 s burst interval and a 0.01 s step, the second pellet of a burst is also delayed by a step.
- Rise IRCCM stores one sample per accepted flare and refreshes it only after the 0.040 s window (`MissileFox2.cpp` 577–590). A flare that is bright on the first sample is not rejected for rising until a later sample is compared. The first look cannot trip the test (comment at lines 537–539).
- Because target heat is 1 and does not fall with aspect inside `measureTarget` except through `seekerIntensity`, a rear-hemisphere round sees intensity 0 in the forward hemisphere and will not lock, regardless of throttle. An all-aspect round still sees 0.2 of tail intensity on the nose, at any throttle.

### Gaps

- `collectPendingTargetFlares` itself was not opened. The call site after physics is enough to place birth after the seeker. The exact `addFlare` order relative to the missile was not logged at runtime.
- Target radius (spawn 3–7 m, fallback 5 m, JSON 78–80) changes the hit sphere and the flare's aft spawn point. It does not change heat.

## Coordinate frames, time steps, and the atmosphere guidance already uses

### Takeaway

Guidance is written in a Y-up world frame in meters. Altitude is `position.y` on a flat plane. The missile does not carry a local-level or body-rate state except a Fox 2 body axis and boresight, both world unit vectors. The atmosphere it reads is ISA temperature and pressure versus geopotential altitude, with density scaled if sea-level density is not 1.225. Guidance uses density for dynamic pressure and ambient pressure for thrust. It does not use wind, viscosity, or humidity.

### Cited Findings

- Positions and velocities are `glm::vec3` meters and m/s (`PhysicsObject.h` 92–94). Gravity is `(0,-1,0)` times 9.81 (`Gravity.h` 23; `PhysicsEngine.cpp` 39). The missile constructor's default velocity is `(0, 0, 50)` m/s (`Missile.h` 15), so +Z is the stock "forward," not a navigation axis.
- Fox 2 body and boresight are world-space unit vectors. Each sub-step the body rotates toward the velocity direction by at most `1.5 rad/s * dt` when speed exceeds 5 m/s (`MissileFox2.cpp` 407–414). Thrust is applied along the body (`Missile.cpp` 576). There is no quaternion missile attitude and no angle-of-attack state. Angle of attack is whatever angle remains between `m_bodyForward` and velocity.
- Custom thrust direction is slewed toward `velocity + commandedAcceleration*dt` while the motor is on (`Missile.cpp` 487–493), which impersonates a small angle of attack, not a moment balance.
- Lateral commands are world vectors with the velocity component removed (`Missile.cpp` 438–440; `MissileFox2.cpp` 762). They are not fin deflections.
- Atmosphere sample argument is `missile->getPosition().y` (`PhysicsEngine.cpp` 109–111). Geometric altitude is clamped, converted to geopotential with a spherical Earth radius, then integrated through the seven lapse layers (`Atmosphere.cpp` 36–47, 111–164). Below the 11 km tropopause the lapse is -0.0065 K/m (lines 28–29).
- `setAmbientPressure` feeds the back-pressure term (`Missile.h` 143–144; `Missile.cpp` 570). `applyGuidance(..., density)` feeds `q = 0.5*rho*v^2` and `a_aero = q*Cn*S/m` (`Missile.cpp` 462–467; `MissileFox2.cpp` 671–674). Sound speed is used in drag's Mach number (`Drag.cpp` 40–41), which changes catalog `Cd0` through `fleemanBodyCd0`. Guidance does not read Mach directly.
- The target uses the same density and sound speed for its own `q` and Mach (`PhysicsEngine.cpp` 139–142; `Target.cpp` 417–438) but integrates with a trapezoid on velocity, not through `PhysicsObject::update` (line 458). The fighter samples the same atmosphere and steps `Jet` at 400 Hz (`ApplicationFighter.cpp` 231–233; `Jet.h` 24; `Fighter.cpp` 77–85). Fighter crash is `y <= ground + 1.5` (`ApplicationFighter.cpp` 235–240).
- No wind vector is added to dynamic pressure or to seeker angles. Relative velocity is inertial velocity difference (`Missile.cpp` 427; `MissileFox2.cpp` 709).

### Inferences

- A guidance law written in this code is already in inertial world axes. Body rates would have to be derived from `m_bodyForward` and `m_boresight`; they are not states.
- Changing sea-level density scales pressure and density together (`Atmosphere.cpp` 158–162) and does not change the temperature profile. Thrust lapse and `q` both move. Sound speed does not.
- The 0.01 s outer step is what the seeker, the 0.040 s rise window, the inhibit timer, and the 3 s memory timer count. Sub-stepping does not change those clocks' units; it calls them more than once per 0.01 s, each time with the sub-step `dt`.

### Gaps

- `F16Airframe.cpp` was not inventoried. The player jet's aero tables are outside the missile seeker, except that the missile inherits the fighter's velocity at release and the fighter's nose as the rail axis.
- Whether `simulationSpeed` in JSON scales `dt` or only the wall clock was not traced past the field declaration (`SimulationConfig.h` 14). The fixed step used in the loop is `m_timeStep` from `fixedTimeStep` (`ApplicationLifecycle.cpp` 52, 521).

## Where a later physical model would have to replace a stand-in

### Takeaway

The code already integrates force, mass, ISA density, and a body-axis `Cd0` build-up. It does not integrate a sensor. Heat, aspect, IRCCM, FOV, and acquisition range are algebraic gates. Replacing them means giving the target and the flare a spectral intensity, putting transmission on the `1/r^2` term, and making lock depend on irradiance at the aperture rather than on a constant 1. The catalog's unpublished motors and the single drag scale are the propulsion and aero stand-ins already marked in comments. This section is the boundary, not a design.

### Cited Findings

- The source itself marks stand-ins: `kStandInMotorNote` (`Fox2Catalog.cpp` 12–15), `kUnpublishedStructuralG` "a sim choice" (`Fox2Flight.h` 104–112), `kAllAspectForwardFraction` "not a radiometric fit" (lines 67–69), rise constants "not an AIM-9M spec" (lines 71–73), imaging and kinematic angles "not published" (lines 95–102), vane angles "not a measured" angle (lines 78–83), AI afterburner "stands in for reheat" (lines 117–119), and `kAim9bDragScale` kept inside a band rather than fit to the handbook range (lines 44–52).
- `resolve` will invent thrust, burn time, CN, a 40° gimbal, a 180°/s track rate, and a 30 g cap when the card left them blank (`Fox2Catalog.cpp` 976–1034). Those outputs are what the game integrates.

### Inferences

- A physical infrared model has nowhere to attach except `measureTarget`, the custom score in `updateHeatSeeker`, and `Flare::update`. Range dependence, aspect, and countermeasures are all decided there.
- A physical terrain or occlusion model has nowhere to attach except the `y = 0` test and the custom clearance term. Fox 2 never calls the clearance term.
- A radar model has nowhere to attach. MAWS is the only threat sensor, and it is a kinematic closest-approach test.

### Gaps

- No flight log was captured, so the inventory does not include measured miss distances, lock ranges, or time-to-hit. It is a static reading of the source.
- Comments that cite OP 2309, OP 3353, Fleeman, and the open IRCCM article were not re-opened. The formulas above are what the code implements, not a check that the citation matches the manual.
