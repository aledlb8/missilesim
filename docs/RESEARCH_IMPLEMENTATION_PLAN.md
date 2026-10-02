# Research-to-game implementation plan

Date: 2026-09-29. Scope: the 15 untracked research documents and the current game integration points. This is an implementation roadmap, not a claim that the research validates real weapon performance.

## Recommended direction

Build one complete radar-guided engagement before expanding the missile catalog. Preserve the current flight controls and force integration, introduce a common world/sensor/track interface, then deliver search, designation, launch, support, autonomous seeker search, interception or miss, and an understandable debrief.

Use an explicitly fictional reference radar and radar-guided round first. A named missile can use the same architecture later, with sourced identity and geometry and separately labeled simulation assumptions. The research does not supply the complete motor, signature, sensor, or damage data needed to claim an accurate reproduction of most named rounds.

The first playable prototype is a player fighter versus a scripted airborne target, with a radar scope and one complete missile engagement. The first combat alpha adds an armed opponent, player defensive systems, multiple airborne weapons, finite stores, and mission outcomes. Neither milestone requires the entire research backlog.

## What exists and what needs attention

Git has no modified tracked runtime files or staged changes. The pending material is two reports and thirteen supporting notes.

| Area | Verified current behavior | Implementation consequence |
| --- | --- | --- |
| Flight | `Fighter` wraps the existing six-degree-of-freedom flight model. Application advances it separately from `PhysicsEngine`. | Keep flight behavior; expose a common platform snapshot rather than replacing the flight model. |
| Missile physics | Atmosphere, force integration, fuel, drag, thrust, Fox 2 seeker behavior, and a 38-entry Fox 2 catalog exist. | Reuse these foundations behind explicit interfaces; migrate incrementally. |
| Sensors | `MissileFox2.cpp::measureTarget` uses scalar heat and aspect. An unspecified acquisition range permits any positive signal to be visible at any distance. | Replace implicit unlimited detection with an explicit, finite simulation sensor profile. |
| Guidance information | `Missile` stores `Target*`; several Fox 2 paths read live target position/velocity directly, including pre-acquisition pursuit. | Radar support and terminal homing must consume observations or remembered tracks, not live target truth. |
| Radar | No radar detector, scan scheduler, radar track manager, radar signature model, or chaff entity exists under `src`. | This is a new subsystem, not a new missile enum. |
| Warnings | The HUD's RWR circle uses an AI target's kinematic MAWS assessment. Its threat branch is disabled in the player Fighter role. | Build actual emission-derived RWR information and separate approach warnings. |
| Weapon ownership | `Application` owns one `unique_ptr<Missile>`. Rail launch refuses a new launch while that missile is flying. | Introduce weapon instances and a collection of active shots before salvos or enemy fire. |
| Damage | `checkMissileTargetHit` sweeps a missile against AI target spheres at their current positions and immediately deactivates a target. It does not test the player fighter. | Use relative motion, common damage recipients, and distinct impact/detonation/damage events. |
| Match lifecycle | A hit changes a target position to zero to avoid repeated scoring. Several non-hit flight endings enter the same detonation-hold flow. | Replace position sentinels and global shot resets with explicit, once-only outcome events. |
| Timing | Physics substeps are bounded nominally at 0.0025 s; default outer step is 0.01 s. Fighter advances first, targets later; queued flares are instantiated after the outer frame's steps. | Establish a consistent simulation clock, time-stamped snapshots, and deterministic event insertion. |
| Assets | `assets/assets.md` identifies the aircraft and missile meshes as fictional. Rendering loads one generic missile mesh; its transform does not apply per-catalog dimensions. | Add asset identity, dimensions, axis conventions, sockets, and collision metadata. |
| World | Ground/collision use a flat height. Render ground, fighter crash, missile impact, and sensor masking do not share terrain queries. | Introduce one terrain source for all four consumers. |

Useful code anchors: `src/application/Application.h:379`, `src/application/ApplicationLifecycle.cpp:518`, `src/application/ApplicationLifecycle.cpp:539`, `src/application/ApplicationFighter.cpp:301`, `src/application/ApplicationHUD.cpp:677`, `src/physics/PhysicsEngine.cpp:64`, `src/physics/PhysicsEngine.cpp:213`, `src/objects/MissileFox2.cpp:275`, `src/sim/Fox2Catalog.cpp:976`, and `src/rendering/RendererRendering.cpp:195`.

## Research review and decisions

| Research file or group | What to carry into the game | What it does not establish |
| --- | --- | --- |
| `reports/Famous Fox 3 missiles.md` | Distinguish midcourse support, seeker search, terminal tracking, and propulsion families. | A ready-to-run catalog or universal active-handover distance. |
| `reports/Sensor weapon and terrain physics.md` | Separate measurements from truth; integrate terrain and signatures into sensing. | Fully validated numerical models or production-ready parameter tables. |
| Fox 3 `amraam.md` | Variant-specific records and different support capabilities. | Complete propulsion, seeker, or datalink performance data. |
| Fox 3 `european.md` | Separate sustained air-breathing propulsion, conventional solid propulsion, and separated-pulse behavior where supported by evidence. | Interchangeable motor curves or measured launch envelopes. |
| Fox 3 `russian.md` | Keep families and export/domestic identities distinct. | Treating capture claims, seeker brochures, and missile range claims as the same quantity. |
| Fox 3 `asia.md` | Preserve source quality and unresolved identities; distinguish card-only entries from playable profiles. | Filling unknowns from adjacent variants or secondary range claims. |
| Fox 3 `flight-model.md` | Explicit flight phases, dated support packets, separate motor state. | A justified default schedule for every named weapon. |
| Sensor `codebase-inventory.md` | A useful map of existing stand-ins, confirmed against source for the main integration points. | Runtime validation; the note explicitly did not run the game. |
| Sensor `radar-detection-physics.md` | Detection quality and finite observations; a separate sensor clock. | Per-radar settings, complete propagation tables, or every fluctuation model. |
| Sensor `airborne-radar-modes.md` | Scanned search, track maintenance, track aging, and weapon support as different operations. | A complete replica of any specific aircraft radar. |
| Sensor `propagation-terrain-clutter.md` | Shared terrain, path visibility, and environment-dependent sensing. | A terrain dataset or a complete weather model. |
| Sensor `rcs-ew-sim-architecture.md` | Signature providers, measurement records, and simulation-owned sensor logic. | Measured fighter signatures or a reason to implement all electronic warfare now. |
| Sensor `infrared-signatures-seekers.md` | Common aircraft state, aspect/throttle-aware signatures, finite seeker observations. | Usable measured intensity tables for all aircraft and seekers. |
| Sensor `chaff-flare-decoys.md` | Countermeasure entities with lifetimes, motion, and sensor-specific signatures. | Universal success probabilities or cartridge data. |
| Sensor `missile-guidance-propulsion.md` | Separate seeker, flight response, propulsion, fuze, damage, and outcome state. | Proof that replacing every existing guidance path with one formula improves the game. |

Resolve these specification issues before translating prose into code:

1. **Separate seeker activation from acquisition.** Some notes describe turn-on in terms of an existing contact. An inactive seeker cannot supply that contact. Activation/search uses only the permitted estimated state and the selected game profile; acquisition requires a subsequent sensor observation.
2. **Keep truth out of public track records.** One proposed contact record contains both true and measured geometry. Put truth and diagnostic associations in a private simulation/debug record; HUD, AI, and weapon support receive only the permitted estimate.
3. **Track coasting is not a radar operating mode.** A scanning radar can hold fresh and stale tracks simultaneously. Keep operating mode and per-track lifecycle independent.
4. **A lost seeker lock does not disable all damage.** Guidance, proximity sensing, physical collision, and damage are separate. Preserve collision checks and allow an independently triggered fuze; do not require guidance lock for every valid hit.
5. **Closest approach is a diagnostic event, not the entire damage model.** Record swept minimum separation, impacts, and detonations independently. Account for both bodies moving. Do not force every burst to happen at closest approach or end every shot at the first opening interval.
6. **Avoid wholesale guidance replacement.** The current custom and Fox 2 paths use different conventions and behaviors. Keep named versions during migration and compare game regression scenarios before changing defaults.
7. **Resolve conflicting research defaults.** The radar notes differ on threshold-versus-probability detection and which propagation terms are available. Choose one versioned game model; avoid stacking two processing gains or applying the same loss twice. Do not infer a named seeker's band from a broad application table.
8. **Treat absent data explicitly.** Unknown research fields remain unknown. A playable profile can supply declared fictional parameters. These are different records, so gameplay tuning cannot overwrite research evidence.

Selected external spot-checks support the broad architecture: NAVAIR describes AMRAAM inertial flight with datalink updates and active terminal guidance; MBDA describes Meteor's distinct propulsion and networked guidance; MathWorks distinguishes timed radar observations and coasted tracks. These checks are not an audit of every citation or every equation in the research.

- [NAVAIR AMRAAM](https://www.navair.navy.mil/product/AMRAAM)
- [MBDA Meteor](https://www.mbda-systems.com/products/air-dominance/meteor)
- [MathWorks radar observation timing](https://www.mathworks.com/help/fusion/ug/simulate-radar-detections.html)
- [MathWorks scanning radar and coasted tracks](https://uk.mathworks.com/help/radar/ug/simulating-a-scanning-radar.html)

## Architecture and gameplay contract

The main dependency flow is:

```text
World snapshot + environment + signature profiles
                    |
                 sensors
                    |
          observations -> track manager
                    |
        HUD / AI / designation / support packets
                    |
          weapon state and flight response
                    |
         next world state + outcome events
```

Suggested ownership, using ordinary C++ composition rather than a full entity-system rewrite:

| Component | Responsibility | Proposed location |
| --- | --- | --- |
| Simulation clock and scenario seed | Scheduling, stable random streams, event ordering | `src/sim/` |
| Platform snapshot adapter | Stable ID, pose, velocity, engine state, team, damage/capability state | `src/sim/`, backed by existing `Fighter` and `Target` |
| Terrain/environment queries | Height, normal, visibility, wind; later propagation extensions | `src/sim/` or `src/world/` |
| Signature providers | Deliberate radar and infrared game profiles | `src/sim/signatures/` |
| Sensor scheduler and observations | Scan timing and bounded, noisy sensor information | `src/sim/sensors/` |
| Track manager | Tentative, confirmed, coasted, lost; estimate, timestamp, uncertainty | `src/sim/tracking/` |
| Stores and weapon instances | Stations, ammunition, owner, support association, active shots | `src/sim/weapons/` |
| Outcome resolver | Collision, detonation, damage, destruction, scoring events | `src/sim/` with physics queries |
| Presentation adapters | Render state, audio events, player-known HUD information | Existing application/rendering/audio files |

Sensors may inspect simulation truth to synthesize measurements. A tracker receives measurements only; a support packet receives a track estimate only. Debug overlays can expose truth, but must be explicitly enabled. Track IDs must not secretly reveal target identity to the player or AI.

Keep the existing physics rates initially. Schedule sensing and support on simulation time, independent of drawing. All observations identify their measurement time; all messages identify both state time and delivery time. Publish a coherent world snapshot at each sensor boundary. Insert new countermeasures at a documented simulation event boundary, not after an arbitrary number of rendered-frame steps. Define a backlog policy and test it rather than assuming time scaling is frame independent.

Required gameplay rules:

- Search contact, confirmed track, selected target, weapon-ready state, seeker emission, and terminal acquisition are distinct states.
- Breaking radar support stops fresh information. It does not automatically destroy a missile or refresh its track from truth.
- A missile can search unsuccessfully after activating. An active transmitter is not proof of target acquisition.
- The player's displayed missile status must respect the selected profile's communication capabilities. Show predicted status when confirmation is unavailable.
- RWR observes emissions; MAWS models approach observations if the aircraft has that capability. An infrared launch alone is not an RWR detection.
- AI uses its own perception, warning history, and reaction delay. It has no privileged access to opposing missile targeting state.
- Flare and chaff releases spend finite stores and affect the appropriate sensor representation. Neither is a universal missile-cancel button.
- Weapon death, target destruction, and mission completion are different events. One expired shot must not reset another airborne shot.
- Camera position, FOV, resolution, HUD visibility, and graphics settings cannot change sensor results.

## Delivery phases

### Phase 0 — Reproducible baseline and data policy

**Work:** Save seeded scenarios; add a headless engagement harness using the runtime components; record launch, observation, track, warning, release, impact, and outcome events. Register the existing Fox 2 checker with CTest, while naming its checks as sanity/regression checks. Preserve current scenario behavior as a migration reference.

Separate research records from resolved game profiles. Use SI units internally, schema versions, source/assumption tags, and required-field validation. Missing data must either block that profile with a readable error or select an explicitly named fallback. Never silently substitute a neighboring real missile.

**Acceptance:** Same seed, inputs, and build reproduce the same simulation event trace. Malformed profiles fail clearly. Current flight-handling tests pass. Performance and existing behavioral limitations are recorded before refactoring.

### Phase 1 — World timing, entities, weapon ownership, outcomes

**Depends on:** Phase 0.

**Work:** Add stable entity IDs and platform adapters for both `Fighter` and `Target`. Introduce a stores/station model and active-weapon collection behind the current launch UI. Give each shot an owner, team, target/track association, state, and outcome. Replace raw target lifetime assumptions and zero-position scoring sentinels. Integrate damage recipients for both player and AI. Make countermeasure birth and entity removal deterministic.

Keep the current one-shot sandbox as a scenario preset, implemented on the new lifecycle. Retain existing Fox 2 physics through an adapter.

**Acceptance:** Two airborne shots coexist; one shot ending does not reset the other. An entity removed mid-engagement leaves no dangling references. Player and AI can receive one damage event each. Scores and destruction fire once. Crossing-motion collisions remain stable under substep changes.

### Phase 2 — Shared terrain, signatures, sensor interfaces

**Depends on:** Phase 1.

**Work:** Start with flat and procedural-ridge terrain providers and render the same height data used by physics and visibility. Define altitude datum and axis conventions. Add synthetic radar and infrared signature profiles using the common aircraft engine/attitude state. Introduce observation and track types plus a sensor scheduler. Preserve legacy Fox 2 sensing until its replacement scenarios pass.

Use a simple, explicitly labeled visibility approximation initially. Keep radar and infrared propagation strategies separate even though they share geometry. Advanced diffraction, reflections, and weather can replace providers later.

**Acceptance:** A visible ridge also blocks the relevant sensor path and collides with aircraft/missiles at the same surface. Body orientation drives aspect, including when velocity differs from the nose. No observation occurs between scheduled sensor updates. Sensors do not change when the camera moves.

### Phase 3 — Playable radar and target management

**Depends on:** Phase 2.

**Work:** Implement one fictional scanning radar with search and single-target track first; add track-while-scan after basic track aging works. Use an explicit detection-quality model, finite scan coverage, observation uncertainty, track association, and loss/reacquisition. Keep scan scheduling within one radar's available time budget.

Build an ImGui radar scope with scan volume, range scale, contact age, track state, designation, and a clear reason when a weapon cannot launch. In the sensor-based scenario, HUD labels use player-known tracks or explicit visual observations. Keep omniscient markers in sandbox/debug mode only.

**Acceptance:** A target outside the scanned volume is not freshly observed; wider searches revisit more slowly. Terrain masking produces aging then loss rather than instant perfect position updates. A stale track cannot silently become a fresh launch-quality track. Search/track transitions are explainable on the display.

### Phase 4 — First Fox 3-style engagement

**Depends on:** Phase 3 and the weapon lifecycle from Phase 1.

**Work:** Add one fictional radar-guided profile. Separate launch clearance, supported midcourse, autonomous search, terminal tracking, memory/loss, and termination. Keep motor state independent. Support packets contain only the shooter's permitted track estimate. Reuse the sensor framework for the missile's seeker. Keep flight response bounded and compare it against regression scenarios rather than tuning to a brochure range.

Implement generic impact, arming, proximity-trigger, and game damage rules as separate policies. Record why a shot ended. Build the HUD support indication and debrief alongside the weapon.

**Acceptance:** Complete the player-versus-scripted-target loop from search to debrief. A support interruption preserves only the last estimate; successful autonomous acquisition can allow the shot to continue; failed search can produce a miss. A lost guidance track does not suppress a physical collision. All outcomes have a visible reason.

**Milestone:** First playable radar engagement. Do not add ten named missiles before this is stable.

### Phase 5 — Defensive gameplay and combat alpha

**Depends on:** Phase 4.

**Work:** Give the opponent the same sensors, stores, weapon interface, and damage rules. Add independent RWR and optional MAWS capabilities, finite chaff/flare dispensers, and countermeasure observation behavior. Migrate Fox 2 signatures behind a versioned profile without changing every round at once. Introduce AI perception history and reaction delay, manual/automatic defensive controls, and clear mission win/loss/restart rules.

Use one opponent and a bounded number of shots for the initial combat scenario. Add readable audio cues for different warnings without exposing information the receiver cannot know.

**Acceptance:** Enemy shots can threaten and damage the player. An IR shot alone does not generate an RF warning. Countermeasures sometimes fail for understandable scene reasons. Both sides spend ammunition. No AI defense starts before the permitted warning. Mission completion waits on the chosen scenario rules rather than a global missile reset.

**Milestone:** Playable combat alpha with attack, support, defense, and debrief.

### Phase 6 — Model fidelity, assets, and catalog expansion

**Depends on:** A stable Phase 4 prototype; full combat validation uses Phase 5.

**Work:** Expand one representative profile at a time. Keep radar-guided solid propulsion as the first archetype; introduce other propulsion/support families only when their distinct behavior has a tested game model. Semi-active support is a separate future capability; do not force Phoenix-style research into the first autonomous profile. Keep unresolved entries as research/catalog cards until they have explicit playable assumptions.

Build an asset manifest with model ID, meter scale, forward/up axes, hardpoints, exhaust sockets, collision proxy, and attribution. Add distinct silhouettes and appropriate fin/body proportions. Match the fighter's visual identity to its flight profile or clearly keep it fictional. Display carried stores at the same sockets used for release, and tie plume/audio state to the simulated engine state.

LOD and camera-relative rendering should follow profiling at the intended map size. Evaluate float precision, depth precision, terrain streaming, and world-bound assumptions before expanding to large theaters. The current fixed Fox 2 world bounds are not a missile performance limit.

**Acceptance:** Mesh bounds, collision proxies, stations, and launch points agree. Named cards expose assumptions. Changing visual quality does not affect sensing or flight. Each new profile passes the common engagement suite and a manual readability pass.

## What to defer

- Detailed electronic warfare, deceptive false-contact techniques, and towed decoys.
- Advanced radar ambiguities and specialized search/illumination modes beyond the initial gameplay need.
- Full spectral atmosphere tables, detailed rain/clutter models, and fine multipath effects.
- High-detail six-degree-of-freedom missile models or reconstructed proprietary control systems.
- A full national/variant missile roster.
- Dynamic launch-zone estimation until the runtime model is stable. Later estimates must use the same versioned game model, a stated target-behavior assumption, and visible uncertainty; they are not guarantees.

Interfaces should permit these additions. None should block the first complete engagement.

## Validation and release gates

| Category | Minimum checks |
| --- | --- |
| Timing | Seeded replay, frame-rate independence, pause/resume, time scaling, event birth order, insertion-order independence. |
| Information | No fresh track without a new observation; no midcourse access to target truth; debug data inaccessible to gameplay consumers. |
| Radar | Scan coverage, contact confirmation, aging, association, masking, reacquisition, and no double-counted losses/quality gains. |
| Weapons | Multiple instances, valid stores consumption, owner separation, stale support, failed search, terminal loss, expiry, safe removal. |
| Damage | Moving-body crossing, armed/unarmed distinction, impact independent of guidance lock, once-only damage/score, player damage. |
| Countermeasures | Finite inventory, scheduled births, lifetime cleanup, correct sensor family, no guaranteed success flag. |
| Assets | Bounds and axes, sockets, rail separation, terrain alignment, consistent engine effects, portable-package contents. |
| Gameplay | Player can understand detection, launch restrictions, support loss, warnings, defensive actions, and the final outcome. |
| Performance | Measure simulation, sensors, terrain queries, rendering, and effects separately on the user's machine. Agree an entity budget from the baseline before adding complex propagation. |

Use analytical checks only within the assumptions of the selected game model; use seeded multi-run tests for probabilistic behavior. Do not make a single random hit/miss the release gate. Automated checks and manual flight sessions are both required for gameplay changes.

Observed baseline on 2026-09-29, using the existing local binaries (no clean rebuild or interactive playtest in this review):

- `ctest --test-dir build/ninja-debug --output-on-failure`: the one registered `flight_handling` test passed.
- `build/ninja-debug/bin/fox2_kinematics.exe`: exited successfully; checked 38 catalog entries. Its reported reference-case range ratios include approximately 4.69 and 1.54 for the selected drag-band runs. The tool labels these residuals; success means its implemented sanity checks passed, not that performance matched the reference cases.
- No radar, Fox 3, terrain-visibility, or complete combat validation currently exists in the registered suite.

## First implementation batch

Keep changes reviewable in this order; the list does not imply these commits already exist:

1. `test(sim): add seeded engagement harness and baseline traces` — Phase 0, including existing-check registration and explicit limits of those checks.
2. `refactor(sim): introduce platform snapshots and deterministic events` — stable identities and a common simulation timeline, preserving flight controls.
3. `refactor(weapons): separate stores, active shots, and outcomes` — multiple weapons, safe references, common damage recipients, once-only scoring.
4. `feat(world): share terrain queries across physics and sensing` — flat/ridge fixtures and visual agreement.
5. `feat(sensors): add scheduled observations and track lifecycle` — fictional signature profiles and strict truth isolation.
6. `feat(radar): add search, designation, track display, and loss feedback` — first usable radar loop.
7. `feat(weapons): add reference radar-guided engagement` — support packets, seeker search, complete outcome/debrief.
8. `feat(combat): add opponent fire and player defensive systems` — warning parity, expendables, mission lifecycle.

Start with items 1 and 2. They provide the evidence and interfaces needed to make later radar and missile changes reviewable without destabilizing the current flying experience.
