# Infrared air-to-air missile flight model (point-mass)

Scope is an unclassified real-time point-mass model for a civilian sandbox that already has gravity, Mach drag, induced drag, altitude-varying thrust, and true proportional navigation. This note is the algorithms and public numbers that let famous Fox 2s differ in kind. It is not a hardware design. Figures are tagged [published], [derived], or [unknown]. No seeker-processing constants were invented. Sources were checked as of 2026-09-28.

Reference area for every missile coefficient below is body cross-section, `S_ref = pi * d * d / 4`, unless a fighter coefficient says wing area.

## Rail launch from a fighter

### Takeaway
An air-to-air Sidewinder leaves a rail already at fighter speed, is steered only after a short inhibit, and then flies proportional navigation. It does not use the surface missile’s vertical ejection, pitch-over, or terrain avoidance. MICA IR is publicly a different kind of shot: inertial midcourse, then seeker. Whether the R-27T has that midcourse is disputed in open sources.

### Cited Findings
- The AIM-9B is a rail-launched, passive-infrared, proportional-navigation missile. On the LAU-7 the missile hangs on three hangers; a detent stops it moving until firing. — [NAVWEPS OP 2309 (3rd rev.), AIM-9B](https://archive.org/details/OP23093rdAIM9B); [NAVWEPS OP 3352, AIM-9D](https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html)
- AIM-9C/D pilot handbook: about 0.8 s can pass after the trigger before the missile leaves the launcher. For the AIM-9D the same handbook says the trigger does not have to be held until the round leaves. That 0.8 s is the firing-sequence delay, not the in-flight guidance inhibit. — [NAVWEPS OP 3353](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%203353%20AIM-9C%20&D%20PIlot%27s%20Handbook_djvu.txt)
- AIM-9B: a disabling circuit blocks any turn command to the steering servos for the first 0.5 s of flight, “to prevent overcontrol of the missile at relatively slow speeds.” The gyro is then free to precess. Maximum guided flight is about 20 s, the servo-grain burn. If the fuze never acts, a mechanical timer self-destructs the warhead about 24 s after launch. The effective control-system time constant is 0.25 s maximum. — [NAVWEPS OP 2309, full text](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%202309%20(3rd)%20AIM-9B_djvu.txt)
- AIM-9B performance under the manual’s “typical conditions”: speed increment about Mach 1.5 above the firing aircraft, depending on altitude. That is the burnout increment, not a rail-exit speed. Motor Mk 15 or Mk 17: nominal impulse 8440 lb·s, burn 2.2 s at 70 °F [published]. — [NAVWEPS OP 2309](https://archive.org/details/OP23093rdAIM9B)
- AIM-9B characteristics table, cells that survived OCR: weight 160 lb; wing span with rollerons 22 in. The length, diameter, and fin-span cells on that same line are not readable. Body diameter is stated as 5 in for the guidance section, the motor, and the influence fuze. The motor is about 75 in long and about 80 lb, which is the motor assembly, not a propellant mass. Guidance section: 20 in, 36 lb. Warhead Mk 8: 25 lb. — [NAVWEPS OP 2309 full text](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%202309%20(3rd)%20AIM-9B_djvu.txt)
- The AIM-9B’s Aero 3A launcher is 84.2 in long (2.14 m), 4.9 in high, 2.0 in wide, and 50 lb. That is the launcher envelope, not the distance the missile slides after the detent releases. — [NAVWEPS OP 2309 full text](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%202309%20(3rd)%20AIM-9B_djvu.txt)
- AIM-9D/C: the pilot handbook says the Mk 36 gives an average of 3500 lbf for 5 s and a maximum speed of Mach 2.5 over the firing aircraft [published]. The description manual’s motor table, same motor, lists normal-operation thrust 2645 lb at 70 °F at sea level, burning time 5 s at 70 °F, and a minimum total impulse of 13,968 lb·s at 70 °F. Those two thrust figures conflict: 2645 lbf times 5 s is 13,225 lb·s, which is below the stated minimum impulse, so 2645 lbf is not that table’s average thrust. The missile “flies a proportional-navigation intercept course.” No loft is described. The motor section weighs about 99 lb. The family figure in the pilot handbook lists the AIM-9D at 195 lb and the AIM-9C at 210 lb; the early-round weights on that figure are 155 lb and 160 lb, matching the AIM-9B table’s 160 lb. — [NAVWEPS OP 3353](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%203353%20AIM-9C%20&D%20PIlot%27s%20Handbook_djvu.txt); [NAVWEPS OP 3352 text](https://archive.org/download/OP3352AIM9D/OP%203352%20AIM-9D%201_text.pdf); [OP 3352 hocr](https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html)
- No opened source states a numerical rail-exit speed in metres per second. Fleeman’s integration notes treat weapons on fighters as rail-launched stores and separate that problem from safe separation of the warhead. — [Fleeman, Tactical Missile Design course slides, 2008-02-24, via KFUPM archive](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- MICA (RF and IR), MBDA brochure: strap-down inertial reference, data link, lock-on before launch and lock-on after launch, imaging IR or active RF terminal seeker, tail control plus thrust-vector control. In short-range combat, LOAL plus acquisition is what MBDA credits for a 360° launch envelope, including a rear-sector threat. Midcourse is inertial with in-flight target updates, then in-flight seeker lock, then homing. It is not proportional navigation from the rail. — [MBDA MICA brochure (media/20541)](https://www.mbda-systems.com/media/20541/download)
- French-language technical description of MICA: inertial guidance for the flight, seeker takes over in the terminal phase; long-range modes are lock-on after launch, with or without data-link updates. — [MICA, French Wikipedia, summarizing the guidance modes](https://fr.wikipedia.org/wiki/MICA)
- R-27 family, one defense write-up: “in control system of all missiles, in addition to seeker, an inertial navigation system with radio-correction is included,” and the IR versions are named in that “all missiles” sentence. A Russian reference site says the initial trajectory uses inertial guidance to a “mathematical” target with radio correction, and lists R-27T separately as the thermal-seeker round with target-designation angles of ±55°. — [Defence Express, R-27](https://defence-ua.com/index.php/en/publications/defense-express-publications/2569-r-27-air-to-air-guided-missile); [Missilery.info, R-27](https://en.missilery.info/missile/p27)
- The same question is contradicted by a specialist forum post: only the radar-homing R-27s use inertial midcourse; the IR versions are lock-on before launch only, and basic R-27s do not loft (the R-27EM did). That post is not a manufacturer document. — [Key Aero forum, AA-10 thread](https://www.key.aero/forum/modern-military-aviation/missiles-and-munitions/28704-questions-about-the-aa-10-alamo)
- Wikipedia’s R-27T range table is a kinematic envelope, not a seeker-lock table: same-altitude effective range about 2–33 km head-on and 0–5.5 km tail-on, altitudes about 20–25,000 m [published as an encyclopedia compilation]. Head-on longer than tail-on is what closing speed does to flight range; it does not by itself prove a head-on lock at 33 km. — [R-27, English Wikipedia](https://en.wikipedia.org/wiki/R-27_(air-to-air_missile))

### Inferences
- Keep the vertical-ejection / pitch-over / terrain-avoidance sequence only on the custom surface round. Air-to-air launch is a rail constraint, then free flight.
- Initial velocity is the shooter’s velocity. The missile is already at that speed on the rail. Any extra is the integral of net acceleration along the rail after the detent releases, until the last hanger clears. No public rail-exit delta was found, so do not add a magic “rail speed.” The Aero 3A launcher is 84.2 in overall [published], so the slide is at most about 2.1 m and is shorter than that once the hangers and the nose are accounted for. Integrate the point-mass along the rail axis for that unknown stroke rather than adding a fixed exit speed. A 0.8 s trigger delay, if modeled, happens before motion. On an AIM-9B, releasing the trigger during that delay can abort the launch; the AIM-9D handbook says the trigger need not be held.
- Guidance law for a classic Sidewinder (AIM-9B through AIM-9M, and an AIM-9X that already has a lock): zero commanded lateral acceleration for `t < t_inhibit`, then true proportional navigation. Use `t_inhibit = 0.5` s for an AIM-9B-class round [published]. The manual’s reason is low-speed overcontrol, not warhead-to-aircraft separation. Fuze arming is a separate distance gate (next section). Lag the achieved lateral acceleration with a first-order time constant of 0.25 s on an AIM-9B [published maximum]. Stop homing commands at 20 s even if the warhead timer runs to 24 s. A straight tail-chase range check can keep integrating kinematics until 24 s or until closing speed is gone, because that check has no steering.
- Do not loft a classic short-range infrared missile. Nothing in the Sidewinder manuals describes a climb-then-dive. Proportional navigation starts when the inhibit ends and the seeker is tracking.
- MICA IR is a different guidance kind [published]: while the seeker is not yet locked, fly an inertial midcourse toward the cued intercept point (data link may update that point). When the target enters the acquisition basket and locks, switch to proportional navigation. A range-maximizing midcourse may be shaped; that is not the same law as a Sidewinder.
- R-27T midcourse: do not treat inertial guidance as settled. One published defense summary includes it for all R-27 seekers; a contradictory open commentary says the IR rounds are lock-before-launch only. Until a manual is opened, model R-27T as lock-before-launch proportional navigation, and keep inertial midcourse as an optional flag marked disputed.

### Gaps
- The missile’s slide along the rail, the detent force, and the rail-exit speed were not found. The Aero 3A overall length of 84.2 in is only an upper bound on that slide.
- A published in-flight guidance-inhibit time was found only for the AIM-9B (0.5 s). AIM-9L/M/X, IRIS-T, and R-73 inhibit times are [unknown]. The AIM-9B’s 20 s guided-flight limit is the servo grain, not a modern battery limit.
- No opened source says a short-range infrared missile lofts. Absence is not a proof that AIM-9X Block II never shapes a LOAL trajectory; that profile was not published in the sources opened here.

## Thrust vector control

### Takeaway
Lateral acceleration from a deflected jet is `T * sin(delta) / mass`, added to aerodynamic lift and still capped by structural g. It exists only while the motor is burning. Fleeman’s open sizing slides give a jet-vane deflection of about ±10° and state that thrust-vector control is what provides maneuverability at low dynamic pressure. AIM-9X, IRIS-T, and MICA are jet-vane missiles; the R-73 is a jet-tab / paddle missile.

### Cited Findings
- AIM-9X airframe, 1998 Navy NTSP: four fixed forward titanium wings; four aft titanium fins; a control actuation system “that uses four jet vanes to direct the flow of the rocket motor exhaust.” The motor is the AIM-9M motor modified to carry that actuation system. An electronic safe-and-arm device “arm[s] the warhead after launch” (no arming time is stated). — [NAVAIR NTSP AIM-9X, N88-NTSP-A-50-9601, May 1998, hosted at GlobalSecurity](https://www.globalsecurity.org/military/library/policy/navy/ntsp/AIM-9X.pdf)
- NAVAIR product page: AIM-9X Block II uses “its datalink, thrust vectoring maneuverability, and advanced imaging infrared seeker to hit targets behind the launching fighter.” Length 9.9 ft (3.02 m), launch weight 186 lb (84.37 kg), diameter 5 in (0.13 m), wingspan 17.6 in (0.45 m). Range and speed are classified. Power plant named as ATK MK-139. — [NAVAIR, AIM-9X Sidewinder](https://www.navair.navy.mil/product/AIM-9X-Sidewinder)
- Fleeman course slide “TVC and Reaction Jet Flight Control”: “TVC and reaction jet flight control provide high maneuverability at low dynamic pressure.” “TVC usually has lower time constant and miss distance than aero control.” Jet vanes “provide roll control and share actuators with aero control, but have reduced ISP.” The jet-vane callout on that slide is ±10°. Other devices on the same figure are listed with ±7°, ±12°, ±7°, ±15°, and ±20°; PDF text order pairs those, in listed order, with liquid injection, hot-gas injection, axial plate, jet tab, and movable nozzle. Treat ±10° as the unambiguous jet-vane number [published, typical sizing value, not a measured AIM-9X vane angle]. — [Fleeman slides, p. 64](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- Same slides, next page, as examples of “jet vane + aero control”: MICA, AIM-9X, IRIS-T, A-Darter (also Sea Sparrow RIM-7, Sea Wolf, Javelin). “Jet tab + aero control”: Archer AA-11, i.e. the R-73. — [Fleeman slides, p. 65](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- IRIS-T: solid motor with thrust-vector control; “turns of 60 g at a rate of 60°/s via thrust vectoring and LOAL” [published as Wikipedia’s reading of Diehl]. Mass 87.4 kg, length 2.94 m, diameter 127 mm, span 447 mm. — [IRIS-T, English Wikipedia](https://en.wikipedia.org/wiki/IRIS-T)
- IRIS-T autopilot paper: combined aerodynamic and thrust-vector control; lateral and roll controllers “scheduled as a function of dynamic pressure” across the envelope. That is the open “control depends on dynamic pressure” statement for this missile. No vane angle is given in the abstract. — [Buschek, Control Engineering Practice, 2003](https://www.sciencedirect.com/science/article/abs/pii/S0967066102000631)
- R-73: Wikipedia gives minimum range about 300 m, aerodynamic range nearly 30 km at altitude, and seeker detection “up to 40° off the missile’s centerline” for the baseline. AusAirpower, citing a Vympel data card, lists launch weight 103 kg, length 2.9 m, diameter 0.17 m, off-boresight engagement ±45°, seeker gimbal ±75°, minimum range 0.3 km on a receding target, and “aerodynamic and TVC controls,” with paddles in the exhaust rather than vanes. GlobalSecurity lists “self overload: up to 40 G,” which is an unsourced aggregator figure. These g and angle figures conflict with each other; none is a primary flight-test report opened here. — [R-73, English Wikipedia](https://en.wikipedia.org/wiki/AA-11_Archer); [AusAirpower, R-73E card](https://www.ausairpower.net/APA-NOTAM-200408-1.html); [GlobalSecurity, AA-11 specs](https://www.globalsecurity.org/military/world/russia/aa-11-specs.htm)
- A-Darter, a different missile, is the clearest opened sentence that TVC g lasts only for the burn: thrust-vector control “allows for turning at up to 100 g … for a period of 8 seconds while the motor burns, after which the missile can still retain 50 g of manoeuvrability.” Use this as the published pattern (TVC ends at burnout; aero authority remains, at a lower g), not as an AIM-9X or IRIS-T number. — [A-Darter, English Wikipedia](https://en.wikipedia.org/wiki/A-Darter)
- PL-10, encyclopedia compilation: thrust-vectoring solid motor, imaging infrared seeker, seeker “may track targets +/-90 degree off boresight,” lock-on after launch with a datalink, “turn capability of over 60 Gs.” The underlying brochure was not opened. — [PL-10, English Wikipedia](https://en.wikipedia.org/wiki/PL-10)
- Python 5 manufacturer brochure (opened via search extract): full-sphere envelope from agility, LOAL, inertial navigation, and a dual-band imaging seeker. The Rafael bullets that were visible do not state thrust-vector control or a deflection angle. Secondary sites that assert Python 5 thrust vectoring were not treated as a manufacturer figure. — [Rafael Python 5 brochure PDF](https://www.rafael.co.il/wp-content/uploads/2024/09/python-Air-to-Air-and-Air-defense-Missile.pdf); [Rafael USA, Python-5](https://www.rafael-usa.com/programs/python-5/)
- A student slide deck claiming AIM-9X jet vanes of ±20° and a 42 kN vane force was not used. It is not a primary source.

### Inferences
- Implement, in the maneuver plane, only while `thrust > 0`:

```cpp
double delta = clamp(delta_cmd, -delta_max, delta_max); // radians
double a_tvc = (thrust * std::sin(delta)) / mass;       // m/s^2, normal to the body axis
double thrust_axial = thrust * std::cos(delta);         // jet vane also costs axial thrust
```

- Add `a_tvc` to the aerodynamic lateral acceleration, then cap the sum:

```cpp
double a_aero = std::min(q * CN_max * S_ref / mass, n_struct * g);
double a_lat  = std::min(n_struct * g, a_aero + a_tvc);
```

- At large angle of attack the jet is along the body, not along the velocity. The simple sum above is what was asked for a point-mass model. If the body axis is carried, the component normal to velocity is `thrust * sin(delta + alpha) / mass` in a planar pitch. Either form goes to zero at burnout because `thrust` is zero: set `delta = 0` when the motor is out, and leave only `a_aero`.
- `delta_max` for a jet-vane Fox 2 (AIM-9X, IRIS-T, MICA): 10° = 0.1745 rad, as Fleeman’s typical jet vane [published sizing value, not a Raytheon measurement]. `sin(10°) = 0.1736`, so TVC alone is about 0.17 * T/m. R-73, if modeled as Fleeman’s jet tab, uses the slide’s jet-tab angle only if that pairing is accepted; the explicit number to prefer is “jet tab, not jet vane,” with deflection [unknown] for the real missile.
- This is how a Fox 2 differs in kind from a thrust-slider change: below the dynamic pressure where aero can trim, a TVC missile can still turn, and a canard-only Sidewinder cannot. After burnout they converge to whatever aero `CN_max` each airframe has. IRIS-T’s 60 g at 60°/s is a TVC-era number and should not be applied after burnout.

### Gaps
- No primary AIM-9X, IRIS-T, R-73, Python 5, or PL-10 vane-angle measurement was found. ±10° is Fleeman’s typical jet vane.
- Python 5 thrust vectoring is [unknown] in the manufacturer text that was opened.
- Post-burnout g for AIM-9X and IRIS-T is [unknown]. The A-Darter 100 g for 8 s, then 50 g, is a different weapon.
- IRIS-T “60 g” is Wikipedia citing Diehl, not a Diehl brochure page opened here. The German-language claim of “well over 100 g” is speculation and was not used.

## Zero-lift drag for a slender canard missile

### Takeaway
Use Fleeman’s body build-up, referenced to body cross-section: skin friction plus base drag plus supersonic wave drag. A Sidewinder-class fineness ratio (about 20–24) sits on his “high fineness, low drag” example, whose comparison `CD0` is 0.2, while the same formulas at Mach 2 produce a body `CD0` near 0.4 before fins. Calibrate with one scale factor on that Mach curve so a tail chase hits the AIM-9B manual’s ranges, and reject the factor if it leaves that band.

### Cited Findings
- Fleeman: tactical-missile body fineness is typically `5 < l/d < 25`. Air-to-air missiles are the high end; his example is AIM-120 at `l/d = 20.5`. Small diameter lowers drag because `D = CD * q * S_ref` and `S_ref` is body cross-section (`D = 0.785 * CD * q * d^2` in his units). — [Fleeman slides, pp. 23–24](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- His L/D comparison, not a Mach sweep, uses three constant `CD0` cases [published as sizing examples]: high-drag low-fineness body `l/d = 10`, `CD0 = 0.5`; low-drag nose `l/d = 10`, `CD0 = 0.2`; high-fineness low-drag `l/d = 20`, `CD0 = 0.2`. — [Fleeman slides, p. 35](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- Body zero-lift build-up, his equations, `q` in psf and `l` in feet [published]:

```text
CD0_body = CD0_friction + CD0_base + CD0_wave

CD0_friction = 0.053 * (l/d) * pow(M / (q_psf * l_ft), 0.2)
  // Jerger; turbulent boundary layer

// M > 1:
CD0_base_coast   = 0.25 / M
CD0_base_powered = (1.0 - Ae/S_ref) * (0.25 / M)
// M < 1:
CD0_base_coast   = 0.12 + 0.13 * M * M
CD0_base_powered = (1.0 - Ae/S_ref) * (0.12 + 0.13 * M * M)

// M > 1 only; atan in radians; Bonney:
CD0_wave = (1.59 + 1.83 / (M*M)) * pow(atan(0.5 / (l_nose/d)), 1.69)
```

Worked rocket baseline on the same slide: `l/d = 18`, `lN/d = 2.4`, Mach 2, 20,000 ft, `CD0_friction = 0.14`, coast base `0.13`, powered base `0.10`, wave `0.14`, so coast `CD0 = 0.41` and powered `CD0 = 0.38` [published example, not a Sidewinder]. — [Fleeman slides, p. 30](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- He also records the method missing a wind-tunnel baseline: at Mach 2, handbook coast `CD0` about 0.53–0.57 versus tunnel `CD0 = 1.05`. “Wind tunnel data / baseline missile data correction required.” A factor of about two is a documented method error on that airframe, not a Sidewinder measurement. — [Fleeman slides, p. 346](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- Fins add their own zero-lift term. His total is `CD0_total ≈ CD0_body + CD0_wing + CD0_tail`. A low-Reynolds example (not a missile) uses `CD0_tail_friction = n_tails * 0.0133 * pow(M/(q*c_mac), 0.2) * (2*S_tail/S_ref)`. — [Fleeman slides, pp. 74 and 358](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- Hoerner, as cited by a 1971 NSW C-style drag note (AD0729009): zero-lift drag of slender body-fin shapes generally peaks near Mach 1.00–1.02, then falls toward the subsonic value plus a supersonic increment. The scan of that report is OCR-damaged; the Mach “1.00 to 1.02” reading matches the prose, the symbols around the equations do not. — [AD0729009](https://apps.dtic.mil/sti/tr/pdf/AD0729009.pdf), citing Hoerner, *Fluid-Dynamic Drag*
- Missile DATCOM’s public manual decomposes body axial force the same way (skin friction, pressure/wave, base) and non-dimensionalizes by free-stream dynamic pressure and a reference area that is the body cross-section for these rounds. It was not re-run here; Fleeman’s closed form is the real-time method. — [Missile DATCOM, Vol. 1, archive text](https://archive.org/stream/DTIC_ADA211086/DTIC_ADA211086_djvu.txt)
- AIM-9X geometry for fineness [published]: length 3.02 m, diameter 0.13 m (5 in). `l/d = 3.02/0.127 = 23.8` [derived]. `S_ref = pi * (0.0635)^2 = 0.01267 m²` [derived]. Older Sidewinders are a few tenths of a metre shorter and stay in the same high-fineness class. — [NAVAIR AIM-9X](https://www.navair.navy.mil/product/AIM-9X-Sidewinder)
- Calibration ranges, AIM-9B, “typical conditions” [published]: 28,000 ft range at 50,000 ft altitude, launch Mach 1.2, target Mach 0.9; 6,000 ft range at sea level, launch Mach 1.2, target Mach 0.9. Impulse 8440 lb·s in 2.2 s, so average thrust `8440/2.2 = 3836 lbf` if thrust is constant [derived from the two published motor numbers]. Launch mass 160 lb [published table]. Guided flight about 20 s; self-destruct about 24 s. — [NAVWEPS OP 2309](https://archive.org/details/OP23093rdAIM9B); [OP 2309 full text](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%202309%20(3rd)%20AIM-9B_djvu.txt)
- AIM-9X range is classified. Do not calibrate an AIM-9X to a forum range. — [NAVAIR AIM-9X](https://www.navair.navy.mil/product/AIM-9X-Sidewinder)

### Inferences
- Band to stay inside, for body-referenced `CD0` of a Sidewinder-class fineness ratio: about 0.2 to 0.5 away from the transonic peak. The 0.2 is Fleeman’s high-fineness comparison case. The Mach-2 body sum on his own formulas is about 0.4 before fins. A transonic peak sits above the subsonic value (Hoerner: near Mach 1.0–1.02). Fin contribution can push the total up, but a required `CD0` of order 1.0 is the wind-tunnel discrepancy he showed on a different baseline, not the Sidewinder starting point. Powered flight uses the reduced base term; coast uses the full base term. That single switch already makes burn and coast different in kind.
- `CD0_wave` is zero in the formula set for `M < 1`. Do not evaluate the Bonney expression subsonically. Across `1.0 < M < 1.2`, blend from the subsonic (friction + base) value up to the supersonic sum so the peak is near Mach 1.0–1.05 rather than a step.
- Nose fineness `lN/d` of a Sidewinder is [unknown]. Use 2 to 2.5 only as a labeled assumption (AIM-9D was given an ogival nose). Wave drag is sensitive to it: `atan(0.5/(lN/d))` is the whole nose contribution.
- `Ae/S_ref` is [unknown] without a nozzle drawing. It only scales powered base drag. A value near 0.2–0.6 changes powered `CD0` by a few hundredths, which is inside the band.
- Calibration, one scale `k` on the whole Mach curve, tail chase, no maneuver (`CN = 0`):

```cpp
// Published AIM-9B acceptance cases (SI):
// h = 0 and h = 15240 m; M_launch = 1.2; M_target = 0.9
// R_pub = 1829 m (6000 ft) at sea level; 8534 m (28000 ft) at 50,000 ft
// V_m(0) = M_launch * a(h)          // shooter speed; rail increment ~ 0
// V_t    = M_target * a(h)          // co-altitude tail chase, same heading
// m0     = 160 lb = 72.57 kg        // OP 2309 table [published]
// m_propellant is [unknown]. The motor assembly is about 80 lb, case included,
// so propellant is less than 80 lb. Hold mass at m0 until a propellant mass exists.
// thrust_sl such that impulse matches 8440 lbf*s = 37540 N*s over 2.2 s
//   -> average thrust_sl = 3836 lbf = 17064 N if thrust is flat [derived]
// thrust(h) = thrust_sl + (p_sl - p(h)) * Ae     // existing back-pressure model
// CD0(M) = k * CD0_fleeman(M, powered_or_coast)
// Straight tail chase: integrate until closing velocity <= 0 or t > 24 s
// (homing commands would already be zero after 20 s; this check does not steer)
// range = integral (V_m - V_t) dt while (V_m > V_t)
// Choose k to match R_pub. Prefer the k that splits the error between the
// two altitudes rather than fitting only one.
```

- Accept `k` only if `k * CD0_fleeman` at the Mach numbers of the flyout stays inside about 0.2–0.6 (coast, outside the transonic peak). If the sea-level case and the 50,000 ft case want very different `k`, the Mach shape is wrong: adjust the transonic peak inside Hoerner’s description before leaving the band. If no `k` in that band hits both ranges, the thrust, mass, or the meaning of “range” in the 1960s manual is inconsistent with the aero model. Do not invent a `CD0` outside the band to force a match. Report the residual.
- AIM-9D is a second check, not a second free `CD0`. Do not fit `k` to it. The pilot handbook’s 3500 lbf for 5 s and the description manual’s 2645 lbf sea-level rating plus 13,968 lb·s minimum impulse disagree, as cited above. Use one of them only as a labeled sensitivity, not as a second truth. “Up to 18 km” has no stated altitude or target Mach. Launch mass on the family figure is 195 lb [published figure]. Gas-generator flight time is a nominal 60 s, which is also when that round self-destructs.

### Gaps
- No wind-tunnel `CD0` for an AIM-9, referenced to body cross-section, was found.
- Sidewinder nose fineness, nozzle exit area, and fin-panel area as used in the tail-friction term are [unknown]. The 18 km AIM-9D figure has no flight condition.
- AIM-9B propellant mass is [unknown]. Launch mass 160 lb is published, and the motor assembly is about 80 lb including the case, so propellant is only bounded above by 80 lb. Average thrust 3836 lbf is [derived] only if the 8440 lb·s is delivered at constant thrust over 2.2 s. Sea-level thrust-to-weight on the 160 lb launch mass is `3836/160 = 24.0` at ignition [derived] and rises as propellant is burned.
- AIM-9D thrust is in conflict between 3500 lbf average (OP 3353) and 2645 lbf at sea level, 70 °F (OP 3352 table). The 18 km figure still has no flight condition.

## Normal force, maximum lift coefficient, and structural g

### Takeaway
A published “maximum g” is the structural cap. Aerodynamic g is `q * CN * S_ref / (m g)` and equals that cap only when dynamic pressure is high enough. The AIM-9B manual’s altitude numbers are dynamic-pressure limited; its sea-level 10 g is the cap. With the manual’s 160 lb and the 4.2 g point, body-referenced `CN_max` is about 5.5, and 10 g is reached only above about 1.0×10^5 Pa.

### Cited Findings
- AIM-9B maneuver capability [published]: 2.7 g at 60,000 ft and Mach 2.3; 4.2 g at 50,000 ft and Mach 2.3; 10 g at sea level. The sea-level figure does not state a Mach number. — [NAVWEPS OP 2309](https://archive.org/details/OP23093rdAIM9B)
- Fleeman normal force for a circular body, `l/d > 5`, slender-body theory (Pitts) plus crossflow (Jorgensen), `alpha` in radians [published]:

```text
CN_body = abs(sin(2*alpha) * cos(alpha/2)) + 2 * (l/d) * sin(alpha)^2
```

The sign follows alpha. His `l/d = 20` chart goes to large `CN` by 20–40° because the reference area is the body, not a wing. — [Fleeman slides, p. 34](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- One surface (canard, wing, or tail), aspect ratio `A < 3`, effective angle `alpha_prime = alpha + delta`, planform `S_surface` [published]:

```text
double gate = sqrt(1.0 + pow(8.0 / (pi * A), 2));
double cn_over_area;
if (M > gate)
  cn_over_area = 4.0 * abs(sin(ap)*cos(ap)) / sqrt(M*M - 1.0) + 2.0 * sin(ap)*sin(ap);
else
  cn_over_area = (pi * A / 2.0) * abs(sin(ap)*cos(ap)) + 2.0 * sin(ap)*sin(ap);
CN_surface = cn_over_area * (S_surface / S_ref);
```

His rocket-baseline example at Mach 2, `alpha = 9°`, `delta = 13°`, `A = 2.82`, gets `CN_wing = 7.91` on body area. Total `CN ≈ CN_body + CN_wing + CN_tail` at zero control, and the wing dominates that particular baseline. — [Fleeman slides, pp. 45 and 74](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- Conventional canard control “stalls at high alpha if statically stable.” Relaxed static margin on a tail-controlled missile raises trim alpha and trim `CN`. Fixed surfaces ahead of a movable canard are his fix for canard stall. — [Fleeman slides, pp. 58 and 70](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- AIM-9X is the opposite arrangement from a classic Sidewinder: fixed forward wings, movable tail, plus jet vanes. Classic AIM-9 maneuver devices are the nose canards. — [AIM-9X NTSP](https://www.globalsecurity.org/military/library/policy/navy/ntsp/AIM-9X.pdf); [AIM-9, English Wikipedia, design section](https://en.wikipedia.org/wiki/AIM-9_Sidewinder)
- Load factor identity used below is Newton’s law, not a missile-specific empirical fit: `n = (q * CN * S_ref) / (m * g)`.

### Inferences
- Two different kinds of airframe, not two thrust values: a canard Sidewinder should use a lower `alpha_max` (canard stall), a tail-controlled AIM-9X a higher `alpha_max` and therefore a higher `CN_max` on the same diameter. TVC does not raise `CN_max`; it adds the thrust term in the previous section at low `q`.
- Turn a published structural g into a lift coefficient so the missile reaches that g only at high `q`:

```cpp
// n_struct  = published maximum g (AIM-9B sea-level figure: 10)
// q_design  = dynamic pressure at which aero alone is allowed to hit n_struct
// CN_max    = n_struct * mass * g / (q_design * S_ref)
double n_aero = (q * CN_max * S_ref) / (mass * g);
double n_cmd  = std::min(n_struct, n_aero);   // TVC added only during burn, then re-clamp
```

- AIM-9B split [derived from the published g, the 160 lb launch mass, and the 1976 standard atmosphere]: both Mach 2.3 points are at similar `CN`, because g scales almost with density. At Mach 2.3, `q ≈ 26.6 kPa` at 60,000 ft (`rho ≈ 0.115 kg/m³`, `a ≈ 295 m/s`) and `q ≈ 43.0 kPa` at 50,000 ft (`rho ≈ 0.186 kg/m³`). The g ratio `4.2/2.7 = 1.56` versus the `q` ratio `1.62` agrees within about 4 percent. Sea-level `q` at the same Mach is about 375 kPa. The manual’s 10 g at sea level does not state a Mach number; it is the structural or control cap, not the aerodynamic ceiling. Set `n_struct = 10` for an AIM-9B-class round. Invert the 50,000 ft point:

```cpp
// m = 160 lb = 72.57 kg          [published]
// S_ref = pi * (0.0635 m)^2 = 0.01267 m^2    // 5-inch body [published]
// q_50k_M23 = 4.30e4 Pa          [derived, 1976 standard atmosphere]
// CN_max = 4.2 * 72.57 * 9.80665 / (4.30e4 * 0.01267) = 5.49   [derived]
double n_aero = (q * 5.49 * 0.01267) / (mass * 9.80665);
```

- With that `CN_max`, `n_aero` hits 10 only when `q` is about `10/4.2` times the 50,000 ft Mach 2.3 value, i.e. about `1.02×10^5 Pa` [derived]. Below that `q`, the missile is aero-limited and must not be given 10 g. Above it, clamp at 10 g. The same `CN` at sea level and Mach 2.3 is about 37 g [derived], which is why the 10 g line is a cap. Do not copy a video-game “30 g” or “35 g” onto the AIM-9L; those figures were not in a manual opened here. `CN_max = 5.49` is on body area and uses launch mass. As propellant burns, that same `CN` produces a higher g because mass is lower. Leave `CN_max` constant through the burn. The structural cap stays 10.
- Later TVC missiles can exceed the AIM-9B’s 10 g during the burn because of `T*sin(delta)/mass`, up to their own `n_struct`. IRIS-T’s published 60 g and PL-10’s reported “over 60 g” are candidates for `n_struct` of those rounds only, and only with the source-quality caveat in the TVC section. They still need a `CN_max` or they will pull 60 g in thin air, which the AIM-9B table shows is the wrong behavior.

### Gaps
- `CN_max = 5.49` uses launch mass and the 50,000 ft point. Propellant mass is still unknown, so the burn does not yet change `CN_max`. No manual g was used for the AIM-9D’s 195 lb round, so that airframe does not inherit 5.49.
- No manual g was found for AIM-9L/M/X. Wikipedia and NAVAIR do not state one. IRIS-T 60 g is second-hand via Wikipedia. R-73 “40 g” is an aggregator figure and conflicts with other cards.
- `S_surface` and aspect ratio of Sidewinder canards are [unknown], so the surface-`CN` formula cannot be evaluated numerically for an AIM-9 without a drawing. The altitude-g inversion does not need those areas.

## Seeker aspect and infrared signature

### Takeaway
Rear-aspect seekers (AIM-9B, R-3S, early Magic) lock on exhaust only, so a nose-on target is invisible to them. The AIM-9D is a cooled middle case: detection range falls as the shot leaves the tail, and a nose-on lock exists only inside 20° with the target in afterburner. All-aspect seekers also see the warm airframe, but the stern is still much brighter, so a head-on lock is at a much shorter range. Score `I(aspect) / range^2`, not a mild constant bias.

### Cited Findings
- AIM-9B uncooled seeker heads “could track only the high temperatures of engine exhaust, making them strictly rear-aspect.” Later cooled seekers “track any part of the aircraft heated by air resistance,” which is the all-aspect step. — [AIM-9, English Wikipedia, design section](https://en.wikipedia.org/wiki/AIM-9_Sidewinder)
- Early seekers were most effective on shorter wavelengths, “such as the 4.2 micrometre emissions of the carbon dioxide efflux of a jet engine,” and were “useful primarily in tail-chase scenarios.” Sensitivity extended toward 8–13 µm “allows dimmer sources like the fuselage itself to be detected.” Those are the all-aspect seekers. Front and side signals are described as lower level than the exhaust, and they need cooling or the sensor’s own heat swamps them. PbS is the older detector; InSb and HgCdTe are the later ones. — [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing)
- Falklands-era account on the same page: Argentine AIM-9B and R.550 Magic “could only fire from the rear aspect.” The AIM-9L could be used from other aspects. R-3S is the copied AIM-9B (K-13), so it inherits the rear-aspect seeker. — [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing); [AIM-9, English Wikipedia, K-13 / R-3S section](https://en.wikipedia.org/wiki/AIM-9_Sidewinder)
- Open countermeasure article: “Fighter aircraft have an IR signature which is much larger in the stern than on the beam or forward quadrant.” A flare-to-target energy ratio of about 2:1 is described as what it takes to seduce a missile with no IRCCM. The same flare “can be much greater than 10:1 in the forward quadrant” if it was 2:1 in the stern. That is an intensity comparison, not a lock-range table. The article’s “2.5:1 within 40 ms” sentence is a hypothetical rise-time example (“might permit”), not a measured aircraft signature and not an AIM-9M constant. — [Journal of Electronic Defense article, “Advanced infrared missile counter-countermeasures,” via The Free Library](https://www.thefreelibrary.com/Advanced+infrared+missile+counter-countermeasures.-a015149906)
- Mahulikar and colleagues: from the frontal aspect the airframe dominates (leading edges and nose heated by the flow, and only when Mach is high); from the rear aspect the hot tailpipe dominates; from the side, the heated rear fuselage and the plume. Plume radiation is band-limited; the tailpipe behaves more like a gray body. Lock-on range is described as depending directly on contrast intensity. — [Mahulikar et al., aircraft IR signature vs aspect, ResearchGate record of the atmospheric-transmission paper](https://www.researchgate.net/publication/245430242_Effect_of_Atmospheric_Transmission_and_Radiance_on_Aircraft_Infared_Signatures); [plume and lock-on range discussion](https://www.researchgate.net/publication/260433935_Aircraft_Powerplant_and_Plume_Infrared_Signature_Modelling_and_Analysis)
- AIM-9D is a third kind, not an all-aspect AIM-9L. The cooled seeker can home on low-temperature exhaust and on shielded tailpipes. Nose-on homing is restricted to targets in afterburner and to firing positions within 20° of dead ahead. As angle off the tail increases, detection range decreases; the handbook gives no exponent. Selecting afterburner doubles detection range. A target boresighted on blue sky is detected at almost twice the range of the same target against cloud or ground. The seeker can be decoyed by the sun, and the pilot is told to hold fire until the missile is pointed at least 30° away from the sun. — [NAVWEPS OP 3352](https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html); [NAVWEPS OP 3353](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%203353%20AIM-9C%20&D%20PIlot%27s%20Handbook_djvu.txt)
- Wikipedia’s R-27T “2–33 km head-on, 0–5.5 km tail-on” figures are engagement ranges, not seeker lock ranges. They run the opposite way from signature (head-on kinematic range is longer) and must not be used as `I(aspect)`. — [R-27, English Wikipedia](https://en.wikipedia.org/wiki/R-27_(air-to-air_missile))

### Inferences
- Let `psi` be the aspect at the target: 0 looking at the nose, `pi` looking up the tail. Irradiance on the seeker falls as `I(psi) / R^2`. A lock requires that irradiance above a threshold, inside the field of view. Because `R_lock ∝ sqrt(I)`, an intensity ratio becomes a range ratio by a square root.
- From the 2:1 stern versus “much greater than 10:1” forward flare comparison, forward-quadrant intensity is under about `2/10 = 0.2` of stern intensity [derived upper bound, not a fitted spectrum]. So an all-aspect head-on lock range is under about `sqrt(0.2) ≈ 0.45` of the tail-on range, and the words “much greater than 10:1” mean the real ratio is smaller than 0.45. Beam aspect is also far below the stern. No opened paper gave a tighter constant, so the sim should expose the forward fraction and default it at or below 0.2, not invent 0.1 as a fact.
- Rear-aspect seeker (AIM-9B, R-3S, early Magic): the forward fraction is zero [published behavior].

```cpp
// psi: 0 = nose-on, pi = tail-on
double rear = std::max(0.0, -std::cos(psi)); // 1 at the tail, 0 at and forward of the beam
double I_rear_aspect = I_tail * rear;
// head-on: I = 0, no lock at any range
```

- All-aspect seeker (AIM-9L and later, R-73, and the imaging weapons): a small forward term plus the same stern lobe.

```cpp
double f_forward = 0.2; // upper bound [derived]; do not exceed this without a new source
double I_all_aspect = I_tail * (f_forward + (1.0 - f_forward) * rear);
// R_lock(psi) / R_lock(tail) = sqrt(I_all_aspect / I_tail)
```

- The lobe `max(0, -cos(psi))` is [derived]: it is the simplest function that is full on the tail, zero on the nose, and zero on the beam, matching “stern much larger than beam or forward” plus “tailpipe not visible from the front.” A sharpening power on `rear` is [unknown]; leave the exponent at 1 until a polar plot is digitized. The AIM-9D handbook’s “decreases correspondingly” with angle off the tail is the same shape statement and still has no exponent.
- AIM-9D is not the all-aspect function. Keep the rear lobe, and add a separate head-on gate. The intensity inside that gate was not published, so do not reuse `f_forward = 0.2`.

```cpp
// AIM-9D only. psi = 0 is nose-on. Angles in radians.
bool head_on_gate = afterburner && (psi <= 20.0 * M_PI / 180.0);
// I inside the 20° afterburner cone: [unknown]. Do not set it to 0.2 * I_tail.
// Afterburner doubles detection range [published]. If R_lock ∝ sqrt(I),
// that is I_tail *= 4 while afterburner is selected [derived from the range sentence].
// Blue sky versus cloud or ground is another published factor of about 2 on range,
// i.e. another factor of about 4 on I, and it is background, not aspect.
```
- Do not add aerodynamic heating to a PbS rear-aspect seeker. Those seekers do not see warm skin [published]. For an all-aspect seeker the forward term is exactly the skin/heating term Wikipedia describes. A Mach dependence can multiply only `f_forward`, using stagnation temperature `T0/T = 1 + 0.2*M*M` as a scale, but the band radiance is not Stefan-Boltzmann `T^4`. Leave that multiplier at 1 unless a spectrum is added later. Afterburner should raise `I_tail`, not `f_forward`.
- Replace the sim’s mild rear-aspect bias with this `I(psi)`. The old bias cannot keep an AIM-9B from locking on the nose.

### Gaps
- No digitized lock-range polar, with a stated target and seeker, was opened. The 0.2 forward/stern bound is an inequality from a flare-ratio argument, not a radiometric measurement.
- Absolute `I_tail` (W/sr) by aircraft and power setting is [unknown] here. Only the aspect shape is constrained. The sim can keep its existing heat scale and multiply by `I(psi)/I_tail`. The intensity inside the AIM-9D’s 20° afterburner nose cone is also [unknown]; the manual states the gate and not the range.
- Plume versus tailpipe spectral split (CO2 band versus gray-body metal) is described qualitatively by Mahulikar and was not reduced to a second fitted constant.

## Gimbal limit versus track rate

### Takeaway
Instantaneous field of view is the small cone the detector sees around the seeker boresight. The gimbal limit is how far that boresight may sit from the missile body axis, not from the velocity vector. Track rate is how fast the boresight may slew. If the target leaves the instantaneous field of view, lock is lost and the round goes ballistic; a coast time was not found.

### Cited Findings
- Instantaneous field of view is “the angle the detector sees.” The overall field of view, also called the tracking angle or off-boresight capability, “includes the movement of the entire seeker assembly.” The seeker is on a gimbal. “The angle between the seeker and the missile” is what the guidance uses. “A target moving rapidly … may be lost from the IFOV, which gives rise to the concept of a tracking rate, normally expressed in degrees per second.” — [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing)
- The same page’s one-degree versus ten-degree tracking-angle discussion is marked citation-needed. It was not used as a number.
- AIM-9B optics: infrared in a 4° solid-angle cone is imaged on the reticle. A figure note says the gyro axis may be offset from the missile axis by 0 to 30°. The AIM-9D pilot handbook says the gimbal limit was increased from 25° to 40°, so the pre-D limit is given as 25° in that later book and as 30° on the AIM-9B figure. Report both; do not average them. Both are measured from the missile axis. — [NAVWEPS OP 2309 full text](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%202309%20(3rd)%20AIM-9B_djvu.txt); [NAVWEPS OP 3353](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%203353%20AIM-9C%20&D%20PIlot%27s%20Handbook_djvu.txt)
- AIM-9D manuals: seeker telescope field of view 2.5° and gimbal angle 40° in any direction from the missile longitudinal axis. The gyro can precess 40° from that axis, and the head coil allows 80° of gimbal freedom, which is the full plus-or-minus travel. Wikipedia and a 1994 survey instead say the field of view is “beyond 25 degrees.” That secondary figure conflicts with both manuals. Use 40° and 2.5°. — [NAVWEPS OP 3352](https://archive.org/download/OP3352AIM9D/OP%203352%20AIM-9D%201_text.pdf); [OP 3352 hocr](https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html); [AIM-9, English Wikipedia](https://en.wikipedia.org/wiki/AIM-9_Sidewinder); [Kopp, “The Sidewinder Story,” 1994](https://www.ausairpower.net/TE-Sidewinder-94.html)
- Wikipedia comparison table, track rate in degrees per second, columns AIM-9B, D, E, G, H, J: 8.0–11.0, 12.0, 12.0, 12.0, 20.0, 16.5. Reticle rate (Hz) in the same columns: 70, 125, 100, 125, 125, 100. The AIM-9E prose on that page instead says “a 100 Hz reticle rate, and a 16.5 deg/sec tracking rate,” which matches the table’s reticle column for the E but not the table’s 12°/s track-rate cell. AIM-9H prose says the track rate went “from the original 12° to 20° per second.” Report the conflict; do not average it. — [AIM-9, English Wikipedia](https://en.wikipedia.org/wiki/AIM-9_Sidewinder)
- AIM-9X: imaging 128×128 focal-plane seeker “with claimed 90° off-boresight capability.” Block II adds lock-on after launch and a datalink for 360° engagements, which is a cueing envelope, not a statement that the gimbal itself exceeds 90° from the body axis. — [AIM-9, English Wikipedia](https://en.wikipedia.org/wiki/AIM-9_Sidewinder); [NAVAIR AIM-9X](https://www.navair.navy.mil/product/AIM-9X-Sidewinder)
- ASRAAM: imaging 128×128 seeker, “approximately 90-degree off-boresight lock-on capability,” plus LOAL and strapdown inertial guidance. — [ASRAAM, English Wikipedia](https://en.wikipedia.org/wiki/ASRAAM)
- R-73 angles conflict, as already cited: Wikipedia baseline “up to 40° off the missile’s centerline”; AusAirpower/Vympel card ±45° engagement and ±75° gimbal; later variants on Wikipedia are listed at ±60° and ±75°. — [R-73 Wikipedia](https://en.wikipedia.org/wiki/AA-11_Archer); [AusAirpower](https://www.ausairpower.net/APA-NOTAM-200408-1.html)
- AIM-9E combat notes list “the missile going ballistic” among reasons shots failed, alongside launches outside the envelope. That is an open description of lost guidance: the round no longer steers. — [AIM-9, English Wikipedia, AIM-9E section](https://en.wikipedia.org/wiki/AIM-9_Sidewinder)
- Early Sidewinder guidance is proportional to seeker-head motion: the spinning mirror is a gyro; the voltage that precesses it onto the target also drives the canards, so missile turn rate follows line-of-sight rate. If the head is on its stop, that signal no longer tracks the target. — [Proportional navigation, English Wikipedia](https://en.wikipedia.org/wiki/Proportional_navigation)

### Inferences
- The sim’s acquisition cone around the velocity vector is the wrong axis for the gimbal stop. Angle of attack puts the velocity vector off the body axis. Use the body axis.

```cpp
// body_forward, seeker_boresight, los_hat are unit vectors
double gimbal = std::acos(clamp(dot(seeker_boresight, body_forward), -1.0, 1.0));
bool on_stop = gimbal >= gimbal_limit;          // mechanical stop from the body axis
double los_off = std::acos(clamp(dot(los_hat, seeker_boresight), -1.0, 1.0));
bool in_ifov = los_off <= 0.5 * ifov;           // ifov is the full cone angle
// slew seeker_boresight toward los_hat at no more than track_rate rad/s,
// and never past gimbal_limit from body_forward
```

- Suggested public numbers, each tagged: AIM-9B `ifov = 4°` [published]. AIM-9B gimbal 25° [OP 3353, the limit that was raised] or 30° [OP 2309 figure note]. Keep the conflict. AIM-9D `ifov = 2.5°` and `gimbal_limit = 40°` from the body axis [published manuals]. Do not use the secondary “beyond 25°.” AIM-9B track rate 8–11°/s, AIM-9H 20°/s [published Wikipedia table]. AIM-9E 12°/s (table) or 16.5°/s (prose): pick one and keep the other as a conflict, do not silently average. Kopp’s 1994 table picks 11°/s for the B, 12°/s for the D, and 16.5°/s for the E, which agrees with the prose and not with the Wikipedia track-rate cell for the E. AIM-9X and ASRAAM off-boresight 90° from the body [published claim], with track rate [unknown]. R-73 baseline gimbal 40° (Wikipedia) or 75° (Vympel card via AusAirpower): do not merge them.
- Break-lock rule, from the open description, with no invented timer: if the target is outside the IFOV and the seeker cannot catch it because of the track-rate limit or the gimbal stop, lock is lost. Commanded homing acceleration goes to zero. The missile coasts ballistically under drag, thrust if still burning, and gravity. Rollerons on a classic Sidewinder are a passive roll damper, not a homing mode. If the target re-enters the IFOV and the irradiance test passes, lock may be regained. A numeric coast or memory-track time is [unknown] and should be zero until a source gives one.
- LOAL (next section) points the inertial system along the cued line of sight, or along the rail if the cue is the boresight. The seeker gimbal then searches inside its stop. The acquisition basket is the IFOV swept through the allowed gimbal angle about the body, not a cone about velocity.

### Gaps
- A single published gimbal angle for AIM-9L/M was not opened. “High off-boresight” on Wikipedia is qualitative. Forum figures of 40° gimbal and 2.5° FOV for the L/M were not used as facts. The AIM-9D’s 40° and 2.5° must not be copied onto the L/M. Track rate in degrees per second was not found in OP 2309 or OP 3352; the B/D/E/H numbers remain the Wikipedia table, with the E conflict above.
- AIM-9X track rate in degrees per second is [unknown]. “90° off-boresight” is a look angle.
- No published break-lock coast duration was found. “Going ballistic” is the failure mode, not a timer.

## IRCCM classes

### Takeaway
Four public behaviors are enough, and they are rules about which source wins. They are not flare-defeat procedures. Only the first class is fully specified numerically in open literature (a roughly 2:1 hotter source steals an unhardened reticle seeker). The 40 ms example in the rise-time article is illustrative, not an AIM-9M constant.

### Cited Findings
- Without counter-countermeasures, a decoy has to outshine the aircraft. The open article’s planning figure is a flare-to-target ratio of about 2:1 in the stern to make an unhardened missile transfer lock. Fighters are much dimmer off the stern, so the same flare is a larger ratio there. — [“Advanced infrared missile counter-countermeasures”](https://www.thefreelibrary.com/Advanced+infrared+missile+counter-countermeasures.-a015149906)
- That article names the switch classes in plain language: rise time (temporal), two-color (spectral), kinematic, and spatial. Rise time: “a sharp rise in the received energy within a specified time limit indicates a flare.” Its numerical illustration is “a threshold of a 2.5:1 energy increase within 40 msec might permit flare detection … while ignoring the relatively slow energy rise from afterburner ignition.” The words are “might permit.” They are not identified as AIM-9M firmware. — same article
- Rosette or crossed-slot scanning, as summarized from Deuerle: the seeker remembers when the target should cross the detectors and rejects signals outside that gate. “Flares tend to stop in the air almost immediately after release, they quickly disappear from the scanner’s gates.” That is kinematic rejection described without a processing schematic. — [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing)
- Imaging seekers “see” the target as a picture and “can distinguish between an aircraft and a point heat source such as a flare.” A separate sentence on the infrared-homing page: flares can be recognized by small size, clouds by larger size. AIM-9X, ASRAAM, MICA IR, Python 5, and PL-10 are publicly imaging seekers. — [Air-to-air missile, English Wikipedia](https://en.wikipedia.org/wiki/Air-to-air_missile); [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing); [Rafael Python 5](https://www.rafael-usa.com/programs/python-5/)
- Python 5’s own description of the difference: conventional seekers “see targets as dots”; the imaging seeker sees an image of target and background and authenticates the target, which is why a separated flare does not simply become the new point source. — [Rafael Python 5 brochure](https://www.rafael.co.il/wp-content/uploads/2024/09/python-Air-to-Air-and-Air-defense-Missile.pdf); earlier public telling archived at [Israeli-weapons.com, 2006](https://web.archive.org/web/20060715230748/http://www.israeli-weapons.com/weapons/missile_systems/air_missiles/python/Python5.html)
- AIM-9M is publicly an AIM-9L follow-on “to better reject flares.” The sentence that says so on the infrared-homing page is tagged citation-needed. No opened primary source states that the AIM-9M’s method is rise-time rather than a two-color or gate method. — [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing)

### Inferences
- Implement four mutually exclusive seeker classes. Do not combine them into a single 0–1 “flare resistance” if the point is that the weapons differ in kind.
- Class `none` (AIM-9B, R-3S, early Magic, and an AIM-9D that has no rise-time or imaging test in the manuals): among sources inside the IFOV, the hottest in-band source wins. A new source with intensity greater than about twice the currently tracked target, using the article’s 2:1 stern figure, takes the track. There is no rate test and no shape test. The AIM-9D handbook’s sun rule is this class in the cockpit: do not point the missile within 30° of the sun, because the sun can pull the seeker off the aircraft. That 30° is a pilot hold-off, larger than the 2.5° IFOV; it is not a second gimbal limit.
- Class `rise` (the open class people associate with AIM-9M; the association itself is not a primary citation): if a new source appears and the tracked intensity jumps by more than a threshold in a short time, do not move the track point to it. Keep steering on the old line of sight. The article’s 2.5:1 in 40 ms is an example of the kind of threshold, marked as hypothetical. Put the threshold in a named constant with a comment that it is not an AIM-9M specification. A slow brightening (afterburner) must not trip it; that is the distinction the article draws.
- Class `kinematic` (rosette / gate seekers, described openly for this behavior, not tied here to one missile’s classified mode): a candidate that separates and then decelerates or drops, so its line-of-sight rate leaves the gate around the aircraft track, is rejected. The track stays on the object whose motion is consistent with the aircraft. This is a rule about which object remains the target, not a recipe for how to fly an aircraft relative to the missile.
- Class `imaging` (AIM-9X, ASRAAM, IRIS-T, MICA IR, Python 5, PL-10): if the aircraft is an extended object in the image and the flare is a separated point source, the aim point stays on the aircraft. A flare that is still inside the aircraft image is not a second object yet; once it is spatially separated it does not pull the track. No further discrimination logic was published in the sources used here, and none should be added.
- Two-color spectral rejection is a fifth class the same article names. It was not one of the four requested rules. Leave it unused rather than inventing band ratios.

### Gaps
- Which of these four classes the AIM-9M actually implements is [unknown] in primary material. “Better flare rejection than the L” is all that is solid, and even that sentence is poorly cited on Wikipedia.
- No production threshold, gate width, or image-processing parameter was published. The 2:1 ratio is the only numeric seduction figure used above, and it applies to the unhardened class.

## Lock-on after launch

### Takeaway
LOAL means the missile is fired on an inertial cue, then the seeker acquires inside a basket. AIM-9X Block II, ASRAAM, IRIS-T, MICA IR, and Python 5 are all publicly described as having it. A classic Sidewinder is lock-before-launch and then proportional navigation after the 0.5 s inhibit.

### Cited Findings
- AIM-9X Block II “adds lock-on after launch capability with a datalink, so the missile can be launched first and then directed to its target afterwards” for 360° engagements. NAVAIR: Block II can hit targets behind the launching fighter using datalink, thrust vectoring, and the imaging seeker. Block I “demonstrated potential” for LOAL; it was not the fielded datalink mode. — [AIM-9, English Wikipedia, Block II](https://en.wikipedia.org/wiki/AIM-9_Sidewinder); [NAVAIR AIM-9X](https://www.navair.navy.mil/product/AIM-9X-Sidewinder); [Military Aerospace, AIM-9X Block II LOAL](https://www.militaryaerospace.com/sensors/article/55240036/raytheon-technologies-corp-air-to-air-missiles-infrared-guided-helmet-mounted-displays)
- ASRAAM: strapdown inertial guidance and LOAL are in the guidance description. In March 2009 the RAAF fired an in-service ASRAAM at a target behind the wing line, the public over-the-shoulder LOAL shot. Off-boresight lock-on about 90°. — [ASRAAM, English Wikipedia](https://en.wikipedia.org/wiki/ASRAAM)
- IRIS-T: can engage targets behind the launch aircraft by close-in agility “and LOAL capability.” Seeker cues listed in compilation include radar, helmet, IRST, missile-approach warner, and data link. The German article adds four vanes in the nozzle, inertial navigation, and LOAL when the target is outside the ±90° seeker field at launch. It also says that 0.5 s passes from firing until that 90° seeker acquires a target behind the aircraft. The same page’s “well over 100 g” sentence is the author’s inference and is not a Diehl figure. — [IRIS-T, English Wikipedia](https://en.wikipedia.org/wiki/IRIS-T); [IRIS-T, German Wikipedia](https://de.wikipedia.org/wiki/IRIS-T)
- MICA: MBDA lists strap-down inertial, data link, LOAL, LOBL, helmet and radar and electro-optical designation, and an imaging IR seeker. Midcourse inertial update, then in-flight seeker lock, then homing. — [MBDA MICA brochure](https://www.mbda-systems.com/media/20541/download)
- Python 5: Rafael states full-sphere launch from “high agility, Lock-On-After-Launch (LOAL) and excellent acquisition,” an inertial navigation system, and both LOBL and LOAL. The seeker “scans the target area” and then locks for the terminal chase. Wikipedia’s “up to 100 degrees off the boresight of the launching aircraft” is an aircraft-cue angle in a compilation, not a measured gimbal stop in the brochure extract. — [Rafael brochure](https://www.rafael.co.il/wp-content/uploads/2024/09/python-Air-to-Air-and-Air-defense-Missile.pdf); [Rafael USA](https://www.rafael-usa.com/programs/python-5/); [Python, English Wikipedia](https://en.wikipedia.org/wiki/Python_(missile))
- R-73 is described, in the infrared-homing history, as able to be fired at targets outside the seeker view: after launch it “would orient itself in the direction indicated by the launcher and then attempt to lock on,” with a helmet sight. That is LOAL in behavior even though the phrase “lock-on after launch” is the article’s paraphrase. The Vympel card’s ±75° gimbal versus ±45° engagement is the related hardware limit. — [Infrared homing, English Wikipedia](https://en.wikipedia.org/wiki/Infrared_homing)
- Classic AIM-9B/D manuals describe a seeker that is already tracking at launch and proportional navigation afterward. They do not describe inertial midcourse. — [OP 2309](https://archive.org/details/OP23093rdAIM9B); [OP 3352](https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html)

### Inferences
- Two launch kinds:
  1. Lock before launch (AIM-9B/L/M and any shot where the seeker has already growled): rail launch, inhibit steering for 0.5 s on an AIM-9B-class model, then proportional navigation. The seeker boresight starts on the locked line of sight, still inside the body-axis gimbal limit.
  2. Lock after launch (AIM-9X Block II, ASRAAM, IRIS-T, MICA IR, Python 5): at release, the missile has a cue (helmet, radar, or IRST line of sight) but no seeker track. Fly inertially toward that line, or toward a predicted intercept point if a data link is updating it (MICA and AIM-9X Block II). Slew the seeker through its gimbal basket about the body axis. When `I(psi)/R^2` exceeds threshold and the target is inside the IFOV, switch to proportional navigation. If the basket is never entered before the motor is out and the energy is gone, the shot misses. There is no pitch-over autopilot and no terrain avoidance.
- R-73: model helmet cue plus a post-launch acquisition turn, because that behavior is described openly. Do not also give it MICA’s long-range inertial cruise unless a source is chosen in the R-27/R-73 conflict notes.
- The IRIS-T 0.5 s figure is a published encyclopedia claim for how fast the seeker can be looking behind the fighter. It is not the AIM-9B guidance inhibit, and it was not read off a Diehl brochure in this pass. Do not apply it to AIM-9X.
- R-27T: leave LOAL off by default because the open sources disagree (see the rail-launch section).

### Gaps
- The angular size of the LOAL acquisition basket, in degrees, is [unknown] for every missile named here. Using the gimbal limit as the basket is an inference, not a published scan pattern.
- Data-link update rate and whether the cue is a line of sight or a full track (position and velocity) are [unknown] outside the qualitative “data link” statements.

## Proximity fuze arming

### Takeaway
The warhead stays inert for a short distance after launch so it cannot endanger the fighter. For the AIM-9B that distance is hundreds of feet for the contact fuze and about 2,000 ft for the influence fuze. The AIM-9D handbook’s minimum range is 1,000 ft in a co-speed tail shot and `1000(1 + 2 ΔMach)` feet when the shooter is faster.

### Cited Findings
- AIM-9B: both fuzes are mechanically armed between 480 and 840 ft from the firing aircraft. The influence fuze is then armed electrically after motor burnout, about 2,000 ft from the firing aircraft. Self-destruct about 24 s if it never passes near the target. — [NAVWEPS OP 2309](https://archive.org/details/OP23093rdAIM9B)
- AIM-9C/D handbook: influence fuzes “prevent warhead detonation during the first 600 to 1000 feet of missile flight to protect the firing aircraft,” and this “prevents effective use of the missile when firing at a range less than 1000 feet.” For the AIM-9D the minimum firing range in a tail-on co-speed shot is that 1,000 ft floor and is stated to be true at all altitudes. Other overtaking shots use `R_min = 1000(1 + 2 ΔMach)` feet. The worked example is Mach 1.5 against Mach 0.8, so `ΔMach = 0.7`, and the result printed is 2,400 ft, which is `1000(1 + 2×0.7)`. The radar AIM-9C uses a different floor, 2,000 ft co-speed, and `R_min = 1000(2 + 3 ΔMach)` feet (same example, 4,100 ft). The sentence that names the co-speed footage in words is partly blank in the OCR; the formula, the “less than 1000 feet” sentence, and the 2,400 ft example agree. — [NAVWEPS OP 3353](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%203353%20AIM-9C%20&D%20PIlot%27s%20Handbook_djvu.txt)
- AIM-9X NTSP: the electronic safe-and-arm device arms the warhead after launch. No time or distance is given. — [AIM-9X NTSP](https://www.globalsecurity.org/military/library/policy/navy/ntsp/AIM-9X.pdf)
- R-73 minimum engagement range “about 300 meters” (Wikipedia) and 0.3 km on a receding target (AusAirpower/Vympel card). That is a minimum firing range, not necessarily the fuze-clock time. — [R-73 Wikipedia](https://en.wikipedia.org/wiki/AA-11_Archer); [AusAirpower](https://www.ausairpower.net/APA-NOTAM-200408-1.html)

### Inferences
- Arming is a distance from the launcher, not a seeker event. For an AIM-9B-class model:

```cpp
bool contact_armed  = range_from_launch > 150.0;  // ~480 ft, early edge of 480–840 ft
bool influence_armed = range_from_launch > 610.0; // ~2000 ft
// A mid value of the mechanical window is 200 m (about 660 ft) if one number is wanted.
```

- For an AIM-9D-class model the handbook’s minimum is 1,000 ft (305 m) co-speed, tail-on, at any altitude, and it grows with overtake:

```cpp
// OP 3353, AIM-9D, feet converted to metres. delta_mach = mach_shooter - mach_target.
// Example: 1.5 vs 0.8 -> 2400 ft = 732 m [published].
double R_min_m = 0.3048 * 1000.0 * (1.0 + 2.0 * std::max(0.0, mach_shooter - mach_target));
```

The AIM-9C formula is a different missile (radar) and is not the infrared default. Arming distance (600–1,000 ft) and minimum firing range (the formula) are the same order but they are not the same gate: the fuze will not function inside the arming distance even if the formula is smaller.

- The surface-launched missile keeps its own arming. Do not copy the air-to-air distances onto it, and do not arm an air-to-air round with the surface missile’s pitch-over logic.
- A modern laser or radar proximity fuze (AIM-9L DSU-15 family, AIM-9X reusing the AIM-9M target detector) still needs a safe-separation arm. The NTSP only says “after launch.” Until a distance is published, reuse the AIM-9D 600–1000 ft window for those rounds and tag it [inferred from the earlier Sidewinder, not published for the 9X].

### Gaps
- AIM-9L/M/X arming time or distance is [unknown]. The 9X statement is only “after launch.”
- The co-speed footage in the AIM-9D sentence “approximately ___ feet” is blank in the OCR. The formula constant, the 2,400 ft example, and the “less than 1000 feet” arming sentence are the readable parts.
- IRIS-T, MICA, ASRAAM, Python 5, and PL-10 arming distances were not found.

## Navigation constant

### Takeaway
Tactical proportional navigation uses an effective navigation ratio N of about 3 to 5. Nothing public assigns a different N to high off-boresight infrared missiles. High off-boresight changes the problem by saturating the g limit early, not by requiring N outside that range. N = 4 is the value Fleeman actually works examples with.

### Cited Findings
- True proportional navigation, the form this sim already uses: acceleration perpendicular to the line of sight, `a = N * V_closing * Omega_LOS`, with N “generally having an integer value 3–5.” The Wikipedia 3-D expression is `a = -N * |V_r| * (R_hat × Omega)`. Pure proportional navigation instead puts the acceleration normal to the missile velocity. Early Sidewinder hardware implemented the idea by driving the canards from the seeker-gyro precession voltage, so turn rate followed line-of-sight rate without an explicit N computed in software. — [Proportional navigation, English Wikipedia](https://en.wikipedia.org/wiki/Proportional_navigation), citing Yanushevsky, *Modern Missile Guidance*, and Siouris, *Missile Guidance and Control Systems* (2004)
- Siouris, as quoted in an open copy of that book: N between 3 and 5 is what is usually used so miss distance is acceptable without excess acceleration. The classical stability bound cited alongside it is N′ > 2. If the missile’s maximum lateral acceleration is three times the target’s, N′ should be at least 3. — [Siouris, *Missile Guidance and Control Systems*, ch. 4, as exposed by the IDU archive text](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/MILITARY%20PLATFORM%20DESIGN/Missile%20Guidance%20And%20Control%20Systems.pdf)
- Fleeman’s guidance slides use proportional navigation with effective ratio N′. One worked seeker example sets “Proportional Guidance Navigation Ratio = 4.” For an ideal missile (`tau = 0`) and N′ = 3, the acceleration ratio at intercept is `n_missile / n_target = 3`. Against an initial heading error, early acceleration grows with N′: `a_M * t0 / (V_M * gamma_M) = N' * (1 - t/t0)^(N' - 2)`. He plots N′ = 2, 2.5, 3, 4, and 6 for that heading-error transient. Higher N′ front-loads the turn. — [Fleeman slides, pp. 211 and 242–244](https://web.archive.org/web/20160104053657/http://faculty.kfupm.edu.sa/AE/aymanma/images/TMDPresentation.pdf)
- OP 3352 defines the navigation constant for the AIM-9D and does not give its number: “the turning rate of the missile is N times the line-of-sight turning rate.” — [NAVWEPS OP 3352](https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html)
- The only opened manual that changes N in flight is the radar AIM-9C, not an infrared missile. A deviated-pursuit computer “automatically changes the navigation constant and fuzing delay” for a forward-hemisphere shot. The two values of N are not printed. The AIM-9D section of the same handbook does not change N; it drops the shooter’s g limit because the gimbal is 40°. — [NAVWEPS OP 3353](https://archive.org/stream/op-2309-2nd-sidewinder-guided-missle-mark-2/OP%203353%20AIM-9C%20&D%20PIlot%27s%20Handbook_djvu.txt)
- No opened source says high off-boresight infrared weapons should use an N outside 3–5.

### Inferences
- Keep the sim’s true-PN form. Default `N = 4` [Fleeman’s worked value, inside the published 3–5 band]. Allow 3, 4, or 5 per missile as a data field, not a derived “HOBS N.” The AIM-9C’s unpublished N switch is not a license to give an infrared missile two values of N. If a forward-hemisphere radar round is ever modeled, the switch exists and the numbers are a gap.
- AIM-9B steering lag, separate from N [published]:

```cpp
// tau = 0.25 s maximum, OP 2309. a_cmd is already capped.
a += (a_cmd - a) * (dt / 0.25);
```
- Heading error is the HOBS issue. A 90° off-boresight shot starts with a large `gamma_M`. The command `N * Vc * omega` saturates on `n_struct` (and on TVC plus aero) immediately. Raising N above 4 does not produce more acceleration once the g limit is active; it only increases the command before saturation and, in Siouris’s and Fleeman’s noise discussions, the glint contribution. Model the g limit. Do not invent N = 6 or N = 1.5 for a Fox 2. (The N ≈ 1.5 figure on the Wikipedia page is a predatory fly, not a missile.)
- Augmented proportional navigation (an extra term in target acceleration) is a published variant and was not required here. Classic infrared missiles did not measure target acceleration; they measured line-of-sight rate.
- MICA midcourse is not this law. Switch to N = 4 only after the seeker locks.

### Gaps
- A flight-manual number for N on any specific Fox 2 was not found. OP 3352 names N for the AIM-9D and does not evaluate it. 3–5 is the tactical-missile band, not a Sidewinder telemetry value. The AIM-9C forward-hemisphere N pair is also unpublished.
- The early Sidewinder’s seeker-to-canard gain is an implicit N that depends on speed. The manuals do not publish that gain as a number N.

## Representative F-16C launch platform

### Takeaway
Use a Block 50 F-16C: wing area 300 ft², F110-GE-129 at 17,155 lbf military and 29,500 lbf afterburner, +9 g, empty weight 18,900 lb, internal fuel 7,000 lb. Clean subsonic `CD0` is about 0.016–0.020 from open textbooks. Combat `CLmax` was not found; corner speed should be computed from the formula, not copied as a single published knot value. This aircraft is only the shooter.

### Cited Findings
- Wikipedia specifications for an F-16C Block 50/52, citing the USAF fact sheet, *International Directory of Military Aircraft*, and the Block 50/52+ flight manual: length 49 ft 5 in (15.06 m); wingspan 32 ft 8 in (9.96 m); wing area 300 ft² (listed as 28 m²); empty weight 18,900 lb (8,573 kg); gross weight 26,500 lb (12,020 kg); internal fuel 7,000 lb (3,200 kg); g limit +9.0. Block 50 engine General Electric F110-GE-129: 17,155 lbf (76.31 kN) dry, 29,500 lbf (131 kN) with afterburner. Block 52 alternative Pratt & Whitney F100-PW-229: 17,800 lbf dry, 29,160 lbf afterburning. A footnote gives thrust-to-weight 1.095, and 1.24 “with loaded weight and 50% internal fuel,” without defining the loaded weight on the page. — [F-16, English Wikipedia, specifications](https://en.wikipedia.org/wiki/General_Dynamics_F-16_Fighting_Falcon). The USAF fact-sheet URL itself returned “Access Denied” when fetched directly (`https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104505/f-16-fighting-falcon/`).
- GlobalSecurity’s F-16 table agrees on wing area 300 ft² / 27.87 m² and on F110-GE-129 afterburning thrust 29,500 lb, but lists empty weight about 20,300 lb in one column. That empty-weight conflict is noted and the USAF-cited 18,900 lb is the one used below. — [GlobalSecurity, F-16 specs](https://www.globalsecurity.org/military/systems/aircraft/f-16-specs.htm)
- F-16.net’s Block 50/52 card conflicts again: empty weight 18,238 lb, wingspan 31 ft 0 in, wing area still 300 ft², F110-GE-129 dry thrust 17,155 lbf and afterburning thrust 28,984 lbf, and a “normal loaded (air-to-air mission)” weight of 26,463 lb. That loaded weight is not defined as half internal fuel plus two wingtip missiles. — [F-16.net, Block 50/52](https://www.f-16.net/f-16_versions_article9.html)
- Shaw AFB public captions for an installed F110-GE-129 say the engine “produces approximately 29,000 pounds of thrust” and that a test-cell run is checked for a stable 29,000 lbf. Tinker AFB says the F110-129 “produces nearly 30,000 pounds of thrust.” These are rounded operational statements, not a rated-thrust table. — [Shaw AFB](https://www.shaw.af.mil/News/Photos/igphoto/2001708355); [Shaw AFB test cell](https://www.shaw.af.mil/DesktopModules/ArticleCS/Print.aspx?PortalId=98&ModuleId=74578&Article=1866151); [Tinker AFB](https://www.tinker.af.mil/News/Article-Display/Article/846020/f110-129-the-end-of-an-era/)
- Jane’s, as quoted in a textbook problem: wing area 27.87 m², “typical combat weight” 8,273 kgf, F110 sea-level thrust 131.6 kN. 8,273 kg is lighter than the USAF empty weight of 8,573 kg, so that “combat weight” was not used. Thrust 131.6 kN matches the Block 50 afterburning figure within rounding. — [problem statement quoting Jane’s](https://www.numerade.com/ask/question/7-the-lockheed-martin-f-16-shown-in-fig-656-in-the-textbook-is-in-a-vertical-accelerated-climb-some-characteristics-of-this-airplane-from-janes-all-the-world-aircraft-are-wing-area-2787-m2-t-97447/)
- Brandt, Stiles, Bertin, and Whitford, *Introduction to Aeronautics: A Design Perspective*, as exposed in an open document mirror: they take span `b = 30 ft`, aspect ratio 3, mean chord `30/3 = 10 ft`, wing area 300 ft². Their estimated subsonic `CD0` is 0.0169. Their Table 4.5, labeled actual F-16 drag polar, gives `CD0` of 0.0193 at Mach 0.3, 0.0202 at 0.86, 0.0444 at 1.05, 0.0448 at 1.5, and 0.0458 at 2.0. Usable `CLmax` for takeoff and landing is given as 1.2, because the landing gear limits the usable angle of attack to about 14°. That is a ground-geometry limit, not the combat angle-of-attack limit. — [document mirror of the textbook](https://vdoc.pub/documents/introduction-to-aeronautics-a-design-perspective-3pp640h9an3g)
- Bertin and Cummings, *Aerodynamics for Engineers*, as exposed in a document mirror: after a wetted-area build-up they cite Webb et al. (1977) that early F-16 flight-test subsonic `CD0`, corrected for engine effects and for missiles, “varied between `CD0 = 0.0160` and `CD0 = 0.0190`.” — [document mirror](https://dokumen.pub/aerodynamics-for-engineers-sixth-edition-9780132832885-0273793276-9780273793274-0132832887.html). The underlying paper is Buckner and Webb, “Selected results from the YF-16 wind tunnel test program,” AIAA 1974-619, which was not opened beyond the citation page (`https://arc.aiaa.org/doi/pdfplus/10.2514/6.1974-619`).
- AIM-9X launch weight 186 lb (84.37 kg) each. Two wingtip rounds are 372 lb [derived from the published unit weight]. — [NAVAIR AIM-9X](https://www.navair.navy.mil/product/AIM-9X-Sidewinder)
- Corner speed is the speed where the structural g limit and the aerodynamic g limit are equal: `n_max = q * CLmax * S / W`, so `V = sqrt(2 * n_max * W / (rho * S * CLmax))`. No opened source stated an F-16C corner speed in knots.

### Inferences
- Shooter to model, Block 50, quantities tagged:
  - Wing area `S = 300 ft² = 27.87 m²` [published].
  - Span for aerodynamics: 30 ft and aspect ratio 3, as in the Brandt textbook model [published in that textbook]. The 32 ft 8 in Wikipedia span includes the tip-rail geometry; using it with 300 ft² would give `AR = b²/S = 3.56` [derived] and would double-count rail span. Prefer AR = 3 and `b = 30 ft` for the lift and drag model.
  - Engine F110-GE-129: military 17,155 lbf (76.3 kN) [published, Wikipedia and F-16.net agree]. Afterburner 29,500 lbf (131 kN) is the USAF-sheet figure as cited by Wikipedia and by GlobalSecurity [published citation]. F-16.net prints 28,984 lbf, and Shaw’s captions round the installed engine to about 29,000 lbf. Code 29,500 lbf and keep the other two as the conflict, not as a blended number.
  - `n_max = +9` [published].
  - Clean subsonic `CD0 ≈ 0.016` to `0.019` [published flight-test band via Bertin/Webb]. The Brandt “actual” polar is the Mach table to use if one curve is stored: 0.019 at low Mach, about 0.020 at Mach 0.86, about 0.044 through the transonic and supersonic points listed above. Those coefficients are on wing area, not body area.
  - Combat `CLmax` is [unknown]. Do not use 1.2; that is the takeoff and landing value limited by a 14° tail-strike angle.
- Combat mass with half internal fuel and two wingtip AIM-9X, pilot mass stated as an estimate because the fact sheet’s empty weight excludes the pilot:

```text
W = 18900 lb empty
  + 0.5 * 7000 lb fuel
  + 2 * 186 lb missiles
  + 200 lb pilot          // [estimate], not in the fact sheet
  = 22972 lb = 10420 kg   // [derived]
```

If the Wikipedia thrust-to-weight note is read literally, `29500 / 1.24 ≈ 23800 lb` is some “loaded weight” at 50% internal fuel [derived]. What that load includes is not stated. The component sum above is the one to code, with the pilot called out as an estimate. Ammo and pylons beyond the two wingtip rails are not included.
- Corner speed, sea level, `rho = 1.225 kg/m³`, `n = 9`, `W = 22972 lbf`, `S = 300 ft²`, left in terms of `CLmax`:

```cpp
// W in newtons, S in m^2: 22972 lbf = 102.2 kN; S = 27.87 m^2
double V_corner = std::sqrt(2.0 * 9.0 * (102.2e3) / (1.225 * 27.87 * CLmax));
// CLmax = 1.2 (takeoff, NOT combat) -> about 211 m/s, 410 kt  [derived, wrong CLmax]
// Combat CLmax unknown -> do not publish a knot number as a fact
```

- This mass, wing, and thrust are the fighter. They do not enter the missile `CD0` or the missile seeker.

### Gaps
- Combat `CLmax` with leading-edge flaps at the angle-of-attack limiter is [unknown] in the sources opened. Corner speed in knots is therefore [unknown].
- The 200 lb pilot is an estimate. Gun ammunition and internal equipment above empty weight are not in the sum.
- USAF fact sheet could not be fetched (access denied). Numbers are Wikipedia’s citation of that sheet, cross-checked against GlobalSecurity on wing area and F110 thrust, not against the PDF. Empty weight is 18,900 lb (Wikipedia), 18,238 lb (F-16.net), or about 20,300 lb (GlobalSecurity). Afterburning thrust is 29,500 lbf, 28,984 lbf, or “about 29,000 lbf,” depending on the source. The combat-mass sum below follows the USAF-cited empty weight and does not adopt F-16.net’s 26,463 lb air-to-air weight, because that figure is a different, unspecified load.
- Clean `CD0` after the two wingtip missiles are hung is [unknown]. Webb’s band had missiles corrected out. Brandt’s “actual” polar does not say the store fit.

## Typical employment numbers for the fighter start

### Takeaway
The only quantified “typical” Sidewinder shots in an opened manual are Mach 1.2 at sea level and Mach 1.2 at 50,000 ft, in both cases against a Mach 0.9 target. Use those as the default calibrated shots. A separate “visual merge at 15,000 ft” altitude was not found and should not be invented.

### Cited Findings
- AIM-9B manual, “missile performance under typical conditions”: range 28,000 ft at 50,000 ft altitude, launch Mach 1.2, target Mach 0.9; range 6,000 ft at sea level, launch Mach 1.2, target Mach 0.9. — [NAVWEPS OP 2309](https://archive.org/details/OP23093rdAIM9B)
- The same manual’s firing-envelope rule of thumb: the pilot should be able to track the target with less than 1.6 g at 40,000 ft and above, or less than 2 g below 40,000 ft. That is a pursuit-geometry limit (the famous 2 g rule), not a preferred cruise altitude. — [NAVWEPS OP 2309](https://archive.org/details/OP23093rdAIM9B)
- No tactics manual, Red Flag note, or later Sidewinder flight manual opened here states a single “normal” employment altitude and Mach for a modern AIM-9 shot. The 50,000 ft case is a performance case in a 1960s manual, not a claim about where most visual fights happen.

### Inferences
- Default fighter start for a Sidewinder acceptance shot: sea level, shooter Mach 1.2, co-altitude target at Mach 0.9, tail aspect. Expected AIM-9B-class kinematic range about 6,000 ft (1.8 km) once `CD0` is calibrated. Second acceptance shot: 50,000 ft (15,240 m), same Mach pair, expected range about 28,000 ft (8.5 km).
- Those two points also check the altitude-thrust model, because the same motor has to produce both ranges with one `CD0` scale factor.
- The 2 g rule can be a firing interlock on a classic AIM-9B: if the fighter’s own lead pursuit would take more than 2 g below 40,000 ft, the shot is outside the envelope the manual taught. It is not a guidance law inside the missile.
- A player spawn that is subsonic and at medium altitude is a scenario choice. It is not a published “normal Sidewinder shot,” and ranges from it should not be used to set `CD0`.

### Gaps
- A post-1970 employment manual stating a preferred altitude and Mach for the AIM-9L/M/X was not found.
- The manual’s “typical” word applies to the two performance cases above. It does not say those were the statistical mode of combat shots.
