# Aircraft RCS, textbook jamming, and data-driven sensor architecture

Sources below were opened on 2026-09-29. Knott, Shaeffer, and Tuley’s *Radar Cross Section* was not opened (no public full text in this pass). Schleher and Neri were not opened. The J/S equations quoted here are the ones printed in the public NAWCWD TP 8347 handbook PDF, not a reconstruction from those books. Falcon BMS and DCS sensor manuals were not opened; nothing below is taken from them.

## Why one square-metre number is not an aircraft RCS model

### Takeaway
RCS is a far-field property of the target at a stated aspect, frequency, and polarization. A single square-metre value is only a mean, and aircraft-like targets also scintillate in time, which Swerling modeled as a chi-squared draw that is either steady through a scan or independent from pulse to pulse.

### Cited Findings
- RCS is defined so that a target intercepting power density \(S_i\) and returning scattered density \(S_s\) at range \(r\) has \(\sigma = \lim_{r \to \infty} 4\pi r^2 S_s / S_i\). The same page writes the monostatic radar equation \(P_r = P_t G_t \sigma A_{\mathrm{eff}} / \bigl((4\pi r^2)^2\bigr)\). The limit is the far-field definition a real-time sim is allowed to use. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- The same article lists the factors that change \(\sigma\): material, size relative to wavelength, absolute size, incident angle, reflected angle, and the polarization of the transmitted and received fields relative to the target. Strength of the emitter and range are explicitly not part of \(\sigma\). — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Orientation is not a small correction. The article shows a polar plot of RCS versus angle for an A-26 Invader and says that, all else equal, a fighter presents a much larger area from the side than from the front, so the return is stronger from the side. That plot is one angular cut, not a single number. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Frequency dependence of a canonical shape is large. For a metal sphere, RCS is approximately proportional to \(f^4\) when the circumference is less than three-quarters of a wavelength, and approximately equal to the physical cross section when the circumference is greater than about 20 wavelengths (Mie scattering in between). A perfectly conducting sphere whose projected area is \(1\,\mathrm{m}^2\) (diameter about \(1.13\,\mathrm{m}\)) has RCS \(1\,\mathrm{m}^2\) in that optical region, independent of frequency. A square flat plate of area \(1\,\mathrm{m}^2\) has \(\sigma = 4\pi A^2 / \lambda^2\), which the article evaluates as \(139.62\,\mathrm{m}^2\) at \(1\,\mathrm{GHz}\) when the plate is broadside; off-normal incidence reflects energy away from the receiver and the RCS falls. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Unclassified typical values the same article prints for a centimetre-wave radar: insect \(0.00001\,\mathrm{m}^2\), bird \(0.01\,\mathrm{m}^2\), human \(1\,\mathrm{m}^2\), small combat aircraft \(2\)–\(3\,\mathrm{m}^2\), large combat aircraft \(5\)–\(6\,\mathrm{m}^2\), cargo aircraft up to \(100\,\mathrm{m}^2\), surface-to-air missile about \(0.1\,\mathrm{m}^2\), coastal trading vessel (\(55\,\mathrm{m}\)) \(300\)–\(4000\,\mathrm{m}^2\), corner reflector with \(1.5\,\mathrm{m}\) edges about \(20{,}000\,\mathrm{m}^2\). The same page says RCS data for current military aircraft is mostly highly classified. Those class-typical numbers are not a measured pattern for a named fighter, and the stealth examples printed beside them are not used here. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Statistical models listed for Monte Carlo use, given an average RCS, are chi-square, Rice, and log-normal. The page does not give the log-normal standard deviation. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Amplitude scintillation is the Swerling model. The general density is
  \[
  p(\sigma)=\frac{m}{\Gamma(m)\,\sigma_{\mathrm{av}}}\left(\frac{m\sigma}{\sigma_{\mathrm{av}}}\right)^{m-1}\exp\left(-\frac{m\sigma}{\sigma_{\mathrm{av}}}\right),\quad \sigma\ge 0.
  \]
  The ratio of standard deviation to mean is \(m^{-1/2}\). \(m\to\infty\) is non-fluctuating. Values of \(m\) between \(0.3\) and \(2\) are said to approximate some simple shapes such as cylinders or cylinders with fins. — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)
- Swerling I: chi-squared with two degrees of freedom (\(m=1\)), many independent scatterers of roughly equal area (as few as half a dozen). PDF \(p(\sigma)=(1/\sigma_{\mathrm{av}})\exp(-\sigma/\sigma_{\mathrm{av}})\). The draw is constant through one scan and independent from scan to scan. The article calls this case particularly useful for aircraft shapes. Swerling II uses the same PDF but redraws from pulse to pulse (locked fire-control or a very fast target). — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)
- Swerling III: four degrees of freedom (\(m=2\)), one large scatterer plus several smaller ones. PDF \(p(\sigma)=(4\sigma/\sigma_{\mathrm{av}}^2)\exp(-2\sigma/\sigma_{\mathrm{av}})\), constant through a scan. Helicopters and propeller aircraft are the article’s examples, because the rotor is a strong return. Swerling IV is the same PDF, pulse to pulse. Swerling V, also called Swerling 0, is constant RCS. — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)
- The physical cause given for the fluctuation is interference among returns from separated points on the target as aspect changes. Two frequencies do not share the same nulls, which is why frequency agility reduces fluctuation loss. That is an amplitude effect in time and aspect, not a substitute for the mean pattern. — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)
- Polarization is measurable on a canonical target. For a half-wave dipole, maximum monostatic RCS when the incident E-field is parallel to the wire and to the receive polarization is given as \(\sigma_{\max}\approx 0.86\,\lambda^2\). A FEKO run in the same thesis produced a normalized peak of \(0.856\,\lambda^2\) against that \(0.86\,\lambda^2\) analytical value; a later mesh-converged peak was about \(0.0085\,\mathrm{m}^2\) at \(3\,\mathrm{GHz}\), \(1.2\%\) under \(0.86\lambda^2=0.0086\,\mathrm{m}^2\). Cross-polarized monostatic RCS is the minimum of that angular cut. — [Stellenbosch University thesis PDF, eq. (3.1)](https://scholar.sun.ac.za/server/api/core/bitstreams/0895adb7-d2ca-4931-9ac0-847db0b895bc/content)

### Inferences
- The data object is a mean \(\sigma_{\mathrm{av}}(\theta,\phi,f,p)\) in the target body frame, plus a Swerling case and a dwell time. \(\theta\) and \(\phi\) are both required: the A-26 figure is one cut of a function the article already says depends on orientation. A log-normal draw is a legitimate alternative fluctuation model only as a named distribution around that mean; no opened source gave its decibel width.
- Store the table per radar band, not per Hertz. The sphere curve is smooth only in the optical plateau; in the resonance region a single band-centre sample can sit on a lobe. Unresolved lobe structure inside a band is what the Swerling draw is for.
- No opened source specified a file format or an angular step in degrees. A practical encoding consistent with the variables above is a regular body-frame grid of \(\sigma\) in dBsm, one array per band and per stored polarization (at least co-pol; cross-pol if the dipole test is to be representable), with the Swerling case and the correlation rule as header fields. Grid spacing finer than the scintillation lobes is a precomputation; the real-time step only interpolates.

### Gaps
- Knott, Shaeffer, and Tuley was not opened, so this note does not quote their sampling rule or their aircraft-pattern figures.
- No opened source gave a standard angular step (for example \(0.1^\circ\) versus \(1^\circ\)) or a published aircraft-pattern file schema.
- The log-normal \(\sigma\) in decibels was not in the opened RCS article.
- Numeric fluctuation loss in decibels (how many dB of extra SNR a Swerling I target needs at a stated \(P_d\)) was not taken from an opened page. Secondary summaries disagree about which case is “one dominant scatterer”; the case assignment above follows the opened Wikipedia article, which cites Swerling’s 1954 RAND paper and Skolnik. The 1954 PDF itself was not opened.
- Named operational-aircraft RCS figures, including stealth examples printed on the Wikipedia page, are omitted on purpose.

## Glint and scintillation as angle error

### Takeaway
Scintillation is the fluctuation of echo amplitude. Angular glint is a separate error on the monopulse angle: an empirical rms of about \(0.35\,L/R\) radians, so the angle noise grows as the target gets closer, while thermal-noise angle error dominates at long range.

### Cited Findings
- Naval Postgraduate School monopulse notes give the empirical rms angle error from target glint as \(\sigma_{\theta g}\approx 0.7\tan(L/(2R))\approx 0.35\,L/R\), where \(L\) is the target extent (length or wingspan) and \(R\) is range, in the same units so the result is radians. The notes’ error-budget figure labels glint, thermal noise, and a range-independent servo/antenna floor, with glint the short-range term. — [Jenn, NPS EC4610 notes, Vol. II](http://www.dcjenn.com/EC4610/VolIIv6.0.pdf)
- The same notes describe glint as a source of monopulse tracking error distinct from thermal noise and from antenna errors. The extracted text ties the formula to target extent and range. — [Jenn, NPS EC4610 notes, Vol. II](http://www.dcjenn.com/EC4610/VolIIv6.0.pdf)
- Amplitude fluctuation is the Swerling model in the previous section, caused by interference among scatterers as aspect changes. It is applied to \(\sigma\), and therefore to SNR, not added as degrees. — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)

### Inferences
- A sim should add a zero-mean angle error to the monopulse (or seeker) angle measurement, with standard deviation \(0.35\,L/R\) while the target is treated as one unresolved body. It should not be implemented as a fixed angular bias or as a shorter lock range. \(L\) is a target data field (wingspan or length), not a radar constant.
- Because \(\sigma_{\theta g}\) scales as \(1/R\), the angle variance to put on a measurement covariance is \((0.35\,L/R)^2\) in each plane for which an extent \(L\) is defined. Thermal angle noise is the other diagonal term and shrinks as SNR rises; the opened notes show that split graphically but the Skolnik (4.3) expression was not re-extracted cleanly enough to quote.
- Swerling and glint come from the same multi-scatterer interference, so a later model can enlarge the angle draw when the realized \(\sigma\) is in a fade. That correlation was not given as a formula on an opened page, so the first implementation should keep them independent: Swerling on amplitude, Jenn’s rms on angle.

### Gaps
- The classical two-point glint function (error largest when the two echoes cancel, and able to point outside the physical span) was not re-opened as a quotable formula. Do not code a two-scatterer singularity from memory.
- Glint bandwidth (how fast to redraw the angle noise) was not on the opened Jenn extract. Do not invent a \(1\,\mathrm{Hz}\) correlation time.
- The notes’ thermal-noise formula was not recovered as a clean equation in this pass.

## Noise jamming, J/S, and burn-through

### Takeaway
For the handbook’s constant-power (saturated) jammer, monostatic J/S grows with the square of the single radar-to-target range and does not contain frequency. Burn-through is the range at which that ratio falls to the smallest J/S the jammer still needs, which the handbook’s worked figure places outside the range where J merely equals S. Self-protection and escort-with-the-target are the same geometry: one range. A jammer that is not on the target is a different range problem, not a different procedure.

### Cited Findings
- The handbook derives J from the one-way range equation and S from the two-way range equation. It splits active electronic attack into concealment, which it calls noise jamming, and deception, which it defines as forging false target signals the receiver accepts as real. “J” is the attack-signal strength in either case. — [NAWCWD TP 8347 PDF, section 4-7](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Constant-power (saturated) monostatic J/S, ratio form, as printed:
  \[
  J/S = (P_j G_{ja}\, 4\pi R^2) / (P_t G_t \sigma)
  \]
  with \(R\) and \(\sigma\) in the same units. In decibels, for the unit system that produces the constant \(10.99\,\mathrm{dB}=10\log_{10}(4\pi)\):
  \[
  10\log(J/S)=10\log P_j+10\log G_{ja}-10\log P_t-10\log G_t-10\log\sigma+10.99\,\mathrm{dB}+20\log R.
  \]
  The handbook’s note on this section: neither \(f\) nor \(\lambda\) appears. This is the equation to quote for saturated noise jamming. It has jammer power and jammer antenna gain. It does not have jammer bandwidth or a radar processing-gain symbol. — [NAWCWD TP 8347 PDF, section 4-7](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- J/S in that section is in dB at the receiver input. J usually must exceed S by some amount. Burn-through range is where the skin return is first detected through the jamming. It is usually a little farther out than crossover, the range where \(J=S\). It is the range where J/S equals the minimum effective J/S. — [NAWCWD TP 8347 PDF, sections 4-7 and 4-8](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Monostatic crossover, \(J=S\):
  \[
  R_{co}=\bigl[(P_t G_t \sigma)/(P_j G_{ja}\,4\pi)\bigr]^{1/2}.
  \]
  Monostatic burn-through, using the required ratio \(J/S\):
  \[
  R_{BT}=\bigl[(P_t G_t \sigma)/(P_j G_{ja}\,4\pi)\cdot(J/S)\bigr]^{1/2},
  \]
  or \(20\log R_{BT}=10\log P_t+10\log G_t+10\log\sigma-10\log P_j-10\log G_{ja}+10\log(J/S)-10.99\). The handbook’s worked plot (one numerical example, not a universal range) has crossover near \(1.29\,\mathrm{NM}\), a required J/S of \(6\,\mathrm{dB}\), and burn-through near \(2.8\,\mathrm{NM}\). Inside crossover, S exceeds J. The \(6\,\mathrm{dB}\) line is crossed farther out, so the radar is already inside the effective-jamming region before J falls all the way to S. — [NAWCWD TP 8347 PDF, section 4-8](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Why the slopes differ is stated with the plot: the jammer line is \(20\,\mathrm{dB}\) per decade of range and the skin return is \(40\,\mathrm{dB}\) per decade, i.e. \(J\propto 1/R^2\) and \(S\propto 1/R^4\) when jammer and target share one range, so \(J/S\propto R^2\). — [NAWCWD TP 8347 PDF, section 4-8](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Self-protection EA is the attack carried on the platform it protects (also called self-screening). Escort is a special case of support jamming. When the jammer is with the target, the J-to-S calculation is the same as self-protection. That is a statement about geometry (one range versus two), not an employment procedure. — [NAWCWD TP 8347 PDF, section 4-7](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- One-way free-space power density from an isotropic antenna falls as \(1/R^2\) because it is spread over the sphere \(4\pi R^2\). The two-way skin return picks up another \(1/R^2\). — [NAWCWD TP 8347 PDF, one-way range equation](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)

### Inferences
- In the sim, saturated self-protection is a jammer component on the same platform as the scattering target: one \(R\), the equation above, and a data field \((J/S)_{\min}\) that plays the role of the handbook’s “minimum effective J/S”. The \(6\,\mathrm{dB}\) figure is an example from one plot, not a default to hard-code for every radar.
- Processing gain and jammer bandwidth did not appear in the saturated equation. The handbook folds receiver behaviour into \((J/S)_{\min}\). Until section 4-10 is transcribed cleanly, do not insert a \(B_r/B_j\) factor or a processing-gain multiplier into this formula.
- A support jammer that is not collocated with the target must not reuse the monostatic equation with the target range silently standing in for both ranges. The PDF text extract of the bistatic summary line was too garbled to quote a two-range formula. Leave stand-off as a separate range pair in the data model, and do not invent the algebra.

### Gaps
- Section 4-10, constant-gain (linear) jamming, is in the handbook table of contents. This pass did not recover a clean equation containing jammer bandwidth or radar processing gain. Schleher and Neri were not opened, so their forms are not quoted.
- No opened page gave a universal required J/S. It is an input, as in the handbook’s \(6\,\mathrm{dB}\) example.
- Atmospheric loss, polarization mismatch, and antenna sidelobe gain toward a stand-off jammer are named as omissions or separate sections in the handbook extracts, and no combined equation was transcribed.

## Deception as an extra measurement, not a circuit

### Takeaway
A false target is another detection the tracker accepts. Range-gate pull-off walks that detection’s delay, so the gated range moves off the skin range. Velocity-gate pull-off walks the Doppler instead. The tracker’s geometry is two measurements, one of which is being moved; this note does not repeat the handbook’s jammer procedure.

### Cited Findings
- The handbook’s definition: deception forges false target signals that the radar receiver accepts and processes as real targets. — [NAWCWD TP 8347 PDF, section 4-7](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- A range gate “select[s] radar echoes from a very short range interval.” A range cell is the smallest range increment the radar can detect. The glossary example: \(50\,\mathrm{yd}\) resolution over \(30\,\mathrm{NM}\) is \(1{,}200\) range cells. Range rate is the rate of change of radar range, equal to target velocity only when the target flies straight toward or away from the radar. — [NAWCWD TP 8347 PDF, glossary](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Glossary definition of range-gate pull-off, which is the measurement geometry and not a build recipe: the jammer initially repeats the skin echo with minimum time delay; the delay is then progressively increased, so the tracking gates are pulled (“walked”) off the target echo. Range-gate walk-off is defined as the same term. The techniques section states that this kind of false target appears at a greater range than the real target because the deceptive signal is delayed. — [NAWCWD TP 8347 PDF, glossary and section 4-13](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Velocity-gate pull-off is described as the Doppler analogue: capture the velocity gate and move it away from the skin echo by shifting the retransmitted frequency, which the tracker reads as a changed Doppler. Velocity-gate walk-off is listed as its own glossary entry. Velocity false targets are a different, non-walking case: false Doppler detections that dwell in a filter long enough to be declared and then jump. — [NAWCWD TP 8347 PDF, section 4-13 and glossary](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Section 4-13 also spells out a phased procedure against a tracking radar (when to retransmit, how the receiver’s sensitivity is driven, when to stop). That procedure is not reproduced here. The sentences cited above are the part a tracker model needs. — [NAWCWD TP 8347 PDF, section 4-13](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)

### Inferences
- Represent deception as one more contact in the same dwell, with its own delay \(\tau\) and Doppler \(f_d\). Skin range is \(c\tau_{\mathrm{skin}}/2\) for a monostatic echo. A range-gate pull-off contact has \(\tau(t)=\tau_{\mathrm{skin}}+\Delta\tau(t)\) with \(\Delta\tau\) starting near zero and increasing, so measured range walks outward from the skin range. A velocity-gate pull-off contact holds a delay near the skin delay and walks \(f_d(t)\) away from \(2v_r/\lambda\). When the false contact is removed, the gate that followed it has no skin echo left in it.
- The sim does not need a repeater waveform, a frequency-memory loop, or a gain schedule. \(\Delta\tau(t)\) and \(\Delta f_d(t)\) are data on the false contact. Association and which gate the tracker follows are tracker logic, downstream of the contact list.
- “Closer-range” false targets are mentioned in the handbook only as a consequence of predicting the next pulse. They are still just contacts with \(\Delta\tau<0\). No timing circuit is required in the data model.

### Gaps
- No opened page gave a standard walk rate in metres per second or hertz per second. Those rates belong in data, not in code constants copied from an example.
- Angle deception and track-while-scan “walking” are named in section 4-13. Their measurement geometry was not extracted cleanly and is out of scope here.
- Nothing in this section is a method for defeating a named radar or missile.

## What open sims compute per frame, per dwell, and ahead of time

### Takeaway
JSBSim keeps truth in a property tree and degrades a sensor by XML-specified noise, lag, bias, drift, delay, and quantization once per flight-dynamics frame. That is the right split for air-data and inertial sensors. A radar still needs a slower dwell clock, because Swerling’s scan-to-scan case is not “once per flight step,” and full-wave electromagnetics is not a frame job.

### Cited Findings
- JSBSim’s `FGSensor` is an XML block: an input property, then optional lag, noise (`PERCENT` or `ABSOLUTE`, `UNIFORM` or `GAUSSIAN`), quantization (bits, min, max), drift rate, gain, bias, and delay in time or in frames. With only an input, the output is the input. Noise is redrawn every frame of the simulation. Gaussian noise is specified as a span of about six sigma (\(-3\) to \(+3\) times the configured magnitude). The lag coefficients are computed from the component `dt`, so the sensor is tied to the flight-dynamics step, not to a separate radar scheduler. — [JSBSim FGSensor class reference](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGSensor.html)
- The property tree that those sensors read is the program’s state interface: hierarchical names, properties created as configuration files are read, and the same names used from files, scripts, and code. Standard properties exist for every vehicle; aerodynamic coefficients and engines add properties only after that vehicle’s file is loaded. — [JSBSim reference manual, Properties](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/properties/) (page text as returned by search; not re-fetched in full)
- missilesim’s configured flight step defaults to \(0.01\,\mathrm{s}\) (\(100\,\mathrm{Hz}\)), inside the \(60\)–\(120\,\mathrm{Hz}\) band. — [SimulationConfig.h](C:/Users/alede/Documents/code/missilesim/src/sim/SimulationConfig.h)
- missilesim already evaluates closed-form atmosphere and aero on that step rather than storing a giant table: `Atmosphere::sample` returns an ISA-style state (temperature, pressure, density, speed of sound, viscosity) from altitude, and Fox 2 zero-lift drag is the Fleeman body build-up in `Fox2Flight.h`, called from the missile, not a thrust-versus-time table. Catalog thrust is a single average when a curve was not published. — [Atmosphere.h](C:/Users/alede/Documents/code/missilesim/src/physics/Atmosphere.h), [Fox2Flight.h](C:/Users/alede/Documents/code/missilesim/src/sim/Fox2Flight.h)
- The current infrared seeker already separates a per-step measurement from guidance. `PhysicsEngine::update` calls `Missile::updateHeatSeeker` on every physics step. `MissileFox2.cpp` builds a `SourceSignal` with range, aspect-weighted intensity, and `irradiance = intensity / range^2`, then applies an optional acquisition gate. — [PhysicsEngine.cpp](C:/Users/alede/Documents/code/missilesim/src/physics/PhysicsEngine.cpp), [MissileFox2.cpp](C:/Users/alede/Documents/code/missilesim/src/objects/MissileFox2.cpp)
- Wikipedia’s RCS article treats computational electromagnetics as an offline prediction on large computers and CAD models, and treats chi-square, Rice, and log-normal draws as what Monte Carlo runs use once an average RCS exists. That is the pre-tabulate versus per-dwell split. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)

### Inferences
- Copy JSBSim’s split, not its sensor list. Truth (position, velocity, attitude, true aspect) updates at the flight step. A radar contact is produced on a dwell, which may span many flight steps. Swerling I/III hold the realized \(\sigma\) for that dwell; Swerling II/IV redraw inside it. Air-data-style noise (JSBSim) stays on the flight step because those sensors have no dwell.
- Pre-tabulate or pre-derive anything that is a property of the body or the hardware: RCS grids, antenna gain versus angle, the Fleeman-style drag evaluation, catalog thrust. Sample atmosphere and RCS at run time. Do not pre-tabulate SNR; it depends on the instantaneous range.
- Per dwell, not per flight frame: transmitted waveform parameters, which targets fall in the beam, one \(\sigma\) lookup plus fluctuation, one noise J and any false contacts, SNR, \(P_d\), and a measurement with covariance. Per flight frame: propagate the platforms, and propagate any track filter that is integrating measurements it already has.
- Falcon BMS and DCS are not used as evidence. No public sensor-data manual for either was opened, so any claim about their lock-range tables would be hearsay. Label them as games if they are consulted later; do not treat an encyclopedia detection range as the radar equation.

### Gaps
- JSBSim’s default \(120\,\mathrm{Hz}\) rate is widely repeated and was not confirmed from an opened page. Only the per-frame sensor contract and the use of `dt` were confirmed. missilesim’s own default is \(100\,\mathrm{Hz}\).
- IEEE real-time radar-simulation papers turned up as abstracts in search (point-target IQ generation, scattering-centre RCS versus angle, precomputed coherent RCS versus frequency and angle). The papers and the associated patent were not opened, so they are not cited as findings.
- The MATLAB RCS modelling page returned “Access Denied” and is not cited.
- No opened sim manual stated “one clutter cell per dwell” as an implemented architecture. That approximation is justified separately from the NRCS definition, in the numerical-budget section.

## A contact schema mapped onto the files that exist

### Takeaway
missilesim already has platforms, an atmosphere, a per-step infrared irradiance, and a Fox 2 card. It does not have an emitter, a dwell, a contact with SNR and covariance, or a track. Those types belong beside `src/sim`, which already owns data-driven weapon cards, and they should feed `Missile` instead of `Missile` reaching into raw `Target*` lists.

### Cited Findings
- Player motion is the F-16 in `src/flight` (`Jet`, `F16Airframe`, `F16Data`). `Fighter` mirrors that state into a `PhysicsObject` for the renderer, HUD, and audio, and the class comment says Application integrates it, not `PhysicsEngine::m_objects`. — [Fighter.h](C:/Users/alede/Documents/code/missilesim/src/objects/Fighter.h)
- Targets, flares, and missiles are `PhysicsObject`s owned by `PhysicsEngine`, which also owns `Atmosphere`, drag, lift, and gravity, and which calls the heat seeker every step. — [PhysicsEngine.h](C:/Users/alede/Documents/code/missilesim/src/physics/PhysicsEngine.h)
- Weapon behaviour that is data rather than code already lives in `src/sim`: `Fox2Catalog::Spec` holds mass, motor, gimbal, IFOV, track rate, aspect kind, and an optional `tailAcquisitionM`. `Fox2Flight.h` turns aspect into a dimensionless intensity. `SimulationConfig` loads the environment step, missile guidance gains, and flare heat. — [Fox2Catalog.h](C:/Users/alede/Documents/code/missilesim/src/sim/Fox2Catalog.h), [Fox2Flight.h](C:/Users/alede/Documents/code/missilesim/src/sim/Fox2Flight.h), [SimulationConfig.h](C:/Users/alede/Documents/code/missilesim/src/sim/SimulationConfig.h)
- Today’s seeker measurement, in `measureTarget`, is `intensity = heatSignature * seekerIntensity(...)` and `irradiance = intensity / range^2`. The target is visible if intensity is positive and, when `tailAcquisitionM > 0`, range is at most `tailAcquisitionM * sqrt(intensity)`. That inequality is an irradiance threshold: \(\mathrm{intensity}/R^2 \ge 1/R_{\mathrm{tail}}^2\). If `tailAcquisitionM` is \(0\), any positive intensity is visible at any range. — [MissileFox2.cpp](C:/Users/alede/Documents/code/missilesim/src/objects/MissileFox2.cpp), [Fox2Catalog.h](C:/Users/alede/Documents/code/missilesim/src/sim/Fox2Catalog.h)
- The missile still tracks a `Target*` plus a position, with lock, memory, and a scalar `countermeasureResistance`. Flares are physics objects with a scalar heat signature and a decay rate, not radar false contacts. A target MAWS block has a fixed `detectionRange` of \(3200\,\mathrm{m}\). — [Missile.h](C:/Users/alede/Documents/code/missilesim/src/objects/Missile.h), [Flare.h](C:/Users/alede/Documents/code/missilesim/src/objects/Flare.h), [Target.h](C:/Users/alede/Documents/code/missilesim/src/objects/Target.h)

### Inferences
Proposed types, and the existing owner of each input:

| Type | Holds | Owner of the truth it reads | Does not own |
| --- | --- | --- | --- |
| Platform | Pose, velocity, body axes, id | `Fighter` / `src/flight` for the player; `PhysicsEngine` for `Target`, `Missile`, `Flare` | Detections |
| Emitter | Power, gain, frequency or band, bandwidth, beam angles, mounted on a platform | New data next to `Fox2Catalog` / `SimulationConfig` (`src/sim`) | Who is detected this frame |
| RCS pattern | \(\sigma_{\mathrm{av}}(\theta,\phi)\) per band and polarization, Swerling case, extent \(L\) | Data file loaded from `src/sim`; sampled using platform attitude | SNR |
| Dwell | Time, beam direction, frequency, lists of skin contacts and false contacts | New scheduler in `src/sim`, called from the same place `PhysicsEngine::update` already calls the seeker, but on the dwell period | Graphics |
| Contact | Platform id, true range and angles, measured range and angles, realized \(\sigma\), SNR, \(P_d\), measurement covariance, delay, Doppler, false-target flag | Produced by the dwell from the radar equation, the RCS table, J/S, and glint | Track identity over time |
| Track | Associated contacts, filtered state, covariance | New filter in `src/sim` | Aerodynamics |
| Seeker | IFOV, gimbal, track rate, irradiance or SNR threshold, which track or which raw contact it is allowed to see | `Missile` plus the existing `Fox2Catalog::Spec` fields | The radar’s search dwell |

- A radar warning or a semi-active missile should consume contacts or tracks. The Fox 2 seeker should keep consuming irradiance, which `measureTarget` already computes, times the existing aspect function. It should not grow a second, unrelated lock-range constant.
- `countermeasureResistance` and a fixed MAWS range are the features a contact list replaces: a flare competes as another irradiance, and a radar decoy competes as another contact. Leave those fields until the contact path exists; do not add a third probability.
- Rendering under `src/rendering` is not on this path.

### Gaps
- There is no detection-probability function in the opened material (no Albersheim approximation, no Swerling \(P_d\) curves). The contact can carry SNR and \(P_d\), but the map from SNR to \(P_d\) is still an open formula choice.
- No clutter or terrain elevation model was read in `src/physics` beyond a flat ground level on `PhysicsEngine`. A ground platform for multipath can use `m_groundLevel`; a clutter cell needs an area the current ground does not provide.
- `MissileFox2.cpp` was only read around `measureTarget`. Other lock paths (heat-seeker versus Fox 2, memory track) were not fully traced.

## What a 100 Hz step can still get physically right

### Takeaway
The flight step can evaluate the far-field radar equation, interpolate an RCS table, draw one Swerling value per dwell, apply one two-ray factor, and treat infrared sources as points with inverse-square irradiance. It cannot run a fresh electromagnetic solution. Fixed lock ranges and fixed flare probabilities throw away the equations the rest of the model just computed.

### Cited Findings
- The far-field RCS limit and the \(1/R^4\) monostatic equation are the closed-form model Wikipedia contrasts with supercomputer CEM. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Normalized RCS is \(\sigma^0=\langle\sigma/A\rangle\), an average cross section per unit ground area. One number per resolution cell is that definition, not a map of every scatterer in the cell. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Point-source irradiance falls as \(1/R^2\): \(I=P/(4\pi R^2)\) for an isotropic source, and doubling the distance divides the intensity by four. Radar pays the inverse square on the way out and again on the way back, so echo power falls as \(1/R^4\). The point-source rule is a good approximation when the source size is under about one-fifth of the range (error under about \(1\%\)). — [Inverse-square law, Wikipedia](https://en.wikipedia.org/wiki/Inverse-square_law)
- missilesim’s flight step is \(0.01\,\mathrm{s}\) by default, and the Fox 2 measurement already implements \(I/R^2\). — [SimulationConfig.h](C:/Users/alede/Documents/code/missilesim/src/sim/SimulationConfig.h), [MissileFox2.cpp](C:/Users/alede/Documents/code/missilesim/src/objects/MissileFox2.cpp)
- Two-ray one-way power, large distance, reflection coefficient about \(-1\):
  \[
  \Delta\phi\approx\frac{4\pi h_t h_r}{\lambda d},\qquad
  P_r\approx P_t\left(\frac{\lambda\sqrt{G}}{4\pi d}\right)^2\bigl|1-e^{-j\Delta\phi}\bigr|^2,
  \]
  and farther out \(P_r\approx P_t G h_t^2 h_r^2/d^4\). The critical distance used by setting \(\Delta\phi=\pi\) is \(d_c=4 h_t h_r/\lambda\). — [Two-ray ground-reflection model, Wikipedia](https://en.wikipedia.org/wiki/Two-ray_ground-reflection_model)
- Swerling I is constant for a scan and independent across scans; it is not a new field solution every pulse. — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)

### Inferences
- Keep these, they stay physical at \(100\,\mathrm{Hz}\): far-field radar equation; RCS by table lookup in aspect, band, and polarization; one Swerling draw per dwell (or per pulse for cases II and IV); one clutter return per dwell equal to \(\sigma^0\) times the cell area; point-source infrared irradiance, which the Fox 2 code already forms; one two-ray factor from the closed form above, not a ray trace. For a monostatic radar the path is used twice, so echo power scales with the square of the one-way power factor. That square is an inference from “out and back,” not a formula printed on the two-ray page, which is a one-way communications model.
- These reintroduce hard-coding and should not be the detection law once the equation exists: a lock range that ignores \(\sigma\) and \(R\) (MAWS `detectionRange` is one; `tailAcquisitionM == 0` disables even the irradiance gate); a single \(\sigma\) for every aspect; a flare capture probability or `countermeasureResistance` that does not compare irradiances and angles; a burn-through range typed in as kilometres instead of evaluated from J/S.
- `tailAcquisitionM * sqrt(intensity)` is not quite a fixed range: it matches a fixed irradiance threshold. The failure mode is the zero sentinel, which locks at any range, and the fact that heat signature is a dimensionless catalog weight rather than watts per steradian.

### Gaps
- No opened source quantified the flight-step cost of a particular RCS grid. The constraint used here is the qualitative one from the RCS article: CEM is an offline tool.
- Multipath over spherical earth, ducting, and clutter spectra were not opened. The two-ray test below is the flat-earth, one-bounce case only.
- The \(1\%\) point-source rule is for source size versus range. A formation or a resolved airframe breaks it; the glint formula is the unresolved-extended-target correction, not a reason to abandon \(1/R^2\) for the amplitude.

## Analytical tests that have a closed form

### Takeaway
Each gate below is a number a unit test can compute without a scene: sphere, plate, dipole, exponential Swerling draw, two-ray nulls, inverse-square irradiance, J/S crossover, and a proportional-navigation collision triangle that commands zero acceleration.

### Cited Findings
- Optical sphere. Projected area \(1\,\mathrm{m}^2\) gives \(\sigma=1\,\mathrm{m}^2\), and \(\sigma\) does not change with frequency once the circumference exceeds about \(20\lambda\). Equivalently \(\sigma=\pi a^2\) in that region, because the article sets RCS equal to the projected physical cross section. In the Rayleigh region (circumference \(< 0.75\lambda\)), \(\sigma\propto f^4\): quartering the wavelength multiplies \(\sigma\) by \(256\) if the Rayleigh approximation still holds at both frequencies. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Broadside flat plate. \(A=1\,\mathrm{m}^2\) at \(1\,\mathrm{GHz}\) gives \(\sigma=4\pi A^2/\lambda^2=139.62\,\mathrm{m}^2\). Doubling frequency, still broadside and still in the same formula, multiplies \(\sigma\) by four because \(\lambda\) is halved. — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section)
- Half-wave dipole, co-polarized, wire parallel to E. \(\sigma/\lambda^2\approx 0.86\). At \(3\,\mathrm{GHz}\), \(\lambda=0.1\,\mathrm{m}\), so \(\sigma\approx 0.0086\,\mathrm{m}^2\). Cross-pol on that cut is the minimum, not \(0.86\lambda^2\). — [Stellenbosch University thesis PDF, eq. (3.1)](https://scholar.sun.ac.za/server/api/core/bitstreams/0895adb7-d2ca-4931-9ac0-847db0b895bc/content)
- Free-space skin return. From the radar equation on the RCS page, \(P_r\propto\sigma/R^4\) with everything else fixed. Doubling range multiplies received power by \(1/16\) (\(-12.04\,\mathrm{dB}\)). One-way power density, and point-source irradiance, fall by \(6.02\,\mathrm{dB}\) per doubling of range: \(I_2/I_1=(R_1/R_2)^2\). — [Radar cross section, Wikipedia](https://en.wikipedia.org/wiki/Radar_cross_section), [Inverse-square law, Wikipedia](https://en.wikipedia.org/wiki/Inverse-square_law)
- Swerling I. With mean \(\sigma_{\mathrm{av}}\), \(P(\sigma<\sigma_{\mathrm{av}})=1-e^{-1}\approx 0.632\). A dwell must repeat one draw; the next dwell must draw independently. The exponential generator consistent with the printed PDF is \(\sigma=-\sigma_{\mathrm{av}}\ln u\) for \(u\) uniform on \((0,1)\). — [Fluctuation loss, Wikipedia](https://en.wikipedia.org/wiki/Chi-squared_target_models)
- Two-ray lobes, one-way, \(\Gamma\approx -1\), large \(d\). \(\Delta\phi=4\pi h_t h_r/(\lambda d)\). The factor \(\lvert 1-e^{-j\Delta\phi}\rvert^2\) is \(0\) when \(\Delta\phi=2\pi n\) (\(n=1,2,\ldots\)), i.e. at \(d_n=2 h_t h_r/(n\lambda)\), and equals \(4\) when \(\Delta\phi=\pi\), i.e. at \(d_c=4 h_t h_r/\lambda\) (a \(6\,\mathrm{dB}\) rise over the free-space one-way term inside the same approximation). Far past that, \(P_r=P_t G h_t^2 h_r^2/d^4\), independent of \(\lambda\). — [Two-ray ground-reflection model, Wikipedia](https://en.wikipedia.org/wiki/Two-ray_ground-reflection_model)
- Saturated monostatic J/S. With \(P_j G_{ja}=P_t G_t\) and \(\sigma=1\,\mathrm{m}^2\), \(J/S=4\pi R^2\). At \(R=1\,\mathrm{m}\), \(J/S=4\pi\) and \(10\log_{10}(J/S)\approx 10.99\,\mathrm{dB}\), which is the handbook’s constant. Crossover for general parameters is \(R_{co}=\bigl[(P_t G_t\sigma)/(P_j G_{ja}4\pi)\bigr]^{1/2}\). Burn-through for a required ratio \(K\) is \(R_{co}\sqrt{K}\). For the handbook’s \(6\,\mathrm{dB}\), \(K=10^{0.6}\approx 3.981\) and \(R_{BT}\approx 2.00\,R_{co}\). — [NAWCWD TP 8347 PDF, sections 4-7 and 4-8](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf)
- Glint. A target of extent \(L\) at range \(R\gg L\) has \(\sigma_{\theta g}\approx 0.35\,L/R\) radians. Example: \(L=15\,\mathrm{m}\), \(R=1500\,\mathrm{m}\) gives \(3.5\,\mathrm{mrad}\). At \(150\,\mathrm{m}\) the same formula gives \(35\,\mathrm{mrad}\), ten times larger. — [Jenn, NPS EC4610 notes, Vol. II](http://www.dcjenn.com/EC4610/VolIIv6.0.pdf)
- Collision triangle. Two objects are on a collision course when the line of sight does not rotate while range decreases. Pure proportional navigation commands \(a_n=N\dot\lambda V\). If \(\dot\lambda=0\), the commanded acceleration is \(0\), and the lines of sight stay parallel through the intercept. \(N\) is described as generally an integer from \(3\) to \(5\); the zero-rate case does not depend on which of those values is used. — [Proportional navigation, Wikipedia](https://en.wikipedia.org/wiki/Proportional_navigation)
- False-target geometry. A contact whose extra delay is \(\Delta\tau\) is reported at range \(c\Delta\tau/2\) beyond the skin range. If \(\Delta\tau\) increases at a constant rate \(\alpha\) seconds per second, measured range rate is \(c\alpha/2\) relative to the skin range rate. That is the whole closed form required of a pull-off contact. — [NAWCWD TP 8347 PDF, glossary](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf) together with the monostatic round-trip already used on the RCS page.

### Inferences
- Run these as deterministic unit tests beside `tools/fox2_kinematics`, not as in-game scenarios. The dipole and plate tests protect the RCS table. The \(1/R^4\) and \(1/R^2\) tests protect the radar equation and the existing irradiance line. The J/S test protects the jammer term. The collision-triangle test protects the handoff from a track’s line-of-sight rate into proportional navigation.
- A monostatic two-ray test may additionally require echo power to scale with \(\lvert 1-e^{-j\Delta\phi}\rvert^4\), because each leg contributes the one-way power factor. Treat a failure of that stronger test as a spec question, not as a failure of the one-way formula, until a radar propagation-factor source is opened.

### Gaps
- No opened source gave a numerical RCS for a specific aircraft at a specific aspect that would make a good regression target. Do not invent one. The sphere, plate, and dipole are the regression targets.
- Swerling III’s closed-form probability below the mean was not needed for the gate and was not checked numerically here.
- Proportional-navigation miss distance under lag, saturation, or a manoeuvre is not a closed form on the opened page. The gate is only the zero-rate collision triangle.

## Implementation notes for missilesim

### Takeaway
Add the contact record and the far-field equation first, then hang RCS tables, dwells, glint, J/S, and false contacts on that record. The Fox 2 seeker should keep its irradiance measurement and eventually read the same aspect and range truth, rather than grow a parallel lock-range system.

### Cited Findings
Stage order. Each stage’s gate is one of the closed forms above. Later stages must not change an earlier gate’s expected number.

1. Contact and track types only. A contact stores true geometry, measured geometry, SNR, \(P_d\), and covariance. Nothing detects yet. Gate: true range equals the distance between two platform positions. Owner: new types in `src/sim`. Platforms stay `Fighter` / `PhysicsEngine`.
2. Free-space radar equation with one constant \(\sigma\), no jammer, no fluctuation. Gate: doubling range drops \(P_r\) by \(12.04\,\mathrm{dB}\); the \(1\,\mathrm{m}^2\) optical sphere returns \(\sigma=1\,\mathrm{m}^2\). Atmosphere stays available through `Atmosphere::sample` but the free-space test uses no loss.
3. Body-frame RCS table, still one draw, no time fluctuation. Gate: \(1\,\mathrm{m}^2\) plate at \(1\,\mathrm{GHz}\), broadside, returns \(139.62\,\mathrm{m}^2\), and four times that at \(2\,\mathrm{GHz}\); half-wave dipole at broadside co-pol returns \(0.86\lambda^2\) and not that value on the cross-pol sample.
4. Dwell clock and Swerling. The \(100\,\mathrm{Hz}\) flight step does not redraw a Swerling I target. Gate: within a dwell, \(\sigma\) is constant; across dwells, the fraction below the mean is about \(0.632\); a Swerling 0 target does not move.
5. Angle measurement. Add \(\sigma_{\theta g}\approx 0.35\,L/R\) into the angle covariance. Gate: \(L=15\,\mathrm{m}\) at \(1500\,\mathrm{m}\) yields \(3.5\,\mathrm{mrad}\) rms, and ten times that at one-tenth the range. Do not add thermal angle noise until its formula is transcribed.
6. Saturated monostatic noise jamming, self-protection only (jammer range equals target range). Gate: \(J/S=(P_j G_{ja}4\pi R^2)/(P_t G_t\sigma)\) and \(R_{BT}=R_{co}\sqrt{K}\). Do not implement stand-off ranges until a clean two-range equation is taken from the handbook.
7. Deception as extra contacts. Data is \(\Delta\tau(t)\) and \(\Delta f_d(t)\). Gate: reported range equals skin range plus \(c\Delta\tau/2\). No waveform, no power-ramp procedure, no named victim radar.
8. Seeker consumption. Point `Missile` at a track’s line-of-sight rate for radar-guided flight, and keep Fox 2 on irradiance. Gate: \(\dot\lambda=0\) produces zero proportional-navigation acceleration, and doubling range quarters `measureTarget` irradiance. Retire `tailAcquisitionM == 0` (lock at any range) and MAWS `detectionRange` only after those paths read the same threshold as a number on the contact or the irradiance.

### Inferences
Data files, next to the existing JSON config rather than inside a shader or a C++ table:

- `assets/config/sensors/emitters.json` — name, \(P_t\), \(G_t\), frequency or band id, receiver bandwidth, beamwidths, \((J/S)_{\min}\) if this emitter is also a victim radar. Bandwidth is stored and unused until a linear-jammer equation is sourced.
- `assets/config/sensors/jammers.json` — host platform, \(P_j\), \(G_{ja}\), mode `saturated` or `deception`. Deception entries hold a delay function and a Doppler function (start, rate, end), not a technique name that implies a circuit.
- `assets/config/rcs/<id>.csv` — header `band,polarization,swerling,sigma_av_m2,extent_m`, then rows `azimuth_deg,elevation_deg,sigma_dbsm`. Azimuth and elevation in the body frame. One file per airframe or flare-decoy type. Missing file means the stage-2 tests have nothing to read; do not fall back to a silent \(1\,\mathrm{m}^2\).
- Infrared stays on the Fox 2 card. If a radiant-intensity table is added later, it should be watts per steradian versus aspect, multiplied by \(1/R^2\) the way `measureTarget` already multiplies `intensity`. Do not add a second acquisition range.

Build discipline: `src/rendering` does not gain a radar. `PhysicsEngine` keeps integrating bodies and sampling `Atmosphere`. `src/sim` owns the dwell and the catalog load, the same way it owns `Fox2Catalog`. `Missile` consumes the result.

### Gaps
- \(P_d(\mathrm{SNR})\) is still unfilled; stage 2 can carry SNR with \(P_d\) left at \(1\) above a stated threshold and \(0\) below, and that threshold must be marked temporary.
- Linear jamming (\(B_r/B_j\), processing gain) and stand-off geometry wait on a clean transcription of handbook sections 4-9 and 4-10.
- Glint time correlation, log-normal width, and any real aircraft pattern are intentionally absent. The analytical gates above are the acceptance tests until those sources are opened.
