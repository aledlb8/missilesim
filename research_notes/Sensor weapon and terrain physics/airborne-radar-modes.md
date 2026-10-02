# Airborne fire-control radar modes

Scope: how an airborne pulse-Doppler fire-control radar searches, measures, and tracks one aircraft target, at the level needed to simulate modes. The radar-range equation and terrain multipath are not re-derived here. Those notes are assumed to output three quantities this note consumes:

- **SNR** — signal-to-thermal-noise ratio for the target in the processed dwell if the target is on boresight, with clutter not included and eclipsing not included.
- **Clutter power** — interference power, in the same units as the signal behind that SNR, for the mainlobe ground return and (if split) the sidelobe return. It is meaningful only in the range-Doppler cell that actually contains that clutter.
- **Line of sight** — whether the beam path to the target is clear. If a multipath note also outputs an elevation bias, track modes add it to the elevation measurement mean. They do not recompute the two-ray field.

Detection is a threshold on SINR after the Doppler-cell test below. Nothing in this note is a lock range or a notch width in knots.

Sources were opened on 2026-09-29 unless noted. Passages from *Principles of Modern Radar*, Vol. 3, were retrieved as text from the PDF URL below; a direct full-file fetch of that PDF failed, so those citations are limited to the retrieved equations. Stimson’s *Introduction to Airborne Radar* was not available as an open text in this pass. Named-radar performance numbers were not collected.

## Pulse-Doppler principle

### Takeaway

Unambiguous range and unambiguous velocity are both set by the PRF and cannot be chosen independently: range resolution is set by pulse bandwidth, Doppler resolution by the coherent processing interval, and blind zones repeat wherever the echo falls in a transmit blank or the sampled Doppler falls on a clutter line. Low, medium, and high PRF are definitions in terms of which of those ambiguities you accept, not fixed kilohertz bands.

### Cited Findings

- Two-way Doppler of a radial speed \(v_r\) is \(|f_D| = 2 v_r / \lambda = 2 v_r f_{tx} / c_0\). The shift occurs once outbound and once on the return. If the target’s velocity is not along the line of sight, only the radial component enters, as \(f_D = (2 v / \lambda) \cos\alpha\). — [Radartutorial, Doppler effect](https://www.radartutorial.eu/11.coherent/en/co06.en.html)
- With the sign of \(f_D\) known, Radartutorial requires \(f_{PRF} > |f_D|\), so \(v_r < c_0 f_{PRF} / (2 f_{tx}) = \lambda\,\mathrm{PRF}/2\). If the sign is unknown, that speed is halved again: \(v_r < c_0 f_{PRF} / (4 f_{tx}) = \lambda\,\mathrm{PRF}/4\). The same page calls this the Doppler dilemma: a PRF chosen for long unambiguous range is a poor choice for unambiguous velocity, and the reverse. — [Radartutorial, Doppler dilemma](https://www.radartutorial.eu/01.basics/en/rb50.en.html)
- Combining the sign-unknown speed limit with \(R_\max = c_0 / (2 f_{PRF})\) gives the product \(R_\max v_r < c_0^2 / (8 f_{tx})\). That product depends on carrier frequency once the centered, two-sided convention is used. — [Radartutorial, Doppler dilemma](https://www.radartutorial.eu/01.basics/en/rb50.en.html)
- Maximum unambiguous range is the longest round trip that still fits between pulses. If the echo delay \(t\) exceeds the pulse repetition time \(T\), the set associates the echo with the wrong pulse (second-time-around / range folding). For a short pulse the formula reduces to \(R = c_0 / (2 f_p)\). Worked example on that page: PRF \(= 1000\,\mathrm{Hz}\) gives \(150\,\mathrm{km}\); a \(100\,\mu\mathrm{s}\) delay is \(15\,\mathrm{km}\) if it belongs to the latest pulse and \(165\,\mathrm{km}\) if it belongs to the previous one. If the whole echo must be received, pulse length \(\tau\) is not ignored and the usable interval is shorter than \(T\). — [Radartutorial, maximum unambiguous range](https://www.radartutorial.eu/01.basics/en/rb09.en.html)
- Staggering the PRT moves a second-time-around echo relative to the following period, which is how a processor can tell it from a stable unambiguous return and solve for the true delay. — [Radartutorial, maximum unambiguous range](https://www.radartutorial.eu/01.basics/en/rb09.en.html)
- An open thesis states the same range formula \(R_u = c / (2\,\mathrm{PRF})\) and tabulates the three regimes by which measurement is ambiguous: low PRF, range unambiguous and velocity usually ambiguous; medium PRF, both ambiguous; high PRF, range highly ambiguous and velocity unambiguous. It sets the largest Doppler that can be distinguished unambiguously to \(\mathrm{PRF}/2\), written \(f_{D,\max} = \mathrm{PRF}/2 = 2 V_u f / c\), so that \(V_u\) in that equation is \(\lambda\,\mathrm{PRF}/4\). The author calls medium PRF “usually the best option of the waveform for airborne radar” even though both dimensions are ambiguous. — [Wu, UCL thesis, Table 2.1 and §2.1](https://discovery.ucl.ac.uk/id/eprint/10123395/9/WU_10123395_Thesis.pdf)
- Retrieved text of *Principles of Modern Radar*, Vol. 3, uses a different symbol for the same physics: the unambiguous velocity **extent** is \(v_U = \lambda f_p / 2\), and because velocity may be positive or negative the centered window is \(\pm v_U/2\) (Doppler \(\pm f_p/2\)). Apparent velocity and Doppler are the true values folded modulo that window. The product of unambiguous range and that full velocity extent is fixed for a given wavelength. The same passage gives blind ranges where the receiver is blanked during transmit: \(n R_U \le R_\mathrm{blind} \le n R_U + c\, t_\mathrm{blank}/2\), with \(t_\mathrm{blank}\) the uncompressed pulse width when pulse compression is used. Minimum range for a complete pulse is \(R_\min = c\, t_\mathrm{blank}/2\). — [Richards, Melvin, Scheer, Holm, *Principles of Modern Radar*, Vol. 3, Ch. 5 (retrieved passages; full PDF fetch failed)](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- The same retrieved chapter classifies regimes the way the thesis does, and adds textbook **typical** X-band airborne numbers, not a named radar: high PRF about \(100\)–\(300\,\mathrm{kHz}\), medium about \(5\)–\(30\,\mathrm{kHz}\), low about \(0.3\)–\(2\,\mathrm{kHz}\). It states that blind-zone incidence follows the ambiguities: many range blinds in high PRF, many velocity blinds in low PRF, a moderate number of both in medium PRF. It also states that low PRF separates poorly from mainlobe clutter when the mainlobe clutter Doppler extent is a large fraction of the PRF. — [same POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- Range resolution: a radar distinguishes two targets on the same bearing if their echoes are separated by half a pulse width in the round-trip sense. For pulse compression the resolution is set by transmit bandwidth \(B_{tx}\), not by the uncompressed width. The same page’s numerical check is \(1.5\,\mathrm{m}\) at a \(100\,\mathrm{MHz}\) bandwidth, which is \(c_0/(2B)\). — [Radartutorial, range resolution](https://www.radartutorial.eu/01.basics/en/rb18.en.html)
- Angular resolution is set by the one-way \(-3\,\mathrm{dB}\) beamwidth: two equal targets at the same range are resolved in angle if they are separated by more than that beamwidth. That is resolution, not measurement precision. — [Radartutorial, *Book 1*, angular resolution](https://www.radartutorial.eu/druck/Book1.pdf)
- Doppler filter resolution from a coherent interval: retrieved POMR Vol. 3 text says the first null of the unwindowed \(N\)-pulse response is at \(\Delta f = 1/T_\mathrm{CPI}\), so the null-to-null mainlobe width is \(2/T_\mathrm{CPI}\). The half-power width is of that order (they note the exact half-power point is slightly inside the convenient \(\mathrm{PRF}/N\) approximation). Unwindowed Doppler sidelobes start at \(-13.2\,\mathrm{dBc}\). — [same POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- *Principles of Modern Radar*, Vol. I, Ch. 18 distinguishes resolution from precision: precision is the standard deviation of the measurement error and improves as SNR rises; accuracy is the bias of the mean. For a signal in additive white Gaussian noise the variance of an unbiased estimator is bounded by the Cramér–Rao lower bound, which depends on how sharply the signal changes with the parameter being estimated. Multipath and refraction shift the mean (a bias), unlike thermal noise. — [Richards, *Principles of Modern Radar*, Vol. I, Ch. 18](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)

### Inferences

- Use one explicit interval in the sim, not a single symbol \(V_u\) copied from a textbook. Sample slow-time at the PRF. Complex (I/Q) samples are unambiguous over one interval of length PRF; the usual centering is \([- \mathrm{PRF}/2,\, \mathrm{PRF}/2)\). Then \(|v_r| < \lambda\,\mathrm{PRF}/4\) when that interval is centered at zero. The full width of the velocity window is \(\lambda\,\mathrm{PRF}/2\). Radartutorial’s sign-known limit \(\lambda\,\mathrm{PRF}/2\), the thesis’s \(V_u = \lambda\,\mathrm{PRF}/4\), and POMR’s \(v_U = \lambda\,\mathrm{PRF}/2\) as a full extent with limits \(\pm v_U/2\) are the same statement. A sim that treats \(\lambda\,\mathrm{PRF}/2\) as a one-sided closing speed will be high by a factor of two.
- Standard formulas to code, with the centered I/Q convention:
  - \(R_u = c_0 / (2\,\mathrm{PRF})\), or \(c_0 (T - \tau)/2\) when the entire echo must land in the listening time.
  - \(f_D = 2 v_r / \lambda\), folded into \([- \mathrm{PRF}/2,\, \mathrm{PRF}/2)\).
  - First speed that aliases to zero Doppler: \(v = n \lambda\,\mathrm{PRF}/2\). The edge of the centered window is half of the first blind speed.
  - Range resolution \(\Delta R = c_0 \tau / 2\) for a simple pulse whose bandwidth is about \(1/\tau\), and \(\Delta R = c_0 / (2B)\) for a compressed pulse. The \(1.5\,\mathrm{m}\) at \(100\,\mathrm{MHz}\) check is the compressed-pulse form. **Standard**, not a rule of thumb.
  - Doppler Rayleigh resolution \(\Delta f_D \approx 1/T_\mathrm{CPI} = \mathrm{PRF}/N\) (bin spacing; null-to-null width \(2/T_\mathrm{CPI}\) unwindowed). \(\Delta v = (\lambda/2)\,\Delta f_D\).
- Blind range is not a flag at one range. Around every multiple \(n R_u\), including \(n = 0\), a slice about \(c_0 t_\mathrm{blank}/2\) long is eclipsed because the receiver is off while the transmitter is on. High duty cycle (high PRF) makes those slices a large fraction of each unambiguous interval. Partial overlap should scale SNR by the fraction of the pulse that is received, not only by a binary kill, because the listening-time formula already treats “whole echo received” and “echo arrives during transmit” as the two ends of one overlap.
- Low PRF is the lookup / long-range unambiguous-range waveform, and it is a poor look-down waveform when mainlobe clutter occupies a large fraction of the PRF. High PRF is the look-down waveform for targets whose Doppler sits clear of mainlobe clutter (typically high closing speed); range is folded and must be solved, if at all, from several PRFs, and eclipsing is frequent. Medium PRF accepts both folds so that, across a small set of PRFs, a target that is blind in range or Doppler on one burst can be clear on another. The X-band kilohertz bands above are **textbook typical values**, not limits that define the regimes. The regime test is whether \(R_u\) and the velocity window cover the tactical range and the tactical closing speeds.
- A single PRF cannot simultaneously be the reported range and the reported range rate in medium or high PRF. The dwell output should keep the apparent \((R, f_D)\) pair for each PRF in the burst. Unfolding is the same congruence idea as staggered-PRT second-time-around resolution. If the burst does not agree on one hypothesis, the measurement stays ambiguous or is dropped. Do not invent a true range inside the mode.
- What each PRF regime consumes from the other notes: line of sight (no return if blocked), SNR (detection and the thermal part of SINR, after an eclipsing scale), and clutter power **only if** the target’s apparent range cell and apparent Doppler cell are the clutter’s cells after folding. High PRF folds many clutter ranges into one apparent gate; if the clutter note’s power is unfolded versus true range, this mode has to sum the folds. It must not treat an unfolded mainlobe power as if it sat in every gate.

### Gaps

- Stimson was not opened. The regime definitions above are from the Wu thesis and retrieved POMR Vol. 3 text, which match the usual Stimson/Skolnik split but were not checked against a Stimson page.
- The full POMR Vol. 3 PDF did not fetch; equation numbers in the retrieved text (for example the blind-range inequality) were not re-read in context beyond those passages.
- The exact Cramér–Rao prefactor that converts bandwidth or \(T_\mathrm{CPI}\) into \(\sigma_R\) and \(\sigma_{f_D}\) was not extracted as a closed form from the opened POMR chapter (the chapter states the bound, not a single engineering constant). See the track-filter section.
- No open source in this pass justified a universal PRF list, duty cycle, or number of PRFs per medium-PRF dwell for a generic fighter radar. Those stay parameters. The X-band bands are labeled typical.

## Clutter spectrum from platform motion

### Takeaway

Mainlobe clutter is not a fixed notch at zero Doppler. Its center is the ground’s radial speed along the beam, \((2 V_a/\lambda)\cos\gamma\), and its width grows with platform speed, beamwidth, and the sine of the angle off the velocity vector. A target that is stationary with respect to the ground has that same Doppler and falls in the notch at every look angle. Each dwell should test the target’s Doppler, after PRF folding, against that interval, and only then apply clutter power.

### Cited Findings

- Naval Postgraduate School lecture notes (Jenn), citing Skolnik’s clutter-spectrum figure, split the ground return into three pieces: mainbeam clutter (strong because of mainbeam gain), sidelobe clutter (weaker but spread over a wide angle and therefore a wide Doppler), and the altitude return (strong because the depression is near normal incidence). — [Jenn, NPS EC4610 notes, Vol. 2](http://faculty.nps.edu/jenn/EC4610/Vol2v7.2.pdf)
- The same notes place mainbeam clutter center at the Doppler of the footprint: \(f_d = 2 v_a \cos\gamma / \lambda\), where \(\gamma\) is the angle between the platform velocity and the beam. The width follows by differentiating that map: \(\Delta f_d = \mathrm{BW}_\mathrm{MLC} \approx (2 v_a / \lambda) \sin\gamma \,\Delta\gamma\), with \(\Delta\gamma\) the beamwidth between first nulls. They approximate \(\Delta\gamma \approx 2.5\,\theta_B\) and write \(\Delta f_d \approx (2 v_a / \lambda) \sin\gamma\,(2.5\,\theta_B)\). A worked sketch in those notes evaluates the expression to \(1800\,\mathrm{Hz}\) for a \(2.5^\circ\) beam at \(60^\circ\) off the velocity vector; that number is an example, not a notch to reuse. They state the scan dependence directly: forward, the spectrum is high and narrow; broadside, it is low and broad. — [Jenn, NPS EC4610 notes](http://faculty.nps.edu/jenn/EC4610/Vol2v7.2.pdf)
- For a circular aperture of radius \(a\), the same notes give a \(3\,\mathrm{dB}\) beamwidth of \(29.2\,\lambda/a\) degrees and a first-null beamwidth of \(69.9\,\lambda/a\) degrees. The ratio of those two quoted numbers is about \(2.4\), which is why they call \(2.5\,\theta_B\) the null approximation. Treat \(2.5\) as their stated approximation, not as an exact aperture identity. — [Jenn, NPS EC4610 notes](http://faculty.nps.edu/jenn/EC4610/Vol2v7.2.pdf)
- Altitude clutter in those notes is centered at zero Doppler unless the aircraft is maneuvering. Sidelobe clutter is reduced by lowering sidelobes, depends on terrain, and in practice extends over the Doppler span of the platform motion (the notes’ wording is the span out to \(\pm 2 v_a/\lambda\)). — [Jenn, NPS EC4610 notes](http://faculty.nps.edu/jenn/EC4610/Vol2v7.2.pdf)
- Retrieved POMR Vol. 3 text: mainlobe clutter Doppler is centered at \((2 v_r/\lambda)\cos\psi_s\), where \(\psi_s\) is the scan angle from the velocity vector, and both mainlobe clutter and the altitude return lie inside the platform’s Doppler bounds. The altitude return peaks at zero Doppler in horizontal flight and moves off zero in climbs and descents. Velocity blind zones are the Doppler regions the radar ignores because clutter dominates; the main one moves with scan angle as the mainlobe clutter moves. — [POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- The same retrieved text separates clutter in range as well as Doppler. The nearest sidelobe clutter is at slant range equal to radar height. Mainlobe clutter starts at a longer slant range, at the lower edge of the elevation beam. A target between those ranges can compete with sidelobe clutter and not with mainlobe clutter. A target beyond the mainlobe ground intersection competes with both. For a half-power elevation beam of \(3^\circ\) (null-to-null taken as about \(7.5^\circ\), i.e. the same \(2.5\times\) factor), most of those mainlobe-clutter-free ranges are too short to be tactically useful. — [POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- Low PRF cannot push mainlobe clutter into a small part of the PRF when the clutter width is a large fraction of the PRF, which is the look-down failure of that regime. High PRF spreads the unambiguous Doppler enough that mainlobe clutter occupies a smaller fraction of the filter bank, at the cost of range folding. — [POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf); regime names also in [Wu thesis](https://discovery.ucl.ac.uk/id/eprint/10123395/9/WU_10123395_Thesis.pdf)
- On a semi-active missile the clutter Doppler is computed with the same radial-velocity rule by treating the clutter patch as a target with inertial speed zero. Mainlobe and sidelobe clutter both appear. For an airborne illuminator the spectrum can extend below the rear-reference feedthrough because of the illuminator backlobe. — [Skolnik (ed.), *Radar Handbook*, Ch. 19, open PDF](https://ww.helitavia.com/skolnik/Skolnik_chapter_19.pdf)

### Inferences

- Per dwell, compute geometry, do not look up a notch:
  1. Unit line of sight \(\hat{u}\) of the beam (and of the target if it is inside the beam).
  2. Platform air vector \(\mathbf{V}_a\). Cone angle from \(\cos\gamma = (\mathbf{V}_a \cdot \hat{u}) / V_a\).
  3. Mainlobe center \(f_\mathrm{MLC} = 2 (\mathbf{V}_a \cdot \hat{u}) / \lambda\).
  4. Width \(\Delta f_\mathrm{MLC} = (2 V_a / \lambda)\,|\sin\gamma|\,\Delta\gamma\). Choose \(\Delta\gamma\) explicitly. **Standard differential:** the derivative above. **Approximation, NPS and retrieved POMR:** \(\Delta\gamma \approx 2.5\,\theta_{3\mathrm{dB}}\) if \(\Delta\gamma\) means null-to-null. Using \(\theta_{3\mathrm{dB}}\) itself is a narrower, more optimistic notch. Two-way illumination is narrower than the one-way beam; either choice must be a named parameter, not a constant in hertz.
  5. Altitude line \(f_\mathrm{alt} = 2 V_\mathrm{vertical}/\lambda\), which is \(0\) in level flight (NPS, POMR). Give it its own small width from the Doppler spread of the nadir sidelobe footprint, not the mainlobe width.
  6. Sidelobe clutter occupies roughly \([-2 V_a/\lambda,\, +2 V_a/\lambda]\) at the lower power the clutter note assigns to sidelobes.
  7. Fold \(f_\mathrm{MLC}\), the width, \(f_\mathrm{alt}\), and the target Doppler \(f_t = 2 v_{r,\mathrm{closing}}/\lambda\) by the dwell PRF into the same unambiguous interval.
  8. Mainlobe conflict if the folded target tone lies in the folded mainlobe interval expanded by about one Doppler bin (\(1/T_\mathrm{CPI}\)). Altitude-line conflict is a separate test near \(f_\mathrm{alt}\).
  9. Range coincidence: apply mainlobe clutter power only if the target’s apparent range matches the apparent range of terrain illuminated by the main beam. Inside the altitude but inside a closer range, only sidelobe clutter power applies (retrieved POMR range split). If line of sight to the target is false, do not form a detection at all.
- Why a target that is co-speed with the ground falls in the notch: the ground is stationary in the earth frame, so its radial speed along \(\hat{u}\) is \(\mathbf{V}_a\cdot\hat{u}\). A target with essentially zero earth velocity has that same radial speed and the same \(f_D\), at whatever \(\gamma\) the beam is using. The notch is centered on the clutter, so the target sits in it. This is not the same thing as co-speed with the fighter. A target with the fighter’s velocity vector has closing speed about zero, so \(f_t \approx 0\). That tone hits the altitude line, and it hits mainlobe clutter only when \(\cos\gamma \approx 0\) (beam near broadside), where the mainlobe center is also near zero and the width is largest because \(\sin\gamma \approx 1\).
- Look-up versus look-down is a geometry test on the same formulas. Look-up: the main beam does not intersect terrain at the target’s range (line of sight to ground in the beam is false, or the intersection range is not the target cell). Clutter power in the target cell is then only whatever sidelobe power the clutter note supplies, often negligible, and SNR alone sets detection. Low PRF is usable. Look-down: the main beam hits the ground near the target range, clutter power is large, and the Doppler test decides whether that power enters the target’s SINR. A target clear of the folded notch is noise-limited; a target inside it is clutter-limited and usually undetected. High PRF is used so the notch is a small piece of a wide velocity window; low PRF look-down often notches out a large fraction of closing speeds, especially off the nose where \(\Delta f_\mathrm{MLC}\) is wide.
- SINR for a cell that fails the notch test should use the clutter power output. SINR for a cell that passes should use thermal SNR, plus sidelobe clutter power if that output exists. Replacing the test by a constant speed gate (for example “reject below 90 knots”) will both hide targets that are clear of mainlobe clutter and pass targets that match the ground speed along the line of sight. The 90-knot figure belongs only to the sim-manual discussion below.

### Gaps

- The NPS notes and the retrieved POMR text do not give a unique rule for whether \(\Delta\gamma\) is the one-way \(3\,\mathrm{dB}\) width, the two-way \(3\,\mathrm{dB}\) width, or null-to-null. The sim must expose that choice. The \(1800\,\mathrm{Hz}\) sketch is one numerical example; platform speed and wavelength in that sketch were not cleanly readable, so it is not reused.
- How much sidelobe clutter power folds into a given apparent range-Doppler cell is a clutter-note calculation (terrain cross section, sidelobe gain, number of PRF folds). This note only states when that power is added.
- Clutter internal motion (windblown vegetation, sea) adds width beyond the platform-motion term. No number for that spread was opened; leave it as an optional extra width parameter, default zero, rather than inventing a knots value.

## Scan, beam, and what search versus track measures

### Takeaway

Beamwidth is set by wavelength over aperture. Search frame time is the number of beams times the dwell, so a larger volume either revisits a target less often or integrates fewer pulses. Search and track-while-scan measure range, range rate, and monopulse angles once per frame; single-target track measures the same quantities at a much higher rate and can form angle rate because the beam stays on the target. Angle precision is on the order of \(\theta_{3\mathrm{dB}} / (k_m \sqrt{2\,\mathrm{SNR}})\), not the beamwidth itself.

### Cited Findings

- For a uniformly illuminated rectangular aperture the one-way \(3\,\mathrm{dB}\) beamwidth is approximately \(0.89\,\lambda/D\) radians (small-angle form of \(2\sin^{-1}(1.4\lambda/(\pi D))\)). The Rayleigh width, peak to first null, is \(\approx \lambda/D\) radians for that illumination. — [Richards chapter extract, eq. (1.9)–(1.10)](https://www.accessengineeringlibrary.com/binary/mheaeworks/e201c9145cbf4b50/173bbcf78de9285a156883a24c6d0354fd4596665d79b45fcb243916b6509451/book-summary.pdf)
- A separate calculator page, attributing Balanis and IEEE Std 145, calls \(\theta_{3\mathrm{dB}} = k\lambda/D\) the aperture rule: about \(58\lambda/D\) degrees for a uniformly illuminated circular aperture and about \(70\lambda/D\) degrees for a dish with a \(10\)–\(15\,\mathrm{dB}\) edge taper. The \(70\) factor is a **rule of thumb** for tapered circular apertures; the \(0.89\,\lambda/D\) radian result is the **standard** uniform-rectangular derivation. NPS’s circular-aperture figures (\(29.2\,\lambda/a\) degrees at \(3\,\mathrm{dB}\), \(a\) the radius) match a uniform circular width of about \(1.02\lambda/D\) radians. — [rftools beamwidth calculator](https://rftools.io/calculators/antenna/antenna-beamwidth/); [Jenn, NPS notes](http://faculty.nps.edu/jenn/EC4610/Vol2v7.2.pdf)
- Azimuth and elevation widths are independent: each uses its own aperture length. Angular resolution in each plane is that \(3\,\mathrm{dB}\) width. — [Radartutorial Book 1](https://www.radartutorial.eu/druck/Book1.pdf)
- Dwell and frame time are one budget. Radartutorial’s surveillance example: a \(1.6^\circ\) beam has \(360/1.6 = 225\) directions; a \(5\,\mathrm{s}\) revisit gives a dwell of \(5/225 \approx 22\,\mathrm{ms}\); \(20\) hits in that dwell force a pulse period near \(1\,\mathrm{ms}\) and an unambiguous range near \(150\,\mathrm{km}\). The numbers are for a rotating ATC radar, but the relation is general: dwell equals frame time divided by the number of beam positions. — [Radartutorial, radar time budget](https://www.radartutorial.eu/01.basics/Time-dependences%20in%20Radar.en.html)
- Monopulse measures angle by forming sum and difference beams on the same pulse, instead of scanning a single beam past the target. The in-phase monopulse ratio \(\eta\) maps to angle by \(\theta \approx (\theta_3 / k_m)\,\eta\), with slope \(1 < k_m < 2\), and the linear map is the usual one inside \(\pm\theta_3/2\). For SNR above about \(13\,\mathrm{dB}\), the variance of the ratio is \(\sigma^2_{y_I} \approx (1/(2\,\mathrm{SNR}))\,(\sigma_d^2/\sigma_s^2 + \eta^2 - 2\rho\,\sigma_d/\sigma_s)\), and the angle variance is \(\sigma^2_{\hat\theta} \approx (\theta_3^2 / k_m^2)\,\sigma^2_{y_I}\). On boresight, with equal uncorrelated sum and difference noise, that reduces to \(\sigma_\theta \approx \theta_3 / (k_m \sqrt{2\,\mathrm{SNR}})\). Off boresight the \(\eta^2\) term increases the error. The chapter notes that the ratio is a biased direction estimate at moderate and low SNR, so the formula is a high-SNR result. — [Richards, POMR Vol. I, Ch. 18, eqs. (18.67)–(18.71) as retrieved from that PDF](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)
- The same chapter’s peak-pick scan example is a different estimator: there the empirical error spread was not the monopulse \(1/\sqrt{\mathrm{SNR}}\) law. Do not use the scan-histogram scaling in place of the monopulse formula. Monopulse can refine angle inside one beamwidth on a single pulse; resolution of two targets still needs about a beamwidth of separation. — [Richards, POMR Vol. I, Ch. 18](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)
- What is measured, from that chapter’s opening: range from time delay, range rate from Doppler, angle from simultaneous beams (monopulse) or from a scanned beam. — [Richards, POMR Vol. I, Ch. 18](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)

### Inferences

- Code \(\theta_{az} = k\lambda/L_{az}\) and \(\theta_{el} = k\lambda/L_{el}\) with \(k\) documented. Default \(k = 0.89\,\mathrm{rad}\) (uniform rectangular) or the NPS circular values if the aperture is round. Use \(70\lambda/D\) in degrees only as a tapered-dish rule of thumb. Wavelength is the carrier wavelength already used for Doppler, so beamwidth, \(R_u\), and \(v\) window stay consistent when frequency changes.
- Airborne bar scan, built from the time-budget identity: \(N_{az} = \Omega_{az} / \theta_\mathrm{step}\), \(N_{el} = N_\mathrm{bars}\), \(T_\mathrm{frame} = N_{az} N_{el} T_\mathrm{dwell}\), \(T_\mathrm{dwell} = N_\mathrm{CPI}\, N / \mathrm{PRF}\). Beam step is on the order of \(\theta_{3\mathrm{dB}}\) (tighter overlap costs more beams). Each medium-PRF CPI in the burst consumes dwell. **Standard relation; the ATC numbers are only an illustration.**
- Coherent processing of \(N\) pulses in the Doppler filter that matches the target raises thermal SNR by about \(N\) relative to one pulse, if the supplied SNR is single-pulse. If the range-equation note already folded \(N\) into SNR, do not apply it again. Shortening the dwell to widen the scan therefore cuts SNR, lowers detection probability, and worsens \(\sigma_\theta\), \(\sigma_R\), and \(\sigma_{v_r}\). Lengthening the frame instead leaves SNR alone but starves the tracker (next section). That is the scan-volume versus \(P_d\) trade. No numeric optimum was found.
- On-boresight SNR must be scaled by the two-way antenna power pattern at the target’s offset from the beam center before the threshold. By the definition of \(\theta_{3\mathrm{dB}}\), a Gaussian one-way power pattern that is \(1/2\) at \(\theta_{3\mathrm{dB}}/2\) is \(\exp(-4\ln 2\cdot \theta^2/\theta_{3\mathrm{dB}}^2)\). Two-way power for the same pattern on transmit and receive is the square of that. This Gaussian shape is an **inference from the \(3\,\mathrm{dB}\) definition**, not a quoted aperture pattern. A hard “inside the \(3\,\mathrm{dB}\) contour or invisible” cutoff is coarser than that and should not be the only model.
- Search (and range-while-search): one illumination per frame. On a detection the dwell can report apparent range, apparent Doppler (range rate), and monopulse azimuth and elevation relative to the current boresight. It does not measure angle rate in that dwell; a rate appears only if a tracker differences successive frames. Update interval is \(T_\mathrm{frame}\).
- Track-while-scan: the same measurements and the same update interval, with a tracker running on several targets while the scan continues. Off-boresight monopulse error is whatever the beam-offset formula gives on that pass; the beam is not pointed at the predicted target unless the scan happens to center it. Reducing the scan volume is how this mode buys a higher update rate without a shorter dwell.
- Single-target track: the beam is steered to the predicted angle every dwell, so the monopulse error is the on-boresight formula and the update interval is \(T_\mathrm{dwell}\), not \(T_\mathrm{frame}\). Angle rate is observable because angle is measured many times per second while the line of sight is continuously available. The volume is not searched while this state owns the antenna. An electronically steered radar can time-share a few such dwells with a search frame; that is search-while-track in the literal sense. Update rate of the dedicated target is then the rate of inserted dwells, which can exceed the search-frame rate. No open manual in this pass gave a universal interleave ratio; keep it a scheduler parameter.
- All three consume line of sight, SNR (after pattern and eclipsing), and clutter power under the Doppler-and-range cell test. Track does not get a free pass through the notch: a tracked target that maneuvers into the mainlobe interval loses detections and the filter coasts.

### Gaps

- No opened source gave a standard bar count, azimuth extent, or frame time for a generic air-combat search. Those are mode parameters. Sim-manual menu values are listed in the mode-name section and are not physics.
- The monopulse variance quote is the high-SNR, SNR \(\gtrsim 13\,\mathrm{dB}\) form. A low-SNR bias correction was mentioned by the chapter but not extracted as a formula to code.
- Two-target angular resolution versus single-target monopulse precision are different; the opened text does not give a detection model for unresolved pairs inside one beam.

## Track filtering

### Takeaway

An \(\alpha\)–\(\beta\) filter or a Kalman filter should carry position and velocity (acceleration if the maneuver model needs it). Measurement variance on each update must be computed from that dwell’s SNR, bandwidth, coherent interval, and beamwidth. A constant meter error will not reproduce longer errors at low SNR or the beamwidth dependence of angle.

### Cited Findings

- Benedict and Bordner’s relation for an \(\alpha\)–\(\beta\) tracker that balances noise smoothing against transient response to a constant-velocity target is \(\beta = \alpha^2 / (2 - \alpha)\), for \(0 < \alpha \le 1\). Secondary open sources quote this as the 1962 result; the relation is repeated in a Naval Surface Warfare Center digest and in later filter comparisons. The 1962 IRE paper itself was not opened. — [DTIC ADA260831, NSWC digest, as indexed](https://archive.org/details/DTIC_ADA260831); [Jeong, Bazan, and others, 2017, as indexed; direct PDF fetch hit a captcha](https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-5231e9f2-04e2-4ff6-833d-5cfaa23ef013/c/A_Study_Jeong_1_2017.pdf)
- The same digest describes Kalata’s tracking index as the design parameter that ties the gains to target maneuverability variance, measurement-noise variance, and the update interval. A later NSWCDD report restates three common \(\alpha\)–\(\beta\) gain relations, including Benedict–Bordner \(\beta = \alpha^2/(2-\alpha)\) and a Kalata steady-state relation of the form \(\beta = 2(2-\alpha) - 4\sqrt{1-\alpha}\) (OCR in the index is degraded; this is the usual printed form of that relation). — [DTIC ADA260831](https://archive.org/details/DTIC_ADA260831); [DTIC ADA396720, as indexed](https://apps.dtic.mil/sti/tr/pdf/ADA396720.pdf)
- Angle measurement variance from SNR and beamwidth is the monopulse result already cited: \(\sigma_\theta \approx \theta_3 / (k_m \sqrt{2\,\mathrm{SNR}})\) on boresight at high SNR, with \(k_m\) between 1 and 2. — [POMR Vol. I, Ch. 18](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)
- POMR Ch. 18 also states that range and Doppler are estimated from delay and frequency, that their precision is SNR-dependent through the Cramér–Rao bound, and that the bound for a signal in white Gaussian noise scales with the noise variance over the energy of \(\partial s/\partial\theta\). Multipath biases the mean and should not be modeled as a larger thermal \(\sigma\). — [POMR Vol. I, Ch. 18](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)
- Blackman’s *Multiple-Target Tracking* and Bar-Shalom’s tracking texts were not opened. The open substitutes above are the sources actually used for the gain relation and the measurement law.

### Inferences

- The one-dimensional filter those sources are talking about, written so the sim can implement it, is the standard \(\alpha\)–\(\beta\) recursion. This recursion is the usual form associated with the Benedict–Bordner gains; the opened secondary sources describe position and velocity smoothing with two gains and sample time \(T\) but their equation images were not recovered as text, so the lines below are the standard recursion, not a transcription of a photographed equation:
  - Predict \(x_p(k) = x_s(k-1) + T\, v_s(k-1)\).
  - Residual \(r = z(k) - x_p(k)\).
  - Smooth \(x_s = x_p + \alpha r\), \(v_s = v_s(k-1) + (\beta/T)\, r\).
  - Benedict–Bordner constraint: \(\beta = \alpha^2/(2-\alpha)\). \(\alpha\) itself is not fixed by that relation; it is chosen from the tracking index, i.e. from process-noise intensity, measurement variance, and \(T\).
- Prefer a Kalman filter when the measurement set is range, range rate, and two angles. State at least \([x,y,z,v_x,v_y,v_z]\) in a Cartesian frame. Measurement function \(h\) maps that state to apparent range, apparent range rate, azimuth, and elevation. Measurement covariance \(R\) is diagonal with the dwell sigmas below, not a constant. Process noise \(Q\) represents maneuver acceleration that the state does not include. Steady-state gains of the one-dimensional white-acceleration Kalman filter are the \(\alpha\)–\(\beta\) (or \(\alpha\)–\(\beta\)–\(\gamma\)) family; that is why the tracking index is defined from \(\sigma_\mathrm{maneuver}\), \(\sigma_\mathrm{meas}\), and \(T\). If \(T\) changes between search and single-target track, the gains must change. Reusing search gains at a track update rate oversmooths or diverges.
- Thermal-noise sigmas to put in \(R\), each flagged:
  - **Angle — quoted, high SNR, boresight:** \(\sigma_{az} = \theta_{az}/(k_m\sqrt{2\,\mathrm{SINR}})\), and the same for elevation with \(\theta_{el}\). Use \(k_m\) as a parameter in \((1, 2)\). Off-boresight, inflate with the POMR term in \(\eta^2\), or equivalently grow \(\sigma\) as the monopulse slope leaves the linear region near the beam edge.
  - **Range and range rate — engineering form, not a retrieved closed CRLB:** \(\sigma_R \approx \Delta R / \sqrt{2\,\mathrm{SINR}}\) with \(\Delta R = c_0/(2B)\), and \(\sigma_{v_r} \approx (\lambda /(2 T_\mathrm{CPI})) / \sqrt{2\,\mathrm{SINR}}\). The \(\sqrt{2\,\mathrm{SINR}}\) structure matches the monopulse result that was quoted; the exact spectral factor in the Cramér–Rao bound was not extracted and is **order unity**, waveform-dependent. Do not treat these two lines as a Skolnik equation number.
  - Use SINR, not SNR, when the cell contains clutter power but still detects. If the notch test fails the threshold, there is no measurement; do not feed the filter a wild \(z\) with a huge \(R\).
- Association: predict the state to the dwell time and accept a hit only if it lies in a gate set by the predicted covariance (a Mahalanobis gate). A maneuver of acceleration \(a\) over a frame \(T_\mathrm{frame}\) moves the target by about \(\tfrac{1}{2} a T^2\) relative to a constant-velocity prediction. That residual is absorbed only if \(Q\) and the gate grew enough. This is the mechanism for “revisit too slow,” not a separate miss-distance rule.
- Coast on missed dwells: predict only, let \(P\) grow with \(Q\), and drop the track when age or covariance exceeds a limit. A display aging timer is not a substitute for that covariance growth.

### Gaps

- Blackman and Bar-Shalom were not opened. No page number from those books is claimed.
- The exact Kalata expression \(\Lambda = T^2 \sigma_a / \sigma_m\) was not recovered cleanly from the OCR of the opened-index reports. The sim can set \(\alpha\) from a chosen process-noise density and the dwell \(R\), or expose \(\alpha\) as a parameter tied to \(T\) and \(\sigma_z\). It should not hard-code \(\alpha = 0.5\) for every mode.
- Process-noise spectral density for a fighter maneuver (a “g” model) was not taken from a cited table. It is a target-model parameter. The measurement side, not \(Q\), is what this radar note fixes from SNR and beamwidth.
- Converted-measurement bias (range-azimuth to Cartesian at long range and poor angle SNR) is real and was not given a formula here. A Cartesian Kalman that treats \(\sigma_{az} R\) as a linear cross-range sigma is the first-order implementation; the bias correction is a gap.

## Air-combat mode names

### Takeaway

RWS, VS, TWS, STT, ACM/boresight, and flood are mode names from open aircraft-radar descriptions and from simulation manuals. Physics decides what a dwell can measure. The manuals add UI rules (speed gates in knots, trackfile caps, flood as an AIM-7 illumination path) that must stay labeled as manual behavior and must not be wired in as clutter physics.

### Cited Findings

- **Physics, already cited above.** A coherent search dwell can measure apparent range, apparent Doppler, and monopulse angles if SNR and the clutter-cell test allow it. A high-PRF dwell can make Doppler unambiguous while leaving range highly ambiguous. A low-PRF dwell does the opposite. Single-target tracking points the beam at one target and raises the update rate. Continuous semi-active illumination requires the illuminator beam to keep the target inside it for the engagement, with a rear reference of the direct transmission on the missile. — [Wu thesis PRF table](https://discovery.ucl.ac.uk/id/eprint/10123395/9/WU_10123395_Thesis.pdf); [POMR Ch. 18 measurements](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf); [Skolnik Ch. 19](https://ww.helitavia.com/skolnik/Skolnik_chapter_19.pdf)
- **Simulation manual, not physics** (VRS TacPack / Superbug A/A radar page, opened). The radar supplies range, range rate, angle, velocity, and acceleration into weapon-employment equations while tracking. It maintains trackfiles in range-while-search (RWS) and track-while-scan (TWS) that can be designated for a missile. Post-launch support on that page is a data link for AIM-120 and a FLOOD mode for AIM-7. Search modes cycle RWS, TWS, and velocity search (VS). VS draws radial groundspeed, not range; the bottom of that scale is co-velocity; the manual’s VS scales are 800 or 2400 knots; VS does not create trackfiles, only raw hits. RWS and TWS create trackfiles, stated as up to 8 primary plus 4 low-priority, 12 total. A “speed gate” has NORM and WIDE: NORM is 90 knots of closure in VS and 90 knots of target groundspeed in RWS and TWS; WIDE removes the gate. SET stores range scale, azimuth scan, elevation bar, target aging, and PRF per weapon. Aging times are stepped in seconds (2, 4, 8, 16, …); aged tracks are extrapolated and then dropped, including when the extrapolation leaves the gimbal limits. ACM submodes on that page are boresight (BST), vertical acquisition (VACQ), wide acquisition (WACQ), and gun acquisition (GACQ), used to acquire into a track at short range. In STT, RAID, expanded TWS, and ACM the range scale is automatic. FLOOD is a state the pilot can leave; with AIM-7 FLOOD the manual removes the steering dot and does not draw launch zones, i.e. it is not presenting a precision launch solution. Antenna elevation **display** scale runs \(\pm 60^\circ\); gimbal limits are mentioned separately and are not given a number on the extracted lines. — [VRS A/A Radar manual](https://forums.vrsimulations.com/support/index.php?title=A/A_Radar)

### Inferences

- Map names onto physics as follows. Manual-only numbers stay out of the detector.
  - **RWS:** search dwells over a selected az/el volume, detections plotted in apparent range versus azimuth, monopulse angles if SNR allows, Doppler available to the filter even if the display is a range-azimuth B-scope. Trackfiles are optional bookkeeping on top of search hits (the manual’s latent track-while-scan), still at the search update rate. Consumes LOS, SNR, clutter power via the cell test.
  - **VS:** a high-PRF search whose reported coordinate is Doppler (radial speed) rather than unfolded range. Range may be absent or only ambiguous. The manual’s co-velocity baseline is zero closing speed (\(f_D \approx 0\)), which is not the mainlobe-clutter center except near broadside. Consumes LOS, SNR, and clutter power in the Doppler cell; sidelobe clutter matters because many ranges fold in. Do not require a range output to call a VS detection valid.
  - **TWS:** RWS-like measurements plus a multi-target filter, usually inside a smaller scan so \(T_\mathrm{frame}\) shrinks. Same three inputs. Quality is worse than STT at the same SNR because the revisit is the frame and the beam is not dedicated. The manual’s cap of 12 trackfiles is a **sim-manual limit**, not a radar law; a physics tracker drops tracks when covariance or age blows up, and any cap is a separate parameter.
  - **STT:** dedicated beam, high-rate range, range rate, angles, and derived angle rates and accelerations for the launch computer. The manual’s “velocity and acceleration” outputs are filter products, not extra radar observables. Consumes LOS, SNR, and clutter power on every CPI. This is the state that can keep a narrow illuminator beam on the target.
  - **ACM / boresight:** a short-range search of a small volume (boresight is the degenerate small volume) that auto-transitions to STT when a detection passes the same SINR test. No special detection physics. Scan widths and ranges of BST/VACQ/WACQ were not in the lines extracted from the manual; do not invent them.
  - **Flood:** from the manual, an AIM-7 illumination path that does not present launch-zone steering. Physics reading, consistent with Skolnik’s requirement that a semi-active missile needs the target illuminated and a rear reference: flood is illumination with a widened beam and without a continuous monopulse track. It consumes line of sight only as a constraint (the target must lie in the wide beam). It does not consume SNR or clutter power to form a track measurement. Beam solid angle is a parameter; no opened physics source gave a flood width.

### Gaps

- Public NATOPS-style tables of RWS waveform, bar counts, and detection range were not opened, on purpose where they would be specific-radar performance, and by availability otherwise. The Chinese-language mode survey that appeared in search was not fetched and is not used.
- Degree widths and nautical-mile caps of ACM submodes were not on the extracted manual lines.
- Whether a real RWS mode is low PRF, medium PRF, or PRF-agile is a radar-design choice. The sim should attach a PRF schedule to the mode as data, not hard-code one schedule from a game.

## Weapon interface

### Takeaway

The radar’s output to a weapon is a time-stamped measurement with a covariance, plus either a continuous illumination window or discrete datalink messages. Semi-active continuous-wave illumination does not itself provide range. The missile’s bistatic Doppler is not the same number as the fighter’s monostatic range rate.

### Cited Findings

- Semi-active homing: an external radar illuminates the target; the missile tracks the reflected energy. The illuminator “maintains the target within its radar beam throughout the engagement.” The missile takes the reflection in a front antenna and a sample of the direct illumination, often through illuminator sidelobes, in a rear antenna. Mixing those two yields a Doppler “roughly proportional to closing velocity.” A narrowband tracker locks that tone. For a stationary illuminator and a head-on geometry the chapter writes \(f_d = (f_0/c)\,2 V_c\). **Rule of thumb stated there:** at X band (about \(10\,\mathrm{GHz}\)) this is \(20\,\mathrm{Hz}\) per \(\mathrm{ft/s}\) of closing velocity. Check against the two-way law: \(\lambda = 0.03\,\mathrm{m}\), \(1\,\mathrm{ft/s} = 0.3048\,\mathrm{m/s}\), \(2v/\lambda \approx 20.3\,\mathrm{Hz}\). The chapter tells the reader to use the actual carrier when the rule of thumb is not enough. Pure CW discrimination against clutter is by Doppler; the opened pages do not describe a range measurement from unmodulated CW. — [Skolnik Ch. 19](https://ww.helitavia.com/skolnik/Skolnik_chapter_19.pdf)
- The same chapter separates command guidance (radar tracks target and missile and uplinks commands), beam riding, active homing (transmitter on the missile), and track-via-missile (the missile retransmits the received illumination for processing on the ground or ship). Accuracy of command and beam-riding falls with range from the radar; homing accuracy improves as the missile closes. — [Skolnik Ch. 19](https://ww.helitavia.com/skolnik/Skolnik_chapter_19.pdf)
- **Simulation manual:** while tracking, the radar provides range, range rate, angle, velocity, and acceleration for weapon equations; AIM-120 gets a post-launch data link; AIM-7 gets FLOOD. Launch zones are not drawn for an angle-only track. Closing velocity on the format is the sum of ownship and target radial speeds along the line of sight, to the nearest 10 knots, and only when range rate exists. — [VRS A/A Radar manual](https://forums.vrsimulations.com/support/index.php?title=A/A_Radar)

### Inferences

- Minimum packet, independent of missile type, stamped at the dwell center \(t_m\):
  - Range and a flag that it is apparent, unfolded, or absent (VS / CW).
  - Monostatic range rate from the fighter radar’s Doppler, with the PRF fold identified.
  - Azimuth and elevation in a named frame (antenna at \(t_m\), body, or local level), plus the boresight that measurement was relative to.
  - Quality: SINR, the four sigmas (or the covariance), hit count, coast count, and whether the last update was a measurement or a prediction.
  - Do not send a unitless “lock strength” that ignores SNR and beamwidth.
- Illumination, only for semi-active: a time interval \([t_0, t_1]\) over which the target is inside the illuminator beam, the beam widths, the carrier, and the modulation (pure CW versus pulse Doppler) so the missile’s rear reference matches. The constraint from Skolnik is continuous containment in the beam, not a single plot. STT can satisfy that with a narrow beam. A scanning TWS frame generally cannot, because the beam leaves the target. Flood satisfies illumination with a wider beam and without the high-rate track packet. The fighter’s monostatic \(v_r\) is a launch-computer input; the missile still forms its own front-minus-rear Doppler from the geometry Skolnik describes (illuminator, missile, and target radial components). Passing only monostatic range rate as if it were the seeker tone is the wrong interface.
- Datalink, for midcourse into an active terminal seeker: discrete messages at transmit times \(t_\mathrm{tx}\), each a target state and covariance predicted or measured at a stated time, not a continuous illumination flag. Command guidance is a different packet (steering commands), not required for a homing weapon.
- The packet still depends on the three inputs: no track update if line of sight fails; sigmas from SINR; SINR uses clutter power only in a conflicting cell. A coasted packet must be marked coasted so the weapon does not treat the prediction as a new measurement.

### Gaps

- Message rates, word formats, and rear-reference modulation details for any named missile were not collected. The interface above is the physical content, not a bus ICD.
- Skolnik’s \(20\,\mathrm{Hz}\) per \(\mathrm{ft/s}\) rule is X-band only. Other bands scale with carrier, as that page says.
- No opened source specified how long a flood illumination may replace STT before a semi-active missile’s Doppler tracker is on its own. That timeout is a weapon parameter, not a radar constant.

## Failure modes

### Takeaway

Doppler blindness, the clutter notch, a slow scan, multipath elevation bias, and range folding are consequences of the PRF, the beam, the platform velocity, SNR, and the line of sight. They should appear by running those tests, not by scripting a miss.

### Cited Findings

- Range folding: if the round-trip time exceeds the PRI, the echo is assigned the delay modulo the PRI, so a far target is displayed at a short range. Example already cited: \(100\,\mu\mathrm{s}\) can be \(15\,\mathrm{km}\) or \(165\,\mathrm{km}\) at a \(1000\,\mathrm{Hz}\) PRF. Staggered PRT is one way the true delay is recovered; a single fixed PRI does not remove the ambiguity. — [Radartutorial, unambiguous range](https://www.radartutorial.eu/01.basics/en/rb09.en.html)
- Eclipsing / blind range: returns that arrive while the receiver is blanked are lost. Retrieved POMR text places those intervals at every multiple of \(R_u\), with width set by the blanking time, using the uncompressed pulse width under pulse compression. High PRF has many such intervals because \(R_u\) is short. — [POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf); listening-time limit also in [Radartutorial](https://www.radartutorial.eu/01.basics/en/rb09.en.html)
- Doppler blindness: frequencies repeat every PRF. A target whose \(f_D\) differs from the clutter notch by an integer number of PRFs lands in the notch after folding. The first speed that aliases to zero Doppler is an integer multiple of \(\lambda\,\mathrm{PRF}/2\), from \(f_D = 2v/\lambda\) together with sampling at the PRF. Low PRF therefore has many velocity blinds inside tactical speeds; high PRF moves those blinds outside typical closing speeds and pays with range blinds. — [Radartutorial Doppler dilemma](https://www.radartutorial.eu/01.basics/en/rb50.en.html); [Wu thesis](https://discovery.ucl.ac.uk/id/eprint/10123395/9/WU_10123395_Thesis.pdf); [POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- Clutter notch: mainlobe clutter is rejected or avoided as a Doppler region whose center moves with scan angle. A target with the ground’s radial speed occupies that region. Off the nose the region is wide compared with a low PRF, so much of the filter bank is blind in look-down. — [Jenn NPS notes](http://faculty.nps.edu/jenn/EC4610/Vol2v7.2.pdf); [POMR Vol. 3 retrieved passages](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)
- Multipath and atmospheric refraction shift the mean of the angle or range estimate. They are biases. Thermal noise widens the variance and, in the high-SNR monopulse formula, does not move the mean. — [POMR Vol. I, Ch. 18](https://www.scp.byu.edu/long/papers/chapters/POMR2011_Ch18.pdf)
- A long frame is a long \(T\) in the tracker. The Benedict–Bordner / tracking-index design exists because measurement noise and the time between updates both change the filter. Extrapolation across missed updates is already how the sim manual’s aging works; the physics version is covariance growth. — [DTIC ADA260831](https://archive.org/details/DTIC_ADA260831); aging behavior as manual UI in [VRS](https://forums.vrsimulations.com/support/index.php?title=A/A_Radar)

### Inferences

- **Doppler blind.** After folding, \(f_t\) equals the notched bins. Mechanisms: alias onto the mainlobe interval, or alias onto the altitude line. Changing PRF moves the blind speeds. A dwell that uses several PRFs should detect on any PRF where the folded tone is clear, and should report the target blind only if every PRF in the burst notches it. Coding “blind if speed is a multiple of 300 knots” will not track the carrier, the PRF, or the scan angle.
- **Clutter notch.** Same test as the clutter section. Failure is “no detection,” or a detection whose SINR used clutter power and lost the threshold. It is not a deletion of the target from the scenario. A target that is co-speed with the ground fails in look-down at any azimuth. A target that is co-speed with the fighter fails near zero Doppler (altitude line, and mainlobe clutter only near broadside).
- **Scan revisit too slow.** \(T_\mathrm{frame}\) large, target acceleration times \(T^2/2\) outside the association gate, hits rejected, coast, track drop. Mitigation that stays physical: shrink the scan (fewer bars or a narrower azimuth) or insert STT dwells. A larger \(\alpha\) without a larger measurement \(R\) is not a fix.
- **Multipath elevation error.** Consume the terrain note’s elevation bias and add it to the measured elevation before the filter. The bias can be a large fraction of a beamwidth at low grazing angle and does not average down like thermal \(\sigma_{el}\). If that note outputs nothing, do not invent a multipath lobe structure here; leave elevation unbiased except for thermal noise, and record the missing input. Line of sight still gates detection (masking and the direct path).
- **Range folding.** Store \(R_\mathrm{app} = (c_0/2)\,(t \bmod T)\) and \(R_u\). A weapon or a tracker that treats \(R_\mathrm{app}\) as truth will place the target on the wrong fold. Unfold only when a multi-PRF burst or a staggered PRI agrees. Maximum unambiguous range is not maximum detection range; SNR can still cross a threshold beyond \(R_u\), and the measurement is then folded. Eclipsing can knock out a target that has plenty of SNR because the echo arrived during the transmit blank.
- Each failure is monotonic in the inputs: LOS false, SNR too low after eclipsing and beamshape, or clutter power admitted into the cell by the Doppler/range test, or \(T_\mathrm{frame}\) too long for \(Q\). No mode should override a failed test with a scripted lock.

### Gaps

- A quantitative multipath elevation-error curve (lobe spacing versus height and range) belongs in the terrain note. It was not derived here.
- Detection curves \(P_d(\mathrm{SINR}, P_{fa}, N)\) such as Albersheim’s approximation were not opened. The failure model is a threshold on SINR; the numeric \(P_{fa}\) threshold is a parameter.
- Partial-eclipse SNR versus blanking overlap was inferred from the listening-time description, not taken from a measured eclipsing-loss table.

## Implementation notes for missilesim

Mode state machine, per-dwell math, and the data the detector is allowed to see. Do not hard-code a lock range, a notch in knots, or a constant angle error.

### State machine

States: `SEARCH_RWS`, `SEARCH_VS`, `TWS`, `ACM_SCAN`, `STT`, `FLOOD`, `COAST`.

- `SEARCH_RWS` and `SEARCH_VS` scan bars. Detections are plots. A tracker may run (the manual’s latent tracks) but does not steer the beam.
- `TWS` is the same scan with the tracker required and the volume usually smaller. Beam steering follows the scan, not the track, except for an optional electronic-scan insert.
- `ACM_SCAN` is a small volume, including boresight as a one-beam volume. The first detection that passes SINR, line of sight, and the clutter-cell test transitions to `STT`. No shorter lock range.
- `STT` steers the beam to the predicted line of sight every dwell. Break track, gimbal limit, or loss of line of sight goes to `COAST`.
- `COAST` predicts with \(Q\), issues no new measurement, and returns to the previous search state when coast time or covariance exceeds a limit. The manual’s 2/4/8/16 s aging steps are optional UI, not the default physics drop.
- `FLOOD` is illumination only, legal for a semi-active weapon after a launch if the design calls for it. No monopulse update. Exit returns to search. Beamwidth is a parameter.
- PRF schedule, bars, azimuth width, and dwell length are data on the state, not implicit in the state name. VS should select a high PRF and may omit unfolded range. RWS may select low or medium PRF. Look-down does not by itself change the state; it changes whether clutter power enters the cell.

### Per-dwell computations

Inputs this dwell may read from the range-equation note and the terrain/clutter note: `los`, `snr_boresight` (thermal, this dwell’s processing, eclipsing not included, clutter not included), `clutter_power_mainlobe`, optional `clutter_power_sidelobe`, optional `multipath_el_bias`. If `snr_boresight` is single-pulse, multiply by \(N\) once for the coherent Doppler filter and document that. If it is already integrated, do not.

For the dwell at time \(t_m\), beam unit vector \(\hat{u}\), wavelength \(\lambda\), aperture lengths, PRF list, \(\tau\) or \(B\), and \(N\):

1. \(\theta_{az}, \theta_{el}\) from \(\lambda\) and aperture, with the chosen \(k\).
2. Pattern scale: two-way power at the target’s angle off \(\hat{u}\). `snr = snr_boresight * pattern`.
3. \(R_u = c_0/(2\,\mathrm{PRF})\). Eclipsing fraction from overlap of the echo with the blank of width \(t_\mathrm{blank}\approx\tau_\mathrm{uncomp}\). Scale SNR by the received fraction. Full eclipse: no detection, reason `eclipsed`.
4. \(f_t = 2 v_r/\lambda\). \(f_\mathrm{MLC} = 2(\mathbf{V}_a\cdot\hat{u})/\lambda\). \(\Delta f_\mathrm{MLC} = (2 V_a/\lambda)|\sin\gamma|\,\Delta\gamma\) with \(\Delta\gamma\) the configured fraction of the beam (default the NPS \(2.5\,\theta_{3\mathrm{dB}}\) null-to-null approximation, overridable). \(f_\mathrm{alt} = 2 V_\mathrm{down}/\lambda\).
5. Fold tones by this PRF. Mainlobe conflict only if folded \(f_t\) hits the folded mainlobe interval plus one bin **and** the apparent range matches the mainlobe ground range. Altitude-line conflict is separate. On conflict, interference is `clutter_power_mainlobe`; else interference is `clutter_power_sidelobe` or zero. \(\mathrm{SINR} = \mathrm{SNR}\times N_0 / (N_0 + P_\mathrm{clutter})\) in the same power units the SNR was defined with.
6. If `los` is false, no detection. If SINR is below the threshold, no detection. Reasons: `masked`, `eclipsed`, `notch`, `snr`.
7. On detection, draw measurement noise from \(\sigma_\theta(\mathrm{SINR},\theta_3,k_m)\), \(\sigma_R \approx (c_0/(2B))/\sqrt{2\,\mathrm{SINR}}\), \(\sigma_{v_r} \approx (\lambda/(2 T_\mathrm{CPI}))/\sqrt{2\,\mathrm{SINR}}\). Add `multipath_el_bias` to elevation only. Report apparent range as \(R \bmod R_u\) unless the PRF set has a unique unfold. Report Doppler in the unambiguous interval actually used.
8. Tracker: predict to \(t_m\), gate with covariance, update \(R\) with those sigmas, or coast. \(T\) is \(T_\mathrm{dwell}\) in `STT` and \(T_\mathrm{frame}\) in search and TWS.
9. Frame advance: step the bar scan by about one beamwidth. \(T_\mathrm{frame} = N_\mathrm{bars}\,(\Omega_{az}/\theta_\mathrm{step})\,T_\mathrm{dwell}\).

### Data schema

```text
RadarModeParams
  wavelength_m, aperture_az_m, aperture_el_m, beamwidth_factor
  mlc_width_beams          # default 2.5; not a Hertz constant
  monopulse_slope_km       # in (1, 2)
  prf_hz[]                 # per state
  pulse_width_s, bandwidth_hz, n_pulses, n_cpi
  scan_az_rad, n_bars, beam_step_fraction
  sinr_threshold
  coast_max_s
  flood_beam_az_rad, flood_beam_el_rad   # used only in FLOOD

DwellInput
  t_s
  beam_az_rad, beam_el_rad, frame
  los                      # terrain / geometry note
  snr_boresight            # range-equation note
  clutter_power_mainlobe   # clutter note; 0 if the cell is not the mainlobe cell
  clutter_power_sidelobe   # optional
  multipath_el_bias_rad    # optional; 0 if the multipath note has no output
  r_m, v_radial_m_s, az_rad, el_rad   # truth, for the sim’s measurement draw only

DwellOutput
  detected
  fail_reason              # none | masked | eclipsed | notch | snr | ambiguous
  t_meas_s
  r_apparent_m, r_u_m, range_unfolded
  vr_apparent_m_s, doppler_hz, prf_hz
  az_rad, el_rad
  sigma_r_m, sigma_vr_m_s, sigma_az_rad, sigma_el_rad
  sinr
  mlc_center_hz, mlc_halfwidth_hz
  in_mlc, in_altitude_line, range_folded, eclipsed

Track
  t_s
  state[6]                 # x,y,z,vx,vy,vz
  covariance[6,6]
  last_meas_t_s, coast_count, hit_count

WeaponSupport
  t_meas_s
  r_m, vr_m_s, az_rad, el_rad
  sigma_r_m, sigma_vr_m_s, sigma_az_rad, sigma_el_rad
  sinr, coast_count, range_is_unfolded
  illumination             # {active, t0_s, t1_s, frequency_hz, waveform, beam_az, beam_el, beam_az_width, beam_el_width}
  datalink[]               # {t_tx_s, t_state_s, state[6], covariance}
```

`WeaponSupport.illumination.active` is true only while the target lies in the illuminator beam (STT narrow beam or FLOOD wide beam) and line of sight is true. `datalink` entries exist only at message times. Search plots do not set illumination. A 90-knot speed gate, a 12-trackfile cap, and display aging steps are optional manual-fidelity fields. They are not `mlc_halfwidth_hz`.
