# Radar detection physics for a real-time flight simulator

Scope: unclassified, open-literature models for turning physical radar and target inputs into a per-dwell signal-to-noise ratio (SNR) and a probability of detection \(P_d\). No named operational mode parameters and no countermeasure procedures. Each numeric claim is tagged **standard formula**, **computed from a standard formula**, **rule of thumb**, or **published typical** (handbook/textbook example, not a measurement campaign with an instrument chain).

Primary opened sources:

- Naval Air Warfare Center *Electronic Warfare and Radar Systems Engineering Handbook* (NAWC handbook), HTML reproduction: [monostatic equation](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation.htm), [bistatic equation](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation-bistatic.htm), [RCS](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-cross-section.htm), [duty cycle](https://www.rfcafe.com/references/electrical/ew-radar-handbook/duty-cycle.htm), [receiver noise](https://www.rfcafe.com/references/electrical/ew-radar-handbook/receiver-sensitivity-noise.htm), [horizon](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-horizon-line-of-sight.htm), [atmosphere](https://www.rfcafe.com/references/electrical/ew-radar-handbook/rf-atmospheric-absorption-ducting.htm).
- Mark A. Richards, “Alternative Forms of Albersheim’s Equation,” 20 June 2014 (PDF read in full): [radarsp.weebly.com](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/albersheim_alternative_forms.pdf).
- David K. Barton, *Radar Equations for Modern Radar* (Artech, 2013), publisher preview text: [pageplace preview](https://api.pageplace.de/preview/DT0400.9781608075225_A24132309/preview-9781608075225_A24132309.pdf). Same book’s detection-theory chapter as indexed at [picture.iczhiku.com copy](https://picture.iczhiku.com/resource/eetop/WhKGWITppyUkeCCX.pdf).
- Christian Wolff, *Radar Tutorial*, RCS page (quotes Skolnik’s simple-shape formulas and the Skolnik 1980 p. 44 table): [radartutorial.eu](https://www.radartutorial.eu/01.basics/en/rb56.en.html).
- Naval Postgraduate School NWDC EM course, Module 3.1 (4/3-earth and radar letter bands): [oc.nps.edu](https://www.oc.nps.edu/NWDC_EM_Course/course_materials/module3_1.html).
- ITU-R P.838-0 (1992) coefficient table, as returned from the recommendation PDF [R-REC-P.838-0](https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.838-0-199203-S!!PDF-E.pdf). The in-force revision is P.838-3 (2005); a direct fetch of the English P.838-3 PDF returned 404, so rain numbers below are **computed from the 1992 coefficients**, not from P.838-3.
- ITU-R P.676-13 (2022) specific-attenuation equation, as returned from the recommendation PDF [R-REC-P.676-13](https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.676-13-202208-I!!PDF-E.pdf). A direct download of that PDF was rejected by ITU, so the equation is from the indexed recommendation text, not a full re-read of the annexes.

Indexed textbook excerpts used where a full fetch was blocked (vdoc / search index, not a page-by-page read):

- Richards, Scheer, Holm, *Principles of Modern Radar*, Vol. I (SciTech, 2010), detection chapter: [vdoc record](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90).
- *Principles of Modern Radar*, Vol. III, radar-band application table: [ftp.idu.ac.id PDF](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf).
- Richards, *Fundamentals of Radar Signal Processing*, Ch. 6 excerpt: [richards_ch006_pr.pdf](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/richards_ch006_pr.pdf).

## Monostatic radar range equation (peak power, average power, pulse width, PRF, integration)

### Takeaway

For a point target the monostatic received **peak** power falls as \(1/R^4\). Pulse width and receiver bandwidth set the single-pulse SNR through the noise power \(kTBF\); PRF and dwell time set how many pulses can be integrated. Coherent integration multiplies SNR by the number of pulses; noncoherent integration does not.

### Cited Findings

- **Standard formula, SI, monostatic peak power at the receiver input** (NAWC handbook, log form quoted in text; linear form is the antilog). With frequency \(f\) in Hz, \(c\) in m/s, range \(R\) in m, RCS \(\sigma\) in m², powers in W, and antenna gains dimensionless:

  \[
  10\log P_r = 10\log P_t + 10\log G_t + 10\log G_r + 10\log\sigma - 20\log f - 40\log R - 30\log(4\pi) + 20\log c
  \]

  \[
  P_r = \frac{P_t G_t G_r \lambda^2 \sigma}{(4\pi)^3 R^4}, \quad \lambda = c/f
  \]

  The handbook states: keep \(\lambda\) or \(c\), \(\sigma\), and \(R\) in the same units; \(\lambda = c/f\). \(P_t\) in this equation “is the peak power of a CW or pulse signal.” For a pulse that is not square, “\(P_t\) is actually the average power within the pulse width.” — [NAWC monostatic section](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation.htm)

- **Symbols and SI units for that equation**

  | Symbol | Meaning | SI unit |
  | --- | --- | --- |
  | \(P_t\) | Peak transmit power during the pulse (average within the pulse if the pulse is not rectangular) | W |
  | \(P_r\) | Peak received power at the receiver input | W |
  | \(G_t, G_r\) | Transmit and receive antenna power gains relative to isotropic. Monostatic and colocated: often \(G_t = G_r\). The handbook says to fold transmission-line loss into the gain rather than add another term | dimensionless (W/W) |
  | \(\lambda\) | Carrier wavelength | m |
  | \(f\) | Carrier frequency | Hz in the SI form above. The handbook’s \(K_1\) charts are for mixed units (MHz or GHz with km) and must not be mixed with the SI form |
  | \(c\) | Speed of light | m/s |
  | \(\sigma\) | Target RCS in the backscatter direction, for the polarization in use | m² |
  | \(R\) | Slant range, radar to target | m |
  | \((4\pi)^3\) | Two spherical spreading factors plus the isotropic reradiation of the captured power | dimensionless when \(R\) is in m and \(\sigma\) in m² |

- **Losses are not inside the quoted log equation.** The same page says polarization mismatch and atmospheric absorption “also need to be included,” and that any transmission-line loss should be combined with antenna gain. A later ideal-form statement of the same equation (no loss factor) is Richards et al., *Principles of Modern Radar*, Vol. I, eq. (2.8): \(P_r = P_t G_t G_r \lambda^2 \sigma / ((4\pi)^3 R^4)\), after which “component, propagation, and signal processing losses” are introduced. — [NAWC monostatic](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation.htm); [POMR Vol. I, Ch. 2 index](https://vdoc.pub/documents/principles-of-modern-radar-volume-i-basic-principles-7amhdto8ej00)

- **Noise floor that \(P_r\) is compared with. Standard formula** (NAWC handbook, eq. [20], quoted in text):

  \[
  S_{\min} = (S/N)_{\min}\,(\mathrm{NF})\,k\,T_0\,B
  \]

  \(k = 1.38\times 10^{-23}\,\mathrm{J/K}\), \(T_0 = 290\,\mathrm{K}\) (“IEEE” convention in that handbook), \(B\) in Hz, NF the noise factor as a power ratio (not dB). Mean noise power of a real receiver is \((\mathrm{NF})\,k T_0 B\) watts. For \(T_0 = 290\,\mathrm{K}\) and \(B = 1\,\mathrm{Hz}\), \(kT_0B = -174\,\mathrm{dBm}\) (**standard convention** as stated there: \(-174\,\mathrm{dBm} + 10\log_{10}(B/\mathrm{Hz})\)). — [NAWC receiver section](https://www.rfcafe.com/references/electrical/ew-radar-handbook/receiver-sensitivity-noise.htm)

- **Pulse width and bandwidth. Rule of thumb from the same handbook:** “\(B\) is approximately equal to \(1/\mathrm{PW}\).” A wider pulse narrows the matched bandwidth and lowers \(S_{\min}\). A 1 µs pulse “requires a bandwidth of approximately 1 MHz.” One cannot set \(B\) independently of the transmitted waveform. Barton’s preview warns that Blake’s bandwidth correction \(C_b\) is unity at an optimum \(B\tau \approx 1.2\) and was defined for the **visibility factor** of a CRT display; applying that factor on top of an electronic detectability factor “is optimistic by ~2 dB.” — [NAWC monostatic](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation.htm); [Barton preview](https://api.pageplace.de/preview/DT0400.9781608075225_A24132309/preview-9781608075225_A24132309.pdf)

- **Duty cycle and average power. Standard formula** (NAWC duty-cycle section, equations quoted in text). Pulse repetition frequency \(\mathrm{PRF} = 1/T = 1/\mathrm{PRI}\). For rectangular pulses,

  \[
  P_{\mathrm{avg}}/P_{\mathrm{peak}} = \mathrm{PW}\times\mathrm{PRF} = \text{duty cycle}
  \]

  \[
  \text{duty cycle (dB)} = 10\log_{10}(\text{duty-cycle ratio})
  \]

  Worked example in the handbook: PRF = 1000 Hz, PW = 1.0 µs, duty cycle = 0.001 = 0.1% = −30 dB, so average power is 30 dB below peak. — [NAWC duty cycle](https://www.rfcafe.com/references/electrical/ew-radar-handbook/duty-cycle.htm)

- **Rule of thumb, handbook ranges, not a specific radar:** pulse radars, duty cycle 0.1–3% (−30 to −15 dB); pulse-Doppler, 5–50% (−13 to −3 dB); CW, 100% (0 dB). PRF bins used in that section: low 0.25–4 kHz, medium 8–40 kHz, high 50–300 kHz. IF bandwidth rules of thumb there: pulse 1–10 MHz; chirp or phase-coded pulse 0.1–10 MHz; CW or pulse-Doppler 0.1–5 kHz. — [NAWC duty cycle](https://www.rfcafe.com/references/electrical/ew-radar-handbook/duty-cycle.htm)

- **Coherent vs noncoherent integration.**
  - Barton’s energy-ratio form: the maximum available output SNR of a matched filter is the ratio of **input signal energy to noise spectral density during the observation time** on the target. “The input signal energy is proportional to the average power of the transmission, regardless of its waveform.” A chain of loss factors then produces the practical SNR. He contrasts this with a peak-power / bandwidth form plus “processing gains,” and warns that stacking those gains can predict an SNR higher than the matched-filter limit. — [Barton preview](https://api.pageplace.de/preview/DT0400.9781608075225_A24132309/preview-9781608075225_A24132309.pdf)
  - NAWC: raising PRF and integrating lets the receiver “pull coherent signals out of the noise thus reducing \(S/N_{\min}\),” unless the integration time exceeds time-on-target of a scanning antenna, or the PRI causes range ambiguities. “Integration will increase the S/N since the signal is coherent and the noise is not.” — [NAWC monostatic](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation.htm); [NAWC receiver](https://www.rfcafe.com/references/electrical/ew-radar-handbook/receiver-sensitivity-noise.htm)
  - Indexed statement of the coherent factor: the SNR at the output of a coherent integrator “is \(n\) times the SNR at the output of the matched filter.” For Swerling 2 or 4 (pulse-to-pulse fluctuation) “coherent integration does not increase SNR,” because the signal itself is uncorrelated pulse to pulse. — [Basic Radar Analysis, dokumen.pub index](https://dokumen.pub/basic-radar-analysis-9781608078783.html)
  - Noncoherent integration (sum of amplitudes or powers, phase discarded) is what Albersheim’s equation parametrizes. It does **not** improve SNR by \(N\). Richards: the equation estimates the single-sample SNR required when \(N\) samples are noncoherently integrated, for a nonfluctuating target and a linear detector. — [Richards 2014](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/albersheim_alternative_forms.pdf)

- **Search vs track, as the sources distinguish them.** The NAWC range equation that inserts \(S_{\min}\) is explicitly “for a tracking radar (target continuously in the antenna beam).” Separately: “With a scanning radar, there is loss if the receiver integration time exceeds the radar’s time on target.” Barton’s contents list a search-radar loss section and a beamshape loss \(L_p\). No numeric beamshape loss (the often-quoted ~1.6 dB) was present in the opened preview text, so none is recorded here. — [NAWC monostatic](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation.htm); [Barton preview TOC](https://api.pageplace.de/preview/DT0400.9781608075225_A24132309/preview-9781608075225_A24132309.pdf)

- **Bistatic (needed for a semi-active missile).** NAWC, equation quoted in text:

  \[
  10\log P_r = 10\log P_t + 10\log G_t + 10\log G_r + 10\log\sigma - 20\log f + 20\log c - 30\log(4\pi) - 20\log R_{Tx} - 20\log R_{Rx}
  \]

  So \(R^4\) becomes \(R_{Tx}^2 R_{Rx}^2\). “The most commonly encountered bistatic radar application is the semi-active missile”: transmitter on the launch platform, receiver in the missile. Bistatic RCS equals monostatic RCS only when transmit and receive antennas are on the same line of sight; in general RCS “varies with angle,” including elevation. — [NAWC bistatic](https://www.rfcafe.com/references/electrical/ew-radar-handbook/two-way-radar-equation-bistatic.htm)

### Inferences

- Combining the quoted \(P_r\) equation with \(S_{\min} = (S/N) k T_0 B\,(\mathrm{NF})\) and the handbook rule \(B \approx 1/\tau\) (\(\tau =\) pulse width) gives the single-pulse matched-filter SNR (**inference**, not a single quoted equation):

  \[
  \mathrm{SNR}_1 = \frac{P_t G_t G_r \lambda^2 \sigma\,\tau}{(4\pi)^3 R^4\, k T_0\,(\mathrm{NF})\, L}
  \]

  \(L \ge 1\) is the extra dimensionless loss (polarization, plumbing if not already in \(G\), signal processing). Atmosphere is range-dependent and should stay outside \(L\), otherwise \(R_{\max}\) is implicit.

- Barton’s energy statement plus \(P_{\mathrm{avg}} = P_t \tau\,\mathrm{PRF}\) gives the coherent dwell form (**inference**): if \(N = \mathrm{PRF}\,T_{\mathrm{obs}}\) pulses are coherently integrated and the target amplitude is constant over \(T_{\mathrm{obs}}\),

  \[
  \mathrm{SNR}_{\mathrm{coh}} = \frac{P_{\mathrm{avg}} T_{\mathrm{obs}} G_t G_r \lambda^2 \sigma}{(4\pi)^3 R^4\, k T_s\, L} = N\cdot\mathrm{SNR}_1
  \]

  up to the coherent processing loss the Basic Radar Analysis excerpt writes as \(L_{ci}\). Use system noise temperature \(T_s\) here when it is known; the NF form is \(T_s = T_0\,(\mathrm{NF})\) only when antenna noise has already been absorbed into an effective noise factor referred to the same point.

- Average-power form without a stated dwell does **not** replace the peak-power form. \(P_{\mathrm{avg}}\) enters only after it is multiplied by an integration time. Using \(P_{\mathrm{avg}}\) in the peak equation understates SNR by \(1/\mathrm{duty\ cycle}\).

- Search dwell: \(T_{\mathrm{obs}}\) cannot exceed time-on-target. For a mechanically scanned beam, time-on-target is set by beamwidth and scan rate (both data). A track dwell can be longer because the beam stays on the target; the SNR formula is the same, only \(N\) or \(T_{\mathrm{obs}}\) changes. That is the entire search/track distinction the opened sources support. A separate “search radar equation” in power-aperture form was not quoted from an opened page and is not required if dwell time is an input.

### Gaps

- The NAWC tracking-radar equation [21] and the antenna-gain definition \(G = 4\pi A_e/\lambda^2\) are images on the handbook pages, so they were not transcribed from pixels. The log monostatic equation above was text.
- No opened page gave a numeric beamshape loss, a numeric integration-efficiency table versus per-pulse SNR, or Blake’s full system-temperature worksheet (antenna temperature, line loss, and receiver temperature as separate entries). Barton only states that \(T_s\) is referred to the antenna output terminal and includes antenna noise, receive-line loss, and the receiver.
- Noncoherent integration **gain in dB** is not a fixed fraction of \(10\log_{10} N\). Use Albersheim (next section) rather than a \(\sqrt{N}\) rule. A \(\sqrt{N}\) rule appeared only on a secondary RF-glossary page and is not used here.

## Noise figure, bandwidth, losses, temperature, and the detectability factor

### Takeaway

SNR is received energy (or power) divided by thermal noise. Noise figure, bandwidth, temperature, and loss all sit in that denominator. \(P_d\) and \(P_{fa}\) do not appear in the range equation; they appear as a required SNR called the detectability factor. For one noncoherently integrated sample at \(P_{fa} = 10^{-6}\), that factor is about 11.2 dB at \(P_d = 0.5\) and about 13.2 dB at \(P_d = 0.9\) for a steady target, and about 21 dB at \(P_d = 0.9\) for a Swerling I target.

### Cited Findings

- **Where each term enters.** From the NAWC \(S_{\min}\) equation: linear noise factor NF multiplies noise power; bandwidth \(B\) (Hz) multiplies noise power; \(T_0 = 290\,\mathrm{K}\) is the reference temperature inside \(kT_0B\), not the outside air temperature. A passive lossy network has noise factor equal to its power loss. Cascaded stages follow a Friis combination (the handbook prints the formula as a figure: a preamp belongs at the antenna, ahead of cable loss). System losses that are not thermal (beamshape, mismatch, quantization) are extra factors on the signal, not a change to \(kT_0\). — [NAWC receiver](https://www.rfcafe.com/references/electrical/ew-radar-handbook/receiver-sensitivity-noise.htm)

- **Blake chart / detectability factor.** Barton, from the opened preview:
  - Electronic detection uses a **detectability factor** \(D(n)\), the SNR required for a stated \(P_d\) at a stated \(P_{fa}\), in place of the older **visibility factor** \(V(n)\) used for a human operator on an A-scope or PPI.
  - The “Pulse-Radar Range-Calculation Worksheet,” the Blake chart, solves the range equation by adding and subtracting decibels. Part (1) builds system noise temperature \(T_s\). Part (2)–(3) sum the remaining factors to \(40\log R_0\).
  - The chart is filled with a **median** RCS \(\sigma_{50}\) and \(V_0(50)\) to get the range \(R_{50}\) at \(P_d = 50\%\). For Swerling Case 1 the **average** RCS “is 1.5 dB greater than the median.”
  - Free-space range is then multiplied by the pattern-propagation factor \(F\), and atmospheric attenuation is iterated because the loss depends on the range just solved. Iteration “may be required when the attenuation coefficient … is large, as in some millimeter-wave radar cases or in microwave radar when precipitation is present.”
  - \(T_s\) “is referred to the output terminal of the receiving antenna” and includes antenna thermal noise, receiving line loss, and receiver noise. — [Barton preview](https://api.pageplace.de/preview/DT0400.9781608075225_A24132309/preview-9781608075225_A24132309.pdf)

- **Steady-target (Swerling 0 / Marcum) operating points.**
  - Indexed Barton sentence: single-pulse steady-target detectability factor \(D_0 = 11.2\,\mathrm{dB}\) at \(P_d = 50\%\), \(P_{fa} = 10^{-6}\); the corresponding single-pulse **visibility** factor is \(V_0(50) = 13.2\,\mathrm{dB}\) (about 2 dB worse, operator vs threshold). — [Barton detection chapter index](https://picture.iczhiku.com/resource/eetop/WhKGWITppyUkeCCX.pdf)
  - POMR Vol. I prose: for a nonfluctuating target, \(P_d = 90\%\) at \(P_{fa} = 10^{-6}\) needs “an SNR of about 13.2 dB.” A lecture note that tells the reader to read the same classical curve says the graph gives 13.2 dB at that point. — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90); [GPCET RS Unit 5](http://www.gpcet.ac.in/wp-content/uploads/2018/08/RS-Unit-5.pdf)
  - **Computed from Albersheim’s equation** as printed by Richards (2014), \(N = 1\), \(P_{fa} = 10^{-6}\): \(P_d = 0.5\) gives \(11.23\,\mathrm{dB}\); \(P_d = 0.9\) gives \(13.11\,\mathrm{dB}\). An indexed worked example in *FMCW Radar Design* states that the same point (\(P_d = 0.9\), \(P_{fa} = 10^{-6}\)) “will be 13.15 dB.” Richards states the approximation error is less than 0.2 dB for \(P_{fa}\) from \(10^{-3}\) to \(10^{-7}\), \(P_d\) from 0.1 to 0.9, and \(N\) from 1 to 8096. The 11.2 / 13.2 dB textbook readings sit inside that band of the formula. — [Richards 2014](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/albersheim_alternative_forms.pdf); [FMCW Radar Design index](https://vdoc.pub/documents/fmcw-radar-design-10c20mtskn3o)

- **Albersheim’s equation, quoted** (Richards 2014, eq. 1). \(\chi_{\mathrm{dB}}\) is the **single-sample** SNR in dB; \(N\) is the number of noncoherently integrated samples; natural log; nonfluctuating target; linear detector:

  \[
  A = \ln\left(\frac{0.62}{P_{fa}}\right), \quad
  B = \ln\left(\frac{P_d}{1-P_d}\right)
  \]

  \[
  \chi_{\mathrm{dB}} = -5\log_{10} N + \left(6.2 + \frac{4.54}{\sqrt{N+0.44}}\right)\log_{10}(A + 0.12AB + 1.7B)
  \]

  Inverse, Richards eq. (2), for turning a measured SNR into \(P_d\):

  \[
  A = \ln\left(\frac{0.62}{P_{fa}}\right), \quad
  Z = \frac{\chi_{\mathrm{dB}} + 5\log_{10} N}{6.2 + 4.54/\sqrt{N+0.44}}, \quad
  B = \frac{10^{Z} - A}{1.7 + 0.12A}, \quad
  P_d = \frac{1}{1+e^{-B}}
  \]

  Original fit: Albersheim, “Closed-Form Approximation to Robertson’s Detection Characteristics,” *Proceedings of the IEEE*, vol. 69, no. 7, p. 839, July 1981, as cited by Richards. — [Richards 2014](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/albersheim_alternative_forms.pdf)

- **Swerling cases, as named by POMR.** Two RCS pdfs (Rayleigh, and chi-square with 4 degrees of freedom) and two rates (dwell-to-dwell, pulse-to-pulse): Case 1 Rayleigh and slow; Case 2 Rayleigh and pulse-to-pulse; Case 3 fourth-degree chi-square and slow; Case 4 fourth-degree chi-square and fast. Nonfluctuating is Swerling 0 or Marcum. POMR prose: at \(P_d = 90\%\), \(P_{fa} = 10^{-6}\), a fluctuating target needs “17.1 to 21 dB” rather than 13.2 dB. The indexed table header shows SW0 at that \(P_{fa}\) starting \(11.1\) and \(13.2\) under columns labeled 50 and 90; the rest of the \(10^{-6}\) row was truncated in the index, so SW1/SW3 cells from that table are **not** copied here. — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90)

- **Swerling I, one sample, exact. Standard formula.** POMR eq. (3.23): \(P_d = (P_{fa})^{1/(1+\mathrm{SNR})}\), SNR linear, average power, white noise, one sample. Equivalent Barton form, eq. (4.27): single-pulse Case-1 detectability factor \(D_{11} = \ln(P_{fa})/\ln(P_d) - 1\). Richards Ch. 6: for \(N = 1\), \(P_{fa} = \exp(-T')\) and \(P_d = \exp[-T'/(1+\chi)]\), which is the same relation; he marks it exact. — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90); [Barton detection chapter index](https://picture.iczhiku.com/resource/eetop/WhKGWITppyUkeCCX.pdf); [Richards Ch. 6 index](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/richards_ch006_pr.pdf)

- **Computed from that Swerling I formula** at \(P_{fa} = 10^{-6}\): \(P_d = 0.5\) requires SNR \(= 18.93\) (12.77 dB); \(P_d = 0.9\) requires SNR \(= 130.1\) (21.14 dB). The 21.1 dB point is the top of POMR’s “17.1 to 21 dB” fluctuating band. These are formula evaluations, not measured SNRs.

- **Swerling III model class only.** POMR: one-dominant-plus-many-small scatterers, chi-square of degree 4, slow (Case 3) or fast (Case 4). At high \(P_d\) the required SNR lies below Swerling I and above Swerling 0; POMR’s 17.1 dB end of the range is consistent with that ordering but the index extract does not say “17.1 dB is SW3,” so that assignment is not made. Shnidman’s equation is the usual closed approximation for SW0–SW4 and noncoherent \(N\); Richards points to it and the indexed POMR/FMCW pages print a coefficient chain, but the extract is too broken to re-implement without a clean copy. — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90); [Richards 2014 footnote](https://radarsp.weebly.com/uploads/2/1/4/7/21471216/albersheim_alternative_forms.pdf)

- **Fluctuation vs threshold, qualitative, from an opened lecture note.** For \(P_d > 0.4\), a fluctuating target needs more single-pulse SNR than a constant target. For \(P_d < 0.4\) it needs less, because some echoes exceed the average. — [GPCET RS Unit 5](http://www.gpcet.ac.in/wp-content/uploads/2018/08/RS-Unit-5.pdf)

### Inferences

- A simulator should compute SNR from power, gain, wavelength, RCS, range, \(T_s\) or NF, bandwidth or pulse width, and losses, then convert SNR to \(P_d\). The Blake-chart idea is that conversion: \(D(P_d, P_{fa}, N, \text{Swerling model})\) is data or a function, never a hard-coded range.
- Do not apply both a Swerling draw on RCS **and** a Swerling \(P_d\) formula. The \(P_d\) formula is already the average over the RCS distribution. A random RCS draw belongs with the steady-target (Albersheim) curve evaluated at the instantaneous SNR.
- For a slow Swerling I target, one dwell of many pulses that all see the same RCS is not \(N\) independent Swerling I samples. Coherently or noncoherently integrating those pulses uses the steady-target integrator on a frozen amplitude; the fluctuation is applied from dwell to dwell. Pulse-to-pulse cases are Swerling II and IV. This matches the POMR rate labels, not an extra formula.

### Gaps

- No clean opened table of Swerling III SNR at \(P_d = 0.5\) and \(0.9\), \(P_{fa} = 10^{-6}\). The 17.1–21 dB POMR sentence is the only fluctuating numeric band tied to that operating point in the extracts.
- Shnidman’s equation was not copied: the available text is garbled. It is the right extension beyond Albersheim (SW0 only) and the one-sample SW1 formula.
- Blake’s five-input recipe for \(T_s\) (sky temperature, rain noise, ohmic loss, receiver noise figure) was named but not printed in the preview pages that were retrieved. The NF form at \(T_0 = 290\,\mathrm{K}\) is the fallback the NAWC handbook actually writes out.
- Antenna-noise temperature versus elevation, and the difference between noise figure referenced to the receiver input versus \(T_s\) referenced to the antenna terminal, will be wrong by the receive-line loss if those reference points are mixed. The opened pages flag the issue and do not give a numeric aircraft-install example worth copying.

## Radar cross section

### Takeaway

RCS is an equivalent area, not the physical silhouette: the power the target scatters back toward the radar, expressed as the area that would intercept the incident power density and reradiate it isotropically. Aircraft RCS is aspect-dependent by tens of decibels; frequency and polarization at one aspect move it by only a few decibels in the 3–18 GHz handbook discussion. Simple shapes have standard frequency laws; a fighter does not.

### Cited Findings

- **Definition, power form, opened in full.** Radar Tutorial:

  \[
  \sigma = 4\pi r^2 \frac{S_r}{S_t}
  \]

  \(\sigma\) in m², \(S_t\) the incident power density at the target (W/m²), \(S_r\) the scattered power density at the receiver (W/m²). The \(r^2\) cancels free-space spreading so \(\sigma\) is a target property in the far field. The same page’s power-balance discussion is the finite-range version of the far-field limit. — [Radar Tutorial RCS](https://www.radartutorial.eu/01.basics/en/rb56.en.html)

- **Definition, field form, as printed in an indexed textbook passage citing Knott.** Richards-style wording on Access Engineering, attributed explicitly to Knott, Shaeffer, and Tuley, *Radar Cross Section* (1993):

  \[
  \sigma = 4\pi \lim_{R\to\infty}\left[R^2 \frac{|E_b|^2}{|E_t|^2}\right]\ \mathrm{m}^2
  \]

  \(E_b\) backscattered field, \(E_t\) incident field. The page says the limit removes range so the result depends only on the scatterer. This is the user’s requested definition. The page was indexed, not opened as full text (paywalled). — [Access Engineering, Modeling Amplitude, eq. (2.36)](https://www.accessengineeringlibrary.com/content/book/9781260468717/toc-chapter/chapter2/section/section3)

- **NAWC handbook wording of the same idea:** RCS “is a measure of the ratio of backscatter power per steradian … to the power density that is intercepted by the target,” and “\(\sigma =\) projected cross section × reflectivity × directivity.” Directivity is the ratio of power scattered toward the radar to the power that would have been scattered if the scattering were isotropic. — [NAWC RCS](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-cross-section.htm)

- **Polarization.** Radar Tutorial: RCS depends on geometry, aspect, transmitter frequency, and “the electrical properties of the target’s surface.” CopRadar, citing the same microwave context: RCS depends on frequency and on polarization (vertical, horizontal, circular); different polarizations produce different patterns. The scalar definitions above are for one transmit/receive polarization pair. A full polarization scattering matrix was not printed in any opened page. — [Radar Tutorial](https://www.radartutorial.eu/01.basics/en/rb56.en.html); [CopRadar vehicle reflection](https://copradar.com/chapts/chapt3/ch3d6.html)

- **Simple shapes, optical region (body large compared with \(\lambda\)), standard formulas from Radar Tutorial**, normal incidence, far field:

  | Shape | Formula | Frequency dependence |
  | --- | --- | --- |
  | Sphere | \(\sigma = \pi r^2\) | none, in this region |
  | Cylinder | \(\sigma_{\max} = 2\pi r h^2 / \lambda\) | \(\propto 1/\lambda\) |
  | Flat plate | \(\sigma_{\max} = 4\pi b^2 h^2 / \lambda^2\) | \(\propto 1/\lambda^2\) (i.e. \(\propto f^2\)) |

  A plate that is not normal to the line of sight uses the projected area in that formula, “but the reflected energy is reflected in a different direction,” so a monostatic radar does not receive the specular beam. — [Radar Tutorial](https://www.radartutorial.eu/01.basics/en/rb56.en.html)

- **Sphere regions. Textbook description of the classical curve, NAWC handbook.** Optical-region rules “apply when \(2\pi r/\lambda > 10\)”; there \(\sigma = \pi r^2\) and is independent of frequency. In the Mie (resonance) region the specular return and creeping waves interfere. The handbook’s Figure 7 discussion: the largest positive excursion is about 4 times the optical RCS, and a nearby minimum is about 0.26 times the optical RCS. Example given there: a 6-inch-diameter sphere has that resonance near 0.6 GHz; “any frequency ten times higher, or above 6 GHz, would give expected results.” A 1 m diameter sphere moves the same features down to about 95 MHz. — [NAWC RCS](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-cross-section.htm)

- **Calibration magnitudes. Published practice / geometry, not a flight measurement.**
  - A sphere whose optical RCS is 1 m² has projected area 1 m². Radar Tutorial: diameter approximately 1.128 m. NAWC: “diameter of about 44 in” (44 in = 1.12 m) for the 1 m² reference sphere.
  - Towed calibration spheres cited by NAWC, with the reference RCS equal to the optical projected area: 6 in → 0.018 m², 14 in → 0.099 m², 22 in → 0.245 m². The handbook warns that if \(\lambda\) is not much smaller than the radius, creeping-wave error remains when these are scaled to a 1 m² reference.
  - Corner reflector: Radar Tutorial’s reproduction of Skolnik, *Introduction to Radar Systems*, 2nd ed., McGraw-Hill, 1980, p. 44, gives 20379 m² (43.1 dBsm) and states that this example has edge length 1.5 m, in a table of X-band point-target examples. That number is a textbook example, not a universal corner-reflector RCS. — [Radar Tutorial](https://www.radartutorial.eu/01.basics/en/rb56.en.html); [NAWC RCS](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-cross-section.htm)

- **Other Skolnik p. 44 entries reproduced by Radar Tutorial** (same citation): bird 0.01 m² (−20 dBsm), man 1 m² (0 dBsm), cabin cruiser 10 m², automobile 100 m², truck 200 m². These are the rows whose numbers were present in the fetched page.

- **Aircraft, why it is statistical and aspect-dependent.** NAWC: “The RCS of real aircraft must be measured. It varies significantly depending upon the direction of the illuminating radar.” In “the normal radar range of 3–18 GHz, the radar return of an aircraft in a given direction will vary by a few dB as frequency and polarization vary (the RCS may change by a factor of 2–5).” The handbook’s example azimuth cut (zero elevation) has a strongest return of 100 m² on the beam and a weakest “slightly more than 1 m²” near 135°/225°. Nose and tail are next highest “largely because of reflections off the engines or propellers.” A sphere is “essentially the same in all directions”; a flat plate has “almost no RCS except when aligned directly toward the radar”; a corner reflector holds a high RCS over roughly ±60°. — [NAWC RCS](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-cross-section.htm)

- **Published typical magnitudes (handbook ranges, not measurements of a named aircraft):** “Missile 0.5 m²; tactical jet 5 to 100 m²; bomber 10 to 1000 m²; ships 3000 to 1,000,000 m².” The same page says the example plot can exceed the upper end on the beam (a bomber “may be much greater than 1000 square meters” at 90°/270°) and that phase, polarization, surface imperfections, and material “all greatly affect the results.” — [NAWC RCS](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-cross-section.htm)

- **Swerling as the statistical model of that complexity.** Many comparable scatterers → Rayleigh RCS (Swerling 1 if the draw holds for a dwell/scan, Swerling 2 if it changes every pulse). One dominant scatterer plus many small ones → 4-degree chi-square (Swerling 3 / 4). POMR: the models describe the echo the radar sees, not an intrinsic label of the airframe, and they change if frequency or polarization diversity changes the fading. — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90)

### Inferences

- For a simulator, store a mean RCS versus aspect (and, if the fidelity is there, versus frequency band and polarization). Apply Swerling fluctuation about that mean. A single “fighter RCS” scalar reproduces neither the handbook’s 1-to-100 m² aspect cut nor the 5-to-100 m² typical band.
- Frequency dependence of a real aircraft is weak next to aspect in the handbook’s 3–18 GHz statement (a factor of 2–5, i.e. about 3–7 dB, at fixed aspect). Specular pieces (plates, inlets) still follow a strong \(1/\lambda^2\) law and a narrow angular lobe; that is why the azimuth cut is spiky. Higher carrier frequency packs more lobes into the same mechanical angle because the body is electrically larger. That lobe-density claim is an inference from “RCS depends on the ratio of the structural dimensions of the body to the wavelength” (Radar Tutorial) together with the plate formula; no opened page gave a scintillation bandwidth versus frequency.
- Optical-region sphere RCS is the right calibration target at X/Ku/Ka for the handbook’s 6-inch-and-larger spheres. A sphere comparable to \(\lambda\) is not \(\pi r^2\).

### Gaps

- Dipole RCS versus frequency was not on any page that was opened. No \(\sigma(\lambda)\) for a half-wave dipole is recorded.
- The trihedral corner formula (the usual \(4\pi a^4/(3\lambda^2)\) or similar) was not printed next to the 20379 m² example. Use the published example as an example, not as a formula.
- Knott’s polarization scattering matrix and the formal far-field limit were not available as full text. The power-density definition that was opened is the one to code; it matches the field definition when power density is proportional to \(|E|^2\).
- CopRadar attributes an extended Skolnik p. 44 list (large fighter 6 m², small fighter 2 m², conventional winged missile 0.5 m², and others) to the same page Radar Tutorial cites. The fetched CopRadar HTML did not contain those cell values (empty table cells). They are not used. The missile 0.5 m² figure does reappear as a NAWC “typical,” which is the number retained.
- No opened measured RCS pattern of a specific fighter, with frequency, polarization, and aspect, was used. The 5–100 m² tactical-jet band is a handbook typical range.

## Frequency bands used by fire-control radars and missile seekers

### Takeaway

IEEE radar letter bands put airborne fire-control in X (8–12 GHz) and missile seekers and close-in fire-control in Ka (27–40 GHz), with Ku (12–18 GHz) used for finer airborne surface modes. K (18–27 GHz) is the water-vapor absorption band and is called out as limited. Shorter wavelength narrows the beam and raises gain for a given aperture, speeds angular RCS lobing, and raises rain loss sharply.

### Cited Findings

- **Band edges, opened page (NPS), consistent with the usual IEEE radar letter bands:**

  | Band | Frequency (GHz) |
  | --- | --- |
  | L | 1–2 |
  | S | 2–4 |
  | C | 4–8 |
  | X | 8–12 |
  | Ku | 12–18 |
  | K | 18–27 |
  | Ka | 27–40 |
  | Millimeter | 40–300 |

  NPS notes these are the bands equipment is named by, in place of a frequency. — [NPS Module 3.1](https://www.oc.nps.edu/NWDC_EM_Course/course_materials/module3_1.html)

- **What the bands are used for. Indexed table, *Principles of Modern Radar*, Vol. III, Table 1-1** (same edges as NPS for X through Ka):
  - X, 8–12 GHz: “Fire-control radar, air interceptor radar, ground-mapping radar, ballistic missile–tracking radar.”
  - Ku, 12–18 GHz: “Air-to-ground SAR and surface-moving target indication.”
  - K, 18–27 GHz: “Limited due to absorption.”
  - Ka, 27–40 GHz: “Missile seekers, close-range fire-control radar.”
  - Millimeter wave, 40–300 GHz: “Fire-control radar,” plus automotive, imaging, and instrumentation uses.
  - The same chapter notes that X-band parts are common, so Ku-band hardware is harder to justify on cost unless something else (resolution) requires it. — [POMR Vol. III index](https://ftp.idu.ac.id/wp-content/uploads/ebook/tdg/ADNVANCED%20MILITARY%20PLATFORM%20DESIGN/Principles%20of%20Modern%20Radar.%20Volume%20%203.pdf)

- **Wavelengths at the band centers (standard formula \(\lambda = c/f\), \(c = 3\times 10^8\,\mathrm{m/s}\)):** X at 10 GHz → 3.0 cm; Ku at 15 GHz → 2.0 cm; Ka at 35 GHz → 8.6 mm. These are labels for the loss tables below, not allocated channels.

- **Why the band changes the link, from opened physics rather than from a slogan:**
  - **Beamwidth and gain.** The range equation carries \(G_t G_r \lambda^2\). For a fixed physical aperture, gain rises and the beam narrows as wavelength shrinks. The explicit \(G = 4\pi A_e/\lambda^2\) line was an image in the NAWC handbook and is not transcribed; the sim should take gain and beamwidth as data (see gaps). POMR’s assignment of X to fire-control and Ka to seekers is the engineering consequence: a small aperture can still form a narrow beam.
  - **RCS scintillation.** Radar Tutorial: simple-body RCS depends on size measured in wavelengths. NAWC: over 3–18 GHz, fixed-aspect RCS moves by a factor of about 2–5, while aspect moves the example jet from just over 1 m² to 100 m². Plate specular RCS grows as \(1/\lambda^2\) and only in a narrow angular lobe.
  - **Weather loss.** Specific rain attenuation rises by more than an order of magnitude from 10 GHz to 35 GHz at the same rain rate (numbers in the next section). K-band is singled out as absorption-limited. NPS: tropospheric **refraction** from VHF through SHF does **not** depend on frequency; gaseous and rain absorption do.

- **Tropospheric refraction is not why one picks X vs Ka.** NPS: “Frequency Dependence of Refraction in VHF to SHF bands: Good news: there is none!” The 4/3-earth model can be shared across these seeker bands. Absorption and beamwidth cannot. — [NPS Module 3.1](https://www.oc.nps.edu/NWDC_EM_Course/course_materials/module3_1.html)

### Inferences

- A fire-control or seeker record needs a frequency (or \(\lambda\)), not a band name. The band name is only a check against the table above.
- Air-to-air X-band paths are only weakly hurt by clear-air gas; Ka-band paths, and any path through heavy rain, are not. Coding one “radar propagation loss” for all bands will not reproduce that.
- Wavelength also sets Doppler scale (\(2v/\lambda\)) and the physical size of a range cell only through bandwidth, not through the carrier. Those are outside this note except as a reminder not to tie range resolution to the band letter.

### Gaps

- IEEE Std 521 was not opened. Band edges used here are the NPS table, which matches the POMR Vol. III table and the usual IEEE radar letters. Waveguide bands (for example X as 8.2–12.4 GHz) are a different convention and were not used.
- No opened page listed a specific seeker as Ku-band versus Ka-band with a citation. The POMR table is the application statement: Ka for missile seekers, X for fire-control, Ku for airborne SAR/GMTI.
- The numerical beamwidth constant (the factor in \(\theta \approx k\lambda/D\)) was not opened. Do not invent \(k\). Store beamwidth.

## Atmospheric attenuation (oxygen, water vapor, rain)

### Takeaway

ITU specific attenuation is a **one-way** loss in dB/km. A monostatic radar pays it twice. Clear-air gas at X-band is a small correction; rain at X-band is tenths of a dB per km one way in heavy rain and several dB per km at Ka. The in-force rain model is P.838-3; the numbers below are computed from the superseded P.838-0 (1992) coefficients because the 2005 PDF could not be fetched.

### Cited Findings

- **Gases. Standard formula, ITU-R P.676-13 eq. (1),** indexed from the recommendation PDF (direct download was rejected):

  \[
  \gamma = \gamma_o + \gamma_w = 0.1820\, f\, \bigl(N''(f)_{\mathrm{oxygen}} + N''(f)_{\mathrm{water\ vapour}}\bigr)
  \quad (\mathrm{dB/km})
  \]

  \(f\) in GHz. \(\gamma_o\) is dry air (oxygen, pressure-induced nitrogen, non-resonant Debye spectrum of oxygen below 10 GHz). \(\gamma_w\) is water vapour, including a wet continuum. The recommendation says the specific attenuation up to 1000 GHz is a sum of oxygen and water-vapour spectral lines, and that Figure 1 is that sum at 1013.25 hPa, 15 °C, for 7.5 g/m³ water vapour (“Standard”) and for a dry atmosphere. If there is no local profile, use the reference atmosphere in ITU-R P.835. — [R-REC-P.676-13](https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.676-13-202208-I!!PDF-E.pdf)

- **No clear-air numeric point is recorded.** Figure 1 was not readable in the index extract. A secondary thesis states “approximately 0.01 dB/km” at 10 GHz from P.676-8 at 20 °C, 1013 hPa, 7.5 g/m³; that PDF could not be opened (access wall), so **0.01 dB/km is not adopted**. NAWC only says that below 10 GHz gaseous attenuation “is reasonably predictable,” and that in the millimeter-wave range it increases and depends strongly on H₂O and O₂, with a figure (not transcribed) of absorption peaks. — [NAWC atmosphere](https://www.rfcafe.com/references/electrical/ew-radar-handbook/rf-atmospheric-absorption-ducting.htm)

- **Rain. Standard formula, ITU-R P.838** (same power law in the 1992 and 2005 texts):

  \[
  \gamma_R = k R^{\alpha} \quad (\mathrm{dB/km})
  \]

  \(R\) is rain rate in mm/h. \(k\) and \(\alpha\) depend on frequency and polarization. P.838-0 (1992) Table 1, horizontal paths, linear polarization, coefficients “tested and found reliable for frequencies up to about 40 GHz”:

  | \(f\) (GHz) | \(k_H\) | \(\alpha_H\) | \(k_V\) | \(\alpha_V\) |
  | --- | --- | --- | --- | --- |
  | 8 | 0.00454 | 1.327 | 0.00395 | 1.310 |
  | 10 | 0.0101 | 1.276 | 0.00887 | 1.264 |
  | 12 | 0.0188 | 1.217 | 0.0168 | 1.200 |
  | 15 | 0.0367 | 1.154 | 0.0335 | 1.128 |
  | 20 | 0.0751 | 1.099 | 0.0691 | 1.065 |
  | 35 | 0.263 | 0.979 | 0.233 | 0.963 |

  For other geometries P.838-0 combines \(k_H\) and \(k_V\) with path elevation and polarization tilt. Values between tabulated frequencies: log-frequency interpolation, log \(k\), linear \(\alpha\). — [R-REC-P.838-0](https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.838-0-199203-S!!PDF-E.pdf)

- **Computed from that table** (one-way \(\gamma_R\), horizontal polarization). Not measurements. Rain rate is an input, not a label: 25 mm/h and 50 mm/h are the heavy-rain samples requested; the recommendation itself does not define the word “heavy” in the fetched text.

  | Band sample | \(f\) | 1 mm/h | 25 mm/h | 50 mm/h |
  | --- | --- | --- | --- | --- |
  | X, low | 8 GHz | 0.0045 dB/km | 0.325 dB/km | 0.816 dB/km |
  | X, mid | 10 GHz | 0.010 dB/km | 0.614 dB/km | 1.49 dB/km |
  | X/Ku edge | 12 GHz | 0.019 dB/km | 0.945 dB/km | 2.20 dB/km |
  | Ku | 15 GHz | 0.037 dB/km | 1.51 dB/km | 3.35 dB/km |
  | K | 20 GHz | 0.075 dB/km | 2.58 dB/km | 5.53 dB/km |
  | Ka | 35 GHz | 0.263 dB/km | 6.15 dB/km | 12.1 dB/km |

  Vertical polarization is lower by roughly 10–20% at these frequencies (the \(k_V, \alpha_V\) pair). Example: 10 GHz, 25 mm/h, vertical, \(\gamma_R = 0.519\,\mathrm{dB/km}\).

- **Two-way.** P.838’s \(\gamma_R\) is specific attenuation along the path, i.e. one way. The NAWC monostatic derivation applies the one-way space loss twice and says atmospheric absorption must be included as well. **Inference:** monostatic gaseous or rain loss in dB is \(2 \int \gamma\, ds\) through the absorbing medium, and bistatic loss is the sum of the two legs. At 10 GHz and 25 mm/h, horizontal, the two-way rate is \(1.23\,\mathrm{dB/km}\) of rainy path; at 35 GHz and 25 mm/h it is \(12.3\,\mathrm{dB/km}\). A 10 km monostatic path entirely in 25 mm/h rain is then about 12 dB at 10 GHz and about 123 dB at 35 GHz (**computed**). Real rain does not fill the whole path; see gaps.

- **Version caveat.** P.838-3 (2005) replaces Table 1 with a continuous fit for \(k(f)\) and \(\alpha(f)\) from 1 to 1000 GHz. The Chinese PDF of P.838-3 was indexed and shows the same \(\gamma_R = k R^{\alpha}\) law and a revised table (at 1 GHz, \(k_H = 0.0000259\), not the 1992 value 0.0000387). The English P.838-3 file returned 404, so the revised X/Ku/Ka coefficients were not copied. Expect tens of percent differences, not a change in the frequency trend. — [P.838-3 record](https://www.itu.int/rec/R-REC-P.838/en); indexed Chinese PDF `R-REC-P.838-3-200503-I!!PDF-C.pdf`

### Inferences

- Separate three losses in data or in code: (1) clear-air gas, function of frequency, pressure, temperature, and water-vapour density, integrated in altitude; (2) rain, \(\gamma(f, R_{\mathrm{mm/h}}, \mathrm{pol})\) times rainy path length; (3) a constant system loss. Putting rain into a fixed \(L\) re-introduces a hard-coded range.
- Because \(\gamma\) depends on \(R\), detection range is solved by iteration, which is exactly the Blake-chart step Barton describes. For a sim that evaluates SNR at a known geometric range, iteration is unnecessary: compute \(\gamma\) at that range and divide SNR by the linear two-way factor \(10^{0.1 \times 2 \gamma R_{\mathrm{km}}}\) only over the rainy portion.
- X-band clear air can be omitted before rain is omitted. Ka-band cannot. K-band (near 22 GHz water-vapour line) should not be treated like Ku.

### Gaps

- No verified clear-air dB/km at X, Ku, or Ka. Implement P.676-13 Annex 1 or Annex 2 from the recommendation itself; do not hard-code 0.01 dB/km from the unopened thesis.
- P.838-3 coefficients were not retrieved. Recompute the table when the 2005 (or later) English recommendation is available. Until then, tag the rain table as P.838-0.
- Effective path length through rain (the path-reduction factor in terrestrial models such as ITU-R P.530) was not opened. Using full slant range through a single rain rate overstates loss whenever the rain cell is smaller than the path.
- Cloud and fog are ITU-R P.840, not opened. Snow and hail are not the P.838 rain law.
- Gaseous attenuation at altitude: sea-level \(\gamma\) applied to an air-to-air path overstates oxygen and water-vapour loss. P.676 points at a vertical profile (P.835) and a slant-path integral; that integral was not copied.

## Refraction and the 4/3-earth radio horizon

### Takeaway

A geometric straight line to the horizon is not the radio horizon. Under a standard troposphere the ray bends downward enough that replacing the earth radius by \(4/3\) of its value, and then drawing a straight line, is the usual model. That factor is a standard-atmosphere assumption, not a constant of nature, and it does not depend on X vs Ka.

### Cited Findings

- **NPS, opened, standard-atmosphere model.** With no atmosphere the ray is straight and the horizon is the geometric tangent. “In typical atmospheric conditions radio waves bend more than light,” so the radio horizon lies beyond the geometric horizon and beyond the optical horizon. For a standard gradient of refractivity \(N = (n-1)\times 10^6\),

  \[
  r_{\mathrm{eff}} = \frac{4}{3} r_e, \qquad d = (2\, z\, r_{\mathrm{eff}})^{1/2}
  \]

  \(d\) is range to the radio horizon, \(z\) is height of the antenna. Their numerical example uses \(r_e = 6300\,\mathrm{km}\), so \(r_{\mathrm{eff}} = 8400\,\mathrm{km}\), and

  \[
  d \approx 130\,\sqrt{Z}\ \mathrm{km}
  \]

  with \(Z\) in km. Range to a target at height \(Z_r\) is \(d + d_2 \approx 130(\sqrt{Z} + \sqrt{Z_r})\) km. They note the factor 130 is only for kilometres. “A standard atmosphere represents average conditions.” If the real profile is known, “there is no simple formula.” — [NPS Module 3.1](https://www.oc.nps.edu/NWDC_EM_Course/course_materials/module3_1.html)

- **When the straight-line picture is drawn on a curved earth.** NPS: on a true-earth plot a standard ray bends downward but less than the earth; on a 4/3-earth plot the same ray is drawn straight; on a flat-earth plot it appears to bend upward. So a geometric line of sight computed with the true radius is shorter than the radio line of sight under the standard gradient.

- **The 4/3 model fails in ducts.** Trapping when \(dN/dZ < -157\,\mathrm{km}^{-1}\) (\(-0.157\,\mathrm{m}^{-1}\)): the ray curvature exceeds the earth’s and energy is ducted. Modified refractivity \(M = N + 0.157\, z\) with \(z\) in metres goes negative in slope inside a duct. NAWC: ducting is a temperature inversion in the lower troposphere that can extend range past the radar horizon, and it is frequency sensitive (“the thicker the duct, the lower the minimum trapped frequency”). — [NPS Module 3.1](https://www.oc.nps.edu/NWDC_EM_Course/course_materials/module3_1.html); [NAWC atmosphere](https://www.rfcafe.com/references/electrical/ew-radar-handbook/rf-atmospheric-absorption-ducting.htm)

- **NAWC horizon rule of thumb.** Equations are images, but the text says the derivation “assume[s] a value for the Earth’s radius that is 4/3 times the actual radius,” and that the constant in the height-in-feet formula “changes from 1.23 to 1.06” when the same geometry is evaluated for an optical line of sight instead of the radio horizon. — [NAWC horizon](https://www.rfcafe.com/references/electrical/ew-radar-handbook/radar-horizon-line-of-sight.htm)

- **Check of that 1.23 factor against the NPS formula (computed, not a new measurement).** \(130\sqrt{Z_{\mathrm{km}}}\) with height converted from feet and range from km to nautical miles is \(1.23\sqrt{h_{\mathrm{ft}}}\) in nmi. Dividing by \(\sqrt{4/3}\) produces a constant of about 1.06, which matches the handbook’s optical constant. Prefer the SI form \(d = \sqrt{2 z r_{\mathrm{eff}}}\) in the sim.

- **Pattern-propagation factor.** Barton: after the free-space range \(R_0\) is obtained, the pattern-propagation factor \(F\) “multiplies \(R_0\)” to get a first range estimate, and atmospheric attenuation is then iterated. Multiplying range by \(F\) is what a one-way voltage factor does inside an \(R^4\) equation (\(P_r \propto F^4/R^4\)). \(F\) includes the antenna pattern and multipath/diffraction, not just the 4/3 horizon. The horizon is a hard mask only in the knife-edge sense; \(F\) can be greater or less than 1 above the horizon (multipath lobes). The \(F\) model itself was not opened. — [Barton preview](https://api.pageplace.de/preview/DT0400.9781608075225_A24132309/preview-9781608075225_A24132309.pdf)

- **Frequency.** NPS: tropospheric refraction from VHF through SHF does not need a frequency correction. The 4/3 horizon is the same at X and at Ka. Diffraction beyond the horizon does depend on frequency; ITU-R P.526 was not opened, and no diffraction formula is recorded.

### Inferences

- Radio line of sight exists when slant range is less than \(d_{\mathrm{radar}} + d_{\mathrm{target}}\) from the 4/3 formula **and** terrain does not intersect the 4/3 ray. Geometric intersection with the true-earth tangent will mask targets the radio can still see, by the ratio \(\sqrt{4/3} \approx 1.15\) in horizon range under the NPS assumption.
- Use \(k = 4/3\) as default data, not as a constant in code. Superrefraction and ducts are a different \(k\), or a failure of the linear-gradient model. NPS’s own \(r_e = 6300\,\mathrm{km}\) is an example; store the earth radius the rest of the sim uses and apply \(4/3\) to that.
- Do not apply 4/3 and also bend the ray. One or the other.

### Gaps

- The algebraic refractivity \(N(P, T, e)\) was an image on the NPS page and was not transcribed. ITU-R P.453 was not opened.
- No numeric comparison of geometric vs 4/3 horizon for a fighter altitude was copied from a source. The formula is enough to compute one.
- Diffraction into the shadow (P.526), multipath lobe structure, and the elevation-angle error from refraction were not opened. The 4/3 model corrects the horizon range; it does not by itself give a height error.
- Ducting is acknowledged and not modelled. No trapping-frequency formula beyond the NAWC qualitative sentence was opened.

## Detection is a probability each dwell

### Takeaway

Each dwell produces an SNR and, from that, a \(P_d\). A detection flag is a draw from \(P_d\), or a threshold on \(P_d\), not a comparison of range with a stored lock range. Single-scan \(P_d\) is the probability on one dwell or scan. Cumulative \(P_d\) is the probability of at least one detection in a sequence of scans, and it must use the \(P_d\) at each range if the target is closing.

### Cited Findings

- **What \(P_d\) is.** NAWC: a threshold is set above the mean noise so that the probability of noise peaks crossing it is acceptably small. “Just because \(N\) is in the denominator doesn’t mean it can be increased to lower” the minimum signal; if noise rises, the signal must rise to hold the same \(S/N\). Their Figure 2 is a nomograph of \(S/N\) versus \(P_d\) and false-alarm probability; the worked point in the text (98% detection, a very small false-alarm rate) is an illustration that \(S/N\), not range, is the detection input. They also say a human operator can work at lower \(S/N\) than an automatic threshold (visibility factor vs detectability factor; Barton’s 13.2 dB vs 11.2 dB at \(P_d = 0.5\), \(P_{fa} = 10^{-6}\), above). — [NAWC receiver](https://www.rfcafe.com/references/electrical/ew-radar-handbook/receiver-sensitivity-noise.htm)

- **Map SNR to \(P_d\) on one dwell.**
  - Nonfluctuating, noncoherent integration of \(N\) samples: Richards’s inverse Albersheim, quoted in the detectability section. Domain: roughly \(0.1 \le P_d \le 0.9\), \(10^{-7} \le P_{fa} \le 10^{-3}\), \(1 \le N \le 8096\), error corresponding to < 0.2 dB in SNR.
  - Swerling I, one independent sample: \(P_d = (P_{fa})^{1/(1+\mathrm{SNR})}\), SNR linear. Exact in the sources cited above.
  - Coherent dwell: fold \(N\) into the SNR first, then use a one-sample formula (\(N = 1\) in Albersheim). Do not also pass \(N\) into Albersheim.

- **Single-scan vs cumulative.**
  - Indexed POMR eq. (3.24): if one dwell has detection probability \(P_d(1)\) and the dwells are independent,

    \[
    P_d(n) = 1 - \bigl[1 - P_d(1)\bigr]^n
    \]

    Their example: \(P_d(1) = 0.90\) gives \(P_d(2) = 0.99\) and \(P_d(3) = 0.999\). The same recurrence with \(P_{fa}\) replaces \(P_d\). Because \(P_{fa}\) is small, \(P_{fa}(n) \approx n\, P_{fa}(1)\) (their eq. 3.25). To hold a three-look cumulative false-alarm probability of \(10^{-6}\), the single-dwell \(P_{fa}\) must be about \(0.33\times 10^{-6}\). — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90)
  - Richards, *Fundamentals of Radar Signal Processing*: “cumulative probability” is commonly the probability of detecting the target at least once in \(N\) scans. If the per-scan \(P_d\) is constant, that is the “1 of \(N\)” binary-integration case. If range changes, each scan has its own \(P_d\) and the constant-\(P_d\) formula is not enough. — [FRSP index](https://dokumen.pub/fundamentals-of-radar-signal-processing-2ndnbsped-0071798323-9780071798327.html)
  - *Radar Technology Encyclopedia* index: cumulative detection over scans, \(P_c = 1 - (1-P_d)^n\), is appropriate when the target moves through more than one resolution cell between samples, so the scans are independent. It is the least efficient way to combine samples; integration within a dwell is better when the samples fall in the same cell. — [encyclopedia index](https://epdf.tips/radar-technology-encyclopedia.html)
  - DTIC ADA274786: the single-scan probability is the blip-scan ratio \(p_D(R)\). A specification \(R_{90}\) is the range at which the cumulative probability on an **approaching** target reaches 0.90, i.e. \(1 - \prod_i [1 - p_D(R_i)] = 0.90\), with \(R_i\) the range at each scan. It is not a single-scan range. — [ADA274786](https://apps.dtic.mil/sti/tr/pdf/ADA274786.pdf)
  - Stimson, *Introduction to Airborne Radar* (Internet Archive index): the cumulative probability of detecting at the range \(R_k\) or before is \(1 - \prod (1 - P_{\mathrm{scan}})\). Because a closing target changes range between scans, an average over the range step is required; normalized curves versus \(R/R_0\) exist for that. — [Archive.org Stimson](https://archive.org/details/airborneradar00pove)

- **\(m\)-of-\(n\) is not the same as cumulative.** POMR gives the binomial sum for “\(m\) of \(n\)” threshold crossings. Cumulative “at least once” is the special case \(m = 1\). Track confirmation logics (2 of 3, 3 of 5) are the binomial, and they need a higher per-dwell \(P_d\) than a 1-of-\(n\) cumulative. — [POMR Vol. I index](https://vdoc.pub/documents/principles-of-modern-radar-vol1-basic-principles-7akpiac61v90)

### Inferences

- Per dwell the sensor model returns SNR and \(P_d\). A lock or track-file update may draw a Bernoulli trial with that \(P_d\), or it may require \(m\) successes in \(n\) dwells. Neither rule is a maximum range.
- For a closing target, update the product with the current \(P_d(R)\). Using \(1-(1-P_d)^n\) with the \(P_d\) evaluated only at the current range overstates cumulative detection, because earlier scans were at longer range and lower \(P_d\).
- Holding \(P_{fa}\) fixed per cell while the number of cells or the number of scans grows lets the cumulative false-alarm count grow as \(n P_{fa}\). If the sim budgets false alarms per scan, the threshold (hence the detectability factor) must move when bandwidth, PRF, or search volume changes the number of tests. That coupling is why \(P_{fa}\) is an input and \(S_{\min}\) is not a constant.
- Independence assumptions: scan-to-scan cumulative formulas assume independent RCS draws or at least independent noise. A Swerling I target is highly correlated inside one dwell and decorrelated on the next scan only if the aspect or frequency changed enough. The model, not the formula, has to say which.

### Gaps

- Stimson’s normalized cumulative-probability curves (the \(p\) vs \(\Delta p\) charts) were not read as figures, only as the existence of \(P_c = 1-\prod(1-P_i)\) and a range average. Do not digitize them from memory.
- No opened source gave a recommended \(P_d\) or \(P_{fa}\) for an aircraft fire-control lock versus a search scan. The 0.9 / \(10^{-6}\) pair is a textbook illustration (GPCET: “a typical radar system will operate with” those values), not a requirement.
- Correlated dwells (high-PRF track at a fixed aspect, Swerling I) do not obey the independent product. No correlation model was opened.

## Minimum radar parameters so lock range is not hard-coded

### Takeaway

Lock range is an output. The data that determine it are the terms in the SNR equation, the number of pulses in the dwell, the detectability curve, the target’s RCS model, and the propagation loss along the path that exists on that frame.

### Cited Findings

Every parameter below appears in an equation quoted or combined in the sections above.

- **Transmitter:** peak power \(P_t\) (W) during the pulse, or average power plus duty cycle; pulse width \(\tau\) (s); PRF (Hz). Duty cycle \(= \tau \times \mathrm{PRF}\) if the pulses are rectangular (NAWC).
- **Antenna:** \(G_t\) and \(G_r\) (linear), or one gain if monostatic and the same aperture is used. Beamwidths in azimuth and elevation, because time-on-target for search is beamwidth divided by scan rate (NAWC scanning caveat; the exact \(T_{\mathrm{on}} = \theta/\omega\) step is the usual geometry and was not a quoted equation). Scan rate, or a scheduled track dwell time.
- **Carrier:** frequency or wavelength. It enters \(\lambda^2\), rain and gas loss, and RCS if RCS is frequency-dependent.
- **Receiver:** noise factor NF (linear) **or** system noise temperature \(T_s\) (K), not both unless the reference terminals are defined. Reference \(T_0 = 290\,\mathrm{K}\) if the NF form is used (NAWC). Noise bandwidth \(B\) (Hz), or an explicit “matched, \(B = 1/\tau\)” flag. The handbook’s \(B \approx 1/\mathrm{PW}\) is a rule of thumb; Barton’s matched-filter energy form does not need a separate \(B\) once the pulse is matched.
- **Losses, as separate numbers:** RF/plumbing if not already inside \(G\); polarization mismatch; signal-processing and detection losses; beamshape loss for search. A single opaque \(L\) is acceptable only if atmosphere is **not** inside it.
- **Integration:** coherent or noncoherent; number of pulses is \(\mathrm{PRF}\times T_{\mathrm{dwell}}\), not a separate magic gain. Coherent processing loss if it is known.
- **Detection policy:** \(P_{fa}\) per decision cell; Swerling case or “nonfluctuating”; single-dwell vs \(m\)-of-\(n\). These set \(D\), the required SNR. They are mode data. They are not a range.
- **Target:** mean RCS (m²) at the aspect, frequency, and polarization, plus the fluctuation model. Bistatic engagements need bistatic RCS and two ranges (NAWC).
- **Path:** monostatic slant range, or \(R_{Tx}\) and \(R_{Rx}\); radio horizon via \(k r_e\) with default \(k = 4/3\); one-way specific attenuation from gas and from rain rate, doubled for monostatic.

**Published typical ranges that are rules of thumb, not defaults to ship as truth** (NAWC duty-cycle and RCS sections): pulse duty cycle 0.1–3%, pulse-Doppler duty cycle 5–50%; tactical-jet RCS anywhere in about 1–100 m² with aspect, handbook “typical” 5–100 m²; missile typical 0.5 m². A sim that needs a number before real data exist can start inside those bands and must keep the number in data.

### Inferences

- If any of \(P_t\), \(G\), \(\lambda\), \(\tau\) or \(B\), NF or \(T_s\), \(L\), PRF, beamwidth or dwell time, \(P_{fa}\), and \(\sigma\) is missing, the missing one has been replaced by a hard-coded range. There is no extra “lock range” term in the sources.
- Gain and beamwidth are both stored. Computing one from the other needs an aperture efficiency that was not opened (POMR only says efficiency is “seldom below 0.5 and seldom above 0.8”).
- Semi-active missile: the illuminator carries \(P_t\), \(G_t\), \(\lambda\), \(\tau\), PRF; the missile carries \(G_r\), NF, \(B\), and its own dwell. Range is the pair \((R_{Tx}, R_{Rx})\). Using a monostatic \(R^4\) with the missile’s range double-counts or drops a leg.

### Gaps

- No opened source provided a minimum parameter list written for a game or a simulator. The list above is the equation’s support, not a quoted checklist.
- Aperture efficiency, beamshape loss, and the \(T_s\) worksheet remain unnamed numbers. Leave them as explicit loss/temperature inputs with defaults of “ideal” (\(L = 1\), \(T_s = T_0\,\mathrm{NF}\)) so the idealization is visible.

## Implementation notes for missilesim

### Takeaway

Once per dwell, compute SNR from data and geometry, then \(P_d\) from SNR. Do not store a lock range. Draw the detection, or feed \(P_d\) to the track logic, and accumulate across scans with the product formula only while the per-scan probabilities are the ones at those ranges.

### Cited Findings

The equations to call are the ones quoted above. In implementation order:

1. **Horizon gate.** \(r_{\mathrm{eff}} = k r_e\) with \(k = 4/3\) unless the atmosphere model says otherwise. Radio horizon range \(d = \sqrt{2 z r_{\mathrm{eff}}} + \sqrt{2 z_t r_{\mathrm{eff}}}\) (NPS). If terrain or the horizon blocks the path, SNR is not the free-space value; \(F\) was not specified in an opened equation beyond Barton’s statement that \(F\) multiplies free-space range. A blocked path can return \(P_d = 0\) without inventing a diffraction model.
2. **RCS.** Look up mean \(\sigma(\mathrm{aspect}, f, \mathrm{pol})\) in m². If a Swerling \(P_d\) formula will be used, do not also randomize \(\sigma\). If Albersheim (steady target) will be used and scintillation is desired, draw \(\sigma\) once per dwell for Swerling 1/3 or once per pulse for Swerling 2/4, then detect as a steady target at that \(\sigma\).
3. **Pulses in the dwell.** \(N = \mathrm{PRF} \times T_{\mathrm{dwell}}\). Search: \(T_{\mathrm{dwell}}\) limited by beamwidth and scan rate. Track: \(T_{\mathrm{dwell}}\) is the time the mode actually spends on that target. Coherent processing may use only a subset of \(N\).
4. **Single-pulse SNR,** inference from the NAWC \(P_r\) and \(S_{\min}\) equations with \(B = 1/\tau\) when the mode says the filter is matched:

   \[
   \mathrm{SNR}_1 = \frac{P_t G_t G_r \lambda^2 \sigma\,\tau}{(4\pi)^3 R_{Tx}^2 R_{Rx}^2\, k T_s\, L_{\mathrm{sys}}}
   \]

   Monostatic: \(R_{Tx} = R_{Rx} = R\). \(k = 1.38\times 10^{-23}\,\mathrm{J/K}\). \(T_s = T_0 (\mathrm{NF})\) with \(T_0 = 290\,\mathrm{K}\) if only a noise figure is known. \(L_{\mathrm{sys}}\) does not include the atmosphere.
5. **Atmosphere.** One-way \(\gamma = \gamma_{\mathrm{gas}}(f, \mathrm{profile}) + k(f,\mathrm{pol}) R_{\mathrm{rain}}^{\alpha}\) (dB/km). Monostatic loss factor \(L_{\mathrm{atm}} = 10^{0.1 \times 2 \gamma_{\mathrm{eff}} R_{\mathrm{km}}}\), where \(\gamma_{\mathrm{eff}} R\) is the integral along the path, not automatically \(\gamma_{\mathrm{sea\ level}}\) times slant range. Divide \(\mathrm{SNR}_1\) by \(L_{\mathrm{atm}}\). Rain coefficients may be the P.838-0 table in this note until P.838-3 is coded. Gas: no numeric default; omit rather than invent 0.01 dB/km, or integrate P.676.
6. **Dwell SNR.** Coherent, constant amplitude over the CPI: \(\mathrm{SNR}_{\mathrm{dwell}} = N_{\mathrm{coh}}\,\mathrm{SNR}_1 / L_{\mathrm{coh}}\), then detect with a one-sample curve. Noncoherent: keep \(\mathrm{SNR}_1\) and pass \(N\) into Albersheim. Swerling 2/4: do not apply the coherent factor \(N\) (Basic Radar Analysis statement).
7. **\(P_d\).** Nonfluctuating: Richards inverse Albersheim. Swerling I, one independent look: \(P_d = P_{fa}^{1/(1+\mathrm{SNR})}\). Swerling III: not implemented until Shnidman or Barton’s formula is taken from a clean source; falling back to Swerling I is conservative at \(P_d = 0.9\) (higher required SNR) and must be labeled as such.
8. **Outputs.** \(\mathrm{SNR}_1\), \(\mathrm{SNR}_{\mathrm{dwell}}\), \(P_d\), and the inputs that produced them (especially \(N\), \(\sigma\), \(L_{\mathrm{atm}}\)). A Boolean detection is optional and is a random draw with probability \(P_d\).
9. **Across scans.** \(P_{\mathrm{cum}} = 1 - \prod_i (1 - P_{d,i})\) with the \(P_{d,i}\) stored from each scan (POMR, Stimson, ADA274786). Do not recompute it from the current range alone. Cumulative false alarms obey the same product; for rare false alarms \(n P_{fa}\) is the POMR approximation. Confirmation logic is \(m\)-of-\(n\), not the cumulative product, if the track file demands repeated hits.

### Inferences

- Data files, not code constants: for each radar mode, \(P_t\), \(G_t\), \(G_r\), \(f\), \(\tau\), PRF, beamwidths, scan rate or track dwell, NF or \(T_s\), \(L_{\mathrm{sys}}\), \(L_{\mathrm{coh}}\), integration type, \(P_{fa}\), and the matched-filter flag. For each target class, an RCS table or a mean plus Swerling case. For the environment, \(k\), rain rate, and a gas model id. A “lock range” field should not exist.
- Bistatic semi-active is the same function with two ranges and two gains. The missile does not carry the illuminator’s \(P_t\).
- Order-of-magnitude checks, not clamps: at \(P_{fa} = 10^{-6}\), a steady target crossing \(P_d = 0.5\) near 11 dB and \(P_d = 0.9\) near 13 dB (one sample) means the SNR-to-\(P_d\) curve is about right. A Swerling I target at \(P_d = 0.9\) near 21 dB means the fluctuation model is about right. If the sim “locks” at a fixed range regardless of those SNRs, the range equation is not what is running.
- Fourth-root sensitivity: NAWC notes \(S_{\min}^{-1} \propto R_{\max}^4\), so 12 dB more sensitivity doubles range. RCS and power enter the same way. Large gameplay effects need large data changes; a factor-of-two RCS change is only about 19% in range.

### Gaps

- Shnidman coefficients, P.676 numeric profile, P.838-3 coefficients, beamshape loss, and multipath \(F\) are unimplemented because the opened sources did not yield a clean formula or table. Wire them in as replaceable functions with the ideal defaults stated above.
- Clutter, jamming, and Doppler visibility (MTI/pulse-Doppler processing gain beyond coherent integration) are outside this note. Barton’s preview treats clutter detectability as a different factor. Until that exists, \(P_d\) here is thermal-noise-limited only, and it will be optimistic against terrain.
- No per-mode numbers for any real fire-control radar or missile seeker are provided, on purpose. Bands and the handbook’s duty-cycle and RCS **ranges** are the only unclassified magnitudes tied to that role.
