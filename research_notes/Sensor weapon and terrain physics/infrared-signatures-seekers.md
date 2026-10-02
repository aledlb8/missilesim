# Aircraft infrared signatures and how a seeker turns them into a track point

Physics for replacing a scalar “heat” lock. Open literature only. Model results are marked as models; the few laboratory or field observations are marked as measurements. Named-aircraft signature tables were not collected. The Infrared & Electro-Optical Systems Handbook (Accetta and Shumaker) and Hudson’s *Infrared System Engineering* monograph were not opened; where a paper cites Hudson, that citation is second-hand.

Seeker band assignments already written up for specific Fox 2 rounds are not repeated here.

## Spectral bands and which emission dominates

### Takeaway

Air-to-air seekers use the short-wave, mid-wave, and long-wave infrared windows, but the emitter that fills each window is different: hot metal is a continuum, the plume is a molecular-band source (CO2 near 4.3 μm, H2O near 2.7 μm), and the skin peaks in the long-wave window. A single heat number cannot stand in for that split.

### Cited Findings

- Spectral radiance of a thermal radiator shifts to shorter wavelength as temperature rises. Wien’s displacement law for the wavelength peak of spectral radiance is λ_max T = b with b = 2.897771955… × 10⁻³ m·K (about 2898 μm·K). A surface near 300 K therefore peaks near 10 μm. — [Wien's displacement law](https://en.wikipedia.org/wiki/Wien%27s_displacement_law)
- Aircraft infrared sources are the powerplant (hot metal), the exhaust plume (H2O, CO2, CO), and the airframe, including aerodynamic heating. That decomposition is a review statement, not a new measurement. — [Rao and Mahulikar, atmospheric transmission and aircraft IR signatures (2005), as extracted](https://www.researchgate.net/publication/245430242_Effect_of_Atmospheric_Transmission_and_Radiance_on_Aircraft_Infared_Signatures); [Mahulikar, Sonawane, and Rao, infrared signature studies of aerospace vehicles (2007), as extracted](https://www.researchgate.net/publication/222815037_Infrared_signature_studies_of_aerospace_vehicles)
- The same 2007 review writes the total signature as hot-parts emission + plume emission + skin emission + reflected skyshine + reflected earthshine + reflected sunshine. Plume radiation is attributed to hot CO2 and water vapour. — [Mahulikar et al. 2007](https://www.researchgate.net/publication/222815037_Infrared_signature_studies_of_aerospace_vehicles)
- In a 2025 model of a bottom view, external skin radiates primarily in LWIR, then MWIR, and negligibly in the authors’ SWIR (2–3 μm). Hot engine parts radiate in all three. The plume radiates mainly in MWIR, then SWIR, and negligibly in LWIR. In that model the plume is almost transparent in LWIR; MWIR plume emission is from CO2, and the authors’ plume H2O transmissivity in MWIR is 0.997–0.998, so they treat MWIR plume emission as CO2 alone. — [Cambridge Aeronautical Journal / same PDF on ResearchGate (2025)](https://www.cambridge.org/core/journals/aeronautical-journal/article/infrared-signature-of-aeroengine-exhaust-plumes-potential-core-and-aircraft-surface-from-direct-bottom-view/D14DC72BD1247D01A1D4883B5B17B687)
- A 2022 airframe-plus-plume model likewise puts 3–5 μm on the plume and the hot skin next to it, and 8–14 μm on the skin. — [Materials 15:7726 (2022)](https://www.mdpi.com/1996-1944/15/21/7726)
- An exhaust-system radiation model finds wall emission in both 3–5 μm and 8–14 μm, and gas emission important only in 3–5 μm, with CO2 outweighing H2O. The gas-mixture spectrum in that model sits in 4.167–4.237 μm (2360–2400 cm⁻¹) and 4.347–4.545 μm (2200–2300 cm⁻¹). — [Chinese Journal of Aeronautics exhaust-system model (2017)](https://www.sciencedirect.com/science/article/pii/S1000936117300493)
- A plume-radiation report on a low-altitude afterburning tactical-missile plume (a model, not an aircraft measurement) identifies the transmitted peak of the CO2 4.3 μm band on the long-wave side near 4.5 μm (about 2200 cm⁻¹). It computes in-band radiance in a “blue spike” (2385–2390 cm⁻¹) and a “red spike” (2220–2225 cm⁻¹). The band centre is the part the atmosphere removes; the wings are what propagate. — [DTIC ADA117013](https://apps.dtic.mil/sti/tr/pdf/ADA117013.pdf)
- Vibration–rotation band centres compiled for propellant products, not a flight measurement: H2O 2.7 μm, CO2 4.3 μm (band strength listed as 130 cm⁻¹/(g m⁻²) versus 3.5 for CO2 at 2.7 μm and 1.8 for H2O at 2.7 μm), CO 4.7 μm. N2 and H2 do not emit in the infrared because they have no dipole moment. — [MIT ICAT-2023-01, Table 4.1, compiled from a cited reference](https://dspace.mit.edu/bitstream/handle/1721.1/153219/ICAT-2023-01_Kelly-Mathesius-thesis.pdf?sequence=1&isAllowed=y)
- Measurement, small turbojet, not a fighter: an MWIR imaging Fourier-transform spectrometer at 1 cm⁻¹, side view, 11.2 m, diesel flow 300 cm³/min, recorded spectral features of water, CO, and CO2 with spatial variation across the plume. — [AFIT ADA510984 abstract](https://apps.dtic.mil/sti/tr/pdf/ADA510984.pdf)
- Measurement-and-model comparison of a different plume class (axisymmetric heated exhaust, not a turbofan installation): NASA TM reports measured temperature, pressure, and infrared images, and compares them with an inverse Monte Carlo image predicted from those fields. — [NASA TM 19960015541](http://hdl.handle.net/2060/19960015541)
- Early reticle seekers used an uncooled lead-sulfide detector behind a spinning reticle (first-generation / spin-scan). Lead sulfide responds in the short-wave infrared, not in 8–12 μm. Second-generation seekers in the same thesis are conical-scan systems; the AIM-9L is shown there as that generation. — [Cranfield thesis, seeker chapter, as extracted](https://dspace.lib.cranfield.ac.uk/bitstreams/94fe5e9e-0715-4e35-a0df-f260b37513e2/download)
- An unclassified imaging-seeker surrogate was built with a cold filter of 3.7–4.8 μm on a 256×256 HgCdTe array. That is a design choice for an MWIR window, not a universal seeker band. — [Schleijpen et al., SPIE, TNO imaging seeker surrogate (2007)](https://publications.tno.nl/publication/34611937/NAmFxx/schleijpen-2007-imaging.pdf)
- A tailpipe-only lock-range model, citing Hudson (1969), says turbojet hot-metal tailpipes of about 500–900 K have spectral-radiant-emittance peaks in 3–5 μm, which is why many seekers are built for that window. The temperature range is Hudson as repeated by this paper; Hudson itself was not opened. — [Ab-Rahman and Hassan, Australian Journal of Basic and Applied Sciences 3(4):3703–3713 (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)

### Inferences

- SWIR (papers above use roughly 1–3 μm or 2–3 μm) is where very hot metal and a reheated plume move their Planck peak, and where reflected sunlight still matters. MWIR (3–5 μm, often narrowed to about 3.7–4.8 μm) is the plume-CO2 and tailpipe-continuum window. LWIR (8–12 or 8–14 μm) is skin and earth. Those are the three bands a table must carry; they are not interchangeable scalings of one heat value.
- The 4.3 μm CO2 fundamental is optically thick at line centre in the atmosphere, so a plume seeker that works at long range is using the band wings (the red and blue spikes), not a flat 3–5 μm average. A band edge at 4.8 μm still includes the red spike near 4.5 μm; a band that stops at 4.2 μm does not.
- H2O is not “absent from every mid-wave measurement.” The strong H2O band is at 2.7 μm, which a 3–5 μm filter can miss, and the AFIT turbojet spectrum still showed water inside an MWIR instrument. A model that sets MWIR H2O emission to zero is a band-definition choice, contradicted as a general claim by that measurement.

### Gaps

- Accetta and Shumaker’s handbook volumes on sources and atmospheric propagation were not opened. No chapter-level quotation from that handbook is available for this note.
- No opened source gives a measured SWIR/MWIR/LWIR intensity split, in W/sr, for a fighter at stated aspect and throttle. The band ranking above is from signature models plus the small-turbojet spectrum.

## Radiometry: Planck’s law through contrast, and what aspect does

### Takeaway

The sim needs band radiance from Planck’s law, radiant intensity in W/sr, and irradiance in W/m² after transmission. Contrast is the difference against the background the target replaces. Aspect changes which surfaces and which gas paths are in the line of sight; it is not a single cosine painted on one heat value.

### Cited Findings

- Wavelength form of Planck’s law for spectral radiance (power per unit projected area, per unit solid angle, per unit wavelength):

  B_λ(λ, T) = (2 h c² / λ⁵) / (exp(h c / (λ k_B T)) − 1)

  Frequency form:

  B_ν(ν, T) = (2 h ν³ / c²) / (exp(h ν / (k_B T)) − 1)

  SI units: B_ν is W·sr⁻¹·m⁻²·Hz⁻¹; B_λ is W·sr⁻¹·m⁻³. The two functions are not the same expression with ν swapped for c/λ, because the spectral increment differs. A blackbody is a Lambertian radiator: radiance does not depend on direction, and the power from an actual surface element falls as cos θ (Lambert’s cosine law) because the projected area does. Emissivity ε is the ratio of actual radiance to this Planck radiance, 0 ≤ ε ≤ 1, and it may depend on wavelength, temperature, angle, and polarization. — [Planck's law](https://en.wikipedia.org/wiki/Planck%27s_law)

- Integrating spectral radiance over wavelength gives band radiance. Integrating Planck’s law over all wavelengths and the hemisphere gives the Stefan–Boltzmann exitance σ T⁴. For a Lambertian surface the radiance and the hemispheric exitance are related by N = S / π (radiance = exitance / π). — [Planck's law](https://en.wikipedia.org/wiki/Planck%27s_law); [Ab-Rahman and Hassan (2009), citing Mahulikar](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)

- Differential irradiance of an unresolved target at the sensor entrance aperture, broadband:

  ΔE = A_tgt τ_atm (L_tgt − L_bg) / R²

  with units W/cm² in that dissertation (equivalently W/m²). The spectral form is the same with L(λ) and τ(λ), per unit wavelength. This ΔE is what is divided by noise-equivalent irradiance to get signal-to-noise ratio. — [UCF doctoral dissertation, infrared search and track radiometry](https://stars.library.ucf.edu/cgi/viewcontent.cgi?article=2378&context=etd2020)

- The same dissertation states that path and background are part of the difference: the target is seen only to the extent its radiance exceeds the background radiance, after atmospheric transmission. SNR = 1 is the mathematical definition of NEI; the author says detection in practice is often taken at 6 < SNR < 10, and in one worked IRST example the designers’ SNR = 10 contour fell near 70–80 km. That range is for the example sensor and target in the dissertation, not a missile result. — [UCF dissertation](https://stars.library.ucf.edu/cgi/viewcontent.cgi?article=2378&context=etd2020)

- A point-source irradiance falls as 1/R². With a single extinction coefficient the transmittance is written τ = exp(−α R), α = absorption + scattering, and the lock range then solves an equation of the form (contrast × area × τ(R)) / R² equal to a seeker threshold, which that paper solves with the Lambert W function. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)

- Skin emission, plume emission, and hot-metal emission do not share an angular shape. A nozzle/plume model reports spectral radiance at 4.35 μm of about 160 W·m⁻²·sr⁻¹·μm⁻¹ in the lateral view and about 34 W·m⁻²·sr⁻¹·μm⁻¹ looking aft. Those units are radiance, not radiant intensity. The same model says the skin shields the plume and the plume heats the skin beside the nozzle. Integrated 3–5 μm radiance at the nozzle from the rear view is given as 45 W·m⁻²·sr⁻¹ in that paper. — [Materials 15:7726 (2022)](https://www.mdpi.com/1996-1944/15/21/7726)
- A bottom-view model (not a sphere of aspects) finds the plume-to-surface balance changing with Mach and with reheat. At Mach 0.3 the paper’s “MWIR radiation ratio” is of order 289 in maximum dry power and 4891 in maximum reheat; at Mach 1.8 the same ratio is 2.2 dry and 45.9 in reheat. The abstract defines the comparison of interest as plume MWIR versus surface LWIR. Reheat keeps plume MWIR above surface LWIR in the cases they ran; dry power does not, once the skin is hot. — [Aeronautical Journal plume-core paper (2025)](https://www.cambridge.org/core/journals/aeronautical-journal/article/infrared-signature-of-aeroengine-exhaust-plumes-potential-core-and-aircraft-surface-from-direct-bottom-view/D14DC72BD1247D01A1D4883B5B17B687)

### Inferences

- Quantities the frame loop should carry, and only these units:

  1. Spectral radiance L_λ = ε(λ) B_λ(λ, T) in W·sr⁻¹·m⁻²·μm⁻¹ (after the usual unit scaling of B_λ).
  2. Band radiance L_band = ∫_{λ1}^{λ2} ε(λ) B_λ dλ in W·m⁻²·sr⁻¹.
  3. Band radiant intensity I_band = ∫ L_band cos θ dA in W/sr. For a uniform Lambertian patch this is L_band times projected area. Hot-metal cavities are not a flat plate: the projected-area factor applies to the aperture that is actually visible.
  4. Irradiance at the seeker E = I_band τ_band / R² in W/m², for a source that does not fill the field of view.
  5. Contrast irradiance ΔE = τ_band A_proj (L_target − L_background) / R². Path radiance added equally to target and adjacent background cancels in ΔL (ΔL_apparent = τ (L_t − L_b)) but still raises photon noise and can shrink reticle modulation on a large pedestal.

- Aspect, in physical terms rather than a fitted lobe: tail aspect is the only direction that looks into the tailpipe cavity, so the graybody continuum is strongest there and is occluded toward the beam and the nose. Beam aspect sees plume path length through the jet and the largest projected skin, and does not see the cavity. Nose aspect hides cavity and most of the plume; what remains is skin, inlets, and any forward plume radiation, which is a different function of throttle. The 2022 model’s higher lateral than rear spectral radiance at 4.35 μm is a warning: a plume volume can be brighter from the side than from behind even while the tailpipe wall makes the rear view brighter in a wall band. One cosine from nose to tail is the wrong shape for all three emitters.
- Reflected sun, sky, and earth are extra intensities. They depend on the illumination direction, not only on the target’s nose angle. They belong in the sum Mahulikar wrote, as separate terms, or they will be missing whenever the seeker looks at a glint.

### Gaps

- A numeric first and second radiation constant (c1, c2) was not taken from an opened CODATA or NIST page. Use the symbolic Planck law above rather than a copied constant.
- No opened measurement gives I(ψ) in W/sr at tail, beam, and nose for the same engine setting. The lateral-versus-aft numbers are model radiances for one nozzle flow, and they must not be copied in as a fighter’s intensity table.

## Plume: temperature, species, throttle, and the reduced-order model

### Takeaway

The plume is hot CO2 and H2O (and some CO) with a non-gray spectrum. Afterburning changes temperature, size, and which band dominates. Open sim practice replaces a full plume calculation with a few sources or a small table, not with one heat scalar.

### Cited Findings

- Asymmetric molecules (H2O, CO2, CO) radiate; the plume is those gases, not a blackbody. A signature-model paper places plume radiation mainly in 4.1–4.8 μm and 5–8 μm, while the nozzle radiates across the infrared. The strongest plume spectral intensity in that paper is 4.15–4.45 μm, identified with the CO2 and CO vibrational band. — [Infrared signature modeling of an aircraft plume (2011), as extracted](https://www.researchgate.net/publication/258494738_Infrared_Signature_Modeling_and_Analysis_of_Aircraft_Plume)
- The 2017 exhaust-system model (above) confines the gas spectrum to the two wings around 4.2 μm and 4.4 μm and finds 8–14 μm gas radiation too weak to matter next to the walls. — [Chinese Journal of Aeronautics (2017)](https://www.sciencedirect.com/science/article/pii/S1000936117300493)
- In the 2025 bottom-view model, complete combustion of a hydrocarbon leaves CO2, H2O, O2, and N2; CO, NOx, and SOx are neglected. Peak plume radiation falls in MWIR in maximum dry power and in SWIR in maximum reheat. MWIR plume emission is stated as 30–37 times the SWIR plume emission in that model. The dry-versus-reheat ratios in the previous section are model outputs for that geometry. — [Aeronautical Journal (2025)](https://www.cambridge.org/core/journals/aeronautical-journal/article/infrared-signature-of-aeroengine-exhaust-plumes-potential-core-and-aircraft-surface-from-direct-bottom-view/D14DC72BD1247D01A1D4883B5B17B687)
- A reduced tailpipe model used for lock range, explicitly a model: plume temperature at the nozzle taken as 0.85 times exhaust-gas temperature; tailpipe treated as a gray body with total emissivity about 0.9, temperature equal to EGT, area equal to the nozzle area; the whole emitter then collapsed to one point radiating into a hemisphere. Stated EGT cases in that paper: 635 °C takeoff, 515 °C continuous, 485 °C cruise. With their assumed seeker, those cases produced model lock ranges of about 37.2 km, 32.8 km, and 31.6 km. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)
- The component model one step up from a single point is the sum of hot parts, plume, skin, and three reflected backgrounds. The 2007 review has a section on standard signature models but the code names in that section were not in the text retrieved here. — [Mahulikar et al. 2007](https://www.researchgate.net/publication/222815037_Infrared_signature_studies_of_aerospace_vehicles)
- The high-fidelity end, for a rocket rather than a turbojet, is a CFD plume (species and temperature on a grid) plus a spectral radiative-transfer calculation, compared with flight spectral imaging. ONERA’s Black Brant comparison was run at 1900–5000 cm⁻¹ in 5 cm⁻¹ steps. A related rocket study uses Reynolds-averaged CFD, a statistical narrow-band gas model, and a line-of-sight transfer step, and reports that afterburning matters across 1.5–6.0 μm while atmospheric attenuation of that plume is strong at low altitude and negligible above about 40 km. — [Rialland et al., J. Phys.: Conf. Ser. 676:012020 (2016)](https://iopscience.iop.org/article/10.1088/1742-6596/676/1/012020/pdf); [point-source rocket-plume paper, Infrared Physics & Technology (2019), abstract](https://www.sciencedirect.com/science/article/abs/pii/S1350449519300180)
- ADA117013’s model of turbulent fluctuations in one afterburning missile plume changed broadside blue-spike intensity by about 24 percent and local station radiance by at most about 37 percent. The authors conclude fluctuations were not important for the total intensity of that one case. That is a sensitivity result, not a measurement, and not a turbojet. — [DTIC ADA117013](https://apps.dtic.mil/sti/tr/pdf/ADA117013.pdf)

### Inferences

- A scalar heat fails for three independent reasons. Throttle moves both the Planck peak (dry MWIR versus reheat SWIR in the 2025 model) and the visible area of hot gas, so “twice as hot” is not twice the in-band intensity. Aspect hides the tailpipe while leaving a side-on plume, so one lobe cannot serve both emitters. The spectrum is a continuum plus narrow wings, so a seeker that does not include 4.3–4.6 μm does not see the plume the way a 3–5 μm integral suggests.
- For a real-time sim the opened literature supports a reduced state, not a CFD plume: either a few point sources (tailpipe cavity, one plume source, one skin source) with band intensity versus aspect and throttle, or a small lobe table of the same intensity. A full narrow-band CFD plume is the validation tool (the rocket papers), not the per-frame model. The single graybody nozzle of the 2009 lock-range paper is smaller than that minimum: it has no separate plume spectrum and no aspect except “the nozzle is the target.”

### Gaps

- No opened source states a general plume static temperature, in kelvin, for military power versus afterburner on a turbofan. The 0.85×EGT factor and the 485/515/635 °C cases are one paper’s model inputs.
- Names of production signature codes (the “standard models” section of the 2007 review) were not in the retrieved text. They are not listed here.
- Particle and soot radiation in afterburning turbojets was not quantified in the sources opened. Rocket papers that include alumina are a different exhaust.

## Atmosphere: band transmission, and a table instead of one extinction coefficient

### Takeaway

Transmission is a band-average along a specific slant path. Water vapour, CO2, ozone, and aerosols do not share one α. The practical sim input is a precomputed τ(band, altitudes, slant range or zenith), plus path radiance, from a MODTRAN-class run. Do not copy a line-data file into the sim.

### Cited Findings

- MODTRAN is a moderate-resolution transmittance and radiance model. The MODTRAN 2/3 report states 2 cm⁻¹ full-width at half-maximum, with molecular absorption accumulated in 1 cm⁻¹ bins. LOWTRAN 7’s molecular band model is a single parameter (pressure). MODTRAN’s band model uses pressure, temperature, and a line width. The code includes molecular continuum absorption, molecular scattering, and aerosol and hydrometeor extinction; slant paths include spherical refraction and Earth curvature. It returns atmospheric transmittance, atmospheric background radiance, single-scattered solar and lunar radiance, direct solar and lunar irradiance, and multiple-scattered solar and thermal radiance. Water-vapour continuum changes near 1 μm and 10 μm in that report are tied to laboratory and field measurements; the band-model transmittance itself is a model. — [Kneizys, Abreu, Anderson, and others, The MODTRAN 2/3 Report and LOWTRAN 7 Model (1996)](https://www.gps.caltech.edu/~vijay/pdf/modrept.pdf)
- Worked MODTRAN geometry in that report: transmittance from 5 km to 10 km altitude at 15° from zenith, U.S. Standard Atmosphere, no haze. At 2 cm⁻¹ the calculation resolves water-line structure below 2180 cm⁻¹, the N2O band near 2220 cm⁻¹, and the CO2 band centre near 2284 cm⁻¹, structure that a 20 cm⁻¹ LOWTRAN curve averages out. — [MODTRAN 2/3 report, Figure 35 discussion](https://www.gps.caltech.edu/~vijay/pdf/modrept.pdf)
- A MODTRAN user’s description of the interface: spectral coverage from the ultraviolet through the far infrared (0.2 μm to beyond 40 μm), molecular transmittance from a statistical band model, and a slant path set by the user’s geometry with refraction, up to about 100 km. Outputs are absorption, extinction, and emission, not a single k. — [Wiley appendix on the MODTRAN band model, as extracted](https://onlinelibrary.wiley.com/doi/pdfdirect/10.1002/9783527696604.app2)
- Model numbers from a runway study that drove MODTRAN, clear air, rural aerosol, 23 km visibility. Sensors at 60 m and 520 m, slant path from the sensor to the ground at 3° to the horizontal. Band-averaged transmittance: at 60 m, MWIR 0.7245 and LWIR 0.8214; at 520 m, MWIR 0.3507 and LWIR 0.4627. The same thesis finds radiative fog more transparent in LWIR (8–14 μm) than in SWIR or MWIR, and rain scattering roughly gray across those bands. A low-visibility table at decision height in that thesis drops MWIR transmittance to 3.5×10⁻⁵, 3.1×10⁻⁴, 1.3×10⁻⁶, and 6.1×10⁻¹² across the four visibility cases they label CAT I through CAT IIIb, while LWIR in those same cases is 6.9×10⁻², 1.2×10⁻¹, 3.0×10⁻², and 1.7×10⁻³. Those are MODTRAN outputs for that airport geometry, not a universal curve. — [KTH thesis, Josefine and Ulrika, MODTRAN runway study](https://www.aphys.kth.se/polopoly_fs/1.934189.1600690015!/Thesis_Josefine_Ulrika.pdf)
- Another textbook MODTRAN example, tropical atmosphere, 27 °C, 75 % relative humidity, rural aerosol, 23 km visibility, 5 km path at sea level: the author reads an atmospheric emissivity of about 0.75 in the 8–12 μm band, i.e. the imager looks through a warm veil, and shows a water-vapour notch in the transmittance plot. Model, not a radiometer flight. — [Electro-optical system analysis text, archive copy of the MODTRAN discussion](https://archive.org/download/9780817440596NaturePhotographyFieldGuide_20181025/9780819495693_Electro_optical_System_Analysis_and_Design.pdf)
- A single-α model still appears in seeker papers: τ = exp(−α R), with α split into water-vapour absorption plus fog/cloud scattering, and CO2 not given its own term. High humidity, cloud, fog, or smoke is said to change the lock range sharply. That is the model being replaced, not a recommended atmosphere. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)
- Ambient thermal emission and hot-plume emission are not the same problem at 4.3 μm. A 2005 aircraft-signature model neglects atmospheric radiance at 4.3 μm on the grounds that atmospheric radiance below 5 μm is negligible, while still noting that CO2 radiance of the atmosphere peaks near 4.3 μm but the spectral emissive power there is small. Ozone is their stated 9.6 μm atmospheric feature; water vapour dominates the low-altitude continuum. — [Rao and Mahulikar (2005)](https://www.researchgate.net/publication/245430242_Effect_of_Atmospheric_Transmission_and_Radiance_on_Aircraft_Infared_Signatures)
- Scaling a vertical optical depth by 1/cos(zenith) inside an instrument band is not valid when the band absorption is nonlinear. That warning is about band-averaged slant paths in general, not only about one sensor. — [continuum-absorption guide (2026), as extracted](https://www.sciencedirect.com/science/article/pii/S0022407326002013)

### Inferences

- Beer’s law τ = exp(−α R) with one α is the wrong table. α would have to be a function of wavelength, of the absorber column (CO2 is well mixed; water vapour is not), and of altitude, and a saturated line does not stay exponential once it is averaged over a seeker band. Two aircraft at the same slant range, one at sea level and one at 10 km, do not share a transmittance. The MODTRAN 5-to-10 km, 15°-from-zenith case is the shape of the input: endpoint altitudes plus a zenith or a range, and an atmosphere/aerosol case.
- Practical sim approach, using only the interface above and not the proprietary line file: offline, run a MODTRAN-class code (or a published stand-in) over a grid of band, observer altitude, target altitude, slant range, and a small set of humidity/aerosol cases. Store band-averaged τ and band-averaged path radiance. In the frame loop, interpolate. Do not fit one extinction coefficient. Do not store HITRAN line lists in the missile.
- The KTH clear-air pairs already show the effect of a longer low-altitude slant (0.72 MWIR at the low sensor versus 0.35 at the higher sensor on a 3° path to the ground). Fog in that study does not dim MWIR and LWIR by the same factor. A seeker band must select its own row.

### Gaps

- The KTH extract states the 3° ground-intersect geometry and the four transmittance numbers; it does not, in the text retrieved, print the slant-path length in metres. Path length was not recomputed here as if the thesis had published it.
- No opened source gives a recommended default τ table for a fighter engagement (co-altitude, look-up, look-down) that could be pasted into the sim. The numbers above are examples of the dependence, not a game database.
- Curtis–Godson path averaging is named in the MODTRAN report’s contents but the approximation formula was not retrieved and is not quoted.

## Seeker optics, NEI, and the error signal

### Takeaway

Noise-equivalent irradiance is the aperture irradiance that produces signal-to-noise ratio 1. SNR is contrast irradiance divided by NEI. A reticle turns angle into modulation phase and amplitude (or frequency deviation). An imaging seeker turns angle into a location on the focal plane. Neither device outputs “heat.”

### Cited Findings

- NEI is the differential irradiance at the entrance aperture that yields SNR = 1. In the UCF development, SNR is obtained by dividing that differential irradiance by NEI, spectrally or after a band average. For one photon-noise example the broadband NEI values are 0.467 pW/cm² and 0.376 pW/cm² at F/1.5 and F/1.8. Those are that sensor’s numbers, not a missile acceptance test. The author treats SNR = 1 as the definition and 6 to 10 as a more realistic detection threshold. — [UCF dissertation](https://stars.library.ucf.edu/cgi/viewcontent.cgi?article=2378&context=etd2020)
- An SPIE abstract on missile windows writes SNR = H_eff / NEI, with H_eff an effective target irradiance and NEI referred to the detector’s peak-response wavelength. It separates a laboratory “dark-system NEI” from a photon-noise factor that grows when the scene, the hot window, or the detector surroundings add photons. Dome heating therefore changes NEI without any change in the aircraft. — [Klein, Proc. SPIE 2286 (1994), abstract](https://doi.org/10.1117/12.187366)
- The 2009 lock-range paper defines NEP as the flux (W) that equals detector noise, and NEI as the irradiance (W/cm²) that equals detector noise. Their example seeker, which is an assumption set, not a named round: optic diameter 3.8 cm, numerical aperture 0.25, optic transmittance 0.81, D* = 5×10¹⁰ cm·Hz¹/²·W⁻¹, instantaneous field of view 1.9×10⁻⁵ sr, noise bandwidth 200 Hz, SNR threshold 3, nozzle area 3660 cm². Background in that model is a 10 °C gray body with emissivity 0.98, giving their background radiance 0.011 W·cm⁻²·sr⁻¹. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)
- Seeker head, in that same paper’s block diagram: dome, optics that focus onto the detector, a reticle (optical modulator) that supplies the directional signal, the detector, and an optional spectral filter in front of the detector. The filter is how the seeker chooses a band and rejects out-of-band background. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)
- Spin-scan: the reticle rotates on the boresight axis. A rising-sun (AM) reticle produces an amplitude that grows with radial miss and a phase that gives the clock angle. Driving the amplitude to zero is the track. On boresight the carrier disappears, so precision is poor. The rising-sun pattern is identified with the original AIM-9B seeker in an open Naval Postgraduate School text. — [NPS seeker appendix, as extracted](https://calhoun.nps.edu/bitstreams/8d630ca0-f4cf-48f1-b901-fb9fc7d695dc/download); [MIL-HDBK-1211 reticle section, as extracted](https://quicksearch.dla.mil/ImageRedirector.aspx?token=54465.116871)
- Conical scan: the reticle is fixed and the optical axis nutates, so the target image circles the reticle. Perfect track is a circle concentric with the pattern, not a stationary spot at the centre, which is why a carrier remains on boresight. In the FM form, the size of the frequency deviation is the radial error and the phase of that modulation is the azimuth. The NPS text calls con-scan FM more robust than spin-scan AM. MIL-HDBK-1211 states the same axial-null limitation of spin-scan and the conical-scan remedy. — [NPS appendix](https://calhoun.nps.edu/bitstreams/8d630ca0-f4cf-48f1-b901-fb9fc7d695dc/download); [MIL-HDBK-1211](https://quicksearch.dla.mil/ImageRedirector.aspx?token=54465.116871)
- First-generation hardware path in the Cranfield thesis: dome, Cassegrain telescope, spinning reticle, uncooled PbS, phase-sensitive detection for pitch and yaw, automatic gain control. Second generation: stationary wagon-wheel reticle, tilted rotating secondary, nutation circle, square wave on axis. A non-reticle variant uses four detectors in a cross and a nutation circle. — [Cranfield thesis](https://dspace.lib.cranfield.ac.uk/bitstreams/94fe5e9e-0715-4e35-a0df-f260b37513e2/download)
- Imaging surrogate (measurement of a research head, not an inventory missile): 3.7–4.8 μm, NEI design aim 1.6×10⁻¹⁰ W/m², field of view 4.4° × 4.4°, field of regard greater than 0.7 rad, 256×256 CMT at 30 μm pitch, frame rate above 100 Hz, NETD design aim below 100 mK, acquisition design aim beyond 10 km against a fast jet of about 250 W/sr, maximum angular rate 650°/s in pitch and yaw. The tracker implemented is a quadrant-vector tracker: the aim point is the vector tracker’s point, and target size comes from the quadrant-box area. In field tests this head tracked a commercial aircraft at 50 km and 30 000 ft, and the quadrant tracker on a medium transport was readily pulled off by flares. — [Schleijpen et al. (2007)](https://publications.tno.nl/publication/34611937/NAmFxx/schleijpen-2007-imaging.pdf)
- Two-colour seekers, as described in an open simulation paper, use one reticle and two bands (a mid-infrared band and a near-infrared band) because a flare and an aircraft do not have the same spectrum when their temperatures differ. The paper’s reticle examples include spin, conscan, crossed slits, and a “sun shine” pattern. That is a discrimination statement, not a build procedure. — [two-colour seeker simulation, Infrared Physics & Technology (2014), abstract](https://www.sciencedirect.com/science/article/abs/pii/S1350449514000759)
- A Johns Hopkins APL seeker simulation treats sensitivity as a minimum detectable irradiance that rises when the aerodynamically heated dome dominates the background. Dome emission is a seeker background, not a target signature. — [Howser, Johns Hopkins APL Technical Digest 16(1) (1995)](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V16-N01/16-01-Howser.pdf)

### Inferences

- If the seeker card stores NEI at the aperture, in W/m², for the band and for a stated background, the frame loop does not need aperture diameter or D*. SNR = ΔE / NEI(band, background, dome). Aperture, F-number, IFOV, and D* are how NEI was produced; they are the wrong runtime inputs once NEI is known, and they are the right inputs only when a card publishes D* and the optical prescription instead of NEI. The 2009 paper’s algebra from D* to NEI is not cleanly recoverable from the PDF text, so it is not restated here as a formula to code.
- IFOV is the solid angle of one resolution element (one detector, or one pixel). Total field of view is the scene the reticle or the array sees at one gimbal angle (4.4° on the surrogate). Field of regard is how far the line of sight can be steered (greater than 0.7 rad on that head). A source outside the field of regard contributes nothing. Two sources inside one IFOV are not two tracks.
- Reticle error signal, open-literature level only: spin-scan AM gives radial miss as amplitude and direction as phase, and the amplitude is zero on axis. Con-scan FM gives radial miss as frequency deviation and direction as phase, including on axis. The track point is the direction that nulls that error, not a centroid of an image. Automatic gain control makes the loop respond to contrast ratios, which is why a brighter source in the same field can steal the null.
- Imaging error signal: the surrogate’s quadrant-vector tracker returns a direction and a box size. A centroid tracker would return the irradiance-weighted pixel location of pixels above a threshold inside a gate. The opened source for imaging is the quadrant tracker, not a centroid equation. Either way the output is an angle in the focal plane, and a second source is rejected only when the algorithm’s gate leaves it out. The TNO transport trial is a measurement that a weak gate does not do that.
- The surrogate’s 250 W/sr and 10 km figures must not be combined as if 10 km were the NEI range. With τ = 1, 250 W/sr at 10 km is 2.5×10⁻⁶ W/m², about four orders of magnitude above the stated NEI of 1.6×10⁻¹⁰ W/m². The 10 km line is a design aim at some higher SNR, or a different assumption, not SNR = 1. The 50 km track is a field result on an unnamed commercial aircraft with no published intensity.

### Gaps

- Rosette (rose-scan) timing equations were not in any page opened here. They are not invented. The reticle families that were opened are spin-scan AM, con-scan FM, and a four-detector cross.
- A centroid formula (threshold, gate, intensity-weighted mean) was not opened. Do not code a specific centroid from this note; code the quantity the opened imaging source actually outputs: a focal-plane aim point from the pixels inside the tracker’s box.
- No opened source gives a production missile’s NEI in W/m². The numbers in this section are a dissertation example, a 2009 assumption set, a window-heating abstract, and one unclassified surrogate’s design aims.

## Tracking limits are data: gimbal, slew, and loss of lock

### Takeaway

Lock holds while contrast irradiance stays above a seeker threshold and the line of sight stays inside the gimbal at a rate the seeker can follow. The range at which that happens is an output. Hard-coding the range deletes atmosphere, throttle, aspect, and NEI.

### Cited Findings

- The 2009 paper’s lock range is an output of temperature, emissivity, area, extinction, and NEI (or D* and the optical prescription). Changing EGT from the cruise case to the takeoff case, or changing α, moves the range. Their plotted ranges are for that assumption set only. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)
- SNR thresholds in opened sources are not one number: the 2009 model uses 3; the UCF author says 6 to 10 is what designers often require, because SNR = 1 is only the NEI definition. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf); [UCF dissertation](https://stars.library.ucf.edu/cgi/viewcontent.cgi?article=2378&context=etd2020)
- The imaging surrogate’s steering limits are design data on the card: field of regard greater than 0.7 rad, field of view 4.4°, maximum angular rate 650°/s. An optical study for that head reduced a ±50° scan to ±40° for aberration reasons. Those angles are not a generic Fox 2 gimbal. — [Schleijpen et al. (2007)](https://publications.tno.nl/publication/34611937/NAmFxx/schleijpen-2007-imaging.pdf)
- Dome heating raises minimum detectable irradiance as speed rises, so the same aircraft irradiance can fall below the threshold without the aircraft changing. — [Howser (1995)](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V16-N01/16-01-Howser.pdf); [Klein (1994) abstract](https://doi.org/10.1117/12.187366)

### Inferences

- Loss of lock is the or of three conditions, each a datum: ΔE < SNR_min × NEI(band, background, dome); the target angle exceeds the gimbal / field of regard (or the instantaneous field, if the gimbal cannot be slewed in time); the line-of-sight rate demanded by the geometry exceeds the seeker’s track rate, so the residual grows until the target leaves the field. SNR_min, NEI, gimbal angle, and track rate are per seeker. Range is not among them.
- Because τ depends on the path, two shots with the same range and the same heat do not share a lock boundary. Because I depends on aspect and throttle, a tail chase and a head-on shot do not either.

### Gaps

- No opened source states a universal track-rate or gimbal limit for “infrared missiles.” Copying the surrogate’s 650°/s or 4.4° field into every round would invent a seeker.
- The numerical factor that converts a falling SNR into a break-lock delay (hang time, coast) was not in the sources opened. Only the threshold condition is supported.

## Background rejection: sun, clouds, terrain, and look-down

### Takeaway

Look-down into warm earth is a long-wave problem because both the earth and the skin peak near 10 μm. A CO2-wing seeker looks in a band where ambient emission is on the Wien tail and the atmospheric band centre does not pass earthshine. Sun glint is the opposite: it is a short-wave and mid-wave reflection problem.

### Cited Findings

- Wien’s law puts a 300 K surface (mammal skin, and by the same constant a warm airframe or the ground) at a radiance peak near 10 μm, inside LWIR. The solar effective temperature 5778 K peaks near 500 nm per unit wavelength, with a large near-infrared share. — [Wien's displacement law](https://en.wikipedia.org/wiki/Wien%27s_displacement_law)
- The 2007 component list includes reflected sunshine, skyshine, and earthshine as signature terms equal in kind to emission. They are not a property of the engine alone. — [Mahulikar et al. 2007](https://www.researchgate.net/publication/222815037_Infrared_signature_studies_of_aerospace_vehicles)
- MODTRAN’s outputs include direct solar irradiance and scattered solar radiance as separate products from thermal path radiance. Sun in the aperture is not the same term as thermal contrast. — [MODTRAN 2/3 report](https://www.gps.caltech.edu/~vijay/pdf/modrept.pdf)
- The tropical 5 km, 8–12 μm MODTRAN example gives path emissivity about 0.75: an LWIR look through low humid air is a look through an emitter, not through a clear window. — [electro-optical text, MODTRAN example](https://archive.org/download/9780817440596NaturePhotographyFieldGuide_20181025/9780819495693_Electro_optical_System_Analysis_and_Design.pdf)
- The 2005 model treats atmospheric spectral emissive power below 5 μm as negligible beside the 8–12 μm window, while CO2 still marks 4.3 μm. Water vapour and the 9.6 μm ozone feature dominate the atmospheric radiance they kept. — [Rao and Mahulikar (2005)](https://www.researchgate.net/publication/245430242_Effect_of_Atmospheric_Transmission_and_Radiance_on_Aircraft_Infared_Signatures)
- Hot CO2 in the plume emits in the wings that still propagate (4.17–4.24 μm and 4.35–4.55 μm in the 2017 model; red spike near 4.5 μm in ADA117013). Ambient air does not put a 300 K blackbody peak in those wings. — [Chinese Journal of Aeronautics (2017)](https://www.sciencedirect.com/science/article/pii/S1000936117300493); [DTIC ADA117013](https://apps.dtic.mil/sti/tr/pdf/ADA117013.pdf)
- Skin LWIR can exceed plume MWIR at high Mach in the dry-power bottom-view model, because aerodynamic heating raises the skin. Look-down LWIR is then skin-versus-earth, not plume-versus-sky. — [Aeronautical Journal (2025)](https://www.cambridge.org/core/journals/aeronautical-journal/article/infrared-signature-of-aeroengine-exhaust-plumes-potential-core-and-aircraft-surface-from-direct-bottom-view/D14DC72BD1247D01A1D4883B5B17B687)
- Clouds, fog, and smoke enter the 2009 model as a larger extinction and as a background the seeker can track instead of the aircraft. The KTH MODTRAN runs show fog and low visibility hitting MWIR much harder than LWIR. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf); [KTH MODTRAN thesis](https://www.aphys.kth.se/polopoly_fs/1.934189.1600690015!/Thesis_Josefine_Ulrika.pdf)

### Inferences

- In LWIR, earth and skin are both near the Wien peak, so ΔL is a small difference of two large radiances, and a humid path adds its own emission (the 0.75 emissivity example). Clutter from terrain temperature variations is then inside the band. A CO2-wing seeker rejects that earthshine twice: the band centre does not transmit the ground, and a 300 K Planck curve is weak at 4.3–4.5 μm compared with 10 μm. The plume is visible because it is hot enough to emit in the wings. That is a spectral fact, not a longer lock range.
- Sun and cloud glint follow the solar spectrum: strong in the visible and SWIR, weaker but not zero in MWIR, negligible beside thermal emission in LWIR. A glint term needs the sun–surface–seeker angle. Clouds can be either a solar reflector (SWIR/MWIR) or a thermal gray body (LWIR). One background radiance for all bands is the same mistake as one heat for all bands.
- Reticle background rejection is spatial as well as spectral. Spin-scan and con-scan modulate a small source and, to a lesser extent, extended structure; a uniform earth pedestal is what AGC and the AC-coupled carrier are for. They do not remove structured clutter. Imaging rejection is a gate on the focal plane. Neither is modeled by turning down a global heat number.

### Gaps

- No opened measurement quantifies terrain-glint irradiance in W/m² for a stated sun angle and band. The solar term is in the MODTRAN output list and in the Mahulikar sum; the magnitude is not filled in here.
- The 2005 claim that atmospheric radiance below 5 μm is negligible is that paper’s modeling choice. It matches the Wien tail for a 300 K emitter, but it is not a measurement of path radiance in the CO2 wings at every altitude.

## What is not a physical heat model

### Takeaway

A constant lock range, a unitless heat percentage, and a flare brightness with no spectrum and no position are not radiometry. The smallest state that is still physical is band intensity versus aspect and throttle, times a path transmission, compared with NEI inside the field of view.

### Cited Findings

- Contrast, not raw intensity, is what the seeker uses. The 2009 paper states detection of the difference between target and background, then immediately collapses the aircraft to one nozzle temperature and one α. That collapse is the non-physical step: the same paper has already listed hot metal, plume, skin, and aerodynamic heating as separate sources. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)
- Two-colour discrimination exists only because flare and aircraft spectra differ. A scalar brighter than the aircraft by a fixed ratio is equally bright in every band, so it cannot represent that test. — [two-colour seeker abstract (2014)](https://www.sciencedirect.com/science/article/abs/pii/S1350449514000759)
- The component sum (hot parts, plume, skin, reflected sun, sky, and earth) is the review’s expression for the total signature. Dropping any term changes a different band. — [Mahulikar et al. 2007](https://www.researchgate.net/publication/222815037_Infrared_signature_studies_of_aerospace_vehicles)
- Lock range in the analytical model is a function of T, α, and NEI. Publishing one range as the seeker’s reach freezes those variables. — [Ab-Rahman and Hassan (2009)](https://www.ajbasweb.com/old/ajbas/2009/3703-3713.pdf)

### Inferences

- Not a physical model: a lock range in metres; a heat percentage; a flare “brightness” that is only a number times a decay, with no band and no position relative to the aircraft; an aspect factor that is one curve for the whole aircraft; an extinction coefficient that does not depend on band or altitude.
- Smallest physical state that matches the sources above:
  1. For the aircraft, I(band, aspect, throttle) in W/sr for at least three contributors (tailpipe, plume, skin), or one pre-summed lobe table per band and throttle if the contributors are baked in. Reflected sun is a fourth contributor if glint matters, and it needs the sun angle.
  2. For a flare, I_flare(band, time) in W/sr and a position. No composition, no recipe: the spectrum is an input table.
  3. τ(band, path) and path radiance from the altitude / slant-range table.
  4. Seeker data: band edges, NEI, SNR_min, IFOV, field of view, gimbal, track rate.
  5. Per frame, ΔE from the sources inside the field, SNR = ΔE / NEI, track point from the reticle error or the focal-plane tracker, lock while the inequalities in the tracking section hold.

### Gaps

- No opened paper writes this state vector in the form a game engine would store. The reduction from “CFD plus MODTRAN” to “I(band, aspect, throttle) × τ” is an inference from the component papers and from the cost of the rocket CFD chains, not a quoted standard file format.
- Flare spectra versus time were not opened and are intentionally not specified. A later note can supply a published flare spectrum if it stays clear of composition and recipes.

## Implementation notes for missilesim

### Takeaway

The Fox 2 path that was read scores a unitless heat times a geometric lobe, divides by range squared, and treats a fixed acquisition range as an irradiance threshold. Replace that with a band-intensity table, a transmission table, and a comparison to NEI. Do not keep the acquisition range as a sensor constant.

### Cited Findings

- `src/objects/MissileFox2.cpp`, `measureTarget`: intensity is `heat * aspect`, and irradiance is `intensity / range²`. The target is visible only if that intensity is positive and the range is within `tailAcquisitionM * sqrt(intensity)` when a tail-acquisition distance is set. There is no wavelength, no τ, no background radiance, and no NEI in that function.
- `src/sim/Fox2Flight.h`, `seekerIntensity`: the lobe is geometric. Rear aspect uses `max(0, −cosΨ)` with Ψ from the nose, so the lobe is zero forward of the beam. All-aspect adds a forward fraction. The afterburner nose-gate path multiplies the tail lobe by 4 when reheat is on, and otherwise returns a small nose-gate fraction inside a half-angle. The header states that the factor of 4 is the inverse-square conversion of a doubled detection range, and that the forward fraction and the nose-gate fraction are not radiometric fits. `kUnhardenedSeductionRatio` is 2: a second source at least twice the primary irradiance takes an unhardened reticle.
- `src/objects/Flare.cpp`: the flare state is a scalar heat that decays as `exp(−heatDecayRate · Δt)` and is cut off below 0.01. No spectrum and no size. The missile file uses that scalar over range squared as flare irradiance (same inverse-square construction as the aircraft).
- The radiometry those three inputs do not represent is the chain in the sections above: I(band, aspect, throttle) in W/sr, τ(band, path), ΔE = τ (I_target − I_background) / R², SNR = ΔE / NEI. — [Planck's law](https://en.wikipedia.org/wiki/Planck%27s_law); [UCF dissertation](https://stars.library.ucf.edu/cgi/viewcontent.cgi?article=2378&context=etd2020); [MODTRAN 2/3 report](https://www.gps.caltech.edu/~vijay/pdf/modrept.pdf)

### Inferences

- Data tables to add, in place of `heatSignature`:
  - Spectral or band intensity versus aspect and throttle, in W/sr, at least for a tailpipe component, a plume component, and a skin component, in SWIR, MWIR (or the narrower 3.7–4.8 μm window), and LWIR. Throttle must be able to move the plume from an MWIR-dominated dry plume to an SWIR-heavier reheat plume, because that shift is a model result in the 2025 bottom-view paper and a Wien shift in general. Aspect must be able to hide the tailpipe without hiding the side-on plume.
  - Flare intensity versus band and time, in W/sr, plus the flare’s position. The exponential decay can stay as a time model of I(t) only after it is an intensity in a band. A unitless heat that is “brighter than the aircraft” will seduce every seeker the same way, which contradicts two-colour discrimination.
  - τ and path radiance on a grid of band, observer altitude, target altitude, slant range, and a few humidity or aerosol cases. Interpolate per frame. Do not use one α, and do not use the KTH or tropical example numbers as the world table; they are illustrations of the dependence.
  - Seeker card: band, NEI in W/m², SNR_min, IFOV, field of view, gimbal limit, track rate. Loss of lock when SNR falls below SNR_min or the line of sight leaves the gimbal or outruns the track rate. Delete the use of a stored acquisition range as the detection test.
- Per-frame sum at the seeker: for each source inside the field of view, E_i = I_i(band, aspect, throttle) × τ(band, path_i) / R_i². Contrast against the in-band background and path radiance as in the UCF difference. A reticle with one detector sums the irradiances that share the field and forms the error from amplitude and phase (spin-scan) or frequency deviation and phase (con-scan); the unhardened “twice as bright” takeover is a ratio of those irradiances, which can stay if it is applied to in-band E and not to unitless heat. An imaging seeker attributes E to pixels and tracks the aim point of the gate (quadrant box or centroid). A flare outside the IFOV or outside the gate must not add into the aircraft’s pixel.
- What this replaces: the product `heat * seekerIntensity(...)`, the division by R² with τ = 1, the visibility test `range ≤ tailAcquisitionM * sqrt(intensity)` (that test is a fixed irradiance threshold in heat units, E ≥ 1 / tailAcquisitionM²), and the flare’s scalar heat decay used as if it were in-band intensity. The geometric lobe and the factor-of-four afterburner scale are stand-ins for a missing I(aspect, throttle) table, not measurements of spectral intensity.

### Gaps

- `src/objects/Missile.cpp` also scores heat over distance squared in a heat-seeker update; that function was not read line by line and is not specified here.
- No intensity table was filled in. Doing so from a classified signature, or from the example lock ranges in the 2009 paper, would put non-physical or non-transferable numbers back into the sim.
