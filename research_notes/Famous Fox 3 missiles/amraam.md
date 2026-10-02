# Famous Fox 3 missiles: AIM-54 Phoenix and AIM-120 AMRAAM

Open-literature notes for a real-time simulator. Each figure is tied to a page opened in this session, or is marked when the live page was blocked and the wording comes from a search-index extract of that URL. Game, forum, and unsourced range numbers are listed only so they are not turned into lock ranges. No seeker schematic, employment procedure, or construction detail is included.

Source classes used below: **manual** = multi-service brevity or a Navy training-system plan as republished; **fact sheet** = NAVAIR, Navy, or USAF page; **SAR / DOT&E / AECA** = Selected Acquisition Report, Director of Operational Test and Evaluation, or Arms Export Control Act notification; **manufacturer** = Raytheon/RTX public statement; **secondary** = Andreas Parsch's designation-systems.net (cites Friedman, Jane's, Gunston, Chant) or GlobalSecurity.org.

## What does "pitbull" / going active mean, and what handover range is published?

### Takeaway

"Pitbull" is an official brevity word for an active-radar missile, the AIM-120 given as the example, being at medium-PRF active range. No opened official source gives that event a distance. The AIM-54 is a Fox 3 because its terminal seeker is active radar, and a Fox 1 only in the separate sense that long-range midcourse is still semi-active.

### Cited Findings

- FOX (number) is "Simulated or actual launch of air-to-air weapons." FOX ONE is a "Semi-active radar-guided missile." FOX TWO is an "IR-guided missile." FOX THREE is an "Active radar-guided missile." — [ATP 1-02.1 / MCRP 3-30B.1 / NTTP 6-02.1 / AFTTP 3-2.5, March 2023, public release](https://upload.wikimedia.org/wikipedia/commons/8/83/ALSSA_Brevity.pdf) (**manual**)
- HUSKY: "Active radar missile is at high pulse repetition frequency active range." — same manual, p. 22 of the PDF text (**manual**)
- PITBULL: "Active radar guided missile (e.g., air intercept missile [AIM]-120) is at MPRF active range." — same manual, p. 31 (**manual**). The definition names the AIM-120 and does not name the AIM-54. It states no nautical miles, no seconds, and no RCS.
- CHEAPSHOT: "Active missile data link terminated between high and medium pulse repetition frequency (MPRF) active." — same manual, p. 9 (**manual**)
- AIM-120, long range: "heads for the target using inertial guidance and receives updated target information via data link from the launch aircraft. It transitions to a self-guiding terminal mode when the target is within range of its own monopulse radar set." It also has a "home-on-jam" mode. "At closer ranges AMRAAM is able to guide itself using its own radar." — [NAVAIR AMRAAM product page](https://www.navair.navy.mil/product/AMRAAM) (**fact sheet**). No distance is attached to "within range."
- USAF November 2007 sheet, same idea without the monopulse or home-on-jam sentences: "Guidance System: Active radar terminal/inertial midcourse." — [USAF fact sheet, permanent GPO copy, November 2007](https://permanent.access.gpo.gov/lps53257/www.af.mil/information/factsheets/factsheet.asp-id=79.htm) (**fact sheet**)
- AIM-54 general characteristics: "Guidance System: Semi-active and active radar homing." — [NAVAIR Phoenix backgrounder, 5 Oct 2004](https://www.navair.navy.mil/node/12701) (**fact sheet**)
- AIM-54A on that same page: "Guidance: Semi-active, update and active radar." AIM-54C: "Guidance: Semi-active, update, inertial and active radar." — same NAVAIR page (**fact sheet**)
- FAS republication of the June 1997 draft Navy training-system plan: "Semi-active and active homing radar and hydraulically operated fins direct and stabilize the missile on course to the target." The three versions then in use were AIM-54A, AIM-54C, and AIM-54 ECCM/Sealed. — [FAS AIM-54 page, citing N88-NTSP-A-50-8007C/D, June 1997](https://web.archive.org/web/20160304112626/http://fas.org/man/dod-101/sys/missile/aim-54.htm) (**manual**, via FAS HTML; the NTSP PDF itself was not opened)
- Secondary, not a manual: for maximum range the AIM-54A "flies an optimized high-altitude trajectory"; "the final 18200 m (20000 yds)" is active radar homing; minimum engagement "is about 3.7 km (2 nm), in which case active homing is used from the beginning." Parsch warns that figures "may therefore be inaccurate." — [designation-systems.net AIM-54, updated 8 Oct 2004](https://www.designation-systems.net/dusrm/m-54.html) (**secondary**: Friedman, Jane's, Gunston, Chant)
- Secondary AMRAAM wording, no handover number: the autopilot "can receive mid-course updates from the aircraft via a data link." "As soon as the target is within range, the AMRAAM activates its active radar seeker." "For the lower portions of the AMRAAM's range envelope (minimum range is said to be 2 km (2200 yds)) ... the AIM-120 is a true fire-and-forget weapon." Parsch marks range figures as "rough estimates only." — [designation-systems.net AIM-120, updated 25 July 2007](https://www.designation-systems.net/dusrm/m-120.html) (**secondary**)

### Inferences

- Under the 2023 brevity manual, a Fox 3 call means an active-radar missile has been launched. The AIM-120 fits that definition directly. The AIM-54 also fits it, because every opened official description gives it an active terminal seeker, not semi-active homing all the way to intercept. It is not "only a Fox 1."
- It is also not an AIM-120-style launch-and-leave weapon from the rail on a long shot. Official text keeps semi-active midcourse and "update" on both AIM-54A and AIM-54C. The C adds inertial. Continuous illumination is a Phoenix midcourse requirement in the secondary literature (Parsch: the AWG-9 "periodically illuminates" each target). That sentence was not on the NAVAIR fact sheet.
- Pitbull and Husky are seeker-PRF states (MPRF vs HPRF), and Cheapshot is early end of the datalink between those states. They are not published ranges. A simulator that stores "pitbull = 10 nm" or any other fixed handover is inventing a number the manual does not contain.
- The 20,000-yard Phoenix active-handover figure and the "about 2 nm" minimum are secondary and explicitly hedged ("about," "said to be," "may therefore be inaccurate"). They are not a lock range.

### Gaps

- No opened primary source states an AIM-120 active-acquisition range in nautical miles, a gimbal or off-boresight angle in degrees, or the seeker RF band. I-band / 8–10 GHz, ±25°, and 5–25 km claims appear on hobby and foreign-language pages that were not used.
- No opened source states whether aircrew were directed to call an AIM-54 "Fox 1," "Fox 3," or both. The classification above is the brevity definition applied to the published guidance description.
- National Museum of the USAF AIM-120 sheet (monopulse, home-on-jam, 40 lb warhead, Mach 4, "in excess of 30 miles," 340 lb) returned Access Denied. Those museum figures are not used. The monopulse, datalink, home-on-jam, and active-radar proximity fuze sentences are already on the opened NAVAIR page.

## What is published about loft, home-on-jam, and datalink rate?

### Takeaway

Home-on-jam is an official AIM-120 mode with no published jammer or range parameters. A loft exists in secondary Phoenix text and in 2025 manufacturer remarks about flying the AIM-120D/F3R "higher and longer," but no opened source gives a loft angle, apogee, or datalink update rate.

### Cited Findings

- Home-on-jam, AIM-120: "The AIM-120 also has a 'home-on-jam' guidance mode to counter electronic jamming. Upon intercept an active-radar proximity fuze detonates the warhead." — [NAVAIR AMRAAM page](https://www.navair.navy.mil/product/AMRAAM) (**fact sheet**)
- AIM-120D versus AIM-120C-7, AECA notification text: "The increase in capability from the AIM-120C-7 to AIM-120D consists of a two-way data link, a more accurate navigation unit, improved High-Angle Off-Boresight (HOBS) capability, and enhanced aircraft-to-missile position handoff." The AIM-120D "features a target detection device with embedded electronic countermeasures." — Congressional Record extracts of the UK and Australia notifications, [10 July 2018](https://www.congress.gov/115/crec/2018/07/10/164/115/CREC-2018-07-10-pt1-PgS4871.pdf) and [21 April 2016 notification](https://www.congress.gov/congressional-record/volume-162/issue-63/senate-section/article/S2413-1) (**AECA**). These were search-index extracts of the Record, not a full PDF re-read.
- SAR mission text, same three D features and no range percentage: the AIM-120D "will have improved accuracy via Global Positioning System (GPS) aided navigation, improved network compatibility, and enhanced aircrew survivability via a two-way datalink capability." — December 2012 SAR as hosted by GlobalSecurity, and the FY2015-budget December 2013 SAR at DTIC, search-index extracts of [the 2012 SAR PDF](https://www.globalsecurity.org/military/library/budget/fy2012/sar/amraam_december_2012_sar.pdf) and [ADA614731](https://apps.dtic.mil/sti/tr/pdf/ADA614731.pdf) (**SAR**). Full PDFs were not paginated in this session.
- Navy fact-file extract: "The AIM-120D features improved accuracy via Global Positioning System aided navigation, kinematics, lethality and hardware and software updates to enhance its electronic protection capabilities." — [Navy fact file](https://www.navy.mil/Resources/Fact-Files/Display-FactFiles/Article/2168352/aim-120-advanced-medium-range-air-to-air-missile-amraam/) (**fact sheet**). Direct fetch returned Access Denied; this is the search-index text of that URL.
- Manufacturer, 9 April 2015, no number on the range claim: the AIM-120D has "significant capability improvements, including increased range, GPS-aided navigation, two-way data link and improved weapons effectiveness." — [Raytheon release](https://www.prnewswire.com/news-releases/latest-amraam-variant-achieves-key-program-milestones-300062962.html) (**manufacturer**; search-index extract)
- Manufacturer, 16 September 2025 press calls, paraphrased by trade press and not opened as a full article: Raytheon vice president Jon Norman declined to give the range increase, called it significant, and said the D/F3R is "flying higher and longer." One account quotes him that propulsion and aerodynamics "always had the capability to go further" and that a smaller guidance section allows "a little bit bigger engine." Another account of the same remarks quotes "We didn't change propulsion" and "We just changed the way it flies for long-range shots." — [FlightGlobal, 16 Sep 2025](https://www.flightglobal.com/fixed-wing/raytheon-f3r-upgrade-delivers-longest-known-f-22-amraam-shot/164526.article); [The Aviationist, 16 Sep 2025](https://theaviationist.com/2025/09/16/longest-aim-120-amraam-shot-in-history/) (**manufacturer**, via secondary writeups; the articles themselves were not fully fetched)
- Phoenix loft, secondary only: "For maximum range, the missile flies an optimized high-altitude trajectory for reduced drag." — [designation-systems.net AIM-54](https://www.designation-systems.net/dusrm/m-54.html) (**secondary**). No altitude or angle.
- NAVAIR's Phoenix fact sheet and chronology do not state a loft angle, a cruise altitude, a datalink rate, or home-on-jam. They do record a June 1973 test shot against a BQM-34E at 110 nautical miles, described as a record, not as maximum range. — [NAVAIR Phoenix backgrounder](https://www.navair.navy.mil/node/12701) (**fact sheet**)

### Inferences

- For the simulator, AIM-120 home-on-jam is a mode flag, not a range bonus. Nothing opened describes when it is selected or how it shares the antenna with the active seeker.
- "Two-way datalink" is an official AIM-120D change relative to the C-7. Earlier missiles are described only as receiving updates. No opened source gives hertz, bits, or a maximum time between updates. Do not invent an update rate.
- "Flying higher and longer" is a manufacturer description of trajectory and software use of an existing motor, and the two press paraphrases disagree on whether the engine got slightly larger. It is not a loft table. The 2025 test distance was not disclosed.
- A Phoenix long-range shot in this model should be allowed to climb if the guidance law asks for it. The only published reason is drag. Do not hard-code 80,000–100,000 ft; that band was not on any page opened here.

### Gaps

- Datalink update rate: not found in the fact sheets, the SAR sentences opened via search, the AECA notices, or the brevity manual.
- AIM-120 loft angle, pitch program, and apogee: unpublished in opened official text.
- Phoenix home-on-jam: not stated on the NAVAIR or FAS pages opened here. ECCM improvements are stated for the C and the ECCM/Sealed variant; that is not the same sentence as home-on-jam.
- Whether the 2025 F3R shots used a larger motor or only a different trajectory: the two trade-press quotations of Norman disagree. Leave motor impulse unchanged unless a primary source is opened later.

## Which numbers are primary, and which are Jane's or marketing?

### Takeaway

Official cards give length, diameter, a weight that disagrees by variant and by office, "in excess of" or "classified" range, and guidance in words. Burn time, thrust, total impulse, seeker band, gimbal angle, and active-handover range are not in those cards. Jane's-lineage numbers in Parsch, Wikipedia ranges, and "50 percent / 100 nm / nearly doubles" claims are a different class and must stay labeled.

### Cited Findings

Variant cards follow. Where two opened sources disagree, both numbers are kept.

#### AIM-54A Phoenix

- Role and platform: long-range air-launched air-intercept missile. US Navy only, F-14 Tomcat, up to six rounds. NAVAIR chronology: sale of 274 missiles to Iran approved in 1972, final delivery May 1979, also for the F-14. — [NAVAIR](https://www.navair.navy.mil/node/12701) (**fact sheet**)
- IOC / deployed: fact-sheet "Date Deployed: 1974." Chronology on the same page: operational evaluation completed November 1974; deployed with VF-1 and VF-2 aboard USS Enterprise in November 1974; Approval for Service Use 28 January 1975. FAS/NTSP: AIM-54A IOC 1974; TECHEVAL completed November 1973; OPEVAL completed November 1974. — NAVAIR (**fact sheet**); [FAS](https://web.archive.org/web/20160304112626/http://fas.org/man/dod-101/sys/missile/aim-54.htm) (**manual** via FAS)
- Contractor: Hughes and Raytheon. Unit cost on the fact sheet: $477,131. — NAVAIR (**fact sheet**)
- Motor: fact sheet, "Solid propellant rocket motor built by Hercules," no burn time, thrust, or impulse. FAS/NTSP, describing missiles then in service: AIM-54A, AIM-54C, and AIM-54C ECCM/Sealed "use the MK 47 MOD 1 rocket motor assembly." Parsch, for the AIM-54A as produced: "Rocketdyne MK 47 or Aerojet MK 60" single-stage solid motor in an MXU-637/B propulsion section, "Mach 4+." — NAVAIR (**fact sheet**); FAS (**manual** via FAS); [Parsch](https://www.designation-systems.net/dusrm/m-54.html) (**secondary**)
- The Mk 47 Mod 0 versus Mk 60 burn-time, thrust, propellant mass, and specific-impulse tables circulating in DCS and FlyAndWire notes were not opened from a manual. They are excluded. A 1997 training plan that says all three in-service versions use Mk 47 Mod 1 does not by itself erase earlier Mk 60 production; it also does not publish a thrust curve.
- Control: "hydraulically operated fins." — FAS (**manual** via FAS). Surface count was not on the fact sheet.
- Mass, length, diameter, span, from the NAVAIR general-characteristics block (one number for the family): length 13 feet (3.9 m); weight 1,024 pounds (460.8 kg); diameter 15 inches (38.1 cm); wing span 3 feet (0.9 m). The pound/kilogram pair is arithmetically inconsistent (1,024 lb is about 464.5 kg, not 460.8 kg). — NAVAIR (**fact sheet**)
- Same page, AIM-54A variant block, different numbers: length 3.96 m; body diameter 380 mm; wingspan 0.92 m; launch weight 443 kg; warhead 60 kg HE continuous rod; fuze IR; propulsion solid; range 135 km. Prose: "Design range of 60 nm (69 mi; 111 km) was easily surpassed in testing." — NAVAIR (**fact sheet**, variant block)
- FAS/NTSP weight line: "1000 pounds - AIM-54A." Length on that table is again 13 feet (3.9 m), span 3 feet, diameter 15 inches. — FAS (**manual** via FAS)
- Parsch table, flagged inaccurate: length 4.01 m (13 ft 1.8 in); wingspan and finspan 92.5 cm (36.4 in); diameter 38.1 cm; weight 453 kg (1,000 lb); speed Mach 4.3; ceiling 24,800 m; range 130 km (72.5 nm). — Parsch (**secondary**)
- Warhead and fuze, three-way conflict. NAVAIR general block: "Proximity fuse, high explosive," warhead weight 135 pounds (60.75 kg). NAVAIR A variant line: "60 kg HE continuous rod," "Fuze: IR." Parsch narrative: "60 kg (132 lb) MK 82 blast-fragmentation" with "a MK 334 radar proximity, an IR proximity, and an impact fuze." FAS/NTSP discusses a Targeting Detecting Device and an MK 11 Mod 3 electronics retrofit on the A, not an IR-only fuze. — all three URLs above
- Range class. Official general block: "In excess of 100 nautical miles (115 statute miles, 184 km)." Speed: "In excess of 3,000 mph (4,800 kmph)." These are "in excess of" statements, not maximums. The 110 nm June 1973 drone shot is a test record on the same chronology, not a card maximum. The 60 nm "design range," 135 km variant-block range, and Parsch 72.5 nm are separate and lower. — NAVAIR; Parsch
- Not produced, so no card: AIM-54B, "Interim model. Simpler construction, non-liquid cooling. Not produced." — NAVAIR (**fact sheet**). Parsch notes some authors claim a 1977 production batch and that Navy inventory listings he saw did not show B models (**secondary**).

#### AIM-54C, including ECCM/Sealed and C+

- What changed, official prose: analog electronics replaced by a reprogrammable-memory digital processor; "faster target discrimination, longer range, increased altitude, improved beam attack capability, better ECM resistance, and greater reliability." "Continuous-rod warhead replaced by controlled fragmentation warhead." — NAVAIR variant block (**fact sheet**)
- The spec line immediately under that sentence still says "Warhead: 60 kg HE continuous rod" and "Fuze: Active radar," "Launch weight: 463 kg," "Range: 150 km," length 3.96 m, diameter 380 mm, wingspan 0.92 m. The warhead line contradicts the sentence above it and also contradicts the general-characteristics "high explosive" / 135 lb line. Keep the contradiction. — same page
- FAS/NTSP is more specific and does not use "continuous rod": AIM-54C got a DSU-28 target detection device. Serials 83001–83054 used the MK 82 Mod 0 warhead with the DSU-28. From serial 83055 (FY83 production) the warhead is WDU-29/B, "a 20-25 percent increase in effectiveness." ECCM/Sealed uses the same armament section as the C. Guidance section adds a solid-state receiver-transmitter, digital electronics unit, and inertial sensor assembly. ECCM/Sealed adds heaters and drops aircraft-supplied liquid thermal conditioning. Control: electronic servo control amplifier replaces the A autopilot. Motor line is still Mk 47 Mod 1, shared with the A. — FAS (**manual** via FAS)
- Weights on that FAS table: AIM-54C "1040 pounds" with a bracket "various, 1020-1040 pounds"; AIM-54C ECCM/Sealed "1023 pounds." — FAS
- Parsch: C weight 462 kg (1,020 lb); speed Mach 5; ceiling 30,500 m; range 150 km (80 nm); warhead "60 kg (132 lb) WDU-29/B blast-fragmentation," and he says the WDU-29/B "offers a 20 to 25 percent increase in effectiveness," matching the NTSP percentage but not the NAVAIR "controlled fragmentation" versus "continuous rod" wording. He dates sealed missiles, "sometimes referred to as AIM-54C+," to first delivery in 1986, and AIM-54C ECCM/Sealed IOC to 1988, with guidance WGU-17/B and control WCU-12/B. Early C guidance/control in his text are WGU-11/B and WCU-7/B. — Parsch (**secondary**)
- IOC conflict on the NAVAIR chronology itself: "1984 — AIM-54C reaches IOC" and "December 1986 — AIM-54C reaches Initial Operational Capability." Also "1985 — The AIM-54C was deployed to the fleet" and "July 1988 — AIM-54C ECCM/Sealed variant reached IOC." FAS/NTSP: AIM-54C IOC 1986; ECCM/Sealed IOC 1988; C OPEVAL completed August 1983; ECCM/Sealed OPEVAL completed July 1988. Parsch: C IOC 1986. Prefer 1986 for C IOC and 1988 for ECCM/Sealed, and do not hide the 1984 line. — NAVAIR; FAS; Parsch
- NAVAIR C+ "High Power Phoenix" paragraph, separate from the sealed missile: internal heaters, "high-power Traveling Wave Tube (TWT) transmitter adapted from the AIM-120 AMRAAM," low-sidelobe antenna, full-scale development begun August 1987, first fully upgraded flight 14 August 1990 direct hit on a QF-4. — NAVAIR (**fact sheet**). No separate weight or range.
- Service end, US: Navy decided in February 2002 to divest beginning FY04; divestment date 30 September 2004. Last shot cited: 15 July 2004, VF-213. — NAVAIR chronology (**fact sheet**)
- Range class for the C: do not promote 150 km, 80 nm, or "longer range" into a maximum. The only family-level official quantitative range statement remains "in excess of 100 nautical miles."

#### AIM-120A

- USAF IOC September 1991. Navy IOC September 1993. — [NAVAIR](https://www.navair.navy.mil/product/AMRAAM) (**fact sheet**). USAF sheet "Date Deployed: September 1991" agrees. — [USAF November 2007](https://permanent.access.gpo.gov/lps53257/www.af.mil/information/factsheets/factsheet.asp-id=79.htm) (**fact sheet**)
- Not field-reprogrammable. ACC sheet: "The AIM-120B, AIM-120C series, and AIM-120D missiles are field re-programmable," which leaves the A out. GlobalSecurity design page: WGU-16/B on the A; EEPROM reprogramming is stated for B and C. — ACC PDF text, search-index extract of [AIM-120_final.pdf](https://www.acc.af.mil/Portals/92/Docs/Fact%20Sheets%20-%202020%20Update/Facts%20Sheets%202022%20Final/AIM-120_final.pdf) (**fact sheet**; direct fetch Access Denied); [GlobalSecurity design page](https://www.globalsecurity.org/military/systems/munitions/aim-120-design.htm) (**secondary**, page says modified 2017, content matches a 1990s training-plan description)
- Seeker and midcourse: active radar terminal, inertial midcourse, datalink updates, monopulse terminal set, home-on-jam, active-radar proximity fuze. Band, gimbal, and handover range not stated. — NAVAIR (**fact sheet**)
- Airframe, USAF 2007 sheet, not broken out by variant: length 143.9 inches (366 cm); launch weight 335 pounds (150.75 kilograms); diameter 7 inches (17.78 cm); wingspan 20.7 inches (52.58 cm). The 335 lb and 150.75 kg pair does not match (150.75 kg is about 332 lb). Power plant: "High performance." Speed: "Supersonic." Warhead: "Blast fragmentation," no mass. — USAF 2007 (**fact sheet**)
- NAVAIR current spec line groups "AIM-120A/B/C/C-4" at 348 pounds, length 12 feet, diameter 7 inches, wingspan "AIM-120A/B, 21 inches." Speed classified. Range classified. Propulsion "Solid-fuel rocket motor." Warhead "Blast fragmentation." — NAVAIR (**fact sheet**)
- Navy fact-file extract groups "AIM-120A/B/C-4" at 348 pounds and wingspan "AIM-120A/B 21 inches," and does not put a bare "C" in that weight group. Speed classified. — Navy fact file, search-index extract (**fact sheet**)
- Motor, secondary-hosted but specific: WPU-6/B, "reduced smoke, hydroxyl terminated, polybutadiene propellant in a boost sustain configuration," steel case, integral blast tube and nozzle, removable exit cone. No thrust or burn time. — GlobalSecurity design page (**secondary**)
- ACC sheet describes that same boost-sustain HTPB layout as "the" propulsion section, without limiting it to the A. — ACC PDF extract (**fact sheet**)
- Warhead hardware names, no mass, on the GlobalSecurity design page: WDU-33/B, FZU-49/B safe-and-arm (described as a modified Mk 3 Mod 5), Mk 44 Mod 1 booster. ACC: "a warhead assembly and a MK44 MOD 1 booster threaded onto a safe and arming (SAF) device." — GlobalSecurity (**secondary**); ACC extract (**fact sheet**)
- Control: "four mid-body fixed wings, four movable rear fins." GlobalSecurity: WCU-11/B, four independently controlled electromechanical actuators; wings fixed, fins movable; A/B fins not interchangeable with C. — ACC extract; GlobalSecurity
- Parsch, estimates: WDU-33/B "23 kg (50 lb)"; weight "157 kg (345 lb)"; speed Mach 4; range "50-70 km (30-45 miles)" as a "typical quoted" band and "rough estimates only." — Parsch (**secondary**). Do not put 50 lb or Mach 4 or 50–70 km on the card as official. NAVAIR's 348 lb is the later official weight; USAF's 335 lb is the older official weight.
- Aircraft, not unique to the A: USAF 2007 list is F-15, F-16, F-22, developmental F-35, Navy F/A-18C–F. NAVAIR current list is USAF F-15, F-16, F-22, F-35A and Navy/Marine Corps F/A-18, F-35B/C, EA-18G, AV-8B. — both fact sheets
- Range class: USAF 2007 "20+ miles (17.38+ nautical miles)." That parenthetical converts 20 statute miles to nautical miles; it is an "in excess of" floor for an early public sheet, not a kinematic maximum, and it is not 20+ nm. NAVAIR now says range is classified. GlobalSecurity's "55–75 km" for the A is secondary.

#### AIM-120B

- Change that is actually sourced: new guidance section WGU-41/B, software in reprogrammable EPROM, new digital processor, other electronics. First delivered "late 1994." — Parsch (**secondary**). GlobalSecurity variants page: "AIM-120B deliveries began in FY 94," and B/C are reprogrammable through the umbilical with Common Field-level Memory Reprogramming Equipment. — [GlobalSecurity variants](https://www.globalsecurity.org/military/systems/munitions/aim-120-variants.htm) (**secondary**; the page still says US deliveries "are of the AIM-120C," so it is stale)
- ACC: B, C-series, and D are field re-programmable. — ACC extract (**fact sheet**)
- No opened source gives the B a new motor, new warhead mass, new wingspan, or a new range. NAVAIR and the Navy fact file keep A and B in the same 348 lb and 21-inch group.
- Parsch's 50–70 km band is for "AIM-120A/B" and is labeled an estimate.

#### AIM-120C-3

- No opened official sentence says "the C-3 clipped the wings" as opposed to the C series as a whole.
- What is official: wingspan "AIM-120C/D, 19 inches" against 21 inches for A/B (NAVAIR and Navy fact file). GlobalSecurity: the C "utilizes 'clipped' wings and fins" for F-22 internal carriage, and those surfaces "are not interchangeable" with A/B. Parsch: basic AIM-120C, which he calls P3I Phase 1, "clipped wings and fins," guidance unit WGU-44/B, "first delivered in 1996." Navy fact-file extract: "The AIM-120C series began deliveries in 1996." — NAVAIR; Navy fact file; GlobalSecurity; Parsch
- The Navy fact file is the opened-index source that uses the designation C-3: "the Navy fielded the Advanced Electronic Protection Improvement Program (EPIP) for AIM-120C3-C7 missiles in September 2019." So C-3 is a fielded configuration that later received a software/electronic-protection update. It does not say EPIP changed the motor or the mass. — Navy fact file extract (**fact sheet**)
- Weight: NAVAIR puts undifferentiated "C" and "C-4" with A/B at 348 lb, and starts the 356 lb group at C-5. That is the evidence that early C-series rounds, including whatever the fact sheet means by "C" and by C-4, were not in the heavier group. — NAVAIR (**fact sheet**)
- Gap: whether C-1 and C-2 were produced, and whether C-3 was the first fielded clipped round, was not settled by a page opened here.

#### AIM-120C-4

- Parsch: first P3I Phase 2 missile, "improved WDU-41/B warhead," "first delivered in 1999." His table then gives the C-5 column the "18 kg (40 lb) WDU-41/B," against 23 kg WDU-33/B for A/B. — Parsch (**secondary**)
- NAVAIR weight: C-4 stays in the 348 lb group with A/B, not the 356 lb C-5/6/7 group. Navy fact file agrees (C-4 at 348 lb). — both **fact sheets**
- Inference, labeled as such: a warhead change that did not move the official weight class is consistent with Parsch's split (new warhead on C-4, heavier motor only from C-5). It does not prove the 18 kg versus 23 kg figures, which remain secondary.
- GlobalSecurity's Phase 2 paragraph bundles "a larger rocket motor, an improved warhead, a quadrant target detection device" and does not assign them to C-4 versus C-5 versus C-6. — GlobalSecurity variants (**secondary**, and older than the dash-number split)

#### AIM-120C-5

- Official weight break: NAVAIR "AIM-120C5/6/7, 356 pounds" against 348 pounds through C-4. Navy fact file: "AIM-120C 5/6/7/D 356 pounds," so that page also puts the D at 356, not at NAVAIR's 358. — NAVAIR (**fact sheet**); Navy fact file extract (**fact sheet**)
- Parsch: C-5 is a C-4 with "a slightly larger motor in the new WPU-16/B propulsion section and a new shorter WCU-28/B control section" plus ECCM. "Deliveries of the AIM-120C-5 began in July 2000." His range cell "> 105 km (65 miles)" is in the table he marks "rough estimates only." — Parsch (**secondary**)
- ACC sheet, different variant: "Beginning with the AIM-120C-7 the missile has an enhanced motor with an additional 5 inches of propellant and is commonly referred to as the '+5 rocket motor.' ... A shortened control actuation section (SCAS) is used with the +5 rocket motor." — ACC extract (**fact sheet**)
- Keep both. The weight step published by NAVAIR and the Navy is at C-5. The only official sentence that says "+5 inches" and names a dash number puts that motor at C-7. Parsch's WPU-16/B-at-C-5 account agrees with the weight step and disagrees with the ACC dash number. A forum citation of a 2003 USAF Weapons File page ("AIM-120C-5 – Lot 12. Implements 5 inch longer enhanced Rocket Motor") was not opened and is not used as a fact.
- No burn time, thrust, or impulse was on any official page. Forum and DCS figures (about 7.75 s, about 15.5 kN, about 51 kg propellant, Isp about 240 s) are excluded.

#### AIM-120C-6

- Parsch: followed the C-5 on the line and "features an updated TDD (Target Detection Device)." — Parsch (**secondary**)
- GlobalSecurity Phase 2 list includes "a quadrant target detection device" without the C-6 dash number. — GlobalSecurity (**secondary**)
- Official weight: same 356 lb group as C-5 and C-7. No separate IOC, range, or fuze mass was on the NAVAIR or Navy cards.
- Navy fact file includes C-6 in the C3–C7 set that received EPIP in September 2019. — Navy fact file extract (**fact sheet**)

#### AIM-120C-7

- Navy fact file: "The AIM-120C-7 missile variant reached IOC in FY 2008." — Navy fact file extract (**fact sheet**)
- Parsch: P3I Phase 3, development begun 1998, "improved ECCM with jamming detection, an upgraded seeker, and longer range" (amount not stated). Tested against "combat-realistic targets" in August and September 2003. He wrote in 2007 that fielding was just starting, after a planned 2004 IOC had slipped. The Navy's later "IOC in FY 2008" is the date to keep. — Parsch (**secondary**); Navy fact file (**fact sheet**)
- GlobalSecurity variants: Phase 3 "scheduled to begin production in FY04 as the AIM-120C-7," new guidance-section hardware and software; antenna, receiver, and signal processing "compressed to create room for future growth"; some software rehosted to C++. — GlobalSecurity (**secondary**)
- DOT&E search-index extract: "In October 2014, the Air Force completed EPIP Basic Phase III operational testing for AIM-120C-7 missiles." — [FY2014 DOT&E AMRAAM extract](https://www.globalsecurity.org/military/library/budget/fy2014/dot-e/af/2014amraam.pdf) (**DOT&E**; PDF not fully read)
- ACC attributes the +5-inch motor to this dash number and later. See the C-5 card. Weight stays in the 356 lb official group.
- "Longer range" here is an unquantified secondary and, via the +5-inch sentence, an official hardware claim. It is not a new maximum to type in as Rmax.
- A Raytheon AIM-120C-7 brochure block (length 12 ft, diameter 7 in, wing span 17.5 in, fin span 17.6 in, weight 356 lb, warhead 45 lb, fuzing "Proximity and contact") appeared only as a search hit of a forum-hosted PDF. It was not opened from RTX. The 45 lb warhead and 17.5/17.6 in spans are not card values. They do conflict with Parsch's 18 kg (40 lb) and with the Navy's 19-inch C/D wingspan, which is why they are noted and not averaged.

#### AIM-120C-8

- 2019 Arms Export Control Act annex, Federal Register text: "The AIM-120C-8 is a form, fit, function refresh of the AIM-120C-7 and is the next generation to be produced. The capabilities of the AIM-120C-7 and C-8 are identical." — search-index extract of the 17 October 2019 Korea notification as mirrored at [Justia / Federal Register](https://regulations.justia.com/regulations/fedreg/2019/12/16/2019-26979.html) (**AECA**). Direct fetch of that mirror was blocked; a 2022 Japan notification republished in the Federal Register uses the same "form, fit, and function refresh of the AIM-120C-7" sentence as far as the extract went: [FR text via Govinfo, 13 June 2024](https://www.govinfo.gov/content/pkg/FR-2024-06-13/pdf/2024-12955.pdf)
- Manufacturer, 18 April 2023, later and different: "F3R testing continues with the AIM-120 C-8 variant – designed for international customers." "All AMRAAMs planned for production are D3 or C8 variants incorporating the F3R functionality." — [Raytheon / RTX release](https://www.prnewswire.com/news-releases/most-advanced-amraam-variant-aim-120d-3-completes-critical-milestone-for-operational-use-301800951.html) (**manufacturer**, page opened)
- Do not collapse these. In 2019 the US government told Congress the C-8's capabilities were identical to the C-7. In 2023 the manufacturer described the C-8 as the international Form-Fit-Function Refresh article built alongside the D-3. Neither statement gives a range, a motor delta, GPS, or a two-way datalink. Parsch in 2007 said the designation AIM-120C-8 had been the former name of what became the AIM-120D. That is a development-name note, not the 2019–2023 production C-8.
- Jane's-style "C-8 is the export D, range about 160 km" was a search snippet of a 2025 Jane's news item. The article was not opened. That range is not used.

#### AIM-120D

- Official capability delta from C-7, AECA: two-way datalink, more accurate navigation unit, improved HOBS, enhanced aircraft-to-missile position handoff, and a target detection device with embedded electronic countermeasures. — Congressional Record extracts cited above (**AECA**)
- SAR: GPS-aided navigation, improved network compatibility, two-way datalink. No percentage. — 2012 and 2013 SAR extracts (**SAR**)
- Navy fact file: GPS-aided navigation, "kinematics, lethality," and electronic-protection hardware and software. Joint procurement of the D series "began in fiscal 2006." "The Navy achieved IOC of the latest hardware variant AIM-120D in January 2015." — Navy fact file extract (**fact sheet**)
- December 2014 SAR extract: services completed operational testing in July 2014; "The US Navy declared IOC in January 2015 and the US Air Force authorized operational fielding on January 26, 2015." — search-index extract of [15-F-0540 AMRAAM SAR Dec 2014](https://www.esd.whs.mil/Portals/54/Documents/FOID/Reading%20Room/Selected_Acquisition_Reports/FY_2014_SARS/15-F-0540_AMRAAM_SAR_Dec_2014.PDF) (**SAR**)
- DOT&E FY2015 extract: FOT&E completed July 2014; "The missile was fielded in January 2015"; 1,405 AIM-120Ds delivered as of 14 October 2015. FY2014 DOT&E extract: captive-carry mean time between failure 452.5 hours against a 450-hour requirement "desired two years after Initial Operational Capability." — [FY15 DOT&E](https://www.globalsecurity.org/military/library/budget/fy2015/dot-e/af/2015amraam.pdf); [FY14 DOT&E](https://www.globalsecurity.org/military/library/budget/fy2014/dot-e/af/2014amraam.pdf) (**DOT&E**, search-index extracts)
- Weight conflict, both official: NAVAIR "AIM-120D, 358 pounds." Navy fact file includes D in the 356-pound C-5/6/7/D group. Wingspan for C/D is 19 inches on both. — NAVAIR; Navy fact file
- ACC: "A Value Control Actuation Section is the replacement for the SCAS, and is used with AIM-120D with modified Guidance Sections." The wording is quoted as indexed; it may mean a new actuation section rather than a part named "Value." — ACC extract (**fact sheet**)
- Manufacturer range language with no figure: "increased range" (April 2015 Raytheon release). Parsch in 2007, before fielding: "a 50% increase in range," plus two-way datalink, GPS-enhanced IMU, "expanded no-escape envelope," improved HOBS. GlobalSecurity variants: "reportedly has a range 50 percent greater ... up to a reported 100 nm." — Raytheon (**manufacturer**); Parsch and GlobalSecurity (**secondary**)
- The 50 percent and 100 nm figures are not in the SAR or AECA sentences opened here. They are marketing or secondary "reportedly" numbers. Air & Space Forces Magazine's "nearly doubles" claim for a later variant was not on a page that returned body text. It is not used.
- HOBS: the AECA text says "improved" HOBS versus C-7 and gives no angle. No degree value is published in the sources opened here.
- Navy fact file also says SIP 2 for the AIM-120D was fielded in June 2021, and "SIP 3 for AIM-120D is planned to field in 2022." That "planned" line is older than the 2023 D-3 audit below. Do not treat SIP-3 software and F3R hardware as the same event just because both use a "3."

#### AIM-120D-3

- Opened manufacturer statement, 18 April 2023: USAF completed the Functional Configuration Audit of the AIM-120D-3. It "is on-track toward fielding by both the Air Force and Navy this year." Hardware is "15 upgraded circuit cards" under Form, Fit, Function Refresh (F3R). Software is "System Improvement Program-3F." No range, weight, or motor figure is in the release. The same release is the C-8 international-F3R sentence quoted above. — [Raytheon / RTX, 18 Apr 2023](https://www.prnewswire.com/news-releases/most-advanced-amraam-variant-aim-120d-3-completes-critical-milestone-for-operational-use-301800951.html) (**manufacturer**)
- What is not established: an IOC day. "On-track toward fielding" in April 2023 is not a declaration. A magazine line that fielding occurred in March 2024 was not confirmed because that page did not return article text.
- What is marketing until the Air Force states a number: any claim that F3R or SIP-3F doubled range, added 5 inches of propellant, or changed burn time. The 2025 manufacturer remarks, as paraphrased, attribute extra reach to flying higher and longer, and they disagree with each other about a slightly larger engine.

#### AIM-260

Performance, dimensions, mass, motor type, seeker, and IOC are unpublished in the official text opened here. Stop.

The 17 March 2026 Federal Register notice of a proposed sale to Australia is the opened official description. The AIM-260 Joint Advanced Tactical Missile "is a GPS-aided air superiority missile with increased range and effectiveness over existing air-to-air weapons with Precise Positioning Services provided by Selective Availability Anti-Spoofing Module or M-Code." Anti-tamper measures are stated. Australia's request was up to 450 rounds, plus 5 integration test vehicles and 30 guided test vehicles. The principal contractor is Lockheed Martin Missiles and Fire Control, Orlando, Florida. A guided test vehicle replaces the warhead with telemetry. Highest classification of the articles in the notice is SECRET. No range, speed, weight, length, or seeker band is in the notice. — [Federal Register, 17 Mar 2026, Transmittal 26-03](https://www.federalregister.gov/documents/2026/03/17/2026-05140/arms-sales-notification) (**AECA**)

Secondary pages that say "at least 200 km," Mach 5, "120++ miles," dual-pulse, or "same size as AMRAAM" were not used. Wikipedia's own range line is tagged as needing a citation. Those figures must not become a card.

#### Numbers that are only a class of claim

- Official "in excess of" or "classified": USAF AMRAAM "20+ miles"; NAVAIR AMRAAM range and speed "Classified"; Phoenix range "in excess of 100 nautical miles" and speed "in excess of 3,000 mph."
- On an official URL but inconsistent with the same page's general-characteristics block: Phoenix variant-block 135 km and 150 km, 443 kg and 463 kg, and the 60 nm "design range."
- Test record, not a maximum: Phoenix 110 nm in June 1973.
- Secondary estimates, Parsch/Jane's/Friedman line: AIM-120A/B 50–70 km, C-5 "> 105 km," Mach 4, warhead 23 kg then 18 kg; Phoenix 72.5 nm and 80 nm, Mach 4.3 and Mach 5, active for the last 20,000 yards.
- "Reportedly" / manufacturer without a figure or with a figure the government text does not repeat: D "50 percent," "100 nm," "increased range," "significant," "higher and longer."
- Not opened, do not use: USAF museum 30 miles and Mach 4; Hellenic Air Force page (fetch denied) 35 nm / 40 nm; Jane's 160 km for C-8; Wikipedia's 40 / 49 / 70–86 nmi family ranges and the "booster only from C-5" motor-mode sentence; DCS and forum thrust, burn time, and pitbull distances.

### Inferences

- A single "AIM-120" or "AIM-54" card is not supported. Mass class changes at C-5 on both Navy pages. Reprogrammability starts at B. Clipped span is the C/D family. Two-way datalink and GPS-aided navigation are D-class statements, not C-7 statements. C-8 must be its own card because the 2019 government sentence and the 2023 manufacturer sentence do not say the same thing.
- The ACC "+5 inches at C-7" sentence and the C-5 weight step can both be stored as sourced claims. They must not be silently merged into one motor.

### Gaps

- Burn time, thrust, total impulse, propellant mass, and boost-versus-sustain split: not published on the fact sheets. ACC says the motor is "a boost-sustain configuration" and does not say a later dash number stopped being boost-sustain. Wikipedia's "booster only" line was not adopted because that page was not used as an opened source and the ACC sheet does not say it.
- Warhead mass, official: the fact sheets say "blast fragmentation" and do not give pounds, except the Phoenix general-characteristics 135 lb, which disagrees with other lines on the same NAVAIR page.
- Per-variant carrier list: fact sheets list aircraft for "AMRAAM," not for each dash number. The C-series clipped surfaces are tied to F-22 internal carriage. F-35, F/A-18E/F, EA-18G, and AV-8B appear on later official platform lists. Parsch's inclusion of the F-14D was not confirmed on a Navy fact sheet opened here.
- AIM-120D-3 fielding date after the April 2023 audit: not found in an opened primary sentence.
- Full SAR and DOT&E PDFs were not read page by page. Sentences quoted from them came from search-index extracts of those PDFs. If a later pass opens the PDF, re-check the wording before it is treated as a transcription.

## How should a simulator store an AIM-120 card without a hard-coded range?

### Takeaway

Store geometry, mass, guidance-mode flags, and a motor whose impulse is explicitly unknown. Compute flyout from launch state. Do not store a maximum range or a pitbull range for any of these missiles.

### Cited Findings

- NAVAIR's current AMRAAM card already refuses the number: "Range: Classified." Speed is classified. The older USAF "20+ miles" is a floor, and the sheet's own nautical-mile parenthesis is just the conversion of 20 statute miles. — [NAVAIR](https://www.navair.navy.mil/product/AMRAAM); [USAF 2007](https://permanent.access.gpo.gov/lps53257/www.af.mil/information/factsheets/factsheet.asp-id=79.htm)
- Official midcourse is inertial plus datalink, then a monopulse active terminal seeker "when the target is within range of its own" set, plus home-on-jam, plus an active-radar proximity fuze. "Within range" is not a constant. — NAVAIR
- The brevity manual's pitbull and husky entries are PRF-state calls, with the AIM-120 as the pitbull example, and they contain no distance. — [March 2023 brevity manual](https://upload.wikimedia.org/wikipedia/commons/8/83/ALSSA_Brevity.pdf)
- Physical quantities that do exist, with conflicts to store rather than average: length 12 ft or 143.9 in; diameter 7 in; A/B span 21 in (USAF sheet 20.7 in); C/D span 19 in on Navy pages; mass 335 lb (USAF 2007), 348 lb (A/B and early C through C-4), 356 lb (C-5/6/7, and D on the Navy fact file), 358 lb (D on the NAVAIR page only). — sources in the cards above
- Motor facts safe to store as qualitative: single reduced-smoke HTPB solid motor, boost-sustain wording on the ACC sheet and on the WPU-6/B description; a disputed extra propellant length (+5 in) whose dash-number start is C-5 by weight and Parsch, and C-7 by the ACC sentence; shortened control section associated with that longer motor. No thrust curve. — ACC extract; GlobalSecurity design page; NAVAIR weights; Parsch

### Inferences

- Range in this sim is an output of mass, reference area, drag, launch altitude and Mach, and a rocket model. A missing impulse should stay missing or be an explicit scenario assumption with a source tag of "not from a manual." It should not be backed out of "20+ miles" or "100 nm."
- Active handover is a detection event. Published inputs do not include seeker power, bandwidth, or detection range. A fixed nautical-mile gate will not get those physics back.
- Guidance enum that the sources actually support: `inertial`, `datalink_receive` (all AIM-120s described), `datalink_two_way` (D, per AECA/SAR, not per C-7), `gps_aided_nav` (D, per SAR/AECA), `active_monopulse`, `home_on_jam`, `active_radar_proximity_fuze`. Phoenix adds `semi_active_midcourse` and, for the C, `inertial`. Phoenix does not get `home_on_jam` from the pages opened here.
- HOBS is a boolean "improved on D versus C-7" with angle unknown. Do not invent ± degrees.
- C-8 gets its own row. Fields that are not in the 2019 "identical to C-7" sentence or the 2023 "international F3R" sentence stay null.

### Gaps

- There is still no open official thrust, burn time, or total impulse with which to close the motor model.
- There is still no open official active-seeker range, band, or gimbal with which to close the terminal model.

## Implementation notes for missilesim

Store one catalog row per dash number, not one AMRAAM row and not one Phoenix row. Minimum fields the opened sources can fill or explicitly leave unknown:

- `designation`, `seeker_terminal` (`active_monopulse` for AIM-120; `active_radar` for AIM-54, band unknown), `midcourse` (AIM-120: inertial + datalink; AIM-54A: semi-active + update; AIM-54C: semi-active + update + inertial)
- `datalink` (`receive` or, for AIM-120D and D-3, `two_way`), `gps_aided` (true only where SAR or AECA said so: D family; also the AIM-260's only opened guidance fact, and that missile has no performance card)
- `home_on_jam` (true for AIM-120 from the NAVAIR page; unknown for Phoenix)
- `hobs` (unknown angle; D is sourced only as "improved" relative to C-7)
- `motor_profile` (`boost_sustain_htpb` as the family description; `propellant_length_delta_in` held as a disputed field, not applied to both C-5 and C-7 at once)
- `thrust_n`, `burn_time_s`, `total_impulse_ns` = null
- `mass_kg`, `length_m`, `diameter_m`, `wing_span_m`, `fin_span_m` as source-tagged pairs where the pages disagree, not as averages
- `controls` (AIM-120: four fixed mid-body wings, four movable tail fins; clipped flag for C/D; SCAS with the longer motor; D actuation section replaced per the ACC sentence. AIM-54: hydraulically operated fins, count unpublished)
- `warhead` and `fuze` as enumerated sourced strings, including the Phoenix continuous-rod versus blast-fragmentation versus WDU-29/B conflict, rather than one picked mass
- `ioc_year` and `carriers` from the fact sheets, with the AIM-54C 1984-versus-1986 IOC conflict kept
- no `r_max`, no `r_pitbull`, no `r_husky`, no `lock_range`

Famous numbers that must not be turned into a lock range or an active-handover range: USAF "20+ miles"; NAVAIR "range classified"; Phoenix "in excess of 100 nautical miles" and the June 1973 110 nm test shot; the NAVAIR variant-block 60 nm, 135 km, and 150 km; Parsch's 20,000 yards, 2 nm, 50–70 km, "> 105 km," 72.5 nm, and 80 nm; any "50 percent," "100 nm," "160 km," or "nearly doubles"; Wikipedia and Hellenic dashboard ranges; every DCS, forum, and heatblur burn time or thrust. Pitbull is an MPRF-state call. Husky is an HPRF-state call. Neither is a distance.
