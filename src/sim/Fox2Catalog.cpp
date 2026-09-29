#include "sim/Fox2Catalog.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace missilesim::fox2
{
    namespace
    {
        const char *kStandInMotorNote =
            "Impulse was not published. The flyout uses the AIM-9B thrust-to-weight for 2.2 s, and that "
            "propellant is not in the mass. It is not a manufacturer curve, and it will not be stretched "
            "to a brochure range.";

        void standInMotor(Spec &spec)
        {
            spec.motor = MotorConfidence::UnpublishedStandIn;
            spec.thrustN = 0.0f;
            spec.burnS = 0.0f;
            spec.propellantKg = 0.0f;
            spec.motorNote = kStandInMotorNote;
        }

        void shapeCoefficient(Spec &spec)
        {
            spec.aeroGIsShapeCoefficient = true;
            spec.cnOverride = 0.0f;
            spec.structuralG = 0.0f;
            spec.structuralGPublished = false;
        }

        void sizedToG(Spec &spec, float structuralG)
        {
            spec.aeroGIsShapeCoefficient = false;
            spec.cnOverride = 0.0f;
            spec.structuralG = structuralG;
            spec.structuralGPublished = true;
            spec.aeroStructuralG = 0.0f;
        }

        void inferredArm(Spec &spec, float metres)
        {
            spec.armDistanceM = metres;
            spec.armDistancePublished = false;
        }

        void aim9dArm(Spec &spec)
        {
            // The AIM-9D handbook says not to shoot under 1,000 ft. Later Sidewinder
            // arming was not reopened, so this window is reused and tagged inferred.
            spec.armDistanceM = 1000.0f * 0.3048f;
            spec.armDistancePublished = false;
        }

        void unpublishedTrack(Spec &spec)
        {
            spec.trackRateDegPerS = kUnpublishedTrackRateDegPerS;
            spec.trackRatePublished = false;
        }

        void jetVane(Spec &spec)
        {
            spec.hasTvc = true;
            spec.vaneDeg = kJetVaneDegrees;
            spec.vanePublished = false;
        }

        void jetTab(Spec &spec)
        {
            spec.hasTvc = true;
            spec.vaneDeg = kJetTabDegreesUncertain;
            spec.vanePublished = false;
        }

        Spec aim9b()
        {
            Spec spec;
            spec.id = "aim-9b";
            spec.displayName = "AIM-9B";
            spec.family = "Sidewinder";
            spec.massKg = kAim9bMassKg;
            spec.lengthM = kAim9bLengthM;
            spec.diameterM = kAim9bDiameterM;
            spec.spanM = 22.0f * 0.0254f;
            spec.thrustN = kAim9bThrustN;
            spec.burnS = kAim9bBurnS;
            spec.propellantKg = 0.0f;
            spec.motor = MotorConfidence::PublishedAverage;
            spec.motorNote = "OP 2309: 8,440 lbf·s in 2.2 s at 70 °F, spread evenly because the manual does not print a thrust curve. Propellant mass was not published, so launch mass is held constant.";
            spec.structuralG = kAim9bStructuralG;
            spec.structuralGPublished = true;
            spec.cnOverride = kAim9bCnMax;
            spec.aspect = AspectKind::RearHemisphere;
            spec.ifovDeg = 4.0f;
            spec.ifovPublished = true;
            spec.gimbalDeg = 25.0f;
            spec.gimbalPublished = true;
            spec.trackRateDegPerS = 11.0f;
            spec.trackRatePublished = true;
            spec.irccm = IrccmKind::None;
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.inhibitS = 0.5f;
            spec.inhibitPublished = true;
            spec.guidanceLimitS = 20.0f;
            spec.guidanceLimitPublished = true;
            spec.selfDestructS = 24.0f;
            spec.selfDestructPublished = true;
            spec.armDistanceM = 150.0f;
            spec.armDistancePublished = true;
            spec.proximityM = 30.0f * 0.3048f;
            spec.proximityPublished = true;
            spec.card = "NAVWEPS OP 2309 rear-aspect lead-sulfide seeker. Gimbal 25° is the figure OP 3353 increased to 40°; OP 2309 also says 30°, and that conflict is kept. Track rate 11°/s is Kopp. CN 5.49 is derived from 4.2 g at 50,000 ft and Mach 2.3 on the 160 lb launch mass. A straight tail chase at the in-band drag scale is still about 4.7 times the sea-level handbook range and 1.5 times the 50,000 ft range; Cd0 was not pushed outside Fleeman's band to close that gap. Arm 150 m is the early edge of 480–840 ft. The influence fuze is about 2,000 ft at burnout; the proximity radius is the manual's 30 ft.";
            return resolve(spec);
        }

        Spec aim9d()
        {
            Spec spec;
            spec.id = "aim-9d";
            spec.displayName = "AIM-9D";
            spec.family = "Sidewinder";
            spec.massKg = kAim9dMassKg;
            spec.lengthM = 2.87f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.63f;
            spec.thrustN = kAim9dThrustN;
            spec.burnS = kAim9dBurnS;
            spec.motor = MotorConfidence::PublishedAverage;
            spec.motorNote = "OP 3353 average 3,500 lbf for 5 s. A conflicting 2,645 lbf is below the manual's minimum impulse and is not used as the flyout.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AfterburnerNoseGate;
            spec.afterburnerScalesTail = true;
            spec.noseGateHalfAngleDeg = 20.0f;
            spec.noseGateFraction = kAim9dNoseIntensityFractionUnpublished;
            spec.ifovDeg = 2.5f;
            spec.ifovPublished = true;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = true;
            spec.trackRateDegPerS = 12.0f;
            spec.trackRatePublished = true;
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.guidanceLimitS = 60.0f;
            spec.selfDestructS = 60.0f;
            spec.armDistanceM = 1000.0f * 0.3048f;
            spec.armDistancePublished = true;
            spec.proximityM = 17.0f * 0.3048f;
            spec.proximityPublished = false;
            spec.card = "Rear aspect, plus a head-on gate only while the target is in afterburner and within 20° of the nose. Afterburner doubles detection range, so the rear lobe is four times brighter; the nose-gate intensity was not published and is held at 0.05 of the unboosted tail, not at the all-aspect 0.2 bound. The AI jet has no reheat flag: throttle above 0.85 stands in for it. Gimbal 40° is the handbook half-angle. Kopp's 'slightly beyond 25°' is the conflict and is not used. The 34 ft continuous-rod ring is a ring diameter; 17 ft is the derived radius, not a published miss distance. 60 s is the gas generator, used here as the guidance-power limit. CN is the AIM-9B shape coefficient, labeled not a measured AIM-9D coefficient, so the heavier round pulls less g. Parsch flags his 2.87 m / 0.63 m / 195 lb table as possibly inaccurate.";
            return resolve(spec);
        }

        Spec aim9h()
        {
            Spec spec;
            spec.id = "aim-9h";
            spec.displayName = "AIM-9H";
            spec.family = "Sidewinder";
            spec.massKg = 186.0f * 0.45359237f;
            spec.lengthM = 2.87f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.63f;
            standInMotor(spec);
            spec.motorNote = "The H uses a Mk 36. No new thrust-time pair was published, so the 1960s 3,500 lbf / 5 s line is not copied onto it. The flyout is the AIM-9B thrust-to-weight stand-in.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::RearHemisphere;
            spec.ifovDeg = 2.5f;
            spec.ifovPublished = false;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            spec.trackRateDegPerS = 20.0f;
            spec.trackRatePublished = true;
            spec.homing = LaunchHoming::LockBeforeLaunch;
            aim9dArm(spec);
            spec.card = "Parsch: 186 lb, solid-state guidance, track rate 20°/s. Kopp only says the track rate is greater than 12°/s and that the G optical system was essentially retained. The 2.5° IFOV and 40° gimbal are that inference, not a new measurement. Still rear-aspect lead sulfide. No head-on lock.";
            return resolve(spec);
        }

        Spec aim9j()
        {
            Spec spec;
            spec.id = "aim-9j";
            spec.displayName = "AIM-9J";
            spec.family = "Sidewinder";
            spec.massKg = 77.0f;
            spec.lengthM = 3.05f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.58f;
            standInMotor(spec);
            spec.motorNote = "Mk 17, but the AIM-9B manual's 8,440 lbf·s is a 1966 Mod and is not copied onto the J. Impulse unpublished; AIM-9B thrust-to-weight stand-in.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::RearHemisphere;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            spec.trackRateDegPerS = 16.5f;
            spec.trackRatePublished = true;
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.guidanceLimitS = 40.0f;
            aim9dArm(spec);
            spec.card = "Kopp/Parsch: about 77 kg, 3.05 m, span 0.58 m, rear-aspect PbS, track 16.5°/s. Gimbal and IFOV were not published; 40° is a sim stop, not the AIM-9D measurement. Kopp's 40 s figure is the longer-burning gas generator and is the guidance limit, not a destruct clock. 'Doubled the single-plane g' has no absolute g, so the shape coefficient is used and the longer body only adds skin friction.";
            return resolve(spec);
        }

        Spec limaFamily(const char *id, const char *name, float massKg, float lengthM, IrccmKind irccm, const char *irccmNote, bool smoke, const char *card)
        {
            Spec spec;
            spec.id = id;
            spec.displayName = name;
            spec.family = "Sidewinder";
            spec.massKg = massKg;
            spec.lengthM = lengthM;
            spec.diameterM = 0.127f;
            spec.spanM = 0.63f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = irccm;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = irccmNote;
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.reducedSmoke = smoke;
            aim9dArm(spec);
            spec.card = card;
            return resolve(spec);
        }

        Spec aim9p5()
        {
            Spec spec;
            spec.id = "aim-9p-5";
            spec.displayName = "AIM-9P-5";
            spec.family = "Sidewinder";
            spec.massKg = 190.0f * 0.45359237f;
            spec.lengthM = 10.0f * 0.3048f;
            spec.diameterM = 0.127f;
            spec.spanM = 1.9f * 0.3048f;
            standInMotor(spec);
            spec.motorNote = "SR.116, not the L/M Mk 36. Impulse was not published. The J-line body is longer, which raises skin friction, and no lower g was invented from Kopp's 'less agile' remark.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            spec.trackRateDegPerS = 16.5f;
            spec.trackRatePublished = false;
            spec.irccm = IrccmKind::Rise;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = "P-5 adds a counter-countermeasures capability. The circuit was not published. Rise-rate rejection here is the open 2.5:1-in-40 ms illustration, and a steady source twice as bright can still seduce it.";
            spec.homing = LaunchHoming::LockBeforeLaunch;
            aim9dArm(spec);
            spec.card = "J/N airframe with an all-aspect InSb seeker. Kopp: 10.0 ft, span 1.9 ft, 190 lb, track rate greater than 16.5°/s, modeled at that lower bound. Smoke is not assumed reduced: P-2 and P-3 are the reduced-smoke P variants, and P-5's smoke was not stated. Gimbal is an unpublished 40° sim stop.";
            return resolve(spec);
        }

        Spec aim9x(const char *id, const char *name, LaunchHoming homing, bool rearHemisphere, const char *card)
        {
            Spec spec;
            spec.id = id;
            spec.displayName = name;
            spec.family = "Sidewinder";
            spec.massKg = 186.0f * 0.45359237f;
            spec.lengthM = 3.02f;
            spec.diameterM = 5.0f * 0.0254f;
            spec.spanM = 17.6f * 0.0254f;
            standInMotor(spec);
            spec.motorNote = "NAVAIR says the AIM-9X incorporates the AIM-9M rocket motor. Mk 139 is the vectored service name, not a second impulse class. Range and speed are classified and were not solved. The flyout is the AIM-9B thrust-to-weight stand-in.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 90.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Imaging focal plane. Separation of about 1° is an unpublished gate, not a Raytheon angle.";
            jetVane(spec);
            spec.homing = homing;
            spec.rearHemisphereDesignation = rearHemisphere;
            spec.reducedSmoke = true;
            aim9dArm(spec);
            spec.card = card;
            return resolve(spec);
        }

        Spec r3s()
        {
            Spec spec;
            spec.id = "r-3s";
            spec.displayName = "R-3S";
            spec.family = "Russia";
            spec.massKg = 75.3f;
            spec.lengthM = 2.838f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.528f;
            spec.thrustN = kR3sThrustN;
            spec.burnS = kR3sBurnS;
            spec.propellantKg = 20.5f;
            spec.motor = MotorConfidence::DerivedAverage;
            spec.motorNote = "Motor-page impulse floor 38,100 N·s. Burn 1.7 s at +60 °C and 3.2 s at −54 °C. 2.45 s is the midpoint, so thrust is 38,100 / 2.45. Hot would be about 22,400 N and cold about 11,900 N. The printed 25,000 N maximum is not the average. Propellant 20.5 kg is inside the 75.3 kg launch mass.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::RearThroughForwardBeam;
            spec.ifovDeg = 3.5f;
            spec.ifovPublished = true;
            spec.gimbalDeg = 28.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.inhibitS = 0.6f;
            spec.inhibitPublished = false;
            spec.guidanceLimitS = 21.0f;
            spec.guidanceLimitPublished = true;
            spec.selfDestructS = 22.0f;
            spec.selfDestructPublished = false;
            spec.armAfterBurnoutS = 0.4f;
            spec.proximityM = 9.0f;
            spec.proximityPublished = true;
            spec.card = "K-13A. Full axle ±28°. The page also prints a 50° tracking-field cone and a 3°30′ clamping field; those are not treated as the same angle. 3°30′ is the IFOV. The 1/4–3/4 aspect sector excludes the forward quarter and is not head-on. Forward-of-beam intensity was not published, so the rear lobe is used and the forward 45° stays dark. Guidance inhibit 0.6 s is the midpoint of 0.5–0.7 s. The optical fuze arms 0.4 s after burnout, the midpoint of 0.1–0.8 s, and responds out to 9 m. Self-destruct 22 s is the midpoint of 21–23 s; a narrative also says 21–28 s. The 7.6 km equal-speed figure is not a drag fit, and the 7.6 km IL-28 acquisition is not the fighter lock range. The 2 g launch veto is a firing interlock, not a guidance law.";
            return resolve(spec);
        }

        Spec r13(const char *id, const char *name, float massKg, float spanM, bool guidanceTimer, const char *card)
        {
            Spec spec;
            spec.id = id;
            spec.displayName = name;
            spec.family = "Russia";
            spec.massKg = massKg;
            spec.lengthM = 2.87f;
            spec.diameterM = 0.127f;
            spec.spanM = spanM;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::RearHemisphere;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            if (guidanceTimer)
            {
                spec.guidanceLimitS = 55.0f;
                spec.selfDestructS = 55.0f;
            }
            inferredArm(spec, 150.0f);
            spec.card = card;
            return resolve(spec);
        }

        Spec r60(bool mike)
        {
            Spec spec;
            spec.id = mike ? "r-60m" : "r-60";
            spec.displayName = mike ? "R-60M" : "R-60";
            spec.family = "Russia";
            spec.massKg = mike ? 44.0f : 43.5f;
            spec.lengthM = mike ? 2.138f : 2.096f;
            spec.diameterM = 0.120f;
            spec.spanM = 0.390f;
            standInMotor(spec);
            sizedToG(spec, 47.0f);
            spec.aspect = mike ? AspectKind::AllAspect : AspectKind::RearHemisphere;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.cueDeg = mike ? 20.0f : 12.0f;
            spec.cuePublished = true;
            spec.gimbalDeg = 35.0f;
            spec.gimbalPublished = false;
            if (mike)
            {
                unpublishedTrack(spec);
            }
            else
            {
                spec.trackRateDegPerS = 35.0f;
                spec.trackRatePublished = true;
            }
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.guidanceLimitS = 24.0f;
            spec.selfDestructS = 25.0f;
            inferredArm(spec, mike ? 150.0f : 250.0f);
            spec.card = mike
                            ? "44 kg, length 2,138 mm, cooled seeker, cue ±20°. All-aspect at the 0.2 forward bound is optimistic at long range: no acquisition kilometres were published, and the limited front hemisphere is a close-in claim. 47 g is the Missilery overload (Weaponsystems prints 42; they are not averaged) and is not restated as an R-60M-only figure. The 35°/s track rate was the R-60 Komar and is not copied. 8 km is a high-altitude launch bracket, not a lock range. Guided flight 23–25 s is the R-60 citation; 24 s and a 25 s end are labeled, and destruct was not separate."
                            : "43.5 kg, warhead 3 kg, length about 2,096 mm, diameter 120 mm, span 390 mm. Komar cue ±12°, track 35°/s. The 30–35° tracking-angle phrase is ambiguous, so 35° is a sim stop and not a measured half-angle. 47 g is Missilery; Weaponsystems 42 g is not averaged. No TVC. Arm 250 m is the low-altitude minimum launch range, not the 8 km high-altitude bracket. Guidance 24 s is the midpoint of the cited 23–25 s, and 25 s ends the flight because a separate destruct time was not published.";
            return resolve(spec);
        }

        Spec archer(const char *id, const char *name, float massKg, float lengthM, float cueDeg, bool cuePublished, IrccmKind irccm, const char *irccmNote, float tailLockM, const char *card)
        {
            Spec spec;
            spec.id = id;
            spec.displayName = name;
            spec.family = "Russia";
            spec.massKg = massKg;
            spec.lengthM = lengthM;
            spec.diameterM = 0.170f;
            spec.spanM = 0.510f;
            standInMotor(spec);
            shapeCoefficient(spec);
            // R-73 and R-73M share the brochure 40 g. RVV-MD does not, and the
            // old index test (id[3]=='3') also matched "r-73m" only by accident.
            const bool publishedArcherG = std::strcmp(id, "r-73") == 0 || std::strcmp(id, "r-73m") == 0;
            spec.structuralG = publishedArcherG ? 40.0f : 0.0f;
            spec.structuralGPublished = publishedArcherG;
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.cueDeg = cueDeg;
            spec.cuePublished = cuePublished;
            spec.gimbalDeg = 75.0f;
            spec.gimbalPublished = true;
            if (std::strcmp(id, "r-73") == 0)
            {
                spec.trackRateDegPerS = 60.0f;
                spec.trackRatePublished = true;
            }
            else
            {
                unpublishedTrack(spec);
            }
            spec.irccm = irccm;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = irccmNote;
            jetTab(spec);
            spec.homing = LaunchHoming::LockAfterLaunch;
            inferredArm(spec, 300.0f);
            if (std::strcmp(id, "r-73") == 0)
            {
                spec.proximityM = 3.5f;
                spec.proximityPublished = true;
            }
            spec.tailAcquisitionM = tailLockM;
            spec.tailAcquisitionPublished = false;
            spec.card = card;
            return resolve(spec);
        }

        Spec r27(bool extended)
        {
            Spec spec;
            spec.id = extended ? "r-27et" : "r-27t";
            spec.displayName = extended ? "R-27ET" : "R-27T";
            spec.family = "Russia";
            spec.massKg = extended ? 343.0f : 245.0f;
            spec.lengthM = extended ? 4.50f : 3.80f;
            spec.diameterM = extended ? 0.260f : 0.230f;
            spec.spanM = extended ? 0.80f : 0.77f;
            standInMotor(spec);
            if (extended)
            {
                shapeCoefficient(spec);
                spec.motorNote = "The ET motor is the larger double-mode grain: two thrust regimes in one burn, not a dual-pulse, and the split was not published. Modeled as one AIM-9B thrust-to-weight burn. It will not reach the advertising range. That is intentional. The 8 g figure is not copied onto the ET.";
            }
            else
            {
                sizedToG(spec, 8.0f);
                spec.motorNote = "Single-mode 230 mm motor. Thrust and burn were not published. The stand-in is a short burn on purpose: front-hemisphere advertising ranges are not tail chases and were not fitted.";
            }
            spec.hasTvc = false;
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.cueDeg = 55.0f;
            spec.cuePublished = !extended;
            spec.gimbalDeg = 55.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.rearHemisphereDesignation = false;
            inferredArm(spec, 500.0f);
            spec.card = extended
                            ? "343 kg, about 4.5 m, diameter 260 mm. Airforce Technology prints 0.23 m for the ET and that conflict is kept; 260 mm is the Missilery engine diameter. Span 0.80 m is the GlobalSecurity wing; 972 mm plumage is the rudder and is not used as the wing. No TVC and no loft. ±55° is the T designation and is not a separate ET measurement, so the cue is tagged. Lock before launch. Inertial midcourse is off: the sources dispute it for the infrared round."
                            : "245 kg, 3.80 m, diameter 230 mm, warhead 39 kg. Operational load factor 8 g with canards and no TVC, so it is not an Archer. Span 0.77 m is the GlobalSecurity wing; Missilery's 972 mm plumage is the rudder. Designation ±55°. A separate gimbal stop was not published, so the stop is that designation angle. Lock before launch, no loft, no Archer g. The 8 g cap is aero and is reached only at high dynamic pressure.";
            return resolve(spec);
        }

        Spec magic(bool second)
        {
            Spec spec;
            spec.id = second ? "magic-2" : "magic-1";
            spec.displayName = second ? "Magic II" : "Magic I";
            spec.family = "France";
            spec.massKg = 89.0f;
            spec.lengthM = 2.75f;
            spec.diameterM = 0.157f;
            spec.spanM = 0.66f;
            standInMotor(spec);
            if (second)
            {
                sizedToG(spec, 50.0f);
                spec.aspect = AspectKind::AllAspect;
                spec.forwardFraction = kAllAspectForwardFraction;
                spec.irccm = IrccmKind::Rise;
                spec.irccmCircuitPublished = false;
                spec.irccmNote = "The published behaviour is an afterburner-ionisation / reheat-spike filter, not an imaging seeker. Rise-rate is the closest class. The 2.5:1 illustration is not Matra firmware.";
                spec.motorNote = "Described as about 10% more powerful than Magic I. That phrase was not turned into newtons. Impulse unpublished; AIM-9B thrust-to-weight stand-in.";
                inferredArm(spec, 150.0f);
            }
            else
            {
                sizedToG(spec, 35.0f);
                spec.aspect = AspectKind::RearCone;
                spec.rearConeHalfAngleDeg = 70.0f;
                spec.irccm = IrccmKind::None;
                spec.armTimeS = 1.8f;
                spec.armTimePublished = true;
                spec.selfDestructS = 26.0f;
                spec.selfDestructPublished = true;
                inferredArm(spec, 300.0f);
            }
            spec.gimbalDeg = 30.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.card = second
                            ? "All-aspect AD3633. 89 kg, 2.75 m, diameter 0.157 m and span 0.66 m are the unsplit French family table, not a Magic II-only weighing. English Magic II range 20 km and the French unsplit 15 km are not averaged. English Mach 2 conflicts with the French Mach 3 table and is not used. 50 g is the English Wikipedia airframe sentence, not a Matra load table. The 1.8 s arm and 26 s self-destruct are not copied: they were not restated for Magic II. Gimbal 30° is the English sentence and is not split by variant. No TVC. No head-on lock is removed; Magic II is all-aspect."
                            : "Rear aspect only: any non-frontal target inside a 140° area, modeled as a 70° half-angle from the tail, not the whole rear hemisphere. No head-on lock. 89 kg / 2.75 m / 0.157 m / 0.66 m is the unsplit French family table. English length 2.72 m and range 10 km conflict with the French 2.75 m and unsplit 15 km and are not averaged. 35 g is the English Wikipedia sentence, not a Matra table. Armed 1.8 s after launch, self-destruct 26 s, minimum employment about 0.3 km used as an arm distance and not a separate fuze drawing. Gimbal 30° is not split by variant. No TVC.";
            return resolve(spec);
        }

        Spec irisT()
        {
            Spec spec;
            spec.id = "iris-t";
            spec.displayName = "IRIS-T";
            spec.family = "Germany";
            spec.massKg = 87.4f;
            spec.lengthM = 2.936f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.447f;
            standInMotor(spec);
            spec.motorNote = "One grain with a published shape: boost, a short low-thrust turn, acceleration to Mach 3, then sustain. Segment magnitudes were not published, so this is not a piecewise Diehl curve and not a dual-pulse. AIM-9B thrust-to-weight stand-in.";
            shapeCoefficient(spec);
            spec.structuralG = 60.0f;
            spec.structuralGPublished = true;
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 90.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Scanning 128×2 two-colour array building a 128×128 image. The 1° separation gate was not published. 60°/s on the Wikipedia agility line is the missile turn rate, not a seeker rate.";
            jetVane(spec);
            spec.homing = LaunchHoming::LockAfterLaunch;
            spec.rearHemisphereDesignation = true;
            inferredArm(spec, 150.0f);
            spec.card = "Saab length 2,936 mm. English infobox 2.94 m is not averaged in. Mass 87.4 kg is the English infobox; German text says about 90 kg and Saab prints no mass. Span 447 mm. Look angle ±90°. Lock before launch and lock after launch, including a target behind the fighter via inertial midcourse. 60 g is Wikipedia, not a number on the Saab page. The structural cap is 60 g while the vanes are burning; the aero coefficient stays the AIM-9B shape coefficient, so the airframe alone does not pull 60 g. Vane angle 10° is Fleeman's sizing value, not a Diehl measurement. Saab's approximate 25 km is not a motor fit.";
            return resolve(spec);
        }

        Spec asraam()
        {
            Spec spec;
            spec.id = "asraam";
            spec.displayName = "ASRAAM";
            spec.family = "United Kingdom";
            spec.massKg = 88.0f;
            spec.lengthM = 2.90f;
            spec.diameterM = 0.166f;
            spec.spanM = 0.45f;
            standInMotor(spec);
            spec.motorNote = "Published as dual-burn boost/sustain. The split was not published, so the flyout is one burn at the AIM-9B thrust-to-weight, not an invented sustain level. Block 6 has no published kinematic change and is not a second missile.";
            sizedToG(spec, 50.0f);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 90.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "128×128 imaging array on Blocks 1–5. Block 6 raises the pixel count by an unpublished amount. The 1° separation gate was not published.";
            spec.hasTvc = false;
            spec.homing = LaunchHoming::LockAfterLaunch;
            spec.rearHemisphereDesignation = true;
            spec.reducedSmoke = true;
            inferredArm(spec, 300.0f);
            spec.card = "MBDA 88 kg, 2.9 m, 166 mm. Designation-systems 87 kg is the conflict. Span about 0.45 m. No thrust-vector control: Germany left the programme to get that, and it became IRIS-T. 50 g soon after launch is an aero figure, reached when dynamic pressure is high, not a jet-vane number. ±90° half-angle, lock after launch, including over the shoulder. Range is not fitted: designation-systems says about 15 km and probably higher; Wikipedia says 25+ km. Low-smoke is the designation-systems line; the live MBDA page does not repeat it. Minimum about 300 m is an employment figure used as the arm distance.";
            return resolve(spec);
        }

        Spec micaIr()
        {
            Spec spec;
            spec.id = "mica-ir";
            spec.displayName = "MICA IR";
            spec.family = "France";
            spec.massKg = 112.0f;
            spec.lengthM = 3.10f;
            spec.diameterM = 0.160f;
            spec.spanM = 0.480f;
            standInMotor(spec);
            spec.motorNote = "One high-impulse low-smoke solid with thrust vectoring. Dual-pulse is MICA NG, not the fielded IR round. Impulse unpublished; AIM-9B thrust-to-weight stand-in. It will not be stretched to 60 km.";
            sizedToG(spec, 50.0f);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 60.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Dual-waveband imaging. The 1° separation gate was not published, and no pixel count was on the MBDA datasheet.";
            jetVane(spec);
            spec.homing = LaunchHoming::LockAfterLaunch;
            spec.rearHemisphereDesignation = true;
            spec.reducedSmoke = true;
            spec.armDistanceM = 500.0f;
            spec.armDistancePublished = true;
            spec.card = "MBDA 112 kg, 3.1 m, 160 mm. Long-chord wings, tail, and thrust vectoring. Rail launch in this sim; ejection exists but is the same inherited velocity. Family minimum 500 m. Structural 50 g. French Wikipedia also says 30 g at several tens of kilometres, which is what low dynamic pressure already does, so there is no second cap. Span 0.480 m is an unfootnoted French infobox. Gimbal was not published: 360° is the launch envelope via lock-after-launch, and 60° is an unpublished terminal basket. Midcourse is inertial toward the predicted intercept, then proportional navigation only after the seeker locks. No loft. The French IR intercept of about 60 km is not the EM 80 km figure and is not a motor fit. Mach 4 is unsplit or EM and is not imposed.";
            return resolve(spec);
        }

        Spec shafrir2()
        {
            Spec spec;
            spec.id = "shafrir-2";
            spec.displayName = "Shafrir 2";
            spec.family = "Israel";
            spec.massKg = 94.0f;
            spec.lengthM = 2.60f;
            spec.diameterM = 0.160f;
            spec.spanM = 0.55f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::RearHemisphere;
            spec.gimbalDeg = 10.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            inferredArm(spec, 600.0f);
            spec.card = "Mass is modeled at 94 kg inside the 93–95 kg band, not averaged as a measurement. Length 2.6 m, diameter 0.16 m, span 0.55 m. Gimbal 10° is the WeaponSystems off-boresight acquisition. Kopp's 45° is a target aspect, not the gimbal. Rear aspect. 'Over 2 g' is not a load factor and is not used. Minimum 0.6 km is an employment minimum used as the arm distance, not a fuze drawing. Maximum 6.4 km and practical 3 km are not a motor fit.";
            return resolve(spec);
        }

        Spec python3()
        {
            Spec spec;
            spec.id = "python-3";
            spec.displayName = "Python 3";
            spec.family = "Israel";
            spec.massKg = 120.0f;
            spec.lengthM = 2.95f;
            spec.diameterM = 0.160f;
            spec.spanM = 0.86f;
            standInMotor(spec);
            sizedToG(spec, 40.0f);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.cueDeg = 30.0f;
            spec.cuePublished = true;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = true;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            inferredArm(spec, 500.0f);
            spec.card = "WeaponSystems 120 kg and 2.95 m. FAS prints 3.00 m; that conflict is not averaged. Diameter 0.16 m, span 0.86 m, warhead 11 kg. Acquisition about 30° off boresight, in-flight tracking 40°. All-aspect, up to 40 g, no TVC. Arm 500 m is the published minimum firing range, tagged as an arm distance rather than a fuze clock. High-altitude 15 km and low-altitude 5 km are not a motor fit. Mach 3.5 is the brochure speed claim and is not imposed on the stand-in motor.";
            return resolve(spec);
        }

        Spec python4()
        {
            Spec spec;
            spec.id = "python-4";
            spec.displayName = "Python 4";
            spec.family = "Israel";
            spec.massKg = 105.0f;
            spec.lengthM = 3.10f;
            spec.diameterM = 0.160f;
            spec.spanM = 0.64f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 60.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Kinematic;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = "Multiple-detector array. Not two-colour and not an imaging focal plane. No spectral ratio was invented.";
            spec.homing = LaunchHoming::LockBeforeLaunch;
            inferredArm(spec, 150.0f);
            spec.card = "Mass, length, diameter, and span were not published as Python 4 measurements. 105 kg, 3.10 m, 0.16 m, and 0.64 m are the Python 5 brochure, stored only because a secondary page claims the same airframe. Rafael has not said that. The FAS Python 4 table is the Python 3 table and is not used. Gimbal 'in excess of 60°' is modeled at 60° as a lower bound, not as 90° or 180°. Lock before launch with a wide helmet cue, not a rear-hemisphere lock-after-launch. No TVC and no published g, so the shape coefficient is used. All-aspect, fourth generation.";
            return resolve(spec);
        }

        Spec python5()
        {
            Spec spec;
            spec.id = "python-5";
            spec.displayName = "Python 5";
            spec.family = "Israel";
            spec.massKg = 105.0f;
            spec.lengthM = 3.10f;
            spec.diameterM = 0.160f;
            spec.spanM = 0.64f;
            standInMotor(spec);
            spec.motorNote = "A two-stage motor is published without a thrust split, so the flyout is one burn. AIM-9B thrust-to-weight stand-in. 'Over 20 km' is trade press and was not fitted. The Wikipedia SPYDER 40 km figure is not used.";
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 90.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Imaging infrared plus CCD. The 1° separation gate was not published.";
            spec.hasTvc = false;
            spec.homing = LaunchHoming::LockAfterLaunch;
            spec.rearHemisphereDesignation = true;
            inferredArm(spec, 150.0f);
            spec.card = "Rafael brochure: 105 kg, 3.10 m, diameter 0.16 m, span 0.64 m. No thrust-vector control in the opened manufacturer text, so none is modeled. Full-sphere launch, including over the shoulder, is lock-after-launch. The seeker stop of 90° is not a Rafael half-angle: the brochure says extremely high off-boresight, and trade-press 'more than 100°' is not used. Structural g was not published. The shape coefficient is not a Rafael CN.";
            return resolve(spec);
        }

        Spec pl5eii()
        {
            Spec spec;
            spec.id = "pl-5eii";
            spec.displayName = "PL-5EII";
            spec.family = "China";
            spec.massKg = 83.0f;
            spec.lengthM = 2.893f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.617f;
            standInMotor(spec);
            sizedToG(spec, 35.0f);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Kinematic;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = "Dual-colour, not an imaging focal plane. No band ratio was invented.";
            spec.homing = LaunchHoming::LockBeforeLaunch;
            spec.tailAcquisitionM = 16000.0f;
            inferredArm(spec, 150.0f);
            spec.card = "AVIC length 2,893 mm; LOEC 2009 prints 2,896 mm and they are not averaged. Diameter 127 mm, span 617 mm, 35 g. Mass was not on the AVIC card. 83 kg is a CASI figure and conflicts with other secondary masses. All-aspect, lock before launch. 16 km is a detection range, used here as the tail lock gate and not as a flight range. The PL-5E angles, 40 g, 500 m, and 14 km are not copied onto the EII. Gimbal 40° is an unpublished sim stop. Mach 3 is the card speed and is not imposed on the stand-in motor.";
            return resolve(spec);
        }

        Spec pl8()
        {
            Spec spec;
            spec.id = "pl-8";
            spec.displayName = "PL-8";
            spec.family = "China";
            spec.massKg = 115.0f;
            spec.lengthM = 2.90f;
            spec.diameterM = 0.160f;
            spec.diameterIsAssumption = true;
            standInMotor(spec);
            sizedToG(spec, 38.0f);
            spec.aspect = AspectKind::RearHemisphere;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.homing = LaunchHoming::LockBeforeLaunch;
            inferredArm(spec, 150.0f);
            spec.card = "CASI: 115 kg, 2.9 m, range 15 km. The 15 km figure is not a motor fit. CASI omitted the diameter; 160 mm is not a PL-8 measurement and is only the body area used for drag. 'Over 38 g' is modeled as 38 g. Python 3's 120 kg, 40 g, and 30°/40° seeker are not copied. Aspect was not on the CASI row, so this stays rear-aspect rather than silently inheriting Python 3. A later PL-8B helmet claim has no angle. Gimbal 40° is an unpublished sim stop. No TVC and no lock-after-launch.";
            return resolve(spec);
        }

        Spec pl9c()
        {
            Spec spec;
            spec.id = "pl-9c";
            spec.displayName = "PL-9C";
            spec.family = "China";
            spec.massKg = 115.0f;
            spec.lengthM = 2.992f;
            spec.diameterM = 0.157f;
            spec.spanM = 0.856f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.cueDeg = 40.0f;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Kinematic;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = "Described as anti-decoy. The circuit is unnamed, so this is the kinematic class and not a published spectral ratio.";
            spec.homing = LaunchHoming::LockBeforeLaunch;
            inferredArm(spec, 150.0f);
            spec.card = "Manufacturer pair 115 kg, diameter 157 mm, span 856 mm, range 20 km. The range is not a motor fit. Length 2.992 m is AVIC; LOEC 2009 prints 2.900 m and they are not averaged. The earlier CASI PL-9 row (123 kg / 15 km) is not this round. No speed, g, or seeker angle was published, so the shape coefficient is used and 40° is an unpublished cue. Helmet or radar slave is described without a degree. Lock before launch. Not described as lock-after-launch. No TVC.";
            return resolve(spec);
        }

        Spec pl10()
        {
            Spec spec;
            spec.id = "pl-10";
            spec.displayName = "PL-10";
            spec.family = "China";
            spec.massKg = 105.0f;
            spec.lengthM = 3.00f;
            spec.diameterM = 0.160f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.structuralG = 60.0f;
            spec.structuralGPublished = true;
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 90.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Imaging seeker. The 1° separation gate was not published.";
            jetVane(spec);
            spec.homing = LaunchHoming::LockAfterLaunch;
            inferredArm(spec, 150.0f);
            spec.card = "Imaging, helmet cue, lock after launch, thrust vectoring. CASI/Sina: 105 kg, 160 mm, length 3.0 m, 20 km. The 20 km figure is not a motor fit. The 89 kg photo estimate and a 33 kg warhead aggregator are not used. ±90° is a secondary 'reportedly' and is not a manufacturer half-angle. 60 g is CASI/media, not a manufacturer limit: it is the structural cap while the vanes burn, and the aero coefficient stays the AIM-9B shape coefficient. Vane angle 10° is Fleeman's sizing value. Not a full-sphere designation; the rear hemisphere was not stated.";
            return resolve(spec);
        }

        Spec aam3()
        {
            Spec spec;
            spec.id = "aam-3";
            spec.displayName = "AAM-3";
            spec.family = "Japan";
            spec.massKg = 91.0f;
            spec.lengthM = 3.10f;
            spec.diameterM = 0.127f;
            spec.spanM = 0.64f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Kinematic;
            spec.irccmCircuitPublished = false;
            spec.irccmNote = "IR/UV two-colour. Not imaging. No band ratio was invented.";
            spec.homing = LaunchHoming::LockBeforeLaunch;
            inferredArm(spec, 150.0f);
            spec.card = "91 kg, 3.1 m, diameter 0.127 m, and span 0.64 m are infobox figures, not a MoD table. 13 km, Mach 2.5, and a 15 kg warhead are not treated as MoD numbers. Forecast International's 7 km / Mach 3.5 / 2.60 m is a conflicting estimate and is not averaged or fitted. No TVC is described. Electric canards and bank-to-turn are noted; no gains were published, so the sim uses the same aero proportional navigation as the other canard rounds. High off-boresight with no degree: 40° is an unpublished sim stop. Lock before launch.";
            return resolve(spec);
        }

        Spec aam5()
        {
            Spec spec;
            spec.id = "aam-5";
            spec.displayName = "AAM-5";
            spec.family = "Japan";
            spec.massKg = 95.0f;
            spec.lengthM = 3.105f;
            spec.diameterM = 0.130f;
            spec.spanM = 0.412f;
            standInMotor(spec);
            shapeCoefficient(spec);
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 60.0f;
            spec.gimbalPublished = false;
            unpublishedTrack(spec);
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Imaging focal plane with image flare rejection. The 1° separation gate was not published.";
            jetVane(spec);
            spec.homing = LaunchHoming::LockAfterLaunch;
            inferredArm(spec, 150.0f);
            spec.card = "Thrust vectoring plus tail, no canards. Imaging, inertial midcourse, lock after launch, laser proximity, helmet on the F-15J. The Japanese text says the original AAM-5 has a 3-axis gimbal and AAM-5B went from 3-axis to 2-axis; English Wikipedia reverses that and is not followed. No degree was in the MoD text. 60° is an unpublished sim stop, not a full-sphere rear hemisphere. 95 kg, 3.105 m, diameter 0.13 m, spans 0.31 / 0.412 m, 35 km, and Mach 3 are Wikipedia or handbook figures, not MoD, and 35 km is not a motor fit. AAM-5B is not a separate kinematic row. Forecast International's pre-disclosure 100 kg / Mach 4 / 7–9 km is not used. Vane angle 10° is Fleeman's sizing value.";
            return resolve(spec);
        }

        Spec aDarter()
        {
            Spec spec;
            spec.id = "a-darter";
            spec.displayName = "A-Darter";
            spec.family = "South Africa";
            spec.massKg = 93.0f;
            spec.lengthM = 2.98f;
            spec.diameterM = 0.166f;
            spec.spanM = 0.488f;
            standInMotor(spec);
            spec.motorNote = "Smokeless propellant is published. Thrust and burn were not. AIM-9B thrust-to-weight stand-in. Jane's 23 km is not a motor fit; DefenceWeb's about 20 km is the conflict.";
            spec.aeroGIsShapeCoefficient = false;
            spec.structuralG = 100.0f;
            spec.structuralGPublished = true;
            spec.aeroStructuralG = 50.0f;
            spec.coastStructuralG = 50.0f;
            spec.aspect = AspectKind::AllAspect;
            spec.forwardFraction = kAllAspectForwardFraction;
            spec.gimbalDeg = 90.0f;
            spec.gimbalPublished = true;
            spec.trackRateDegPerS = 120.0f;
            spec.trackRatePublished = true;
            spec.irccm = IrccmKind::Imaging;
            spec.irccmCircuitPublished = true;
            spec.irccmNote = "Imaging two-colour seeker. The 1° separation gate was not published.";
            jetVane(spec);
            spec.homing = LaunchHoming::LockAfterLaunch;
            spec.reducedSmoke = true;
            inferredArm(spec, 150.0f);
            spec.card = "Denel and Jane's: 93 kg, 2.98 m, 166 mm. Tail span 488 mm is Jane's and is not in the three-line Denel block. 180° look is read as ±90° from the nose. Track 120°/s is published. Jane's 100 g via thrust vectoring during the burn and 50 g after: the aero coefficient is sized to 50 g, and the burn cap is 100 g. Wikipedia's 89 kg, 10 km, 8 s, and 50 g coast are not used. Airforce Technology's 90 kg, 0.16 m, and 10 km are not used. Lock after launch. The rear hemisphere beyond the 90° look was not stated, so designation stays inside that look angle. Vane angle 10° is Fleeman's sizing value, not a Denel measurement.";
            return resolve(spec);
        }

        const std::vector<Spec> &allRounds()
        {
            static const std::vector<Spec> rounds = {
                aim9b(),
                aim9d(),
                aim9h(),
                aim9j(),
                limaFamily("aim-9l", "AIM-9L", 86.0f, 2.85f, IrccmKind::Kinematic,
                           "Hard-wired flare rejection is described. The circuit was not published, so this is the kinematic class, not a Rise-rate AIM-9M.",
                           false,
                           "Parsch: 86 kg, 2.85 m, span 0.63 m, diameter 0.127 m. All-aspect InSb. Gimbal, IFOV, and track rate were classified or unpublished; 40° is a sim stop and 180°/s only lets the gimbal stop the seeker. Do not read a video-game 30 g or 35 g onto the L. The shape coefficient is not a measured Lima CN. Lock before launch. Arming reuses the AIM-9D 1,000 ft window and is tagged inferred. The motor is a Mk 36 without a published thrust-time pair."),
                limaFamily("aim-9m", "AIM-9M", 86.0f, 2.85f, IrccmKind::Rise,
                           "Reduced-smoke AIM-9M is associated with expanded IRCCM. That association is not a primary circuit citation. 2.5:1 in 40 ms is an open illustration, not AIM-9M firmware. A steady source twice as bright can still seduce it.",
                           true,
                           "Same Parsch geometry row as the L: 86 kg, 2.85 m, span 0.63 m. The USAF fact sheet's 190 lb and 2.87 m conflict and are not averaged. Reduced-smoke motor. The seeker changes from the L are the IRCCM class and the smoke, not a new published impulse. NAVAIR's statement that the AIM-9X uses this motor does not publish the newtons. Lock before launch. Arming is the inferred AIM-9D window."),
                aim9p5(),
                aim9x("aim-9x-blk1", "AIM-9X Block I", LaunchHoming::LockBeforeLaunch, false,
                      "Block I is lock-before-launch only. NAVAIR geometry used here: 9.9 ft, 186 lb, 5 in diameter, wing span 17.6 in. Parsch's 11 in span is not used. 90° off-boresight is the look angle; a 180° field of view is a derived reading, not a written half-angle. Track rate was not published. Structural g was not published. The AIM-9B shape coefficient on this heavier round is the aero cap, which understates public descriptions of 9X agility and does not invent a g. Thrust vectoring can reach that cap at low dynamic pressure. Vane angle 10° is Fleeman's jet-vane sizing value, not a measured AIM-9X angle. Same drag scale as the other slender rounds; no guessed Cd reduction."),
                aim9x("aim-9x-blk2", "AIM-9X Block II", LaunchHoming::LockAfterLaunch, true,
                      "Same flyout as Block I. Block II adds a datalink and lock-after-launch, including a target behind the fighter. The datalink in this sim is ideal: it flies the predicted intercept until the imaging seeker locks, and that waveform was not published. Structural g, vane angle, and motor impulse are the same unpublished quantities as Block I. AIM-9X Block III was cancelled and is not modeled."),
                aim9x("aim-9x-blk2plus", "AIM-9X Block II+", LaunchHoming::LockAfterLaunch, true,
                      "A fielded 2026 variant. No kinematic difference from Block II was published, so the flyout matches Block II, including lock-after-launch and the rear hemisphere. The card is the difference: there is no published thrust, mass, or g delta to apply."),
                limaFamily("aim-9l-i", "AIM-9L/I", 84.0f, 2.87f, IrccmKind::Kinematic,
                           "Hard-wired flare rejection. The circuit was not published.",
                           false,
                           "German guidance-section variant of the Lima, not a new motor. Bundeswehr card: 2.87 m, diameter 12.7 cm, span 63 cm, 84 kg. Unpublished US AIM-9L thrust is not copied, and the US L's 86 kg is not copied either. All-aspect. Lock before launch. The kinematic IRCCM is a stand-in for the hard-wired module."),
                limaFamily("aim-9l-i-1", "AIM-9L/I-1", 84.0f, 2.87f, IrccmKind::Rise,
                           "Described as the operational equivalent of the AIM-9M's expanded IRCCM. The circuit was not published. Rise-rate here is the open illustration, not Diehl firmware.",
                           false,
                           "Same German airframe card as the L/I: 84 kg, 2.87 m, span 0.63 m. The further guidance-section upgrade is the IRCCM class. No published motor change. Not an imaging seeker."),
                r3s(),
                r13("r-13m", "R-13M", 90.0f, 0.632f, true,
                    "Eduard: about 90 kg, length about 2.87 m, diameter 0.127 m, span 632 mm. Weaponsystems 87.7 kg is the conflict and is not averaged. Rear aspect. Motor unknown; a 3–5 s burn was not opened and is not invented. Guided flight about 55 s is control duration, not a burn, and both the guidance limit and the end of flight use it. 'Up to 7 g' on the secondary page is not labeled as the missile load factor and is not used. Gimbal 40° is an unpublished sim stop. The shape coefficient is not a measured R-13 CN."),
                r13("r-13m1", "R-13M1", 90.6f, 0.651f, false,
                    "Weaponsystems is the only opened page: 90.6 kg, span 651 mm, range 0.3–17 km. The printed length '2.876 mm' is a unit error and is not used as 2.876 m. Length 2.87 m is the R-13M Eduard figure and is not an M1 measurement. 'Up to 8 g' is not labeled as the missile load factor. The 55 s guided-flight time was not restated, so it is not copied. Rear aspect, unpublished motor, shape coefficient, gimbal a 40° sim stop. The range band is not a motor fit."),
                r60(false),
                r60(true),
                archer("r-73", "R-73", 105.0f, 2.90f, 45.0f, true, IrccmKind::None, "", 0.0f,
                       "R-73E brochure: 105 kg, 2.9 × 0.17 × 0.51 m. AusAirpower 103 kg conflicts. Cue ±45° (brochure; Wikipedia ±40° is not used), coordinator ±75°, line-of-sight rate 60°/s. Pre-launch designation uses the cue; in flight the seeker may use the coordinator. Front-hemisphere launch 30 km, also a 20 km table; neither is fitted as a tail chase. Rear minimum 0.3 km is a minimum firing range, used as the arm distance and not a fuze clock. 40 g is the published overload with gas-dynamic control while the motor burns. The aero coefficient is the AIM-9B shape coefficient, labeled not a measured Archer CN. Jet-tab deflection was not published; 15° is Fleeman's uncertain jet-tab band, not a Vympel angle. The unlabeled 7,700 N is not used as the motor. Baseline IRCCM is none. Lock after launch inside the cue, not a rear-hemisphere shot. Kill radius about 3.5 m is Missilery."),
                archer("r-73m", "R-73M", 110.0f, 2.90f, 45.0f, true, IrccmKind::Kinematic,
                       "Digital dual-band seeker. That is not automatically an imaging focal plane. No band ratio was invented.",
                       0.0f,
                       "Missilery RMD-2 mass 110 kg. The R-73E brochure mass is 105 kg and is the conflict. Dimensions are the R-73 row. The designation '+90°' is not clearly a ±90° half-angle, so the gimbal stays 75° and the cue stays 45°. 40 g and the jet tab are the R-73 page figures, not a separate M measurement. Track rate 60°/s was not restated. R-74M2 is not modeled."),
                archer("rvv-md", "RVV-MD", 106.0f, 2.92f, 60.0f, true, IrccmKind::Kinematic,
                       "Dual-band infrared. Kinematic class, not an imaging focal plane, and the R-73's 40 g is not copied.",
                       10000.0f / std::sqrt(0.2f),
                       "KTRV: 106 kg, 2.92 × 0.17 × 0.51 m, cue ±60°, coordinator ±75°, front hemisphere up to 40 km, rear minimum 0.3 km. The 40 km figure is not a tail chase and is not fitted. Lock range 10 km against a fighter in the front hemisphere, engines at maximum, aspect q = 15°. Dividing by sqrt(0.2) gives about 22 km of tail lock. Because 0.2 is an upper bound on forward intensity, that tail figure is a lower bound and is labeled derived. Gas-dynamic control while the motor burns; 15° is the uncertain jet-tab stand-in, not a KTRV angle. Structural g was not published, so the shape coefficient is the aero cap and thrust vectoring can reach it only while the motor burns. 40 g is not copied from the R-73."),
                r27(false),
                r27(true),
                magic(false),
                magic(true),
                irisT(),
                asraam(),
                micaIr(),
                shafrir2(),
                python3(),
                python4(),
                python5(),
                pl5eii(),
                pl8(),
                pl9c(),
                pl10(),
                aam3(),
                aam5(),
                aDarter(),
            };
            return rounds;
        }
    }

    Spec resolve(Spec spec)
    {
        const bool missingMotor = spec.motor == MotorConfidence::UnpublishedStandIn || spec.burnS <= 0.0f || spec.thrustN <= 0.0f;
        if (missingMotor)
        {
            spec.motor = MotorConfidence::UnpublishedStandIn;
            spec.resolvedThrustN = kAim9bThrustN * (spec.massKg / kAim9bMassKg);
            spec.resolvedBurnS = kAim9bBurnS;
            spec.propellantKg = 0.0f;
            if (spec.motorNote == nullptr || spec.motorNote[0] == '\0')
            {
                spec.motorNote = kStandInMotorNote;
            }
        }
        else
        {
            spec.resolvedThrustN = spec.thrustN;
            spec.resolvedBurnS = spec.burnS;
        }

        if (spec.cnOverride > 0.0f)
        {
            spec.resolvedCnMax = spec.cnOverride;
        }
        else if (spec.aeroGIsShapeCoefficient || spec.structuralG <= 0.0f)
        {
            spec.resolvedCnMax = kAim9bCnMax;
            spec.aeroGIsShapeCoefficient = true;
        }
        else
        {
            const float sizingG = spec.aeroStructuralG > 0.0f ? spec.aeroStructuralG : spec.structuralG;
            spec.resolvedCnMax = cnMaxForStructuralG(sizingG, spec.massKg, spec.diameterM);
        }

        // Structural caps. What the round can actually pull at a given moment
        // is the lesser of this and the fin (dynamic-pressure) and thrust-
        // vectoring authority, worked out in flight.
        if (spec.structuralGPublished)
        {
            spec.resolvedBurnG = spec.structuralG;
            spec.resolvedCoastG = spec.coastStructuralG > 0.0f ? spec.coastStructuralG : spec.structuralG;
        }
        else
        {
            spec.resolvedBurnG = kUnpublishedStructuralG;
            spec.resolvedCoastG = kUnpublishedStructuralG;
        }

        if (spec.gimbalDeg <= 0.0f)
        {
            spec.gimbalDeg = 40.0f;
            spec.gimbalPublished = false;
        }
        if (!spec.trackRatePublished && spec.trackRateDegPerS <= 0.0f)
        {
            spec.trackRateDegPerS = kUnpublishedTrackRateDegPerS;
        }
        return spec;
    }

    const Spec *catalog()
    {
        return allRounds().data();
    }

    int catalogCount()
    {
        return static_cast<int>(allRounds().size());
    }

    const Spec *find(const char *id)
    {
        if (id == nullptr || id[0] == '\0')
        {
            return nullptr;
        }
        const std::vector<Spec> &rounds = allRounds();
        for (const Spec &spec : rounds)
        {
            if (std::strcmp(spec.id, id) == 0)
            {
                return &spec;
            }
        }
        return nullptr;
    }

    const char *motorConfidenceLabel(MotorConfidence confidence)
    {
        switch (confidence)
        {
        case MotorConfidence::PublishedAverage:
            return "Published average";
        case MotorConfidence::DerivedAverage:
            return "Derived average";
        case MotorConfidence::UnpublishedStandIn:
            return "Unpublished stand-in";
        }
        return "Unpublished stand-in";
    }

    const char *aspectLabel(AspectKind aspect)
    {
        switch (aspect)
        {
        case AspectKind::RearHemisphere:
            return "Rear hemisphere";
        case AspectKind::RearCone:
            return "Rear cone";
        case AspectKind::RearThroughForwardBeam:
            return "Rear through the beam";
        case AspectKind::AllAspect:
            return "All-aspect";
        case AspectKind::AfterburnerNoseGate:
            return "Rear, plus afterburner nose gate";
        }
        return "Rear hemisphere";
    }

    const char *irccmLabel(IrccmKind kind)
    {
        switch (kind)
        {
        case IrccmKind::None:
            return "None";
        case IrccmKind::Rise:
            return "Rise-rate";
        case IrccmKind::Kinematic:
            return "Kinematic";
        case IrccmKind::Imaging:
            return "Imaging";
        }
        return "None";
    }
}
