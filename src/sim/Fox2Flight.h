#pragma once

// Point-mass helpers shared by the Fox 2 catalog and the kinematics check.
// Formulas follow research_notes/Famous Fox 2 missiles/flight-model.md.
// Every constant that is not a published manual figure is named as such.

#include <algorithm>
#include <cmath>

namespace missilesim::fox2
{
    constexpr float kGravity = 9.80665f;
    constexpr float kPi = 3.14159265358979323846f;

    // Fleeman rocket-baseline nose fineness (lN/d = 2.4). A Sidewinder nose
    // fineness was not published. AIM-9D is described as ogival, which is why
    // this baseline is used for every slender round here.
    constexpr float kAssumedNoseFineness = 2.4f;

    // Nozzle-exit to body-area ratio. Unpublished. It only scales powered base
    // drag and the altitude thrust term, by a few hundredths of Cd0 inside the
    // band Fleeman plots.
    constexpr float kAssumedExitAreaRatio = 0.4f;

    // Sea-level design exit pressure for the back-pressure term already used
    // by Missile::applyThrust. Not a measured Sidewinder nozzle pressure.
    constexpr float kSeaLevelPressure = 101325.0f;

    // AIM-9B, NAVWEPS OP 2309. Launch mass 160 lb. Impulse 8440 lbf·s in 2.2 s
    // at 70 °F, spread evenly because the manual does not print a thrust curve.
    constexpr float kAim9bMassKg = 160.0f * 0.45359237f;
    constexpr float kAim9bImpulseLbfS = 8440.0f;
    constexpr float kAim9bBurnS = 2.2f;
    constexpr float kAim9bThrustN = (kAim9bImpulseLbfS / kAim9bBurnS) * 4.448221615f;
    constexpr float kAim9bDiameterM = 5.0f * 0.0254f;
    constexpr float kAim9bLengthM = 111.5f * 0.0254f;

    // Body-referenced CN from the manual's 4.2 g at 50,000 ft and Mach 2.3,
    // using the 160 lb launch mass. flight-model.md derives 5.49. The AIM-9D
    // and later rounds do not inherit this as their own measured coefficient.
    constexpr float kAim9bCnMax = 5.49f;
    constexpr float kAim9bStructuralG = 10.0f;

    // In-band scale on Fleeman's unscaled body Cd0. tools/fox2_kinematics
    // set this from 0.60 / (peak unscaled coast Cd), which was 0.666548 at
    // 15,240 m and Mach 1.2. Scaled coast Cd is about 0.45 at Mach 1.5 sea
    // level and at Mach 2 / 15,240 m, inside the 0.2–0.6 slender-body band.
    // A straight Cl = 0 AIM-9B tail chase at this k is still long of OP 2309:
    // 8,578 m versus 1,829 m at sea level (×4.69) and 13,122 m versus 8,534 m
    // at 15,240 m (×1.54). Raising k until the handbook range matches pushes
    // Cd0 out of the band, so the residual is kept and Cd0 is not retuned.
    constexpr float kAim9bDragScale = 0.900161f;

    // AIM-9D, OP 3353: 3,500 lbf average for 5 s is the flyout. 2,645 lbf is a
    // conflicting figure and is not used (it sits under the manual's minimum impulse).
    constexpr float kAim9dThrustN = 3500.0f * 4.448221615f;
    constexpr float kAim9dBurnS = 5.0f;
    constexpr float kAim9dMassKg = 195.0f * 0.45359237f;

    // R-3S motor page: impulse floor 38,100 N·s. Burn 1.7 s hot and 3.2 s cold.
    // 2.45 s is the midpoint of that band, so 38,100 / 2.45 is a derived average,
    // not a printed thrust. Peak 25,000 N is not the average.
    constexpr float kR3sImpulseNs = 38100.0f;
    constexpr float kR3sBurnS = 2.45f;
    constexpr float kR3sThrustN = kR3sImpulseNs / kR3sBurnS;

    // All-aspect forward/stern intensity upper bound. Derived from a flare
    // ratio, not a radiometric fit. Do not raise it.
    constexpr float kAllAspectForwardFraction = 0.2f;

    // Rise-time illustration from the open IRCCM article. Not an AIM-9M spec.
    constexpr float kRiseRatioIllustration = 2.5f;
    constexpr float kRiseWindowIllustrationS = 0.040f;

    // Unhardened reticle: a source about twice as bright takes the track.
    constexpr float kUnhardenedSeductionRatio = 2.0f;

    // Fleeman jet-vane sizing value. Not a measured AIM-9X vane angle.
    constexpr float kJetVaneDegrees = 10.0f;

    // Fleeman lists a jet tab among devices of about ±15° or ±20°. Pairing
    // 15° with the R-73's gas-dynamic vanes is uncertain; it is not a Vympel angle.
    constexpr float kJetTabDegreesUncertain = 15.0f;

    // AIM-9D head-on gate. The manual did not publish the intensity inside the
    // afterburner nose cone. This is below the 0.2 all-aspect forward bound and
    // is not 0.2 times tail intensity. Afterburner doubles AIM-9D detection
    // range, so the rear lobe is scaled by four while this gate stays put.
    constexpr float kAim9dNoseIntensityFractionUnpublished = 0.05f;

    // Stand-in used only when a brochure does not print a track rate, so the
    // gimbal — not this rate — is what stops the seeker. Cards say so.
    constexpr float kUnpublishedTrackRateDegPerS = 180.0f;

    // Imaging seekers reject a second source once it is angularly separate.
    // The angle was not published.
    constexpr float kImagingSeparationDeg = 1.0f;

    // Kinematic IRCCM: a decelerating flare is rejected once it is this far
    // off the target, or outside the IFOV if that cone is wider.
    constexpr float kKinematicSeparationDeg = 2.0f;
    constexpr float kKinematicFlareSpeedFraction = 0.85f;

    // Airframe load limit for rounds whose structural g was never published.
    // A sim choice, not a manufacturer figure. The published limits in this
    // catalog for dogfight rounds run 35-60 g (PL-5EII 35, PL-8 38, Python 3
    // 40, R-60 47, Magic 2 50, IRIS-T 60); this sits below all of them. The
    // dynamic-pressure limit still decides what the fins can actually pull.
    // (Previously the cap was the g the AIM-9B coefficient reaches at
    // kAim9bStructuralDynamicPressure, about 8.5 g for an AIM-9X; that is a
    // property of the 1950s coefficient, not of any later airframe.)
    constexpr float kUnpublishedStructuralG = 30.0f;

    // Body axis eases toward velocity. Not a published airframe rate.
    constexpr float kBodyAlignRateRadPerS = 1.5f;

    // The AI target has no afterburner flag. Throttle above this stands in
    // for reheat, and only for seekers that publish an afterburner dependence.
    constexpr float kAiAfterburnerThrottle = 0.85f;

    // Fin aspect ratio and Oswald factor were not published. Used only for
    // induced drag. Do not pair CN 5.49 with an AR of 2; that Cd_i is not a
    // Sidewinder measurement.
    constexpr float kAssumedFinAspectRatio = 4.0f;
    constexpr float kAssumedOswaldEfficiency = 0.8f;

    inline float bodyArea(float diameterM)
    {
        const float radius = 0.5f * diameterM;
        return kPi * radius * radius;
    }

    // Fleeman body build-up, referenced to body cross-section.
    // qPascal is dynamic pressure. powered selects the reduced base term.
    // The returned value is UNSCALED. Multiply by the AIM-9B calibration k.
    inline float fleemanBodyCd0(float mach, float dynamicPressurePa, float lengthM, float diameterM, bool powered)
    {
        mach = std::max(mach, 0.05f);
        const float fineness = std::max(lengthM / std::max(diameterM, 1.0e-4f), 1.0f);
        const float lengthFeet = lengthM * 3.280839895013123f;
        const float qPsf = std::max(dynamicPressurePa, 1.0f) / 47.880258888889f;
        const float friction = 0.053f * fineness * std::pow(mach / std::max(qPsf * lengthFeet, 1.0e-4f), 0.2f);

        const auto baseCoefficient = [](float m, bool motorOn) {
            const float raw = (m > 1.0f) ? (0.25f / m) : (0.12f + 0.13f * m * m);
            return motorOn ? ((1.0f - kAssumedExitAreaRatio) * raw) : raw;
        };
        const auto waveCoefficient = [](float m) {
            if (m <= 1.0f)
            {
                return 0.0f;
            }
            const float noseAngle = std::atan(0.5f / kAssumedNoseFineness);
            return (1.59f + 1.83f / (m * m)) * std::pow(noseAngle, 1.69f);
        };

        if (mach <= 1.0f)
        {
            return friction + baseCoefficient(mach, powered);
        }

        const float supersonic = friction + baseCoefficient(mach, powered) + waveCoefficient(mach);
        if (mach >= 1.2f)
        {
            return supersonic;
        }

        // Hoerner: zero-lift drag of slender body-fin shapes peaks near
        // Mach 1.00–1.02, then falls toward the supersonic sum. The peak
        // height is not a Sidewinder measurement; 1.25 keeps the hump above
        // both endpoints without leaving Fleeman's slender-body band once
        // the AIM-9B scale is applied.
        const float atSonic = friction + baseCoefficient(1.0f, powered);
        const float atBlend = friction + baseCoefficient(1.2f, powered) + waveCoefficient(1.2f);
        const float peak = 1.25f * std::max(atSonic, atBlend);
        constexpr float kPeakMach = 1.02f;
        if (mach <= kPeakMach)
        {
            const float t = (mach - 1.0f) / (kPeakMach - 1.0f);
            return atSonic + t * (peak - atSonic);
        }
        const float t = (mach - kPeakMach) / (1.2f - kPeakMach);
        return peak + t * (atBlend - peak) + (supersonic - atBlend) * t;
    }

    inline float scaledCd0(float mach, float dynamicPressurePa, float lengthM, float diameterM, bool powered, float scale)
    {
        return scale * fleemanBodyCd0(mach, dynamicPressurePa, lengthM, diameterM, powered);
    }

    // psi = 0 looking at the nose, pi looking up the tail.
    // rear = 1 on the tail, 0 on and forward of the beam.
    inline float rearLobe(float cosPsi)
    {
        return std::max(0.0f, -cosPsi);
    }

    inline float aspectIntensity(float cosPsi, float forwardFraction)
    {
        const float forward = std::clamp(forwardFraction, 0.0f, kAllAspectForwardFraction);
        const float rear = rearLobe(cosPsi);
        return forward + (1.0f - forward) * rear;
    }

    enum class AspectKind
    {
        RearHemisphere,
        RearCone,
        RearThroughForwardBeam,
        AllAspect,
        AfterburnerNoseGate
    };

    // cosPsi = dot(target nose, direction from the target to the missile).
    // 1 is head-on, -1 is up the tail.
    inline float seekerIntensity(AspectKind kind, float cosPsi, float forwardFraction, float rearConeHalfAngleDeg,
                                 float noseGateHalfAngleDeg, float noseGateFraction, bool afterburnerScalesTail,
                                 bool targetAfterburner)
    {
        const float rear = rearLobe(cosPsi);
        switch (kind)
        {
        case AspectKind::RearHemisphere:
        case AspectKind::RearThroughForwardBeam:
            // Forward-of-beam intensity was not published for the R-3S 1/4–3/4
            // sector. The rear lobe is already zero forward of the beam, which
            // also excludes the forward 45°. No forward intensity is invented.
            return rear;
        case AspectKind::RearCone:
        {
            const float half = std::clamp(rearConeHalfAngleDeg, 1.0f, 179.0f) * (kPi / 180.0f);
            if (cosPsi > -std::cos(half))
            {
                return 0.0f;
            }
            return rear;
        }
        case AspectKind::AllAspect:
            return aspectIntensity(cosPsi, forwardFraction);
        case AspectKind::AfterburnerNoseGate:
        {
            float tail = rear;
            if (afterburnerScalesTail && targetAfterburner)
            {
                tail *= 4.0f;
            }
            const float noseHalf = std::clamp(noseGateHalfAngleDeg, 0.0f, 89.0f) * (kPi / 180.0f);
            if (targetAfterburner && cosPsi >= std::cos(noseHalf))
            {
                return noseGateFraction;
            }
            return tail;
        }
        }
        return 0.0f;
    }

    struct TailChaseRequest
    {
        float altitudeM = 0.0f;
        float launchMach = 1.2f;
        float targetMach = 0.9f;
        float massKg = kAim9bMassKg;
        float propellantKg = 0.0f;
        float diameterM = kAim9bDiameterM;
        float lengthM = kAim9bLengthM;
        float seaLevelThrustN = kAim9bThrustN;
        float burnTimeS = kAim9bBurnS;
        float dragScale = 1.0f;
        float maxTimeS = 24.0f;
        float dt = 0.01f;
    };

    struct TailChaseResult
    {
        float rangeM = 0.0f;
        float flightTimeS = 0.0f;
        float burnoutSpeedMps = 0.0f;
        float maxMach = 0.0f;
        bool closed = false;
    };

    // Horizontal co-altitude tail chase, Cl = 0. Range is the distance closed
    // before the missile is no longer faster than the target, which is the
    // acceptance integral in flight-model.md. Atmosphere samples must be the
    // sim's Atmosphere::sample so the calibration and the game agree.
    template <typename SampleFn>
    inline TailChaseResult integrateTailChase(const TailChaseRequest &request, SampleFn sample)
    {
        TailChaseResult result;
        const auto air = sample(request.altitudeM);
        const float targetSpeed = request.targetMach * air.speedOfSound;
        float speed = request.launchMach * air.speedOfSound;
        float mass = std::max(request.massKg, 0.1f);
        const float area = bodyArea(request.diameterM);
        const float exitArea = kAssumedExitAreaRatio * area;
        const float propellant = std::max(request.propellantKg, 0.0f);
        const float massFlow = (propellant > 0.0f && request.burnTimeS > 0.0f)
                                   ? (propellant / request.burnTimeS)
                                   : 0.0f;
        const float dryMass = std::max(mass - propellant, 0.1f);

        float time = 0.0f;
        while (time < request.maxTimeS && speed > targetSpeed)
        {
            const bool powered = time < request.burnTimeS && request.seaLevelThrustN > 0.0f;
            const float mach = speed / std::max(air.speedOfSound, 1.0f);
            const float dynamicPressure = 0.5f * air.density * speed * speed;
            const float cd0 = scaledCd0(mach, dynamicPressure, request.lengthM, request.diameterM, powered, request.dragScale);
            const float drag = dynamicPressure * cd0 * area;
            const float backPressure = (kSeaLevelPressure - air.pressure) * exitArea;
            const float thrust = powered ? std::max(request.seaLevelThrustN + backPressure, 0.0f) : 0.0f;
            const float acceleration = (thrust - drag) / mass;

            const float step = std::min(request.dt, request.maxTimeS - time);
            const float closing = speed - targetSpeed;
            result.rangeM += closing * step;
            speed = std::max(0.0f, speed + acceleration * step);
            if (massFlow > 0.0f && powered)
            {
                mass = std::max(dryMass, mass - massFlow * step);
            }
            time += step;
            result.maxMach = std::max(result.maxMach, mach);
            if (powered)
            {
                result.burnoutSpeedMps = speed;
            }
        }
        result.flightTimeS = time;
        result.closed = speed <= targetSpeed;
        return result;
    }

    // Dynamic pressure at which the AIM-9B aero model reaches its 10 g cap.
    // flight-model.md: about 1.02e5 Pa, from 10/4.2 times the 50,000 ft point.
    constexpr float kAim9bStructuralDynamicPressure = 1.02e5f;

    // CN that lets a missile reach structuralG at that same dynamic pressure.
    // Used only when a structural g was actually published. Mass is launch mass.
    inline float cnMaxForStructuralG(float structuralG, float massKg, float diameterM)
    {
        const float area = bodyArea(diameterM);
        if (structuralG <= 0.0f || area <= 0.0f || massKg <= 0.0f)
        {
            return 0.0f;
        }
        return structuralG * massKg * kGravity / (kAim9bStructuralDynamicPressure * area);
    }
}
