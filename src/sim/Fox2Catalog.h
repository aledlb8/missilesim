#pragma once

// Sourced Fox 2 rounds. Unpublished motors are not given a solved thrust curve.
// resolve() fills the flyout numbers the missile actually integrates.
// Custom is not a row: it is the absence of a catalog id.

#include "sim/Fox2Flight.h"

namespace missilesim::fox2
{
    enum class MotorConfidence
    {
        PublishedAverage,
        DerivedAverage,
        UnpublishedStandIn
    };

    enum class IrccmKind
    {
        None,
        Rise,
        Kinematic,
        Imaging
    };

    enum class LaunchHoming
    {
        LockBeforeLaunch,
        LockAfterLaunch
    };

    struct Spec
    {
        const char *id = "";
        const char *displayName = "";
        const char *family = "";

        float massKg = 0.0f;
        float lengthM = 0.0f;
        float diameterM = 0.0f;
        bool diameterIsAssumption = false;
        float spanM = 0.0f;

        float thrustN = 0.0f;
        float burnS = 0.0f;
        float propellantKg = 0.0f;
        MotorConfidence motor = MotorConfidence::UnpublishedStandIn;
        const char *motorNote = "";

        float structuralG = 0.0f;
        bool structuralGPublished = false;
        // Aero CN is sized to this g when it differs from the burn cap (A-Darter coasts at 50 g).
        float aeroStructuralG = 0.0f;
        float coastStructuralG = 0.0f;
        bool aeroGIsShapeCoefficient = false;
        float cnOverride = 0.0f;

        AspectKind aspect = AspectKind::RearHemisphere;
        float forwardFraction = kAllAspectForwardFraction;
        float rearConeHalfAngleDeg = 70.0f;
        float noseGateHalfAngleDeg = 20.0f;
        float noseGateFraction = kAim9dNoseIntensityFractionUnpublished;
        bool afterburnerScalesTail = false;

        float ifovDeg = 0.0f;
        bool ifovPublished = false;
        float gimbalDeg = 0.0f;
        bool gimbalPublished = false;
        float cueDeg = 0.0f;
        bool cuePublished = false;
        float trackRateDegPerS = kUnpublishedTrackRateDegPerS;
        bool trackRatePublished = false;

        IrccmKind irccm = IrccmKind::None;
        bool irccmCircuitPublished = false;
        const char *irccmNote = "";

        bool hasTvc = false;
        float vaneDeg = 0.0f;
        bool vanePublished = false;

        LaunchHoming homing = LaunchHoming::LockBeforeLaunch;
        bool rearHemisphereDesignation = false;

        float inhibitS = 0.0f;
        bool inhibitPublished = false;
        float guidanceLimitS = 0.0f;
        bool guidanceLimitPublished = false;
        float selfDestructS = 0.0f;
        bool selfDestructPublished = false;
        float armDistanceM = 0.0f;
        bool armDistancePublished = false;
        float armAfterBurnoutS = 0.0f;
        float armTimeS = 0.0f;
        bool armTimePublished = false;
        float proximityM = 0.0f;
        bool proximityPublished = false;

        // Tail-aspect acquisition range. 0 means no numeric gate (I > 0 locks at any range).
        float tailAcquisitionM = 0.0f;
        bool tailAcquisitionPublished = false;

        // Conditioned tail-chase range used by the checker. 0 means the brochure
        // figure is not a tail chase and must not be fitted.
        float publishedMaxRangeM = 0.0f;
        const char *rangeNote = "";
        bool reducedSmoke = false;
        const char *card = "";

        float resolvedThrustN = 0.0f;
        float resolvedBurnS = 0.0f;
        float resolvedCnMax = 0.0f;
        float resolvedBurnG = 0.0f;
        float resolvedCoastG = 0.0f;
    };

    Spec resolve(Spec spec);

    const Spec *catalog();
    int catalogCount();
    const Spec *find(const char *id);

    const char *motorConfidenceLabel(MotorConfidence confidence);
    const char *aspectLabel(AspectKind aspect);
    const char *irccmLabel(IrccmKind kind);
}
