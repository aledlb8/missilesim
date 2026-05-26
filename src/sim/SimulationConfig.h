#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    struct EnvironmentConfig
    {
        float fixedTimeStep = 0.01f;
        float simulationSpeed = 1.0f;
        float gravity = 9.81f;
        float seaLevelAirDensity = 1.225f;
        bool groundCollisionEnabled = true;
        float groundRestitution = 0.5f;
    };

    struct VisualizationConfig
    {
        bool showTrajectory = true;
        bool showTargetInfo = true;
        bool showPredictedTargetPath = true;
        bool showInterceptPoint = true;
        int trajectoryPoints = 140;
        float trajectoryTime = 12.0f;
    };

    struct CameraConfig
    {
        float fov = 50.0f;
        float speed = 35.0f;
    };

    struct MissileAirframeConfig
    {
        glm::vec3 initialPosition{0.0f, 0.0f, 0.0f};
        glm::vec3 initialVelocity{0.0f, 0.0f, 50.0f};
        float dryMass = 100.0f;
        float dragCoefficient = 0.1f; // zero-lift drag coefficient Cd0 at low Mach
        float crossSectionalArea = 0.1f;
        float liftCoefficient = 0.1f;
        // Lift-induced drag inputs. aspectRatio <= 0 disables induced drag.
        // Coefficients are referenced to the body cross-section, so these are
        // missile (high normal-force) values, not aircraft-wing values.
        float aspectRatio = 18.0f;
        float oswaldEfficiency = 0.8f;
        // Maximum usable normal-force coefficient (control/stall ceiling).
        // Bounds the aerodynamic g the airframe can pull at a given q.
        float maxLiftCoefficient = 20.0f;
        // Structural load-factor limit in g (0 = unlimited).
        float maxLoadFactorG = 40.0f;
        // Compressibility (transonic drag-rise) curve as (mach, Cd0/Cd0_lowmach)
        // samples sorted by mach. Empty => the built-in supersonic curve is used.
        std::vector<glm::vec2> machDragMultiplier;
    };

    struct MissileMotorConfig
    {
        float thrust = 10000.0f; // sea-level full-burn thrust (N); Ve = thrust / mdot
        float fuelMass = 100.0f;
        float fuelConsumptionRate = 0.5f;     // propellant mass flow rate mdot (kg/s)
        float nozzleExitArea = 0.01f;         // nozzle exit area A_e (m^2) for the back-pressure term
        float nozzleExitPressure = 101325.0f; // design exit pressure P_e (Pa)
    };

    struct MissileGuidanceConfig
    {
        bool enabled = true;
        float navigationGain = 4.0f;
        float maxSteeringForce = 20000.0f;
        float trackingAngle = 85.0f;
        float proximityFuseRadius = 18.0f;
        float countermeasureResistance = 0.65f;
        bool terrainAvoidanceEnabled = true;
        float terrainClearance = 90.0f;
        float terrainLookAheadTime = 6.0f;
        float seekerCueRadiusPixels = 44.0f;
    };

    struct MissileConfig
    {
        MissileAirframeConfig airframe;
        MissileMotorConfig motor;
        MissileGuidanceConfig guidance;
    };

    struct TargetSpawnConfig
    {
        float distanceScaleMin = 0.82f;
        float distanceScaleMax = 1.18f;
        float radiusMin = 3.0f;
        float radiusMax = 7.0f;
        float fallbackRadius = 5.0f;
        float fallbackSpacing = 100.0f;
        glm::vec3 fallbackPosition{100.0f, 100.0f, 100.0f};
        float minimumAltitudeFloor = 80.0f;
        float minimumAltitudeDistanceFraction = 0.08f;
        float minimumAltitudeCeiling = 500.0f;
        float maximumAltitudeDistanceFraction = 0.28f;
        float maximumAltitudeCeiling = 3200.0f;
        float minimumAltitudeBand = 60.0f;
        float warmupTimeStep = 0.016f;
    };

    struct TargetMawsConfig
    {
        bool enabled = true;
        float detectionRange = 3200.0f;
        float reactionTimeWindow = 6.0f;
        float closestApproachThreshold = 140.0f;
    };

    struct TargetFlareConfig
    {
        bool enabled = true;
        int inventory = 24;
        int burstSize = 2;
        float burstInterval = 0.12f;
        float cooldown = 0.9f;
        float ejectSpeed = 45.0f;
        float aftLaunchOffset = 1.2f;
        float lateralLaunchOffset = 0.8f;
        float lateralEjectFraction = 0.18f;
        float downwardEjectFraction = 0.12f;
        float lifetime = 4.0f;
        float heatSignature = 5.5f;
        float heatDecayRate = 1.3f;
        float mass = 0.9f;
        float dragCoefficient = 1.1f;
        float crossSectionalArea = 0.018f;
    };

    struct TargetEvasiveConfig
    {
        float cruisePitchRateDegrees = 10.0f;
        float defensivePitchRateDegrees = 18.0f;
        float defensiveBreakWeight = 1.45f;
        float defensiveAwayWeight = 0.85f;
        float defensiveThreatAwayWeight = 1.0f;
        float defensiveIncomingWeight = 0.25f;
        float altitudeCorrectionRange = 260.0f;
        float maxAltitudePitchBias = 0.42f;
        float innerDistanceAltitudeOffset = 35.0f;
        float outerDistanceAltitudeOffset = -25.0f;
        float defensiveLowEnergyAltitudeOffset = -55.0f;
        float defensiveThreatBelowAltitudeOffset = 40.0f;
        float defensiveThreatAboveAltitudeOffset = -30.0f;
        float defensiveSpeedBlend = 0.48f;
        float defensiveSpeedUrgencyBlend = 0.24f;
        float nearDistanceSpeedupThreshold = 0.15f;
        float farDistanceSlowdownThreshold = 0.25f;
        float farDistanceSpeedFloor = 0.18f;
        float recoveryEnergyThreshold = 0.15f;
        float repositionDistanceThreshold = 0.35f;
    };

    struct TargetGroupConfig
    {
        int count = 1;
        int maxCount = 20;
        float minSpeed = 180.0f;
        float maxSpeed = 320.0f;
        float preferredDistance = 1500.0f;
        TargetSpawnConfig spawn;
        TargetMawsConfig maws;
        TargetFlareConfig flares;
        TargetEvasiveConfig evasive;
    };

    struct SimulationConfig
    {
        int schemaVersion = 1;
        std::string name = "Baseline sandbox";
        EnvironmentConfig environment;
        VisualizationConfig visualization;
        CameraConfig camera;
        MissileConfig missile;
        TargetGroupConfig targets;
    };

    struct SimulationConfigLoadResult
    {
        SimulationConfig config;
        std::filesystem::path path;
        bool loaded = false;
        std::string error;
    };

    std::filesystem::path resolveDefaultSimulationConfigPath();
    SimulationConfigLoadResult loadSimulationConfig(const std::filesystem::path &path);
    SimulationConfigLoadResult loadDefaultSimulationConfig();
}
