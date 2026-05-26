#pragma once

#include "Flare.h"
#include "PhysicsObject.h"
#include "physics/Aerodynamics.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <limits>
#include <vector>

class Missile;

struct MAWSConfig
{
    bool enabled = true;
    float detectionRange = 3200.0f;
    float reactionTimeWindow = 6.0f;
    float closestApproachThreshold = 140.0f;
};

struct FlareDispenserConfig
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

// Behavioural intent for the target's autopilot. These are AI/tactics weights
// (how it wants to maneuver), not aerodynamic fudge factors - the energy state
// (speed loss in turns and climbs) now emerges from the real lift/drag/thrust
// model in Target.cpp, so the former scalar "energy penalty" knobs are gone.
struct EvasiveManeuverConfig
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

struct TargetAIConfig
{
    float minSpeed = 180.0f;
    float maxSpeed = 320.0f;
    float preferredDistance = 1500.0f;
};

enum class TargetAIState
{
    PATROL,
    REPOSITION,
    DEFENSIVE,
    RECOVERING
};

class Target : public PhysicsObject
{
public:
    Target(const glm::vec3 &position = glm::vec3(0.0f), float radius = 5.0f);
    ~Target() override = default;

    std::string getType() const override { return "Target"; }

    float getRadius() const { return m_radius; }
    void setRadius(float radius) { m_radius = radius; }

    bool isActive() const { return m_isActive; }
    void setActive(bool active);

    void setAIConfig(const TargetAIConfig &config);
    const TargetAIConfig &getAIConfig() const { return m_aiConfig; }
    void setMAWSConfig(const MAWSConfig &config);
    const MAWSConfig &getMAWSConfig() const { return m_mawsConfig; }
    void setFlareDispenserConfig(const FlareDispenserConfig &config);
    const FlareDispenserConfig &getFlareDispenserConfig() const { return m_flareConfig; }
    void setEvasiveManeuverConfig(const EvasiveManeuverConfig &config);
    const EvasiveManeuverConfig &getEvasiveManeuverConfig() const { return m_evasiveConfig; }

    // Force-based flight model inputs. The target flies through real lift,
    // drag, thrust and gravity using the same aerodynamic model as the missile.
    const missilesim::physics::AeroProfile *getAeroProfile() const override { return &m_aeroProfile; }
    void setAeroProfile(const missilesim::physics::AeroProfile &profile) { m_aeroProfile = profile; }
    float getCommandedLiftCoefficient() const override { return m_commandedLiftCoefficient; }
    void setEngineThrust(float newtons) { m_maxThrust = (newtons >= 0.0f) ? newtons : 0.0f; }
    float getEngineThrust() const { return m_maxThrust; }
    // Local atmospheric conditions, supplied by the physics engine each step.
    void setAmbientConditions(float density, float speedOfSound)
    {
        m_ambientDensity = (density >= 0.0f) ? density : 0.0f;
        m_speedOfSound = (speedOfSound > 0.0f) ? speedOfSound : m_speedOfSound;
    }

    // Smoothed acceleration for steady visual banking (avoids roll jitter from
    // the patrol controller's small per-step heading corrections).
    glm::vec3 getRenderAcceleration() const override { return m_smoothedAcceleration; }

    float getHeatSignature() const { return m_heatSignature; }
    TargetAIState getAIState() const { return m_aiState; }
    float getCommandedSpeed() const { return m_commandedSpeed; }
    float getReferenceDistance() const { return m_referenceDistance; }

    bool isMissileWarningActive() const { return m_missileWarningActive; }
    int getRemainingFlares() const { return m_remainingFlares; }
    float getThreatTimeToClosestApproach() const { return m_threatTimeToClosestApproach; }
    float getThreatClosestApproachDistance() const { return m_threatClosestApproachDistance; }
    bool hasThreatAssessment() const { return m_threatAssessment.active; }
    const glm::vec3 &getThreatMissilePosition() const { return m_threatAssessment.missilePosition; }
    const glm::vec3 &getThreatMissileVelocity() const { return m_threatAssessment.missileVelocity; }
    float getThreatDistance() const { return m_threatAssessment.distance; }
    float getThreatClosingSpeed() const { return m_threatAssessment.closingSpeed; }

    void updateThreatAssessment(const std::vector<Missile *> &missiles);
    std::vector<FlareLaunchRequest> consumePendingFlareLaunches();

    void update(float deltaTime) override;
    bool isPointInside(const glm::vec3 &point) const;

private:
    struct ThreatAssessment
    {
        bool active = false;
        glm::vec3 missilePosition = glm::vec3(0.0f);
        glm::vec3 missileVelocity = glm::vec3(0.0f);
        float distance = std::numeric_limits<float>::infinity();
        float timeToClosestApproach = std::numeric_limits<float>::infinity();
        float closestApproachDistance = std::numeric_limits<float>::infinity();
        float closingSpeed = 0.0f;
    };

    void updateAutonomousFlight(float deltaTime);
    float computeDesiredSpeed(float referenceDistance) const;
    float computeDesiredAltitude(float referenceDistance, float currentSpeed) const;
    void enforceAirspaceConstraint();
    void updateCountermeasures(float deltaTime, const glm::vec3 &currentVelocity);
    void resetCountermeasureState();

    float m_radius;
    bool m_isActive = true;
    float m_heatSignature = 1.0f;

    // Force-based flight model state.
    missilesim::physics::AeroProfile m_aeroProfile;
    float m_commandedLiftCoefficient = 0.0f;
    float m_maxThrust = 60000.0f;       // engine thrust ceiling (N)
    float m_ambientDensity = 1.225f;    // local air density (kg/m^3)
    float m_speedOfSound = 340.294f;    // local speed of sound (m/s)
    glm::vec3 m_smoothedAcceleration{0.0f}; // low-pass acceleration for rendering

    TargetAIConfig m_aiConfig;
    MAWSConfig m_mawsConfig;
    FlareDispenserConfig m_flareConfig;
    EvasiveManeuverConfig m_evasiveConfig;

    glm::vec3 m_homeAnchor;
    glm::vec3 m_referencePosition = glm::vec3(0.0f);
    glm::vec3 m_referenceVelocity = glm::vec3(0.0f);
    float m_referenceDistance = std::numeric_limits<float>::infinity();
    float m_nominalAltitude = 180.0f;
    float m_altitudeExcursion = 60.0f;
    float m_patrolPhase = 0.0f;
    int m_orbitDirection = 1;
    float m_commandedSpeed = 0.0f;
    TargetAIState m_aiState = TargetAIState::PATROL;

    int m_remainingFlares = 24;
    std::vector<FlareLaunchRequest> m_pendingFlareLaunches;

    ThreatAssessment m_threatAssessment;
    bool m_missileWarningActive = false;
    float m_threatTimeToClosestApproach = std::numeric_limits<float>::infinity();
    float m_threatClosestApproachDistance = std::numeric_limits<float>::infinity();
    float m_countermeasureCooldown = 0.0f;
    float m_burstShotTimer = 0.0f;
    int m_pendingBurstShots = 0;
    int m_flareSpreadSign = 1;
};

inline void Target::setMAWSConfig(const MAWSConfig &config)
{
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };

    m_mawsConfig.enabled = config.enabled;
    m_mawsConfig.detectionRange = std::max(finiteOr(config.detectionRange, m_mawsConfig.detectionRange), 0.0f);
    m_mawsConfig.reactionTimeWindow = std::max(finiteOr(config.reactionTimeWindow, m_mawsConfig.reactionTimeWindow), 0.1f);
    m_mawsConfig.closestApproachThreshold = std::max(finiteOr(config.closestApproachThreshold, m_mawsConfig.closestApproachThreshold), 0.0f);
}

inline void Target::setFlareDispenserConfig(const FlareDispenserConfig &config)
{
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };

    const int previousInventory = std::max(m_flareConfig.inventory, 0);
    const int spentFlares = std::clamp(previousInventory - m_remainingFlares, 0, previousInventory);

    m_flareConfig.enabled = config.enabled;
    m_flareConfig.inventory = std::max(config.inventory, 0);
    m_flareConfig.burstSize = std::max(config.burstSize, 1);
    m_flareConfig.burstInterval = std::max(finiteOr(config.burstInterval, m_flareConfig.burstInterval), 0.0f);
    m_flareConfig.cooldown = std::max(finiteOr(config.cooldown, m_flareConfig.cooldown), 0.0f);
    m_flareConfig.ejectSpeed = std::max(finiteOr(config.ejectSpeed, m_flareConfig.ejectSpeed), 0.0f);
    m_flareConfig.aftLaunchOffset = std::max(finiteOr(config.aftLaunchOffset, m_flareConfig.aftLaunchOffset), 0.0f);
    m_flareConfig.lateralLaunchOffset = std::max(finiteOr(config.lateralLaunchOffset, m_flareConfig.lateralLaunchOffset), 0.0f);
    m_flareConfig.lateralEjectFraction = std::max(finiteOr(config.lateralEjectFraction, m_flareConfig.lateralEjectFraction), 0.0f);
    m_flareConfig.downwardEjectFraction = std::max(finiteOr(config.downwardEjectFraction, m_flareConfig.downwardEjectFraction), 0.0f);
    m_flareConfig.lifetime = std::max(finiteOr(config.lifetime, m_flareConfig.lifetime), 0.0f);
    m_flareConfig.heatSignature = std::max(finiteOr(config.heatSignature, m_flareConfig.heatSignature), 0.0f);
    m_flareConfig.heatDecayRate = std::max(finiteOr(config.heatDecayRate, m_flareConfig.heatDecayRate), 0.0f);
    m_flareConfig.mass = std::max(finiteOr(config.mass, m_flareConfig.mass), 0.01f);
    m_flareConfig.dragCoefficient = std::max(finiteOr(config.dragCoefficient, m_flareConfig.dragCoefficient), 0.0f);
    m_flareConfig.crossSectionalArea = std::max(finiteOr(config.crossSectionalArea, m_flareConfig.crossSectionalArea), 0.0001f);

    m_remainingFlares = std::clamp(m_flareConfig.inventory - spentFlares, 0, m_flareConfig.inventory);
    if (!m_flareConfig.enabled || m_remainingFlares == 0)
    {
        m_pendingFlareLaunches.clear();
        m_pendingBurstShots = 0;
        m_burstShotTimer = 0.0f;
        m_countermeasureCooldown = 0.0f;
    }
}

inline void Target::setEvasiveManeuverConfig(const EvasiveManeuverConfig &config)
{
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };
    const auto nonNegative = [&](float value, float fallback) {
        return std::max(finiteOr(value, fallback), 0.0f);
    };
    const auto signedFinite = [&](float value, float fallback) {
        return finiteOr(value, fallback);
    };
    const auto unitInterval = [&](float value, float fallback) {
        return glm::clamp(finiteOr(value, fallback), 0.0f, 1.0f);
    };

    m_evasiveConfig.cruisePitchRateDegrees = nonNegative(config.cruisePitchRateDegrees, m_evasiveConfig.cruisePitchRateDegrees);
    m_evasiveConfig.defensivePitchRateDegrees = nonNegative(config.defensivePitchRateDegrees, m_evasiveConfig.defensivePitchRateDegrees);
    m_evasiveConfig.defensiveBreakWeight = nonNegative(config.defensiveBreakWeight, m_evasiveConfig.defensiveBreakWeight);
    m_evasiveConfig.defensiveAwayWeight = nonNegative(config.defensiveAwayWeight, m_evasiveConfig.defensiveAwayWeight);
    m_evasiveConfig.defensiveThreatAwayWeight = nonNegative(config.defensiveThreatAwayWeight, m_evasiveConfig.defensiveThreatAwayWeight);
    m_evasiveConfig.defensiveIncomingWeight = nonNegative(config.defensiveIncomingWeight, m_evasiveConfig.defensiveIncomingWeight);
    m_evasiveConfig.altitudeCorrectionRange = std::max(finiteOr(config.altitudeCorrectionRange, m_evasiveConfig.altitudeCorrectionRange), 1.0f);
    m_evasiveConfig.maxAltitudePitchBias = nonNegative(config.maxAltitudePitchBias, m_evasiveConfig.maxAltitudePitchBias);
    m_evasiveConfig.innerDistanceAltitudeOffset = signedFinite(config.innerDistanceAltitudeOffset, m_evasiveConfig.innerDistanceAltitudeOffset);
    m_evasiveConfig.outerDistanceAltitudeOffset = signedFinite(config.outerDistanceAltitudeOffset, m_evasiveConfig.outerDistanceAltitudeOffset);
    m_evasiveConfig.defensiveLowEnergyAltitudeOffset = signedFinite(config.defensiveLowEnergyAltitudeOffset, m_evasiveConfig.defensiveLowEnergyAltitudeOffset);
    m_evasiveConfig.defensiveThreatBelowAltitudeOffset = signedFinite(config.defensiveThreatBelowAltitudeOffset, m_evasiveConfig.defensiveThreatBelowAltitudeOffset);
    m_evasiveConfig.defensiveThreatAboveAltitudeOffset = signedFinite(config.defensiveThreatAboveAltitudeOffset, m_evasiveConfig.defensiveThreatAboveAltitudeOffset);
    m_evasiveConfig.defensiveSpeedBlend = unitInterval(config.defensiveSpeedBlend, m_evasiveConfig.defensiveSpeedBlend);
    m_evasiveConfig.defensiveSpeedUrgencyBlend = unitInterval(config.defensiveSpeedUrgencyBlend, m_evasiveConfig.defensiveSpeedUrgencyBlend);
    m_evasiveConfig.nearDistanceSpeedupThreshold = unitInterval(config.nearDistanceSpeedupThreshold, m_evasiveConfig.nearDistanceSpeedupThreshold);
    m_evasiveConfig.farDistanceSlowdownThreshold = unitInterval(config.farDistanceSlowdownThreshold, m_evasiveConfig.farDistanceSlowdownThreshold);
    m_evasiveConfig.farDistanceSpeedFloor = unitInterval(config.farDistanceSpeedFloor, m_evasiveConfig.farDistanceSpeedFloor);
    m_evasiveConfig.recoveryEnergyThreshold = unitInterval(config.recoveryEnergyThreshold, m_evasiveConfig.recoveryEnergyThreshold);
    m_evasiveConfig.repositionDistanceThreshold = unitInterval(config.repositionDistanceThreshold, m_evasiveConfig.repositionDistanceThreshold);
}
