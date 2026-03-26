#pragma once

#include "Flare.h"
#include "PhysicsObject.h"
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
    float lifetime = 4.0f;
    float heatSignature = 5.5f;
    float heatDecayRate = 1.3f;
    float mass = 0.9f;
    float dragCoefficient = 1.1f;
    float crossSectionalArea = 0.018f;
};

struct EvasiveManeuverConfig
{
    float lateralAcceleration = 32.0f;
    float verticalBias = 0.25f;
    float speedMultiplier = 1.12f;
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

    float getHeatSignature() const { return m_heatSignature; }
    TargetAIState getAIState() const { return m_aiState; }
    float getCommandedSpeed() const { return m_commandedSpeed; }
    float getReferenceDistance() const { return m_referenceDistance; }

    bool isMissileWarningActive() const { return m_missileWarningActive; }
    int getRemainingFlares() const { return m_remainingFlares; }
    float getThreatTimeToClosestApproach() const { return m_threatTimeToClosestApproach; }
    float getThreatClosestApproachDistance() const { return m_threatClosestApproachDistance; }

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
