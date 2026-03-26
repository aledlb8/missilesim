#pragma once

#include "Flare.h"
#include "PhysicsObject.h"
#include <glm/glm.hpp>
#include <limits>
#include <vector>

class Missile;

enum class TargetMovementPattern
{
    STATIONARY,
    LINEAR,
    CIRCULAR,
    SINUSOIDAL,
    RANDOM
};

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
    bool enabled = true;
    float lateralAcceleration = 32.0f;
    float verticalBias = 0.25f;
    float maxOffset = 260.0f;
    float weavePeriod = 1.4f;
    float recoveryRate = 0.85f;
    float speedMultiplier = 1.12f;
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

    void setHeatSignature(float signature) { m_heatSignature = (signature >= 0.0f) ? signature : 0.0f; }
    float getHeatSignature() const { return m_heatSignature; }

    void setMAWSConfig(const MAWSConfig &config);
    const MAWSConfig &getMAWSConfig() const { return m_mawsConfig; }

    void setFlareDispenserConfig(const FlareDispenserConfig &config);
    const FlareDispenserConfig &getFlareDispenserConfig() const { return m_flareConfig; }

    void setEvasiveManeuverConfig(const EvasiveManeuverConfig &config);
    const EvasiveManeuverConfig &getEvasiveManeuverConfig() const { return m_evasiveConfig; }

    void setMovementPattern(TargetMovementPattern pattern) { m_movementPattern = pattern; }
    TargetMovementPattern getMovementPattern() const { return m_movementPattern; }

    void setMovementSpeed(float speed) { m_movementSpeed = speed; }
    float getMovementSpeed() const { return m_movementSpeed; }

    void setMovementAmplitude(float amplitude) { m_movementAmplitude = amplitude; }
    float getMovementAmplitude() const { return m_movementAmplitude; }

    void setMovementDirection(const glm::vec3 &direction)
    {
        if (glm::dot(direction, direction) > 0.0001f)
        {
            m_movementDirection = glm::normalize(direction);
        }
    }
    const glm::vec3 &getMovementDirection() const { return m_movementDirection; }

    void setMovementCenter(const glm::vec3 &center) { m_movementCenter = center; }
    const glm::vec3 &getMovementCenter() const { return m_movementCenter; }

    void setMovementPeriod(float seconds) { m_movementPeriod = seconds; }
    float getMovementPeriod() const { return m_movementPeriod; }

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
        float timeToClosestApproach = std::numeric_limits<float>::infinity();
        float closestApproachDistance = std::numeric_limits<float>::infinity();
        float closingSpeed = 0.0f;
    };

    void applyLinearMovement(float deltaTime);
    void applyCircularMovement(float deltaTime);
    void applySinusoidalMovement(float deltaTime);
    void applyRandomMovement(float deltaTime);
    void enforceAirspaceConstraint();
    void applyEvasiveOffset(float deltaTime, const glm::vec3 &baseVelocity);
    void updateCountermeasures(float deltaTime, const glm::vec3 &currentVelocity);
    void resetCountermeasureState();

    float m_radius;
    bool m_isActive = true;
    float m_heatSignature = 1.0f;

    TargetMovementPattern m_movementPattern = TargetMovementPattern::STATIONARY;
    float m_movementSpeed = 10.0f;
    float m_movementAmplitude = 30.0f;
    glm::vec3 m_movementDirection = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 m_movementCenter;
    float m_movementPeriod = 10.0f;

    float m_movementTime = 0.0f;
    glm::vec3 m_initialPosition;
    glm::vec3 m_patternPosition;
    float m_wavePathDistance = 0.0f;

    MAWSConfig m_mawsConfig;
    FlareDispenserConfig m_flareConfig;
    EvasiveManeuverConfig m_evasiveConfig;
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
    float m_evasionPhase = 0.0f;
    glm::vec3 m_evasionOffset = glm::vec3(0.0f);
    glm::vec3 m_evasionVelocity = glm::vec3(0.0f);
};
