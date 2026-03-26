#include "Target.h"
#include "Missile.h"
#include <algorithm>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>
#include <iostream>

namespace
{
    constexpr float kMinimumAltitudeMeters = 40.0f;
    constexpr float kReferenceDistanceFloorMeters = 250.0f;
    constexpr float kMinimumSpeedMetersPerSecond = 40.0f;
    constexpr float kCruisePitchRateDegrees = 10.0f;
    constexpr float kDefensivePitchRateDegrees = 18.0f;
    constexpr float kCruiseMaxTurnRateDegrees = 14.0f;
    constexpr float kDefensiveMaxTurnRateDegrees = 22.0f;
    constexpr float kCruiseParasiticDragFactor = 0.00005f;
    constexpr float kDefensiveParasiticDragFactor = 0.00008f;
    constexpr float kBoundaryRecoverySoftZoneMeters = 180.0f;
    constexpr float kBoundaryRecoveryBiasMetersPerSecond = 26.0f;

    glm::vec3 patrolCenterReference(float altitude)
    {
        return glm::vec3(0.0f, altitude, 0.0f);
    }

    float horizontalDistanceFromCenter(const glm::vec3 &position)
    {
        return glm::length(glm::vec2(position.x, position.z));
    }

    glm::vec2 horizontalComponents(const glm::vec3 &vector)
    {
        return glm::vec2(vector.x, vector.z);
    }

    glm::vec3 normalizeOrFallback(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        if (glm::length2(vector) > 0.0001f)
        {
            return glm::normalize(vector);
        }

        if (glm::length2(fallback) > 0.0001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(1.0f, 0.0f, 0.0f);
    }

    glm::vec3 perpendicularTo(const glm::vec3 &direction)
    {
        const glm::vec3 referenceAxis = (std::abs(direction.y) < 0.9f)
                                            ? glm::vec3(0.0f, 1.0f, 0.0f)
                                            : glm::vec3(1.0f, 0.0f, 0.0f);
        return normalizeOrFallback(glm::cross(direction, referenceAxis), glm::vec3(0.0f, 0.0f, 1.0f));
    }

    glm::vec3 rotateTowardsDirection(const glm::vec3 &currentDirection, const glm::vec3 &targetDirection, float maxRadiansDelta)
    {
        const glm::vec3 current = normalizeOrFallback(currentDirection, glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::vec3 target = normalizeOrFallback(targetDirection, current);

        if (maxRadiansDelta <= 0.0f)
        {
            return current;
        }

        const float cosTheta = glm::clamp(glm::dot(current, target), -1.0f, 1.0f);
        const float angle = std::acos(cosTheta);
        if (angle <= maxRadiansDelta || angle < 0.0001f)
        {
            return target;
        }

        glm::vec3 relative = target - (current * cosTheta);
        if (glm::length2(relative) < 0.0001f)
        {
            relative = perpendicularTo(current);
        }
        else
        {
            relative = glm::normalize(relative);
        }

        return glm::normalize((current * std::cos(maxRadiansDelta)) + (relative * std::sin(maxRadiansDelta)));
    }

    glm::vec3 clampMagnitude(const glm::vec3 &vector, float maxMagnitude)
    {
        if (maxMagnitude <= 0.0f)
        {
            return glm::vec3(0.0f);
        }

        const float magnitudeSq = glm::length2(vector);
        if (magnitudeSq <= maxMagnitude * maxMagnitude)
        {
            return vector;
        }

        return glm::normalize(vector) * maxMagnitude;
    }
} // namespace

Target::Target(const glm::vec3 &position, float radius)
    : PhysicsObject(position, glm::vec3(0.0f), 0.0f),
      m_radius(radius),
      m_homeAnchor(position),
      m_referencePosition(position)
{
    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
    {
        std::cerr << "Error: Invalid target position detected during creation" << std::endl;
        setPosition(glm::vec3(0.0f, 180.0f, 0.0f));
        m_homeAnchor = m_position;
        m_referencePosition = m_position;
    }

    if (radius <= 0.0f || std::isnan(radius) || std::isinf(radius))
    {
        std::cerr << "Error: Invalid target radius detected during creation" << std::endl;
        m_radius = 5.0f;
    }

    m_orbitDirection = ((position.x + position.z) >= 0.0f) ? 1 : -1;
    setAIConfig(m_aiConfig);
    m_referencePosition = patrolCenterReference(m_homeAnchor.y);
    m_referenceVelocity = glm::vec3(0.0f);
    m_referenceDistance = horizontalDistanceFromCenter(m_position);

    const glm::vec3 radialFromOrigin = normalizeOrFallback(glm::vec3(m_position.x, 0.0f, m_position.z), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 tangent = normalizeOrFallback(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), radialFromOrigin) * static_cast<float>(m_orbitDirection),
                                                  glm::vec3(0.0f, 0.0f, 1.0f));
    m_velocity = tangent * m_commandedSpeed;
    enforceAirspaceConstraint();
}

void Target::setActive(bool active)
{
    m_isActive = active;
    if (!m_isActive)
    {
        m_velocity = glm::vec3(0.0f);
        resetCountermeasureState();
        return;
    }

    const float cruiseSpeed = m_aiConfig.minSpeed + ((m_aiConfig.maxSpeed - m_aiConfig.minSpeed) * 0.55f);
    const glm::vec3 tangent = normalizeOrFallback(perpendicularTo(glm::vec3(m_position.x, 0.0f, m_position.z)), glm::vec3(0.0f, 0.0f, 1.0f));
    m_commandedSpeed = glm::clamp(cruiseSpeed, m_aiConfig.minSpeed, m_aiConfig.maxSpeed);
    m_velocity = tangent * m_commandedSpeed;
    m_referencePosition = patrolCenterReference(m_homeAnchor.y);
    m_referenceVelocity = glm::vec3(0.0f);
    m_referenceDistance = horizontalDistanceFromCenter(m_position);
    m_remainingFlares = m_flareConfig.inventory;
    resetCountermeasureState();
}

void Target::setAIConfig(const TargetAIConfig &config)
{
    m_aiConfig.minSpeed = std::max(config.minSpeed, kMinimumSpeedMetersPerSecond);
    m_aiConfig.maxSpeed = std::max(config.maxSpeed, m_aiConfig.minSpeed + 10.0f);
    m_aiConfig.preferredDistance = std::max(config.preferredDistance, kReferenceDistanceFloorMeters);
    m_altitudeExcursion = glm::clamp(m_aiConfig.preferredDistance * 0.06f, 35.0f, 140.0f);
    m_nominalAltitude = glm::clamp(std::max(m_homeAnchor.y, 120.0f), std::max(m_radius + 20.0f, kMinimumAltitudeMeters), 600.0f);

    if (glm::length2(m_velocity) > 0.0001f)
    {
        m_velocity = glm::normalize(m_velocity) * glm::clamp(glm::length(m_velocity), m_aiConfig.minSpeed, m_aiConfig.maxSpeed);
    }

    if (m_commandedSpeed <= 0.0f)
    {
        m_commandedSpeed = m_aiConfig.minSpeed + ((m_aiConfig.maxSpeed - m_aiConfig.minSpeed) * 0.55f);
    }
    else
    {
        m_commandedSpeed = glm::clamp(m_commandedSpeed, m_aiConfig.minSpeed, m_aiConfig.maxSpeed);
    }
}

void Target::updateThreatAssessment(const std::vector<Missile *> &missiles)
{
    ThreatAssessment bestThreat;
    m_referencePosition = patrolCenterReference(m_homeAnchor.y);
    m_referenceVelocity = glm::vec3(0.0f);
    m_referenceDistance = horizontalDistanceFromCenter(m_position);

    if (!m_isActive)
    {
        m_threatAssessment = bestThreat;
        m_missileWarningActive = false;
        m_threatTimeToClosestApproach = std::numeric_limits<float>::infinity();
        m_threatClosestApproachDistance = std::numeric_limits<float>::infinity();
        return;
    }

    for (Missile *missile : missiles)
    {
        if (!missile)
        {
            continue;
        }

        const glm::vec3 relativePosition = missile->getPosition() - m_position;
        const float distance = glm::length(relativePosition);

        if (!m_mawsConfig.enabled || distance > m_mawsConfig.detectionRange || distance < 0.1f)
        {
            continue;
        }

        const glm::vec3 relativeVelocity = missile->getVelocity() - m_velocity;
        const float relativeSpeedSq = glm::dot(relativeVelocity, relativeVelocity);
        if (relativeSpeedSq <= 0.0001f)
        {
            continue;
        }

        const float closingSpeed = -glm::dot(relativePosition / distance, relativeVelocity);
        if (closingSpeed <= 0.0f)
        {
            continue;
        }

        const float timeToClosestApproach = std::max(-glm::dot(relativePosition, relativeVelocity) / relativeSpeedSq, 0.0f);
        if (timeToClosestApproach > m_mawsConfig.reactionTimeWindow)
        {
            continue;
        }

        const float closestApproachDistance = glm::length(relativePosition + (relativeVelocity * timeToClosestApproach));
        if (closestApproachDistance > m_mawsConfig.closestApproachThreshold)
        {
            continue;
        }

        if (!bestThreat.active ||
            timeToClosestApproach < bestThreat.timeToClosestApproach ||
            (std::abs(timeToClosestApproach - bestThreat.timeToClosestApproach) < 0.05f &&
             closestApproachDistance < bestThreat.closestApproachDistance))
        {
            bestThreat.active = true;
            bestThreat.distance = distance;
            bestThreat.missilePosition = missile->getPosition();
            bestThreat.missileVelocity = missile->getVelocity();
            bestThreat.timeToClosestApproach = timeToClosestApproach;
            bestThreat.closestApproachDistance = closestApproachDistance;
            bestThreat.closingSpeed = closingSpeed;
        }
    }

    m_threatAssessment = bestThreat;
    m_missileWarningActive = bestThreat.active;
    m_threatTimeToClosestApproach = bestThreat.timeToClosestApproach;
    m_threatClosestApproachDistance = bestThreat.closestApproachDistance;
}

std::vector<FlareLaunchRequest> Target::consumePendingFlareLaunches()
{
    std::vector<FlareLaunchRequest> launches = std::move(m_pendingFlareLaunches);
    m_pendingFlareLaunches.clear();
    return launches;
}

bool Target::isPointInside(const glm::vec3 &point) const
{
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
        std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z))
    {
        std::cerr << "Error: Invalid point coordinates in isPointInside check" << std::endl;
        return false;
    }

    if (std::isnan(m_position.x) || std::isnan(m_position.y) || std::isnan(m_position.z) ||
        std::isinf(m_position.x) || std::isinf(m_position.y) || std::isinf(m_position.z))
    {
        std::cerr << "Error: Invalid target position in isPointInside check" << std::endl;
        return false;
    }

    return glm::length(point - m_position) <= m_radius;
}

void Target::update(float deltaTime)
{
    m_forces = glm::vec3(0.0f);
    m_acceleration = glm::vec3(0.0f);
    m_previousPosition = m_position;

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        return;
    }

    if (!m_isActive)
    {
        m_velocity = glm::vec3(0.0f);
        return;
    }

    updateAutonomousFlight(deltaTime);
    enforceAirspaceConstraint();
    updateCountermeasures(deltaTime, m_velocity);
}

void Target::updateAutonomousFlight(float deltaTime)
{
    const glm::vec3 previousPosition = m_position;
    const glm::vec3 previousVelocity = m_velocity;
    const float minSpeed = std::max(m_aiConfig.minSpeed, kMinimumSpeedMetersPerSecond);
    const float maxSpeed = std::max(m_aiConfig.maxSpeed, minSpeed + 10.0f);
    const float cruiseSpeed = minSpeed + ((maxSpeed - minSpeed) * 0.55f);
    const float distanceBand = std::max(m_aiConfig.preferredDistance * 0.18f, 120.0f);

    float currentSpeed = glm::length(m_velocity);
    if (currentSpeed < 0.1f)
    {
        currentSpeed = cruiseSpeed;
    }

    m_patrolPhase += deltaTime * glm::mix(0.18f, 0.38f, glm::clamp(currentSpeed / maxSpeed, 0.0f, 1.0f));
    if (m_patrolPhase >= glm::two_pi<float>())
    {
        m_patrolPhase = std::fmod(m_patrolPhase, glm::two_pi<float>());
    }

    const glm::vec3 referenceOffset = m_position - m_referencePosition;
    const glm::vec3 horizontalOffset(referenceOffset.x, 0.0f, referenceOffset.z);
    const float referenceDistance = glm::length(horizontalOffset);
    m_referenceDistance = referenceDistance;

    const glm::vec3 awayDirection = normalizeOrFallback(horizontalOffset, m_velocity);
    glm::vec3 tangentDirection = normalizeOrFallback(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), awayDirection) * static_cast<float>(m_orbitDirection),
                                                     perpendicularTo(awayDirection));
    if (glm::dot(tangentDirection, normalizeOrFallback(m_velocity, tangentDirection)) < 0.0f)
    {
        tangentDirection = -tangentDirection;
    }

    const float distanceCorrection = glm::clamp((m_aiConfig.preferredDistance - referenceDistance) / distanceBand, -1.0f, 1.0f);
    glm::vec3 desiredDirection = normalizeOrFallback((tangentDirection * 1.15f) + (awayDirection * distanceCorrection * 0.85f),
                                                     tangentDirection);
    const float outerRecaptureRange = std::max(m_aiConfig.preferredDistance * 0.35f, 250.0f);
    const float outerRecaptureBlend = glm::clamp((referenceDistance - (m_aiConfig.preferredDistance * 1.08f)) / outerRecaptureRange, 0.0f, 1.0f);
    if (outerRecaptureBlend > 0.0f)
    {
        const glm::vec3 recaptureDirection = normalizeOrFallback((-awayDirection * 1.75f) + (tangentDirection * 0.30f), -awayDirection);
        desiredDirection = normalizeOrFallback(glm::mix(desiredDirection, recaptureDirection, outerRecaptureBlend), recaptureDirection);
    }

    const float energyFraction = glm::clamp((currentSpeed - minSpeed) / std::max(maxSpeed - minSpeed, 1.0f), 0.0f, 1.0f);
    if (m_threatAssessment.active)
    {
        const float threatBlend = 1.0f - glm::clamp(m_threatAssessment.timeToClosestApproach / std::max(m_mawsConfig.reactionTimeWindow, 0.1f), 0.0f, 1.0f);
        const glm::vec3 incomingDirection = normalizeOrFallback(m_threatAssessment.missilePosition - m_position, -awayDirection);
        glm::vec3 breakDirection = normalizeOrFallback(glm::cross(incomingDirection, glm::vec3(0.0f, 1.0f, 0.0f)) * static_cast<float>(m_orbitDirection),
                                                       tangentDirection);
        if (glm::dot(breakDirection, tangentDirection) < 0.0f)
        {
            breakDirection = -breakDirection;
        }

        desiredDirection = normalizeOrFallback((breakDirection * 1.45f) +
                                                   (awayDirection * (0.85f + threatBlend)) -
                                                   (incomingDirection * 0.25f),
                                               breakDirection);
        m_aiState = TargetAIState::DEFENSIVE;
    }
    else if (energyFraction < 0.15f)
    {
        m_aiState = TargetAIState::RECOVERING;
    }
    else if (std::abs(distanceCorrection) > 0.35f)
    {
        m_aiState = TargetAIState::REPOSITION;
    }
    else
    {
        m_aiState = TargetAIState::PATROL;
    }

    const float desiredAltitude = computeDesiredAltitude(referenceDistance, currentSpeed);
    const float altitudeError = desiredAltitude - m_position.y;
    desiredDirection = normalizeOrFallback(glm::vec3(desiredDirection.x,
                                                     desiredDirection.y + glm::clamp(altitudeError / 260.0f, -0.42f, 0.42f),
                                                     desiredDirection.z),
                                           desiredDirection);

    const glm::vec3 currentDirection = normalizeOrFallback(previousVelocity, desiredDirection);
    const glm::vec3 desiredHorizontalDirection = normalizeOrFallback(glm::vec3(desiredDirection.x, 0.0f, desiredDirection.z),
                                                                     glm::vec3(currentDirection.x, 0.0f, currentDirection.z));
    const float currentPitch = std::asin(glm::clamp(currentDirection.y, -1.0f, 1.0f));
    const float desiredPitch = std::asin(glm::clamp(desiredDirection.y, -1.0f, 1.0f));
    const float maxPitchDelta = glm::radians(m_threatAssessment.active ? kDefensivePitchRateDegrees : kCruisePitchRateDegrees) * deltaTime;
    const float limitedPitch = currentPitch + glm::clamp(desiredPitch - currentPitch, -maxPitchDelta, maxPitchDelta);
    const float horizontalPitchScale = std::max(std::cos(limitedPitch), 0.05f);
    desiredDirection = normalizeOrFallback(glm::vec3(desiredHorizontalDirection.x * horizontalPitchScale,
                                                     std::sin(limitedPitch),
                                                     desiredHorizontalDirection.z * horizontalPitchScale),
                                           desiredDirection);

    const float desiredSpeed = computeDesiredSpeed(referenceDistance);
    const float accelRate = m_threatAssessment.active ? 18.0f : 9.0f;
    const float decelRate = m_threatAssessment.active ? 24.0f : 12.0f;
    const float lateralAccelerationLimit = m_threatAssessment.active
                                               ? (m_evasiveConfig.lateralAcceleration * 0.72f * m_evasiveConfig.speedMultiplier)
                                               : (m_evasiveConfig.lateralAcceleration * 0.38f);
    const float maxTurnRate = std::min(lateralAccelerationLimit / std::max(currentSpeed, minSpeed * 0.8f),
                                       glm::radians(m_threatAssessment.active ? kDefensiveMaxTurnRateDegrees : kCruiseMaxTurnRateDegrees));
    const glm::vec3 limitedDirection = rotateTowardsDirection(currentDirection, desiredDirection, maxTurnRate * deltaTime);
    const glm::vec3 desiredVelocity = limitedDirection * desiredSpeed;

    const float longitudinalVelocity = glm::dot(desiredVelocity - previousVelocity, currentDirection);
    const glm::vec3 lateralVelocityDelta = (desiredVelocity - previousVelocity) - (currentDirection * longitudinalVelocity);

    const float maxLongitudinalDelta = ((longitudinalVelocity >= 0.0f) ? accelRate : decelRate) * deltaTime;
    const float clampedLongitudinalDelta = glm::clamp(longitudinalVelocity, -maxLongitudinalDelta, maxLongitudinalDelta);
    const glm::vec3 clampedLateralDelta = clampMagnitude(lateralVelocityDelta, lateralAccelerationLimit * deltaTime);

    const float lateralDeltaMagnitude = glm::length(clampedLateralDelta);
    const float lateralAccelFraction = glm::clamp(lateralDeltaMagnitude / std::max(lateralAccelerationLimit * deltaTime, 0.0001f), 0.0f, 1.0f);
    const float climbPenalty = std::max(limitedDirection.y, 0.0f) * glm::mix(1.5f, 4.0f, lateralAccelFraction);
    const float inducedDrag = lateralAccelFraction * lateralAccelFraction * (m_threatAssessment.active ? 16.0f : 8.5f);
    const float aoaPenalty = std::abs(desiredPitch - currentPitch) * (m_threatAssessment.active ? 9.0f : 5.0f);
    const float parasiticDrag = currentSpeed * currentSpeed *
                                (m_threatAssessment.active ? kDefensiveParasiticDragFactor : kCruiseParasiticDragFactor);
    const float energyLossDelta = (climbPenalty + inducedDrag + aoaPenalty + parasiticDrag) * deltaTime;

    glm::vec3 updatedVelocity = previousVelocity +
                                (currentDirection * clampedLongitudinalDelta) +
                                clampedLateralDelta;

    const float updatedSpeed = glm::length(updatedVelocity);
    glm::vec3 updatedDirection = normalizeOrFallback(updatedVelocity, limitedDirection);
    float clampedSpeed = updatedSpeed - energyLossDelta;
    clampedSpeed = glm::clamp(clampedSpeed, minSpeed, maxSpeed);

    // Keep the target flying the turn rather than teleporting to a new heading.
    m_velocity = updatedDirection * clampedSpeed;
    m_commandedSpeed = desiredSpeed;
    m_position += (previousVelocity + m_velocity) * (0.5f * deltaTime);
    m_acceleration = (m_velocity - previousVelocity) / std::max(deltaTime, 0.0001f);
}

float Target::computeDesiredSpeed(float referenceDistance) const
{
    const float minSpeed = std::max(m_aiConfig.minSpeed, kMinimumSpeedMetersPerSecond);
    const float maxSpeed = std::max(m_aiConfig.maxSpeed, minSpeed + 10.0f);
    const float cruiseSpeed = minSpeed + ((maxSpeed - minSpeed) * 0.55f);
    const float distanceBand = std::max(m_aiConfig.preferredDistance * 0.18f, 120.0f);
    const float distanceError = glm::clamp((m_aiConfig.preferredDistance - referenceDistance) / distanceBand, -1.0f, 1.0f);
    const float outerRecaptureRange = std::max(m_aiConfig.preferredDistance * 0.35f, 250.0f);
    const float outerRecaptureBlend = glm::clamp((referenceDistance - (m_aiConfig.preferredDistance * 1.08f)) / outerRecaptureRange, 0.0f, 1.0f);

    float desiredSpeed = cruiseSpeed;

    if (m_threatAssessment.active)
    {
        const float threatBlend = 1.0f - glm::clamp(m_threatAssessment.timeToClosestApproach / std::max(m_mawsConfig.reactionTimeWindow, 0.1f), 0.0f, 1.0f);
        desiredSpeed = glm::mix(cruiseSpeed, maxSpeed, 0.48f + (0.24f * threatBlend));
    }
    else if (outerRecaptureBlend > 0.0f)
    {
        desiredSpeed = glm::mix(cruiseSpeed, minSpeed, outerRecaptureBlend);
    }
    else if (distanceError > 0.15f)
    {
        desiredSpeed = glm::mix(cruiseSpeed, maxSpeed, distanceError);
    }
    else if (distanceError < -0.25f)
    {
        desiredSpeed = glm::mix(cruiseSpeed, minSpeed + ((maxSpeed - minSpeed) * 0.18f), -distanceError);
    }

    return glm::clamp(desiredSpeed, minSpeed, maxSpeed);
}

float Target::computeDesiredAltitude(float referenceDistance, float currentSpeed) const
{
    const float minAltitude = std::max(m_radius + 12.0f, kMinimumAltitudeMeters);
    const float maxAltitude = std::max(minAltitude + 120.0f, glm::clamp(m_aiConfig.preferredDistance * 0.35f, 180.0f, 700.0f));
    const float minSpeed = std::max(m_aiConfig.minSpeed, kMinimumSpeedMetersPerSecond);
    const float maxSpeed = std::max(m_aiConfig.maxSpeed, minSpeed + 10.0f);
    const float midSpeed = minSpeed + ((maxSpeed - minSpeed) * 0.5f);

    float desiredAltitude = m_nominalAltitude + (std::sin(m_patrolPhase) * m_altitudeExcursion);
    if (referenceDistance < m_aiConfig.preferredDistance * 0.75f)
    {
        desiredAltitude += 35.0f;
    }
    else if (referenceDistance > m_aiConfig.preferredDistance * 1.2f)
    {
        desiredAltitude -= 25.0f;
    }

    if (m_threatAssessment.active)
    {
        if (currentSpeed < midSpeed)
        {
            desiredAltitude -= 55.0f;
        }
        else if (m_threatAssessment.missilePosition.y < m_position.y)
        {
            desiredAltitude += 40.0f;
        }
        else
        {
            desiredAltitude -= 30.0f;
        }
    }

    return glm::clamp(desiredAltitude, minAltitude, maxAltitude);
}

void Target::enforceAirspaceConstraint()
{
    const float minimumAltitude = std::max(m_radius + 2.0f, kMinimumAltitudeMeters);
    const float maximumAltitude = std::max(minimumAltitude + 120.0f, glm::clamp(m_aiConfig.preferredDistance * 0.4f, 220.0f, 800.0f));
    const float patrolRadius = std::max(m_aiConfig.preferredDistance, kReferenceDistanceFloorMeters);
    const float maximumHorizontalDistance = std::max(patrolRadius * (m_threatAssessment.active ? 1.90f : 1.45f),
                                                     patrolRadius + (m_threatAssessment.active ? 1600.0f : 700.0f));

    const glm::vec2 horizontalPosition = horizontalComponents(m_position);
    const float horizontalDistance = glm::length(horizontalPosition);
    if (horizontalDistance > maximumHorizontalDistance && horizontalDistance > 0.001f)
    {
        const glm::vec2 radialDirection = horizontalPosition / horizontalDistance;
        const glm::vec2 constrainedPosition = radialDirection * maximumHorizontalDistance;
        m_position.x = constrainedPosition.x;
        m_position.z = constrainedPosition.y;

        glm::vec2 horizontalVelocity = horizontalComponents(m_velocity);
        const float outwardSpeed = glm::dot(horizontalVelocity, radialDirection);
        const glm::vec2 currentHorizontalDirection = (glm::length2(horizontalVelocity) > 0.0001f)
                                                         ? glm::normalize(horizontalVelocity)
                                                         : glm::vec2(-radialDirection.y, radialDirection.x);
        glm::vec2 tangentDirection(-radialDirection.y, radialDirection.x);
        if (glm::dot(tangentDirection, currentHorizontalDirection) < 0.0f)
        {
            tangentDirection = -tangentDirection;
        }

        const float overshootDistance = horizontalDistance - maximumHorizontalDistance;
        const float recoveryBlend = glm::clamp(overshootDistance / kBoundaryRecoverySoftZoneMeters, 0.0f, 1.0f);
        if (outwardSpeed > 0.0f)
        {
            horizontalVelocity -= radialDirection * (outwardSpeed * (0.92f + (0.08f * recoveryBlend)));
        }

        if (glm::length2(horizontalVelocity) < 0.0001f)
        {
            horizontalVelocity = tangentDirection * std::min(std::max(m_aiConfig.minSpeed * 0.7f, m_commandedSpeed * 0.55f), m_aiConfig.maxSpeed);
        }

        const float tangentialSpeed = glm::dot(horizontalVelocity, tangentDirection);
        const float minimumTangentialSpeed = std::min(std::max(m_aiConfig.minSpeed * 0.48f, 55.0f), m_aiConfig.maxSpeed * 0.68f);
        if (tangentialSpeed < minimumTangentialSpeed)
        {
            horizontalVelocity += tangentDirection * (minimumTangentialSpeed - tangentialSpeed);
        }

        horizontalVelocity -= radialDirection * (kBoundaryRecoveryBiasMetersPerSecond * (0.35f + (0.65f * recoveryBlend)));

        m_velocity.x = horizontalVelocity.x;
        m_velocity.z = horizontalVelocity.y;
    }

    if (m_position.y < minimumAltitude)
    {
        m_position.y = minimumAltitude;
        if (m_velocity.y < 0.0f)
        {
            m_velocity.y = std::abs(m_velocity.y) * 0.35f;
        }
    }
    else if (m_position.y > maximumAltitude)
    {
        m_position.y = maximumAltitude;
        if (m_velocity.y > 0.0f)
        {
            m_velocity.y = -m_velocity.y * 0.25f;
        }
    }

    if (m_homeAnchor.y < minimumAltitude)
    {
        m_homeAnchor.y = minimumAltitude;
    }
}

void Target::updateCountermeasures(float deltaTime, const glm::vec3 &currentVelocity)
{
    if (!m_isActive)
    {
        return;
    }

    m_countermeasureCooldown = std::max(m_countermeasureCooldown - deltaTime, 0.0f);
    m_burstShotTimer = std::max(m_burstShotTimer - deltaTime, 0.0f);

    if (m_missileWarningActive &&
        m_flareConfig.enabled &&
        m_remainingFlares > 0 &&
        m_pendingBurstShots <= 0 &&
        m_countermeasureCooldown <= 0.0f)
    {
        m_pendingBurstShots = std::min(m_flareConfig.burstSize, m_remainingFlares);
        m_countermeasureCooldown = m_flareConfig.cooldown;
        m_burstShotTimer = 0.0f;
    }

    while (m_pendingBurstShots > 0 && m_burstShotTimer <= 0.0f && m_remainingFlares > 0)
    {
        const glm::vec3 velocityDirection = normalizeOrFallback(currentVelocity, glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::vec3 aftDirection = -velocityDirection;
        const glm::vec3 lateralDirection = perpendicularTo(velocityDirection) * static_cast<float>(m_flareSpreadSign);

        FlareLaunchRequest request;
        request.position = m_position + (aftDirection * (m_radius + 1.2f)) + (lateralDirection * 0.8f);
        request.velocity = currentVelocity +
                           (aftDirection * m_flareConfig.ejectSpeed) +
                           (lateralDirection * m_flareConfig.ejectSpeed * 0.18f) +
                           glm::vec3(0.0f, -m_flareConfig.ejectSpeed * 0.12f, 0.0f);
        request.mass = m_flareConfig.mass;
        request.dragCoefficient = m_flareConfig.dragCoefficient;
        request.crossSectionalArea = m_flareConfig.crossSectionalArea;
        request.lifetime = m_flareConfig.lifetime;
        request.heatSignature = m_flareConfig.heatSignature;
        request.heatDecayRate = m_flareConfig.heatDecayRate;
        m_pendingFlareLaunches.push_back(request);

        --m_pendingBurstShots;
        --m_remainingFlares;
        m_flareSpreadSign *= -1;
        m_burstShotTimer += m_flareConfig.burstInterval;
    }
}

void Target::resetCountermeasureState()
{
    m_pendingFlareLaunches.clear();
    m_threatAssessment = ThreatAssessment{};
    m_missileWarningActive = false;
    m_threatTimeToClosestApproach = std::numeric_limits<float>::infinity();
    m_threatClosestApproachDistance = std::numeric_limits<float>::infinity();
    m_countermeasureCooldown = 0.0f;
    m_burstShotTimer = 0.0f;
    m_pendingBurstShots = 0;
    m_flareSpreadSign = 1;
}
