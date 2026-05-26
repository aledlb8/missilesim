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

    // Force-based flight model defaults (physical airframe constants; the
    // energy state now emerges from real lift/drag/thrust rather than scripted
    // energy-penalty scalars).
    setMass(8000.0f);        // light combat aircraft / large UCAV class
    setMaxLoadFactorG(9.0f); // structural load-factor limit
    m_maxThrust = 75000.0f;  // engine thrust ceiling (N)
    m_aeroProfile.referenceArea = 12.0f;       // wing reference area (m^2)
    m_aeroProfile.baseDragCoefficient = 0.022f; // clean zero-lift drag Cd0
    m_aeroProfile.aspectRatio = 6.0f;
    m_aeroProfile.oswaldEfficiency = 0.85f;
    m_aeroProfile.maxLiftCoefficient = 1.4f;
    m_aeroProfile.machDragMultiplier = missilesim::physics::defaultSupersonicDragRiseCurve();

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

    // Low-pass the acceleration used for visual banking so the rendered roll
    // tracks the sustained turn rather than per-step heading corrections.
    const float bankSmoothing = glm::clamp(deltaTime / 0.25f, 0.0f, 1.0f);
    m_smoothedAcceleration = glm::mix(m_smoothedAcceleration, m_acceleration, bankSmoothing);
}

void Target::updateAutonomousFlight(float deltaTime)
{
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

        desiredDirection = normalizeOrFallback((breakDirection * m_evasiveConfig.defensiveBreakWeight) +
                                                   (awayDirection * (m_evasiveConfig.defensiveAwayWeight +
                                                                     (threatBlend * m_evasiveConfig.defensiveThreatAwayWeight))) -
                                                   (incomingDirection * m_evasiveConfig.defensiveIncomingWeight),
                                               breakDirection);
        m_aiState = TargetAIState::DEFENSIVE;
    }
    else if (energyFraction < m_evasiveConfig.recoveryEnergyThreshold)
    {
        m_aiState = TargetAIState::RECOVERING;
    }
    else if (std::abs(distanceCorrection) > m_evasiveConfig.repositionDistanceThreshold)
    {
        m_aiState = TargetAIState::REPOSITION;
    }
    else
    {
        m_aiState = TargetAIState::PATROL;
    }

    const float desiredAltitude = computeDesiredAltitude(referenceDistance, currentSpeed);
    const float altitudeError = desiredAltitude - m_position.y;
    const float maxAltitudePitchBias = std::max(m_evasiveConfig.maxAltitudePitchBias, 0.0f);
    desiredDirection = normalizeOrFallback(glm::vec3(desiredDirection.x,
                                                     desiredDirection.y + glm::clamp(altitudeError / m_evasiveConfig.altitudeCorrectionRange,
                                                                                    -maxAltitudePitchBias,
                                                                                    maxAltitudePitchBias),
                                                     desiredDirection.z),
                                           desiredDirection);

    const glm::vec3 currentDirection = normalizeOrFallback(previousVelocity, desiredDirection);
    const glm::vec3 desiredHorizontalDirection = normalizeOrFallback(glm::vec3(desiredDirection.x, 0.0f, desiredDirection.z),
                                                                     glm::vec3(currentDirection.x, 0.0f, currentDirection.z));
    const float currentPitch = std::asin(glm::clamp(currentDirection.y, -1.0f, 1.0f));
    const float desiredPitch = std::asin(glm::clamp(desiredDirection.y, -1.0f, 1.0f));
    const float pitchRateDegrees = m_threatAssessment.active
                                       ? m_evasiveConfig.defensivePitchRateDegrees
                                       : m_evasiveConfig.cruisePitchRateDegrees;
    const float maxPitchDelta = glm::radians(pitchRateDegrees) * deltaTime;
    const float limitedPitch = currentPitch + glm::clamp(desiredPitch - currentPitch, -maxPitchDelta, maxPitchDelta);
    const float horizontalPitchScale = std::max(std::cos(limitedPitch), 0.05f);
    desiredDirection = normalizeOrFallback(glm::vec3(desiredHorizontalDirection.x * horizontalPitchScale,
                                                     std::sin(limitedPitch),
                                                     desiredHorizontalDirection.z * horizontalPitchScale),
                                           desiredDirection);

    const float desiredSpeed = computeDesiredSpeed(referenceDistance);

    // --- Heading control: turn rate limited by available aerodynamic g ------
    // The lateral acceleration the airframe can pull is bounded by the lift
    // available at the local dynamic pressure and by its structural g-limit,
    // exactly like the missile. This replaces the former scripted turn-rate
    // and lateral-acceleration tuning scalars.
    const float dynamicPressure = 0.5f * std::max(m_ambientDensity, 0.0f) * currentSpeed * currentSpeed;
    const float referenceArea = (m_aeroProfile.referenceArea > 0.0f) ? m_aeroProfile.referenceArea : 1.0f;
    const float vehicleMass = std::max(m_mass, 1.0f);
    const float structuralLimit = (m_maxLoadFactorG > 0.0f) ? (m_maxLoadFactorG * 9.80665f) : 1.0e9f;
    const float aerodynamicLimit = (dynamicPressure * m_aeroProfile.maxLiftCoefficient * referenceArea) / vehicleMass;
    const float maxLateralAcceleration = std::min(aerodynamicLimit, structuralLimit);
    const float maxTurnRate = maxLateralAcceleration / std::max(currentSpeed, 1.0f);
    const glm::vec3 limitedDirection = rotateTowardsDirection(currentDirection, desiredDirection, maxTurnRate * deltaTime);

    // Operating lift coefficient for this turn (centripetal accel = omega * v),
    // used to charge real induced drag.
    const float turnAngle = std::acos(glm::clamp(glm::dot(currentDirection, limitedDirection), -1.0f, 1.0f));
    const float lateralAcceleration = (deltaTime > 0.0f) ? ((turnAngle / deltaTime) * currentSpeed) : 0.0f;
    const float liftCoefficient = glm::clamp(
        (vehicleMass * lateralAcceleration) / std::max(dynamicPressure * referenceArea, 1e-4f),
        0.0f, m_aeroProfile.maxLiftCoefficient);
    m_commandedLiftCoefficient = liftCoefficient;

    // --- Energy: real thrust vs aerodynamic drag and gravity along the path -
    const float mach = (m_speedOfSound > 1e-3f) ? (currentSpeed / m_speedOfSound) : 0.0f;
    const float dragCoefficient = missilesim::physics::zeroLiftDragCoefficient(m_aeroProfile, mach) +
                                  missilesim::physics::inducedDragCoefficient(m_aeroProfile, liftCoefficient);
    const float dragForce = dynamicPressure * dragCoefficient * referenceArea;
    const float gravityAlongPath = 9.80665f * limitedDirection.y; // decelerates in a climb, accelerates in a dive
    const float engineResponseTime = 1.0f;                        // engine speed-tracking time constant (s)
    const float desiredAlongAcceleration = (desiredSpeed - currentSpeed) / engineResponseTime;
    const float requiredThrust =
        (vehicleMass * desiredAlongAcceleration) + dragForce + (vehicleMass * gravityAlongPath);
    const float thrustForce = glm::clamp(requiredThrust, 0.0f, m_maxThrust);
    const float alongAcceleration = ((thrustForce - dragForce) / vehicleMass) - gravityAlongPath;

    float newSpeed = currentSpeed + (alongAcceleration * deltaTime);
    // Envelope slightly wider than the AI speed band: a numeric glitch can
    // never freeze or reverse the target, but in normal flight the physical
    // thrust/drag balance, not this clamp, governs the speed.
    newSpeed = glm::clamp(newSpeed, minSpeed * 0.5f, maxSpeed * 1.15f);

    // Fly the turn rather than teleporting to a new heading.
    m_velocity = limitedDirection * newSpeed;
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
        const float defensiveSpeedBlend = glm::clamp(m_evasiveConfig.defensiveSpeedBlend +
                                                         (m_evasiveConfig.defensiveSpeedUrgencyBlend * threatBlend),
                                                     0.0f,
                                                     1.0f);
        desiredSpeed = glm::mix(cruiseSpeed, maxSpeed, defensiveSpeedBlend);
    }
    else if (outerRecaptureBlend > 0.0f)
    {
        desiredSpeed = glm::mix(cruiseSpeed, minSpeed, outerRecaptureBlend);
    }
    else if (distanceError > m_evasiveConfig.nearDistanceSpeedupThreshold)
    {
        desiredSpeed = glm::mix(cruiseSpeed, maxSpeed, distanceError);
    }
    else if (distanceError < -m_evasiveConfig.farDistanceSlowdownThreshold)
    {
        desiredSpeed = glm::mix(cruiseSpeed,
                                minSpeed + ((maxSpeed - minSpeed) * m_evasiveConfig.farDistanceSpeedFloor),
                                -distanceError);
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
        desiredAltitude += m_evasiveConfig.innerDistanceAltitudeOffset;
    }
    else if (referenceDistance > m_aiConfig.preferredDistance * 1.2f)
    {
        desiredAltitude += m_evasiveConfig.outerDistanceAltitudeOffset;
    }

    if (m_threatAssessment.active)
    {
        if (currentSpeed < midSpeed)
        {
            desiredAltitude += m_evasiveConfig.defensiveLowEnergyAltitudeOffset;
        }
        else if (m_threatAssessment.missilePosition.y < m_position.y)
        {
            desiredAltitude += m_evasiveConfig.defensiveThreatBelowAltitudeOffset;
        }
        else
        {
            desiredAltitude += m_evasiveConfig.defensiveThreatAboveAltitudeOffset;
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
        request.position = m_position +
                           (aftDirection * (m_radius + m_flareConfig.aftLaunchOffset)) +
                           (lateralDirection * m_flareConfig.lateralLaunchOffset);
        request.velocity = currentVelocity +
                           (aftDirection * m_flareConfig.ejectSpeed) +
                           (lateralDirection * m_flareConfig.ejectSpeed * m_flareConfig.lateralEjectFraction) +
                           glm::vec3(0.0f, -m_flareConfig.ejectSpeed * m_flareConfig.downwardEjectFraction, 0.0f);
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
