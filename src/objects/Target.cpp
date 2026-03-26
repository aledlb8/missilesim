#include "Target.h"
#include "Missile.h"
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

namespace
{
constexpr float kMinimumMovementPeriodSeconds = 0.1f;
constexpr float kMinimumMovementRadiusMeters = 0.1f;
constexpr float kMinimumForwardSpeedFraction = 0.35f;

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

glm::vec3 clampMagnitude(const glm::vec3 &vector, float maxMagnitude)
{
    const float magnitudeSq = glm::length2(vector);
    const float maxMagnitudeSq = maxMagnitude * maxMagnitude;
    if (magnitudeSq <= maxMagnitudeSq || magnitudeSq <= 0.0001f)
    {
        return vector;
    }

    return glm::normalize(vector) * maxMagnitude;
}
} // namespace

Target::Target(const glm::vec3 &position, float radius)
    : PhysicsObject(position, glm::vec3(0.0f), 0.0f),
      m_radius(radius),
      m_initialPosition(position),
      m_patternPosition(position)
{
    m_movementCenter = position;

    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
    {
        std::cerr << "Error: Invalid target position detected during creation" << std::endl;
        setPosition(glm::vec3(0.0f, 100.0f, 0.0f));
        m_initialPosition = m_position;
        m_patternPosition = m_position;
        m_movementCenter = m_position;
    }

    if (radius <= 0.0f || std::isnan(radius) || std::isinf(radius))
    {
        std::cerr << "Error: Invalid target radius detected during creation" << std::endl;
        m_radius = 5.0f;
    }

    m_remainingFlares = m_flareConfig.inventory;
    enforceAirspaceConstraint();
    m_patternPosition = m_position;
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

    m_remainingFlares = m_flareConfig.inventory;
    resetCountermeasureState();
}

void Target::setMAWSConfig(const MAWSConfig &config)
{
    m_mawsConfig.enabled = config.enabled;
    m_mawsConfig.detectionRange = std::max(config.detectionRange, 0.0f);
    m_mawsConfig.reactionTimeWindow = std::max(config.reactionTimeWindow, 0.1f);
    m_mawsConfig.closestApproachThreshold = std::max(config.closestApproachThreshold, m_radius);
}

void Target::setFlareDispenserConfig(const FlareDispenserConfig &config)
{
    m_flareConfig.enabled = config.enabled;
    m_flareConfig.inventory = std::max(config.inventory, 0);
    m_flareConfig.burstSize = std::max(config.burstSize, 1);
    m_flareConfig.burstInterval = std::max(config.burstInterval, 0.01f);
    m_flareConfig.cooldown = std::max(config.cooldown, 0.0f);
    m_flareConfig.ejectSpeed = std::max(config.ejectSpeed, 0.0f);
    m_flareConfig.lifetime = std::max(config.lifetime, 0.1f);
    m_flareConfig.heatSignature = std::max(config.heatSignature, 0.0f);
    m_flareConfig.heatDecayRate = std::max(config.heatDecayRate, 0.0f);
    m_flareConfig.mass = std::max(config.mass, 0.05f);
    m_flareConfig.dragCoefficient = std::max(config.dragCoefficient, 0.0f);
    m_flareConfig.crossSectionalArea = std::max(config.crossSectionalArea, 0.0001f);
    m_remainingFlares = m_flareConfig.inventory;
}

void Target::setEvasiveManeuverConfig(const EvasiveManeuverConfig &config)
{
    m_evasiveConfig.enabled = config.enabled;
    m_evasiveConfig.lateralAcceleration = std::max(config.lateralAcceleration, 0.0f);
    m_evasiveConfig.verticalBias = std::clamp(config.verticalBias, -1.0f, 1.0f);
    m_evasiveConfig.maxOffset = std::max(config.maxOffset, 0.0f);
    m_evasiveConfig.weavePeriod = std::max(config.weavePeriod, 0.2f);
    m_evasiveConfig.recoveryRate = std::max(config.recoveryRate, 0.1f);
    m_evasiveConfig.speedMultiplier = std::max(config.speedMultiplier, 0.5f);
}

void Target::updateThreatAssessment(const std::vector<Missile *> &missiles)
{
    ThreatAssessment bestThreat;

    if (!m_isActive || !m_mawsConfig.enabled)
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
        if (distance > m_mawsConfig.detectionRange || distance < 0.1f)
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
        m_velocity = glm::vec3(0.0f);
        return;
    }

    if (!m_isActive)
    {
        m_velocity = glm::vec3(0.0f);
        return;
    }

    const glm::vec3 previousPosition = m_position;
    const glm::vec3 previousPatternPosition = m_patternPosition;

    m_movementTime += deltaTime;

    switch (m_movementPattern)
    {
    case TargetMovementPattern::LINEAR:
        applyLinearMovement(deltaTime);
        break;
    case TargetMovementPattern::CIRCULAR:
        applyCircularMovement(deltaTime);
        break;
    case TargetMovementPattern::SINUSOIDAL:
        applySinusoidalMovement(deltaTime);
        break;
    case TargetMovementPattern::RANDOM:
        applyRandomMovement(deltaTime);
        break;
    case TargetMovementPattern::STATIONARY:
    default:
        break;
    }

    m_position = m_patternPosition;
    enforceAirspaceConstraint();
    m_patternPosition = m_position;

    const glm::vec3 baseVelocity = (m_patternPosition - previousPatternPosition) / deltaTime;
    applyEvasiveOffset(deltaTime, baseVelocity);

    m_position = m_patternPosition + m_evasionOffset;
    enforceAirspaceConstraint();
    m_velocity = (m_position - previousPosition) / deltaTime;

    updateCountermeasures(deltaTime, m_velocity);
}

void Target::applyLinearMovement(float deltaTime)
{
    const float minimumAltitude = std::max(m_radius + 2.0f, 5.0f);
    const glm::vec3 velocity = m_movementDirection * m_movementSpeed;
    m_patternPosition += velocity * deltaTime;

    for (int i = 0; i < 3; ++i)
    {
        if (m_patternPosition[i] > m_movementCenter[i] + m_movementAmplitude)
        {
            m_patternPosition[i] = m_movementCenter[i] + m_movementAmplitude;
            m_movementDirection[i] = -m_movementDirection[i];
        }
        else
        {
            float lowerBound = m_movementCenter[i] - m_movementAmplitude;
            if (i == 1)
            {
                lowerBound = std::max(lowerBound, minimumAltitude);
            }

            if (m_patternPosition[i] < lowerBound)
            {
                m_patternPosition[i] = lowerBound;
                m_movementDirection[i] = -m_movementDirection[i];
            }
        }
    }
}

void Target::applyCircularMovement(float deltaTime)
{
    (void)deltaTime;

    const float radius = std::max(m_movementAmplitude, kMinimumMovementRadiusMeters);
    const float requestedAngularRate = (2.0f * glm::pi<float>()) / std::max(m_movementPeriod, kMinimumMovementPeriodSeconds);
    const float verticalAmplitude = radius * 0.2f;
    const float maxPathDerivativeMagnitude = std::sqrt((radius * radius) + ((2.0f * verticalAmplitude) * (2.0f * verticalAmplitude)));
    const float maxAngularRateFromSpeed = (m_movementSpeed > 0.0f)
                                              ? (m_movementSpeed / std::max(maxPathDerivativeMagnitude, kMinimumMovementRadiusMeters))
                                              : 0.0f;
    const float effectiveAngularRate = std::min(requestedAngularRate, maxAngularRateFromSpeed);
    const float angle = m_movementTime * effectiveAngularRate;

    m_patternPosition.x = m_movementCenter.x + radius * std::cos(angle);
    m_patternPosition.z = m_movementCenter.z + radius * std::sin(angle);
    m_patternPosition.y = m_movementCenter.y + verticalAmplitude * std::sin(angle * 2.0f);
}

void Target::applySinusoidalMovement(float deltaTime)
{
    const glm::vec3 forwardDirection = normalizeOrFallback(m_movementDirection, glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 lateralDirection = perpendicularTo(forwardDirection);
    const float amplitude = std::max(m_movementAmplitude, 0.0f);
    const float requestedAngularRate = (2.0f * glm::pi<float>()) / std::max(m_movementPeriod, kMinimumMovementPeriodSeconds);
    const float cruiseSpeed = std::max(m_movementSpeed, 0.0f);

    float effectiveAngularRate = requestedAngularRate;
    if (amplitude > 0.001f && cruiseSpeed > 0.0f)
    {
        const float reservedForwardSpeed = cruiseSpeed * kMinimumForwardSpeedFraction;
        const float maxLateralSpeed = std::sqrt(std::max((cruiseSpeed * cruiseSpeed) - (reservedForwardSpeed * reservedForwardSpeed), 0.0f));
        const float maxAngularRateFromSpeed = maxLateralSpeed / amplitude;
        effectiveAngularRate = std::min(requestedAngularRate, maxAngularRateFromSpeed);
    }
    else if (cruiseSpeed <= 0.0f)
    {
        effectiveAngularRate = 0.0f;
    }

    const float angle = m_movementTime * effectiveAngularRate;
    const float lateralVelocityMagnitude = amplitude * effectiveAngularRate * std::cos(angle);
    const float forwardSpeed = std::sqrt(std::max((cruiseSpeed * cruiseSpeed) - (lateralVelocityMagnitude * lateralVelocityMagnitude), 0.0f));

    m_wavePathDistance += forwardSpeed * deltaTime;
    m_patternPosition = m_initialPosition +
                        (forwardDirection * m_wavePathDistance) +
                        (lateralDirection * (amplitude * std::sin(angle)));

    if (glm::length(m_patternPosition - m_initialPosition) > std::max(amplitude * 10.0f, 250.0f))
    {
        m_initialPosition = m_patternPosition;
        m_wavePathDistance = 0.0f;
        m_movementTime = 0.0f;
    }
}

void Target::applyRandomMovement(float deltaTime)
{
    const float minimumAltitude = std::max(m_radius + 2.0f, 5.0f);

    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    if (m_movementTime >= m_movementPeriod)
    {
        m_movementTime = 0.0f;

        glm::vec3 newDirection(dis(gen), dis(gen) * 0.5f, dis(gen));
        m_movementDirection = normalizeOrFallback(newDirection, glm::vec3(1.0f, 0.1f, 0.0f));
    }

    m_patternPosition += m_movementDirection * m_movementSpeed * deltaTime;

    for (int i = 0; i < 3; ++i)
    {
        if (m_patternPosition[i] > m_movementCenter[i] + m_movementAmplitude)
        {
            m_patternPosition[i] = m_movementCenter[i] + m_movementAmplitude;
            m_movementDirection[i] = -std::abs(m_movementDirection[i]);
        }
        else
        {
            float lowerBound = m_movementCenter[i] - m_movementAmplitude;
            if (i == 1)
            {
                lowerBound = std::max(lowerBound, minimumAltitude);
            }

            if (m_patternPosition[i] < lowerBound)
            {
                m_patternPosition[i] = lowerBound;
                m_movementDirection[i] = std::abs(m_movementDirection[i]);
            }
        }
    }
}

void Target::enforceAirspaceConstraint()
{
    const float minimumAltitude = std::max(m_radius + 2.0f, 5.0f);

    if (m_position.y < minimumAltitude)
    {
        m_position.y = minimumAltitude;
        if (m_evasionOffset.y < 0.0f)
        {
            m_evasionOffset.y = 0.0f;
        }
        if (m_evasionVelocity.y < 0.0f)
        {
            m_evasionVelocity.y = 0.0f;
        }
    }

    if (m_initialPosition.y < minimumAltitude)
    {
        m_initialPosition.y = minimumAltitude;
    }

    if (m_movementCenter.y < minimumAltitude)
    {
        m_movementCenter.y = minimumAltitude;
    }

    if (m_movementDirection.y < 0.0f && m_position.y <= minimumAltitude + 0.01f)
    {
        m_movementDirection.y = std::abs(m_movementDirection.y);
        m_movementDirection = normalizeOrFallback(m_movementDirection, glm::vec3(1.0f, 0.1f, 0.0f));
    }
}

void Target::applyEvasiveOffset(float deltaTime, const glm::vec3 &baseVelocity)
{
    if (!m_evasiveConfig.enabled || m_evasiveConfig.maxOffset <= 0.0f)
    {
        m_evasionVelocity *= std::max(0.0f, 1.0f - (m_evasiveConfig.recoveryRate * deltaTime));
        m_evasionOffset += m_evasionVelocity * deltaTime;
        return;
    }

    if (m_missileWarningActive && m_threatAssessment.active)
    {
        m_evasionPhase += deltaTime;
        if (m_evasionPhase >= m_evasiveConfig.weavePeriod)
        {
            m_evasionPhase = std::fmod(m_evasionPhase, m_evasiveConfig.weavePeriod);
        }

        const float weaveSign = (m_evasionPhase < (m_evasiveConfig.weavePeriod * 0.5f)) ? 1.0f : -1.0f;
        const glm::vec3 baseDirection = normalizeOrFallback(baseVelocity, m_movementDirection);
        const glm::vec3 toMissile = normalizeOrFallback(m_threatAssessment.missilePosition - m_position, -baseDirection);
        glm::vec3 lateralDirection = glm::cross(toMissile, glm::vec3(0.0f, 1.0f, 0.0f));
        lateralDirection = normalizeOrFallback(lateralDirection, perpendicularTo(toMissile));
        const float verticalSign = (m_position.y <= m_movementCenter.y) ? 1.0f : (toMissile.y > 0.0f ? -1.0f : 1.0f);
        const glm::vec3 desiredDirection = normalizeOrFallback(
            (lateralDirection * weaveSign) +
                (glm::vec3(0.0f, 1.0f, 0.0f) * m_evasiveConfig.verticalBias * verticalSign) +
                (baseDirection * 0.35f),
            lateralDirection);

        m_evasionVelocity += desiredDirection * m_evasiveConfig.lateralAcceleration * deltaTime;
        const float baseSpeed = std::max(glm::length(baseVelocity), std::max(m_movementSpeed, 10.0f));
        const float maxOffsetSpeed = std::max(baseSpeed * std::max(m_evasiveConfig.speedMultiplier, 0.5f) * 0.65f, 18.0f);
        m_evasionVelocity = clampMagnitude(m_evasionVelocity, maxOffsetSpeed);
    }
    else
    {
        const float recoveryBlend = glm::clamp(deltaTime * m_evasiveConfig.recoveryRate, 0.0f, 1.0f);
        const glm::vec3 recoveryVelocity = -m_evasionOffset * m_evasiveConfig.recoveryRate;
        m_evasionVelocity = glm::mix(m_evasionVelocity, recoveryVelocity, recoveryBlend);
    }

    m_evasionOffset += m_evasionVelocity * deltaTime;

    const float offsetMagnitude = glm::length(m_evasionOffset);
    if (offsetMagnitude > m_evasiveConfig.maxOffset)
    {
        const glm::vec3 offsetDirection = m_evasionOffset / offsetMagnitude;
        m_evasionOffset = offsetDirection * m_evasiveConfig.maxOffset;
        const float outwardVelocity = glm::dot(m_evasionVelocity, offsetDirection);
        if (outwardVelocity > 0.0f)
        {
            m_evasionVelocity -= offsetDirection * outwardVelocity;
        }
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
        const glm::vec3 velocityDirection = normalizeOrFallback(currentVelocity, m_movementDirection);
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
    m_evasionPhase = 0.0f;
    m_evasionOffset = glm::vec3(0.0f);
    m_evasionVelocity = glm::vec3(0.0f);
}
