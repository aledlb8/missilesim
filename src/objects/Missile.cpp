#include "Missile.h"
#include "Flare.h"
#include "Target.h" // Include Target class
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace
{
    constexpr float kDecoyAngularSeparationMinDegrees = 0.15f;
    constexpr float kDecoyAngularSeparationMaxDegrees = 1.6f;
    constexpr float kMinimumVelocityCoherence = 0.05f;

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

        return glm::vec3(0.0f, 0.0f, 1.0f);
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
        const glm::vec3 current = normalizeOrFallback(currentDirection, glm::vec3(0.0f, 0.0f, 1.0f));
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

    glm::vec3 predictInterceptPoint(const glm::vec3 &missilePosition,
                                    float missileSpeed,
                                    const glm::vec3 &targetPosition,
                                    const glm::vec3 &targetVelocity,
                                    float maxLookAheadTime)
    {
        if (missileSpeed < 0.1f)
        {
            return targetPosition;
        }

        const glm::vec3 relativePosition = targetPosition - missilePosition;
        const float missileSpeedSq = missileSpeed * missileSpeed;
        const float a = glm::dot(targetVelocity, targetVelocity) - missileSpeedSq;
        const float b = 2.0f * glm::dot(relativePosition, targetVelocity);
        const float c = glm::dot(relativePosition, relativePosition);
        const float epsilon = 0.0001f;

        float timeToIntercept = glm::length(relativePosition) / missileSpeed;
        bool hasInterceptSolution = false;

        if (std::abs(a) < epsilon)
        {
            if (std::abs(b) > epsilon)
            {
                const float candidate = -c / b;
                if (candidate > 0.0f)
                {
                    timeToIntercept = candidate;
                    hasInterceptSolution = true;
                }
            }
        }
        else
        {
            const float discriminant = (b * b) - (4.0f * a * c);
            if (discriminant >= 0.0f)
            {
                const float sqrtDiscriminant = std::sqrt(discriminant);
                const float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
                const float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

                if (t1 > 0.0f && t2 > 0.0f)
                {
                    timeToIntercept = std::min(t1, t2);
                    hasInterceptSolution = true;
                }
                else if (t1 > 0.0f)
                {
                    timeToIntercept = t1;
                    hasInterceptSolution = true;
                }
                else if (t2 > 0.0f)
                {
                    timeToIntercept = t2;
                    hasInterceptSolution = true;
                }
            }
        }

        if (!hasInterceptSolution)
        {
            timeToIntercept = glm::length(relativePosition) / missileSpeed;
        }

        timeToIntercept = glm::clamp(timeToIntercept, 0.0f, std::max(maxLookAheadTime, 0.0f));
        return targetPosition + (targetVelocity * timeToIntercept);
    }

    float computeAngleDegreesFromDot(float dotValue)
    {
        return glm::degrees(std::acos(glm::clamp(dotValue, -1.0f, 1.0f)));
    }
} // namespace

Missile::Missile(const glm::vec3 &position, const glm::vec3 &velocity,
                 float mass, float dragCoefficient, float crossSectionalArea, float liftCoefficient)
    : PhysicsObject(position, velocity, std::max(mass, 0.01f)),
      m_dragCoefficient(dragCoefficient),
      m_crossSectionalArea(crossSectionalArea),
      m_liftCoefficient(liftCoefficient),
      m_dryMass(std::max(mass, 0.01f))
{
    synchronizeMass();
}

void Missile::setMass(float mass)
{
    m_dryMass = std::max(mass, 0.01f);
    synchronizeMass();
}

void Missile::setTrackingAngle(float angleDegrees)
{
    if (std::isnan(angleDegrees) || std::isinf(angleDegrees))
    {
        return;
    }

    m_trackingAngleDegrees = glm::clamp(angleDegrees, 5.0f, 180.0f);
}

void Missile::setProximityFuseRadius(float radiusMeters)
{
    if (std::isnan(radiusMeters) || std::isinf(radiusMeters))
    {
        return;
    }

    m_proximityFuseRadius = std::max(radiusMeters, 0.0f);
}

void Missile::setCountermeasureResistance(float resistance)
{
    if (std::isnan(resistance) || std::isinf(resistance))
    {
        return;
    }

    m_countermeasureResistance = glm::clamp(resistance, 0.0f, 1.0f);
}

void Missile::setFuel(float kg)
{
    m_fuel = std::max(kg, 0.0f);
    if (m_fuel <= 0.0f)
    {
        m_thrustEnabled = false;
    }
    synchronizeMass();
}

void Missile::synchronizeMass()
{
    m_mass = std::max(m_dryMass + m_fuel, 0.01f);
}

void Missile::setTarget(const glm::vec3 &targetPosition)
{
    m_hasTarget = true;
    m_targetPosition = targetPosition;
    m_trackedSourceVelocity = glm::vec3(0.0f);
    m_targetObject = nullptr;
    m_trackedFlare = nullptr;
    m_trackingDecoy = false;
    m_selfDestructRequested = false;
}

void Missile::setTargetObject(Target *target)
{
    m_targetObject = target;
    m_hasTarget = (target != nullptr);

    if (m_hasTarget)
    {
        m_targetPosition = target->getPosition();
        m_trackedSourceVelocity = target->getVelocity();
        m_trackedFlare = nullptr;
        m_trackingDecoy = false;
        m_selfDestructRequested = false;
    }
    else
    {
        m_trackedSourceVelocity = glm::vec3(0.0f);
        m_trackedFlare = nullptr;
        m_trackingDecoy = false;
        m_selfDestructRequested = false;
    }
}

void Missile::clearTarget()
{
    m_hasTarget = false;
    m_targetObject = nullptr;
    m_trackedFlare = nullptr;
    m_trackingDecoy = false;
    m_trackedSourceVelocity = glm::vec3(0.0f);
    m_selfDestructRequested = false;
}

void Missile::updateHeatSeeker(const std::vector<Target *> &targets, const std::vector<Flare *> &flares, float deltaTime)
{
    if (!m_guidanceEnabled || !m_thrustEnabled || deltaTime <= 0.0f)
    {
        return;
    }

    const glm::vec3 seekerForward = (!m_thrustEnabled || glm::length2(m_velocity) < 0.0001f)
                                        ? normalizeOrFallback(m_thrustDirection, m_velocity)
                                        : normalizeOrFallback(m_velocity, m_thrustDirection);
    const float seekerCosLimit = std::cos(glm::radians(m_trackingAngleDegrees));
    const float resistanceBlend = glm::clamp(m_countermeasureResistance, 0.0f, 1.0f);

    Target *bestTarget = nullptr;
    float bestTargetScore = -1.0f;
    for (Target *target : targets)
    {
        if (target == nullptr || !target->isActive())
        {
            continue;
        }

        const glm::vec3 targetPosition = target->getPosition();
        const glm::vec3 targetVelocity = target->getVelocity();
        const glm::vec3 relativePosition = targetPosition - m_position;
        const float distanceSq = std::max(glm::dot(relativePosition, relativePosition), 1.0f);
        const glm::vec3 directionToTarget = normalizeOrFallback(relativePosition, seekerForward);
        const float alignment = glm::clamp(glm::dot(seekerForward, directionToTarget), -1.0f, 1.0f);
        if (alignment < seekerCosLimit)
        {
            continue;
        }

        float targetHeat = std::max(target->getHeatSignature(), 0.0f);
        const float targetSpeed = glm::length(targetVelocity);
        if (targetSpeed > 0.5f)
        {
            const glm::vec3 exhaustDirection = -glm::normalize(targetVelocity);
            const glm::vec3 missileFromTarget = normalizeOrFallback(m_position - targetPosition, exhaustDirection);
            const float rearAspect = glm::clamp(glm::dot(missileFromTarget, exhaustDirection), 0.0f, 1.0f);
            targetHeat *= glm::mix(0.55f, 1.0f, rearAspect);
        }

        const float angleWeight = glm::smoothstep(seekerCosLimit, 1.0f, alignment);
        const bool retainingPrimaryTrack = !m_trackingDecoy && target == m_targetObject;
        const float primaryTrackBias = retainingPrimaryTrack
                                           ? (1.0f + m_lockRetentionBias + (resistanceBlend * 1.35f))
                                           : 1.0f;
        const float score = (targetHeat / distanceSq) * angleWeight * primaryTrackBias;
        if (score > bestTargetScore)
        {
            bestTargetScore = score;
            bestTarget = target;
        }
    }

    if (bestTarget == nullptr)
    {
        clearTarget();
        return;
    }

    const glm::vec3 primaryTargetPosition = bestTarget->getPosition();
    const glm::vec3 primaryTargetVelocity = bestTarget->getVelocity();
    const glm::vec3 primaryRelativePosition = primaryTargetPosition - m_position;
    const glm::vec3 primaryDirection = normalizeOrFallback(primaryRelativePosition, seekerForward);
    float bestScore = bestTargetScore;
    float primaryScore = bestTargetScore;
    glm::vec3 bestPosition = primaryTargetPosition;
    glm::vec3 bestVelocity = primaryTargetVelocity;
    const Flare *bestFlare = nullptr;
    bool bestIsDecoy = false;

    m_targetObject = bestTarget;
    m_hasTarget = true;
    m_selfDestructRequested = false;

    for (Flare *flare : flares)
    {
        if (flare == nullptr || !flare->isActive())
        {
            continue;
        }

        const glm::vec3 relativePosition = flare->getPosition() - m_position;
        const float distanceSq = std::max(glm::dot(relativePosition, relativePosition), 1.0f);
        const glm::vec3 directionToFlare = normalizeOrFallback(relativePosition, seekerForward);
        const float alignment = glm::clamp(glm::dot(seekerForward, directionToFlare), -1.0f, 1.0f);
        if (alignment < seekerCosLimit)
        {
            continue;
        }

        float score = flare->getHeatSignature() / distanceSq;
        score *= glm::smoothstep(seekerCosLimit, 1.0f, alignment);

        const float primaryFlareAlignment = glm::clamp(glm::dot(primaryDirection, directionToFlare), 0.0f, 1.0f);
        const float directionalCoherence = glm::mix(0.1f, 1.0f, primaryFlareAlignment);
        const float angularSeparationDegrees = computeAngleDegreesFromDot(primaryFlareAlignment);
        const float separationCoherence = glm::smoothstep(
            kDecoyAngularSeparationMinDegrees,
            kDecoyAngularSeparationMaxDegrees,
            angularSeparationDegrees);
        const float targetSpeed = std::max(glm::length(primaryTargetVelocity), 10.0f);
        const float velocityMismatch = glm::length(flare->getVelocity() - primaryTargetVelocity) / targetSpeed;
        const float velocityCoherence = glm::clamp(
            1.0f - glm::smoothstep(0.08f, 0.45f, velocityMismatch),
            kMinimumVelocityCoherence,
            1.0f);
        const float decoyCoherence = directionalCoherence * separationCoherence * velocityCoherence;
        const float irccmBlend = 0.45f + (0.55f * resistanceBlend);
        score *= glm::mix(1.0f, decoyCoherence, irccmBlend);

        if (!m_trackingDecoy && primaryScore > 0.0f)
        {
            const float switchMargin = glm::mix(1.35f, 2.5f, resistanceBlend);
            if (score <= (primaryScore * switchMargin))
            {
                continue;
            }
        }

        if (m_trackingDecoy && m_trackedFlare == flare)
        {
            score *= 1.0f + m_lockRetentionBias;
        }

        if (score > bestScore)
        {
            bestScore = score;
            bestPosition = flare->getPosition();
            bestVelocity = flare->getVelocity();
            bestFlare = flare;
            bestIsDecoy = true;
        }
    }

    m_targetPosition = bestPosition;
    m_trackedSourceVelocity = bestVelocity;
    m_trackedFlare = bestFlare;
    m_trackingDecoy = bestIsDecoy;
}

bool Missile::consumeSelfDestructRequest()
{
    const bool requested = m_selfDestructRequested;
    m_selfDestructRequested = false;
    return requested;
}

void Missile::applyGuidance(float deltaTime)
{
    // Only apply guidance if enabled and target exists
    if (!m_guidanceEnabled || !m_hasTarget)
    {
        return;
    }

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        return;
    }

    try
    {
        glm::vec3 targetVelocity = m_trackedSourceVelocity;

        if (m_targetObject != nullptr && m_targetObject->isActive())
        {
            if (!m_trackingDecoy)
            {
                m_targetPosition = m_targetObject->getPosition();
                targetVelocity = m_targetObject->getVelocity();
            }
        }
        else if (m_targetObject != nullptr)
        {
            clearTarget();
            return;
        }

        // Safety check for invalid target position (NaN or infinity)
        if (std::isnan(m_targetPosition.x) || std::isnan(m_targetPosition.y) || std::isnan(m_targetPosition.z) ||
            std::isinf(m_targetPosition.x) || std::isinf(m_targetPosition.y) || std::isinf(m_targetPosition.z))
        {
            std::cerr << "Error: Invalid target position detected (NaN or infinity)" << std::endl;
            m_hasTarget = false;
            m_guidanceEnabled = false;
            return;
        }

        // Check if our own position is valid
        if (std::isnan(m_position.x) || std::isnan(m_position.y) || std::isnan(m_position.z) ||
            std::isinf(m_position.x) || std::isinf(m_position.y) || std::isinf(m_position.z))
        {
            std::cerr << "Error: Invalid missile position detected (NaN or infinity)" << std::endl;
            m_hasTarget = false;
            m_guidanceEnabled = false;
            return;
        }

        const glm::vec3 currentDirection = normalizeOrFallback(m_velocity, m_thrustDirection);
        const float speed = glm::length(m_velocity);
        if (speed < 0.1f)
        {
            if (m_thrustEnabled)
            {
                const glm::vec3 boostDirection = normalizeOrFallback(m_targetPosition - m_position, currentDirection);
                m_thrustDirection = boostDirection;
            }
            return;
        }

        const glm::vec3 relativePosition = m_targetPosition - m_position;
        const float distanceToTarget = glm::length(relativePosition);
        if (distanceToTarget < 0.5f)
        {
            return;
        }

        const glm::vec3 lineOfSight = relativePosition / distanceToTarget;
        const glm::vec3 interceptPoint = predictInterceptPoint(m_position, speed, m_targetPosition, targetVelocity, 20.0f);
        const glm::vec3 interceptDirection = normalizeOrFallback(interceptPoint - m_position, lineOfSight);
        const float leadBlend = glm::clamp((m_navigationGain - 1.0f) / 3.0f, 0.0f, 1.0f);
        glm::vec3 desiredDirection = normalizeOrFallback(
            glm::mix(lineOfSight, interceptDirection, leadBlend),
            interceptDirection);

        const float seekerAlignment = glm::clamp(glm::dot(currentDirection, interceptDirection), -1.0f, 1.0f);
        const float seekerAngleDegrees = glm::degrees(std::acos(seekerAlignment));
        if (seekerAngleDegrees > m_trackingAngleDegrees)
        {
            m_selfDestructRequested = true;
            return;
        }

        if (m_terrainAvoidanceEnabled)
        {
            const float currentClearance = m_position.y - m_groundReferenceAltitude;
            const float predictedClearance = currentClearance + (desiredDirection.y * speed * m_terrainLookAheadTime);

            if (predictedClearance < m_terrainClearance)
            {
                const float deficit = m_terrainClearance - predictedClearance;
                const float terrainBlend = glm::clamp(deficit / std::max(m_terrainClearance + 20.0f, 20.0f), 0.0f, 1.0f);
                const glm::vec3 terrainForward = normalizeOrFallback(
                    glm::vec3(desiredDirection.x, 0.0f, desiredDirection.z),
                    glm::vec3(currentDirection.x, 0.0f, currentDirection.z));
                const glm::vec3 climbDirection = normalizeOrFallback(
                    glm::vec3(terrainForward.x, glm::mix(0.35f, 1.4f, terrainBlend), terrainForward.z),
                    glm::vec3(0.0f, 1.0f, 0.0f));
                desiredDirection = normalizeOrFallback(
                    glm::mix(desiredDirection, climbDirection, 0.3f + (terrainBlend * 0.6f)),
                    climbDirection);
            }
        }

        const float maxLateralAcceleration = std::max(m_maxSteeringForce / std::max(m_mass, 0.01f), 0.0f);
        if (maxLateralAcceleration <= 0.0f)
        {
            return;
        }

        const float maxTurnRate = maxLateralAcceleration / std::max(speed, 0.1f);
        const glm::vec3 steeredDirection = rotateTowardsDirection(currentDirection, desiredDirection, maxTurnRate * deltaTime);

        glm::vec3 commandedAcceleration = ((steeredDirection - currentDirection) * speed) / deltaTime;
        commandedAcceleration -= currentDirection * glm::dot(commandedAcceleration, currentDirection);

        const float commandedAccelerationMagnitude = glm::length(commandedAcceleration);
        if (commandedAccelerationMagnitude > maxLateralAcceleration)
        {
            commandedAcceleration = (commandedAcceleration / commandedAccelerationMagnitude) * maxLateralAcceleration;
        }

        applyForce(commandedAcceleration * m_mass);

        if (m_thrustEnabled)
        {
            m_thrustDirection = steeredDirection;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in missile guidance: " << e.what() << std::endl;
        m_guidanceEnabled = false;
    }
    catch (...)
    {
        std::cerr << "Unknown error in missile guidance" << std::endl;
        m_guidanceEnabled = false;
    }
}

bool Missile::applyThrust(float deltaTime)
{
    // Check if thrust is enabled and we have fuel
    if (!m_thrustEnabled || m_fuel <= 0.0f || m_thrust <= 0.0f)
    {
        if (m_fuel <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
            synchronizeMass();
        }
        return false;
    }

    try
    {
        if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime) || m_fuelConsumptionRate <= 0.0f)
        {
            return false;
        }

        // Safety checks for valid thrust direction
        if (glm::length2(m_thrustDirection) < 0.001f ||
            std::isnan(m_thrustDirection.x) || std::isnan(m_thrustDirection.y) || std::isnan(m_thrustDirection.z) ||
            std::isinf(m_thrustDirection.x) || std::isinf(m_thrustDirection.y) || std::isinf(m_thrustDirection.z))
        {
            // Default to current velocity direction, or forward if velocity is too small
            if (glm::length2(m_velocity) > 0.001f)
            {
                m_thrustDirection = glm::normalize(m_velocity);
            }
            else
            {
                m_thrustDirection = glm::vec3(0.0f, 0.0f, 1.0f); // Default forward direction
            }
        }

        // Calculate fuel consumption for this step
        const float requestedFuel = m_fuelConsumptionRate * deltaTime;
        if (requestedFuel <= 0.0f)
        {
            return false;
        }

        // Limit consumption to available fuel
        const float fuelConsumed = std::min(requestedFuel, m_fuel);
        const float throttleFraction = glm::clamp(fuelConsumed / requestedFuel, 0.0f, 1.0f);
        if (throttleFraction <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
            synchronizeMass();
            return false;
        }

        // Calculate thrust force - proportional to fuel consumption
        const float thrustMagnitude = m_thrust * throttleFraction;

        // Apply thrust force in the thrust direction
        glm::vec3 thrustForce = m_thrustDirection * thrustMagnitude;
        applyForce(thrustForce);

        // Update remaining fuel and wet mass after the burn.
        m_fuel -= fuelConsumed;
        if (m_fuel <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
        }
        synchronizeMass();

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error applying thrust: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown error applying thrust" << std::endl;
        return false;
    }
}

void Missile::update(float deltaTime)
{
    // applyGuidance(deltaTime);

    // Apply thrust if enabled
    applyThrust(deltaTime);

    // Continue with normal physics update
    PhysicsObject::update(deltaTime);

    // When guidance is inactive, keep thrust aligned with the current flight path.
    if ((!m_guidanceEnabled || !m_hasTarget || !m_thrustEnabled) && glm::length2(m_velocity) > 0.001f)
    {
        m_thrustDirection = glm::normalize(m_velocity);
    }
}
