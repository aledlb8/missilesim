#include "Missile.h"
#include "Target.h" // Include Target class
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>

namespace
{
float computeCloseRangeBlend(float distance, float fullAuthorityDistance, float lowAuthorityDistance)
{
    if (lowAuthorityDistance <= fullAuthorityDistance)
    {
        return distance <= fullAuthorityDistance ? 1.0f : 0.0f;
    }

    return 1.0f - glm::smoothstep(fullAuthorityDistance, lowAuthorityDistance, distance);
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

    return glm::vec3(0.0f, 0.0f, 1.0f);
}

float computeTerrainUrgency(float deficit, float recoveryWindow)
{
    if (recoveryWindow <= 0.0f)
    {
        return 0.0f;
    }

    return glm::clamp(deficit / recoveryWindow, 0.0f, 1.0f);
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
    const bool targetChanged = (m_targetObject != nullptr) || !m_hasTarget ||
                               (glm::length2(m_targetPosition - targetPosition) > 0.0001f);
    m_hasTarget = true;
    m_targetPosition = targetPosition;
    m_targetObject = nullptr;
    if (targetChanged || !m_targetTrackInitialized)
    {
        initializeTargetTrack(targetPosition, glm::vec3(0.0f));
    }
}

void Missile::setTargetObject(Target *target)
{
    const bool targetChanged = (m_targetObject != target);
    m_targetObject = target;
    m_hasTarget = (target != nullptr);

    if (m_hasTarget)
    {
        m_targetPosition = target->getPosition();
        if (targetChanged || !m_targetTrackInitialized)
        {
            initializeTargetTrack(m_targetPosition, target->getVelocity());
        }
    }
    else
    {
        resetTargetTrack();
    }
}

void Missile::clearTarget()
{
    m_hasTarget = false;
    m_targetObject = nullptr;
    resetTargetTrack();
}

void Missile::initializeTargetTrack(const glm::vec3 &position, const glm::vec3 &velocity)
{
    m_filteredTargetPosition = position;
    m_filteredTargetVelocity = velocity;
    m_targetTrackInitialized = true;
}

void Missile::resetTargetTrack()
{
    m_filteredTargetPosition = glm::vec3(0.0f);
    m_filteredTargetVelocity = glm::vec3(0.0f);
    m_targetTrackInitialized = false;
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
        glm::vec3 targetVelocity(0.0f);

        // If we have a target object, update the target position
        if (m_targetObject != nullptr && m_targetObject->isActive())
        {
            // Always update target position to enable continuous tracking
            m_targetPosition = m_targetObject->getPosition();
            targetVelocity = m_targetObject->getVelocity();
        }
        else if (m_targetObject != nullptr)
        {
            // If target is no longer active, clear it
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

        const float speed = glm::length(m_velocity);
        if (speed < 0.1f)
        {
            return;
        }

        glm::vec3 relativePosition = m_targetPosition - m_position;
        float distanceToTarget = glm::length(relativePosition);

        // If very close to target, no need for guidance
        if (distanceToTarget < 1.0f)
        {
            return;
        }

        if (!m_targetTrackInitialized)
        {
            initializeTargetTrack(m_targetPosition, targetVelocity);
        }

        const float closeRangeBlend = computeCloseRangeBlend(distanceToTarget, 1500.0f, 8500.0f);
        const float terminalBlend = computeCloseRangeBlend(distanceToTarget, 350.0f, 2200.0f);
        const float smoothingTimeConstant = glm::mix(1.6f, 0.08f, closeRangeBlend);
        const float trackAlpha = glm::clamp(1.0f - std::exp(-deltaTime / std::max(smoothingTimeConstant, 0.01f)), 0.02f, 1.0f);

        m_filteredTargetPosition = glm::mix(m_filteredTargetPosition, m_targetPosition, trackAlpha);
        m_filteredTargetVelocity = glm::mix(m_filteredTargetVelocity, targetVelocity, trackAlpha);

        relativePosition = m_filteredTargetPosition - m_position;
        distanceToTarget = glm::length(relativePosition);
        if (distanceToTarget < 1.0f)
        {
            return;
        }

        glm::vec3 lineOfSight = relativePosition / distanceToTarget;
        glm::vec3 velocityDirection = m_velocity / speed;
        glm::vec3 relativeVelocity = m_filteredTargetVelocity - m_velocity;

        const float maxLateralAcceleration = m_maxSteeringForce / std::max(m_mass, 1.0f);
        const float allowedLateralAcceleration = maxLateralAcceleration * glm::mix(0.35f, 1.0f, closeRangeBlend);
        const float scheduledNavigationGain = m_navigationGain * glm::mix(0.55f, 1.0f, closeRangeBlend);

        // Range-scheduled PN: stay calm in midcourse, then tighten up as the intercept develops.
        const float rangeSq = std::max(glm::dot(relativePosition, relativePosition), 1.0f);
        const float closingSpeed = std::max(-glm::dot(relativeVelocity, lineOfSight), 0.0f);
        const glm::vec3 lineOfSightRate = glm::cross(relativePosition, relativeVelocity) / rangeSq;
        glm::vec3 commandedAcceleration = scheduledNavigationGain * closingSpeed * glm::cross(lineOfSightRate, velocityDirection);

        // Use a filtered lead point and cap look-ahead at long range so target jitter does not whip the missile around.
        glm::vec3 targetPredictedPosition = m_filteredTargetPosition;
        float timeToIntercept = distanceToTarget / speed;
        bool hasInterceptSolution = false;
        const float missileSpeedSq = speed * speed;
        const float a = glm::dot(m_filteredTargetVelocity, m_filteredTargetVelocity) - missileSpeedSq;
        const float b = 2.0f * glm::dot(relativePosition, m_filteredTargetVelocity);
        const float c = glm::dot(relativePosition, relativePosition);
        const float epsilon = 0.0001f;

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
            timeToIntercept = distanceToTarget / speed;
        }

        const float maxLeadLookAhead = glm::mix(5.0f, 18.0f, closeRangeBlend);
        timeToIntercept = glm::clamp(timeToIntercept, 0.0f, maxLeadLookAhead);
        targetPredictedPosition = m_filteredTargetPosition + m_filteredTargetVelocity * timeToIntercept;

        glm::vec3 leadDirection = normalizeOrFallback(targetPredictedPosition - m_position, lineOfSight);
        const float midcourseLeadBlend = glm::mix(0.14f, 0.9f, terminalBlend);
        const glm::vec3 horizontalLineOfSight = normalizeOrFallback(glm::vec3(lineOfSight.x, 0.0f, lineOfSight.z), lineOfSight);
        const glm::vec3 horizontalLeadDirection = normalizeOrFallback(glm::vec3(leadDirection.x, 0.0f, leadDirection.z), horizontalLineOfSight);
        const glm::vec3 horizontalGuidanceDirection = normalizeOrFallback((horizontalLineOfSight * (1.0f - midcourseLeadBlend)) +
                                                                              (horizontalLeadDirection * midcourseLeadBlend),
                                                                          horizontalLineOfSight);
        const glm::vec3 fullGuidanceDirection = normalizeOrFallback((lineOfSight * (1.0f - midcourseLeadBlend)) +
                                                                        (leadDirection * midcourseLeadBlend),
                                                                    lineOfSight);
        const float altitudeError = std::abs(m_filteredTargetPosition.y - m_position.y);
        const float altitudeUrgency = glm::clamp(altitudeError / std::max(distanceToTarget * 0.2f, 75.0f), 0.0f, 1.0f);
        const float verticalGuidanceBlend = std::max(glm::mix(0.08f, 1.0f, terminalBlend), altitudeUrgency);
        glm::vec3 desiredGuidanceDirection = normalizeOrFallback((horizontalGuidanceDirection * (1.0f - verticalGuidanceBlend)) +
                                                                     (fullGuidanceDirection * verticalGuidanceBlend),
                                                                 horizontalGuidanceDirection);

        if (m_terrainAvoidanceEnabled)
        {
            const float currentClearance = m_position.y - m_groundReferenceAltitude;
            const float targetClearance = std::max(m_filteredTargetPosition.y - m_groundReferenceAltitude, 0.0f);
            const float terminalClearance = std::max(targetClearance + 4.0f, 2.0f);
            const float scheduledClearance = glm::mix(m_terrainClearance, terminalClearance, terminalBlend);
            const float terrainLookAheadTime = glm::clamp(glm::mix(m_terrainLookAheadTime, 0.75f, terminalBlend), 0.5f, m_terrainLookAheadTime);
            const float projectedVerticalSpeed = glm::mix(m_velocity.y, desiredGuidanceDirection.y * speed, 0.65f);
            const float predictedClearance = currentClearance + (projectedVerticalSpeed * terrainLookAheadTime);
            const float descentPenalty = std::max(-m_velocity.y, 0.0f) * glm::mix(0.7f, 0.2f, terminalBlend);
            const float immediateDeficit = scheduledClearance - currentClearance;
            const float projectedDeficit = (scheduledClearance + descentPenalty) - predictedClearance;
            const float recoveryWindow = std::max(scheduledClearance * 0.8f, 25.0f);
            const float terrainUrgency = std::max(
                computeTerrainUrgency(immediateDeficit, recoveryWindow),
                computeTerrainUrgency(projectedDeficit, recoveryWindow));

            float timeToGround = std::numeric_limits<float>::infinity();
            if (m_velocity.y < -0.5f && currentClearance > 0.0f)
            {
                timeToGround = currentClearance / -m_velocity.y;
            }

            const float timeUrgency = std::isfinite(timeToGround)
                                          ? (1.0f - glm::smoothstep(0.8f, terrainLookAheadTime * 1.5f, timeToGround))
                                          : 0.0f;
            const float terrainAvoidanceBlend = std::max(terrainUrgency, timeUrgency);

            if (terrainAvoidanceBlend > 0.0f)
            {
                const glm::vec3 terrainForward = normalizeOrFallback(
                    glm::vec3(horizontalGuidanceDirection.x, 0.0f, horizontalGuidanceDirection.z),
                    glm::vec3(velocityDirection.x, 0.0f, velocityDirection.z));
                const float climbBias = glm::mix(0.25f, 1.85f, terrainAvoidanceBlend);
                const glm::vec3 terrainSafeDirection = normalizeOrFallback(
                    glm::vec3(terrainForward.x, climbBias, terrainForward.z),
                    glm::vec3(0.0f, 1.0f, 0.0f));
                const float terrainBlend = glm::mix(0.22f, 0.9f, terrainAvoidanceBlend);
                desiredGuidanceDirection = normalizeOrFallback(
                    glm::mix(desiredGuidanceDirection, terrainSafeDirection, terrainBlend),
                    terrainSafeDirection);
                commandedAcceleration += glm::vec3(
                    0.0f,
                    allowedLateralAcceleration * glm::mix(0.2f, 1.0f, terrainAvoidanceBlend),
                    0.0f);
            }
        }

        glm::vec3 guidanceLateral = desiredGuidanceDirection - (velocityDirection * glm::dot(velocityDirection, desiredGuidanceDirection));
        if (glm::length2(guidanceLateral) > 0.0001f)
        {
            guidanceLateral = glm::normalize(guidanceLateral);
            const float guidanceError = std::acos(glm::clamp(glm::dot(velocityDirection, desiredGuidanceDirection), -1.0f, 1.0f));
            const float guidanceDeadZone = glm::mix(glm::radians(0.2f), glm::radians(0.05f), closeRangeBlend);
            if (guidanceError > guidanceDeadZone)
            {
                const float pursuitError = guidanceError - guidanceDeadZone;
                const float pursuitAcceleration = std::min(speed * pursuitError * glm::mix(1.1f, 2.2f, terminalBlend),
                                                           maxLateralAcceleration * glm::mix(0.28f, 0.55f, closeRangeBlend));
                commandedAcceleration += guidanceLateral * pursuitAcceleration;
            }
        }

        // Clamp lateral acceleration to the airframe's turn authority.
        const float commandedAccelerationMagnitude = glm::length(commandedAcceleration);
        if (commandedAccelerationMagnitude > allowedLateralAcceleration)
        {
            commandedAcceleration = (commandedAcceleration / commandedAccelerationMagnitude) * allowedLateralAcceleration;
        }

        applyForce(commandedAcceleration * m_mass);

        // Preserve energy by keeping thrust mostly along the velocity vector and only biasing toward lead.
        if (m_thrustEnabled)
        {
            const float alignment = glm::clamp(glm::dot(velocityDirection, desiredGuidanceDirection), -1.0f, 1.0f);
            const float turnDemand = 1.0f - std::max(alignment, 0.0f);
            const float leadBlendCap = glm::mix(0.12f, 0.4f, terminalBlend);
            const float desiredLeadBlend = (speed < 25.0f) ? 1.0f : glm::clamp(turnDemand * leadBlendCap, 0.0f, leadBlendCap);
            glm::vec3 desiredThrustDirection = (velocityDirection * (1.0f - desiredLeadBlend)) + (desiredGuidanceDirection * desiredLeadBlend);

            if (glm::length2(desiredThrustDirection) > 0.0001f)
            {
                desiredThrustDirection = glm::normalize(desiredThrustDirection);
                glm::vec3 currentThrustDirection = (glm::length2(m_thrustDirection) > 0.0001f) ? glm::normalize(m_thrustDirection) : velocityDirection;
                const float thrustTurnBlend = glm::clamp((deltaTime * glm::mix(1.4f, 3.0f, terminalBlend)) +
                                                             (turnDemand * glm::mix(0.03f, 0.1f, terminalBlend)),
                                                         0.0f, glm::mix(0.12f, 0.4f, terminalBlend));
                glm::vec3 blendedThrustDirection = (currentThrustDirection * (1.0f - thrustTurnBlend)) + (desiredThrustDirection * thrustTurnBlend);

                if (glm::length2(blendedThrustDirection) > 0.0001f)
                {
                    m_thrustDirection = glm::normalize(blendedThrustDirection);
                }
            }
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
