#include "Missile.h"
#include "Target.h" // Include Target class
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <iostream>
#include <cmath>

Missile::Missile(const glm::vec3 &position, const glm::vec3 &velocity,
                 float mass, float dragCoefficient, float crossSectionalArea, float liftCoefficient)
    : PhysicsObject(position, velocity, mass), m_dragCoefficient(dragCoefficient), m_crossSectionalArea(crossSectionalArea), m_liftCoefficient(liftCoefficient)
{
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
            m_hasTarget = false;
            m_targetObject = nullptr;
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

        glm::vec3 lineOfSight = relativePosition / distanceToTarget;
        glm::vec3 velocityDirection = m_velocity / speed;
        glm::vec3 relativeVelocity = targetVelocity - m_velocity;

        // True proportional navigation: command lateral acceleration from LOS rotation rate.
        const float rangeSq = std::max(glm::dot(relativePosition, relativePosition), 1.0f);
        const float closingSpeed = std::max(-glm::dot(relativeVelocity, lineOfSight), 0.0f);
        const glm::vec3 lineOfSightRate = glm::cross(relativePosition, relativeVelocity) / rangeSq;
        glm::vec3 commandedAcceleration = m_navigationGain * closingSpeed * glm::cross(lineOfSightRate, velocityDirection);

        // Solve a lead point as a secondary pursuit term and for thrust shaping.
        glm::vec3 targetPredictedPosition = m_targetPosition;
        float timeToIntercept = distanceToTarget / speed;
        bool hasInterceptSolution = false;
        const float missileSpeedSq = speed * speed;
        const float a = glm::dot(targetVelocity, targetVelocity) - missileSpeedSq;
        const float b = 2.0f * glm::dot(relativePosition, targetVelocity);
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

        timeToIntercept = glm::clamp(timeToIntercept, 0.0f, 5.0f);
        targetPredictedPosition = m_targetPosition + targetVelocity * timeToIntercept;

        glm::vec3 leadDirection = targetPredictedPosition - m_position;
        if (glm::length2(leadDirection) > 0.0001f)
        {
            leadDirection = glm::normalize(leadDirection);
        }
        else
        {
            leadDirection = lineOfSight;
        }

        glm::vec3 leadLateral = leadDirection - (velocityDirection * glm::dot(velocityDirection, leadDirection));
        if (glm::length2(leadLateral) > 0.0001f)
        {
            leadLateral = glm::normalize(leadLateral);
            const float leadError = std::acos(glm::clamp(glm::dot(velocityDirection, leadDirection), -1.0f, 1.0f));
            const float pursuitAcceleration = std::min(speed * leadError * 2.0f, (m_maxSteeringForce / std::max(m_mass, 1.0f)) * 0.35f);
            commandedAcceleration += leadLateral * pursuitAcceleration;
        }

        // Clamp lateral acceleration to the airframe's turn authority.
        const float maxLateralAcceleration = m_maxSteeringForce / std::max(m_mass, 1.0f);
        const float commandedAccelerationMagnitude = glm::length(commandedAcceleration);
        if (commandedAccelerationMagnitude > maxLateralAcceleration)
        {
            commandedAcceleration = (commandedAcceleration / commandedAccelerationMagnitude) * maxLateralAcceleration;
        }

        applyForce(commandedAcceleration * m_mass);

        // Preserve energy by keeping thrust mostly along the velocity vector and only biasing toward lead.
        if (m_thrustEnabled)
        {
            const float alignment = glm::clamp(glm::dot(velocityDirection, leadDirection), -1.0f, 1.0f);
            const float turnDemand = 1.0f - std::max(alignment, 0.0f);
            const float desiredLeadBlend = (speed < 25.0f) ? 1.0f : glm::clamp(turnDemand * 0.35f, 0.0f, 0.35f);
            glm::vec3 desiredThrustDirection = (velocityDirection * (1.0f - desiredLeadBlend)) + (leadDirection * desiredLeadBlend);

            if (glm::length2(desiredThrustDirection) > 0.0001f)
            {
                desiredThrustDirection = glm::normalize(desiredThrustDirection);
                glm::vec3 currentThrustDirection = (glm::length2(m_thrustDirection) > 0.0001f) ? glm::normalize(m_thrustDirection) : velocityDirection;
                const float thrustTurnBlend = glm::clamp((deltaTime * 3.0f) + (turnDemand * 0.1f), 0.0f, 0.4f);
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
    if (!m_thrustEnabled || m_fuel <= 0.0f)
    {
        return false;
    }

    try
    {
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
        float fuelConsumed = m_fuelConsumptionRate * deltaTime;

        // Limit consumption to available fuel
        fuelConsumed = std::min(fuelConsumed, m_fuel);

        // Update remaining fuel
        m_fuel -= fuelConsumed;

        // If fuel runs out, disable thrust
        if (m_fuel <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
            return false;
        }

        // Calculate thrust force - proportional to fuel consumption
        float thrustMagnitude = m_thrust * (fuelConsumed / (m_fuelConsumptionRate * deltaTime));

        // Apply thrust force in the thrust direction
        glm::vec3 thrustForce = m_thrustDirection * thrustMagnitude;
        applyForce(thrustForce);

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
