#include "PhysicsObject.h"
#include <algorithm>
#include <cmath>
#include <glm/gtx/norm.hpp>

namespace
{
    constexpr float kStandardGravity = 9.80665f; // m/s^2, for load-factor conversion

    bool isFiniteVec(const glm::vec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
} // namespace

PhysicsObject::PhysicsObject(const glm::vec3 &position, const glm::vec3 &velocity, float mass)
    : m_previousPosition(position), m_position(position), m_velocity(velocity), m_acceleration(0.0f), m_forces(0.0f), m_mass(mass)
{
}

void PhysicsObject::update(float deltaTime)
{
    m_previousPosition = m_position;

    // Calculate acceleration from forces (F = ma -> a = F/m). Stability now
    // comes from sub-stepped integration in the engine rather than arbitrary
    // magnitude clamps, so the only limit applied here is the airframe's real
    // structural load-factor limit (when one is configured).
    if (m_mass > 0.0f)
    {
        m_acceleration = m_forces / m_mass;

        if (!isFiniteVec(m_acceleration))
        {
            m_acceleration = glm::vec3(0.0f);
            m_forces = glm::vec3(0.0f);
        }
        else if (m_maxLoadFactorG > 0.0f)
        {
            const float accelerationLimit = m_maxLoadFactorG * kStandardGravity;
            const float accelMagnitude = glm::length(m_acceleration);
            if (accelMagnitude > accelerationLimit)
            {
                m_acceleration = (m_acceleration / accelMagnitude) * accelerationLimit;
            }
        }
    }
    else
    {
        m_acceleration = glm::vec3(0.0f);
    }

    // Semi-implicit Euler integration.
    m_velocity += m_acceleration * deltaTime;
    if (!isFiniteVec(m_velocity))
    {
        m_velocity = glm::vec3(0.0f, 0.0f, 0.1f); // Recover from a numerical blow-up
    }

    m_position += m_velocity * deltaTime;
    if (!isFiniteVec(m_position))
    {
        m_position = glm::vec3(0.0f);
    }
}

void PhysicsObject::applyForce(const glm::vec3 &force)
{
    // Reject only non-finite forces. Force magnitudes are no longer clamped:
    // each force source is physically bounded (drag by dynamic pressure, thrust
    // by the motor, control by the structural g-limit), and stability is
    // provided by sub-stepped integration in the physics engine.
    if (!isFiniteVec(force))
    {
        return;
    }

    m_forces += force;
}

void PhysicsObject::resetForces()
{
    // Reset accumulated forces at the beginning of each frame
    m_forces = glm::vec3(0.0f);
}