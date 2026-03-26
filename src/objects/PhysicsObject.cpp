#include "PhysicsObject.h"
#include <algorithm>
#include <cmath>
#include <glm/gtx/norm.hpp>

PhysicsObject::PhysicsObject(const glm::vec3 &position, const glm::vec3 &velocity, float mass)
    : m_previousPosition(position), m_position(position), m_velocity(velocity), m_acceleration(0.0f), m_forces(0.0f), m_mass(mass)
{
}

void PhysicsObject::update(float deltaTime)
{
    m_previousPosition = m_position;

    // Calculate acceleration from forces (F = ma -> a = F/m)
    // Protect against division by zero
    if (m_mass > 0.0f)
    {
        m_acceleration = m_forces / m_mass;

        // Check for NaN or infinity in acceleration
        if (std::isnan(m_acceleration.x) || std::isnan(m_acceleration.y) || std::isnan(m_acceleration.z) ||
            std::isinf(m_acceleration.x) || std::isinf(m_acceleration.y) || std::isinf(m_acceleration.z))
        {
            // Reset to prevent instability
            m_acceleration = glm::vec3(0.0f);
            m_forces = glm::vec3(0.0f);
        }

        // Limit acceleration magnitude to prevent simulation instability
        float accelMagnitude = glm::length(m_acceleration);
        if (accelMagnitude > 1000.0f)
        { // Set a reasonable maximum acceleration
            m_acceleration = glm::normalize(m_acceleration) * 1000.0f;
        }
    }
    else
    {
        m_acceleration = glm::vec3(0.0f);
    }

    // Update velocity (v = v0 + a*t)
    m_velocity += m_acceleration * deltaTime;

    // Check for NaN or infinity in velocity
    if (std::isnan(m_velocity.x) || std::isnan(m_velocity.y) || std::isnan(m_velocity.z) ||
        std::isinf(m_velocity.x) || std::isinf(m_velocity.y) || std::isinf(m_velocity.z))
    {
        // Reset to prevent instability
        m_velocity = glm::vec3(0.0f, 0.0f, 0.1f); // Small default velocity
    }

    // Limit velocity to prevent excessive speeds
    float velocityMagnitude = glm::length(m_velocity);
    if (velocityMagnitude > 2000.0f)
    { // Set a reasonable maximum velocity
        m_velocity = glm::normalize(m_velocity) * 2000.0f;
    }

    // Update position (p = p0 + v*t)
    m_position += m_velocity * deltaTime;

    // Check for NaN or infinity in position
    if (std::isnan(m_position.x) || std::isnan(m_position.y) || std::isnan(m_position.z) ||
        std::isinf(m_position.x) || std::isinf(m_position.y) || std::isinf(m_position.z))
    {
        // Reset to prevent instability
        m_position = glm::vec3(0.0f);
    }
}

void PhysicsObject::applyForce(const glm::vec3 &force)
{
    // Validate the force vector before applying
    if (std::isnan(force.x) || std::isnan(force.y) || std::isnan(force.z) ||
        std::isinf(force.x) || std::isinf(force.y) || std::isinf(force.z))
    {
        // Invalid force, don't apply it
        return;
    }

    // Check if force magnitude is excessive
    float forceMagnitude = glm::length(force);
    const float maxSingleForce = std::max(50000.0f, m_mass * 500.0f);
    const float maxTotalForce = std::max(100000.0f, m_mass * 1000.0f);

    if (forceMagnitude > maxSingleForce)
    { // Set a reasonable maximum force
        // Apply a capped force to prevent instability
        glm::vec3 normalizedForce = force / forceMagnitude; // Manual normalization
        m_forces += normalizedForce * maxSingleForce;
    }
    else
    {
        // Add the force to accumulated forces
        m_forces += force;
    }

    // Final safety check on total accumulated forces
    float totalForceMagnitude = glm::length(m_forces);
    if (totalForceMagnitude > maxTotalForce)
    { // Set a reasonable maximum accumulated force
        m_forces = glm::normalize(m_forces) * maxTotalForce;
    }
}

void PhysicsObject::resetForces()
{
    // Reset accumulated forces at the beginning of each frame
    m_forces = glm::vec3(0.0f);
}
