#include "Target.h"
#include <glm/gtx/norm.hpp>
#include <iostream>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <random>

Target::Target(const glm::vec3& position, float radius)
    : PhysicsObject(position, glm::vec3(0.0f), 0.0f)  // Targets don't move or have mass
    , m_radius(radius) {
    // Store the initial position for movement patterns
    m_initialPosition = position;
    m_movementCenter = position;
    
    // Validate position
    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z)) {
        std::cerr << "Error: Invalid target position detected during creation" << std::endl;
        // Use a default valid position instead
        setPosition(glm::vec3(0.0f, 100.0f, 0.0f));
    }
    
    // Validate radius
    if (radius <= 0.0f || std::isnan(radius) || std::isinf(radius)) {
        std::cerr << "Error: Invalid target radius detected during creation" << std::endl;
        m_radius = 5.0f; // Use a default valid radius
    }

    enforceAirspaceConstraint();
}

bool Target::isPointInside(const glm::vec3& point) const {
    // Validate point
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
        std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z)) {
        std::cerr << "Error: Invalid point coordinates in isPointInside check" << std::endl;
        return false;
    }
    
    // Validate position
    if (std::isnan(m_position.x) || std::isnan(m_position.y) || std::isnan(m_position.z) ||
        std::isinf(m_position.x) || std::isinf(m_position.y) || std::isinf(m_position.z)) {
        std::cerr << "Error: Invalid target position in isPointInside check" << std::endl;
        return false;
    }
    
    try {
        // Calculate distance from point to center
        float distance = glm::length(point - m_position);
        
        // Check if distance is less than radius
        return distance <= m_radius;
    }
    catch (...) {
        std::cerr << "Error: Exception during distance calculation in isPointInside" << std::endl;
        return false;
    }
}

void Target::update(float deltaTime)
{
    // Scripted targets manage their own kinematics instead of integrating forces.
    m_forces = glm::vec3(0.0f);
    m_acceleration = glm::vec3(0.0f);

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        m_velocity = glm::vec3(0.0f);
        return;
    }

    // Skip movement if target is not active or pattern is stationary.
    if (!m_isActive || m_movementPattern == TargetMovementPattern::STATIONARY) {
        m_velocity = glm::vec3(0.0f);
        return;
    }

    const glm::vec3 previousPosition = m_position;

    // Update movement time
    m_movementTime += deltaTime;

    // Apply the appropriate movement pattern
    switch (m_movementPattern) {
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
            
        default:
            // No movement for STATIONARY or undefined patterns
            break;
    }

    enforceAirspaceConstraint();

    m_velocity = (m_position - previousPosition) / deltaTime;
}

void Target::applyLinearMovement(float deltaTime)
{
    const float minimumAltitude = std::max(m_radius + 2.0f, 5.0f);

    // Calculate velocity based on direction and speed
    glm::vec3 velocity = m_movementDirection * m_movementSpeed;
    
    // Update position
    m_position += velocity * deltaTime;
    
    // Bounce at boundaries (set to +/- m_movementAmplitude from center)
    for (int i = 0; i < 3; i++) {
        if (m_position[i] > m_movementCenter[i] + m_movementAmplitude) {
            m_position[i] = m_movementCenter[i] + m_movementAmplitude;
            m_movementDirection[i] = -m_movementDirection[i]; // Reverse direction
        }
        else {
            float lowerBound = m_movementCenter[i] - m_movementAmplitude;
            if (i == 1) {
                lowerBound = std::max(lowerBound, minimumAltitude);
            }

            if (m_position[i] < lowerBound) {
                m_position[i] = lowerBound;
                m_movementDirection[i] = -m_movementDirection[i]; // Reverse direction
            }
        }
    }
}

void Target::applyCircularMovement(float deltaTime)
{
    // Calculate angle based on time and period (in radians)
    float angle = (m_movementTime / m_movementPeriod) * 2.0f * glm::pi<float>();
    
    // Calculate new position on circle
    m_position.x = m_movementCenter.x + m_movementAmplitude * std::cos(angle);
    m_position.z = m_movementCenter.z + m_movementAmplitude * std::sin(angle);
    
    // Height can oscillate for added complexity
    m_position.y = m_movementCenter.y + (m_movementAmplitude * 0.2f) * std::sin(angle * 2.0f);
}

void Target::applySinusoidalMovement(float deltaTime)
{
    // Calculate normalized time in the wave cycle (0 to 1)
    float t = std::fmod(m_movementTime, m_movementPeriod) / m_movementPeriod;
    
    // Convert to angle (0 to 2π)
    float angle = t * 2.0f * glm::pi<float>();
    
    // Create a perpendicular vector to movement direction for the sine wave
    glm::vec3 cross;
    
    // Choose an appropriate cross vector to ensure perpendicular direction
    if (std::abs(m_movementDirection.y) < 0.9f) {
        cross = glm::normalize(glm::cross(m_movementDirection, glm::vec3(0.0f, 1.0f, 0.0f)));
    } else {
        cross = glm::normalize(glm::cross(m_movementDirection, glm::vec3(1.0f, 0.0f, 0.0f)));
    }
    
    // Calculate progress along the main direction
    float linearProgress = m_movementSpeed * m_movementTime;
    
    // Apply sine wave offset perpendicular to the movement direction
    float sineOffset = m_movementAmplitude * std::sin(angle);
    
    // Set position
    m_position = m_initialPosition + 
                 (m_movementDirection * linearProgress) + 
                 (cross * sineOffset);
    
    // Reset position if we've gone too far
    if (glm::length(m_position - m_initialPosition) > m_movementAmplitude * 10.0f) {
        m_position = m_initialPosition;
        m_movementTime = 0.0f;
    }
}

void Target::applyRandomMovement(float deltaTime)
{
    const float minimumAltitude = std::max(m_radius + 2.0f, 5.0f);

    // Change direction randomly
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(-1.0f, 1.0f);
    
    // Only change direction occasionally for smoother movement
    if (m_movementTime >= m_movementPeriod) {
        // Reset timer
        m_movementTime = 0.0f;
        
        // Set a new random direction
        glm::vec3 newDirection;
        newDirection.x = dis(gen);
        newDirection.y = dis(gen) * 0.5f;  // Less vertical movement
        newDirection.z = dis(gen);
        
        m_movementDirection = glm::normalize(newDirection);
    }
    
    // Move in the current direction
    m_position += m_movementDirection * m_movementSpeed * deltaTime;
    
    // Constrain within bounds
    for (int i = 0; i < 3; i++) {
        if (m_position[i] > m_movementCenter[i] + m_movementAmplitude) {
            m_position[i] = m_movementCenter[i] + m_movementAmplitude;
            m_movementDirection[i] = -std::abs(m_movementDirection[i]); // Reverse direction
        }
        else {
            float lowerBound = m_movementCenter[i] - m_movementAmplitude;
            if (i == 1) {
                lowerBound = std::max(lowerBound, minimumAltitude);
            }

            if (m_position[i] < lowerBound) {
                m_position[i] = lowerBound;
                m_movementDirection[i] = std::abs(m_movementDirection[i]); // Reverse direction
            }
        }
    }
}

void Target::enforceAirspaceConstraint()
{
    const float minimumAltitude = std::max(m_radius + 2.0f, 5.0f);

    if (m_position.y < minimumAltitude) {
        m_position.y = minimumAltitude;
    }

    if (m_initialPosition.y < minimumAltitude) {
        m_initialPosition.y = minimumAltitude;
    }

    if (m_movementCenter.y < minimumAltitude) {
        m_movementCenter.y = minimumAltitude;
    }

    if (m_movementDirection.y < 0.0f && m_position.y <= minimumAltitude + 0.01f) {
        m_movementDirection.y = std::abs(m_movementDirection.y);

        if (glm::length2(m_movementDirection) > 0.0001f) {
            m_movementDirection = glm::normalize(m_movementDirection);
        } else {
            m_movementDirection = glm::vec3(1.0f, 0.1f, 0.0f);
        }
    }
}
