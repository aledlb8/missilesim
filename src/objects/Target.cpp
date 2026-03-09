#include "Target.h"
#include <glm/gtx/norm.hpp>
#include <iostream>
#include <glm/gtc/constants.hpp>
#include <cmath>
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
} // namespace

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
    (void)deltaTime;

    const float radius = std::max(m_movementAmplitude, kMinimumMovementRadiusMeters);
    const float requestedAngularRate = (2.0f * glm::pi<float>()) / std::max(m_movementPeriod, kMinimumMovementPeriodSeconds);

    // The orbit includes a small vertical weave, so cap the angular rate against the 3D path length.
    const float verticalAmplitude = radius * 0.2f;
    const float maxPathDerivativeMagnitude = std::sqrt((radius * radius) + ((2.0f * verticalAmplitude) * (2.0f * verticalAmplitude)));
    const float maxAngularRateFromSpeed = (m_movementSpeed > 0.0f)
                                              ? (m_movementSpeed / std::max(maxPathDerivativeMagnitude, kMinimumMovementRadiusMeters))
                                              : 0.0f;
    const float effectiveAngularRate = std::min(requestedAngularRate, maxAngularRateFromSpeed);
    const float angle = m_movementTime * effectiveAngularRate;

    m_position.x = m_movementCenter.x + radius * std::cos(angle);
    m_position.z = m_movementCenter.z + radius * std::sin(angle);
    m_position.y = m_movementCenter.y + verticalAmplitude * std::sin(angle * 2.0f);
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
    m_position = m_initialPosition +
                 (forwardDirection * m_wavePathDistance) +
                 (lateralDirection * (amplitude * std::sin(angle)));

    // Reset the reference track once the target has traversed far beyond its local engagement pocket.
    if (glm::length(m_position - m_initialPosition) > std::max(amplitude * 10.0f, 250.0f)) {
        m_initialPosition = m_position;
        m_wavePathDistance = 0.0f;
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
