#pragma once

#include "PhysicsObject.h"
#include <glm/glm.hpp>

enum class TargetMovementPattern {
    STATIONARY,     // Target does not move
    LINEAR,         // Target moves in a straight line
    CIRCULAR,       // Target moves in a circular pattern
    SINUSOIDAL,     // Target moves in a sine wave pattern
    RANDOM          // Target moves randomly
};

class Target : public PhysicsObject {
public:
    Target(const glm::vec3& position = glm::vec3(0.0f), float radius = 5.0f);
    ~Target() override = default;
    
    // Override object type
    std::string getType() const override { return "Target"; }
    
    // Target-specific properties
    float getRadius() const { return m_radius; }
    void setRadius(float radius) { m_radius = radius; }
    
    // Target activation state
    bool isActive() const { return m_isActive; }
    void setActive(bool active) { m_isActive = active; }
    
    // Movement properties
    void setMovementPattern(TargetMovementPattern pattern) { m_movementPattern = pattern; }
    TargetMovementPattern getMovementPattern() const { return m_movementPattern; }
    
    void setMovementSpeed(float speed) { m_movementSpeed = speed; }
    float getMovementSpeed() const { return m_movementSpeed; }
    
    void setMovementAmplitude(float amplitude) { m_movementAmplitude = amplitude; }
    float getMovementAmplitude() const { return m_movementAmplitude; }
    
    void setMovementDirection(const glm::vec3& direction) {
        m_movementDirection = glm::normalize(direction);
    }
    const glm::vec3& getMovementDirection() const { return m_movementDirection; }
    
    void setMovementCenter(const glm::vec3& center) { m_movementCenter = center; }
    const glm::vec3& getMovementCenter() const { return m_movementCenter; }
    
    // Time-based movement control
    void setMovementPeriod(float seconds) { m_movementPeriod = seconds; }
    float getMovementPeriod() const { return m_movementPeriod; }
    
    // Update method to apply movement pattern
    void update(float deltaTime) override;
    
    // Check if a point is inside the target (for hit detection)
    bool isPointInside(const glm::vec3& point) const;
    
private:
    // Target properties
    float m_radius;
    bool m_isActive = true;
    
    // Movement properties
    TargetMovementPattern m_movementPattern = TargetMovementPattern::STATIONARY;
    float m_movementSpeed = 10.0f;         // Movement speed in m/s
    float m_movementAmplitude = 30.0f;     // Movement amplitude for patterns (circular radius, sine amplitude)
    glm::vec3 m_movementDirection = glm::vec3(1.0f, 0.0f, 0.0f); // Direction for linear movement
    glm::vec3 m_movementCenter;           // Center point for circular movement
    float m_movementPeriod = 10.0f;        // Time to complete one movement cycle (seconds)
    
    // Movement tracking
    float m_movementTime = 0.0f;           // Current time in the movement cycle
    glm::vec3 m_initialPosition;           // Initial position for reference
    
    // Movement pattern implementations
    void applyLinearMovement(float deltaTime);
    void applyCircularMovement(float deltaTime);
    void applySinusoidalMovement(float deltaTime);
    void applyRandomMovement(float deltaTime);
    void enforceAirspaceConstraint();
}; 
