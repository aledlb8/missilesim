#pragma once

#include <glm/glm.hpp>
#include <string>

class PhysicsObject {
public:
    PhysicsObject(const glm::vec3& position = glm::vec3(0.0f), 
                 const glm::vec3& velocity = glm::vec3(0.0f),
                 float mass = 1.0f);
    virtual ~PhysicsObject() = default;
    
    // Update physics state
    virtual void update(float deltaTime);
    
    // Force methods
    void applyForce(const glm::vec3& force);
    void resetForces();
    
    // Getters and setters
    const glm::vec3& getPosition() const { return m_position; }
    void setPosition(const glm::vec3& position) { m_position = position; }

    const glm::vec3& getPreviousPosition() const { return m_previousPosition; }
    
    const glm::vec3& getVelocity() const { return m_velocity; }
    void setVelocity(const glm::vec3& velocity) { m_velocity = velocity; }
    
    const glm::vec3& getAcceleration() const { return m_acceleration; }
    
    float getMass() const { return m_mass; }
    void setMass(float mass) { m_mass = mass; }
    
    // Aerodynamic properties - these should be overridden by derived classes
    virtual float getDragCoefficient() const { return 0.0f; }
    virtual float getCrossSectionalArea() const { return 0.0f; }
    virtual float getLiftCoefficient() const { return 0.0f; }
    
    // Object type for rendering and other systems
    virtual std::string getType() const { return "PhysicsObject"; }
    
protected:
    glm::vec3 m_previousPosition; // Position at the start of the last integration step
    glm::vec3 m_position;     // Position in 3D space (meters)
    glm::vec3 m_velocity;     // Velocity (meters/second)
    glm::vec3 m_acceleration; // Acceleration (meters/second²)
    glm::vec3 m_forces;       // Accumulated forces for this frame
    float m_mass;             // Mass in kg
}; 
