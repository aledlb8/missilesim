#pragma once

#include <glm/glm.hpp>
#include <string>

#include "physics/Aerodynamics.h"

class PhysicsObject
{
public:
    PhysicsObject(const glm::vec3 &position = glm::vec3(0.0f),
                  const glm::vec3 &velocity = glm::vec3(0.0f),
                  float mass = 1.0f);
    virtual ~PhysicsObject() = default;

    // Update physics state
    virtual void update(float deltaTime);

    // Force methods
    void applyForce(const glm::vec3 &force);
    void resetForces();

    // Getters and setters
    const glm::vec3 &getPosition() const { return m_position; }
    void setPosition(const glm::vec3 &position) { m_position = position; }

    const glm::vec3 &getPreviousPosition() const { return m_previousPosition; }

    const glm::vec3 &getVelocity() const { return m_velocity; }
    void setVelocity(const glm::vec3 &velocity) { m_velocity = velocity; }

    const glm::vec3 &getAcceleration() const { return m_acceleration; }

    // Acceleration used for visual orientation (e.g. banking). Defaults to the
    // physical acceleration; bodies whose per-step acceleration is noisy may
    // return a smoothed value so the rendered attitude does not jitter.
    virtual glm::vec3 getRenderAcceleration() const { return m_acceleration; }

    float getMass() const { return m_mass; }
    void setMass(float mass) { m_mass = mass; }

    // Physical structural limit on the airframe's total load factor, in g.
    // 0 means unlimited. This replaces the old arbitrary acceleration clamp:
    // the airframe can pull only as many g as its structure tolerates.
    float getMaxLoadFactorG() const { return m_maxLoadFactorG; }
    void setMaxLoadFactorG(float loadFactorG) { m_maxLoadFactorG = (loadFactorG >= 0.0f) ? loadFactorG : 0.0f; }

    // Aerodynamic properties - these should be overridden by derived classes
    virtual float getDragCoefficient() const { return 0.0f; }
    virtual float getCrossSectionalArea() const { return 0.0f; }
    virtual float getLiftCoefficient() const { return 0.0f; }

    // Optional Mach-dependent aerodynamic profile. When non-null, the force
    // models use it instead of the constant coefficients above. Bodies without
    // a profile (e.g. flares) fall back to the constant-coefficient path.
    virtual const missilesim::physics::AeroProfile *getAeroProfile() const { return nullptr; }

    // Operating lift coefficient currently commanded by the airframe's
    // autopilot/maneuver. Drag uses it to charge the induced (turn) drag.
    // Defaults to 0 (non-maneuvering bodies incur no induced drag).
    virtual float getCommandedLiftCoefficient() const { return 0.0f; }

    // Object type for rendering and other systems
    virtual std::string getType() const { return "PhysicsObject"; }

    // Render interpolation ("Fix Your Timestep", Glenn Fiedler). Physics runs
    // in fixed steps that do not line up with frames; drawing the raw state
    // makes motion judder (a frame may see zero, one or two steps). The
    // application marks the start of every fixed step, then before drawing
    // blends the last two step states by the leftover step fraction.
    virtual void beginFixedStep() { m_stepStartPosition = m_position; }
    virtual void setRenderBlend(float alpha);
    const glm::vec3 &getRenderPosition() const { return m_renderPosition; }
    // Render position one frame ago, for emitting trails between frames.
    const glm::vec3 &getPreviousRenderPosition() const { return m_previousRenderPosition; }

protected:
    glm::vec3 m_stepStartPosition; // position at the start of the current fixed step
    glm::vec3 m_renderPosition;
    glm::vec3 m_previousRenderPosition;
    glm::vec3 m_previousPosition; // Position at the start of the last integration step
    glm::vec3 m_position;         // Position in 3D space (meters)
    glm::vec3 m_velocity;         // Velocity (meters/second)
    glm::vec3 m_acceleration;     // Acceleration (meters/second²)
    glm::vec3 m_forces;           // Accumulated forces for this frame
    float m_mass;                 // Mass in kg
    float m_maxLoadFactorG = 0.0f; // Structural load-factor limit in g (0 = unlimited)
};