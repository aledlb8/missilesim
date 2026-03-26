#pragma once

#include "PhysicsObject.h"
#include "Target.h" // Include complete Target definition instead of forward declaration
#include <vector>

class Flare;

class Missile : public PhysicsObject
{
public:
    Missile(const glm::vec3 &position = glm::vec3(0.0f),
            const glm::vec3 &velocity = glm::vec3(0.0f, 0.0f, 50.0f),
            float mass = 100.0f,
            float dragCoefficient = 0.1f,
            float crossSectionalArea = 0.1f,
            float liftCoefficient = 0.1f);

    ~Missile() override = default;

    // Override aerodynamic properties
    float getDragCoefficient() const override { return m_dragCoefficient; }
    void setDragCoefficient(float coefficient) { m_dragCoefficient = coefficient; }

    float getCrossSectionalArea() const override { return m_crossSectionalArea; }
    void setCrossSectionalArea(float area) { m_crossSectionalArea = area; }

    float getLiftCoefficient() const override { return m_liftCoefficient; }
    void setLiftCoefficient(float coefficient) { m_liftCoefficient = coefficient; }

    // Override object type
    std::string getType() const override { return "Missile"; }

    // Guidance system - static target position (old system)
    void setTarget(const glm::vec3 &targetPosition);

    // Guidance system - dynamic target tracking
    void setTargetObject(Target *target);

    // Get tracked target
    Target *getTargetObject() const { return m_targetObject; }

    void clearTarget();

    bool hasTarget() const { return m_hasTarget; }
    const glm::vec3 &getTargetPosition() const { return m_targetPosition; }

    // Guidance parameters
    void setGuidanceEnabled(bool enabled) { m_guidanceEnabled = enabled; }
    bool isGuidanceEnabled() const { return m_guidanceEnabled; }
    void setMass(float mass);
    float getDryMass() const { return m_dryMass; }
    void setNavigationGain(float gain) { m_navigationGain = gain; }
    float getNavigationGain() const { return m_navigationGain; }
    void setMaxSteeringForce(float force) { m_maxSteeringForce = force; }
    float getMaxSteeringForce() const { return m_maxSteeringForce; }
    void setTrackingAngle(float angleDegrees);
    float getTrackingAngle() const { return m_trackingAngleDegrees; }
    void setProximityFuseRadius(float radiusMeters);
    float getProximityFuseRadius() const { return m_proximityFuseRadius; }
    void setTerrainAvoidanceEnabled(bool enabled) { m_terrainAvoidanceEnabled = enabled; }
    bool isTerrainAvoidanceEnabled() const { return m_terrainAvoidanceEnabled; }
    void setTerrainClearance(float clearance) { m_terrainClearance = (clearance >= 0.0f) ? clearance : 0.0f; }
    float getTerrainClearance() const { return m_terrainClearance; }
    void setTerrainLookAheadTime(float seconds) { m_terrainLookAheadTime = (seconds >= 0.5f) ? seconds : 0.5f; }
    float getTerrainLookAheadTime() const { return m_terrainLookAheadTime; }
    void setGroundReferenceAltitude(float altitude) { m_groundReferenceAltitude = altitude; }
    float getGroundReferenceAltitude() const { return m_groundReferenceAltitude; }
    void setCountermeasureResistance(float resistance);
    float getCountermeasureResistance() const { return m_countermeasureResistance; }
    bool isTrackingDecoy() const { return m_trackingDecoy; }

    // Apply guidance force
    void updateHeatSeeker(const std::vector<Flare *> &flares, float deltaTime);
    void applyGuidance(float deltaTime);
    bool consumeSelfDestructRequest();

    // Thrust system
    void setThrust(float newtons) { m_thrust = newtons; }
    float getThrust() const { return m_thrust; }

    void setThrustDirection(const glm::vec3 &direction)
    {
        if (glm::dot(direction, direction) > 0.0001f)
        {
            m_thrustDirection = glm::normalize(direction);
        }
    }
    const glm::vec3 &getThrustDirection() const { return m_thrustDirection; }

    void setThrustEnabled(bool enabled) { m_thrustEnabled = enabled; }
    bool isThrustEnabled() const { return m_thrustEnabled; }

    void setFuel(float kg);
    float getFuel() const { return m_fuel; }

    void setFuelConsumptionRate(float kgPerSecond) { m_fuelConsumptionRate = (kgPerSecond >= 0.0f) ? kgPerSecond : 0.0f; }
    float getFuelConsumptionRate() const { return m_fuelConsumptionRate; }

    // Apply thrust and return true if thrust was applied
    bool applyThrust(float deltaTime);

    // Override update to handle thrust
    void update(float deltaTime) override;

private:
    void synchronizeMass();

    // Aerodynamic properties
    float m_dragCoefficient;
    float m_crossSectionalArea;
    float m_liftCoefficient;
    float m_dryMass = 0.0f;

    // Guidance properties
    bool m_guidanceEnabled = true;
    bool m_hasTarget = false;
    glm::vec3 m_targetPosition = glm::vec3(0.0f);
    glm::vec3 m_trackedSourceVelocity = glm::vec3(0.0f);
    Target *m_targetObject = nullptr; // Pointer to target object for continuous tracking
    const Flare *m_trackedFlare = nullptr;
    float m_navigationGain = 4.0f;       // Legacy tuning input, remapped as intercept lead aggressiveness
    float m_maxSteeringForce = 20000.0f; // Lateral control authority in Newtons
    float m_trackingAngleDegrees = 85.0f;
    float m_proximityFuseRadius = 18.0f;
    float m_countermeasureResistance = 0.65f;
    float m_lockRetentionBias = 0.25f;
    bool m_trackingDecoy = false;
    bool m_selfDestructRequested = false;
    bool m_terrainAvoidanceEnabled = true;
    float m_terrainClearance = 90.0f;
    float m_terrainLookAheadTime = 6.0f;
    float m_groundReferenceAltitude = 0.0f;

    // Thrust properties
    float m_thrust = 10000.0f;                                 // Thrust force in Newtons
    glm::vec3 m_thrustDirection = glm::vec3(0.0f, 0.0f, 1.0f); // Direction of thrust
    bool m_thrustEnabled = false;                              // Whether thrust is active
    float m_fuel = 0.0f;                                       // Fuel amount in kg
    float m_fuelConsumptionRate = 0.5f;                        // Fuel consumption in kg/second
};
