#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>

#include "Atmosphere.h"
#include "forces/Gravity.h"
#include "forces/Drag.h"
#include "forces/Lift.h"
#include "objects/PhysicsObject.h"
#include "objects/Target.h"
#include "objects/Missile.h"

class PhysicsEngine {
public:
    PhysicsEngine();
    ~PhysicsEngine() = default;
    
    void update(float deltaTime);
    void addObject(PhysicsObject* object);
    void removeObject(PhysicsObject* object);
    
    // Getter and setter for gravity
    float getGravity() const;
    void setGravity(float gravity);
    
    // Getter and setter for air density
    float getAirDensity() const;
    void setAirDensity(float density);
    
    // Ground collision properties
    void setGroundEnabled(bool enabled) { m_groundEnabled = enabled; }
    bool isGroundEnabled() const { return m_groundEnabled; }
    void setGroundRestitution(float restitution) { m_groundRestitution = restitution; }
    float getGroundRestitution() const { return m_groundRestitution; }
    
    // Target management
    void addTarget(Target* target);
    void removeTarget(Target* target);
    const std::vector<Target*>& getTargets() const { return m_targets; }
    
    // Missile guidance
    void setMissileTarget(Missile* missile, Target* target);
    
    // Hit detection
    bool checkMissileTargetHit(Missile* missile);
    
private:
    void handleGroundCollision(PhysicsObject* object);
    void handleTargetCollisions();
    
    std::vector<PhysicsObject*> m_objects;
    std::vector<Target*> m_targets;
    
    // Physics forces
    std::unique_ptr<Gravity> m_gravity;
    std::unique_ptr<Drag> m_drag;
    std::unique_ptr<Lift> m_lift;
    std::unique_ptr<Atmosphere> m_atmosphere;
    
    // Ground collision properties
    bool m_groundEnabled = true;
    float m_groundLevel = 0.0f;  // Y coordinate of the ground
    float m_groundRestitution = 0.5f;  // Bounciness of the ground (0 to 1)
}; 