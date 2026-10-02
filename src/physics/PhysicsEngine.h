#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>

#include "Atmosphere.h"
#include "forces/Gravity.h"
#include "forces/Drag.h"
#include "forces/Lift.h"
#include "objects/PhysicsObject.h"
#include "objects/Flare.h"
#include "objects/Target.h"
#include "objects/Missile.h"

// First contact of a body with the ground during one update() call.
struct GroundContact
{
    const PhysicsObject *object = nullptr;
    glm::vec3 position{0.0f};  // where it met the ground
    glm::vec3 velocity{0.0f};  // velocity at contact, before any bounce
    float timeIntoUpdate = 0.0f; // seconds into the update() step
};

class PhysicsEngine
{
public:
    PhysicsEngine();
    ~PhysicsEngine() = default;

    void update(float deltaTime);
    void addObject(PhysicsObject *object);
    void removeObject(PhysicsObject *object);

    // Getter and setter for gravity
    float getGravity() const;
    void setGravity(float gravity);

    // Getter and setter for configurable sea-level reference density
    float getAirDensity() const;
    void setAirDensity(float density);
    float getAirDensityAtAltitude(float altitude) const;
    Atmosphere::State getAtmosphereState(float altitude) const;

    // Ground collision properties
    void setGroundEnabled(bool enabled) { m_groundEnabled = enabled; }
    bool isGroundEnabled() const { return m_groundEnabled; }
    float getGroundLevel() const { return m_groundLevel; }
    void setGroundRestitution(float restitution) { m_groundRestitution = restitution; }
    float getGroundRestitution() const { return m_groundRestitution; }
    // Bodies that touched the ground during the last update(). A bounce can
    // carry a body back above the ground by the end of the step, so its end
    // position alone does not show the contact.
    const std::vector<GroundContact> &getGroundContacts() const { return m_groundContacts; }

    // Target management
    void addTarget(Target *target);
    void removeTarget(Target *target);
    const std::vector<Target *> &getTargets() const { return m_targets; }

    // Flare management
    void addFlare(Flare *flare);
    void removeFlare(Flare *flare);
    const std::vector<Flare *> &getFlares() const { return m_flares; }

    // Hit detection and every other outcome belong to sim::World, which sees
    // all platforms (including the player's fighter) and both bodies' motion.

private:
    void integrateStep(float deltaTime, float timeIntoUpdate);
    void handleGroundCollision(PhysicsObject *object, float timeIntoUpdate);

    std::vector<PhysicsObject *> m_objects;
    std::vector<Target *> m_targets;
    std::vector<Flare *> m_flares;
    std::vector<GroundContact> m_groundContacts;

    // Physics forces
    std::unique_ptr<Gravity> m_gravity;
    std::unique_ptr<Drag> m_drag;
    std::unique_ptr<Lift> m_lift;
    std::unique_ptr<Atmosphere> m_atmosphere;

    // Ground collision properties
    bool m_groundEnabled = true;
    float m_groundLevel = 0.0f;       // Y coordinate of the ground
    float m_groundRestitution = 0.5f; // Bounciness of the ground (0 to 1)
};