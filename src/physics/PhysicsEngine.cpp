#include "PhysicsEngine.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <stdexcept>

PhysicsEngine::PhysicsEngine()
{
    // Create force components with default values
    m_atmosphere = std::make_unique<Atmosphere>(1.225f); // Standard air density at sea level (kg/m³)
    m_gravity = std::make_unique<Gravity>(9.81f);        // Standard gravity (m/s²)
    m_drag = std::make_unique<Drag>(m_atmosphere.get());
    m_lift = std::make_unique<Lift>(m_atmosphere.get());
}

void PhysicsEngine::update(float deltaTime)
{
    try
    {
        // Update all physics objects
        for (auto *object : m_objects)
        {
            if (!object)
                continue;

            if (object->getType() == "Target")
            {
                continue;
            }

            // Reset forces before applying new ones
            object->resetForces();

            // Apply gravity
            if (m_gravity)
            {
                m_gravity->applyTo(object);
            }

            // Apply aerodynamic forces
            if (object->getType() == "Missile")
            {
                Missile *missile = static_cast<Missile *>(object);
                missile->setGroundReferenceAltitude(m_groundLevel);

                // Apply drag
                if (m_drag)
                {
                    m_drag->applyTo(object);
                }

                // Apply lift
                if (m_lift)
                {
                    m_lift->applyTo(object);
                }

                // Apply missile guidance
                if (missile->isGuidanceEnabled() && missile->hasTarget())
                {
                    missile->applyGuidance(deltaTime);
                }
            }

            // Update object physics (position, velocity, etc.)
            object->update(deltaTime);

            // Handle ground collision
            if (m_groundEnabled)
            {
                handleGroundCollision(object);
            }
        }

        // Update scripted targets separately from the rigid-body objects list.
        for (auto *target : m_targets)
        {
            if (target && target->isActive())
            {
                // Apply the update to handle movement patterns
                target->update(deltaTime);
            }
        }

        // Handle target-missile collisions
        handleTargetCollisions();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in PhysicsEngine::update: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in PhysicsEngine::update" << std::endl;
    }
}

void PhysicsEngine::handleGroundCollision(PhysicsObject *object)
{
    if (!object)
        return;

    const glm::vec3 &pos = object->getPosition();
    glm::vec3 vel = object->getVelocity();

    // Check if object is below ground level
    if (pos.y < m_groundLevel)
    {
        // Calculate new position (at ground level)
        glm::vec3 newPos = pos;
        newPos.y = m_groundLevel;

        // Apply restitution to velocity (bounce effect)
        if (vel.y < 0)
        {
            vel.y = -vel.y * m_groundRestitution;

            // Apply some friction to horizontal velocity components
            const float friction = 0.8f;
            vel.x *= friction;
            vel.z *= friction;

            // If velocity is very small, stop movement to prevent endless tiny bounces
            if (glm::length(vel) < 0.1f)
            {
                vel = glm::vec3(0.0f);
            }
        }

        // Update object with new position and velocity
        object->setPosition(newPos);
        object->setVelocity(vel);
    }
}

void PhysicsEngine::handleTargetCollisions()
{
    // Check each missile against each target
    for (auto *object : m_objects)
    {
        // Check if object is a missile
        if (object->getType() == "Missile")
        {
            Missile *missile = static_cast<Missile *>(object);
            checkMissileTargetHit(missile);
        }
    }
}

bool PhysicsEngine::checkMissileTargetHit(Missile *missile)
{
    if (!missile)
        return false;

    const glm::vec3 &missilePos = missile->getPosition();

    // Check against each target
    for (auto *target : m_targets)
    {
        // Skip inactive targets
        if (!target->isActive())
            continue;

        // Check if missile position is inside target radius
        if (target->isPointInside(missilePos))
        {
            // Hit detected! Deactivate the target
            target->setActive(false);
            return true;
        }
    }

    return false;
}

void PhysicsEngine::addObject(PhysicsObject *object)
{
    try
    {
        if (!object)
        {
            std::cerr << "Warning: Attempted to add null physics object" << std::endl;
            return;
        }

        // Check if the object is already in the list
        if (std::find(m_objects.begin(), m_objects.end(), object) == m_objects.end())
        {
            m_objects.push_back(object);
        }
        else
        {
            std::cerr << "Warning: Object already exists in physics engine" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Exception in addObject: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Error: Unknown exception in addObject" << std::endl;
    }
}

void PhysicsEngine::removeObject(PhysicsObject *object)
{
    try
    {
        if (!object)
        {
            std::cerr << "Warning: Attempted to remove null physics object" << std::endl;
            return;
        }

        auto it = std::find(m_objects.begin(), m_objects.end(), object);
        if (it != m_objects.end())
        {
            m_objects.erase(it);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Exception in removeObject: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Error: Unknown exception in removeObject" << std::endl;
    }
}

void PhysicsEngine::addTarget(Target *target)
{
    try
    {
        if (!target)
        {
            std::cerr << "Warning: Attempted to add null target" << std::endl;
            return;
        }

        // Check if target is already in targets list
        if (std::find(m_targets.begin(), m_targets.end(), target) == m_targets.end())
        {
            m_targets.push_back(target);
        }
        else
        {
            std::cerr << "Warning: Target already exists in physics engine" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Exception in addTarget: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Error: Unknown exception in addTarget" << std::endl;
    }
}

void PhysicsEngine::removeTarget(Target *target)
{
    try
    {
        if (!target)
        {
            std::cerr << "Warning: Attempted to remove null target" << std::endl;
            return;
        }

        // Remove from targets list
        auto targetIt = std::find(m_targets.begin(), m_targets.end(), target);
        if (targetIt != m_targets.end())
        {
            m_targets.erase(targetIt);

            // Also remove from physics objects list
            auto objectIt = std::find(m_objects.begin(), m_objects.end(), target);
            if (objectIt != m_objects.end())
            {
                m_objects.erase(objectIt);
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Exception in removeTarget: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Error: Unknown exception in removeTarget" << std::endl;
    }
}

void PhysicsEngine::setMissileTarget(Missile *missile, Target *target)
{
    try
    {
        // Safety checks
        if (!missile)
        {
            std::cerr << "Warning: Attempted to set target for null missile" << std::endl;
            return;
        }

        if (!target)
        {
            std::cerr << "Warning: Attempted to set null target for missile" << std::endl;
            missile->clearTarget(); // Make sure the missile doesn't have a bad target
            missile->setGuidanceEnabled(false);
            return;
        }

        if (!target->isActive())
        {
            std::cerr << "Warning: Attempted to set inactive target for missile" << std::endl;
            missile->clearTarget();
            missile->setGuidanceEnabled(false);
            return;
        }

        // Validate target position
        const glm::vec3 &targetPos = target->getPosition();
        if (std::isnan(targetPos.x) || std::isnan(targetPos.y) || std::isnan(targetPos.z) ||
            std::isinf(targetPos.x) || std::isinf(targetPos.y) || std::isinf(targetPos.z))
        {
            std::cerr << "Warning: Target has invalid position (NaN or infinity)" << std::endl;
            missile->clearTarget();
            missile->setGuidanceEnabled(false);
            return;
        }

        // Print debug information
        std::cout << "Setting missile target:" << std::endl;
        std::cout << "  Missile position: (" << missile->getPosition().x << ", "
                  << missile->getPosition().y << ", " << missile->getPosition().z << ")" << std::endl;
        std::cout << "  Target position: (" << target->getPosition().x << ", "
                  << target->getPosition().y << ", " << target->getPosition().z << ")" << std::endl;
        std::cout << "  Target is active: " << (target->isActive() ? "Yes" : "No") << std::endl;
        std::cout << "  Target movement pattern: " << static_cast<int>(target->getMovementPattern()) << std::endl;
        std::cout << "  Target movement speed: " << target->getMovementSpeed() << std::endl;

        // Set the target object for guidance (new approach)
        missile->setTargetObject(target);
        missile->setGuidanceEnabled(true);

        std::cout << "Target set successfully, guidance enabled with continuous tracking" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Exception in setMissileTarget: " << e.what() << std::endl;
        if (missile)
        {
            missile->clearTarget();
            missile->setGuidanceEnabled(false);
        }
    }
    catch (...)
    {
        std::cerr << "Error: Unknown exception in setMissileTarget" << std::endl;
        if (missile)
        {
            missile->clearTarget();
            missile->setGuidanceEnabled(false);
        }
    }
}

float PhysicsEngine::getGravity() const
{
    return m_gravity->getMagnitude();
}

void PhysicsEngine::setGravity(float gravity)
{
    m_gravity->setMagnitude(gravity);
}

float PhysicsEngine::getAirDensity() const
{
    return m_atmosphere->getDensity();
}

void PhysicsEngine::setAirDensity(float density)
{
    m_atmosphere->setDensity(density);
}

float PhysicsEngine::getAirDensityAtAltitude(float altitude) const
{
    return m_atmosphere->calculateDensityAtAltitude(altitude);
}

Atmosphere::State PhysicsEngine::getAtmosphereState(float altitude) const
{
    return m_atmosphere->sample(altitude);
}
