#include "Drag.h"
#include "objects/PhysicsObject.h"
#include "physics/Aerodynamics.h"
#include "physics/Atmosphere.h"
#include <glm/gtc/constants.hpp>

Drag::Drag(Atmosphere *atmosphere)
    : m_atmosphere(atmosphere)
{
}

void Drag::applyTo(PhysicsObject *object)
{
    if (!object || !m_atmosphere)
        return;

    // Get object properties needed for drag calculation
    glm::vec3 velocity = object->getVelocity();
    float speed = glm::length(velocity);

    // If object isn't moving, no drag
    if (speed < 0.001f)
        return;

    // Normalize velocity to get direction
    glm::vec3 direction = velocity / speed;

    // Sample local atmospheric state at the object's altitude.
    const float altitude = object->getPosition().y;
    const Atmosphere::State state = m_atmosphere->sample(altitude);
    const float density = state.densityKgPerCubicMeter;
    if (density <= 0.0f)
        return;

    // Resolve the drag coefficient and reference area. Bodies with a Mach-aware
    // aerodynamic profile use a compressibility-corrected Cd0 evaluated at the
    // local Mach number (so drag rises through the transonic regime); bodies
    // without a profile keep the constant-coefficient model.
    float dragCoefficient;
    float area;
    const missilesim::physics::AeroProfile *profile = object->getAeroProfile();
    if (profile != nullptr && !profile->machDragMultiplier.empty())
    {
        const float speedOfSound = state.speedOfSoundMetersPerSecond;
        const float mach = (speedOfSound > 1e-3f) ? (speed / speedOfSound) : 0.0f;
        dragCoefficient = missilesim::physics::zeroLiftDragCoefficient(*profile, mach);
        area = profile->referenceArea;

        // Induced (lift-dependent) drag, charged on the lift coefficient the
        // guidance maneuver is currently commanding, so a hard turn bleeds
        // energy (Cd_i = Cl^2 / (pi*AR*e)).
        const float operatingLiftCoefficient = object->getCommandedLiftCoefficient();
        if (operatingLiftCoefficient != 0.0f)
        {
            dragCoefficient += missilesim::physics::inducedDragCoefficient(*profile, operatingLiftCoefficient);
        }
    }
    else
    {
        dragCoefficient = object->getDragCoefficient();
        area = object->getCrossSectionalArea();
    }

    // Calculate drag force magnitude: F_drag = q * Cd * A, where q = 0.5 * rho * v^2.
    const float dynamicPressure = 0.5f * density * speed * speed;
    float dragMagnitude = dynamicPressure * dragCoefficient * area;

    // Apply drag force in the opposite direction of velocity
    glm::vec3 dragForce = -dragMagnitude * direction;
    object->applyForce(dragForce);
}