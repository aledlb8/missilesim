#include "Lift.h"
#include "objects/PhysicsObject.h"
#include "physics/Aerodynamics.h"
#include "physics/Atmosphere.h"
#include <glm/gtc/constants.hpp>

Lift::Lift(Atmosphere *atmosphere)
    : m_atmosphere(atmosphere)
{
}

void Lift::applyTo(PhysicsObject *object)
{
    if (!object || !m_atmosphere)
        return;

    // Missiles realise their lift through the proportional-navigation
    // autopilot (Missile::applyGuidance), which applies the maneuvering force
    // directly and reports the operating lift coefficient to the drag model.
    // Applying lift here as well would double-count that force.
    if (object->getType() == "Missile")
        return;

    // Get object properties needed for lift calculation
    glm::vec3 velocity = object->getVelocity();
    float speed = glm::length(velocity);

    // If object isn't moving, no lift
    if (speed < 0.001f)
        return;

    // Normalize velocity to get direction
    glm::vec3 direction = velocity / speed;

    // Sample local atmospheric state at the object's altitude.
    const float altitude = object->getPosition().y;
    const float density = m_atmosphere->calculateDensityAtAltitude(altitude);
    if (density <= 0.0f)
        return;

    // Reference area and operating lift coefficient.
    float operatingLiftCoefficient = object->getLiftCoefficient();
    float area = object->getCrossSectionalArea();
    const missilesim::physics::AeroProfile *profile = object->getAeroProfile();
    if (profile != nullptr)
    {
        area = profile->referenceArea;
    }

    // q = 0.5 * rho * v^2.
    const float dynamicPressure = 0.5f * density * speed * speed;

    // Non-missile lifting bodies use a simple world-up lift approximation.
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 liftDirection = up - glm::dot(up, direction) * direction;
    if (glm::length(liftDirection) > 0.001f)
    {
        liftDirection = glm::normalize(liftDirection);
    }
    else
    {
        liftDirection = glm::vec3(1.0f, 0.0f, 0.0f);
    }

    // Apply lift force: F_lift = q * Cl * A.
    glm::vec3 liftForce = (dynamicPressure * operatingLiftCoefficient * area) * liftDirection;
    object->applyForce(liftForce);
}