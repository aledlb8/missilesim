#include "Drag.h"
#include "objects/PhysicsObject.h"
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
    const float density = m_atmosphere->calculateDensityAtAltitude(altitude);
    if (density <= 0.0f)
        return;

    // Get object properties
    float dragCoefficient = object->getDragCoefficient();
    float area = object->getCrossSectionalArea();

    // Calculate drag force magnitude: F_drag = q * Cd * A, where q = 0.5 * rho * v^2.
    const float dynamicPressure = 0.5f * density * speed * speed;
    float dragMagnitude = dynamicPressure * dragCoefficient * area;

    // Apply drag force in the opposite direction of velocity
    glm::vec3 dragForce = -dragMagnitude * direction;
    object->applyForce(dragForce);
}