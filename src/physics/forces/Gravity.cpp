#include "Gravity.h"
#include "objects/PhysicsObject.h"

Gravity::Gravity(float magnitude)
    : m_magnitude(magnitude)
{
}

void Gravity::applyTo(PhysicsObject *object)
{
    if (!object)
        return;

    // F = m * g
    glm::vec3 force = object->getMass() * m_magnitude * m_direction;
    object->applyForce(force);
}