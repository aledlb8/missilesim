#include "Lift.h"
#include "objects/PhysicsObject.h"
#include "physics/Atmosphere.h"
#include <glm/gtc/constants.hpp>

Lift::Lift(Atmosphere* atmosphere)
    : m_atmosphere(atmosphere) {
}

void Lift::applyTo(PhysicsObject* object) {
    if (!object || !m_atmosphere) return;
    
    // Get object properties needed for lift calculation
    glm::vec3 velocity = object->getVelocity();
    float speed = glm::length(velocity);
    
    // If object isn't moving, no lift
    if (speed < 0.001f) return;
    
    // Normalize velocity to get direction
    glm::vec3 direction = velocity / speed;
    
    // Get air density and object properties
    float density = m_atmosphere->getDensity();
    float liftCoefficient = object->getLiftCoefficient();
    float area = object->getCrossSectionalArea();
    
    // Calculate lift force magnitude: F_lift = 0.5 * rho * v^2 * Cl * A
    float liftMagnitude = 0.5f * density * speed * speed * liftCoefficient * area;
    
    // Calculate lift direction (perpendicular to velocity)
    // For simplicity, we'll apply lift in the up direction (world Y-axis)
    // In a more sophisticated model, we'd use the object's orientation
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    
    // Make sure lift is perpendicular to velocity
    // Remove any component of 'up' that's parallel to direction
    glm::vec3 liftDirection = up - glm::dot(up, direction) * direction;
    
    // Normalize the lift direction
    if (glm::length(liftDirection) > 0.001f) {
        liftDirection = glm::normalize(liftDirection);
    } else {
        // If velocity is straight up or down, use a default lift direction
        liftDirection = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    
    // Apply lift force
    glm::vec3 liftForce = liftMagnitude * liftDirection;
    object->applyForce(liftForce);
} 