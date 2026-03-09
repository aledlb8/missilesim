#include "Lift.h"
#include "objects/PhysicsObject.h"
#include "objects/Missile.h"
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
    
    // Sample local atmospheric state at the object's altitude.
    const float altitude = object->getPosition().y;
    const float density = m_atmosphere->calculateDensityAtAltitude(altitude);
    if (density <= 0.0f) return;

    // Get air density and object properties
    float liftCoefficient = object->getLiftCoefficient();
    float area = object->getCrossSectionalArea();
    
    // Calculate lift force magnitude: F_lift = q * Cl * A, where q = 0.5 * rho * v^2.
    const float dynamicPressure = 0.5f * density * speed * speed;
    float liftMagnitude = dynamicPressure * liftCoefficient * area;
    
    glm::vec3 liftDirection(0.0f);
    float liftScale = 1.0f;

    if (object->getType() == "Missile") {
        const Missile* missile = dynamic_cast<const Missile*>(object);
        if (!missile) {
            return;
        }

        const glm::vec3 thrustDirection = missile->getThrustDirection();
        glm::vec3 steeringComponent = thrustDirection - glm::dot(thrustDirection, direction) * direction;
        const float steeringDemand = glm::length(steeringComponent);

        if (steeringDemand < 0.001f) {
            return;
        }

        liftDirection = steeringComponent / steeringDemand;
        liftScale = glm::clamp(steeringDemand * 3.0f, 0.0f, 1.0f);
    } else {
        // Non-missile objects still use a simple world-up lift approximation.
        const glm::vec3 up(0.0f, 1.0f, 0.0f);
        liftDirection = up - glm::dot(up, direction) * direction;

        if (glm::length(liftDirection) > 0.001f) {
            liftDirection = glm::normalize(liftDirection);
        } else {
            liftDirection = glm::vec3(1.0f, 0.0f, 0.0f);
        }
    }

    // Apply lift force
    glm::vec3 liftForce = (liftMagnitude * liftScale) * liftDirection;
    object->applyForce(liftForce);
} 
