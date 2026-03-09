#pragma once

#include <glm/glm.hpp>

// Forward declarations
class PhysicsObject;
class Atmosphere;

class Lift {
public:
    Lift(Atmosphere* atmosphere);
    ~Lift() = default;
    
    // Apply lift force to an object
    void applyTo(PhysicsObject* object);
    
private:
    Atmosphere* m_atmosphere; // Reference to the atmosphere model
}; 