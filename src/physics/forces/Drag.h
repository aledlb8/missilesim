#pragma once

#include <glm/glm.hpp>

// Forward declarations
class PhysicsObject;
class Atmosphere;

class Drag {
public:
    Drag(Atmosphere* atmosphere);
    ~Drag() = default;
    
    // Apply drag force to an object
    void applyTo(PhysicsObject* object);
    
private:
    Atmosphere* m_atmosphere; // Reference to the atmosphere model
}; 