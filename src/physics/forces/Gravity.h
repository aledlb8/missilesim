#pragma once

#include <glm/glm.hpp>

// Forward declaration to avoid circular includes
class PhysicsObject;

class Gravity
{
public:
    Gravity(float magnitude = 9.81f);
    ~Gravity() = default;

    // Apply gravity force to an object
    void applyTo(PhysicsObject *object);

    // Getter and setter for magnitude
    float getMagnitude() const { return m_magnitude; }
    void setMagnitude(float magnitude) { m_magnitude = magnitude; }

private:
    float m_magnitude;                  // Magnitude of gravity in m/s²
    glm::vec3 m_direction = {0, -1, 0}; // Direction of gravity (down by default)
};