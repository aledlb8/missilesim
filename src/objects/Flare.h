#pragma once

#include "PhysicsObject.h"
#include <glm/glm.hpp>

struct FlareLaunchRequest
{
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 velocity = glm::vec3(0.0f);
    float mass = 0.9f;
    float dragCoefficient = 1.1f;
    float crossSectionalArea = 0.018f;
    float lifetime = 4.0f;
    float heatSignature = 5.0f;
    float heatDecayRate = 1.4f;
};

class Flare : public PhysicsObject
{
public:
    explicit Flare(const FlareLaunchRequest &request);
    ~Flare() override = default;

    std::string getType() const override { return "Flare"; }

    float getDragCoefficient() const override { return m_dragCoefficient; }
    float getCrossSectionalArea() const override { return m_crossSectionalArea; }

    bool isActive() const { return m_isActive; }
    void setActive(bool active) { m_isActive = active; }

    float getLifetimeRemaining() const { return m_lifetimeRemaining; }
    float getHeatSignature() const { return m_currentHeatSignature; }
    float getInitialHeatSignature() const { return m_initialHeatSignature; }

    void update(float deltaTime) override;

private:
    bool m_isActive = true;
    float m_dragCoefficient = 1.1f;
    float m_crossSectionalArea = 0.018f;
    float m_lifetimeRemaining = 0.0f;
    float m_initialHeatSignature = 0.0f;
    float m_currentHeatSignature = 0.0f;
    float m_heatDecayRate = 1.4f;
};