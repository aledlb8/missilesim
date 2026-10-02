#pragma once

#include "PhysicsObject.h"

#include "flight/Jet.h"

#include <algorithm>

#include <glm/gtc/quaternion.hpp>

// Player fighter. The mesh is always models/jet.obj. The F-16 card flies the
// NASA TP-1538 airframe; every other card flies that card's point mass.
// Integrated by Application, not by PhysicsEngine::m_objects; this class
// mirrors the jet's state into PhysicsObject for the renderer, HUD and audio.
class Fighter : public PhysicsObject
{
public:
    // Throttle lever above this is afterburner (War Thunder's WEP zone).
    static constexpr float kAfterburnerDetent = 1.0f;
    static constexpr float kMaxLever = 1.1f;

    Fighter();

    // Replaces the flight model and keeps the current position, velocity, and nose.
    void setAircraft(const char *aircraftId);

    std::string getType() const override { return "Fighter"; }
    float getRadius() const { return m_radius; }
    glm::vec3 getRenderAcceleration() const override { return m_acceleration; }

    void place(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &nose);
    void setInstructorInput(const missilesim::flight::InstructorInput &input, float sampleDeltaTime);
    void adjustThrottle(float delta);
    void toggleAfterburner();

    // Lever 0..kMaxLever; 1.0 is military power.
    float getThrottleLever() const { return m_lever; }
    // 0..1 of military power, for plumes and the HUD bar.
    float getThrottle() const { return std::min(m_lever, 1.0f); }
    bool isAfterburner() const { return m_lever > kAfterburnerDetent + 1.0e-3f; }
    float getEnginePower() const { return m_jet.enginePower(); }

    glm::vec3 getNose() const { return m_jet.forward(); }
    glm::vec3 getRight() const { return m_jet.right(); }
    glm::vec3 getUp() const { return m_jet.up(); }
    // Interpolated attitude for drawing (see PhysicsObject::setRenderBlend).
    glm::vec3 getRenderNose() const { return m_renderAttitude * glm::vec3(1.0f, 0.0f, 0.0f); }
    glm::vec3 getRenderUp() const { return m_renderAttitude * glm::vec3(0.0f, 0.0f, -1.0f); }

    float getLoadFactor() const { return m_jet.telemetry().normalLoad; }
    float getAngleOfAttack() const { return m_jet.telemetry().alpha; }
    float getMach() const { return m_jet.telemetry().mach; }
    const missilesim::flight::Jet &jet() const { return m_jet; }

    void updateFlight(float deltaTime, float density, float speedOfSound, float gravity);

    void beginFixedStep() override;
    void setRenderBlend(float alpha) override;

private:
    void syncFromJet();

    float m_radius = 5.0f;
    float m_lever = 0.85f;
    float m_leverBeforeAfterburner = 1.0f;
    missilesim::flight::Jet m_jet;
    glm::quat m_stepStartAttitude{1.0f, 0.0f, 0.0f, 0.0f};
    glm::quat m_renderAttitude{1.0f, 0.0f, 0.0f, 0.0f};
};
