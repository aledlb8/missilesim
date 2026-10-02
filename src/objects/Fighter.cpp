#include "Fighter.h"

#include "flight/AircraftCatalog.h"

#include <algorithm>
#include <cmath>

#include <glm/gtx/norm.hpp>

namespace
{
    glm::vec3 unitOr(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        return glm::length2(vector) > 1.0e-8f ? glm::normalize(vector) : fallback;
    }
}

Fighter::Fighter()
    : PhysicsObject(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 250.0f), missilesim::flight::Jet::kCombatMassKg)
{
    setMaxLoadFactorG(0.0f);
    place(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 250.0f), glm::vec3(0.0f, 0.0f, 1.0f));
}

void Fighter::setAircraft(const char *aircraftId)
{
    const glm::vec3 position = m_position;
    const glm::vec3 velocity = m_velocity;
    const glm::vec3 nose = getNose();
    const float lever = m_lever;
    const float leverBeforeAfterburner = m_leverBeforeAfterburner;
    m_jet.configure(aircraftId);
    setMass(m_jet.mass());
    place(position, velocity, nose);
    m_lever = lever;
    m_leverBeforeAfterburner = leverBeforeAfterburner;
    missilesim::flight::InstructorInput input;
    input.aimDirection = nose;
    setInstructorInput(input, 0.0f);
}

void Fighter::place(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &nose)
{
    const glm::vec3 forward = unitOr(nose, glm::vec3(0.0f, 0.0f, 1.0f));
    m_jet.reset(position, velocity, forward, glm::vec3(0.0f, 1.0f, 0.0f));
    m_lever = 0.85f;
    missilesim::flight::InstructorInput input;
    input.aimDirection = forward;
    setInstructorInput(input, 0.0f);
    syncFromJet();
    m_previousPosition = position;
    m_stepStartPosition = position;
    m_renderPosition = position;
    m_previousRenderPosition = position;
    m_stepStartAttitude = m_jet.attitude();
    m_renderAttitude = m_stepStartAttitude;
}

void Fighter::adjustThrottle(float delta)
{
    m_lever = std::clamp(m_lever + delta, 0.0f, kMaxLever);
}

void Fighter::setInstructorInput(const missilesim::flight::InstructorInput &input, float sampleDeltaTime)
{
    missilesim::flight::JetControls controls;
    controls.instructor = input;
    controls.throttle = std::min(m_lever, 1.0f);
    controls.afterburner = isAfterburner();
    m_jet.setControls(controls, sampleDeltaTime);
}

void Fighter::toggleAfterburner()
{
    if (isAfterburner())
    {
        m_lever = std::min(m_leverBeforeAfterburner, kAfterburnerDetent);
    }
    else
    {
        m_leverBeforeAfterburner = m_lever;
        m_lever = kMaxLever;
    }
}

void Fighter::syncFromJet()
{
    m_position = m_jet.position();
    m_velocity = m_jet.velocity();
    // Felt acceleration direction for anything that banks visuals off it.
    m_acceleration = m_jet.attitude() * m_jet.telemetry().bodyAcceleration;
}

void Fighter::updateFlight(float deltaTime, float density, float speedOfSound, float gravity)
{
    if (!(deltaTime > 0.0f) || !std::isfinite(deltaTime))
    {
        return;
    }

    m_previousPosition = m_position;
    m_jet.step(deltaTime, {density, speedOfSound}, gravity);
    syncFromJet();
}

void Fighter::beginFixedStep()
{
    PhysicsObject::beginFixedStep();
    m_stepStartAttitude = m_jet.attitude();
}

void Fighter::setRenderBlend(float alpha)
{
    PhysicsObject::setRenderBlend(alpha);
    m_renderAttitude = glm::normalize(glm::slerp(m_stepStartAttitude, m_jet.attitude(), std::clamp(alpha, 0.0f, 1.0f)));
}
