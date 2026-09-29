#include "Jet.h"

#include <algorithm>
#include <cmath>

namespace missilesim::flight
{
namespace
{
    // Stevens & Lewis throttle gearing puts military power at lever 0.77.
    constexpr float kMilitaryLever = 0.77f;
}

Jet::Jet()
    : m_airframe(kCombatMassKg)
{
}

void Jet::reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up)
{
    m_airframe.reset(position, velocity, forward, up);
    m_flcs = {};
    m_instructor = {};
    m_command = {};
}

void Jet::step(float deltaTime, const AirData &air, float gravity)
{
    if (!(deltaTime > 0.0f))
    {
        return;
    }

    const float lever = m_controls.afterburner ? 1.0f : std::clamp(m_controls.throttle, 0.0f, 1.0f) * kMilitaryLever;
    m_airframe.setThrottle(lever);
    m_instructor.observeAim(m_controls.instructor.aimDirection, deltaTime);

    const int steps = std::max(1, static_cast<int>(std::ceil(deltaTime / kInnerStepSeconds - 1.0e-4f)));
    const float dt = deltaTime / static_cast<float>(steps);
    for (int i = 0; i < steps; ++i)
    {
        m_command = m_instructor.update(m_airframe, m_controls.instructor, gravity);
        m_airframe.setSurfaceCommand(m_flcs.update(m_airframe, m_command, gravity));
        m_airframe.step(dt, air, gravity);
    }
}
}
