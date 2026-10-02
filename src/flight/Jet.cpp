#include "Jet.h"

#include "AircraftCatalog.h"

#include <algorithm>
#include <cmath>

namespace missilesim::flight
{
namespace
{
    // Stevens & Lewis throttle gearing puts military power at lever 0.77.
    // Brochure cards do not use this gear.
    constexpr float kMilitaryLever = 0.77f;
}

Jet::Jet()
    : m_airframe(kCombatMassKg)
{
    configure(defaultAircraftId());
}

Jet::Jet(const char *aircraftId)
    : m_airframe(kCombatMassKg)
{
    configure(aircraftId);
}

void Jet::configure(const char *aircraftId)
{
    const AircraftCard *card = findAircraft(aircraftId);
    if (card == nullptr || card->tableModel)
    {
        m_table = true;
        m_id = card != nullptr ? card->id : defaultAircraftId();
        m_airframe = F16Airframe(kCombatMassKg);
    }
    else
    {
        m_table = false;
        m_id = card->id;
        m_card.bind(card);
    }
    m_flcs = {};
    m_instructor = {};
    m_command = {};
}

void Jet::reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up)
{
    if (m_table)
    {
        m_airframe.reset(position, velocity, forward, up);
    }
    else
    {
        m_card.reset(position, velocity, forward, up);
    }
    m_flcs = {};
    m_instructor = {};
    m_command = {};
    m_controls = {};
    m_controls.instructor.aimDirection = this->forward();
}

void Jet::setControls(const JetControls &controls, float sampleDeltaTime)
{
    m_controls = controls;
    m_instructor.observeAim(controls.instructor.aimDirection, sampleDeltaTime);
}

void Jet::step(float deltaTime, const AirData &air, float gravity)
{
    if (!(deltaTime > 0.0f) || !std::isfinite(deltaTime))
    {
        return;
    }

    const int steps = std::max(1, static_cast<int>(std::ceil(deltaTime / kInnerStepSeconds - 1.0e-4f)));
    const float dt = deltaTime / static_cast<float>(steps);
    if (m_table)
    {
        const float lever = m_controls.afterburner ? 1.0f : std::clamp(m_controls.throttle, 0.0f, 1.0f) * kMilitaryLever;
        m_airframe.setThrottle(lever);
        for (int i = 0; i < steps; ++i)
        {
            m_command = m_instructor.update(view(), m_controls.instructor, gravity, dt);
            m_airframe.setSurfaceCommand(m_flcs.update(m_airframe, m_command, gravity));
            m_airframe.step(dt, air, gravity);
        }
        return;
    }

    m_card.setThrottle(std::clamp(m_controls.throttle, 0.0f, 1.0f), m_controls.afterburner);
    for (int i = 0; i < steps; ++i)
    {
        m_command = m_instructor.update(view(), m_controls.instructor, gravity, dt);
        m_card.step(dt, air, gravity, m_command);
    }
}

const glm::vec3 &Jet::position() const
{
    return m_table ? m_airframe.position() : m_card.position();
}

const glm::vec3 &Jet::velocity() const
{
    return m_table ? m_airframe.velocity() : m_card.velocity();
}

glm::quat Jet::attitude() const
{
    return m_table ? m_airframe.attitude() : m_card.attitude();
}

glm::vec3 Jet::forward() const
{
    return m_table ? m_airframe.forward() : m_card.forward();
}

glm::vec3 Jet::right() const
{
    return m_table ? m_airframe.right() : m_card.right();
}

glm::vec3 Jet::up() const
{
    return m_table ? m_airframe.up() : m_card.up();
}

const glm::vec3 &Jet::bodyRates() const
{
    return m_table ? m_airframe.bodyRates() : m_card.bodyRates();
}

const AirframeTelemetry &Jet::telemetry() const
{
    return m_table ? m_airframe.telemetry() : m_card.telemetry();
}

float Jet::mass() const
{
    return m_table ? m_airframe.mass() : m_card.mass();
}

float Jet::enginePower() const
{
    return m_table ? m_airframe.enginePower() : m_card.enginePower();
}

AirframeView Jet::view() const
{
    if (!m_table)
    {
        return m_card.view();
    }
    AirframeView view;
    view.forward = m_airframe.forward();
    view.right = m_airframe.right();
    view.up = m_airframe.up();
    view.bodyRates = m_airframe.bodyRates();
    view.telemetry = m_airframe.telemetry();
    view.rollRateLimit = FlightControlSystem::rollRateLimit(view.telemetry.dynamicPressure, view.telemetry.alpha);
    return view;
}
}
