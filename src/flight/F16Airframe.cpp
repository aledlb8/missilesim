#include "F16Airframe.h"

#include "F16Data.h"

#include <algorithm>
#include <cmath>

#include <glm/gtx/norm.hpp>

namespace missilesim::flight
{
namespace
{
    constexpr float kFeetPerMeter = 3.2808399f;
    constexpr float kNewtonsPerLbf = 4.448221615f;

    // The deck is the F100-PW-200 installed in the TP-1538 airplane. It is
    // scaled to the Block 50's F110-GE-129 by the ratio of uninstalled
    // ratings, keeping the deck's installation losses and its altitude/Mach
    // lapse. F110-GE-129: 17,155 lbf military, 29,500 lbf afterburner
    // (research_notes flight-model.md). F100-PW-200: 14,690 / 23,930 lbf
    // (Wikipedia specifications; another source gives 22,600 lbf with
    // afterburner, which would make the afterburner scale 1.31).
    constexpr float kMilitaryScale = 17155.0f / 14690.0f;
    constexpr float kAfterburnerScale = 29500.0f / 23930.0f;

    // TP-1538 appendix A actuators: 0.0495 s first-order lags with rate limits.
    constexpr float kActuatorLagSeconds = 0.0495f;
    constexpr float kElevatorRateDegPerSec = 60.0f;
    constexpr float kAileronRateDegPerSec = 80.0f;
    constexpr float kRudderRateDegPerSec = 120.0f;

    // Centre of gravity at the reference station, as in TP-1538's runs.
    constexpr float kCg = f16::kReferenceCg;

    // Zero-lift drag rise above the low-speed data. Brandt's "actual" F-16
    // polar (research_notes flight-model.md), taken relative to its Mach 0.3
    // value because the low-speed tables already carry the subsonic CD0.
    float machDragRise(float mach)
    {
        constexpr float kMach[] = {0.30f, 0.86f, 1.05f, 1.50f, 2.00f};
        constexpr float kCd0[] = {0.0193f, 0.0202f, 0.0444f, 0.0448f, 0.0458f};
        if (mach <= kMach[0])
        {
            return 0.0f;
        }
        for (int i = 1; i < 5; ++i)
        {
            if (mach <= kMach[i])
            {
                const float t = (mach - kMach[i - 1]) / (kMach[i] - kMach[i - 1]);
                return kCd0[i - 1] + (kCd0[i] - kCd0[i - 1]) * t - kCd0[0];
            }
        }
        return kCd0[4] - kCd0[0];
    }

    float approach(float value, float target, float maxRate, float deltaTime)
    {
        const float wanted = (target - value) / kActuatorLagSeconds;
        return value + std::clamp(wanted, -maxRate, maxRate) * deltaTime;
    }
}

glm::quat attitudeFromAxes(const glm::vec3 &forwardIn, const glm::vec3 &upIn)
{
    glm::vec3 forward = glm::length2(forwardIn) > 1.0e-8f ? glm::normalize(forwardIn) : glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 up = upIn - forward * glm::dot(upIn, forward);
    if (glm::length2(up) < 1.0e-8f)
    {
        const glm::vec3 helper = std::abs(forward.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        up = helper - forward * glm::dot(helper, forward);
    }
    up = glm::normalize(up);
    const glm::vec3 right = glm::cross(forward, up);
    const glm::vec3 down = -up;
    return glm::normalize(glm::quat_cast(glm::mat3(forward, right, down)));
}

F16Airframe::F16Airframe(float massKg)
    : m_mass(massKg)
{
    // TP-1538 inertias belong to its 20,500 lb test weight. They are scaled
    // with mass for the heavier loadout: an assumption, the store and fuel
    // distribution of that loadout was not published.
    const float scale = massKg / f16::kTestWeightKg;
    const float ixx = f16::kIxx * scale;
    const float iyy = f16::kIyy * scale;
    const float izz = f16::kIzz * scale;
    const float ixz = f16::kIxz * scale;
    const float gamma = ixx * izz - ixz * ixz;
    m_inertia.c1 = ((iyy - izz) * izz - ixz * ixz) / gamma;
    m_inertia.c2 = ((ixx - iyy + izz) * ixz) / gamma;
    m_inertia.c3 = izz / gamma;
    m_inertia.c4 = ixz / gamma;
    m_inertia.c5 = (izz - ixx) / iyy;
    m_inertia.c6 = ixz / iyy;
    m_inertia.c7 = 1.0f / iyy;
    m_inertia.c8 = (ixx * (ixx - iyy) + ixz * ixz) / gamma;
    m_inertia.c9 = ixx / gamma;
}

void F16Airframe::reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up)
{
    m_position = position;
    m_velocity = velocity;
    m_attitude = attitudeFromAxes(forward, up);
    m_rates = glm::vec3(0.0f);
    m_surfaces = {};
    m_surfaceCommand = {};
    m_telemetry = {};
    m_telemetry.airspeed = glm::length(velocity);
}

void F16Airframe::setThrottle(float lever)
{
    m_throttle = std::clamp(lever, 0.0f, 1.0f);
}

void F16Airframe::stepActuators(float deltaTime)
{
    const auto limited = [](float value, float limit) { return std::clamp(value, -limit, limit); };
    m_surfaces.elevator = limited(approach(m_surfaces.elevator, limited(m_surfaceCommand.elevator, f16::kElevatorLimitDeg),
                                           kElevatorRateDegPerSec, deltaTime),
                                  f16::kElevatorLimitDeg);
    m_surfaces.aileron = limited(approach(m_surfaces.aileron, limited(m_surfaceCommand.aileron, f16::kAileronLimitDeg),
                                          kAileronRateDegPerSec, deltaTime),
                                 f16::kAileronLimitDeg);
    m_surfaces.rudder = limited(approach(m_surfaces.rudder, limited(m_surfaceCommand.rudder, f16::kRudderLimitDeg),
                                         kRudderRateDegPerSec, deltaTime),
                                f16::kRudderLimitDeg);
}

F16Airframe::Loads F16Airframe::computeLoads(const AirData &air, float altitudeFt)
{
    Loads loads;
    const float speed = glm::length(m_velocity);
    const float mach = speed / std::max(air.speedOfSound, 1.0f);
    const float thrust = std::max(f16::thrustLbf(m_power, altitudeFt, mach, kMilitaryScale, kAfterburnerScale), -20000.0f) *
                         kNewtonsPerLbf;
    loads.force = glm::vec3(thrust, 0.0f, 0.0f);

    m_telemetry.airspeed = speed;
    m_telemetry.mach = mach;
    m_telemetry.thrust = thrust;
    m_telemetry.dynamicPressure = 0.5f * std::max(air.density, 0.0f) * speed * speed;
    if (speed < 1.0f)
    {
        m_telemetry.alpha = 0.0f;
        m_telemetry.beta = 0.0f;
        return loads;
    }

    const glm::vec3 body = toBody(m_velocity);
    const float alpha = std::atan2(body.z, body.x);
    const float beta = std::asin(std::clamp(body.y / speed, -1.0f, 1.0f));
    m_telemetry.alpha = alpha;
    m_telemetry.beta = beta;

    const float alphaDeg = glm::degrees(alpha);
    const float betaDeg = glm::degrees(beta);
    const float p = m_rates.x;
    const float q = m_rates.y;
    const float r = m_rates.z;
    const float el = m_surfaces.elevator;
    const float ail = m_surfaces.aileron;
    const float rdr = m_surfaces.rudder;
    const f16::Damping d = f16::damping(alphaDeg);
    const float chordTerm = f16::kChordM / (2.0f * speed);
    const float spanTerm = f16::kSpanM / (2.0f * speed);

    const float cxt = f16::cx(alphaDeg, el) + chordTerm * d.cxq * q;
    const float cyt = f16::cy(betaDeg, ail, rdr) + spanTerm * (d.cyr * r + d.cyp * p);
    const float czt = f16::cz(alphaDeg, betaDeg, el) + chordTerm * d.czq * q;
    const float dail = ail / 20.0f;
    const float drdr = rdr / 30.0f;
    const float clt = f16::cl(alphaDeg, betaDeg) + f16::dlda(alphaDeg, betaDeg) * dail + f16::dldr(alphaDeg, betaDeg) * drdr +
                      spanTerm * (d.clr * r + d.clp * p);
    const float cmt = f16::cm(alphaDeg, el) + chordTerm * d.cmq * q + czt * (f16::kReferenceCg - kCg);
    const float cnt = f16::cn(alphaDeg, betaDeg) + f16::dnda(alphaDeg, betaDeg) * dail + f16::dndr(alphaDeg, betaDeg) * drdr +
                      spanTerm * (d.cnr * r + d.cnp * p) - cyt * (f16::kReferenceCg - kCg) * f16::kChordM / f16::kSpanM;

    const float qs = m_telemetry.dynamicPressure * f16::kWingAreaM2;
    loads.force += qs * glm::vec3(cxt, cyt, czt);
    loads.force -= (body / speed) * (qs * machDragRise(mach));
    loads.moment = glm::vec3(qs * f16::kSpanM * clt, qs * f16::kChordM * cmt, qs * f16::kSpanM * cnt);
    return loads;
}

void F16Airframe::step(float deltaTime, const AirData &air, float gravity)
{
    if (!(deltaTime > 0.0f))
    {
        return;
    }

    stepActuators(deltaTime);
    const float commandedPower = f16::throttleGearing(m_throttle);
    m_power = std::clamp(m_power + f16::powerRate(m_power, commandedPower) * deltaTime, 0.0f, 100.0f);

    const Loads loads = computeLoads(air, std::max(m_position.y, 0.0f) * kFeetPerMeter);
    const glm::vec3 specificForce = loads.force / m_mass;
    const glm::vec3 gravityWorld(0.0f, -gravity, 0.0f);
    const float g = std::max(gravity, 0.1f);
    m_telemetry.bodyAcceleration = specificForce + toBody(gravityWorld);
    m_telemetry.normalLoad = -specificForce.z / g;
    m_telemetry.lateralLoad = specificForce.y / g;

    // Stevens & Lewis moment equations with the engine's angular momentum.
    const Inertia &c = m_inertia;
    const float he = f16::kEngineMomentum;
    const float p = m_rates.x;
    const float q = m_rates.y;
    const float r = m_rates.z;
    const float l = loads.moment.x;
    const float m = loads.moment.y;
    const float n = loads.moment.z;
    const glm::vec3 rateDot((c.c2 * p + c.c1 * r + c.c4 * he) * q + c.c3 * l + c.c4 * n,
                            (c.c5 * p - c.c7 * he) * r + c.c6 * (r * r - p * p) + c.c7 * m,
                            (c.c8 * p - c.c2 * r + c.c9 * he) * q + c.c4 * l + c.c9 * n);

    // Semi-implicit Euler: rates and velocity first, then attitude and position.
    m_rates += rateDot * deltaTime;
    m_velocity += (m_attitude * specificForce + gravityWorld) * deltaTime;
    m_position += m_velocity * deltaTime;

    const float rate = glm::length(m_rates);
    if (rate > 1.0e-7f)
    {
        m_attitude = glm::normalize(m_attitude * glm::angleAxis(rate * deltaTime, m_rates / rate));
    }
}
}
