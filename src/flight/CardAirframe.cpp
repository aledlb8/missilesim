#include "CardAirframe.h"

#include <algorithm>
#include <cmath>

namespace missilesim::flight
{
namespace
{
    constexpr float kGravity = 9.80665f;
    constexpr float kSeaLevelDensity = 1.225f;
    // Cd = Cd0 + k Cl^2. This k is not an Oswald efficiency and not a published derivative.
    constexpr float kInduced = 0.12f;
    // Lift-curve stand-in, per radian, used only to turn a load into a displayed angle.
    constexpr float kClPerRad = 4.5f;
    // Usable maximum lift: the NASA TP-1538 F-16 tables, trimmed, at that
    // jet's 25 deg fly-by-wire limit give CL 1.55 (1.05 is only its 15 deg
    // value). No other card publishes a lift curve, so the one measured
    // fighter stands in for all of them.
    constexpr float kClMax = 1.55f;
    // Shown when the card has no area, so alpha still shrinks as speed rises. Not a wing.
    constexpr float kDisplayAreaM2 = 40.0f;
    constexpr float kRollRateRad = 180.0f * 0.01745329252f;
    constexpr float kMaxSpeedMps = 1300.0f;

    // A point mass obeys a rate or load command at once; an airframe does
    // not. These stand-ins give it the response a fly-by-wire fighter has,
    // so a small aim nudge builds roll and g over a few tenths of a second
    // instead of snapping. They are handling stand-ins, not card data.
    // Roll mode: first-order lag on roll rate, and a roll-acceleration cap.
    constexpr float kRollModeSeconds = 0.22f;
    constexpr float kRollAccelRad = 600.0f * 0.01745329252f;
    // Load follows the command with a short lag and a g-onset limit.
    constexpr float kLoadLagSeconds = 0.16f;
    constexpr float kOnsetGPerSecond = 15.0f;
    // Angle of attack settles to its new trim through the short period.
    constexpr float kShortPeriodSeconds = 0.25f;
    // Dynamic pressure for full roll authority; less below it.
    constexpr float kFullRollPressurePa = 9000.0f;

    float rollAuthority(float dynamicPressure)
    {
        return std::clamp(dynamicPressure / kFullRollPressurePa, 0.3f, 1.0f);
    }

    glm::vec3 rotateAround(const glm::vec3 &vector, const glm::vec3 &axis, float radians)
    {
        const float cosine = std::cos(radians);
        const float sine = std::sin(radians);
        return vector * cosine + glm::cross(axis, vector) * sine + axis * glm::dot(axis, vector) * (1.0f - cosine);
    }

    glm::vec3 unitPerpendicular(const glm::vec3 &vector, const glm::vec3 &axis)
    {
        glm::vec3 perpendicular = vector - axis * glm::dot(vector, axis);
        if (glm::dot(perpendicular, perpendicular) < 1.0e-6f)
        {
            const glm::vec3 helper = std::abs(axis.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
            perpendicular = helper - axis * glm::dot(helper, axis);
        }
        return glm::normalize(perpendicular);
    }
}

void CardAirframe::bind(const AircraftCard *card)
{
    m_card = card;
    m_mass = card == nullptr          ? kStandInMassKg
             : card->flyingMassKg > 0.0f ? card->flyingMassKg
             : card->massKg > 0.0f       ? card->massKg
                                         : kStandInMassKg;
    m_military = card != nullptr ? card->militaryThrustN : 0.0f;
    const float publishedCeiling = card == nullptr ? 0.0f
                                                    : (card->maxThrustN > 0.0f ? card->maxThrustN : card->militaryThrustN);
    m_ceiling = publishedCeiling > 0.0f ? publishedCeiling : m_mass * kGravity * kStandInThrustToWeight;
    m_hasMilitary = m_military > 0.0f && m_ceiling > m_military;
    m_area = card != nullptr && card->areaPublished ? card->wingAreaM2 : 0.0f;
    m_gPos = card != nullptr && card->positiveGPublished ? card->positiveG : kStandInPositiveG;
    m_gNeg = card != nullptr && card->negativeGPublished ? card->negativeG : kStandInNegativeG;
    m_matchSpeed = card != nullptr && card->speedEquality && card->seaLevelSpeedMps > 0.0f;
    m_veq = m_matchSpeed ? card->seaLevelSpeedMps : kStandInSeaLevelMps;
    m_cd0 = kStandInCd0;
    if (m_area > 0.0f && m_matchSpeed)
    {
        const float dynamicPressure = 0.5f * kSeaLevelDensity * m_veq * m_veq;
        const float dynamicForce = dynamicPressure * m_area;
        if (dynamicForce > 1.0f)
        {
            const float liftCoefficient = m_mass * kGravity / dynamicForce;
            m_cd0 = std::clamp(m_ceiling / dynamicForce - kInduced * liftCoefficient * liftCoefficient, 0.012f, 0.08f);
        }
    }
}

void CardAirframe::reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up)
{
    m_position = position;
    m_velocity = velocity;
    const glm::vec3 path = glm::dot(velocity, velocity) > 4.0f ? glm::normalize(velocity)
                                                               : (glm::dot(forward, forward) > 1.0e-8f ? glm::normalize(forward)
                                                                                                        : glm::vec3(0.0f, 0.0f, 1.0f));
    m_lift = unitPerpendicular(glm::dot(up, up) > 1.0e-8f ? up : glm::vec3(0.0f, 1.0f, 0.0f), path);
    m_alpha = 0.0f;
    m_beta = 0.0f;
    m_rates = glm::vec3(0.0f);
    m_rollRate = 0.0f;
    m_load = std::max(glm::dot(m_lift, glm::vec3(0.0f, 1.0f, 0.0f)), 0.0f);
    m_trimmed = false;
    m_attitude = attitudeFromAxes(path, m_lift);
    m_telemetry = {};
    m_telemetry.airspeed = glm::length(velocity);
    m_telemetry.normalLoad = 1.0f;
}

void CardAirframe::setThrottle(float militaryFraction, bool afterburner)
{
    m_throttle = std::clamp(militaryFraction, 0.0f, 1.0f);
    m_afterburner = afterburner;
}

float CardAirframe::enginePower() const
{
    if (!(m_ceiling > 0.0f))
    {
        return 0.0f;
    }
    const float thrust = m_throttle * ((m_hasMilitary && !m_afterburner) ? m_military : m_ceiling);
    return std::clamp(thrust / m_ceiling * 100.0f, 0.0f, 100.0f);
}

void CardAirframe::step(float deltaTime, const AirData &air, float gravity, const PilotCommand &command)
{
    if (!(m_mass > 0.0f) || !(deltaTime > 0.0f) || !std::isfinite(deltaTime))
    {
        return;
    }

    const float g = std::max(gravity, 0.1f);
    const float density = std::max(air.density, 0.0f);
    const float speedOfSound = air.speedOfSound > 1.0f ? air.speedOfSound : 340.0f;
    float speed = glm::length(m_velocity);
    glm::vec3 path = speed > 2.0f ? m_velocity / speed : forward();
    m_lift = unitPerpendicular(m_lift, path);

    const float dynamicPressure = 0.5f * density * speed * speed;

    // Roll: the rate builds toward the command through the roll mode, no
    // faster than the airframe can accelerate it, and both shrink at low q.
    const float authority = rollAuthority(dynamicPressure);
    const float rollLimit = kRollRateRad * authority;
    const float rollCommand = std::clamp(command.rollRate, -rollLimit, rollLimit);
    const float rollBlend = 1.0f - std::exp(-deltaTime / kRollModeSeconds);
    const float rollStep = kRollAccelRad * authority * deltaTime;
    m_rollRate += std::clamp((rollCommand - m_rollRate) * rollBlend, -rollStep, rollStep);
    const float roll = m_rollRate;
    m_lift = unitPerpendicular(rotateAround(m_lift, path, roll * deltaTime), path);

    const glm::vec3 gravityWorld(0.0f, -g, 0.0f);
    const float holdLoad = -glm::dot(gravityWorld, m_lift) / g;
    float load = holdLoad + command.pitchRate * speed / g;
    load = std::clamp(load, std::min(m_gNeg, m_gPos), std::max(m_gPos, 0.1f));
    if (m_area > 0.0f && dynamicPressure > 1.0f)
    {
        const float aeroLoad = kClMax * dynamicPressure * m_area / (m_mass * g);
        load = std::clamp(load, -aeroLoad, aeroLoad);
    }
    else if (!(m_area > 0.0f))
    {
        const float fade = std::clamp(speed / 120.0f, 0.0f, 1.0f);
        load = holdLoad + (load - holdLoad) * fade;
    }

    // The achieved load lags the commanded one and rises no faster than the
    // onset limit.
    const float loadBlend = 1.0f - std::exp(-deltaTime / kLoadLagSeconds);
    const float onsetStep = kOnsetGPerSecond * deltaTime;
    m_load += std::clamp((load - m_load) * loadBlend, -onsetStep, onsetStep);
    load = m_load;

    const float thrust = m_throttle * ((m_hasMilitary && !m_afterburner) ? m_military : m_ceiling);
    float drag = 0.0f;
    if (m_area > 0.0f && dynamicPressure > 0.0f)
    {
        const float liftCoefficient = load * m_mass * g / (dynamicPressure * m_area);
        const float dragCoefficient = m_cd0 + kInduced * liftCoefficient * liftCoefficient;
        drag = dynamicPressure * m_area * std::max(dragCoefficient, 0.0f);
    }
    else if (m_veq > 1.0f)
    {
        const float ratio = speed / m_veq;
        drag = m_ceiling * (density / kSeaLevelDensity) * ratio * ratio;
        drag *= 1.0f + kInduced * std::max(0.0f, load * load - 1.0f);
    }

    const glm::vec3 specificWorld = path * ((thrust - drag) / m_mass) + m_lift * (load * g);
    m_velocity += (specificWorld + gravityWorld) * deltaTime;
    speed = glm::length(m_velocity);
    if (speed > kMaxSpeedMps)
    {
        m_velocity *= kMaxSpeedMps / speed;
        speed = kMaxSpeedMps;
    }
    m_position += m_velocity * deltaTime;
    path = speed > 2.0f ? m_velocity / speed : path;
    m_lift = unitPerpendicular(m_lift, path);

    const float displayArea = m_area > 0.0f ? m_area : kDisplayAreaM2;
    const float liftCoefficient = dynamicPressure > 1.0f ? load * m_mass * g / (dynamicPressure * displayArea) : 0.0f;
    // Alpha is applied to the nose, but it has to move slowly. An instant
    // offset is a pitch error the aim loop over-corrects on the next step.
    // It also settles through the short-period pitch response rather than
    // jumping to the new trim, so a small nudge does not pop the nose.
    const float targetAlpha = std::clamp(liftCoefficient / kClPerRad, -0.26f, 0.44f);
    const float alphaStep = 0.52f * deltaTime;
    const float alphaBlend = 1.0f - std::exp(-deltaTime / kShortPeriodSeconds);
    m_alpha += std::clamp((targetAlpha - m_alpha) * alphaBlend, -alphaStep, alphaStep);
    if (!m_trimmed)
    {
        // A reset places the jet already flying: start at its trim, not with
        // a nose that rises by itself over the first moments.
        m_alpha = targetAlpha;
        m_trimmed = true;
    }
    const float sideslip = std::clamp(command.sideslip, -0.10f, 0.10f);
    const float betaBlend = 1.0f - std::exp(-deltaTime / 0.20f);
    m_beta += (sideslip - m_beta) * betaBlend;

    // Sideslip as the F-16 model defines it, asin(v_body_right / V): positive
    // beta is airflow from the right, so the nose sits left of the path. The
    // instructor's fine-aim rudder relies on that sign to swing the nose onto
    // the aim point.
    const glm::vec3 right = glm::normalize(glm::cross(path, m_lift));
    const glm::vec3 nose = path * (std::cos(m_alpha) * std::cos(m_beta)) + m_lift * (std::sin(m_alpha) * std::cos(m_beta)) -
                           right * std::sin(m_beta);
    m_attitude = attitudeFromAxes(nose, m_lift);
    // Yaw rate is positive nose-right, which is beta falling.
    m_rates = glm::vec3(roll, speed > 5.0f ? (load - holdLoad) * g / speed : 0.0f, -(sideslip - m_beta) / 0.20f);

    m_telemetry.airspeed = speed;
    m_telemetry.alpha = m_alpha;
    m_telemetry.beta = m_beta;
    m_telemetry.mach = speed / speedOfSound;
    m_telemetry.dynamicPressure = 0.5f * density * speed * speed;
    m_telemetry.normalLoad = load;
    m_telemetry.lateralLoad = 0.0f;
    m_telemetry.thrust = thrust;
    m_telemetry.bodyAcceleration = glm::inverse(m_attitude) * (specificWorld + gravityWorld);
}

AirframeView CardAirframe::view() const
{
    AirframeView view;
    view.forward = forward();
    view.right = right();
    view.up = up();
    view.bodyRates = m_rates;
    view.telemetry = m_telemetry;
    view.rollRateLimit = kRollRateRad * rollAuthority(m_telemetry.dynamicPressure);
    return view;
}
}
