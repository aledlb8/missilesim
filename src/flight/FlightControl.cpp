#include "FlightControl.h"

#include "F16Data.h"

#include <algorithm>
#include <cmath>

namespace missilesim::flight
{
namespace
{
    // Loop bandwidths, 1/s. Inner (rate) loops sit well inside the 0.0495 s
    // actuators; the outer (AoA, sideslip) loops sit well inside the inner ones.
    constexpr float kAlphaBandwidth = 3.5f;
    constexpr float kSideslipBandwidth = 2.5f;
    constexpr float kPitchRateBandwidth = 9.0f;
    constexpr float kRollRateBandwidth = 7.0f;
    constexpr float kYawRateBandwidth = 6.0f;

    // Keeps the commanded AoA inside the tabulated data on the negative side.
    constexpr float kAlphaFloorDeg = -8.0f;
    constexpr float kSpinPreventionAlphaDeg = 29.0f;
    constexpr float kMinimumControlSpeed = 5.0f;
    // TP-1538 stabilator actuator, used to predict how long a reversal takes.
    constexpr float kElevatorSlewDegPerSec = 60.0f;
    constexpr float kActuatorLagSeconds = 0.0495f;

    float bisect(float lo, float hi, float target, float (*fn)(float, const void *), const void *ctx, bool decreasing)
    {
        for (int i = 0; i < 24; ++i)
        {
            const float mid = 0.5f * (lo + hi);
            const float value = fn(mid, ctx);
            const bool above = decreasing ? value > target : value < target;
            if (above)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }
        return 0.5f * (lo + hi);
    }

    struct CzContext
    {
        float betaDeg;
        float elevatorDeg;
        float pitchDampingPerAlpha; // chord/(2V) * q, multiplied by CZq(alpha) inside
    };

    float totalCz(float alphaDeg, const void *raw)
    {
        const auto &c = *static_cast<const CzContext *>(raw);
        return f16::cz(alphaDeg, c.betaDeg, c.elevatorDeg) + c.pitchDampingPerAlpha * f16::damping(alphaDeg).czq;
    }

    struct CmContext
    {
        float alphaDeg;
    };

    float elevatorCm(float elevatorDeg, const void *raw)
    {
        return f16::cm(static_cast<const CmContext *>(raw)->alphaDeg, elevatorDeg);
    }
}

float FlightControlSystem::rollRateLimit(float dynamicPressure, float alpha)
{
    const float alphaDeg = glm::degrees(alpha);
    const float limitDeg = kMaxRollRateDeg - 0.0115f * std::max(0.0f, 10500.0f - dynamicPressure) -
                           4.0f * std::max(0.0f, alphaDeg - 15.0f);
    return glm::radians(std::max(limitDeg, 80.0f));
}

Surfaces FlightControlSystem::update(const F16Airframe &airframe, const PilotCommand &command, float gravity)
{
    const AirframeTelemetry &t = airframe.telemetry();
    const glm::vec3 &rates = airframe.bodyRates();
    const float speed = t.airspeed;
    const float qbar = t.dynamicPressure;
    if (speed < kMinimumControlSpeed || qbar < 1.0f)
    {
        m_status = {};
        return {};
    }

    const float p = rates.x;
    const float q = rates.y;
    const float r = rates.z;
    const float alpha = t.alpha;
    const float beta = t.beta;
    const float alphaDeg = glm::degrees(alpha);
    const float betaDeg = glm::degrees(beta);
    const float cosAlpha = std::cos(alpha);
    const float sinAlpha = std::sin(alpha);
    const float cosBeta = std::max(std::cos(beta), 0.2f);
    const float tanBeta = std::tan(beta);
    const glm::vec3 &a = t.bodyAcceleration;
    const float qs = qbar * f16::kWingAreaM2;
    const float chordTerm = f16::kChordM / (2.0f * speed);
    const float spanTerm = f16::kSpanM / (2.0f * speed);
    const Surfaces &now = airframe.surfaces();
    const F16Airframe::Inertia &c = airframe.inertia();
    const float he = f16::kEngineMomentum;

    m_status.spinPrevention = alphaDeg > kSpinPreventionAlphaDeg;
    m_status.rollRateLimit = rollRateLimit(qbar, alpha);

    // ---- Pitch: rate command, bounded by the AoA and g limiters.
    // Each limit is turned into the AoA that reaches it, then into the pitch
    // rate that drives AoA there with the limiter bandwidth, from
    // alpha_dot = q - tan(beta)(p cos a + r sin a) + (az cos a - ax sin a) / (V cos b).
    const CzContext czContext{betaDeg, now.elevator, chordTerm * q};
    const float weightOverQs = airframe.mass() * std::max(gravity, 0.1f) / qs;
    const auto alphaForLoad = [&](float load) {
        const float czRequired = -load * weightOverQs;
        if (czRequired >= totalCz(kAlphaFloorDeg, &czContext))
        {
            return kAlphaFloorDeg;
        }
        if (czRequired <= totalCz(40.0f, &czContext))
        {
            return 40.0f;
        }
        return bisect(kAlphaFloorDeg, 40.0f, czRequired, totalCz, &czContext, true);
    };
    const float alphaCeilingByLoad = alphaForLoad(kMaxNormalLoad);
    const float alphaFloorByLoad = alphaForLoad(kMinNormalLoad);
    const float alphaCeiling = glm::radians(std::min(kAlphaLimitDeg, alphaCeilingByLoad));
    const float alphaFloor = glm::radians(std::max(kAlphaFloorDeg, alphaFloorByLoad));
    const float kinematicPitch = tanBeta * (p * cosAlpha + r * sinAlpha) - (a.z * cosAlpha - a.x * sinAlpha) / (speed * cosBeta);

    // Approach each AoA bound no faster than the elevator can stop it.
    // Without this a low-speed pull overshoots into the deep-stall region
    // above 40 deg, where the F-16 data have almost no nose-down moment left.
    // Stopping model: the elevator slews to the opposite stop at its rate
    // limit, so the stopping pitch acceleration changes linearly from its
    // value at the current deflection (negative while the surface is still
    // pulling toward the bound) to the net full-authority value, then holds.
    // The largest rate that still stops inside the AoA margin is found by
    // bisection on that stopping distance.
    const float pitchAuthority = c.c7 * qs * f16::kChordM;
    const auto approachRate = [&](float bound, float stopElevator) {
        const float toward = bound > alpha ? 1.0f : -1.0f;
        const float boundDeg = glm::degrees(bound);
        const float rampTime = std::max(std::abs(stopElevator - now.elevator), 0.5f) / kElevatorSlewDegPerSec;
        const float startDecel = -toward * pitchAuthority * f16::cm(boundDeg, now.elevator);
        const float fullDecel = std::max(-toward * pitchAuthority * f16::cm(boundDeg, stopElevator), 0.05f);
        const float jerk = std::max((fullDecel - startDecel) / rampTime, 1.0e-3f);
        const auto stoppingDistance = [&](float rate) {
            // Phase 1: decel d0 + j t while the surface swings.
            const float disc = startDecel * startDecel + 2.0f * jerk * rate;
            const float stopTime = (-startDecel + std::sqrt(std::max(disc, 0.0f))) / jerk;
            if (stopTime <= rampTime)
            {
                return rate * stopTime - 0.5f * startDecel * stopTime * stopTime - jerk * stopTime * stopTime * stopTime / 6.0f;
            }
            // Phase 2: constant full authority.
            const float t = rampTime;
            const float left = rate - startDecel * t - 0.5f * jerk * t * t;
            return rate * t - 0.5f * startDecel * t * t - jerk * t * t * t / 6.0f + left * left / (2.0f * fullDecel);
        };
        const float gap = std::abs(bound - alpha);
        const float margin = std::max(gap - std::abs(q - kinematicPitch) * kActuatorLagSeconds, 0.0f);
        float lo = 0.0f;
        float hi = 6.0f;
        for (int i = 0; i < 18; ++i)
        {
            const float mid = 0.5f * (lo + hi);
            (stoppingDistance(mid) <= margin ? lo : hi) = mid;
        }
        const float direct = kAlphaBandwidth * (bound - alpha);
        return toward > 0.0f ? std::min(direct, lo) : std::max(direct, -lo);
    };
    const float pitchCeiling = approachRate(alphaCeiling, f16::kElevatorLimitDeg) + kinematicPitch;
    const float pitchFloor = std::min(approachRate(alphaFloor, -f16::kElevatorLimitDeg) + kinematicPitch, pitchCeiling);
    const float pitchRateCommand = std::clamp(command.pitchRate, pitchFloor, pitchCeiling);
    m_status.pitchRateCommand = pitchRateCommand;
    m_status.alphaLimited = command.pitchRate > pitchCeiling && kAlphaLimitDeg <= alphaCeilingByLoad;
    m_status.loadLimited = (command.pitchRate > pitchCeiling && alphaCeilingByLoad < kAlphaLimitDeg) ||
                           (command.pitchRate < pitchFloor && alphaFloorByLoad > kAlphaFloorDeg);
    const float pitchAccel = kPitchRateBandwidth * (pitchRateCommand - q);
    const float cmRequired = (pitchAccel - (c.c5 * p - c.c7 * he) * r - c.c6 * (r * r - p * p)) / (c.c7 * qs * f16::kChordM);
    const float cmFromElevator = cmRequired - chordTerm * f16::damping(alphaDeg).cmq * q;
    const CmContext cmContext{alphaDeg};
    Surfaces out;
    if (cmFromElevator >= f16::cm(alphaDeg, -f16::kElevatorLimitDeg))
    {
        out.elevator = -f16::kElevatorLimitDeg;
    }
    else if (cmFromElevator <= f16::cm(alphaDeg, f16::kElevatorLimitDeg))
    {
        out.elevator = f16::kElevatorLimitDeg;
    }
    else
    {
        out.elevator = bisect(-f16::kElevatorLimitDeg, f16::kElevatorLimitDeg, cmFromElevator, elevatorCm, &cmContext, true);
    }

    // ---- Lateral-directional: stability-axis roll rate + sideslip command.
    float rollCommand = std::clamp(command.rollRate, -m_status.rollRateLimit, m_status.rollRateLimit);
    const float pedalFade = std::clamp((30.0f - alphaDeg) / 10.0f, 0.0f, 1.0f);
    float sideslipCommand = std::clamp(command.sideslip, -glm::radians(kMaxSideslipDeg), glm::radians(kMaxSideslipDeg)) * pedalFade;
    float bodyRollCommand = rollCommand * cosAlpha;
    float yawRateCommand = 0.0f;
    if (m_status.spinPrevention)
    {
        // Roll CAS disengaged; the surfaces only oppose a yaw-rate build-up.
        bodyRollCommand = 0.0f;
        sideslipCommand = 0.0f;
    }
    else
    {
        // beta_dot = p sin a - r cos a + (ay - sin b * Vdot) / (V cos b)
        const float speedRate = a.x * cosAlpha * cosBeta + a.y * std::sin(beta) + a.z * sinAlpha * cosBeta;
        const float betaRateWanted = kSideslipBandwidth * (sideslipCommand - beta);
        yawRateCommand = (p * sinAlpha + (a.y - std::sin(beta) * speedRate) / (speed * cosBeta) - betaRateWanted) /
                         std::max(cosAlpha, 0.2f);
    }

    const float rollAccel = kRollRateBandwidth * (bodyRollCommand - p);
    const float yawAccel = kYawRateBandwidth * (yawRateCommand - r);
    const float rollMomentTerm = rollAccel - (c.c2 * p + c.c1 * r + c.c4 * he) * q;
    const float yawMomentTerm = yawAccel - (c.c8 * p - c.c2 * r + c.c9 * he) * q;
    const float determinant = c.c3 * c.c9 - c.c4 * c.c4;
    const float rollMoment = (rollMomentTerm * c.c9 - yawMomentTerm * c.c4) / determinant;
    const float yawMoment = (yawMomentTerm * c.c3 - rollMomentTerm * c.c4) / determinant;
    const float qsb = qs * f16::kSpanM;
    const f16::Damping d = f16::damping(alphaDeg);
    const float clNeeded = rollMoment / qsb - f16::cl(alphaDeg, betaDeg) - spanTerm * (d.clr * r + d.clp * p);
    const float cnNeeded = yawMoment / qsb - f16::cn(alphaDeg, betaDeg) - spanTerm * (d.cnr * r + d.cnp * p);

    // [dlda/20 dldr/30; dnda/20 dndr/30] [ail; rdr] = [cl; cn]
    const float l1 = f16::dlda(alphaDeg, betaDeg) / 20.0f;
    const float l2 = f16::dldr(alphaDeg, betaDeg) / 30.0f;
    const float n1 = f16::dnda(alphaDeg, betaDeg) / 20.0f;
    const float n2 = f16::dndr(alphaDeg, betaDeg) / 30.0f;
    const float controlDet = l1 * n2 - l2 * n1;
    if (std::abs(controlDet) > 1.0e-7f)
    {
        out.aileron = (clNeeded * n2 - cnNeeded * l2) / controlDet;
        out.rudder = (cnNeeded * l1 - clNeeded * n1) / controlDet;
    }
    out.aileron = std::clamp(out.aileron, -f16::kAileronLimitDeg, f16::kAileronLimitDeg);
    out.rudder = std::clamp(out.rudder, -f16::kRudderLimitDeg, f16::kRudderLimitDeg);
    return out;
}
}
