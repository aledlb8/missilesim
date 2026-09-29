#include "MouseAimInstructor.h"

#include <algorithm>
#include <cmath>

#include <glm/gtx/norm.hpp>

namespace missilesim::flight
{
namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    // Nose rotation rate asked per radian of pitch-plane error, 1/s.
    constexpr float kPitchGain = 2.0f;
    // A held pitch key asks more than any limit allows, so it flies the
    // limiter: full aft stick is a max-performance pull, full forward -3 g.
    constexpr float kKeyPitchRate = 1.5f;
    // Roll rate asked per radian of bank error, 1/s.
    constexpr float kRollGain = 1.5f;
    // Flight-path turn rate wanted per radian of aim error when choosing the
    // bank, 1/s. With the pitch and roll gains above it was picked by a sweep
    // in tools/flight_harness (fastest capture with no overshoot past 3 deg).
    constexpr float kBankTurnGain = 1.4f;
    // Sideslip asked per radian of lateral error while fine aiming.
    constexpr float kYawGain = 0.8f;
    // Fine-aim band where the rudder helps, and the band past which the
    // instructor only pulls once the lift vector is on the aim point.
    constexpr float kFineAngle = 3.0f * kPi / 180.0f;
    constexpr float kFineFade = 8.0f * kPi / 180.0f;
    constexpr float kAlignStart = 10.0f * kPi / 180.0f;
    constexpr float kAlignFull = 30.0f * kPi / 180.0f;
    // Aim points further off than this below the nose are reached by rolling
    // and pulling, not by pushing negative g.
    constexpr float kPushAngle = 25.0f * kPi / 180.0f;
    // Roll-direction commitment band.
    constexpr float kCommitAngle = 120.0f * kPi / 180.0f;
    constexpr float kReleaseAngle = 60.0f * kPi / 180.0f;
    // Aim-rate feed-forward low-pass time constant, s.
    constexpr float kAimRateFilter = 0.06f;

    float smoothstep(float edge0, float edge1, float x)
    {
        const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
}

void MouseAimInstructor::observeAim(const glm::vec3 &aimDirection, float deltaTime)
{
    if (glm::length2(aimDirection) < 1.0e-8f || !(deltaTime > 0.0f))
    {
        return;
    }
    const glm::vec3 aim = glm::normalize(aimDirection);
    if (!m_hasPreviousAim)
    {
        m_previousAim = aim;
        m_hasPreviousAim = true;
        return;
    }
    // Small-angle angular velocity of the aim direction.
    const glm::vec3 raw = glm::cross(m_previousAim, aim) / deltaTime;
    const float blend = 1.0f - std::exp(-deltaTime / kAimRateFilter);
    m_aimRate += (raw - m_aimRate) * blend;
    m_previousAim = aim;
}

PilotCommand MouseAimInstructor::update(const F16Airframe &airframe, const InstructorInput &input, float gravity)
{
    const glm::vec3 forward = airframe.forward();
    const glm::vec3 right = airframe.right();
    const glm::vec3 up = airframe.up();
    const float speed = std::max(airframe.telemetry().airspeed, 1.0f);
    const float g = std::max(gravity, 0.1f);

    const glm::vec3 aim = glm::length2(input.aimDirection) > 1.0e-8f ? glm::normalize(input.aimDirection) : forward;
    const glm::vec3 aimRate = m_aimRate;

    // Aim in body terms: x along the nose, y toward the right wing, z over the canopy.
    const float ax = glm::dot(aim, forward);
    const float ay = glm::dot(aim, right);
    const float az = glm::dot(aim, up);
    const float angleOff = std::acos(std::clamp(ax, -1.0f, 1.0f));
    m_status.angleOff = angleOff;

    // ---- Roll: put the lift vector on the acceleration the flight path
    // needs. That is a turn toward the aim point (proportional to the error,
    // plus the aim's own motion) and support against gravity. On the nose it
    // reduces to wings level; off to the side, a coordinated bank; well below,
    // roll inverted and pull.
    glm::vec3 toward = aim - forward * ax;
    toward = glm::length2(toward) > 1.0e-10f ? glm::normalize(toward) : up;
    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 turn = toward * (speed * kBankTurnGain * angleOff) + speed * glm::cross(aimRate, forward);
    const glm::vec3 support = (worldUp - forward * glm::dot(worldUp, forward)) * g;
    glm::vec3 wanted = turn + support;
    wanted -= forward * glm::dot(wanted, forward);
    const float wantedSide = glm::dot(wanted, right);
    const float wantedUp = glm::dot(wanted, up);
    float bankError = std::atan2(wantedSide, wantedUp);
    // A big roll commits to a direction until it is mostly done, so an error
    // passing through +-180 deg mid-roll does not reverse it.
    if (m_rollCommit == 0.0f && std::abs(bankError) > kCommitAngle)
    {
        m_rollCommit = bankError > 0.0f ? 1.0f : -1.0f;
    }
    else if (m_rollCommit != 0.0f && std::abs(bankError) < kReleaseAngle)
    {
        m_rollCommit = 0.0f;
    }
    if (m_rollCommit != 0.0f && bankError * m_rollCommit < 0.0f)
    {
        bankError += m_rollCommit * 2.0f * kPi;
    }
    // With almost no acceleration wanted the direction is noise; ease off.
    // And unload to roll: while the airframe is pulling much harder than the
    // acceleration now wanted, swinging that lift round throws the nose off,
    // so the roll waits for the pull to come off.
    const float wantedMagnitude = glm::length(wanted);
    const float liftNow = std::abs(airframe.telemetry().normalLoad) * g;
    const float unload = std::clamp((wantedMagnitude + g) / std::max(liftNow, g), 0.25f, 1.0f);
    const float authority = smoothstep(0.05f * g, 0.3f * g, wantedMagnitude) * unload;

    // ---- Pitch: rotate the nose toward the aim point in the plane of
    // symmetry. A pitch-rate command makes the nose error decay first-order;
    // the fly-by-wire caps the rate at the AoA and g limits.
    const float pitchError = std::atan2(az, ax);
    float pitchRate = kPitchGain * pitchError + glm::dot(aimRate, right);
    if (pitchError < 0.0f && angleOff > kPushAngle)
    {
        pitchRate = std::max(pitchRate, 0.0f);
    }
    // For big corrections pull only as the lift vector comes round.
    const float alignWeight = smoothstep(kAlignStart, kAlignFull, angleOff);
    pitchRate *= (1.0f - alignWeight) + alignWeight * std::clamp(std::cos(bankError), 0.0f, 1.0f);

    PilotCommand command;
    command.pitchRate = pitchRate;
    command.rollRate = kRollGain * bankError * authority;
    const float lateralError = std::atan2(ay, ax);
    command.sideslip = -kYawGain * lateralError * (1.0f - smoothstep(kFineAngle, kFineFade, angleOff));

    // ---- Keyboard assists own their axis while held (pitch also owns roll,
    // so the instructor does not bank against a manual pull).
    const float rollLimit = glm::radians(FlightControlSystem::kMaxRollRateDeg);
    if (std::abs(input.pitchKey) > 0.01f)
    {
        command.pitchRate = input.pitchKey * kKeyPitchRate;
        command.rollRate = input.rollKey * rollLimit;
    }
    if (std::abs(input.rollKey) > 0.01f)
    {
        command.rollRate = input.rollKey * rollLimit;
    }
    if (std::abs(input.yawKey) > 0.01f)
    {
        command.sideslip = -input.yawKey * glm::radians(FlightControlSystem::kMaxSideslipDeg);
    }
    return command;
}
}
