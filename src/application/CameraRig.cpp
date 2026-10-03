#include "CameraRig.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

namespace
{
    const glm::vec3 kWorldUp(0.0f, 1.0f, 0.0f);

    // Chase frame follows the subject's heading with this rate, 1/s.
    constexpr float kChaseFollowRate = 5.0f;
    // Released orbit offsets ease back with this rate, 1/s.
    constexpr float kOrbitReturnRate = 2.5f;
    // Mouse-aim rig placement, in multiples of the aircraft's radius.
    constexpr float kAimCameraDistance = 5.0f;
    constexpr float kAimCameraHeight = 1.1f;

    float damp(float rate, float deltaTime)
    {
        return 1.0f - std::exp(-rate * std::max(deltaTime, 0.0f));
    }

    float smoothstep(float edge0, float edge1, float x)
    {
        const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    glm::vec3 unitOr(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        return glm::length2(v) > 1.0e-10f ? glm::normalize(v) : fallback;
    }

    // Rig convention (OpenGL camera): local -z forward, +y up, +x right.
    glm::vec3 rigForward(const glm::quat &q) { return q * glm::vec3(0.0f, 0.0f, -1.0f); }
    glm::vec3 rigUp(const glm::quat &q) { return q * glm::vec3(0.0f, 1.0f, 0.0f); }
    glm::vec3 rigRight(const glm::quat &q) { return q * glm::vec3(1.0f, 0.0f, 0.0f); }

    glm::quat lookRotation(const glm::vec3 &forward, const glm::vec3 &up)
    {
        const glm::vec3 f = unitOr(forward, glm::vec3(0.0f, 0.0f, -1.0f));
        glm::vec3 u = up - f * glm::dot(up, f);
        if (glm::length2(u) < 1.0e-8f)
        {
            const glm::vec3 helper = std::abs(f.y) < 0.9f ? kWorldUp : glm::vec3(0.0f, 0.0f, 1.0f);
            u = helper - f * glm::dot(helper, f);
        }
        u = glm::normalize(u);
        const glm::vec3 r = glm::cross(f, u);
        return glm::normalize(glm::quat_cast(glm::mat3(r, u, -f)));
    }

    // Up for a view along `forward`: the horizon normally; blending to the
    // rig's current up when looking steeply up or down, where the horizon's
    // up is undefined and following it would spin the view (MouseFlight
    // switches at |forward.y| > 0.9; blending avoids the step).
    glm::vec3 horizonUp(const glm::vec3 &forward, const glm::vec3 &currentUp)
    {
        const float steep = smoothstep(0.82f, 0.95f, std::abs(forward.y));
        const glm::vec3 world = kWorldUp - forward * glm::dot(kWorldUp, forward);
        const glm::vec3 current = currentUp - forward * glm::dot(currentUp, forward);
        const glm::vec3 blended = world * (1.0f - steep) + current * steep;
        if (glm::length2(blended) > 1.0e-6f)
        {
            return glm::normalize(blended);
        }
        return unitOr(current, unitOr(world, glm::vec3(0.0f, 0.0f, 1.0f)));
    }
}

void MouseAimCamera::reset(const glm::vec3 &aimDirection)
{
    m_aim = unitOr(aimDirection, glm::vec3(0.0f, 0.0f, 1.0f));
    m_look = m_aim;
    m_rig = lookRotation(m_aim, horizonUp(m_aim, kWorldUp));
    m_freeLook = false;
}

void MouseAimCamera::turn(float yawRadians, float pitchRadians)
{
    // About the camera's own axes, in world space, so screen-right always
    // moves the aim screen-right whatever the rig's roll.
    glm::vec3 &target = m_freeLook ? m_look : m_aim;
    target = glm::angleAxis(-yawRadians, rigUp(m_rig)) * target;
    target = glm::angleAxis(pitchRadians, rigRight(m_rig)) * target;
    target = unitOr(target, rigForward(m_rig));
}

void MouseAimCamera::setFreeLook(bool held)
{
    if (held && !m_freeLook)
    {
        m_look = m_aim;
    }
    m_freeLook = held;
}

void MouseAimCamera::setAim(const glm::vec3 &aimDirection)
{
    m_aim = unitOr(aimDirection, m_aim);
}

void MouseAimCamera::update(float deltaTime, const glm::vec3 &subject, float size, float baseFov, float smoothing)
{
    const glm::vec3 target = m_freeLook ? m_look : m_aim;
    const glm::quat goal = lookRotation(target, horizonUp(target, rigUp(m_rig)));
    m_rig = glm::normalize(glm::slerp(m_rig, goal, damp(smoothing, deltaTime)));

    const glm::vec3 forward = rigForward(m_rig);
    const glm::vec3 up = rigUp(m_rig);
    const float radius = std::max(size, 1.0f);
    m_pose.position = subject - forward * (radius * kAimCameraDistance * m_distanceScale) + up * (radius * kAimCameraHeight * m_distanceScale);
    m_pose.forward = forward;
    m_pose.up = up;
    m_pose.fov = baseFov;
}

void ChaseCamera::orbit(float yawRadians, float pitchRadians)
{
    m_yaw = std::remainder(m_yaw + yawRadians, glm::two_pi<float>());
    m_pitch = std::clamp(m_pitch + pitchRadians, -1.3f, 1.3f);
}

void ChaseCamera::update(float deltaTime, const glm::vec3 &subject, const glm::vec3 &heading,
                         float distance, float height, float lookAhead, float fov)
{
    const glm::vec3 direction = unitOr(heading, m_valid ? rigForward(m_frame) : glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::quat goal = lookRotation(direction, horizonUp(direction, m_valid ? rigUp(m_frame) : kWorldUp));
    if (!m_valid)
    {
        m_frame = goal;
        m_valid = true;
    }
    else
    {
        m_frame = glm::normalize(glm::slerp(m_frame, goal, damp(kChaseFollowRate, deltaTime)));
    }
    if (!m_orbiting)
    {
        const float keep = 1.0f - damp(kOrbitReturnRate, deltaTime);
        m_yaw *= keep;
        m_pitch *= keep;
    }

    const glm::vec3 forward = rigForward(m_frame);
    const glm::vec3 up = rigUp(m_frame);
    const glm::quat offset = glm::angleAxis(-m_yaw, up) * glm::angleAxis(m_pitch, rigRight(m_frame));
    const glm::vec3 viewDirection = offset * forward;
    const glm::vec3 viewUp = offset * up;
    m_pose.position = subject - viewDirection * distance + viewUp * height;
    m_pose.forward = unitOr(subject + viewDirection * lookAhead - m_pose.position, viewDirection);
    m_pose.up = unitOr(viewUp - m_pose.forward * glm::dot(viewUp, m_pose.forward), up);
    m_pose.fov = fov;
}
