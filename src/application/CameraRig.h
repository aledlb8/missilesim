#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Camera rigs. Both produce a full orientation (forward and up), so the view
// never flips when the subject goes through the vertical, and both damp with
// 1 - exp(-lambda * dt) so they behave the same at any frame rate (Rory
// Driscoll, "Frame rate independent damping using lerp", 2016).
struct CameraPose
{
    glm::vec3 position{0.0f};
    glm::vec3 forward{0.0f, 0.0f, 1.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    float fov = 60.0f;
};

// War Thunder style mouse-aim camera, after Brian Hernandez's MouseFlight
// (MIT), a public re-creation of that scheme. The mouse turns an aim
// direction about the camera's own axes; the camera rig follows the aircraft
// position exactly and turns toward the aim with damping, so pointing stays
// precise while the view stays smooth. The rig's up is the horizon, except
// looking steeply up or down, where it keeps its own up instead of flipping.
class MouseAimCamera
{
public:
    void reset(const glm::vec3 &aimDirection);

    // Mouse motion in radians: positive yaw turns right, positive pitch up.
    void turn(float yawRadians, float pitchRadians);

    // Free look: the aim freezes (the aircraft keeps flying to it) and the
    // mouse looks around instead; releasing swings the view back.
    void setFreeLook(bool held);
    void setDistanceScale(float scale) { m_distanceScale = scale; }
    float distanceScale() const { return m_distanceScale; }

    // subject: interpolated aircraft position; size: its bounding radius.
    void update(float deltaTime, const glm::vec3 &subject, float size, float baseFov, float smoothing);

    const glm::vec3 &aimDirection() const { return m_aim; }
    bool isFreeLook() const { return m_freeLook; }
    const CameraPose &pose() const { return m_pose; }

private:
    glm::vec3 m_aim{0.0f, 0.0f, 1.0f};
    glm::vec3 m_look{0.0f, 0.0f, 1.0f};
    glm::quat m_rig{1.0f, 0.0f, 0.0f, 0.0f};
    bool m_freeLook = false;
    float m_distanceScale = 1.0f;
    CameraPose m_pose;
};

// Chase camera for a missile or a target: sits behind the subject's heading,
// turns with it smoothly, and can be orbited (right mouse) with the offset
// easing back when released.
class ChaseCamera
{
public:
    void reset() { m_valid = false; m_yaw = 0.0f; m_pitch = 0.0f; }
    void setOrbiting(bool orbiting) { m_orbiting = orbiting; }
    void orbit(float yawRadians, float pitchRadians);

    void update(float deltaTime, const glm::vec3 &subject, const glm::vec3 &heading,
                float distance, float height, float lookAhead, float fov);

    const CameraPose &pose() const { return m_pose; }

private:
    glm::quat m_frame{1.0f, 0.0f, 0.0f, 0.0f};
    bool m_valid = false;
    bool m_orbiting = false;
    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    CameraPose m_pose;
};
