#pragma once

#include "FlightControl.h"

#include <glm/glm.hpp>

namespace missilesim::flight
{
    struct InstructorInput
    {
        glm::vec3 aimDirection{0.0f, 0.0f, 1.0f}; // world direction the player wants the nose on
        // Keyboard assists, -1..1. A non-zero axis takes that axis away from
        // the instructor while it is held; the fly-by-wire still protects.
        float pitchKey = 0.0f;
        float rollKey = 0.0f;
        float yawKey = 0.0f;
    };

    struct InstructorStatus
    {
        float angleOff = 0.0f; // rad between nose and aim
    };

    // Mouse-aim "instructor" in the manner War Thunder documents it: the
    // player only says where the nose should go; the instructor flies the
    // aircraft there through the normal controls. It rolls the lift vector
    // onto the acceleration the flight path needs (so: wings level on target,
    // a coordinated bank for a target to the side, roll-and-pull for a big
    // correction), points the nose with a pitch-rate command, adds a touch of
    // rudder for fine aiming. It never overrides the player near the ground.
    // Stall, AoA and g protection come from the fly-by-wire it drives.
    class MouseAimInstructor
    {
    public:
        // Once per input sample, even on frames with no physics steps. Time
        // is in simulation seconds; zero clears motion and rebases the aim.
        void observeAim(const glm::vec3 &aimDirection, float sampleDeltaTime);

        PilotCommand update(const F16Airframe &airframe, const InstructorInput &input, float gravity, float deltaTime);

        const InstructorStatus &status() const { return m_status; }

    private:
        // Manual control engages immediately. On release, return smoothly
        // from the aircraft's achieved rate, not a saturated stick demand.
        struct AxisHandoff
        {
            float apply(float automatic, float manual, float achieved, bool held, float deltaTime);
            bool wasHeld = false;
            float remaining = 0.0f;
            float releaseValue = 0.0f;
        };

        InstructorStatus m_status;
        glm::vec3 m_previousAim{0.0f, 0.0f, 1.0f};
        glm::vec3 m_aimRate{0.0f}; // world angular velocity of the aim, rad/s
        bool m_hasPreviousAim = false;
        float m_rollCommit = 0.0f; // +-1 while a big roll is committed to a direction
        AxisHandoff m_pitchHandoff;
        AxisHandoff m_rollHandoff;
        AxisHandoff m_yawHandoff;
    };
}
