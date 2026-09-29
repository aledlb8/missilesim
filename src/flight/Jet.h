#pragma once

#include "F16Airframe.h"
#include "FlightControl.h"
#include "MouseAimInstructor.h"

namespace missilesim::flight
{
    struct JetControls
    {
        InstructorInput instructor;
        float throttle = 0.8f;   // 0..1 idle to military
        bool afterburner = false; // full afterburner, overrides the lever
    };

    // Airframe + fly-by-wire + mouse-aim instructor, stepped together at a
    // fixed inner rate. The game and tools/flight_harness both fly this.
    class Jet
    {
    public:
        // Block 50 combat mass from research_notes flight-model.md: empty
        // 18,900 lb + half of 7,000 lb fuel + two 186 lb AIM-9X + 200 lb pilot.
        static constexpr float kCombatMassKg = 22972.0f * 0.45359237f;
        static constexpr float kInnerStepSeconds = 1.0f / 400.0f;

        Jet();

        void reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up);
        // Submit once per input sample, independently of the physics step.
        // sampleDeltaTime is elapsed simulation time between input samples;
        // zero rebases the aim without motion (reset, pause or camera change).
        void setControls(const JetControls &controls, float sampleDeltaTime);
        const JetControls &controls() const { return m_controls; }

        void step(float deltaTime, const AirData &air, float gravity);

        const F16Airframe &airframe() const { return m_airframe; }
        const FlightControlSystem &flightControl() const { return m_flcs; }
        const MouseAimInstructor &instructor() const { return m_instructor; }
        const PilotCommand &lastCommand() const { return m_command; }

    private:
        F16Airframe m_airframe;
        FlightControlSystem m_flcs;
        MouseAimInstructor m_instructor;
        JetControls m_controls;
        PilotCommand m_command;
    };
}
