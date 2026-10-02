#pragma once

#include "CardAirframe.h"
#include "F16Airframe.h"
#include "FlightControl.h"
#include "MouseAimInstructor.h"

namespace missilesim::flight
{
    struct JetControls
    {
        InstructorInput instructor;
        float throttle = 0.8f;   // 0..1 idle to military
        bool afterburner = false; // full afterburner, overrides the lever on the F-16
    };

    // Airframe + fly-by-wire + mouse-aim instructor, stepped together at a
    // fixed inner rate. The F-16 card uses the NASA TP-1538 airframe. Every
    // other card uses CardAirframe. The game and tools/flight_harness both
    // fly this.
    class Jet
    {
    public:
        // Block 50 combat mass from research_notes flight-model.md: empty
        // 18,900 lb + half of 7,000 lb fuel + two 186 lb AIM-9X + 200 lb pilot.
        static constexpr float kCombatMassKg = 22972.0f * 0.45359237f;
        static constexpr float kInnerStepSeconds = 1.0f / 400.0f;

        Jet();
        explicit Jet(const char *aircraftId);

        // Switches the whole flight model. Kinematics stay until reset().
        void configure(const char *aircraftId);
        const char *aircraftId() const { return m_id; }
        bool usesTableModel() const { return m_table; }

        void reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up);
        // Submit once per input sample, independently of the physics step.
        // sampleDeltaTime is elapsed simulation time between input samples;
        // zero rebases the aim without motion (reset, pause or camera change).
        void setControls(const JetControls &controls, float sampleDeltaTime);
        const JetControls &controls() const { return m_controls; }

        void step(float deltaTime, const AirData &air, float gravity);

        const glm::vec3 &position() const;
        const glm::vec3 &velocity() const;
        glm::quat attitude() const;
        glm::vec3 forward() const;
        glm::vec3 right() const;
        glm::vec3 up() const;
        const glm::vec3 &bodyRates() const;
        const AirframeTelemetry &telemetry() const;
        float mass() const;
        float enginePower() const;
        AirframeView view() const;

        // The NASA TP-1538 body. Valid while usesTableModel() is true.
        const F16Airframe &airframe() const { return m_airframe; }
        const FlightControlSystem &flightControl() const { return m_flcs; }
        const MouseAimInstructor &instructor() const { return m_instructor; }
        const PilotCommand &lastCommand() const { return m_command; }

    private:
        const char *m_id = "f-16c-block-50";
        bool m_table = true;
        F16Airframe m_airframe;
        CardAirframe m_card;
        FlightControlSystem m_flcs;
        MouseAimInstructor m_instructor;
        JetControls m_controls;
        PilotCommand m_command;
    };
}
