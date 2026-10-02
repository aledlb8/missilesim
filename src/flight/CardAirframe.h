#pragma once

#include "AircraftCatalog.h"
#include "F16Airframe.h"
#include "MouseAimInstructor.h"

namespace missilesim::flight
{
    // Point-mass flight for a brochure card. Thrust, mass, area, load, and a
    // sea-level speed come from that card when the cell is filled. Empty cells
    // use the stand-ins named in AircraftCatalog.h. This is not the F-16
    // polar, and it does not invent an Oswald efficiency.
    class CardAirframe
    {
    public:
        void bind(const AircraftCard *card);
        void reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up);
        // militaryFraction is 0..1 of the military rating when the card has
        // one, otherwise of the single ceiling. Afterburner selects the
        // maximum rating and does not invent a higher one.
        void setThrottle(float militaryFraction, bool afterburner);
        void step(float deltaTime, const AirData &air, float gravity, const PilotCommand &command);

        const glm::vec3 &position() const { return m_position; }
        const glm::vec3 &velocity() const { return m_velocity; }
        const glm::quat &attitude() const { return m_attitude; }
        const glm::vec3 &bodyRates() const { return m_rates; }
        float mass() const { return m_mass; }
        float enginePower() const;
        const AirframeTelemetry &telemetry() const { return m_telemetry; }

        glm::vec3 forward() const { return m_attitude * glm::vec3(1.0f, 0.0f, 0.0f); }
        glm::vec3 right() const { return m_attitude * glm::vec3(0.0f, 1.0f, 0.0f); }
        glm::vec3 up() const { return m_attitude * glm::vec3(0.0f, 0.0f, -1.0f); }

        AirframeView view() const;

    private:
        const AircraftCard *m_card = nullptr;
        float m_mass = 0.0f;
        float m_military = 0.0f;
        float m_ceiling = 0.0f;
        bool m_hasMilitary = false;
        float m_area = 0.0f;
        float m_gPos = kStandInPositiveG;
        float m_gNeg = kStandInNegativeG;
        float m_veq = kStandInSeaLevelMps;
        bool m_matchSpeed = false;
        float m_cd0 = kStandInCd0;

        glm::vec3 m_position{0.0f};
        glm::vec3 m_velocity{0.0f};
        glm::vec3 m_lift{0.0f, 1.0f, 0.0f};
        glm::quat m_attitude{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 m_rates{0.0f};
        float m_rollRate = 0.0f; // achieved roll rate about the flight path, rad/s
        float m_load = 1.0f;     // achieved normal load, g
        bool m_trimmed = false;  // angle of attack set to trim on the first step
        float m_alpha = 0.0f;
        float m_beta = 0.0f;
        float m_throttle = 0.85f;
        bool m_afterburner = false;
        AirframeTelemetry m_telemetry;
    };
}
