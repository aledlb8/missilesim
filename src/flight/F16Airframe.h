#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace missilesim::flight
{
    // Air at the aircraft, from Atmosphere::sample.
    struct AirData
    {
        float density = 1.225f;
        float speedOfSound = 340.3f;
    };

    // Surface positions in degrees, Stevens & Lewis sign convention:
    // positive elevator is trailing edge down (nose down), positive aileron
    // rolls left, positive rudder yaws left.
    struct Surfaces
    {
        float elevator = 0.0f;
        float aileron = 0.0f;
        float rudder = 0.0f;
    };

    // Quantities the flight controls and the HUD read after each step.
    struct AirframeTelemetry
    {
        float airspeed = 0.0f;       // m/s
        float alpha = 0.0f;          // rad
        float beta = 0.0f;           // rad
        float mach = 0.0f;
        float dynamicPressure = 0.0f; // Pa
        float normalLoad = 1.0f;     // Nz, g, positive pulling up
        float lateralLoad = 0.0f;    // Ny, g
        float thrust = 0.0f;         // N
        // Specific force plus gravity in body axes (x fwd, y right, z down):
        // the non-rotational part of the body-axis velocity derivative.
        glm::vec3 bodyAcceleration{0.0f};
    };

    // Rigid-body six-degree-of-freedom F-16 (NASA TP-1538 low-speed data).
    // World axes are the simulator's (y up). Body axes are x forward, y right
    // wing, z down; the attitude quaternion rotates body vectors into world.
    class F16Airframe
    {
    public:
        explicit F16Airframe(float massKg);

        void reset(const glm::vec3 &position, const glm::vec3 &velocity, const glm::vec3 &forward, const glm::vec3 &up);

        // Commands go through TP-1538's first-order actuators with rate limits.
        void setSurfaceCommand(const Surfaces &command) { m_surfaceCommand = command; }
        // Lever 0..1; 0.77 is military power, 1.0 full afterburner.
        void setThrottle(float lever);

        void step(float deltaTime, const AirData &air, float gravity);

        const glm::vec3 &position() const { return m_position; }
        const glm::vec3 &velocity() const { return m_velocity; }
        const glm::quat &attitude() const { return m_attitude; }
        // Body rates p, q, r in rad/s.
        const glm::vec3 &bodyRates() const { return m_rates; }
        const Surfaces &surfaces() const { return m_surfaces; }
        float enginePower() const { return m_power; }
        float throttle() const { return m_throttle; }
        float mass() const { return m_mass; }
        const AirframeTelemetry &telemetry() const { return m_telemetry; }

        glm::vec3 forward() const { return m_attitude * glm::vec3(1.0f, 0.0f, 0.0f); }
        glm::vec3 right() const { return m_attitude * glm::vec3(0.0f, 1.0f, 0.0f); }
        glm::vec3 up() const { return m_attitude * glm::vec3(0.0f, 0.0f, -1.0f); }
        glm::vec3 toBody(const glm::vec3 &world) const { return glm::inverse(m_attitude) * world; }

        // Rigid-body inertia terms in Stevens & Lewis form (c1..c9), SI units.
        struct Inertia
        {
            float c1, c2, c3, c4, c5, c6, c7, c8, c9;
        };
        const Inertia &inertia() const { return m_inertia; }

    private:
        struct Loads
        {
            glm::vec3 force{0.0f};  // body axes, N, aerodynamic + thrust
            glm::vec3 moment{0.0f}; // body axes, N m
        };

        Loads computeLoads(const AirData &air, float altitudeFt);
        void stepActuators(float deltaTime);

        float m_mass;
        Inertia m_inertia{};

        glm::vec3 m_position{0.0f};
        glm::vec3 m_velocity{0.0f};
        glm::quat m_attitude{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 m_rates{0.0f};
        float m_power = 30.0f;
        float m_throttle = 0.5f;
        Surfaces m_surfaces;
        Surfaces m_surfaceCommand;
        AirframeTelemetry m_telemetry;
    };

    // Quaternion whose body axes (x fwd, y right, z down) map onto the given
    // world forward / up directions.
    glm::quat attitudeFromAxes(const glm::vec3 &forward, const glm::vec3 &up);
}
