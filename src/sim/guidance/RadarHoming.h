#pragma once

// Fictional active-radar homing. Not a named missile.
//
// Midcourse is proportional navigation on a support packet. The packet has no
// entity id. Between packets the aim coasts at that packet's velocity. A
// missing or stale packet is not replaced with a body's true position, and a
// paused datalink does not end the shot.
//
// The seeker may search once the coasted estimate is inside
// seekerActivationRangeM. The cone axis is the missile nose
// (SensorBody.forward), not the velocity. A look is monostaticSnr, the
// terrain line of sight, that cone, and, when seekerGateM is set, a sphere
// around where the seeker expects the target: the coasted support point while
// searching, the extrapolated measurement after that. Acquisition is that
// look, not the shooter's track. After a hit, guidance follows the measured
// position and a velocity taken from successive measurements. Memory keeps
// looking, and a new measurement returns it to terminal.
//
// The seeker is pulse-Doppler (spec.doppler). An echo lost in the ground's
// main-lobe clutter is never seen. While it searches on the support cue and
// while it tracks, only echoes inside the velocity gate around the expected
// closing speed count, so chaff that has stopped in the air is ignored unless
// the target, beaming, has almost no closing speed of its own. In memory the
// gate opens again: the seeker re-searches every speed inside its sphere.
//
// Search timeout and memory timeout both end the shot as
// ShotEndReason::SelfDestruct. Motor burn is a separate bool the caller
// updates from burn time. This guidance never turns a motor on or off.

#include "sim/SimEvents.h"
#include "sim/Terrain.h"
#include "sim/sensors/SensorTypes.h"

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    struct SupportPacket
    {
        double time = 0.0;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        bool fresh = false;
    };

    struct RadarHomingSpec
    {
        float navigationGain = 4.0f;             // clamped to 3..5 when used
        float maxAcceleration = 300.0f;          // m/s^2, a declared cap, not a brochure g
        float datalinkPeriodS = 2.0f;
        float seekerActivationRangeM = 12000.0f; // seeker may search inside this range of the coasted estimate
        float seekerConeHalfRad = 0.5f;          // about the missile nose (body forward)
        float seekerSnrThreshold = 10.0f;
        float seekerGateM = 0.0f;                // acquisition sphere radius about the expected point; 0 is ungated
        DopplerFilter doppler;                   // clutter notch and velocity gate; zero widths are off
        RadarSet seekerRadar;                    // fictional active seeker, same equation as monostaticSnr
        RadarCrossSectionProfile targetRcs;
        double searchTimeoutS = 4.0;
        double memoryTimeoutS = 3.0;
    };

    struct RadarHomingState
    {
        enum class Phase : std::uint8_t
        {
            Midcourse,
            SeekerSearch,
            Terminal,
            Memory,
            Ended,
        };

        Phase phase = Phase::Midcourse;
        // Search timeout and memory timeout use ShotEndReason::SelfDestruct.
        ShotEndReason endReason = ShotEndReason::None;
        SupportPacket lastSupport;
        bool haveSupport = false;
        double phaseStart = 0.0;
        glm::vec3 seekerAimPoint{0.0f}; // last acquired measurement position
        bool seekerTracking = false;
    };

    struct HomingCommand
    {
        glm::vec3 acceleration{0.0f}; // inertial, perpendicular to missile velocity when possible
        RadarHomingState::Phase phase = RadarHomingState::Phase::Midcourse;
        bool supportFresh = false;
        bool seekerOn = false;
        bool seekerTracking = false;
    };

    class RadarHoming
    {
    public:
        void reset(const RadarHomingSpec &spec);

        // packet is null when this step brought no new support. Body poses are
        // read only to synthesize a seeker measurement. After that, guidance
        // follows the measurement, not a body id.
        HomingCommand step(double time, float dt, const glm::vec3 &missilePosition, const glm::vec3 &missileVelocity,
                           const SupportPacket *packet, const Terrain &terrain, const SensorBody &missile,
                           const std::vector<SensorBody> &bodies);

        const RadarHomingState &state() const { return m_state; }

    private:
        void remember(double time, float dt, const glm::vec3 &measuredPosition);
        void endAsSelfDestruct();

        RadarHomingSpec m_spec{};
        RadarHomingState m_state{};
        glm::vec3 m_measuredVelocity{0.0f};
        double m_measurementTime = 0.0;
        bool m_haveMeasurement = false;
    };

    // Prints PASS/FAIL lines. Returns the number of failed checks.
    int runRadarHomingChecks();
}
