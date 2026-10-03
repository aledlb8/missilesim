#pragma once

// Radar-warning and missile-approach picture for one defended aircraft.
//
// An RWR hears radar emissions only. An infrared emission is never a radar
// hit, and a radar emission is never an approach warning. Approach warnings
// come from closing bodies, and only when that aircraft has MAWS fitted.
// Terrain blocks an emission the same way it blocks a sensor look.

#include "sim/Terrain.h"
#include "sim/sensors/SensorTypes.h"

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    enum class WarningKind : std::uint8_t
    {
        RadarSearch,
        RadarTrack,
        RadarLaunch, // a held beam that is also guiding a round in the air
        MissileSeeker,
        Approach,
    };

    struct Emission
    {
        EntityId source;
        glm::vec3 position{0.0f};
        SensorFamily family = SensorFamily::Radar; // Radar or Infrared
        bool missileSeeker = false;                 // aircraft radar vs a missile's own transmitter
        bool singleTargetTrack = false;             // aircraft radar in STT
        bool guidingMissile = false;                // STT that also carries a round's datalink
    };

    struct Warning
    {
        WarningKind kind;
        EntityId source;
        double time = 0.0;
        float azimuthRad = 0.0f; // positive toward the defended aircraft's right wing
        float rangeM = 0.0f;
    };

    struct WarningPicture
    {
        std::vector<Warning> radar;    // RWR only
        std::vector<Warning> approach; // MAWS only
    };

    struct ClosingBody
    {
        EntityId id;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        // An infrared missile still produces MAWS when the set has it, and it
        // never produces an RWR hit. RWR entries come only from radar emissions.
        bool infraredOnly = false;
    };

    struct WarningSet
    {
        bool radarWarning = true;
        bool approachWarning = false;        // MAWS fitted
        float approachConeHalfRad = 1.0f;    // about the tail (opposite the nose)
        float approachMaxRangeM = 8000.0f;
    };

    // Azimuth matches the sensor axes (sensorAxes): right = forward × up, the
    // right wing, and azimuth = atan2(dot(los, right), dot(los, forward)).
    WarningPicture hear(double time, const WarningSet &set, const SensorBody &ownship, const Terrain &terrain,
                        const std::vector<Emission> &emissions, const std::vector<ClosingBody> &closing);

    // False until `now` reaches firstWarningTime + reactionDelayS. A negative
    // firstWarningTime means nothing has been heard yet.
    bool defenseAllowed(double firstWarningTime, double now, double reactionDelayS);

    // Contract checks for warnings and chaff. Returns the number of failures.
    int runDefenseChecks();
}
