#pragma once

// Track lifecycle for sensor model 1.
//
// A track is tentative, confirmed, coasting, or lost. Coasting is the life of
// a track whose last look produced no measurement. It is not a radar mode:
// this store has no search, track, or flood state, and it does not steer a beam.
//
// onLook consumes measurements only. A track id identifies the track. It is
// not the platform that was measured, and the estimate has nowhere to put one.
//
// A track is published as Lost once, by the look that lost it, and is dropped
// on the next look. Ids are never reused.

#include "sim/sensors/SensorTypes.h"

#include <functional>
#include <vector>

namespace missilesim::sim
{
    constexpr int kDefaultConfirmHits = 2;
    constexpr double kDefaultCoastLimitS = 4.0;
    // Noiseless position association. Two fighters inside this radius share a
    // track; model 1 has no resolution cell to separate them.
    constexpr float kDefaultAssociationGateM = 2000.0f;

    enum class TrackLife : std::uint8_t
    {
        Tentative,
        Confirmed,
        Coasting,
        Lost,
    };

    struct TrackEstimate
    {
        EntityId id;
        TrackLife life = TrackLife::Tentative;
        double time = 0.0;
        double lastMeasurementTime = 0.0;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        int hitCount = 0;
        int missCount = 0;
    };

    // What one look could have measured. A scanning sensor looks at one beam
    // at a time, so a track outside it was not missed, only not revisited.
    struct LookCoverage
    {
        // Receives each living track's predicted position at the look time.
        // Empty means the look covered every track.
        std::function<bool(const glm::vec3 &predicted)> covers;
        // An uncovered track coasts once its last measurement is older than
        // this. Zero or less coasts it on the first uncovered look.
        double revisitS = 0.0;
    };

    class TrackStore
    {
    public:
        void setConfirmHits(int hits);
        void setCoastLimit(double seconds);
        void setGateRadius(float meters);

        // One scheduled look. An empty list is a look that saw nothing, and
        // every living track coasts or is lost. Calling this between looks
        // would age tracks early, so the scheduler calls it only when a sensor
        // grid time is actually sampled.
        void onLook(double time, const std::vector<Observation> &observations);
        void onLook(double time, const std::vector<Observation> &observations, const LookCoverage &coverage);

        const std::vector<TrackEstimate> &tracks() const { return m_published; }

    private:
        struct Record
        {
            TrackEstimate estimate;
            glm::vec3 measuredPosition{0.0f};
        };

        int m_confirmHits = kDefaultConfirmHits;
        double m_coastLimitS = kDefaultCoastLimitS;
        float m_gateRadiusM = kDefaultAssociationGateM;
        std::uint32_t m_nextId = 1;
        std::vector<Record> m_records;
        std::vector<TrackEstimate> m_published;

        void publish();
    };
}
