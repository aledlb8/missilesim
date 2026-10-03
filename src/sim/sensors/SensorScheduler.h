#pragma once

// Schedules radar and infrared looks on simulation time.
//
// update() is a function of the sample time, the terrain, and the body poses.
// It does not read a camera, a frame rate, or a field of view. A look is
// emitted only when that sensor's grid time has arrived. Sampling twice with
// the same arguments repeats; sampling between grid times emits nothing and
// does not age tracks.
//
// Missed grid times are dropped. The late sample produces one look of the
// pose it was actually given, stamped with that sample's time, because the
// scheduler has no earlier pose to invent.
//
// The two families do not share a propagation term. Each keeps its own tracks.

#include "sim/sensors/SensorTypes.h"
#include "sim/tracking/TrackStore.h"

#include <vector>

namespace missilesim::sim
{
    class Terrain;

    class SensorScheduler
    {
    public:
        // Replacing a set clears that family's clock and tracks.
        // A non-positive revisit leaves the family off.
        void setRadar(const RadarSet &radar, const RadarCrossSectionProfile &signature);
        void setInfrared(const InfraredSet &infrared, const InfraredSignatureProfile &signature);

        TrackStore &radarTracks() { return m_radarTracks; }
        TrackStore &infraredTracks() { return m_infraredTracks; }
        const TrackStore &radarTracks() const { return m_radarTracks; }
        const TrackStore &infraredTracks() const { return m_infraredTracks; }

        SensorProducts update(double time, const Terrain &terrain, const SensorBody &ownship,
                              const std::vector<SensorBody> &bodies);

    private:
        struct Clock
        {
            bool enabled = false;
            double periodS = 1.0;
            double nextS = 0.0;

            bool due(double time) const;
            // Grid times crossed by this sample, including the one that is taken.
            int consume(double time);
        };

        RadarSet m_radar;
        RadarCrossSectionProfile m_radarSignature;
        InfraredSet m_infrared;
        InfraredSignatureProfile m_infraredSignature;
        Clock m_radarClock;
        Clock m_infraredClock;
        TrackStore m_radarTracks;
        TrackStore m_infraredTracks;

        void lookRadar(double time, const Terrain &terrain, const SensorBody &ownship,
                       const std::vector<SensorBody> &bodies, SensorProducts &out) const;
        void lookInfrared(double time, const Terrain &terrain, const SensorBody &ownship,
                          const std::vector<SensorBody> &bodies, SensorProducts &out) const;
    };
}
