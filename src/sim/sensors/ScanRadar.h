#pragma once

// Fictional scanning radar: one raster search and single-target track.
//
// The beam steps one beamwidth across azimuth, then the next bar. The frame
// is that many dwells, so a wider azimuth is a longer revisit. Single-target
// track leaves the raster and holds the beam on the designated track's
// predicted bearing every dwell, out to the antenna's gimbal limit. Past it the
// beam drops back to the raster. Clearing the designation returns to search.
// A dwell only ages the tracks inside its beam; the rest coast once a frame
// passes without a measurement. Azimuth is positive toward the right wing.
//
// The radar is pulse-Doppler (setDoppler): an echo in the ground's main-lobe
// clutter notch is never measured, in search or track, and a held beam only
// takes echoes inside the velocity gate around its track's closing speed, so
// chaff released by the target does not pull the track off it.
//
// Detection is the sensor model's one-pulse SNR and its temporary threshold.
// Angle sigma is the boresight monopulse term. The point is not moved by it,
// and range sigma stays 0 because no waveform bandwidth was sourced.

#include "sim/sensors/SensorTypes.h"
#include "sim/tracking/TrackStore.h"

#include <vector>

namespace missilesim::sim
{
    class Terrain;

    struct ScanVolume
    {
        float azimuthHalfRad = 0.0f;
        float elevationHalfRad = 0.0f;
        float beamwidthRad = 0.0f;
        int bars = 1;
        double dwellS = 0.0;
        // How far off the nose (azimuth and elevation) a held beam may point.
        // The raster only covers the search volume; single-target track may
        // follow its contact out to the antenna's limit. Zero, or less than
        // the volume, keeps the track inside the search volume.
        float gimbalHalfRad = 0.0f;
    };

    struct BeamPoint
    {
        float azimuthRad = 0.0f;
        float elevationRad = 0.0f;
        int bar = 0;
        int column = 0;
        double frameS = 0.0;
    };

    double searchFrameSeconds(const ScanVolume &volume);

    // Search raster at this simulation time. The pattern repeats every frame.
    BeamPoint beamAt(const ScanVolume &volume, double time);

    // Confirmed, and the last measurement is no older than maxAgeSeconds.
    // Coasting, tentative, and lost are not a launch solution.
    bool isLaunchQuality(const TrackEstimate &track, double now, double maxAgeSeconds);

    class ScanRadar
    {
    public:
        void setRadar(const RadarSet &radar, const RadarCrossSectionProfile &signature);
        void setVolume(const ScanVolume &volume);
        void setDoppler(const DopplerFilter &filter) { m_doppler = filter; }
        void setDesignatedTrack(EntityId trackId);

        EntityId designatedTrack() const { return m_designated; }

        TrackStore &tracks() { return m_tracks; }
        const TrackStore &tracks() const { return m_tracks; }
        const BeamPoint &beam() const { return m_beam; }
        // The last dwell held the beam on the designation instead of the raster.
        bool singleTargetTrack() const { return m_beam.bar < 0; }

        // True when this transmitter, at the given pose, puts energy on the
        // point: anywhere in the scan volume while searching, inside the held
        // beam in single-target track. A receiver's view, so it reads geometry.
        bool illuminates(const SensorBody &ownship, const glm::vec3 &point) const;

        SensorProducts update(double time, const Terrain &terrain, const SensorBody &ownship,
                              const std::vector<SensorBody> &bodies);

    private:
        struct Clock
        {
            bool enabled = false;
            double periodS = 1.0;
            double nextS = 0.0;

            bool due(double time) const;
            // Dwells crossed by this sample, including the one that is taken.
            int consume(double time);
        };

        void restartClock();
        // True while the designation is still tentative, confirmed, or
        // coasting and inside the gimbal. gateClosing is set when the track
        // has a velocity (two hits or more): it receives the predicted closing
        // speed the velocity gate is centred on.
        bool steerToDesignated(double time, const SensorBody &ownship, BeamPoint &beam, bool &gateClosing,
                               float &expectedClosing) const;
        // expectedClosing is null while searching: no velocity gate.
        void illuminate(double time, const Terrain &terrain, const SensorBody &ownship,
                        const std::vector<SensorBody> &bodies, const BeamPoint &beam, const float *expectedClosing,
                        SensorProducts &out) const;

        RadarSet m_radar;
        RadarCrossSectionProfile m_signature;
        ScanVolume m_volume;
        DopplerFilter m_doppler;
        Clock m_clock;
        TrackStore m_tracks;
        BeamPoint m_beam;
        EntityId m_designated = kNoEntity;
    };

    int runScanRadarChecks();
}
