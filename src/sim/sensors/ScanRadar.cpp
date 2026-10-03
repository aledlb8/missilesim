#include "sim/sensors/ScanRadar.h"

#include "sim/Terrain.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace missilesim::sim
{
    namespace
    {
        constexpr float kBeamEdgeSlackRad = 1.0e-4f;

        bool sameBody(EntityId a, EntityId b)
        {
            return a.valid() && a == b;
        }

        int barCount(const ScanVolume &volume)
        {
            return std::max(volume.bars, 1);
        }

        // Float half-angles that are an exact number of beams land a few ulps
        // high. That ulp must not grow an extra column and stretch the frame.
        int columnCount(const ScanVolume &volume)
        {
            const double beam = static_cast<double>(volume.beamwidthRad);
            const double width = std::max(0.0, 2.0 * static_cast<double>(volume.azimuthHalfRad));
            if (!(beam > 0.0))
            {
                return 1;
            }
            return std::max(1, static_cast<int>(std::ceil(width / beam - 1.0e-5)));
        }

        // One beam is centered on the nose. Further beams step one beamwidth,
        // symmetric, so adjacent half-power edges meet.
        float azimuthCenter(const ScanVolume &volume, int column)
        {
            const int columns = columnCount(volume);
            const float step = volume.beamwidthRad;
            if (columns <= 1 || !(step > 0.0f))
            {
                return 0.0f;
            }
            const float span = static_cast<float>(columns - 1) * step;
            return -0.5f * span + static_cast<float>(column) * step;
        }

        // Bars sample the elevation limits inclusive. A single bar has no span
        // and sits on the nose, the center of a symmetric volume.
        float elevationCenter(const ScanVolume &volume, int bar)
        {
            const int bars = barCount(volume);
            if (bars <= 1)
            {
                return 0.0f;
            }
            const float fraction = static_cast<float>(bar) / static_cast<float>(bars - 1);
            return -volume.elevationHalfRad + fraction * (2.0f * volume.elevationHalfRad);
        }

        int rasterSlot(const ScanVolume &volume, double time)
        {
            const int slots = barCount(volume) * columnCount(volume);
            if (slots < 1 || !(volume.dwellS >= 1.0e-6))
            {
                return 0;
            }
            const double scaled = (time + 1.0e-9) / volume.dwellS;
            long long index = static_cast<long long>(std::floor(scaled));
            const long long span = slots;
            if (index < 0)
            {
                const long long wrapped = index % span;
                index = wrapped < 0 ? wrapped + span : wrapped;
            }
            else
            {
                index %= span;
            }
            return static_cast<int>(index);
        }

        bool inScanVolume(float azimuth, float elevation, const ScanVolume &volume)
        {
            return std::abs(azimuth) <= volume.azimuthHalfRad + kBeamEdgeSlackRad &&
                   std::abs(elevation) <= volume.elevationHalfRad + kBeamEdgeSlackRad;
        }

        // Where a beam may point: the search volume for the raster, the gimbal
        // limit for a held beam.
        bool inReach(float azimuth, float elevation, const ScanVolume &volume, bool held)
        {
            if (!held)
            {
                return inScanVolume(azimuth, elevation, volume);
            }
            const float azimuthLimit = std::max(volume.gimbalHalfRad, volume.azimuthHalfRad);
            const float elevationLimit = std::max(volume.gimbalHalfRad, volume.elevationHalfRad);
            return std::abs(azimuth) <= azimuthLimit + kBeamEdgeSlackRad &&
                   std::abs(elevation) <= elevationLimit + kBeamEdgeSlackRad;
        }

        bool heldBeam(const BeamPoint &beam)
        {
            return beam.bar < 0;
        }

        bool inBeam(float azimuth, float elevation, const BeamPoint &beam, const ScanVolume &volume)
        {
            const float half = 0.5f * std::max(volume.beamwidthRad, 0.0f);
            return std::abs(azimuth - beam.azimuthRad) <= half + kBeamEdgeSlackRad &&
                   std::abs(elevation - beam.elevationRad) <= half + kBeamEdgeSlackRad;
        }

        // Boresight monopulse with slope 1.5. Not drawn onto the point.
        float monopulseSigmaRad(float beamwidthRad, double snr)
        {
            if (!(snr > 0.0) || !(beamwidthRad > 0.0f))
            {
                return 0.0f;
            }
            return static_cast<float>(static_cast<double>(beamwidthRad) / (1.5 * std::sqrt(2.0 * snr)));
        }
    }

    double searchFrameSeconds(const ScanVolume &volume)
    {
        const double dwell = volume.dwellS > 0.0 ? volume.dwellS : 0.0;
        return static_cast<double>(barCount(volume) * columnCount(volume)) * dwell;
    }

    BeamPoint beamAt(const ScanVolume &volume, double time)
    {
        const int columns = columnCount(volume);
        const int slot = rasterSlot(volume, time);
        BeamPoint point;
        point.bar = columns > 0 ? slot / columns : 0;
        point.column = columns > 0 ? slot % columns : 0;
        point.azimuthRad = azimuthCenter(volume, point.column);
        point.elevationRad = elevationCenter(volume, point.bar);
        point.frameS = searchFrameSeconds(volume);
        return point;
    }

    bool isLaunchQuality(const TrackEstimate &track, double now, double maxAgeSeconds)
    {
        // Age is the last measurement, not the last coast update. A coasted
        // track can still have a recent time and must not launch.
        return track.life == TrackLife::Confirmed && (now - track.lastMeasurementTime) <= maxAgeSeconds;
    }

    bool ScanRadar::Clock::due(double time) const
    {
        return enabled && time + 1.0e-9 >= nextS;
    }

    int ScanRadar::Clock::consume(double time)
    {
        if (!due(time) || periodS < 1.0e-6)
        {
            return 0;
        }

        int overdue = 0;
        while (nextS <= time + 1.0e-9 && overdue < 64)
        {
            ++overdue;
            nextS += periodS;
        }
        if (nextS <= time + 1.0e-9)
        {
            const double skipped = (time - nextS) / periodS;
            overdue += 1 + static_cast<int>(skipped);
            nextS = time + periodS;
        }
        return overdue;
    }

    void ScanRadar::restartClock()
    {
        m_clock = Clock{};
        // A shorter period never advances, so every sample would look. Off.
        m_clock.enabled = m_volume.dwellS >= 1.0e-6;
        m_clock.periodS = m_clock.enabled ? m_volume.dwellS : 1.0;
    }

    void ScanRadar::setRadar(const RadarSet &radar, const RadarCrossSectionProfile &signature)
    {
        m_radar = radar;
        m_signature = signature;
        // Track ids restart at 1. Keeping the old designation would steer at
        // whatever new track inherited that number.
        m_tracks = TrackStore{};
        m_designated = kNoEntity;
        restartClock();
    }

    void ScanRadar::setVolume(const ScanVolume &volume)
    {
        m_volume = volume;
        // The dwell grid is defined from simulation time 0. Tracks stay.
        restartClock();
    }

    void ScanRadar::setDesignatedTrack(EntityId trackId)
    {
        m_designated = trackId;
    }

    bool ScanRadar::steerToDesignated(double time, const SensorBody &ownship, BeamPoint &beam, bool &gateClosing,
                                      float &expectedClosing) const
    {
        gateClosing = false;
        if (!m_designated.valid())
        {
            return false;
        }

        const TrackEstimate *track = nullptr;
        for (const TrackEstimate &candidate : m_tracks.tracks())
        {
            if (candidate.id != m_designated || candidate.life == TrackLife::Lost)
            {
                continue;
            }
            track = &candidate;
            break;
        }
        if (track == nullptr)
        {
            return false;
        }

        const double dt = std::max(0.0, time - track->time);
        const glm::vec3 predicted = track->position + track->velocity * static_cast<float>(dt);
        const SensorBearing bearing = bearingFrom(sensorAxes(ownship), ownship.position, predicted);
        if (!inReach(bearing.azimuthRad, bearing.elevationRad, m_volume, true))
        {
            // Past the gimbal the antenna cannot hold it. The raster resumes,
            // and the track coasts until it is back in reach or lost.
            return false;
        }
        beam = BeamPoint{};
        beam.azimuthRad = bearing.azimuthRad;
        beam.elevationRad = bearing.elevationRad;
        // Not a search cell. Zero would read as bar 0, column 0 of the raster.
        beam.bar = -1;
        beam.column = -1;
        beam.frameS = searchFrameSeconds(m_volume);
        // One hit is a position without a rate; the gate opens on the second.
        gateClosing = track->hitCount >= 2;
        expectedClosing = closingSpeedMps(ownship.position, ownship.velocity, predicted, track->velocity);
        return true;
    }

    void ScanRadar::illuminate(double time, const Terrain &terrain, const SensorBody &ownship,
                               const std::vector<SensorBody> &bodies, const BeamPoint &beam, const float *expectedClosing,
                               SensorProducts &out) const
    {
        const SensorAxes axes = sensorAxes(ownship);
        for (const SensorBody &body : bodies)
        {
            if (!body.alive || sameBody(body.id, ownship.id))
            {
                continue;
            }

            const SensorBearing bearing = bearingFrom(axes, ownship.position, body.position);
            if (!inReach(bearing.azimuthRad, bearing.elevationRad, m_volume, heldBeam(beam)) ||
                !inBeam(bearing.azimuthRad, bearing.elevationRad, beam, m_volume))
            {
                continue;
            }

            // Doppler: lost in the ground clutter, or outside the held beam's velocity gate.
            if (inClutterNotch(m_doppler, terrain, ownship.position, body.position, body.velocity))
            {
                continue;
            }
            if (expectedClosing != nullptr &&
                !inVelocityGate(m_doppler, closingSpeedMps(ownship.position, ownship.velocity, body.position, body.velocity),
                                *expectedClosing))
            {
                continue;
            }

            const float aspect = aspectFromNoseRad(body.position, body.forward, ownship.position);
            // A positive body cross section (chaff) replaces the installed profile.
            const float rcs = body.radarCrossSectionM2 > 0.0f ? body.radarCrossSectionM2
                                                              : meanRadarCrossSectionM2(m_signature, aspect);
            const float range = std::max(bearing.rangeM, kSensorRangeFloorM);
            const double snr = monostaticSnr(m_radar, rcs, range);
            const bool visible = terrain.lineOfSight(ownship.position, body.position);

            LookDebug debug;
            debug.truthPlatform = body.id;
            debug.family = SensorFamily::Radar;
            debug.quality = snr;
            debug.signature = rcs;
            debug.trueRangeM = bearing.rangeM;
            if (!visible)
            {
                debug.reason = LookFail::Masked;
            }
            else if (snr < static_cast<double>(m_radar.snrThreshold))
            {
                debug.reason = LookFail::BelowThreshold;
            }
            else
            {
                debug.detected = true;
                debug.reason = LookFail::None;

                Observation observation;
                observation.sensor = ownship.id;
                observation.family = SensorFamily::Radar;
                observation.time = time;
                observation.origin = ownship.position;
                observation.rangeM = bearing.rangeM;
                observation.azimuthRad = bearing.azimuthRad;
                observation.elevationRad = bearing.elevationRad;
                observation.position = ownship.position + bearing.lineOfSight * bearing.rangeM;
                observation.quality = snr;
                const float sigma = monopulseSigmaRad(m_volume.beamwidthRad, snr);
                observation.sigmaAzimuthRad = sigma;
                observation.sigmaElevationRad = sigma;
                observation.sigmaRangeM = 0.0f;
                out.observations.push_back(observation);
            }
            out.debug.push_back(debug);
        }
    }

    SensorProducts ScanRadar::update(double time, const Terrain &terrain, const SensorBody &ownship,
                                     const std::vector<SensorBody> &bodies)
    {
        SensorProducts products;
        if (!m_clock.due(time))
        {
            return products;
        }

        // Several crossed dwells are still one look, of this pose. The earlier
        // poses were not kept, same as the sensor scheduler.
        const int overdue = m_clock.consume(time);
        products.droppedLooks += std::max(0, overdue - 1);

        BeamPoint steered;
        bool gateClosing = false;
        float expectedClosing = 0.0f;
        const bool held = steerToDesignated(time, ownship, steered, gateClosing, expectedClosing);
        m_beam = held ? steered : beamAt(m_volume, time);

        illuminate(time, terrain, ownship, bodies, m_beam, held && gateClosing ? &expectedClosing : nullptr, products);

        // Only a track this dwell pointed at can be missed by it. The others
        // wait for their own dwell, and fall to coasting only after a full
        // frame, plus one dwell of slack, passes without a measurement.
        const SensorAxes axes = sensorAxes(ownship);
        const BeamPoint beam = m_beam;
        LookCoverage coverage;
        coverage.covers = [this, &axes, &ownship, &beam](const glm::vec3 &predicted) {
            const SensorBearing bearing = bearingFrom(axes, ownship.position, predicted);
            return inReach(bearing.azimuthRad, bearing.elevationRad, m_volume, heldBeam(beam)) &&
                   inBeam(bearing.azimuthRad, bearing.elevationRad, beam, m_volume);
        };
        coverage.revisitS = searchFrameSeconds(m_volume) + m_volume.dwellS;
        m_tracks.onLook(time, products.observations, coverage);
        return products;
    }

    bool ScanRadar::illuminates(const SensorBody &ownship, const glm::vec3 &point) const
    {
        const SensorBearing bearing = bearingFrom(sensorAxes(ownship), ownship.position, point);
        if (!inReach(bearing.azimuthRad, bearing.elevationRad, m_volume, singleTargetTrack()))
        {
            return false;
        }
        // The raster sweeps the whole volume each frame. A held beam paints only its own cell.
        return !singleTargetTrack() || inBeam(bearing.azimuthRad, bearing.elevationRad, m_beam, m_volume);
    }

    namespace
    {
        struct Checks
        {
            int failures = 0;

            void expect(bool passed, const char *name, const std::string &detail = std::string())
            {
                if (!passed)
                {
                    ++failures;
                }
                std::printf("%s  %s%s%s\n", passed ? "PASS" : "FAIL", name, detail.empty() ? "" : "  ", detail.c_str());
            }
        };

        std::string format(const char *pattern, double a, double b = 0.0, double c = 0.0, double d = 0.0)
        {
            char buffer[320];
            std::snprintf(buffer, sizeof(buffer), pattern, a, b, c, d);
            return buffer;
        }

        float deg(double value)
        {
            return static_cast<float>(value * 3.14159265358979323846 / 180.0);
        }

        RadarSet referenceRadar()
        {
            RadarSet radar;
            radar.peakPowerW = 10000.0f;
            radar.gainTransmit = 1000.0f;
            radar.gainReceive = 1000.0f;
            radar.wavelengthM = 0.03f;
            radar.pulseWidthS = 1.0e-6f;
            radar.systemTemperatureK = 290.0f;
            radar.systemLoss = 1.0f;
            radar.snrThreshold = 10.0f;
            return radar;
        }

        RadarCrossSectionProfile unitSignature()
        {
            RadarCrossSectionProfile profile;
            profile.noseM2 = 1.0f;
            profile.beamM2 = 1.0f;
            profile.tailM2 = 1.0f;
            return profile;
        }

        Terrain flatTerrain()
        {
            TerrainConfig config;
            config.kind = TerrainKind::Flat;
            return Terrain(config);
        }

        SensorBody ownshipAt(float altitude)
        {
            SensorBody ownship;
            ownship.id = EntityId{1};
            ownship.position = glm::vec3(0.0f, altitude, 0.0f);
            ownship.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            ownship.up = glm::vec3(0.0f, 1.0f, 0.0f);
            return ownship;
        }

        // Positive azimuth is the right wing, -X for the +Z nose of ownshipAt().
        SensorBody bodyAt(EntityId id, float azimuth, float range, float altitude)
        {
            SensorBody body;
            body.id = id;
            body.position = glm::vec3(-std::sin(azimuth) * range, altitude, std::cos(azimuth) * range);
            body.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            body.up = glm::vec3(0.0f, 1.0f, 0.0f);
            return body;
        }

        int slotCount(const ScanVolume &volume)
        {
            if (!(volume.dwellS > 0.0))
            {
                return 0;
            }
            return static_cast<int>(std::lround(searchFrameSeconds(volume) / volume.dwellS));
        }

        bool trackNear(const TrackStore &store, const glm::vec3 &point, float radius, TrackEstimate &out)
        {
            bool found = false;
            float best = radius;
            for (const TrackEstimate &track : store.tracks())
            {
                const float distance = glm::length(track.position - point);
                if (distance <= best)
                {
                    best = distance;
                    out = track;
                    found = true;
                }
            }
            return found;
        }

        bool sawBody(const SensorProducts &products, const glm::vec3 &point, float radius)
        {
            for (const Observation &observation : products.observations)
            {
                if (glm::length(observation.position - point) <= radius)
                {
                    return true;
                }
            }
            return false;
        }

        void prepare(ScanRadar &radar, const ScanVolume &volume)
        {
            radar.setRadar(referenceRadar(), unitSignature());
            radar.setVolume(volume);
        }

        void checkOutsideVolume(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume volume;
            volume.azimuthHalfRad = deg(30.0);
            volume.elevationHalfRad = deg(10.0);
            volume.beamwidthRad = deg(12.0);
            volume.bars = 2;
            volume.dwellS = 0.25;

            const SensorBody ownship = ownshipAt(100.0f);
            const float azimuth = deg(80.0);
            const SensorBody contact = bodyAt(EntityId{8}, azimuth, 5000.0f, 100.0f);
            const float measured = bearingFrom(sensorAxes(ownship), ownship.position, contact.position).azimuthRad;

            ScanRadar radar;
            prepare(radar, volume);
            const int slots = slotCount(volume);
            bool quiet = slots > 1 && measured > volume.azimuthHalfRad + 0.5f;
            for (int index = 0; index < slots; ++index)
            {
                const double time = static_cast<double>(index) * volume.dwellS;
                const SensorProducts products = radar.update(time, flat, ownship, {contact});
                const BeamPoint raster = beamAt(volume, time);
                if (!products.observations.empty() || !products.debug.empty() || radar.beam().bar != raster.bar ||
                    radar.beam().column != raster.column ||
                    std::abs(radar.beam().azimuthRad - raster.azimuthRad) > 1.0e-4f)
                {
                    quiet = false;
                }
            }
            checks.expect(quiet, "scan: 80 deg is outside a 30 deg volume",
                          format("azimuth %.3f rad, slots %.0f", static_cast<double>(measured), static_cast<double>(slots)));
        }

        void checkNoseBeam(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume volume;
            volume.azimuthHalfRad = deg(30.0);
            volume.elevationHalfRad = deg(10.0);
            volume.beamwidthRad = deg(12.0);
            volume.bars = 1;
            volume.dwellS = 0.25;

            const SensorBody ownship = ownshipAt(100.0f);
            const SensorBody contact = bodyAt(EntityId{6}, 0.0f, 5000.0f, 100.0f);
            ScanRadar radar;
            prepare(radar, volume);

            const int slots = slotCount(volume);
            const float half = 0.5f * volume.beamwidthRad + kBeamEdgeSlackRad;
            int covered = 0;
            int elsewhere = 0;
            bool selective = slots > 1;
            bool sigmaOk = false;
            bool sawCover = false;
            bool farBeam = false;
            for (int index = 0; index < slots; ++index)
            {
                const double time = static_cast<double>(index) * volume.dwellS;
                const BeamPoint raster = beamAt(volume, time);
                const bool covers = std::abs(raster.azimuthRad) <= half && std::abs(raster.elevationRad) <= half;
                const SensorProducts products = radar.update(time, flat, ownship, {contact});
                if (products.droppedLooks != 0)
                {
                    selective = false;
                }
                if (covers)
                {
                    ++covered;
                    sawCover = true;
                    const bool hit = products.observations.size() == 1 && products.observations.front().time == time &&
                                     products.observations.front().sensor == ownship.id &&
                                     glm::length(products.observations.front().position - contact.position) < 1.0e-2f &&
                                     products.debug.size() == 1 && products.debug.front().detected &&
                                     products.debug.front().truthPlatform == contact.id;
                    if (!hit)
                    {
                        selective = false;
                    }
                    else
                    {
                        const Observation &observation = products.observations.front();
                        const float expected = static_cast<float>(static_cast<double>(volume.beamwidthRad) /
                                                                   (1.5 * std::sqrt(2.0 * observation.quality)));
                        const double direct = monostaticSnr(referenceRadar(), 1.0f, 5000.0f);
                        sigmaOk = std::abs(observation.sigmaAzimuthRad - expected) < 1.0e-6f &&
                                  std::abs(observation.sigmaElevationRad - expected) < 1.0e-6f &&
                                  observation.sigmaRangeM == 0.0f && std::abs(observation.quality - direct) < 1.0e-4;
                    }
                }
                else
                {
                    ++elsewhere;
                    if (!products.observations.empty())
                    {
                        selective = false;
                    }
                    if (std::abs(raster.azimuthRad) > volume.beamwidthRad)
                    {
                        farBeam = true;
                    }
                }
            }
            checks.expect(selective && sawCover && covered >= 1 && farBeam && elsewhere >= 1,
                          "scan: the nose is seen only in its beam",
                          format("covered %.0f, elsewhere %.0f", static_cast<double>(covered), static_cast<double>(elsewhere)));
            checks.expect(sigmaOk, "scan: angle sigma is the monopulse term");
        }

        void checkFrameScales(Checks &checks)
        {
            ScanVolume narrow;
            narrow.azimuthHalfRad = deg(15.0);
            narrow.elevationHalfRad = deg(8.0);
            narrow.beamwidthRad = deg(10.0);
            narrow.bars = 2;
            narrow.dwellS = 0.25;
            ScanVolume wide = narrow;
            wide.azimuthHalfRad = deg(30.0);

            const double narrowFrame = searchFrameSeconds(narrow);
            const double wideFrame = searchFrameSeconds(wide);
            const double narrowColumns = narrowFrame / (static_cast<double>(narrow.bars) * narrow.dwellS);
            const double wideColumns = wideFrame / (static_cast<double>(wide.bars) * wide.dwellS);
            checks.expect(std::abs(narrowColumns - 3.0) < 1.0e-6 && std::abs(wideColumns - 6.0) < 1.0e-6 &&
                              std::abs(wideFrame / narrowFrame - 2.0) < 1.0e-9,
                          "scan: doubling azimuth doubles columns and the frame",
                          format("columns %.0f -> %.0f, frames %.3f %.3f", narrowColumns, wideColumns, narrowFrame, wideFrame));

            ScanVolume pattern = wide;
            pattern.bars = 4;
            pattern.elevationHalfRad = deg(8.0);
            const BeamPoint origin = beamAt(pattern, 0.0);
            const BeamPoint nextColumn = beamAt(pattern, pattern.dwellS);
            const BeamPoint nextBar = beamAt(pattern, pattern.dwellS * 6.0);
            const BeamPoint top = beamAt(pattern, pattern.dwellS * 18.0);
            const BeamPoint wrapped = beamAt(pattern, searchFrameSeconds(pattern));
            const bool stepped = origin.bar == 0 && origin.column == 0 && nextColumn.bar == 0 && nextColumn.column == 1 &&
                                 std::abs((nextColumn.azimuthRad - origin.azimuthRad) - pattern.beamwidthRad) < 1.0e-4f &&
                                 nextBar.bar == 1 && nextBar.column == 0 && top.bar == 3 && top.column == 0 &&
                                 std::abs(origin.elevationRad + pattern.elevationHalfRad) < 1.0e-5f &&
                                 std::abs(top.elevationRad - pattern.elevationHalfRad) < 1.0e-5f && wrapped.bar == 0 &&
                                 wrapped.column == 0 && std::abs(origin.frameS - searchFrameSeconds(pattern)) < 1.0e-9 &&
                                 std::abs(searchFrameSeconds(pattern) - 6.0) < 1.0e-9;
            checks.expect(stepped, "scan: the raster steps one beam and spans elevation");
        }

        void checkSingleTargetTrack(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume volume;
            volume.azimuthHalfRad = deg(30.0);
            volume.elevationHalfRad = deg(15.0);
            volume.beamwidthRad = deg(12.0);
            volume.bars = 1;
            volume.dwellS = 0.25;

            const double dwell = volume.dwellS;
            const int slots = slotCount(volume);
            int noseIndex = 0;
            float noseAbs = 1.0e9f;
            for (int index = 0; index < slots; ++index)
            {
                const float magnitude = std::abs(beamAt(volume, index * dwell).azimuthRad);
                if (magnitude < noseAbs)
                {
                    noseAbs = magnitude;
                    noseIndex = index;
                }
            }
            const int edgeIndex = slots - 1;
            const float noseAzimuth = beamAt(volume, noseIndex * dwell).azimuthRad;
            const float edgeAzimuth = beamAt(volume, edgeIndex * dwell).azimuthRad;
            const float range = 8000.0f;
            const float altitude = 100.0f;
            const SensorBody ownship = ownshipAt(altitude);
            const SensorBody nose = bodyAt(EntityId{11}, noseAzimuth, range, altitude);
            const SensorBody edge = bodyAt(EntityId{12}, edgeAzimuth, range, altitude);
            const float separation = glm::length(nose.position - edge.position);
            const bool geometry = slots >= 3 && noseIndex != edgeIndex && noseAbs < 1.0e-3f &&
                                  std::abs(edgeAzimuth) > volume.beamwidthRad &&
                                  std::abs(edgeAzimuth) <= volume.azimuthHalfRad && separation > 2500.0f;

            ScanRadar radar;
            prepare(radar, volume);
            const std::vector<SensorBody> bodies = {nose, edge};
            for (int index = 0; index < 2 * slots; ++index)
            {
                radar.update(static_cast<double>(index) * dwell, flat, ownship, bodies);
            }

            TrackEstimate noseTrack;
            TrackEstimate edgeTrack;
            const bool found = trackNear(radar.tracks(), nose.position, 500.0f, noseTrack) &&
                               trackNear(radar.tracks(), edge.position, 500.0f, edgeTrack);
            const bool primed = geometry && found && edgeTrack.life == TrackLife::Confirmed && noseTrack.hitCount >= 2 &&
                                edgeTrack.hitCount >= 2 && noseTrack.id != nose.id && edgeTrack.id != edge.id;
            // The last dwells pointed at the edge. That is not a miss on the nose.
            checks.expect(found && noseTrack.life == TrackLife::Confirmed,
                          "scan: a dwell elsewhere does not coast a track");
            const bool searchPaints = radar.illuminates(ownship, nose.position) && radar.illuminates(ownship, edge.position) &&
                                      !radar.singleTargetTrack();

            radar.setDesignatedTrack(edgeTrack.id);
            const double noseMeasurement = noseTrack.lastMeasurementTime;
            const int noseHits = noseTrack.hitCount;
            bool steered = true;
            bool noseQuiet = true;
            bool edgeSeen = true;
            bool searchWouldSeeNose = false;
            const float half = 0.5f * volume.beamwidthRad + kBeamEdgeSlackRad;
            for (int index = 2 * slots; index < 3 * slots; ++index)
            {
                const double time = static_cast<double>(index) * dwell;
                const BeamPoint search = beamAt(volume, time);
                if (std::abs(search.azimuthRad - noseAzimuth) <= half && std::abs(search.elevationRad) <= half)
                {
                    searchWouldSeeNose = true;
                }
                const SensorProducts products = radar.update(time, flat, ownship, bodies);
                if (radar.beam().bar != -1 || radar.beam().column != -1 ||
                    std::abs(radar.beam().azimuthRad - edgeAzimuth) > 1.0e-3f)
                {
                    steered = false;
                }
                if (!sawBody(products, edge.position, 500.0f))
                {
                    edgeSeen = false;
                }
                if (sawBody(products, nose.position, 500.0f))
                {
                    noseQuiet = false;
                }
            }

            TrackEstimate noseAfter;
            TrackEstimate edgeAfter;
            const bool stillThere = trackNear(radar.tracks(), nose.position, 500.0f, noseAfter) &&
                                    trackNear(radar.tracks(), edge.position, 500.0f, edgeAfter);
            const bool noseHeld = stillThere && noseAfter.life == TrackLife::Coasting && noseAfter.hitCount == noseHits &&
                                  noseAfter.lastMeasurementTime == noseMeasurement;
            const bool edgeTracked = stillThere && edgeAfter.life == TrackLife::Confirmed && edgeAfter.hitCount > edgeTrack.hitCount;
            const bool trackPaints = radar.singleTargetTrack() && radar.designatedTrack() == edgeTrack.id &&
                                     radar.illuminates(ownship, edge.position) && !radar.illuminates(ownship, nose.position);
            checks.expect(searchPaints && trackPaints, "scan: search paints the volume, track paints its beam");
            checks.expect(primed && steered && noseQuiet && edgeSeen && searchWouldSeeNose && noseHeld && edgeTracked,
                          "scan: single-target track holds the beam",
                          format("sep %.0f m, nose hits %.0f, edge hits %.0f", static_cast<double>(separation),
                                 static_cast<double>(noseHits), stillThere ? static_cast<double>(edgeAfter.hitCount) : -1.0));

            radar.setDesignatedTrack(kNoEntity);
            for (int index = 3 * slots; index < 4 * slots; ++index)
            {
                radar.update(static_cast<double>(index) * dwell, flat, ownship, bodies);
            }
            TrackEstimate noseResumed;
            const bool resumed = trackNear(radar.tracks(), nose.position, 500.0f, noseResumed) &&
                                 noseResumed.lastMeasurementTime > noseMeasurement && noseResumed.hitCount > noseHits &&
                                 radar.beam().bar >= 0 && radar.beam().column >= 0;
            checks.expect(resumed, "scan: clearing designation resumes search");
        }

        // A held beam follows its contact out of the search volume as far as
        // the gimbal, and lets go past it.
        void checkGimbalTrack(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume volume;
            volume.azimuthHalfRad = deg(30.0);
            volume.elevationHalfRad = deg(15.0);
            volume.beamwidthRad = deg(12.0);
            volume.bars = 1;
            volume.dwellS = 0.05;
            volume.gimbalHalfRad = deg(60.0);

            const float range = 5000.0f;
            const float altitude = 100.0f;
            const SensorBody ownship = ownshipAt(altitude);
            const EntityId id{21};
            ScanRadar radar;
            prepare(radar, volume);

            double time = 0.0;
            SensorBody body = bodyAt(id, 0.0f, range, altitude);
            for (int index = 0; index < 4 * slotCount(volume); ++index)
            {
                radar.update(time, flat, ownship, {body});
                time += volume.dwellS;
            }
            TrackEstimate track;
            const bool primed = trackNear(radar.tracks(), body.position, 500.0f, track) && track.life == TrackLife::Confirmed;
            radar.setDesignatedTrack(track.id);

            // Half a degree a dwell, out to 50 degrees: past the volume, inside the gimbal.
            bool heldThroughout = true;
            for (float azimuth = 0.0f; azimuth <= deg(50.0); azimuth += deg(0.5))
            {
                body = bodyAt(id, azimuth, range, altitude);
                radar.update(time, flat, ownship, {body});
                time += volume.dwellS;
                heldThroughout = heldThroughout && radar.singleTargetTrack();
            }
            TrackEstimate wide;
            const bool following = trackNear(radar.tracks(), body.position, 500.0f, wide) && wide.id == track.id &&
                                   wide.life == TrackLife::Confirmed && radar.illuminates(ownship, body.position);

            // On out to 75 degrees: the antenna cannot follow past 60.
            for (float azimuth = deg(50.0); azimuth <= deg(75.0); azimuth += deg(0.5))
            {
                body = bodyAt(id, azimuth, range, altitude);
                radar.update(time, flat, ownship, {body});
                time += volume.dwellS;
            }
            const bool released = !radar.singleTargetTrack() && radar.beam().bar >= 0 &&
                                  !radar.illuminates(ownship, body.position);
            checks.expect(primed && heldThroughout && following && released,
                          "scan: a held beam follows past the volume to the gimbal",
                          format("track %.0f, wide %.0f", primed ? 1.0 : 0.0, following ? 1.0 : 0.0));
        }

        // Looking down, a beaming contact sits in the ground's clutter notch;
        // the same contact flying at the radar does not.
        void checkClutterNotch(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume volume;
            volume.azimuthHalfRad = deg(30.0);
            volume.elevationHalfRad = deg(15.0);
            volume.beamwidthRad = deg(12.0);
            volume.bars = 1;
            volume.dwellS = 0.05;
            DopplerFilter doppler;
            doppler.clutterNotchMps = 30.0f;
            doppler.clutterReachM = 20000.0f;
            doppler.velocityGateMps = 40.0f;

            const auto seen = [&](const glm::vec3 &velocity, float ownAltitude, float contactAltitude) {
                SensorBody ownship = ownshipAt(ownAltitude);
                SensorBody contact;
                contact.id = EntityId{7};
                contact.position = glm::vec3(0.0f, contactAltitude, 5000.0f);
                contact.velocity = velocity;
                ownship.forward = glm::normalize(contact.position - ownship.position);
                ownship.up = glm::normalize(glm::cross(glm::cross(ownship.forward, glm::vec3(0.0f, 1.0f, 0.0f)), ownship.forward));
                ownship.velocity = ownship.forward * 250.0f;
                ScanRadar radar;
                prepare(radar, volume);
                radar.setDoppler(doppler);
                bool measured = false;
                for (int index = 0; index < 2 * slotCount(volume); ++index)
                {
                    const SensorProducts products = radar.update(index * volume.dwellS, flat, ownship, {contact});
                    measured = measured || !products.observations.empty();
                }
                return measured;
            };
            const glm::vec3 beaming(250.0f, 0.0f, 0.0f);
            const glm::vec3 hot(0.0f, 0.0f, -250.0f);
            const bool downBeam = seen(beaming, 1500.0f, 500.0f);
            const bool downHot = seen(hot, 1500.0f, 500.0f);
            const bool upBeam = seen(beaming, 500.0f, 1500.0f);
            checks.expect(!downBeam && downHot && upBeam, "scan: a beaming contact below is lost in the ground clutter",
                          format("down beam %.0f, down hot %.0f, up beam %.0f", downBeam ? 1.0 : 0.0, downHot ? 1.0 : 0.0,
                                 upBeam ? 1.0 : 0.0));
        }

        void checkRidge(Checks &checks)
        {
            TerrainConfig ridgeConfig;
            ridgeConfig.kind = TerrainKind::Ridge;
            ridgeConfig.baseHeightM = 0.0f;
            ridgeConfig.ridgeHeightM = 400.0f;
            ridgeConfig.ridgeDistanceM = 2000.0f;
            ridgeConfig.ridgeHalfWidthM = 600.0f;
            ridgeConfig.ridgeBearingDeg = 0.0f;
            ridgeConfig.hillAmplitudeM = 0.0f;
            const Terrain ridge(ridgeConfig);

            ScanVolume bore;
            bore.azimuthHalfRad = deg(25.0);
            bore.elevationHalfRad = deg(25.0);
            bore.beamwidthRad = deg(60.0);
            bore.bars = 1;
            bore.dwellS = 0.25;

            SensorBody ownship = ownshipAt(50.0f);
            SensorBody contact = bodyAt(EntityId{42}, 0.0f, 4000.0f, 50.0f);
            const bool lowBlocked = !ridge.lineOfSight(ownship.position, contact.position);

            ScanRadar masked;
            prepare(masked, bore);
            const SensorProducts low = masked.update(0.0, ridge, ownship, {contact});
            const bool lowMasked = low.observations.empty() && low.debug.size() == 1 &&
                                   low.debug.front().reason == LookFail::Masked && low.debug.front().truthPlatform == contact.id &&
                                   masked.tracks().tracks().empty();

            ownship.position.y = 900.0f;
            contact.position.y = 900.0f;
            const bool highClear = ridge.lineOfSight(ownship.position, contact.position);
            ScanRadar clear;
            prepare(clear, bore);
            const SensorProducts high = clear.update(0.0, ridge, ownship, {contact});
            const bool highDetected = high.observations.size() == 1 && high.debug.size() == 1 && high.debug.front().detected &&
                                      std::abs(high.observations.front().azimuthRad) < 1.0e-3f &&
                                      std::abs(high.observations.front().elevationRad) < 1.0e-3f &&
                                      high.observations.front().quality >= static_cast<double>(referenceRadar().snrThreshold);
            checks.expect(lowBlocked && lowMasked && highClear && highDetected, "scan: a ridge masks the low path",
                          format("low %.0f, high %.0f", lowBlocked ? 1.0 : 0.0, highClear ? 1.0 : 0.0));

            const bool idSplit = highDetected && contact.id == EntityId{42} && clear.tracks().tracks().size() == 1 &&
                                 clear.tracks().tracks().front().id != contact.id &&
                                 high.debug.front().truthPlatform == contact.id;
            checks.expect(idSplit, "scan: platform 42 is not the track id");
        }

        void checkLaunchQuality(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume bore;
            bore.azimuthHalfRad = deg(20.0);
            bore.elevationHalfRad = deg(20.0);
            bore.beamwidthRad = deg(50.0);
            bore.bars = 1;
            bore.dwellS = 0.5;

            const SensorBody ownship = ownshipAt(100.0f);
            const SensorBody contact = bodyAt(EntityId{4}, 0.0f, 5000.0f, 100.0f);
            ScanRadar radar;
            prepare(radar, bore);

            radar.update(0.0, flat, ownship, {contact});
            TrackEstimate tentative;
            const bool gotTentative = trackNear(radar.tracks(), contact.position, 500.0f, tentative);
            const SensorProducts gap = radar.update(0.1, flat, ownship, {contact});
            TrackEstimate duringGap;
            const bool held = gotTentative && gap.observations.empty() && gap.debug.empty() &&
                              trackNear(radar.tracks(), contact.position, 500.0f, duringGap) &&
                              duringGap.life == TrackLife::Tentative && duringGap.hitCount == 1 &&
                              duringGap.lastMeasurementTime == 0.0;

            radar.update(0.5, flat, ownship, {contact});
            TrackEstimate confirmed;
            const bool gotConfirmed = trackNear(radar.tracks(), contact.position, 500.0f, confirmed);
            const bool fresh = gotConfirmed && confirmed.life == TrackLife::Confirmed &&
                               isLaunchQuality(confirmed, confirmed.lastMeasurementTime, 1.0) &&
                               isLaunchQuality(confirmed, confirmed.lastMeasurementTime + 1.0, 1.0);
            const bool stale = gotConfirmed && !isLaunchQuality(confirmed, confirmed.lastMeasurementTime + 1.0 + 1.0e-3, 1.0);

            radar.update(1.0, flat, ownship, {});
            TrackEstimate coasting;
            const bool gotCoast = trackNear(radar.tracks(), contact.position, 500.0f, coasting);
            TrackEstimate lost = gotCoast ? coasting : TrackEstimate{};
            lost.life = TrackLife::Lost;
            const bool notLaunch = held && !isLaunchQuality(tentative, tentative.time, 2.0) && fresh && stale && gotCoast &&
                                   coasting.life == TrackLife::Coasting && !isLaunchQuality(coasting, coasting.time, 100.0) &&
                                   !isLaunchQuality(lost, lost.lastMeasurementTime, 100.0);
            checks.expect(notLaunch, "scan: launch quality is a fresh confirmed track");
        }

        void checkLateSample(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume volume;
            volume.azimuthHalfRad = deg(30.0);
            volume.elevationHalfRad = deg(10.0);
            volume.beamwidthRad = deg(12.0);
            volume.bars = 1;
            volume.dwellS = 0.25;

            int noseSlot = -1;
            const int slots = slotCount(volume);
            for (int index = 0; index < slots; ++index)
            {
                const BeamPoint raster = beamAt(volume, index * volume.dwellS);
                if (std::abs(raster.azimuthRad) < 1.0e-4f && std::abs(raster.elevationRad) < 1.0e-4f)
                {
                    noseSlot = index;
                    break;
                }
            }

            const SensorBody ownship = ownshipAt(100.0f);
            const SensorBody contact = bodyAt(EntityId{5}, 0.0f, 5000.0f, 100.0f);
            ScanRadar radar;
            prepare(radar, volume);
            radar.update(0.0, flat, ownship, {contact});
            const double time = noseSlot > 0 ? static_cast<double>(noseSlot) * volume.dwellS : 0.0;
            const SensorProducts late = radar.update(time, flat, ownship, {contact});
            const BeamPoint raster = beamAt(volume, time);
            const bool once = noseSlot > 1 && late.droppedLooks == noseSlot - 1 && late.observations.size() == 1 &&
                              late.observations.front().time == time && radar.beam().column == raster.column &&
                              std::abs(radar.beam().azimuthRad) < 1.0e-4f;
            checks.expect(once, "scan: a late sample looks once",
                          format("dropped %.0f", static_cast<double>(late.droppedLooks)));
        }

        void checkBelowThreshold(Checks &checks)
        {
            const Terrain flat = flatTerrain();
            ScanVolume bore;
            bore.azimuthHalfRad = deg(20.0);
            bore.elevationHalfRad = deg(20.0);
            bore.beamwidthRad = deg(50.0);
            bore.bars = 1;
            bore.dwellS = 0.25;

            const SensorBody ownship = ownshipAt(100.0f);
            const SensorBody contact = bodyAt(EntityId{9}, 0.0f, 30000.0f, 100.0f);
            ScanRadar radar;
            prepare(radar, bore);
            const SensorProducts products = radar.update(0.0, flat, ownship, {contact});
            const bool gated = products.observations.empty() && products.debug.size() == 1 &&
                               products.debug.front().reason == LookFail::BelowThreshold && !products.debug.front().detected &&
                               products.debug.front().quality > 0.0 &&
                               products.debug.front().quality < static_cast<double>(referenceRadar().snrThreshold) &&
                               radar.tracks().tracks().empty();
            checks.expect(gated, "scan: SNR under the threshold is not a detection",
                          format("SNR %.3f", products.debug.empty() ? 0.0 : products.debug.front().quality));
        }
    }

    int runScanRadarChecks()
    {
        Checks checks;
        {
            // The azimuth sign is the aircraft's own right wing, body FRD.
            SensorBody level = ownshipAt(0.0f);
            level.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            level.up = glm::vec3(0.0f, 1.0f, 0.0f);
            const SensorAxes axes = sensorAxes(level);
            const SensorBearing right = bearingFrom(axes, level.position, level.position + axes.right * 1000.0f);
            const SensorBearing above = bearingFrom(axes, level.position, level.position + axes.up * 1000.0f);
            checks.expect(glm::length(axes.right - glm::cross(axes.forward, axes.up)) < 1.0e-6f &&
                              glm::length(axes.right - glm::vec3(-1.0f, 0.0f, 0.0f)) < 1.0e-6f &&
                              std::abs(right.azimuthRad - deg(90.0)) < 1.0e-5f &&
                              std::abs(above.elevationRad - deg(90.0)) < 1.0e-5f,
                          "scan: positive azimuth is the right wing");
        }
        checkOutsideVolume(checks);
        checkNoseBeam(checks);
        checkFrameScales(checks);
        checkSingleTargetTrack(checks);
        checkGimbalTrack(checks);
        checkClutterNotch(checks);
        checkRidge(checks);
        checkLaunchQuality(checks);
        checkLateSample(checks);
        checkBelowThreshold(checks);
        std::printf("scan radar checks: %d failed\n", checks.failures);
        return checks.failures;
    }
}
