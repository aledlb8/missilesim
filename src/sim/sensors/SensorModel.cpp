#include "sim/sensors/SensorScheduler.h"

#include "sim/Terrain.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace missilesim::sim
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        glm::vec3 unitOr(const glm::vec3 &vector, const glm::vec3 &fallback)
        {
            const float length = glm::length(vector);
            return length > 1.0e-6f ? vector / length : fallback;
        }

        bool sameBody(EntityId a, EntityId b)
        {
            return a.valid() && a == b;
        }
    }

    SensorAxes sensorAxes(const SensorBody &body)
    {
        SensorAxes axes;
        axes.forward = unitOr(body.forward, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::vec3 up = body.up - axes.forward * glm::dot(body.up, axes.forward);
        if (glm::length(up) < 1.0e-6f)
        {
            const glm::vec3 helper = std::abs(axes.forward.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                                      : glm::vec3(1.0f, 0.0f, 0.0f);
            up = helper - axes.forward * glm::dot(helper, axes.forward);
        }
        axes.up = unitOr(up, glm::vec3(0.0f, 1.0f, 0.0f));
        axes.right = unitOr(glm::cross(axes.forward, axes.up), glm::vec3(-1.0f, 0.0f, 0.0f));
        return axes;
    }

    SensorBearing bearingFrom(const SensorAxes &axes, const glm::vec3 &origin, const glm::vec3 &point)
    {
        SensorBearing bearing;
        const glm::vec3 offset = point - origin;
        bearing.rangeM = glm::length(offset);
        bearing.lineOfSight = bearing.rangeM > 1.0e-3f ? offset / bearing.rangeM : axes.forward;
        bearing.azimuthRad =
            std::atan2(glm::dot(bearing.lineOfSight, axes.right), glm::dot(bearing.lineOfSight, axes.forward));
        bearing.elevationRad = std::asin(glm::clamp(glm::dot(bearing.lineOfSight, axes.up), -1.0f, 1.0f));
        return bearing;
    }

    float aspectFromNoseRad(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &observer)
    {
        const glm::vec3 toObserver = observer - position;
        const float distance = glm::length(toObserver);
        const float noseLength = glm::length(forward);
        if (distance < 1.0e-3f || noseLength < 1.0e-6f)
        {
            return 0.0f;
        }
        const float cosine = glm::clamp(glm::dot(forward / noseLength, toObserver / distance), -1.0f, 1.0f);
        return std::acos(cosine);
    }

    float meanRadarCrossSectionM2(const RadarCrossSectionProfile &profile, float aspectRad)
    {
        const float nose = std::max(profile.noseM2, 0.0f);
        const float beam = std::max(profile.beamM2, 0.0f);
        const float tail = std::max(profile.tailM2, 0.0f);
        const float aspect = std::clamp(aspectRad, 0.0f, static_cast<float>(kPi));
        const float halfPi = static_cast<float>(kPi * 0.5);
        if (aspect <= halfPi)
        {
            return nose + (beam - nose) * (aspect / halfPi);
        }
        return beam + (tail - beam) * ((aspect - halfPi) / halfPi);
    }

    float closingSpeedMps(const glm::vec3 &sensorPosition, const glm::vec3 &sensorVelocity, const glm::vec3 &bodyPosition,
                          const glm::vec3 &bodyVelocity)
    {
        const glm::vec3 offset = bodyPosition - sensorPosition;
        const float range = glm::length(offset);
        if (!(range > 1.0e-3f))
        {
            return 0.0f;
        }
        return glm::dot(sensorVelocity - bodyVelocity, offset / range);
    }

    bool inClutterNotch(const DopplerFilter &filter, const Terrain &terrain, const glm::vec3 &sensorPosition,
                        const glm::vec3 &bodyPosition, const glm::vec3 &bodyVelocity)
    {
        if (!(filter.clutterNotchMps > 0.0f) || !(filter.clutterReachM > 0.0f))
        {
            return false;
        }
        const glm::vec3 offset = bodyPosition - sensorPosition;
        const float range = glm::length(offset);
        if (!(range > 1.0e-3f) || range >= filter.clutterReachM)
        {
            return false;
        }
        const glm::vec3 lineOfSight = offset / range;
        // Speed along the line of sight relative to the ground, which is still.
        if (std::abs(glm::dot(bodyVelocity, lineOfSight)) >= filter.clutterNotchMps)
        {
            return false;
        }
        // Clutter only where the beam goes on past the echo into the ground.
        if (lineOfSight.y >= 0.0f)
        {
            return false;
        }
        const glm::vec3 reach = sensorPosition + lineOfSight * filter.clutterReachM;
        return terrain.segmentHit(bodyPosition, reach);
    }

    bool inVelocityGate(const DopplerFilter &filter, float closingMps, float expectedClosingMps)
    {
        if (!(filter.velocityGateMps > 0.0f))
        {
            return true;
        }
        return std::abs(closingMps - expectedClosingMps) <= filter.velocityGateMps;
    }

    double monostaticSnr(const RadarSet &radar, float rcsM2, float rangeM)
    {
        // SNR = Pt Gt Gr λ² σ τ / ((4π)³ R⁴ k Ts Lsys), one pulse, monostatic.
        // Lsys is the set's system loss alone. Atmospheric loss is not applied,
        // and N is not applied: this model integrates nothing.
        if (radar.peakPowerW <= 0.0f || radar.gainTransmit <= 0.0f || radar.gainReceive <= 0.0f ||
            radar.wavelengthM <= 0.0f || radar.pulseWidthS <= 0.0f || radar.systemTemperatureK <= 0.0f ||
            radar.systemLoss <= 0.0f || rcsM2 <= 0.0f)
        {
            return 0.0;
        }

        const double range = std::max(static_cast<double>(rangeM), static_cast<double>(kSensorRangeFloorM));
        const double fourPi = 4.0 * kPi;
        const double fourPiCubed = fourPi * fourPi * fourPi;
        const double lambda = static_cast<double>(radar.wavelengthM);
        const double numerator = static_cast<double>(radar.peakPowerW) * static_cast<double>(radar.gainTransmit) *
                                 static_cast<double>(radar.gainReceive) * (lambda * lambda) * static_cast<double>(rcsM2) *
                                 static_cast<double>(radar.pulseWidthS);
        const double denominator = fourPiCubed * (range * range * range * range) * kBoltzmannJPerK *
                                   static_cast<double>(radar.systemTemperatureK) * static_cast<double>(radar.systemLoss);
        return numerator / denominator;
    }

    float infraredIntensityWPerSr(const InfraredSignatureProfile &profile, float aspectRad, float throttle,
                                  bool afterburner)
    {
        const float aspect = std::clamp(aspectRad, 0.0f, static_cast<float>(kPi));
        const float aft = std::max(0.0f, -std::cos(aspect));
        const float side = std::sin(aspect);
        const float power = afterburner ? std::max(profile.afterburnerScale, 0.0f) : std::clamp(throttle, 0.0f, 1.0f);
        const float hot = (std::max(profile.tailpipeWPerSr, 0.0f) * aft + std::max(profile.plumeBeamWPerSr, 0.0f) * side) * power;
        return std::max(profile.skinWPerSr, 0.0f) + hot;
    }

    double infraredIrradianceWPerM2(float intensityWPerSr, float rangeM)
    {
        const double range = std::max(static_cast<double>(rangeM), static_cast<double>(kSensorRangeFloorM));
        return static_cast<double>(std::max(intensityWPerSr, 0.0f)) / (range * range);
    }

    bool SensorScheduler::Clock::due(double time) const
    {
        return enabled && time + 1.0e-9 >= nextS;
    }

    int SensorScheduler::Clock::consume(double time)
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

    void SensorScheduler::setRadar(const RadarSet &radar, const RadarCrossSectionProfile &signature)
    {
        m_radar = radar;
        m_radarSignature = signature;
        m_radarClock = Clock{};
        m_radarClock.enabled = radar.revisitS > 0.0;
        m_radarClock.periodS = radar.revisitS;
        m_radarTracks = TrackStore{};
    }

    void SensorScheduler::setInfrared(const InfraredSet &infrared, const InfraredSignatureProfile &signature)
    {
        m_infrared = infrared;
        m_infraredSignature = signature;
        m_infraredClock = Clock{};
        m_infraredClock.enabled = infrared.revisitS > 0.0;
        m_infraredClock.periodS = infrared.revisitS;
        m_infraredTracks = TrackStore{};
    }

    void SensorScheduler::lookRadar(double time, const Terrain &terrain, const SensorBody &ownship,
                                    const std::vector<SensorBody> &bodies, SensorProducts &out) const
    {
        const SensorAxes axes = sensorAxes(ownship);
        for (const SensorBody &body : bodies)
        {
            if (!body.alive || sameBody(body.id, ownship.id))
            {
                continue;
            }

            const SensorBearing bearing = bearingFrom(axes, ownship.position, body.position);
            const float trueRange = bearing.rangeM;
            const float range = std::max(trueRange, kSensorRangeFloorM);
            const float aspect = aspectFromNoseRad(body.position, body.forward, ownship.position);
            const float rcs = meanRadarCrossSectionM2(m_radarSignature, aspect);
            const double snr = monostaticSnr(m_radar, rcs, range);
            const bool visible = terrain.lineOfSight(ownship.position, body.position);

            LookDebug debug;
            debug.truthPlatform = body.id;
            debug.family = SensorFamily::Radar;
            debug.quality = snr;
            debug.signature = rcs;
            debug.trueRangeM = trueRange;
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
                observation.rangeM = trueRange;
                observation.azimuthRad = bearing.azimuthRad;
                observation.elevationRad = bearing.elevationRad;
                observation.position = ownship.position + bearing.lineOfSight * trueRange;
                observation.quality = snr;
                out.observations.push_back(observation);
            }
            out.debug.push_back(debug);
        }
    }

    void SensorScheduler::lookInfrared(double time, const Terrain &terrain, const SensorBody &ownship,
                                       const std::vector<SensorBody> &bodies, SensorProducts &out) const
    {
        const SensorAxes axes = sensorAxes(ownship);
        const float nei = m_infrared.neiWPerM2;
        for (const SensorBody &body : bodies)
        {
            if (!body.alive || sameBody(body.id, ownship.id))
            {
                continue;
            }

            const SensorBearing bearing = bearingFrom(axes, ownship.position, body.position);
            const float trueRange = bearing.rangeM;
            const float range = std::max(trueRange, kSensorRangeFloorM);
            const float aspect = aspectFromNoseRad(body.position, body.forward, ownship.position);
            const float intensity =
                infraredIntensityWPerSr(m_infraredSignature, aspect, body.throttle, body.afterburner);
            const double irradiance = infraredIrradianceWPerM2(intensity, range);
            const double quality = nei > 0.0f ? irradiance / static_cast<double>(nei) : 0.0;
            const bool visible = terrain.lineOfSight(ownship.position, body.position);

            LookDebug debug;
            debug.truthPlatform = body.id;
            debug.family = SensorFamily::Infrared;
            debug.quality = quality;
            debug.signature = intensity;
            debug.trueRangeM = trueRange;
            if (!visible)
            {
                debug.reason = LookFail::Masked;
            }
            else if (nei <= 0.0f || quality < static_cast<double>(m_infrared.snrMin))
            {
                debug.reason = LookFail::BelowThreshold;
            }
            else
            {
                debug.detected = true;
                debug.reason = LookFail::None;

                Observation observation;
                observation.sensor = ownship.id;
                observation.family = SensorFamily::Infrared;
                observation.time = time;
                observation.origin = ownship.position;
                observation.rangeM = trueRange;
                observation.azimuthRad = bearing.azimuthRad;
                observation.elevationRad = bearing.elevationRad;
                observation.position = ownship.position + bearing.lineOfSight * trueRange;
                observation.quality = quality;
                observation.irradianceWPerM2 = irradiance;
                out.observations.push_back(observation);
            }
            out.debug.push_back(debug);
        }
    }

    SensorProducts SensorScheduler::update(double time, const Terrain &terrain, const SensorBody &ownship,
                                           const std::vector<SensorBody> &bodies)
    {
        SensorProducts products;
        if (m_radarClock.due(time))
        {
            const int overdue = m_radarClock.consume(time);
            products.droppedLooks += std::max(0, overdue - 1);

            const std::size_t radarBegin = products.observations.size();
            lookRadar(time, terrain, ownship, bodies, products);
            std::vector<Observation> radarObservations(products.observations.begin() + static_cast<std::ptrdiff_t>(radarBegin),
                                                       products.observations.end());
            m_radarTracks.onLook(time, radarObservations);
        }
        if (m_infraredClock.due(time))
        {
            const int overdue = m_infraredClock.consume(time);
            products.droppedLooks += std::max(0, overdue - 1);

            const std::size_t infraredBegin = products.observations.size();
            lookInfrared(time, terrain, ownship, bodies, products);
            std::vector<Observation> infraredObservations(
                products.observations.begin() + static_cast<std::ptrdiff_t>(infraredBegin), products.observations.end());
            m_infraredTracks.onLook(time, infraredObservations);
        }
        return products;
    }
}
