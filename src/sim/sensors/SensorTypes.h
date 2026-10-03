#pragma once

// Sensor model 1: fictional radar and infrared profiles, scheduled looks, and
// measurements a tracker is allowed to see.
//
// This is a declared game model, not a measured radar and not a measured
// aircraft signature. Research fields that were not recovered (clear-air
// attenuation, rain, clutter, diffraction, glint, thermal angle noise) are
// left out rather than filled with a stand-in number. One detection gate is
// applied, once: a look detects or it does not. Nothing here multiplies in a
// second processing gain or an atmospheric loss.
//
// A look may read a platform's true pose in order to build a measurement.
// The measurement (Observation) carries geometry, time, and quality. The
// platform that produced it is recorded only on LookDebug, which HUD, AI, and
// weapon support must not read. Fox 2 seekers do not use this path.

#include "sim/EntityId.h"

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    class Terrain;

    constexpr int kSensorModelVersion = 1;

    // Boltzmann's constant (J/K), CODATA 2019. The radar equation below is the
    // only consumer.
    constexpr double kBoltzmannJPerK = 1.380649e-23;

    // Numerical floor so a contact inside one metre does not make the range
    // equation divide by zero. It is not a detection range.
    constexpr float kSensorRangeFloorM = 1.0f;

    enum class SensorFamily : std::uint8_t
    {
        Radar,
        Infrared,
    };

    // Why a scheduled look at one body produced no measurement.
    enum class LookFail : std::uint8_t
    {
        None,
        Masked,          // the terrain's line of sight is blocked
        BelowThreshold,  // the single detection gate failed
    };

    // Pose a sensor is allowed to read. Aspect uses forward, never a velocity,
    // so a body flying sideways presents its nose aspect to someone ahead of it.
    struct SensorBody
    {
        EntityId id;
        glm::vec3 position{0.0f};
        // World velocity (m/s). Only Doppler processing reads it; aspect never does.
        glm::vec3 velocity{0.0f};
        glm::vec3 forward{0.0f, 0.0f, 1.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        float throttle = 0.0f; // 0..1 of military power
        bool afterburner = false;
        bool alive = true;
        // When positive, this body's radar cross section (m²) replaces the
        // installed profile. Chaff uses it. Zero keeps the profile.
        float radarCrossSectionM2 = 0.0f;
    };

    // Body-frame mean radar cross section (m²) at three aspects. Linear in the
    // nose angle between them. Not a pattern of any named aircraft.
    struct RadarCrossSectionProfile
    {
        float noseM2 = 1.0f;
        float beamM2 = 1.0f;
        float tailM2 = 1.0f;
    };

    // Fictional monostatic radar. Every field is a model input. There is no
    // lock range. systemLoss is dimensionless and does not include the air.
    // snrThreshold is a linear power ratio and a temporary detection gate:
    // the probability-of-detection curves are not what this model runs.
    struct RadarSet
    {
        float peakPowerW = 0.0f;
        float gainTransmit = 0.0f;
        float gainReceive = 0.0f;
        float wavelengthM = 0.0f;
        float pulseWidthS = 0.0f;
        float systemTemperatureK = 290.0f;
        float systemLoss = 1.0f;
        float snrThreshold = 1.0f;
        double revisitS = 1.0;
    };

    // Fictional in-band radiant intensity (W/sr). Skin is present at every
    // aspect. The tailpipe lobe is aft and the plume lobe is broadside; both
    // scale with throttle, or with afterburnerScale when reheat is on.
    // afterburnerScale replaces the throttle fraction. It does not multiply
    // the skin, and it is not a measured radiance ratio.
    struct InfraredSignatureProfile
    {
        float skinWPerSr = 0.0f;
        float tailpipeWPerSr = 0.0f;
        float plumeBeamWPerSr = 0.0f;
        float afterburnerScale = 1.0f;
    };

    // Fictional infrared receiver. The gate is irradiance / NEI, once.
    // No transmittance table is loaded, so the path factor is omitted.
    struct InfraredSet
    {
        float neiWPerM2 = 0.0f;
        float snrMin = 1.0f;
        double revisitS = 0.1;
    };

    // Orthonormal sensor frame. right = forward × up, the right wing, the same
    // axis as Jet::right() (body FRD). With a +Z nose and a +Y up it is -X.
    struct SensorAxes
    {
        glm::vec3 forward{0.0f, 0.0f, 1.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        glm::vec3 right{-1.0f, 0.0f, 0.0f};
    };

    // Re-orthonormalizes the body's forward and up. A degenerate up is
    // replaced with world up, or world X when the nose is near vertical.
    SensorAxes sensorAxes(const SensorBody &body);

    // Bearing of a point from the sensor. Azimuth is positive toward the right
    // wing; elevation is positive above the nose-right plane.
    struct SensorBearing
    {
        float azimuthRad = 0.0f;
        float elevationRad = 0.0f;
        float rangeM = 0.0f;
        glm::vec3 lineOfSight{0.0f, 0.0f, 1.0f}; // forward when the point is coincident
    };

    SensorBearing bearingFrom(const SensorAxes &axes, const glm::vec3 &origin, const glm::vec3 &point);

    // Pulse-Doppler processing, shared by every radar in this model. A radar
    // sorts echoes by closing speed, not just by strength:
    //  - Main-lobe clutter: when the beam, carried on past an echo, lands on
    //    the ground, the ground's own echo fills the speeds near the ground's.
    //    An echo whose speed along the line of sight is inside that notch is
    //    lost in it: a beaming aircraft, or chaff that has stopped in the air.
    //    Looking up, past the echo there is only sky, and nothing is notched.
    //  - Velocity gate: a radar tracking one target only accepts echoes whose
    //    closing speed is near the one it expects.
    // Declared game parameters, not a measured radar. Zero turns a stage off.
    struct DopplerFilter
    {
        float clutterNotchMps = 0.0f; // half-width of the notch, ground speed along the line of sight
        float clutterReachM = 0.0f;   // ground further than this from the radar is below the noise
        float velocityGateMps = 0.0f; // half-width of the tracking gate on closing speed
    };

    // Positive while the range between the two shrinks.
    float closingSpeedMps(const glm::vec3 &sensorPosition, const glm::vec3 &sensorVelocity, const glm::vec3 &bodyPosition,
                          const glm::vec3 &bodyVelocity);

    // The echo sits in the main-lobe clutter notch: the beam looks down past it
    // onto the ground within reach, and its own speed along the line of sight
    // (the ground's is zero) is inside the notch.
    bool inClutterNotch(const DopplerFilter &filter, const Terrain &terrain, const glm::vec3 &sensorPosition,
                        const glm::vec3 &bodyPosition, const glm::vec3 &bodyVelocity);

    // The closing speed is inside the velocity gate around the expected one.
    // Always true with the gate off.
    bool inVelocityGate(const DopplerFilter &filter, float closingMps, float expectedClosingMps);

    // One measurement. No platform identity.
    struct Observation
    {
        EntityId sensor;
        SensorFamily family = SensorFamily::Radar;
        double time = 0.0; // simulation time of the body state that was sampled
        glm::vec3 origin{0.0f};
        float rangeM = 0.0f;
        float azimuthRad = 0.0f;   // toward the sensor's right wing, about its up axis
        float elevationRad = 0.0f; // above the sensor's nose-right plane
        // Point on the measured bearing at the measured range. With no angle
        // or range noise drawn, this is the body's true position.
        glm::vec3 position{0.0f};
        double quality = 0.0;      // radar: linear SNR. infrared: irradiance / NEI
        double irradianceWPerM2 = 0.0;
        // Monopulse angle uncertainty (rad). The mean position stays geometric.
        // Range uncertainty stays 0 until a bandwidth is sourced.
        float sigmaAzimuthRad = 0.0f;
        float sigmaElevationRad = 0.0f;
        float sigmaRangeM = 0.0f;
    };

    // Private diagnostic for one body on one look. Gameplay consumers of
    // observations and tracks do not receive this.
    struct LookDebug
    {
        EntityId truthPlatform;
        SensorFamily family = SensorFamily::Radar;
        LookFail reason = LookFail::None;
        bool detected = false;
        double quality = 0.0;
        double signature = 0.0; // radar: m². infrared: W/sr
        float trueRangeM = 0.0f;
    };

    // Angle at the body between its nose and the direction toward the observer.
    // 0 is nose-on, pi is tail-on.
    float aspectFromNoseRad(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &observer);

    float meanRadarCrossSectionM2(const RadarCrossSectionProfile &profile, float aspectRad);

    // Single-pulse monostatic SNR. No pulse integration, no atmosphere, no clutter.
    double monostaticSnr(const RadarSet &radar, float rcsM2, float rangeM);

    float infraredIntensityWPerSr(const InfraredSignatureProfile &profile, float aspectRad, float throttle,
                                  bool afterburner);

    // Intensity over range squared. The missing transmittance is not a loss term.
    double infraredIrradianceWPerM2(float intensityWPerSr, float rangeM);

    struct SensorProducts
    {
        std::vector<Observation> observations;
        std::vector<LookDebug> debug;
        // Grid times that fell between samples and were not given their own look.
        // A late sample still produces one look, of the state it was given.
        int droppedLooks = 0;
    };
}
