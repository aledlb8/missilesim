#pragma once

// Sound generators for everything that makes noise in the simulation. Each
// voice is driven by physical parameters (thrust, exhaust velocity, spool
// speed, charge mass, ...) and emits source pressure in Pa at 1 m into up to
// three directivity lobes. See Synthesis.h for the scaling laws used.
//
// Threading: constructors and set*() run on the game thread; render() runs on
// the audio thread. Parameters cross over through lock-free mailboxes.

#include "Synthesis.h"

#include <cstdint>

namespace missilesim::audio::synth
{
    // ------------------------------------------------------------------
    // Solid rocket motor + airframe flow of a missile in flight.
    // Lobes: 0 large-scale mixing noise (aft), 1 crackle (aft-oblique),
    //        2 fine-scale mixing, combustion rumble, shock hiss, flow (broad).
    // ------------------------------------------------------------------

    struct RocketMotorParams
    {
        float thrustNewtons = 0.0f;      // delivered thrust; 0 while the motor is cold / burnt out
        float exhaustVelocity = 2200.0f; // effective exhaust velocity (m/s)
        float nozzleDiameter = 0.1f;     // m
        float bodyDiameter = 0.127f;     // m, for airframe flow noise
        float airDensity = 1.225f;       // kg/m^3 at the missile
    };

    class RocketMotorVoice : public SoundSource
    {
    public:
        explicit RocketMotorVoice(uint64_t seed);
        static EmitterSpec spec(float bodyLength, float bodyDiameter);

        void setParams(const RocketMotorParams &params) { m_params.write(params); }

        void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) override;
        bool isFinished() const override { return m_released && m_releaseGain <= 0.0f; }

    private:
        LatestValue<RocketMotorParams> m_params;
        Random m_random;

        // Combustion state
        bool m_burning = false;
        float m_thrust = 0.0f;       // smoothed delivered thrust (N)
        float m_tailThrust = 0.0f;   // thrust at burnout, drives the tail-off
        float m_tailSeconds = -1.0f; // <0: no tail-off in progress
        float m_ignitionSeconds = -1.0f;
        float m_ignitionAmplitude = 0.0f;
        float m_chuff = 0.0f;
        float m_chuffTimer = 0.0f;

        BandNoise m_large, m_fine, m_rumble, m_hiss, m_flow;
        Crackle m_crackle;
        Flicker m_mixingFlicker{0.025f, 0.3f};
        Flicker m_crackleFlicker{0.05f, 0.5f};
        Flicker m_flowFlicker{0.08f, 0.6f};

        // Per-block gains (start/end for click-free ramps)
        float m_gainLarge = 0.0f, m_gainFine = 0.0f, m_gainCrackle = 0.0f;
        float m_gainRumble = 0.0f, m_gainHiss = 0.0f, m_gainFlow = 0.0f;
        float m_releaseGain = 1.0f;
    };

    // ------------------------------------------------------------------
    // Afterburning low-bypass turbofan (F110/F100 class) + airframe flow.
    // Lobes: 0 jet exhaust (mixing noise, crackle, turbine tones; aft),
    //        1 inlet (fan tones, buzz-saw, compressor whine; forward),
    //        2 fine-scale jet noise, core/afterburner rumble, flow (broad).
    // ------------------------------------------------------------------

    struct TurbofanParams
    {
        float throttle = 0.7f;     // 0 idle .. 1 full; >= 0.93 selects afterburner
        float airDensity = 1.225f; // kg/m^3 at the aircraft
    };

    class TurbofanVoice : public SoundSource
    {
    public:
        explicit TurbofanVoice(uint64_t seed, float initialThrottle = 0.7f);
        static EmitterSpec spec();

        void setParams(const TurbofanParams &params) { m_params.write(params); }

        void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) override;
        bool isFinished() const override { return m_released && m_releaseGain <= 0.0f; }

    private:
        LatestValue<TurbofanParams> m_params;
        Random m_random;

        float m_n1 = 0.8f; // fan spool, fraction of max rpm
        float m_afterburner = 0.0f;
        bool m_afterburnerSelected = false;
        float m_lightOffDelay = -1.0f;
        float m_lightOffSeconds = -1.0f;
        float m_lightOffAmplitude = 0.0f;

        BandNoise m_large, m_fine, m_core, m_augmentorRumble, m_fanBroadband, m_flow;
        Crackle m_crackle;
        Flicker m_jetFlicker{0.04f, 0.4f};
        Flicker m_fanFlicker{0.015f, 0.25f};
        Flicker m_augmentorFlicker{0.02f, 0.2f};
        Flicker m_flowFlicker{0.1f, 0.7f};

        Wavetable m_buzzSaw;
        Sine m_bladePass, m_bladePass2, m_compressor, m_turbine;
        RandomWalk m_toneWander;

        float m_gainLarge = 0.0f, m_gainFine = 0.0f, m_gainCrackle = 0.0f, m_gainCore = 0.0f;
        float m_gainRumble = 0.0f, m_gainFanBroadband = 0.0f, m_gainBpf = 0.0f, m_gainBpf2 = 0.0f;
        float m_gainBuzz = 0.0f, m_gainCompressor = 0.0f, m_gainTurbine = 0.0f, m_gainFlow = 0.0f;
        float m_releaseGain = 1.0f;
    };

    // ------------------------------------------------------------------
    // High-explosive warhead detonation (single omni lobe, one-shot).
    // Friedlander blast wave scaled with Kinney-Graham, fireball afterburn
    // roar, fireball-growth pressure pulse and supersonic fragment crackle.
    // ------------------------------------------------------------------

    class ExplosionVoice : public SoundSource
    {
    public:
        ExplosionVoice(uint64_t seed, float chargeKg, float ambientPressurePa = 101325.0f);
        static EmitterSpec spec();

        void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) override;
        bool isFinished() const override { return m_sample >= m_lengthSamples; }

    private:
        Random m_random;
        int64_t m_sample = 0;
        int64_t m_lengthSamples = 0;

        float m_blastPeak = 0.0f; // Pa at 1 m (far-field equivalent)
        float m_blastDuration = 0.0f;
        float m_fireballRms = 0.0f;
        float m_fireballDecay = 0.0f;
        float m_pulseAmplitude = 0.0f;
        float m_pulseHz = 0.0f;
        float m_fragmentRms = 0.0f;

        BandNoise m_fireball, m_sizzle;
        Crackle m_fragments;
        Flicker m_fireballFlicker{0.02f, 0.15f};
        float m_flicker = 1.0f;
    };

    // ------------------------------------------------------------------
    // Decoy flare: impulse-cartridge pop, pellet ignition, magnesium burn.
    // ------------------------------------------------------------------

    struct FlareParams
    {
        float heat = 1.0f; // current / initial heat signature, 0..1
    };

    class FlareVoice : public SoundSource
    {
    public:
        explicit FlareVoice(uint64_t seed);
        static EmitterSpec spec();

        void setParams(const FlareParams &params) { m_params.write(params); }

        void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) override;
        bool isFinished() const override { return m_released && m_releaseGain <= 0.0f; }

    private:
        LatestValue<FlareParams> m_params;
        Random m_random;
        int64_t m_sample = 0;
        float m_heat = 1.0f;

        BandNoise m_fizz, m_roar;
        Crackle m_sputter;
        Flicker m_burnFlicker{0.012f, 0.18f};
        float m_gainFizz = 0.0f, m_gainRoar = 0.0f, m_gainSputter = 0.0f;
        float m_releaseGain = 1.0f;
    };

    // ------------------------------------------------------------------
    // Chaff cartridge: a short dispenser crack, then a high-band rustle that
    // follows the cloud. Not a magnesium burn, so there is no low roar.
    // ------------------------------------------------------------------

    struct ChaffParams
    {
        float bloom = 1.0f; // current RCS / RCS at release, 0..1
    };

    class ChaffVoice : public SoundSource
    {
    public:
        explicit ChaffVoice(uint64_t seed);
        static EmitterSpec spec();

        void setParams(const ChaffParams &params) { m_params.write(params); }

        void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) override;
        bool isFinished() const override { return m_released && m_releaseGain <= 0.0f; }

    private:
        LatestValue<ChaffParams> m_params;
        Random m_random;
        int64_t m_sample = 0;
        float m_bloom = 1.0f;

        BandNoise m_rustle;
        Crackle m_ticks;
        Flicker m_rustleFlicker{0.008f, 0.22f};
        float m_gainRustle = 0.0f, m_gainTicks = 0.0f;
        float m_releaseGain = 1.0f;
    };

    // ------------------------------------------------------------------
    // Cold-launch ejection: gas-generator slam, canister/cell clank and the
    // hiss of venting gas (single omni lobe, one-shot).
    // ------------------------------------------------------------------

    class LaunchEjectVoice : public SoundSource
    {
    public:
        explicit LaunchEjectVoice(uint64_t seed);
        static EmitterSpec spec();

        void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) override;
        bool isFinished() const override { return m_sample >= m_lengthSamples; }

    private:
        Random m_random;
        int64_t m_sample = 0;
        int64_t m_lengthSamples = 0;
        BandNoise m_hiss;
        // Structural ring of the launch cell: damped modes excited by the slam.
        std::array<float, 4> m_modeHz{};
    };
}
