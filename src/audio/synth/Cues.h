#pragma once

// Listener-attached sounds: the pilot's headset cues and wind on the listener.
//
// Headset cues are electronic signals, so they are designed as clean,
// band-limited tones on the headset bus (digital full scale), untouched by the
// world's exposure. Wind is an acoustic pressure at the ears and is mixed with
// the world.
//
// Threading: set*() on the game thread, render() on the audio thread.

#include "../engine/AudioEngine.h"
#include "Synthesis.h"

#include <cstdint>

namespace missilesim::audio::synth
{
    // ------------------------------------------------------------------
    // Infrared seeker audio (Sidewinder-style). While searching the seeker
    // hears background clutter as a soft, low growl; a target in the field of
    // view raises the growl in pitch and level as the signal strengthens, and a
    // lock turns it into a steady, clean tone.
    // ------------------------------------------------------------------

    struct SeekerToneParams
    {
        bool powered = false;
        bool locked = false;
        float signal = 0.0f; // 0 = clutter only, 1 = strong target return
    };

    class SeekerToneVoice : public LocalSource
    {
    public:
        explicit SeekerToneVoice(uint64_t seed);

        void setParams(const SeekerToneParams &params) { m_params.write(params); }

        Bus bus() const override { return Bus::Headset; }
        void render(float *left, float *right, int frames) override;

    private:
        LatestValue<SeekerToneParams> m_params;
        Random m_random;
        RandomWalk m_growl;
        BandNoise m_clutter;
        float m_phase = 0.0f;
        float m_lockPhase = 0.0f;
        float m_power = 0.0f; // smoothed 0..1
        float m_signal = 0.0f;
        float m_lock = 0.0f;
        float m_growlGain = 1.0f;
    };

    // ------------------------------------------------------------------
    // Missile approach warning: a two-tone warble whose repetition rate and
    // level climb with the threat's urgency.
    // ------------------------------------------------------------------

    enum class WarningTimbre : std::uint8_t
    {
        Off = 0,
        Search,   // slow single beep
        Track,    // faster single beep
        Launch,   // rapid, long beeps at the track pitch: a round is on its way
        Seeker,   // two-tone warble, higher than an infrared approach
        Approach, // the existing missile-approach warble
    };

    struct MissileWarningParams
    {
        WarningTimbre timbre = WarningTimbre::Off;
        float urgency = 0.0f; // 0..1
    };

    class MissileWarningVoice : public LocalSource
    {
    public:
        MissileWarningVoice() = default;

        void setParams(const MissileWarningParams &params) { m_params.write(params); }

        Bus bus() const override { return Bus::Headset; }
        void render(float *left, float *right, int frames) override;

    private:
        LatestValue<MissileWarningParams> m_params;
        float m_gate = 0.0f;    // 0..1 position inside the current pulse cycle
        float m_phase = 0.0f;   // tone oscillator phase (cycles)
        float m_active = 0.0f;  // smoothed on/off
        float m_urgency = 0.0f; // smoothed
        bool m_highTone = true;
        // The tone that is fading out keeps its own voice instead of the default warble.
        WarningTimbre m_timbre = WarningTimbre::Approach;
    };

    // ------------------------------------------------------------------
    // Air rushing past the listener: broadband rush plus low buffeting,
    // decorrelated between the ears, growing with the square of airspeed.
    // ------------------------------------------------------------------

    class ListenerWindVoice : public LocalSource
    {
    public:
        explicit ListenerWindVoice(uint64_t seed);

        // Airspeed of the listener relative to still air (m/s, physical).
        void setAirspeed(float metersPerSecond) { m_airspeed.write(metersPerSecond); }

        Bus bus() const override { return Bus::Acoustic; }
        void render(float *left, float *right, int frames) override;

    private:
        LatestValue<float> m_airspeed;
        Random m_random;
        float m_speed = 0.0f;
        float m_gain = 0.0f;
        BandNoise m_rushLeft, m_rushRight, m_buffetLeft, m_buffetRight;
        Flicker m_gustLeft{0.05f, 0.9f};
        Flicker m_gustRight{0.05f, 0.9f};
        float m_gustGainLeft = 1.0f;
        float m_gustGainRight = 1.0f;
    };
}
