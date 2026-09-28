#include "Cues.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace missilesim::audio::synth
{
    namespace
    {
        constexpr float kSampleSeconds = 1.0f / kSampleRateF;

        float blockCoefficient(float seconds, int frames)
        {
            return smoothingCoefficient(seconds, static_cast<float>(frames));
        }

        float raisedCosine(float x)
        {
            const float t = clampf(x, 0.0f, 1.0f);
            return 0.5f - 0.5f * std::cos(kPi * t);
        }
    }

    // ==================================================================
    // Seeker tone
    // ==================================================================

    namespace
    {
        constexpr float kGrowlLowHz = 210.0f;
        constexpr float kGrowlHighHz = 660.0f;
        constexpr float kLockToneHz = 1170.0f;
        constexpr int kGrowlHarmonics = 6;

        constexpr float kClutterLevel = 0.03f;   // ~ -30 dBFS
        constexpr float kGrowlMinLevel = 0.018f; // faint target at the edge of the FOV
        constexpr float kGrowlMaxLevel = 0.11f;  // strong return
        constexpr float kLockLevel = 0.12f;      // ~ -18 dBFS
    }

    SeekerToneVoice::SeekerToneVoice(uint64_t seed) : m_random(seed)
    {
        m_growl.setTimeConstant(0.012f);
        m_clutter.set(140.0f, 900.0f);
    }

    void SeekerToneVoice::render(float *left, float *right, int frames)
    {
        const SeekerToneParams &params = m_params.read();
        m_power += ((params.powered ? 1.0f : 0.0f) - m_power) * blockCoefficient(0.08f, frames);
        m_signal += (clampf(params.signal, 0.0f, 1.0f) - m_signal) * blockCoefficient(0.15f, frames);
        m_lock += ((params.powered && params.locked ? 1.0f : 0.0f) - m_lock) * blockCoefficient(0.05f, frames);
        if (m_power < 1.0e-4f)
        {
            return;
        }

        // Reticle-chopped signal: the growl's roughness is a fast random
        // amplitude flutter that the lock tone no longer has.
        const float dt = static_cast<float>(frames) * kSampleSeconds;
        const float previousGrowl = m_growlGain;
        m_growlGain = std::exp(0.55f * m_growl.advance(m_random, dt) - 0.15f);
        const float growlHz = lerpf(kGrowlLowHz, kGrowlHighHz, std::pow(m_signal, 0.8f));
        const float growlLevel = lerpf(kGrowlMinLevel, kGrowlMaxLevel, m_signal) * (1.0f - m_lock) * m_power;
        const float clutterLevel = kClutterLevel * (1.0f - 0.6f * m_signal) * (1.0f - m_lock) * m_power;
        const float lockLevel = kLockLevel * m_lock * m_power;

        static const std::array<float, kGrowlHarmonics> harmonicWeights = []
        {
            std::array<float, kGrowlHarmonics> weights{};
            for (int h = 0; h < kGrowlHarmonics; ++h)
            {
                weights[h] = std::pow(static_cast<float>(h + 1), -1.3f);
            }
            return weights;
        }();

        for (int i = 0; i < frames; ++i)
        {
            const float flutter = lerpf(previousGrowl, m_growlGain, static_cast<float>(i) / static_cast<float>(frames));

            m_phase += growlHz * kSampleSeconds;
            m_phase -= std::floor(m_phase);
            float growl = 0.0f;
            for (int h = 0; h < kGrowlHarmonics; ++h)
            {
                growl += std::sin(kTwoPi * m_phase * static_cast<float>(h + 1)) * harmonicWeights[h];
            }

            m_lockPhase += kLockToneHz * kSampleSeconds;
            m_lockPhase -= std::floor(m_lockPhase);
            const float lockTone = std::sin(kTwoPi * m_lockPhase) + 0.12f * std::sin(2.0f * kTwoPi * m_lockPhase);

            const float sample = growl * 0.55f * growlLevel * flutter +
                                 m_clutter.process(m_random) * clutterLevel * flutter +
                                 lockTone * lockLevel;
            left[i] += sample;
            right[i] += sample;
        }
    }

    // ==================================================================
    // Missile approach warning
    // ==================================================================

    namespace
    {
        constexpr float kWarningHighHz = 1320.0f;
        constexpr float kWarningLowHz = 980.0f;
        constexpr float kWarningDuty = 0.55f;
        constexpr float kWarningEdgeSeconds = 0.004f;
    }

    void MissileWarningVoice::render(float *left, float *right, int frames)
    {
        const MissileWarningParams &params = m_params.read();
        m_active += ((params.active ? 1.0f : 0.0f) - m_active) * blockCoefficient(0.03f, frames);
        m_urgency += (clampf(params.urgency, 0.0f, 1.0f) - m_urgency) * blockCoefficient(0.3f, frames);
        if (m_active < 1.0e-4f)
        {
            // Restart on a fresh pulse next time the warning comes on.
            m_gate = 0.0f;
            m_highTone = true;
            return;
        }

        const float rate = 2.5f + 7.5f * m_urgency; // pulses per second
        const float pulseSeconds = kWarningDuty / rate;
        const float level = (0.08f + 0.06f * m_urgency) * m_active;

        for (int i = 0; i < frames; ++i)
        {
            m_gate += rate * kSampleSeconds;
            if (m_gate >= 1.0f)
            {
                m_gate -= 1.0f;
                m_highTone = !m_highTone;
            }

            float envelope = 0.0f;
            if (m_gate < kWarningDuty)
            {
                const float t = m_gate / rate;
                envelope = raisedCosine(std::min(t, pulseSeconds - t) / kWarningEdgeSeconds);
            }

            m_phase += (m_highTone ? kWarningHighHz : kWarningLowHz) * kSampleSeconds;
            m_phase -= std::floor(m_phase);
            // Slightly hollow timbre (odd harmonic) so it cuts through the roar.
            const float tone = std::sin(kTwoPi * m_phase) + 0.22f * std::sin(3.0f * kTwoPi * m_phase);
            const float sample = tone * envelope * level;
            left[i] += sample;
            right[i] += sample;
        }
    }

    // ==================================================================
    // Listener wind
    // ==================================================================

    namespace
    {
        constexpr float kWindReferencePa = 0.02f; // ~60 dB SPL at 10 m/s
        // ~124 dB: wind conveys speed but stays well under the engines the
        // chase cameras sit behind.
        constexpr float kWindMaxPa = 30.0f;
    }

    ListenerWindVoice::ListenerWindVoice(uint64_t seed) : m_random(seed)
    {
        m_gustLeft.setDepth(0.35f, 0.3f);
        m_gustRight.setDepth(0.35f, 0.3f);
        m_buffetLeft.set(12.0f, 90.0f);
        m_buffetRight.set(12.0f, 90.0f);
    }

    void ListenerWindVoice::render(float *left, float *right, int frames)
    {
        m_speed += (std::max(m_airspeed.read(), 0.0f) - m_speed) * blockCoefficient(0.2f, frames);
        const float relative = m_speed / 10.0f;
        const float target = std::min(kWindReferencePa * relative * relative, kWindMaxPa);
        const float previous = m_gain;
        m_gain = target;
        if (previous < 1.0e-5f && m_gain < 1.0e-5f)
        {
            return;
        }

        const float rushTop = clampf(400.0f + 15.0f * m_speed, 600.0f, 8000.0f);
        m_rushLeft.set(120.0f, rushTop);
        m_rushRight.set(120.0f, rushTop);

        const float previousGustLeft = m_gustGainLeft;
        const float previousGustRight = m_gustGainRight;
        m_gustGainLeft = m_gustLeft.advance(m_random, frames);
        m_gustGainRight = m_gustRight.advance(m_random, frames);

        constexpr float kShare = 0.70710678f; // rush and buffet carry equal power
        for (int i = 0; i < frames; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(frames);
            const float gain = lerpf(previous, m_gain, t) * kShare;
            const float gustLeft = lerpf(previousGustLeft, m_gustGainLeft, t);
            const float gustRight = lerpf(previousGustRight, m_gustGainRight, t);
            left[i] += (m_rushLeft.process(m_random) + m_buffetLeft.process(m_random)) * gain * gustLeft;
            right[i] += (m_rushRight.process(m_random) + m_buffetRight.process(m_random)) * gain * gustRight;
        }
    }
}
