#pragma once

// Physically motivated synthesis building blocks shared by all voices.
//
// Levels are derived from real acoustics wherever possible:
//   * jet / rocket noise power = efficiency(M) * mechanical jet power (Lighthill
//     M^5 scaling, saturating at ~0.6% for rockets, Eldred/NASA SP-8072),
//   * pressure at 1 m from sound power via p^2 = W * rho * c / (4 pi),
//   * spectral peaks from Strouhal scaling (f = St * U / D).
// Generators therefore emit plausible absolute pressures (Pa at 1 m), and the
// engine's propagation and exposure stages take it from there.

#include "../engine/Acoustics.h"
#include "../engine/Dsp.h"

#include <array>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace missilesim::audio::synth
{
    constexpr float kAirImpedance = 413.0f; // rho * c at sea level (Pa s / m)

    // RMS pressure at 1 m for an omnidirectional source of the given power.
    inline float pressureAtOneMeter(float acousticWatts, float impedance = kAirImpedance)
    {
        return std::sqrt(std::max(acousticWatts, 0.0f) * impedance / (4.0f * kPi));
    }

    // Fraction of mechanical jet power radiated as sound.
    inline float jetAcousticEfficiency(float jetMach)
    {
        const float m = std::max(jetMach, 0.0f);
        return clampf(1.0e-5f * m * m * m * m * m, 1.0e-7f, 0.006f);
    }

    // Builds a directivity pattern from 9 dB values (0..180 deg from the nose)
    // and offsets it so the pattern radiates unit power averaged over the sphere.
    // The generator's omni-equivalent level then maps to real sound power.
    inline DirectivityPattern powerNormalisedPattern(std::initializer_list<float> db)
    {
        DirectivityPattern pattern;
        int i = 0;
        for (float value : db)
        {
            if (i < DirectivityPattern::kPoints)
            {
                pattern.gainDb[i++] = value;
            }
        }

        // Integrate g^2(theta) * sin(theta) over the sphere (trapezoid on a fine grid).
        constexpr int kSteps = 180;
        double integral = 0.0;
        for (int s = 0; s <= kSteps; ++s)
        {
            const float theta = kPi * static_cast<float>(s) / kSteps;
            const float g = pattern.gainAt(std::cos(theta));
            const double weight = (s == 0 || s == kSteps) ? 0.5 : 1.0;
            integral += weight * g * g * std::sin(theta);
        }
        integral *= (kPi / kSteps) * 0.5; // mean over the sphere
        const float offsetDb = -10.0f * std::log10(static_cast<float>(std::max(integral, 1.0e-9)));
        for (float &value : pattern.gainDb)
        {
            value += offsetDb;
        }
        return pattern;
    }

    // Band-limited turbulence noise with a calibrated RMS: white noise through a
    // 2nd-order high-pass and 2nd-order low-pass (Butterworth-like), normalised
    // by the equivalent noise bandwidth so `process` returns unit-RMS noise.
    class BandNoise
    {
    public:
        void set(float lowHz, float highHz)
        {
            const float nyquist = kSampleRateF * 0.5f;
            m_lowHz = clampf(lowHz, 5.0f, nyquist * 0.9f);
            m_highHz = clampf(highHz, m_lowHz * 1.05f, nyquist * 0.98f);
            m_highPass.set(m_lowHz, 0.7071f);
            m_lowPass.set(m_highHz, 0.7071f);
            // ENBW of a 2nd-order Butterworth low-pass is 1.11 fc; subtract the
            // band removed by the high-pass (approx 0.9 fc).
            const float enbw = std::max(1.11f * m_highHz - 0.9f * m_lowHz, 0.25f * m_highHz);
            m_gain = std::sqrt(nyquist / enbw);
        }

        float process(Random &random)
        {
            // Uniform white noise (unit variance); the filtering makes it Gaussian.
            const float white = random.bipolar() * 1.7320508f;
            return m_lowPass.lowpass(m_highPass.highpass(white)) * m_gain;
        }

    private:
        Svf m_highPass;
        Svf m_lowPass;
        float m_lowHz = 100.0f;
        float m_highHz = 1000.0f;
        float m_gain = 1.0f;
    };

    // Organic amplitude modulation: log-normal flicker built from two random
    // walks (fast turbulence puffs + slow plume breathing). Mean power ~1.
    class Flicker
    {
    public:
        Flicker(float fastSeconds = 0.03f, float slowSeconds = 0.35f)
        {
            m_fast.setTimeConstant(fastSeconds);
            m_slow.setTimeConstant(slowSeconds);
        }

        void setDepth(float fastDepth, float slowDepth)
        {
            m_fastDepth = fastDepth;
            m_slowDepth = slowDepth;
        }

        // Advance once per `interval` samples; returns the gain to apply.
        float advance(Random &random, int intervalSamples)
        {
            const float dt = static_cast<float>(intervalSamples) / kSampleRateF;
            const float fast = m_fast.advance(random, dt);
            const float slow = m_slow.advance(random, dt);
            const float sigma2 = m_fastDepth * m_fastDepth + m_slowDepth * m_slowDepth;
            // exp(sigma*N) has mean power exp(2 sigma^2); compensate.
            return std::exp(m_fastDepth * fast + m_slowDepth * slow - sigma2);
        }

    private:
        RandomWalk m_fast;
        RandomWalk m_slow;
        float m_fastDepth = 0.3f;
        float m_slowDepth = 0.2f;
    };

    // Jet "crackle": a Poisson train of steepened shocklets. Each event is a
    // near-instant compression followed by an exponential expansion; a gentle
    // high-pass supplies the rarefaction, giving the strongly positive-skewed
    // pressure signature that makes supersonic exhaust sound torn, not hissy.
    class Crackle
    {
    public:
        void set(float eventsPerSecond, float riseSeconds, float decaySeconds)
        {
            m_probability = clampf(eventsPerSecond / kSampleRateF, 0.0f, 0.5f);
            m_riseRetain = std::exp(-1.0f / std::max(riseSeconds * kSampleRateF, 0.2f));
            m_decayRetain = std::exp(-1.0f / std::max(decaySeconds * kSampleRateF, 0.5f));

            // Energy of one unit event (difference of exponentials), summed over samples.
            const double a = m_decayRetain;
            const double b = m_riseRetain;
            const double energy = 1.0 / (1.0 - a * a) - 2.0 / (1.0 - a * b) + 1.0 / (1.0 - b * b);
            // Amplitude distribution: exponential variates raised to 1.4 -> E[x^2] = Gamma(3.8) = 4.69.
            const double meanSquare = std::max(static_cast<double>(m_probability) * 4.69 * energy, 1.0e-12);
            m_gain = static_cast<float>(1.0 / std::sqrt(meanSquare));
            m_highPass.setCutoff(180.0f);
        }

        // Unit-RMS crackle.
        float process(Random &random)
        {
            float impulse = 0.0f;
            if (random.unit() < m_probability)
            {
                const float u = std::max(random.unit(), 1.0e-6f);
                impulse = std::pow(-std::log(u), 1.4f);
            }
            m_slow = m_slow * m_decayRetain + impulse;
            m_fast = m_fast * m_riseRetain + impulse;
            return m_highPass.highpass(m_slow - m_fast) * m_gain;
        }

    private:
        float m_probability = 0.0f;
        float m_riseRetain = 0.0f;
        float m_decayRetain = 0.0f;
        float m_slow = 0.0f;
        float m_fast = 0.0f;
        float m_gain = 1.0f;
        OnePole m_highPass;
    };

    // Single-cycle wavetable oscillator (for dense harmonic spectra such as fan
    // buzz-saw tones) with linear interpolation.
    class Wavetable
    {
    public:
        static constexpr int kSize = 4096;

        // Harmonic amplitudes/phases define the cycle; normalised to unit RMS.
        void build(const std::vector<float> &amplitudes, const std::vector<float> &phases)
        {
            m_table.assign(kSize + 1, 0.0f);
            double sumSquares = 0.0;
            for (size_t h = 0; h < amplitudes.size(); ++h)
            {
                sumSquares += 0.5 * amplitudes[h] * amplitudes[h];
            }
            const float norm = sumSquares > 0.0 ? static_cast<float>(1.0 / std::sqrt(sumSquares)) : 0.0f;
            for (int i = 0; i < kSize; ++i)
            {
                double v = 0.0;
                const double x = 2.0 * 3.14159265358979 * i / kSize;
                for (size_t h = 0; h < amplitudes.size(); ++h)
                {
                    v += amplitudes[h] * std::sin(x * static_cast<double>(h + 1) + phases[h]);
                }
                m_table[i] = static_cast<float>(v) * norm;
            }
            m_table[kSize] = m_table[0];
        }

        float process(float frequencyHz)
        {
            m_phase += frequencyHz / kSampleRateF;
            m_phase -= std::floor(m_phase);
            const float position = m_phase * kSize;
            const int index = static_cast<int>(position);
            const float t = position - static_cast<float>(index);
            return lerpf(m_table[index], m_table[index + 1], t);
        }

        bool ready() const { return !m_table.empty(); }

    private:
        std::vector<float> m_table;
        float m_phase = 0.0f;
    };

    // Sine oscillator with phase accumulation (unit amplitude).
    class Sine
    {
    public:
        float process(float frequencyHz)
        {
            m_phase += frequencyHz / kSampleRateF;
            m_phase -= std::floor(m_phase);
            return std::sin(kTwoPi * m_phase);
        }

        void setPhase(float phase) { m_phase = phase; }

    private:
        float m_phase = 0.0f;
    };

    // Kinney & Graham free-air blast parameters for a TNT-equivalent charge.
    struct BlastParameters
    {
        float peakOverpressurePa = 0.0f;
        float positiveDurationSeconds = 0.0f;
    };

    inline BlastParameters kinneyGrahamBlast(float chargeKg, float distanceMeters, float ambientPressurePa = 101325.0f)
    {
        const float cubeRoot = std::cbrt(std::max(chargeKg, 0.01f));
        const float z = std::max(distanceMeters, 0.5f) / cubeRoot;
        const float ratio = 808.0f * (1.0f + (z / 4.5f) * (z / 4.5f)) /
                            std::sqrt((1.0f + (z / 0.048f) * (z / 0.048f)) *
                                      (1.0f + (z / 0.32f) * (z / 0.32f)) *
                                      (1.0f + (z / 1.35f) * (z / 1.35f)));
        const double numerator = 980.0 * (1.0 + std::pow(z / 0.54, 10.0));
        const double denominator = (1.0 + std::pow(z / 0.02, 3.0)) * (1.0 + std::pow(z / 0.74, 6.0)) *
                                   std::sqrt(1.0 + std::pow(z / 6.9, 2.0));
        BlastParameters result;
        result.peakOverpressurePa = ratio * ambientPressurePa;
        result.positiveDurationSeconds = static_cast<float>(numerator / denominator) * 0.001f * cubeRoot;
        return result;
    }
}
