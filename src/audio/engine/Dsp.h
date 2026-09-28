#pragma once

// Real-time DSP building blocks shared by the acoustics engine and the sound
// generators. Everything here is allocation-free and safe to call from the
// audio thread. Filters follow the topology-preserving-transform (TPT) forms
// so coefficients can be modulated every block without zipper artefacts.

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace missilesim::audio
{
    constexpr int kSampleRate = 48000;
    constexpr float kSampleRateF = static_cast<float>(kSampleRate);
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;

    // Reference pressure for sound pressure level (dB SPL re 20 uPa).
    constexpr float kReferencePressurePa = 20.0e-6f;

    inline float dbToGain(float db) { return std::pow(10.0f, db * 0.05f); }
    inline float gainToDb(float gain) { return 20.0f * std::log10(gain > 1.0e-12f ? gain : 1.0e-12f); }
    inline float pascalsToSpl(float pa) { return gainToDb(pa / kReferencePressurePa); }
    inline float splToPascals(float spl) { return kReferencePressurePa * dbToGain(spl); }

    inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
    inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

    // Coefficient for a one-pole smoother that reaches ~63% of a step in `seconds`
    // when advanced once per `intervalSamples`.
    inline float smoothingCoefficient(float seconds, float intervalSamples = 1.0f)
    {
        if (seconds <= 0.0f)
        {
            return 1.0f;
        }
        return 1.0f - std::exp(-intervalSamples / (seconds * kSampleRateF));
    }

    // 4-point, 3rd-order Hermite interpolation (x = fractional position in [0,1)).
    inline float hermite4(float xm1, float x0, float x1, float x2, float t)
    {
        const float c1 = 0.5f * (x1 - xm1);
        const float c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
        const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
        return ((c3 * t + c2) * t + c1) * t + x0;
    }

    // Small, fast, statistically solid PRNG (xoshiro128+ style mixing of a 64-bit LCG).
    class Random
    {
    public:
        explicit Random(uint64_t seed = 0x9E3779B97F4A7C15ull) { reseed(seed); }

        void reseed(uint64_t seed)
        {
            m_state = seed ? seed : 0x9E3779B97F4A7C15ull;
            next();
            next();
        }

        uint32_t next()
        {
            // PCG-XSH-RR 64/32
            const uint64_t old = m_state;
            m_state = old * 6364136223846793005ull + 1442695040888963407ull;
            const uint32_t xorshifted = static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
            const uint32_t rot = static_cast<uint32_t>(old >> 59u);
            return (xorshifted >> rot) | (xorshifted << ((32u - rot) & 31u));
        }

        // Uniform in [0, 1).
        float unit() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }
        // Uniform in [-1, 1).
        float bipolar() { return unit() * 2.0f - 1.0f; }
        // Approximately standard-normal (Irwin-Hall with 4 terms, variance-corrected).
        float gaussian()
        {
            const float sum = unit() + unit() + unit() + unit();
            return (sum - 2.0f) * 1.7320508f;
        }

    private:
        uint64_t m_state = 0;
    };

    // Pink (1/f) noise, Paul Kellet's refined filter. Unit-ish RMS.
    class PinkNoise
    {
    public:
        float process(float white)
        {
            m_b0 = 0.99886f * m_b0 + white * 0.0555179f;
            m_b1 = 0.99332f * m_b1 + white * 0.0750759f;
            m_b2 = 0.96900f * m_b2 + white * 0.1538520f;
            m_b3 = 0.86650f * m_b3 + white * 0.3104856f;
            m_b4 = 0.55000f * m_b4 + white * 0.5329522f;
            m_b5 = -0.7616f * m_b5 - white * 0.0168980f;
            const float pink = m_b0 + m_b1 + m_b2 + m_b3 + m_b4 + m_b5 + m_b6 + white * 0.5362f;
            m_b6 = white * 0.115926f;
            return pink * 0.11f;
        }

    private:
        float m_b0 = 0.0f, m_b1 = 0.0f, m_b2 = 0.0f, m_b3 = 0.0f, m_b4 = 0.0f, m_b5 = 0.0f, m_b6 = 0.0f;
    };

    // TPT one-pole filter providing simultaneous low-pass and high-pass outputs.
    class OnePole
    {
    public:
        void setCutoff(float hz)
        {
            const float fc = clampf(hz, 1.0f, kSampleRateF * 0.49f);
            const float g = std::tan(kPi * fc / kSampleRateF);
            m_g = g / (1.0f + g);
        }

        float lowpass(float x)
        {
            const float v = (x - m_s) * m_g;
            const float y = v + m_s;
            m_s = y + v;
            return y;
        }

        float highpass(float x) { return x - lowpass(x); }

        void reset(float value = 0.0f) { m_s = value; }

    private:
        float m_g = 1.0f;
        float m_s = 0.0f;
    };

    // Cytomic / Simper TPT state-variable filter. Stable under fast modulation.
    class Svf
    {
    public:
        struct Outputs
        {
            float low;
            float band;
            float high;
        };

        void set(float cutoffHz, float q)
        {
            const float fc = clampf(cutoffHz, 5.0f, kSampleRateF * 0.49f);
            m_g = std::tan(kPi * fc / kSampleRateF);
            m_k = 1.0f / (q > 0.05f ? q : 0.05f);
            m_a1 = 1.0f / (1.0f + m_g * (m_g + m_k));
            m_a2 = m_g * m_a1;
            m_a3 = m_g * m_a2;
        }

        Outputs process(float x)
        {
            const float v3 = x - m_ic2;
            const float v1 = m_a1 * m_ic1 + m_a2 * v3;
            const float v2 = m_ic2 + m_a2 * m_ic1 + m_a3 * v3;
            m_ic1 = 2.0f * v1 - m_ic1;
            m_ic2 = 2.0f * v2 - m_ic2;
            return {v2, v1, x - m_k * v1 - v2};
        }

        float lowpass(float x) { return process(x).low; }
        float bandpass(float x) { return process(x).band; }
        float highpass(float x) { return process(x).high; }
        // Constant-peak-gain band-pass (unity at the centre frequency).
        float bandpassNormalized(float x) { return process(x).band * m_k; }

        void reset()
        {
            m_ic1 = 0.0f;
            m_ic2 = 0.0f;
        }

    private:
        float m_g = 0.0f, m_k = 1.0f, m_a1 = 1.0f, m_a2 = 0.0f, m_a3 = 0.0f;
        float m_ic1 = 0.0f, m_ic2 = 0.0f;
    };

    // RBJ-cookbook biquad (transposed direct form II) for static EQ shapes.
    class Biquad
    {
    public:
        void setLowShelf(float hz, float gainDb, float slope = 1.0f) { design(Shape::LowShelf, hz, gainDb, slope); }
        void setHighShelf(float hz, float gainDb, float slope = 1.0f) { design(Shape::HighShelf, hz, gainDb, slope); }
        void setPeak(float hz, float gainDb, float q) { design(Shape::Peak, hz, gainDb, q); }
        void setHighPass(float hz, float q) { design(Shape::HighPass, hz, 0.0f, q); }
        void setLowPass(float hz, float q) { design(Shape::LowPass, hz, 0.0f, q); }

        float process(float x)
        {
            const float y = m_b0 * x + m_z1;
            m_z1 = m_b1 * x - m_a1 * y + m_z2;
            m_z2 = m_b2 * x - m_a2 * y;
            return y;
        }

        void reset()
        {
            m_z1 = 0.0f;
            m_z2 = 0.0f;
        }

    private:
        enum class Shape
        {
            LowShelf,
            HighShelf,
            Peak,
            HighPass,
            LowPass
        };

        void design(Shape shape, float hz, float gainDb, float qOrSlope)
        {
            const float w0 = kTwoPi * clampf(hz, 5.0f, kSampleRateF * 0.49f) / kSampleRateF;
            const float cw = std::cos(w0);
            const float sw = std::sin(w0);
            const float a = std::pow(10.0f, gainDb / 40.0f);
            float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;

            switch (shape)
            {
            case Shape::LowShelf:
            case Shape::HighShelf:
            {
                const float alpha = sw * 0.5f * std::sqrt((a + 1.0f / a) * (1.0f / qOrSlope - 1.0f) + 2.0f);
                const float sqa = 2.0f * std::sqrt(a) * alpha;
                if (shape == Shape::LowShelf)
                {
                    b0 = a * ((a + 1.0f) - (a - 1.0f) * cw + sqa);
                    b1 = 2.0f * a * ((a - 1.0f) - (a + 1.0f) * cw);
                    b2 = a * ((a + 1.0f) - (a - 1.0f) * cw - sqa);
                    a0 = (a + 1.0f) + (a - 1.0f) * cw + sqa;
                    a1 = -2.0f * ((a - 1.0f) + (a + 1.0f) * cw);
                    a2 = (a + 1.0f) + (a - 1.0f) * cw - sqa;
                }
                else
                {
                    b0 = a * ((a + 1.0f) + (a - 1.0f) * cw + sqa);
                    b1 = -2.0f * a * ((a - 1.0f) + (a + 1.0f) * cw);
                    b2 = a * ((a + 1.0f) + (a - 1.0f) * cw - sqa);
                    a0 = (a + 1.0f) - (a - 1.0f) * cw + sqa;
                    a1 = 2.0f * ((a - 1.0f) - (a + 1.0f) * cw);
                    a2 = (a + 1.0f) - (a - 1.0f) * cw - sqa;
                }
                break;
            }
            case Shape::Peak:
            {
                const float alpha = sw / (2.0f * qOrSlope);
                b0 = 1.0f + alpha * a;
                b1 = -2.0f * cw;
                b2 = 1.0f - alpha * a;
                a0 = 1.0f + alpha / a;
                a1 = -2.0f * cw;
                a2 = 1.0f - alpha / a;
                break;
            }
            case Shape::HighPass:
            {
                const float alpha = sw / (2.0f * qOrSlope);
                b0 = (1.0f + cw) * 0.5f;
                b1 = -(1.0f + cw);
                b2 = (1.0f + cw) * 0.5f;
                a0 = 1.0f + alpha;
                a1 = -2.0f * cw;
                a2 = 1.0f - alpha;
                break;
            }
            case Shape::LowPass:
            {
                const float alpha = sw / (2.0f * qOrSlope);
                b0 = (1.0f - cw) * 0.5f;
                b1 = 1.0f - cw;
                b2 = (1.0f - cw) * 0.5f;
                a0 = 1.0f + alpha;
                a1 = -2.0f * cw;
                a2 = 1.0f - alpha;
                break;
            }
            }

            const float inv = 1.0f / a0;
            m_b0 = b0 * inv;
            m_b1 = b1 * inv;
            m_b2 = b2 * inv;
            m_a1 = a1 * inv;
            m_a2 = a2 * inv;
        }

        float m_b0 = 1.0f, m_b1 = 0.0f, m_b2 = 0.0f, m_a1 = 0.0f, m_a2 = 0.0f;
        float m_z1 = 0.0f, m_z2 = 0.0f;
    };

    // Ornstein-Uhlenbeck process: band-limited random wander with unit variance.
    // Used for turbulence, flame flicker and anything that must drift organically.
    class RandomWalk
    {
    public:
        void setTimeConstant(float seconds) { m_timeConstant = seconds > 1.0e-4f ? seconds : 1.0e-4f; }

        float advance(Random &rng, float dtSeconds)
        {
            const float theta = dtSeconds / m_timeConstant;
            m_value += -m_value * theta + std::sqrt(2.0f * theta) * rng.gaussian();
            return m_value;
        }

        float value() const { return m_value; }

    private:
        float m_timeConstant = 1.0f;
        float m_value = 0.0f;
    };

    // Lock-free "latest value" mailbox (triple buffer) for one writer thread and
    // one reader thread. Writes never block; the reader always sees a complete,
    // most-recent value.
    template <typename T>
    class LatestValue
    {
    public:
        LatestValue() = default;
        explicit LatestValue(const T &initial) { m_buffers.fill(initial); }

        void write(const T &value)
        {
            m_buffers[m_writeIndex] = value;
            const uint8_t previous = m_middle.exchange(static_cast<uint8_t>(m_writeIndex | kDirtyBit), std::memory_order_acq_rel);
            m_writeIndex = static_cast<uint8_t>(previous & kIndexMask);
        }

        // Returns the newest value. `changed` reports whether a new write arrived.
        const T &read(bool *changed = nullptr)
        {
            const bool dirty = (m_middle.load(std::memory_order_acquire) & kDirtyBit) != 0;
            if (dirty)
            {
                const uint8_t previous = m_middle.exchange(m_readIndex, std::memory_order_acq_rel);
                m_readIndex = static_cast<uint8_t>(previous & kIndexMask);
            }
            if (changed)
            {
                *changed = dirty;
            }
            return m_buffers[m_readIndex];
        }

    private:
        static constexpr uint8_t kDirtyBit = 0x80;
        static constexpr uint8_t kIndexMask = 0x03;

        std::array<T, 3> m_buffers{};
        std::atomic<uint8_t> m_middle{1};
        uint8_t m_writeIndex = 0;
        uint8_t m_readIndex = 2;
    };

    // Bounded single-producer / single-consumer queue.
    template <typename T, size_t Capacity>
    class SpscQueue
    {
        static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

    public:
        bool push(const T &value)
        {
            const size_t head = m_head.load(std::memory_order_relaxed);
            const size_t tail = m_tail.load(std::memory_order_acquire);
            if (head - tail >= Capacity)
            {
                return false;
            }
            m_items[head & (Capacity - 1)] = value;
            m_head.store(head + 1, std::memory_order_release);
            return true;
        }

        bool pop(T &out)
        {
            const size_t tail = m_tail.load(std::memory_order_relaxed);
            const size_t head = m_head.load(std::memory_order_acquire);
            if (tail == head)
            {
                return false;
            }
            out = m_items[tail & (Capacity - 1)];
            m_tail.store(tail + 1, std::memory_order_release);
            return true;
        }

    private:
        std::array<T, Capacity> m_items{};
        std::atomic<size_t> m_head{0};
        std::atomic<size_t> m_tail{0};
    };
}
