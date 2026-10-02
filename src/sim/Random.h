#pragma once

#include <cstdint>
#include <string_view>

namespace missilesim::sim
{
    // Deterministic random numbers for the simulation.
    //
    // std::uniform_real_distribution and friends are implementation-defined, so
    // the same seed can give different targets on a different standard library.
    // Everything here is fully specified integer arithmetic: a seed reproduces
    // the same draws on every compiler and platform.
    //
    // Each consumer draws from its own named stream (RandomStreams::stream), so
    // adding a draw in one system does not shift the numbers another system
    // sees, and a scenario stays reproducible as the game grows.

    // 64-bit FNV-1a, used to turn a stream name into a stable key.
    constexpr std::uint64_t fnv1a64(std::string_view text, std::uint64_t hash = 0xcbf29ce484222325ull)
    {
        for (const char c : text)
        {
            hash ^= static_cast<std::uint8_t>(c);
            hash *= 0x100000001b3ull;
        }
        return hash;
    }

    // SplitMix64 (Steele, Lea and Flood, 2014): decorrelates nearby seeds.
    constexpr std::uint64_t splitMix64(std::uint64_t &state)
    {
        std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
        return z ^ (z >> 31);
    }

    // PCG32 (O'Neill, 2014), XSH-RR output on a 64-bit LCG state.
    class RandomStream
    {
    public:
        constexpr RandomStream() = default;
        constexpr RandomStream(std::uint64_t seed, std::uint64_t sequence) { reseed(seed, sequence); }

        constexpr void reseed(std::uint64_t seed, std::uint64_t sequence)
        {
            m_state = 0;
            m_increment = (sequence << 1u) | 1u;
            nextU32();
            m_state += seed;
            nextU32();
        }

        constexpr std::uint32_t nextU32()
        {
            const std::uint64_t old = m_state;
            m_state = old * 6364136223846793005ull + m_increment;
            const auto xorShifted = static_cast<std::uint32_t>(((old >> 18u) ^ old) >> 27u);
            const auto rotation = static_cast<std::uint32_t>(old >> 59u);
            return (xorShifted >> rotation) | (xorShifted << ((32u - rotation) & 31u));
        }

        // Uniform in [0, 1): the top 24 bits fill a float mantissa exactly.
        float uniform01() { return static_cast<float>(nextU32() >> 8) * (1.0f / 16777216.0f); }

        // Uniform in [low, high). Reversed bounds are swapped; equal bounds return low.
        float uniform(float low, float high)
        {
            if (high < low)
            {
                const float swap = low;
                low = high;
                high = swap;
            }
            return low + (high - low) * uniform01();
        }

        // Uniform integer in [0, bound) without modulo bias (Lemire, 2019).
        std::uint32_t below(std::uint32_t bound)
        {
            if (bound == 0)
            {
                return 0;
            }
            std::uint64_t product = static_cast<std::uint64_t>(nextU32()) * bound;
            auto low = static_cast<std::uint32_t>(product);
            if (low < bound)
            {
                const std::uint32_t threshold = (0u - bound) % bound;
                while (low < threshold)
                {
                    product = static_cast<std::uint64_t>(nextU32()) * bound;
                    low = static_cast<std::uint32_t>(product);
                }
            }
            return static_cast<std::uint32_t>(product >> 32u);
        }

    private:
        std::uint64_t m_state = 0x853c49e6748fea9bull;
        std::uint64_t m_increment = 0xda3e39cb94b95bdbull;
    };

    // The scenario seed plus a stream name gives an independent stream.
    class RandomStreams
    {
    public:
        explicit RandomStreams(std::uint64_t seed = 0) : m_seed(seed) {}

        std::uint64_t seed() const { return m_seed; }

        RandomStream stream(std::string_view name) const
        {
            std::uint64_t mix = m_seed ^ fnv1a64(name);
            const std::uint64_t streamSeed = splitMix64(mix);
            const std::uint64_t sequence = splitMix64(mix);
            return RandomStream(streamSeed, sequence);
        }

    private:
        std::uint64_t m_seed;
    };
}
