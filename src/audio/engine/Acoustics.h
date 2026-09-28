#pragma once

// Physical sound propagation.
//
// Every emitter synthesizes its sound at the *source* (pressure in pascals at a
// 1 m reference distance) into an emission history. Listeners never hear "the
// emitter now": for every ear and every propagation path the engine solves the
// retarded-time equation
//
//     c * (t - tau) = |ear(t) - source(tau)|
//
// and reads the history at the emission time tau. This single equation yields,
// with no special cases: propagation delay (see the flash, hear the bang),
// exact Doppler shift for moving sources *and* listeners, interaural time
// differences, and supersonic behaviour (silence ahead of the Mach cone, a
// shock when it sweeps past, and the time-reversed sound that follows). The
// history is stored as a small mip pyramid so heavily Doppler-compressed reads
// stay alias-free.
//
// On top of the geometry each path gets spherical spreading, convective
// amplification, source directivity, ISO 9613-1 atmospheric absorption,
// turbulence scintillation, a ground-reflected image path and head shadowing.

#include "Dsp.h"

#include <glm/glm.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

namespace missilesim::audio
{
    constexpr int kBlockSize = 64;
    constexpr int kMaxLobes = 3;
    constexpr int kMipLevels = 5;
    constexpr int kEarCount = 2;
    constexpr int kPathCount = 2; // direct + ground reflection
    constexpr int kMaxBranches = 4;

    // ------------------------------------------------------------------
    // Source-side interfaces
    // ------------------------------------------------------------------

    // Per-block state handed to a generator. Positions/velocities are world-space
    // SI units in *simulation* time (physical airspeed, physical speed of sound),
    // independent of any simulation speed-up; `axis` is the unit body-forward
    // (nose) direction.
    struct SourceContext
    {
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 axis{0.0f, 0.0f, -1.0f};
        float speedOfSound = 340.0f;
        double timeSeconds = 0.0;
    };

    // A sound generator. Runs exclusively on the audio thread.
    class SoundSource
    {
    public:
        virtual ~SoundSource() = default;

        // Write `frames` samples of emitted pressure (Pa at 1 m) into each lobe
        // buffer. Buffers arrive zeroed; the generator adds its content.
        virtual void render(const SourceContext &context, float *const *lobes, int lobeCount, int frames) = 0;

        // The owner stopped the emitter; begin a natural tail-off.
        virtual void release() { m_released = true; }

        // True once the generator will only produce silence from now on.
        virtual bool isFinished() const { return m_released; }

    protected:
        bool m_released = false;
    };

    // Radiation pattern of one lobe as a function of the angle between the body
    // axis (nose) and the direction to the listener: 0 deg = straight ahead,
    // 180 deg = straight behind (down the exhaust).
    struct DirectivityPattern
    {
        static constexpr int kPoints = 9; // 0, 22.5, ..., 180 degrees
        std::array<float, kPoints> gainDb{};

        static DirectivityPattern omni() { return {}; }
        float gainAt(float cosAngle) const;
    };

    struct EmitterSpec
    {
        int lobeCount = 1;
        std::array<DirectivityPattern, kMaxLobes> lobes{};
        // How far back the emission history reaches; bounds the audible range to
        // roughly historySeconds * c.
        float historySeconds = 6.0f;
        // Body dimensions drive the sonic-boom N-wave (length 0 disables booms).
        float bodyLengthMeters = 0.0f;
        float bodyDiameterMeters = 0.0f;
        float reverbSend = 1.0f;
        float groundReflection = 1.0f;
        // Continuous sources that "already existed" before they were spawned
        // (e.g. an engine running when the scenario starts) fade their emission
        // in over this time instead of switching on. 0 keeps onsets intact.
        float fadeInSeconds = 0.0f;
    };

    // ------------------------------------------------------------------
    // Cross-thread control block. The game thread writes kinematics and the
    // release flag; the audio thread reads them.
    // ------------------------------------------------------------------

    struct Kinematics
    {
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 axis{0.0f, 0.0f, -1.0f};
        uint64_t stampSample = 0;
    };

    class EmitterControl
    {
    public:
        explicit EmitterControl(const Kinematics &initial) : m_kinematics(initial) {}

        void release() { m_released.store(true, std::memory_order_release); }
        bool isReleased() const { return m_released.load(std::memory_order_acquire); }
        bool isRetired() const { return m_retired.load(std::memory_order_acquire); }

    private:
        friend class AudioEngine;
        friend class AcousticEmitter;

        LatestValue<Kinematics> m_kinematics;
        std::atomic<bool> m_released{false};
        std::atomic<bool> m_retired{false};
    };

    // ------------------------------------------------------------------
    // Listener / medium, evaluated once per block by the engine.
    // ------------------------------------------------------------------

    // Distance -> -3 dB frequency of atmospheric absorption, sampled on a log grid.
    struct AbsorptionTable
    {
        static constexpr int kPoints = 48;
        static constexpr float kMinDistance = 1.0f;
        static constexpr float kMaxDistance = 40000.0f;
        std::array<float, kPoints> cutoffHz{};

        static AbsorptionTable compute(float temperatureK, float pressurePa, float relativeHumidityPercent);
        float cutoffAt(float distanceMeters) const;
    };

    struct ListenerFrame
    {
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 forward{0.0f, 0.0f, -1.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        glm::vec3 right{1.0f, 0.0f, 0.0f};
        std::array<glm::vec3, kEarCount> ears{};
    };

    struct MediumFrame
    {
        // Propagation speed in *engine* (wall-clock) time. When the simulation
        // runs faster or slower than real time this is the physical speed of
        // sound scaled with it (within limits), so delays, Doppler ratios and
        // Mach cones stay consistent with what is on screen.
        float speedOfSound = 340.3f;
        float physicalSpeedOfSound = 340.3f;
        // Simulation seconds per wall-clock second; converts the wall-clock
        // kinematics back to physical airspeed for the generators.
        float timeScale = 1.0f;
        float ambientPressurePa = 101325.0f;
        float groundLevel = 0.0f;
        bool groundPresent = true;
        const AbsorptionTable *absorption = nullptr;
    };

    struct BlockOutput
    {
        float *left = nullptr;
        float *right = nullptr;
        float *reverbSend = nullptr;
    };

    // ------------------------------------------------------------------
    // Emitter (audio-thread object; allocated and destroyed on the game thread)
    // ------------------------------------------------------------------

    class AcousticEmitter
    {
    public:
        AcousticEmitter(const EmitterSpec &spec,
                        std::shared_ptr<SoundSource> source,
                        std::shared_ptr<EmitterControl> control,
                        uint64_t seed);
        ~AcousticEmitter();

        AcousticEmitter(const AcousticEmitter &) = delete;
        AcousticEmitter &operator=(const AcousticEmitter &) = delete;

        // Audio thread: advance one block starting at absolute sample `blockStart`.
        void processBlock(uint64_t blockStart,
                          const ListenerFrame &listener,
                          const MediumFrame &medium,
                          const BlockOutput &output);

        // Audio thread: true once released, silent, and fully propagated.
        bool canRetire() const { return m_retireReady; }
        EmitterControl &control() { return *m_control; }

    private:
        struct Branch
        {
            double tauStart = 0.0;
            double tauEnd = 0.0;
            float rate = 1.0f;
            std::array<float, kMaxLobes> weightStart{};
            std::array<float, kMaxLobes> weightEnd{};
            float earGain = 1.0f;
            float mipLevel = 0.0f;
            OnePole absorptionA;
            OnePole absorptionB;
            OnePole shading;
            // Sonic-boom N-wave currently sweeping over this branch.
            int boomSample = -1;
            int boomLength = 0;
            int boomRise = 0;
            float boomPeak = 0.0f;
            bool active = false;
            bool dying = false;
        };

        struct Path
        {
            std::array<Branch, kMaxBranches> branches{};
            RandomWalk scintillation;
            double lastBoomTau = -1.0e18;
        };

        struct KinematicSample
        {
            glm::vec3 position{0.0f};
            glm::vec3 axis{0.0f, 0.0f, -1.0f};
        };

        void updateKinematics(uint64_t blockEnd);
        void renderEmission(uint64_t blockStart, const MediumFrame &medium);
        void writeHistory(const std::array<float *, kMaxLobes> &lobes, uint64_t blockStart);
        void propagate(uint64_t blockStart,
                       const ListenerFrame &listener,
                       const MediumFrame &medium,
                       const BlockOutput &output);

        KinematicSample sampleAt(double tau) const;
        glm::vec3 velocityAt(double tau) const;
        int solveRoots(const glm::vec3 &receiver, double t, float speedOfSound, double *roots) const;
        void configureBranch(Branch &branch,
                             double tau,
                             int ear,
                             int path,
                             const glm::vec3 &receiver,
                             const ListenerFrame &listener,
                             const MediumFrame &medium,
                             float scintillationGain,
                             bool born);
        void renderBranch(Branch &branch, float *destination, float *send, float sendGain);
        float readHistory(int lobe, float level, double tau) const;
        float readLevel(int lobe, int level, double tau) const;

        EmitterSpec m_spec;
        std::shared_ptr<SoundSource> m_source;
        std::shared_ptr<EmitterControl> m_control;
        Random m_random;

        // Emission history: [level][lobe] ring buffers. Level k holds a 2^k
        // decimated, half-band filtered copy of level 0.
        std::array<std::array<std::vector<float>, kMaxLobes>, kMipLevels> m_history{};
        std::array<uint64_t, kMipLevels> m_mask{};
        std::array<uint64_t, kMipLevels> m_written{}; // samples written per level
        uint64_t m_capacity = 0;

        // Kinematic history, one sample per block boundary.
        std::vector<KinematicSample> m_track;
        uint64_t m_trackCount = 0;
        uint64_t m_startSample = 0;
        bool m_started = false;
        float m_maxSpeed = 0.0f;

        // Smoothed kinematic state at the current block boundary.
        glm::vec3 m_position{0.0f};
        glm::vec3 m_velocity{0.0f};
        glm::vec3 m_axis{0.0f, 0.0f, -1.0f};

        std::array<std::array<Path, kPathCount>, kEarCount> m_paths{};

        std::array<std::array<float, kBlockSize>, kMaxLobes> m_scratch{};

        bool m_sourceFinished = false;
        uint64_t m_finishSample = 0;
        bool m_retireReady = false;
    };
}
