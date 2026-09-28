#pragma once

// Real-time audio engine: owns the output device and the audio thread, runs
// every AcousticEmitter through the physical propagation model, and masters the
// result the way a human ear / a well-mixed film would:
//
//   emitters (Pa at the ears) --+--> terrain echoes (outdoor reverb) --+
//                               +-------------------------------------+
//   acoustic local sources (Pa, e.g. wind) ---------------------------+
//                                                                      v
//        HDR exposure (adaptive "hearing" gain driven by measured SPL)
//                                                               |
//        temporary threshold shift (muffling + tinnitus after a blast)
//                                                               |
//   headset sources (digital) --> look-ahead limiter --> DC block --> device
//
// Threading: every public method except render() is for the game thread.
// render() is the audio thread (called by the device, or manually offline).

#include "Acoustics.h"

#include <glm/glm.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

struct ma_device;

namespace missilesim::audio
{
    // Non-spatial sound attached to the listener (cockpit cues, wind on the mic).
    class LocalSource
    {
    public:
        enum class Bus
        {
            // Physical pressure at the ears (Pa). Subject to exposure like the world.
            Acoustic,
            // Digital-level signal (full scale = 1.0) delivered straight to the
            // headset, independent of how loud the outside world is.
            Headset
        };

        virtual ~LocalSource() = default;
        virtual Bus bus() const = 0;
        // Add `frames` samples into left/right.
        virtual void render(float *left, float *right, int frames) = 0;
        virtual void release() { m_released = true; }
        virtual bool isFinished() const { return m_released; }

    protected:
        bool m_released = false;
    };

    struct EnvironmentSettings
    {
        float temperatureK = 288.15f;
        float pressurePa = 101325.0f;
        float relativeHumidityPercent = 50.0f;
        float groundLevel = 0.0f;
        float masterVolume = 1.0f;
        float reverbAmount = 1.0f;
        // False when there is no ground plane to reflect from.
        bool groundPresent = true;
        // Simulation speed relative to real time (slow motion < 1 < fast
        // forward). Kinematics are supplied in real (wall-clock) units.
        float timeScale = 1.0f;
        bool paused = false;
    };

    struct EngineStats
    {
        float cpuLoad = 0.0f;
        int activeEmitters = 0;
        float loudnessSpl = 0.0f;
        float exposureDb = 0.0f;
        float thresholdShift = 0.0f;
    };

    class AudioEngine
    {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine &) = delete;
        AudioEngine &operator=(const AudioEngine &) = delete;

        // Opens the default playback device. The engine also works without a
        // device through render() (offline rendering / tests).
        bool start();
        void stop();
        bool isRunning() const { return m_device != nullptr; }

        std::shared_ptr<EmitterControl> spawn(const EmitterSpec &spec,
                                              std::shared_ptr<SoundSource> source,
                                              const glm::vec3 &position,
                                              const glm::vec3 &velocity,
                                              const glm::vec3 &axis);
        void moveEmitter(EmitterControl &control,
                         const glm::vec3 &position,
                         const glm::vec3 &velocity,
                         const glm::vec3 &axis);

        // Returns false if the command queue is saturated.
        bool addLocalSource(std::shared_ptr<LocalSource> source);

        void setListener(const glm::vec3 &position,
                         const glm::vec3 &velocity,
                         const glm::vec3 &forward,
                         const glm::vec3 &up);
        void setEnvironment(const EnvironmentSettings &settings);
        const EnvironmentSettings &environment() const { return m_environment; }

        // Frees retired emitters / local sources. Call once per game frame.
        void collectGarbage();

        EngineStats stats() const;
        uint64_t clock() const { return m_clock.load(std::memory_order_acquire); }

        // Audio thread entry point: writes interleaved stereo float frames.
        void render(float *interleavedStereo, uint32_t frames);

    private:
        struct ListenerState
        {
            glm::vec3 position{0.0f};
            glm::vec3 velocity{0.0f};
            glm::vec3 forward{0.0f, 0.0f, -1.0f};
            glm::vec3 up{0.0f, 1.0f, 0.0f};
            uint64_t stampSample = 0;
        };

        struct EnvironmentPacket
        {
            EnvironmentSettings settings;
            AbsorptionTable absorption;
        };

        struct LocalHolder
        {
            std::shared_ptr<LocalSource> source;
        };

        struct Command
        {
            AcousticEmitter *emitter = nullptr;
            LocalHolder *local = nullptr;
        };

        class Reverb;
        class Master;

        void processBlock();
        void updateListener(uint64_t blockEnd);

        ma_device *m_device = nullptr;

        // Game-thread side.
        EnvironmentSettings m_environment;
        AbsorptionTable m_gameAbsorption;
        uint64_t m_nextSeed = 0x5EED5EEDull;

        // Cross-thread.
        std::atomic<uint64_t> m_clock{0};
        LatestValue<ListenerState> m_listenerMailbox;
        LatestValue<EnvironmentPacket> m_environmentMailbox;
        SpscQueue<Command, 512> m_commands;
        SpscQueue<Command, 1024> m_graveyard;
        std::atomic<float> m_statCpu{0.0f};
        std::atomic<int> m_statEmitters{0};
        std::atomic<float> m_statLoudness{0.0f};
        std::atomic<float> m_statExposure{0.0f};
        std::atomic<float> m_statShift{0.0f};

        // Audio-thread side.
        std::vector<AcousticEmitter *> m_emitters;
        std::vector<LocalHolder *> m_locals;
        std::vector<Command> m_pendingRetire;
        ListenerFrame m_listener;
        ListenerState m_listenerSmoothed;
        bool m_listenerInitialised = false;
        EnvironmentSettings m_audioSettings;
        AbsorptionTable m_audioAbsorption;
        std::unique_ptr<Reverb> m_reverb;
        std::unique_ptr<Master> m_master;

        std::array<float, kBlockSize> m_left{};
        std::array<float, kBlockSize> m_right{};
        std::array<float, kBlockSize> m_send{};
        std::array<float, kBlockSize> m_headsetLeft{};
        std::array<float, kBlockSize> m_headsetRight{};
        std::array<float, kBlockSize * 2> m_carry{};
        int m_carryFrames = 0;
        int m_carryRead = 0;
    };
}
