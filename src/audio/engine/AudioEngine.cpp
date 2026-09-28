#include "AudioEngine.h"

#include <miniaudio.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#if defined(_M_X64) || defined(__x86_64__) || defined(_M_IX86) || defined(__i386__)
#include <xmmintrin.h>
#define MISSILESIM_AUDIO_HAS_SSE 1
#endif

namespace missilesim::audio
{
    namespace
    {
        constexpr float kHalfHeadWidth = 0.0875f;
        constexpr size_t kMaxActiveEmitters = 512;

        // HDR exposure: the quietest scene is shown at kFloorSpl -> kFloorDbfs and
        // anything louder raises the window with a (1 - kExposureSlope) residual.
        // Half of every decibel above the floor survives, so a jet going from
        // idle to full power or receding into the distance keeps its dynamics
        // while a 140 dB blast still fits under the limiter.
        constexpr float kFloorSpl = 75.0f;
        constexpr float kFloorDbfs = -36.0f;
        constexpr float kExposureSlope = 0.5f;
        constexpr float kPascalToDbfsOffset = 93.9794f; // 20*log10(1 / 20e-6)
        constexpr float kExposureAttackSeconds = 0.08f;  // acoustic-reflex speed
        constexpr float kExposureReleaseSeconds = 5.0f; // hearing recovery
        constexpr float kLoudnessWindowSeconds = 0.35f;

        // Temporary threshold shift after a violent overpressure.
        constexpr float kShiftOnsetSpl = 150.0f;
        constexpr float kShiftFullSpl = 166.0f;
        constexpr float kShiftImpulseMarginDb = 20.0f; // crackle peaks sit ~13-15 dB over RMS
        constexpr float kShiftRecoverySeconds = 3.0f;
        constexpr float kTinnitusHz = 4150.0f;
        constexpr float kTinnitusLevel = 0.006f;

        constexpr int kLimiterLookahead = 48;
        constexpr float kLimiterCeiling = 0.89f; // -1 dBFS
        constexpr float kLimiterReleaseSeconds = 0.12f;

        // Propagation follows the simulation speed within this range. Beyond it
        // the emission histories would stop covering the audible distance (slow
        // motion) or everything would go supersonic (fast forward).
        constexpr float kMinTimeScale = 0.5f;
        constexpr float kMaxTimeScale = 4.0f;

        float speedOfSoundFor(float temperatureK)
        {
            return 20.0468f * std::sqrt(std::max(temperatureK, 150.0f));
        }

        void maDataCallback(ma_device *device, void *output, const void *input, ma_uint32 frameCount)
        {
            (void)input;
            auto *engine = static_cast<AudioEngine *>(device->pUserData);
            engine->render(static_cast<float *>(output), static_cast<uint32_t>(frameCount));
        }
    }

    // ------------------------------------------------------------------
    // Outdoor acoustic space: discrete terrain echoes feeding a dark,
    // Hadamard feedback-delay-network tail.
    // ------------------------------------------------------------------

    class AudioEngine::Reverb
    {
    public:
        Reverb()
        {
            m_inputHighPass.setHighPass(90.0f, 0.6f);
            m_inputLowPass.setCutoff(5200.0f);
            m_echo.assign(kEchoSize, 0.0f);

            constexpr std::array<float, kLines> lengthsMs = {43.1f, 53.7f, 67.9f, 79.3f, 97.1f, 113.9f, 131.3f, 149.9f};
            constexpr float rt60 = 2.4f;
            for (int i = 0; i < kLines; ++i)
            {
                const int length = static_cast<int>(lengthsMs[i] * 0.001f * kSampleRateF);
                m_lines[i].assign(static_cast<size_t>(length), 0.0f);
                m_gain[i] = std::pow(10.0f, -3.0f * static_cast<float>(length) / (rt60 * kSampleRateF));
                m_damping[i].setCutoff(2800.0f);
            }
        }

        void process(const float *input, float *left, float *right, int frames, float amount)
        {
            constexpr std::array<float, kTaps> tapSeconds = {0.087f, 0.143f, 0.221f, 0.337f, 0.482f, 0.690f};
            constexpr std::array<float, kTaps> tapGain = {0.30f, 0.26f, 0.21f, 0.16f, 0.11f, 0.07f};
            constexpr std::array<float, kTaps> tapPan = {-0.6f, 0.5f, -0.2f, 0.7f, -0.7f, 0.3f};
            constexpr float predelaySeconds = 0.045f;
            constexpr float lateGain = 0.55f;
            constexpr float inputGain = 0.3f;
            const float wet = amount * 0.35f;

            for (int n = 0; n < frames; ++n)
            {
                const float x = m_inputLowPass.lowpass(m_inputHighPass.process(input[n]));
                m_echo[m_echoWrite & kEchoMask] = x;

                float earlyL = 0.0f;
                float earlyR = 0.0f;
                for (int tap = 0; tap < kTaps; ++tap)
                {
                    const size_t delay = static_cast<size_t>(tapSeconds[tap] * kSampleRateF);
                    const float echo = m_echo[(m_echoWrite - delay) & kEchoMask] * tapGain[tap];
                    earlyL += echo * (0.5f - 0.5f * tapPan[tap]);
                    earlyR += echo * (0.5f + 0.5f * tapPan[tap]);
                }

                const size_t predelay = static_cast<size_t>(predelaySeconds * kSampleRateF);
                const float feed = (m_echo[(m_echoWrite - predelay) & kEchoMask] + 0.5f * (earlyL + earlyR)) * inputGain;
                ++m_echoWrite;

                std::array<float, kLines> out{};
                for (int i = 0; i < kLines; ++i)
                {
                    out[i] = m_lines[i][m_position[i]];
                }

                // Fast Walsh-Hadamard transform, normalised: a lossless, maximally
                // diffusing feedback matrix.
                std::array<float, kLines> mixed = out;
                for (int span = 1; span < kLines; span <<= 1)
                {
                    for (int i = 0; i < kLines; i += span << 1)
                    {
                        for (int j = i; j < i + span; ++j)
                        {
                            const float a = mixed[j];
                            const float b = mixed[j + span];
                            mixed[j] = a + b;
                            mixed[j + span] = a - b;
                        }
                    }
                }

                constexpr float normalise = 0.35355339f; // 1/sqrt(8)
                for (int i = 0; i < kLines; ++i)
                {
                    const float sign = (i & 1) ? -1.0f : 1.0f;
                    const float value = m_damping[i].lowpass(mixed[i] * normalise) * m_gain[i] + feed * sign;
                    m_lines[i][m_position[i]] = value;
                    if (++m_position[i] >= m_lines[i].size())
                    {
                        m_position[i] = 0;
                    }
                }

                const float lateL = out[0] - out[2] + out[4] - out[6];
                const float lateR = out[1] - out[3] + out[5] - out[7];
                left[n] += wet * (earlyL + lateGain * lateL);
                right[n] += wet * (earlyR + lateGain * lateR);
            }
        }

    private:
        static constexpr int kLines = 8;
        static constexpr int kTaps = 6;
        static constexpr size_t kEchoSize = 65536;
        static constexpr size_t kEchoMask = kEchoSize - 1;

        Biquad m_inputHighPass;
        OnePole m_inputLowPass;
        std::vector<float> m_echo;
        size_t m_echoWrite = 0;
        std::array<std::vector<float>, kLines> m_lines;
        std::array<size_t, kLines> m_position{};
        std::array<float, kLines> m_gain{};
        std::array<OnePole, kLines> m_damping{};
    };

    // ------------------------------------------------------------------
    // Master section: exposure, threshold shift, limiter.
    // ------------------------------------------------------------------

    class AudioEngine::Master
    {
    public:
        Master()
        {
            m_weightHighPass.setHighPass(60.0f, 0.5f);
            m_weightShelf.setHighShelf(1500.0f, 4.0f);
            m_dcLeft.setCutoff(12.0f);
            m_dcRight.setCutoff(12.0f);
            m_exposureDb = exposureTarget(kFloorSpl);
            m_currentGain = dbToGain(m_exposureDb);
        }

        void process(float *left,
                     float *right,
                     const float *headsetLeft,
                     const float *headsetRight,
                     int frames,
                     float volume,
                     bool paused)
        {
            // --- Metering (world signal, physical units) ---
            const float loudnessCoefficient = smoothingCoefficient(kLoudnessWindowSeconds);
            const float peakRelease = smoothingCoefficient(0.05f);
            float blockPeak = 0.0f;
            for (int n = 0; n < frames; ++n)
            {
                const float mono = 0.5f * (left[n] + right[n]);
                const float weighted = m_weightShelf.process(m_weightHighPass.process(mono));
                m_meanSquare += (weighted * weighted - m_meanSquare) * loudnessCoefficient;
                const float magnitude = std::max(std::abs(left[n]), std::abs(right[n]));
                m_peak = (magnitude > m_peak) ? magnitude : m_peak + (magnitude - m_peak) * peakRelease;
                blockPeak = std::max(blockPeak, magnitude);
            }
            m_loudnessSpl = 10.0f * std::log10(std::max(m_meanSquare, 1.0e-20f)) + kPascalToDbfsOffset;
            const float peakSpl = gainToDb(std::max(m_peak, 1.0e-10f)) + kPascalToDbfsOffset;

            // --- Exposure ---
            const float windowTop = std::max({m_loudnessSpl, peakSpl - 10.0f, kFloorSpl});
            const float target = exposureTarget(windowTop);
            const float blockSamples = static_cast<float>(frames);
            const float coefficient = (target < m_exposureDb)
                                          ? smoothingCoefficient(kExposureAttackSeconds, blockSamples)
                                          : smoothingCoefficient(kExposureReleaseSeconds, blockSamples);
            m_exposureDb += (target - m_exposureDb) * coefficient;

            // --- Temporary threshold shift ---
            // Only impulsive overpressure stuns the ear: a blast front that
            // towers over the running loudness. Steady roar, however loud, is
            // handled by exposure alone (a chase camera behind a rocket would
            // otherwise stay muffled for the whole flight).
            const float blockPeakSpl = gainToDb(std::max(blockPeak, 1.0e-10f)) + kPascalToDbfsOffset;
            const float onsetSpl = std::max(kShiftOnsetSpl, m_loudnessSpl + kShiftImpulseMarginDb);
            const float shock = clampf((blockPeakSpl - onsetSpl) / (kShiftFullSpl - kShiftOnsetSpl), 0.0f, 1.0f);
            m_shift = std::max(m_shift * std::exp(-blockSamples / (kShiftRecoverySeconds * kSampleRateF)), shock);
            const float muffleHz = 20000.0f * std::pow(900.0f / 20000.0f, m_shift);
            m_muffleLeft.setCutoff(muffleHz);
            m_muffleRight.setCutoff(muffleHz);
            const float tinnitus = kTinnitusLevel * m_shift * m_shift;

            // --- Pause fade ---
            const float fadeTarget = paused ? 0.0f : 1.0f;
            const float fadeCoefficient = smoothingCoefficient(0.12f);

            const float gainStart = m_currentGain;
            const float gainEnd = dbToGain(m_exposureDb);
            m_currentGain = gainEnd;
            const float inverseFrames = 1.0f / static_cast<float>(frames);
            const float limiterRelease = smoothingCoefficient(kLimiterReleaseSeconds);
            const float limiterAttack = smoothingCoefficient(static_cast<float>(kLimiterLookahead) / (3.0f * kSampleRateF));
            const float tinnitusStep = kTwoPi * kTinnitusHz / kSampleRateF;

            for (int n = 0; n < frames; ++n)
            {
                const float gain = lerpf(gainStart, gainEnd, static_cast<float>(n) * inverseFrames);
                m_fade += (fadeTarget - m_fade) * fadeCoefficient;

                float l = left[n] * gain;
                float r = right[n] * gain;
                if (m_shift > 1.0e-4f)
                {
                    l = m_muffleLeft.lowpass(l);
                    r = m_muffleRight.lowpass(r);
                    m_tinnitusPhase += tinnitusStep;
                    if (m_tinnitusPhase > kTwoPi)
                    {
                        m_tinnitusPhase -= kTwoPi;
                    }
                    const float tone = tinnitus * std::sin(m_tinnitusPhase);
                    l += tone;
                    r += tone;
                }
                else
                {
                    m_muffleLeft.reset(l);
                    m_muffleRight.reset(r);
                }

                l = (l + headsetLeft[n]) * m_fade;
                r = (r + headsetRight[n]) * m_fade;

                // Look-ahead peak limiter.
                const float peak = std::max(std::abs(l), std::abs(r));
                const float wanted = (peak > kLimiterCeiling) ? kLimiterCeiling / peak : 1.0f;
                m_gainHistory[m_limiterWrite] = wanted;
                m_delayLeft[m_limiterWrite] = l;
                m_delayRight[m_limiterWrite] = r;
                m_limiterWrite = (m_limiterWrite + 1) % kLimiterLookahead;

                float windowMinimum = 1.0f;
                for (float g : m_gainHistory)
                {
                    windowMinimum = std::min(windowMinimum, g);
                }
                const float follow = (windowMinimum < m_limiterGain) ? limiterAttack : limiterRelease;
                m_limiterGain += (windowMinimum - m_limiterGain) * follow;

                float outL = m_delayLeft[m_limiterWrite] * m_limiterGain;
                float outR = m_delayRight[m_limiterWrite] * m_limiterGain;
                outL = clampf(outL, -kLimiterCeiling, kLimiterCeiling);
                outR = clampf(outR, -kLimiterCeiling, kLimiterCeiling);

                left[n] = m_dcLeft.highpass(outL) * volume;
                right[n] = m_dcRight.highpass(outR) * volume;
            }
        }

        float loudnessSpl() const { return m_loudnessSpl; }
        float exposureDb() const { return m_exposureDb; }
        float thresholdShift() const { return m_shift; }

    private:
        static float exposureTarget(float windowTopSpl)
        {
            const float floorGain = kFloorDbfs - kFloorSpl + kPascalToDbfsOffset;
            return floorGain - kExposureSlope * (windowTopSpl - kFloorSpl);
        }

        Biquad m_weightHighPass;
        Biquad m_weightShelf;
        float m_meanSquare = 0.0f;
        float m_peak = 0.0f;
        float m_loudnessSpl = 0.0f;
        float m_exposureDb = 0.0f;
        float m_currentGain = 1.0f;

        float m_shift = 0.0f;
        OnePole m_muffleLeft, m_muffleRight;
        float m_tinnitusPhase = 0.0f;
        float m_fade = 1.0f;

        std::array<float, kLimiterLookahead> m_gainHistory{};
        std::array<float, kLimiterLookahead> m_delayLeft{};
        std::array<float, kLimiterLookahead> m_delayRight{};
        int m_limiterWrite = 0;
        float m_limiterGain = 1.0f;

        OnePole m_dcLeft, m_dcRight;
    };

    // ------------------------------------------------------------------
    // Engine
    // ------------------------------------------------------------------

    AudioEngine::AudioEngine()
        : m_reverb(std::make_unique<Reverb>()),
          m_master(std::make_unique<Master>())
    {
        m_emitters.reserve(kMaxActiveEmitters);
        m_locals.reserve(64);
        m_pendingRetire.reserve(1024);
        m_gameAbsorption = AbsorptionTable::compute(m_environment.temperatureK,
                                                    m_environment.pressurePa,
                                                    m_environment.relativeHumidityPercent);
        m_audioAbsorption = m_gameAbsorption;
        setEnvironment(m_environment);
    }

    AudioEngine::~AudioEngine()
    {
        stop();

        Command command;
        while (m_commands.pop(command))
        {
            delete command.emitter;
            delete command.local;
        }
        while (m_graveyard.pop(command))
        {
            delete command.emitter;
            delete command.local;
        }
        for (const Command &pending : m_pendingRetire)
        {
            delete pending.emitter;
            delete pending.local;
        }
        for (AcousticEmitter *emitter : m_emitters)
        {
            delete emitter;
        }
        for (LocalHolder *local : m_locals)
        {
            delete local;
        }
    }

    bool AudioEngine::start()
    {
        if (m_device)
        {
            return true;
        }

        ma_device_config config = ma_device_config_init(ma_device_type_playback);
        config.playback.format = ma_format_f32;
        config.playback.channels = 2;
        config.sampleRate = kSampleRate;
        config.periodSizeInFrames = 256;
        config.performanceProfile = ma_performance_profile_low_latency;
        config.dataCallback = maDataCallback;
        config.pUserData = this;

        auto *device = new ma_device;
        if (ma_device_init(nullptr, &config, device) != MA_SUCCESS)
        {
            std::cerr << "[audio] failed to open playback device; running silent" << std::endl;
            delete device;
            return false;
        }
        if (ma_device_start(device) != MA_SUCCESS)
        {
            std::cerr << "[audio] failed to start playback device; running silent" << std::endl;
            ma_device_uninit(device);
            delete device;
            return false;
        }
        m_device = device;
        return true;
    }

    void AudioEngine::stop()
    {
        if (!m_device)
        {
            return;
        }
        ma_device_uninit(m_device);
        delete m_device;
        m_device = nullptr;
    }

    std::shared_ptr<EmitterControl> AudioEngine::spawn(const EmitterSpec &spec,
                                                       std::shared_ptr<SoundSource> source,
                                                       const glm::vec3 &position,
                                                       const glm::vec3 &velocity,
                                                       const glm::vec3 &axis)
    {
        if (!source)
        {
            return nullptr;
        }

        Kinematics initial;
        initial.position = position;
        initial.velocity = velocity;
        initial.axis = axis;
        initial.stampSample = clock();

        auto control = std::make_shared<EmitterControl>(initial);
        m_nextSeed = m_nextSeed * 6364136223846793005ull + 1442695040888963407ull;
        auto *emitter = new AcousticEmitter(spec, std::move(source), control, m_nextSeed);

        Command command;
        command.emitter = emitter;
        if (!m_commands.push(command))
        {
            delete emitter;
            return nullptr;
        }
        return control;
    }

    void AudioEngine::moveEmitter(EmitterControl &control,
                                  const glm::vec3 &position,
                                  const glm::vec3 &velocity,
                                  const glm::vec3 &axis)
    {
        Kinematics kinematics;
        kinematics.position = position;
        kinematics.velocity = velocity;
        kinematics.axis = axis;
        kinematics.stampSample = clock();
        control.m_kinematics.write(kinematics);
    }

    bool AudioEngine::addLocalSource(std::shared_ptr<LocalSource> source)
    {
        if (!source)
        {
            return false;
        }
        auto *holder = new LocalHolder{std::move(source)};
        Command command;
        command.local = holder;
        if (!m_commands.push(command))
        {
            delete holder;
            return false;
        }
        return true;
    }

    void AudioEngine::setListener(const glm::vec3 &position,
                                  const glm::vec3 &velocity,
                                  const glm::vec3 &forward,
                                  const glm::vec3 &up)
    {
        ListenerState state;
        state.position = position;
        state.velocity = velocity;
        state.forward = forward;
        state.up = up;
        state.stampSample = clock();
        m_listenerMailbox.write(state);
    }

    void AudioEngine::setEnvironment(const EnvironmentSettings &settings)
    {
        const bool atmosphereChanged = std::abs(settings.temperatureK - m_environment.temperatureK) > 0.25f ||
                                       std::abs(settings.pressurePa - m_environment.pressurePa) > 50.0f ||
                                       std::abs(settings.relativeHumidityPercent - m_environment.relativeHumidityPercent) > 0.5f;
        m_environment = settings;

        EnvironmentPacket packet;
        packet.settings = settings;
        if (atmosphereChanged)
        {
            m_gameAbsorption = AbsorptionTable::compute(settings.temperatureK, settings.pressurePa, settings.relativeHumidityPercent);
        }
        packet.absorption = m_gameAbsorption;
        m_environmentMailbox.write(packet);
    }

    void AudioEngine::collectGarbage()
    {
        Command command;
        while (m_graveyard.pop(command))
        {
            if (command.emitter)
            {
                command.emitter->control().m_retired.store(true, std::memory_order_release);
            }
            delete command.emitter;
            delete command.local;
        }
    }

    EngineStats AudioEngine::stats() const
    {
        EngineStats result;
        result.cpuLoad = m_statCpu.load(std::memory_order_relaxed);
        result.activeEmitters = m_statEmitters.load(std::memory_order_relaxed);
        result.loudnessSpl = m_statLoudness.load(std::memory_order_relaxed);
        result.exposureDb = m_statExposure.load(std::memory_order_relaxed);
        result.thresholdShift = m_statShift.load(std::memory_order_relaxed);
        return result;
    }

    void AudioEngine::render(float *interleavedStereo, uint32_t frames)
    {
#ifdef MISSILESIM_AUDIO_HAS_SSE
        // Flush denormals to zero: decaying filter tails must never stall the CPU.
        const unsigned int savedCsr = _mm_getcsr();
        _mm_setcsr(savedCsr | 0x8040u);
#endif
        const auto started = std::chrono::steady_clock::now();

        uint32_t written = 0;
        while (written < frames)
        {
            if (m_carryRead >= m_carryFrames)
            {
                processBlock();
                m_carryFrames = kBlockSize;
                m_carryRead = 0;
            }
            const uint32_t available = static_cast<uint32_t>(m_carryFrames - m_carryRead);
            const uint32_t count = std::min(available, frames - written);
            std::copy_n(m_carry.data() + static_cast<size_t>(m_carryRead) * 2,
                        static_cast<size_t>(count) * 2,
                        interleavedStereo + static_cast<size_t>(written) * 2);
            m_carryRead += static_cast<int>(count);
            written += count;
        }

        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        const double budget = static_cast<double>(frames) / kSampleRate;
        const float load = static_cast<float>(elapsed / std::max(budget, 1.0e-6));
        const float previous = m_statCpu.load(std::memory_order_relaxed);
        m_statCpu.store(previous + (load - previous) * 0.1f, std::memory_order_relaxed);

#ifdef MISSILESIM_AUDIO_HAS_SSE
        _mm_setcsr(savedCsr);
#endif
    }

    void AudioEngine::updateListener(uint64_t blockEnd)
    {
        const ListenerState &target = m_listenerMailbox.read();
        const float blockSeconds = static_cast<float>(kBlockSize) / kSampleRateF;
        const double ahead = (static_cast<double>(blockEnd) - static_cast<double>(target.stampSample)) / kSampleRate;
        const glm::vec3 goal = target.position + target.velocity * static_cast<float>(std::clamp(ahead, 0.0, 0.25));

        if (!m_listenerInitialised)
        {
            m_listenerSmoothed = target;
            m_listenerSmoothed.position = goal;
            m_listenerInitialised = true;
        }
        else
        {
            glm::vec3 predicted = m_listenerSmoothed.position + m_listenerSmoothed.velocity * blockSeconds;
            const glm::vec3 error = goal - predicted;
            if (glm::length(error) > 20.0f + glm::length(target.velocity) * 0.15f)
            {
                predicted = goal;
            }
            else
            {
                predicted += error * smoothingCoefficient(0.045f, static_cast<float>(kBlockSize));
            }
            m_listenerSmoothed.position = predicted;
            m_listenerSmoothed.velocity += (target.velocity - m_listenerSmoothed.velocity) *
                                           smoothingCoefficient(0.02f, static_cast<float>(kBlockSize));

            const float turn = smoothingCoefficient(0.015f, static_cast<float>(kBlockSize));
            m_listenerSmoothed.forward = glm::mix(m_listenerSmoothed.forward, target.forward, turn);
            m_listenerSmoothed.up = glm::mix(m_listenerSmoothed.up, target.up, turn);
        }

        glm::vec3 forward = m_listenerSmoothed.forward;
        forward = (glm::length(forward) > 1.0e-4f) ? glm::normalize(forward) : glm::vec3(0.0f, 0.0f, -1.0f);
        glm::vec3 right = glm::cross(forward, m_listenerSmoothed.up);
        right = (glm::length(right) > 1.0e-4f) ? glm::normalize(right) : glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 up = glm::cross(right, forward);

        m_listener.position = m_listenerSmoothed.position;
        m_listener.velocity = m_listenerSmoothed.velocity;
        m_listener.forward = forward;
        m_listener.up = up;
        m_listener.right = right;
        m_listener.ears[0] = m_listener.position - right * kHalfHeadWidth;
        m_listener.ears[1] = m_listener.position + right * kHalfHeadWidth;
    }

    void AudioEngine::processBlock()
    {
        const uint64_t blockStart = m_clock.load(std::memory_order_relaxed);
        const uint64_t blockEnd = blockStart + kBlockSize;

        // Retry retirements that did not fit in the graveyard earlier.
        while (!m_pendingRetire.empty() && m_graveyard.push(m_pendingRetire.back()))
        {
            m_pendingRetire.pop_back();
        }

        Command command;
        while (m_emitters.size() < kMaxActiveEmitters && m_commands.pop(command))
        {
            if (command.emitter)
            {
                m_emitters.push_back(command.emitter);
            }
            if (command.local)
            {
                if (m_locals.size() < m_locals.capacity())
                {
                    m_locals.push_back(command.local);
                }
                else
                {
                    m_pendingRetire.push_back({nullptr, command.local});
                }
            }
        }

        bool environmentChanged = false;
        const EnvironmentPacket &packet = m_environmentMailbox.read(&environmentChanged);
        if (environmentChanged)
        {
            m_audioSettings = packet.settings;
            m_audioAbsorption = packet.absorption;
        }

        updateListener(blockEnd);

        MediumFrame medium;
        medium.timeScale = std::max(m_audioSettings.timeScale, 1.0e-3f);
        medium.physicalSpeedOfSound = speedOfSoundFor(m_audioSettings.temperatureK);
        medium.speedOfSound = medium.physicalSpeedOfSound * clampf(medium.timeScale, kMinTimeScale, kMaxTimeScale);
        medium.ambientPressurePa = m_audioSettings.pressurePa;
        medium.groundLevel = m_audioSettings.groundLevel;
        medium.groundPresent = m_audioSettings.groundPresent;
        medium.absorption = &m_audioAbsorption;

        m_left.fill(0.0f);
        m_right.fill(0.0f);
        m_send.fill(0.0f);
        m_headsetLeft.fill(0.0f);
        m_headsetRight.fill(0.0f);

        const BlockOutput output{m_left.data(), m_right.data(), m_send.data()};
        for (AcousticEmitter *emitter : m_emitters)
        {
            emitter->processBlock(blockStart, m_listener, medium, output);
        }

        m_reverb->process(m_send.data(), m_left.data(), m_right.data(), kBlockSize, m_audioSettings.reverbAmount);

        for (LocalHolder *local : m_locals)
        {
            LocalSource &source = *local->source;
            if (source.bus() == LocalSource::Bus::Headset)
            {
                source.render(m_headsetLeft.data(), m_headsetRight.data(), kBlockSize);
            }
            else
            {
                source.render(m_left.data(), m_right.data(), kBlockSize);
            }
        }

        m_master->process(m_left.data(),
                          m_right.data(),
                          m_headsetLeft.data(),
                          m_headsetRight.data(),
                          kBlockSize,
                          m_audioSettings.masterVolume,
                          m_audioSettings.paused);

        for (int n = 0; n < kBlockSize; ++n)
        {
            m_carry[static_cast<size_t>(n) * 2] = m_left[n];
            m_carry[static_cast<size_t>(n) * 2 + 1] = m_right[n];
        }

        // Retire finished work; memory is released on the game thread.
        for (size_t i = 0; i < m_emitters.size();)
        {
            if (m_emitters[i]->canRetire())
            {
                const Command retired{m_emitters[i], nullptr};
                if (!m_graveyard.push(retired))
                {
                    m_pendingRetire.push_back(retired);
                }
                m_emitters[i] = m_emitters.back();
                m_emitters.pop_back();
            }
            else
            {
                ++i;
            }
        }
        for (size_t i = 0; i < m_locals.size();)
        {
            if (m_locals[i]->source->isFinished())
            {
                const Command retired{nullptr, m_locals[i]};
                if (!m_graveyard.push(retired))
                {
                    m_pendingRetire.push_back(retired);
                }
                m_locals[i] = m_locals.back();
                m_locals.pop_back();
            }
            else
            {
                ++i;
            }
        }

        m_statEmitters.store(static_cast<int>(m_emitters.size()), std::memory_order_relaxed);
        m_statLoudness.store(m_master->loudnessSpl(), std::memory_order_relaxed);
        m_statExposure.store(m_master->exposureDb(), std::memory_order_relaxed);
        m_statShift.store(m_master->thresholdShift(), std::memory_order_relaxed);
        m_clock.store(blockEnd, std::memory_order_release);
    }
}
