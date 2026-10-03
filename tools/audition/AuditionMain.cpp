// Offline audition renderer for the audio engine.
//
// Renders scripted scenarios (launches, flybys, explosions at several ranges,
// flares, a full engagement) through the exact real-time engine and writes one
// WAV per scenario, so the sound design can be judged by ear and analysed
// without launching the simulator.
//
// Usage: AudioAudition [output-directory] [scenario-name-filter] [reverb-amount] [trace]

#include "audio/engine/AudioEngine.h"
#include "audio/synth/Cues.h"
#include "audio/synth/Voices.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace missilesim::audio;
using namespace missilesim::audio::synth;

namespace
{
    constexpr int kFrameSamples = 480; // 100 Hz "game" update rate
    constexpr float kFrameSeconds = static_cast<float>(kFrameSamples) / kSampleRateF;
    constexpr float kGravity = 9.81f;

    // Command-line override for A/B-ing the outdoor reverb (1 = default).
    float g_reverbAmount = 1.0f;
    // "trace" as the 4th argument prints the engine meters twice a second.
    bool g_trace = false;

    glm::vec3 safeNormalize(const glm::vec3 &v, const glm::vec3 &fallback = glm::vec3(0.0f, 0.0f, -1.0f))
    {
        const float length = glm::length(v);
        return length > 1.0e-5f ? v / length : fallback;
    }

    bool writeWav(const std::filesystem::path &path, const std::vector<float> &interleaved)
    {
        std::ofstream file(path, std::ios::binary);
        if (!file)
        {
            return false;
        }
        const uint32_t dataBytes = static_cast<uint32_t>(interleaved.size() * sizeof(int16_t));
        auto u32 = [&](uint32_t v) { file.write(reinterpret_cast<const char *>(&v), 4); };
        auto u16 = [&](uint16_t v) { file.write(reinterpret_cast<const char *>(&v), 2); };
        file.write("RIFF", 4);
        u32(36 + dataBytes);
        file.write("WAVEfmt ", 8);
        u32(16);
        u16(1);
        u16(2);
        u32(kSampleRate);
        u32(kSampleRate * 4);
        u16(4);
        u16(16);
        file.write("data", 4);
        u32(dataBytes);

        Random dither(12345);
        for (float sample : interleaved)
        {
            // TPDF dither to 16 bit.
            const float noise = (dither.unit() - dither.unit()) / 32768.0f;
            const float clamped = clampf(sample + noise, -1.0f, 1.0f);
            const int16_t value = static_cast<int16_t>(std::lround(clamped * 32767.0f));
            file.write(reinterpret_cast<const char *>(&value), 2);
        }
        return true;
    }

    // A scenario drives the engine at a fixed game rate and records the output.
    class Scene
    {
    public:
        explicit Scene(float groundLevel = 0.0f)
        {
            EnvironmentSettings settings;
            settings.groundLevel = groundLevel;
            settings.reverbAmount = g_reverbAmount;
            m_engine.setEnvironment(settings);
        }

        AudioEngine &engine() { return m_engine; }
        float time() const { return m_time; }

        void listener(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &velocity = glm::vec3(0.0f))
        {
            m_engine.setListener(position, velocity, forward, glm::vec3(0.0f, 1.0f, 0.0f));
        }

        // Advances the scenario without recording, so steady-state sources
        // (a jet that has "always" been flying) are already audible when the
        // recording starts.
        void preroll(float seconds, const std::function<void(float time, float dt)> &update)
        {
            run(seconds, update, false);
        }

        void run(float seconds, const std::function<void(float time, float dt)> &update, bool record = true)
        {
            std::vector<float> buffer(static_cast<size_t>(kFrameSamples) * 2);
            const int frames = static_cast<int>(std::ceil(seconds / kFrameSeconds));
            const auto started = std::chrono::steady_clock::now();
            for (int f = 0; f < frames; ++f)
            {
                update(m_time, kFrameSeconds);
                m_engine.render(buffer.data(), kFrameSamples);
                m_engine.collectGarbage();
                m_time += kFrameSeconds;
                if (!record)
                {
                    continue;
                }
                m_output.insert(m_output.end(), buffer.begin(), buffer.end());

                const EngineStats stats = m_engine.stats();
                if (g_trace && (f % 50) == 0)
                {
                    std::printf("    t=%5.1f  SPL %5.1f dB  exposure %6.1f dB  shift %.2f  emitters %d\n",
                                m_time, stats.loudnessSpl, stats.exposureDb, stats.thresholdShift, stats.activeEmitters);
                }
                m_maxLoudness = std::max(m_maxLoudness, stats.loudnessSpl);
                m_maxShift = std::max(m_maxShift, stats.thresholdShift);
                m_maxEmitters = std::max(m_maxEmitters, stats.activeEmitters);
            }
            m_renderSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        }

        bool finish(const std::filesystem::path &path) const
        {
            float peak = 0.0f;
            double sumSquares = 0.0;
            bool finite = true;
            for (float s : m_output)
            {
                finite = finite && std::isfinite(s);
                peak = std::max(peak, std::abs(s));
                sumSquares += static_cast<double>(s) * s;
            }
            const double rms = std::sqrt(sumSquares / std::max<size_t>(m_output.size(), 1));
            const double audioSeconds = static_cast<double>(m_output.size() / 2) / kSampleRate;
            std::printf("  %-34s peak %6.1f dBFS  rms %6.1f dBFS  maxSPL %5.1f dB  shift %.2f  emitters %2d  realtime x%.1f%s\n",
                        path.filename().string().c_str(),
                        gainToDb(peak),
                        gainToDb(static_cast<float>(rms)),
                        m_maxLoudness,
                        m_maxShift,
                        m_maxEmitters,
                        audioSeconds / std::max(m_renderSeconds, 1.0e-9),
                        finite ? "" : "  ** NON-FINITE SAMPLES **");
            return finite && writeWav(path, m_output);
        }

    private:
        AudioEngine m_engine;
        std::vector<float> m_output;
        float m_time = 0.0f;
        double m_renderSeconds = 0.0;
        float m_maxLoudness = 0.0f;
        float m_maxShift = 0.0f;
        int m_maxEmitters = 0;
    };

    // Missile on a scripted flight: kinematics + motor parameters.
    struct ScriptedMissile
    {
        std::shared_ptr<RocketMotorVoice> voice;
        std::shared_ptr<EmitterControl> control;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        float thrust = 0.0f;

        void spawn(AudioEngine &engine, const glm::vec3 &p, const glm::vec3 &v, uint64_t seed)
        {
            position = p;
            velocity = v;
            voice = std::make_shared<RocketMotorVoice>(seed);
            control = engine.spawn(RocketMotorVoice::spec(3.0f, 0.127f), voice, p, v, safeNormalize(v, glm::vec3(0, 1, 0)));
        }

        void publish(AudioEngine &engine) const
        {
            RocketMotorParams params;
            params.thrustNewtons = thrust;
            params.exhaustVelocity = 2200.0f;
            params.nozzleDiameter = 0.1f;
            params.bodyDiameter = 0.127f;
            params.airDensity = 1.2f;
            voice->setParams(params);
            engine.moveEmitter(*control, position, velocity, safeNormalize(velocity, glm::vec3(0, 1, 0)));
        }
    };

    struct ScriptedJet
    {
        std::shared_ptr<TurbofanVoice> voice;
        std::shared_ptr<EmitterControl> control;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        float throttle = 0.8f;

        void spawn(AudioEngine &engine, const glm::vec3 &p, const glm::vec3 &v, float initialThrottle, uint64_t seed)
        {
            position = p;
            velocity = v;
            throttle = initialThrottle;
            voice = std::make_shared<TurbofanVoice>(seed, initialThrottle);
            control = engine.spawn(TurbofanVoice::spec(), voice, p, v, safeNormalize(v));
        }

        void publish(AudioEngine &engine) const
        {
            TurbofanParams params;
            params.throttle = throttle;
            params.airDensity = 1.15f;
            voice->setParams(params);
            engine.moveEmitter(*control, position, velocity, safeNormalize(velocity));
        }
    };

    struct ScriptedFlare
    {
        std::shared_ptr<FlareVoice> voice;
        std::shared_ptr<EmitterControl> control;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        float age = 0.0f;
        bool alive = true;

        void update(AudioEngine &engine, float dt)
        {
            if (!alive)
            {
                return;
            }
            age += dt;
            // Draggy pellet: decelerates hard and drops.
            const float speed = glm::length(velocity);
            velocity += (-velocity * speed * 0.004f + glm::vec3(0.0f, -kGravity, 0.0f)) * dt;
            position += velocity * dt;
            FlareParams params;
            params.heat = std::exp(-age / 3.0f);
            voice->setParams(params);
            engine.moveEmitter(*control, position, velocity, safeNormalize(velocity));
            if (age > 4.0f)
            {
                control->release();
                alive = false;
            }
        }
    };

    void spawnExplosion(AudioEngine &engine, const glm::vec3 &position, float chargeKg, uint64_t seed)
    {
        engine.spawn(ExplosionVoice::spec(), std::make_shared<ExplosionVoice>(seed, chargeKg), position, glm::vec3(0.0f), glm::vec3(0, 1, 0));
    }

    // Integrates a boost-sustain motor with simple drag; the nose follows `aim`.
    void flyMissile(ScriptedMissile &missile, const glm::vec3 &aim, float mass, float dt)
    {
        const glm::vec3 direction = safeNormalize(glm::length(missile.velocity) > 5.0f ? glm::mix(safeNormalize(missile.velocity), aim, 0.25f) : aim);
        const float speed = glm::length(missile.velocity);
        const glm::vec3 drag = -missile.velocity * speed * (0.5f * 1.2f * 0.3f * 0.0127f) / mass;
        const glm::vec3 acceleration = direction * (missile.thrust / mass) + drag + glm::vec3(0.0f, -kGravity, 0.0f);
        missile.velocity += acceleration * dt;
        // Keep the flight path pointing along the commanded aim (idealised autopilot).
        const float newSpeed = glm::length(missile.velocity);
        missile.velocity = glm::mix(missile.velocity, aim * newSpeed, std::min(dt * 3.0f, 1.0f));
        missile.position += missile.velocity * dt;
    }

    // Calibrated white noise (1 Pa RMS at 1 m), for engine diagnostics.
    class WhiteNoiseVoice : public SoundSource
    {
    public:
        void render(const SourceContext &, float *const *lobes, int, int frames) override
        {
            for (int i = 0; i < frames; ++i)
            {
                lobes[0][i] += m_random.bipolar() * 1.7320508f;
            }
        }

    private:
        Random m_random{99};
    };

    // Stationary noise 100 m away, 30 m up: the ground reflection must carve
    // notches at odd multiples of c / (2 * path difference) = 175 Hz.
    void diagnosticGroundComb(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        EmitterSpec spec;
        spec.reverbSend = 0.0f;
        engine.spawn(spec, std::make_shared<WhiteNoiseVoice>(), glm::vec3(0.0f, 30.0f, -100.0f), glm::vec3(0.0f), glm::vec3(0, 0, 1));
        scene.run(4.0f, [](float, float) {});
    }

    // ------------------------------------------------------------------
    // Scenarios
    // ------------------------------------------------------------------

    void missileLaunchSide(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(160.0f, 1.7f, 0.0f), glm::vec3(-1.0f, 0.1f, 0.0f));

        ScriptedMissile missile;
        bool ejected = false;
        bool spawned = false;
        const glm::vec3 pad(0.0f, 1.0f, 0.0f);
        scene.run(18.0f, [&](float t, float dt)
        {
            if (!ejected && t >= 0.5f)
            {
                ejected = true;
                engine.spawn(LaunchEjectVoice::spec(), std::make_shared<LaunchEjectVoice>(7), pad, glm::vec3(0.0f), glm::vec3(0, 1, 0));
                missile.spawn(engine, pad, glm::vec3(0.0f, 28.0f, 0.0f), 11);
                spawned = true;
            }
            if (!spawned)
            {
                return;
            }
            const float flight = t - 0.5f;
            if (flight < 0.45f)
            {
                missile.thrust = 0.0f; // eject coast
            }
            else if (flight < 2.9f)
            {
                missile.thrust = 40000.0f * std::min(0.35f + (flight - 0.45f) / 0.25f * 0.65f, 1.0f);
            }
            else if (flight < 8.0f)
            {
                missile.thrust = 10000.0f;
            }
            else
            {
                missile.thrust = 0.0f;
            }
            const float pitch = glm::radians(glm::mix(88.0f, 25.0f, glm::clamp((flight - 0.5f) / 1.8f, 0.0f, 1.0f)));
            const glm::vec3 aim(0.0f, std::sin(pitch), std::cos(pitch));
            flyMissile(missile, aim, 100.0f, dt);
            missile.publish(engine);
        });
    }

    void missileSupersonicFlyby(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        ScriptedMissile missile;
        const float speed = 780.0f; // ~Mach 2.3
        missile.spawn(engine, glm::vec3(-3500.0f, 45.0f, -30.0f), glm::vec3(speed, 0.0f, 0.0f), 21);
        missile.thrust = 10000.0f;
        scene.run(14.0f, [&](float, float dt)
        {
            missile.position += missile.velocity * dt;
            missile.publish(engine);
        });
    }

    void missileCoastFlyby(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        ScriptedMissile missile;
        missile.spawn(engine, glm::vec3(-1200.0f, 20.0f, -18.0f), glm::vec3(260.0f, 0.0f, 0.0f), 31);
        missile.thrust = 0.0f;
        scene.run(10.0f, [&](float, float dt)
        {
            missile.position += missile.velocity * dt;
            missile.publish(engine);
        });
    }

    void jetFlyby(Scene &scene, float throttle, float speed, float altitude, uint64_t seed)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.2f, -1.0f));
        ScriptedJet jet;
        // Passes overhead 18 s after spawning; the first 10 s are pre-rolled so
        // the recording opens on an approach that has been audible for a while.
        jet.spawn(engine, glm::vec3(-speed * 18.0f, altitude, -60.0f), glm::vec3(speed, 0.0f, 0.0f), throttle, seed);
        const auto fly = [&](float, float dt)
        {
            jet.position += jet.velocity * dt;
            jet.publish(engine);
        };
        scene.preroll(10.0f, fly);
        scene.run(18.0f, fly);
    }

    void jetPowerSweep(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        ScriptedJet jet;
        // Slow orbit ~450 m away so the whole power sweep is heard from the side/rear.
        jet.spawn(engine, glm::vec3(450.0f, 150.0f, 0.0f), glm::vec3(0.0f, 0.0f, -120.0f), 0.2f, 41);
        scene.run(22.0f, [&](float t, float)
        {
            const float angle = t * 0.12f;
            jet.position = glm::vec3(450.0f * std::cos(angle), 150.0f, -450.0f * std::sin(angle));
            jet.velocity = glm::vec3(-450.0f * 0.12f * std::sin(angle), 0.0f, -450.0f * 0.12f * std::cos(angle));
            if (t < 4.0f)
            {
                jet.throttle = 0.15f;
            }
            else if (t < 10.0f)
            {
                jet.throttle = 0.9f;
            }
            else if (t < 16.0f)
            {
                jet.throttle = 1.0f;
            }
            else
            {
                jet.throttle = 0.5f;
            }
            jet.publish(engine);
        });
    }

    void explosionAt(Scene &scene, float range, float seconds)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        bool fired = false;
        scene.run(seconds, [&](float t, float)
        {
            if (!fired && t >= 0.3f)
            {
                fired = true;
                spawnExplosion(engine, glm::vec3(range * 0.6f, 120.0f, -range * 0.8f), 10.0f, 51);
            }
        });
    }

    void flareSalvo(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.3f, -1.0f));
        ScriptedJet jet;
        jet.spawn(engine, glm::vec3(-900.0f, 140.0f, -120.0f), glm::vec3(260.0f, 0.0f, 0.0f), 1.0f, 61);
        std::vector<ScriptedFlare> flares;
        float nextFlare = 2.4f;
        int fired = 0;
        scene.run(14.0f, [&](float t, float dt)
        {
            jet.position += jet.velocity * dt;
            jet.publish(engine);
            if (fired < 8 && t >= nextFlare)
            {
                ScriptedFlare flare;
                flare.position = jet.position + glm::vec3(-6.0f, -1.0f, (fired % 2 == 0) ? 1.0f : -1.0f);
                flare.velocity = jet.velocity + glm::vec3(0.0f, -20.0f, (fired % 2 == 0) ? 30.0f : -30.0f);
                flare.voice = std::make_shared<FlareVoice>(100 + fired);
                flare.control = engine.spawn(FlareVoice::spec(), flare.voice, flare.position, flare.velocity, safeNormalize(flare.velocity));
                flares.push_back(flare);
                ++fired;
                nextFlare += (fired % 2 == 0) ? 0.35f : 0.12f;
            }
            for (ScriptedFlare &flare : flares)
            {
                flare.update(engine, dt);
            }
        });
    }

    // Chase camera 15 m behind and 3 m above the missile from ejection through
    // boost and sustain, the way the simulator's missile camera follows it.
    void missileChaseCamera(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        ScriptedMissile missile;
        const glm::vec3 pad(0.0f, 1.0f, 0.0f);
        auto wind = std::make_shared<ListenerWindVoice>(5);
        engine.addLocalSource(wind);
        bool spawned = false;
        const auto follow = [&]()
        {
            const glm::vec3 forward = safeNormalize(missile.velocity, glm::vec3(0, 1, 0));
            glm::vec3 camera = missile.position - forward * 15.0f + glm::vec3(0.0f, 3.0f, 12.0f * std::max(forward.y, 0.0f));
            camera.y = std::max(camera.y, 2.0f);
            scene.listener(camera, forward, missile.velocity);
            wind->setAirspeed(glm::length(missile.velocity));
        };
        scene.listener(pad - glm::vec3(0.0f, 0.0f, -15.0f) + glm::vec3(0.0f, 3.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        scene.run(14.0f, [&](float t, float dt)
        {
            if (!spawned && t >= 0.5f)
            {
                spawned = true;
                engine.spawn(LaunchEjectVoice::spec(), std::make_shared<LaunchEjectVoice>(8), pad, glm::vec3(0.0f), glm::vec3(0, 1, 0));
                missile.spawn(engine, pad, glm::vec3(0.0f, 28.0f, 0.0f), 12);
            }
            if (!spawned)
            {
                return;
            }
            const float flight = t - 0.5f;
            missile.thrust = flight < 0.85f ? 0.0f : (flight < 2.35f ? 30000.0f : (flight < 9.0f ? 10000.0f : 0.0f));
            const float pitch = glm::radians(glm::mix(88.0f, 20.0f, glm::clamp((flight - 0.9f) / 1.5f, 0.0f, 1.0f)));
            flyMissile(missile, glm::vec3(0.0f, std::sin(pitch), std::cos(pitch)), 90.0f, dt);
            missile.publish(engine);
            follow();
        });
    }

    // Chase camera 25 m behind a jet that lights the afterburner and breaks.
    void jetChaseCamera(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        ScriptedJet jet;
        jet.spawn(engine, glm::vec3(0.0f, 600.0f, 0.0f), glm::vec3(0.0f, 0.0f, -230.0f), 0.75f, 91);
        auto wind = std::make_shared<ListenerWindVoice>(6);
        engine.addLocalSource(wind);
        const auto fly = [&](float t, float dt)
        {
            jet.throttle = (t < 4.0f) ? 0.75f : (t < 11.0f ? 1.0f : 0.6f);
            if (t > 6.0f && t < 9.0f)
            {
                const glm::vec3 turn = safeNormalize(glm::vec3(jet.velocity.z, 0.0f, -jet.velocity.x));
                jet.velocity = safeNormalize(jet.velocity + turn * 0.6f * dt) * glm::length(jet.velocity);
            }
            jet.velocity = safeNormalize(jet.velocity) * (230.0f + 60.0f * glm::clamp((t - 4.0f) / 5.0f, 0.0f, 1.0f));
            jet.position += jet.velocity * dt;
            jet.publish(engine);
            const glm::vec3 forward = safeNormalize(jet.velocity);
            scene.listener(jet.position - forward * 25.0f + glm::vec3(0.0f, 6.0f, 0.0f), forward, jet.velocity);
            wind->setAirspeed(glm::length(jet.velocity));
        };
        scene.preroll(1.0f, fly);
        scene.run(15.0f, fly);
    }

    // Headset cues on their own: seeker search growl, a target drifting into
    // the seeker's view, lock tone, then a missile warning of rising urgency.
    void cockpitCues(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(0.0f, 1.7f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        auto seeker = std::make_shared<SeekerToneVoice>(3);
        auto warning = std::make_shared<MissileWarningVoice>();
        engine.addLocalSource(seeker);
        engine.addLocalSource(warning);
        scene.run(16.0f, [&](float t, float)
        {
            SeekerToneParams tone;
            tone.powered = t > 0.5f && t < 8.0f;
            tone.signal = glm::clamp((t - 2.0f) / 3.0f, 0.0f, 1.0f);
            tone.locked = t > 5.5f;
            seeker->setParams(tone);

            MissileWarningParams maws;
            maws.timbre = t > 9.0f && t < 15.5f ? WarningTimbre::Approach : WarningTimbre::Off;
            maws.urgency = glm::clamp((t - 9.0f) / 5.0f, 0.0f, 1.0f);
            warning->setParams(maws);
        });
    }

    void engagement(Scene &scene)
    {
        AudioEngine &engine = scene.engine();
        scene.listener(glm::vec3(-40.0f, 1.7f, 30.0f), glm::vec3(0.3f, 0.35f, -1.0f));

        ScriptedJet jet;
        jet.spawn(engine, glm::vec3(-2600.0f, 900.0f, -2200.0f), glm::vec3(250.0f, 0.0f, 20.0f), 0.8f, 71);
        ScriptedMissile missile;
        std::vector<ScriptedFlare> flares;
        bool launched = false;
        bool detonated = false;
        float nextFlare = 0.0f;
        int flareCount = 0;

        scene.run(24.0f, [&](float t, float dt)
        {
            // Target: cruise, then break and go to afterburner when the missile launches.
            if (launched && !detonated)
            {
                jet.throttle = 1.0f;
                const glm::vec3 breakTurn = safeNormalize(glm::vec3(jet.velocity.z, 0.0f, -jet.velocity.x));
                jet.velocity = safeNormalize(jet.velocity + breakTurn * 60.0f * dt) * 290.0f;
            }
            jet.position += jet.velocity * dt;
            if (!detonated)
            {
                jet.publish(engine);
            }

            if (!launched && t >= 2.0f)
            {
                launched = true;
                engine.spawn(LaunchEjectVoice::spec(), std::make_shared<LaunchEjectVoice>(72), glm::vec3(0, 1, 0), glm::vec3(0.0f), glm::vec3(0, 1, 0));
                missile.spawn(engine, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 28.0f, 0.0f), 73);
                nextFlare = t + 1.5f;
            }

            if (launched && !detonated)
            {
                const float flight = t - 2.0f;
                missile.thrust = flight < 0.4f ? 0.0f : (flight < 2.8f ? 40000.0f : (flight < 9.0f ? 10000.0f : 0.0f));
                const glm::vec3 toTarget = jet.position - missile.position;
                const glm::vec3 aim = flight < 0.9f ? glm::vec3(0, 1, 0) : safeNormalize(toTarget + jet.velocity * (glm::length(toTarget) / 900.0f));
                flyMissile(missile, aim, 100.0f, dt);
                missile.publish(engine);

                if (flareCount < 10 && t >= nextFlare)
                {
                    ScriptedFlare flare;
                    flare.position = jet.position;
                    flare.velocity = jet.velocity + glm::vec3(0.0f, -25.0f, 0.0f);
                    flare.voice = std::make_shared<FlareVoice>(200 + flareCount);
                    flare.control = engine.spawn(FlareVoice::spec(), flare.voice, flare.position, flare.velocity, safeNormalize(flare.velocity));
                    flares.push_back(flare);
                    ++flareCount;
                    nextFlare += 0.25f;
                }

                if (glm::length(jet.position - missile.position) < 18.0f || flight > 14.0f)
                {
                    detonated = true;
                    spawnExplosion(engine, missile.position, 10.0f, 74);
                    missile.control->release();
                    jet.control->release();
                    std::printf("    detonation at t=%.2f s, range to listener %.0f m\n", t, glm::length(missile.position));
                }
            }
            for (ScriptedFlare &flare : flares)
            {
                flare.update(engine, dt);
            }
        });
    }

    struct Scenario
    {
        const char *name;
        std::function<void(Scene &)> body;
    };
}

int main(int argc, char **argv)
{
    const std::filesystem::path outputDirectory = (argc > 1) ? argv[1] : "audition";
    const std::string filter = (argc > 2) ? argv[2] : "";
    if (argc > 3)
    {
        g_reverbAmount = std::stof(argv[3]);
    }
    g_trace = argc > 4 && std::string(argv[4]) == "trace";
    std::filesystem::create_directories(outputDirectory);

    const std::vector<Scenario> scenarios = {
        {"01_missile_launch_side", missileLaunchSide},
        {"02_missile_supersonic_flyby", missileSupersonicFlyby},
        {"03_missile_coast_flyby", missileCoastFlyby},
        {"04_jet_flyby_military", [](Scene &s) { jetFlyby(s, 0.85f, 270.0f, 110.0f, 81); }},
        {"05_jet_flyby_afterburner", [](Scene &s) { jetFlyby(s, 1.0f, 300.0f, 80.0f, 82); }},
        {"06_jet_power_sweep", jetPowerSweep},
        {"07_explosion_120m", [](Scene &s) { explosionAt(s, 120.0f, 7.0f); }},
        {"08_explosion_700m", [](Scene &s) { explosionAt(s, 700.0f, 9.0f); }},
        {"09_explosion_2500m", [](Scene &s) { explosionAt(s, 2500.0f, 15.0f); }},
        {"10_flare_salvo", flareSalvo},
        {"11_engagement", engagement},
        {"12_missile_chase_camera", missileChaseCamera},
        {"13_jet_chase_camera", jetChaseCamera},
        {"14_cockpit_cues", cockpitCues},
        {"90_diag_ground_comb", diagnosticGroundComb},
    };

    std::printf("Rendering audition scenarios to %s\n", std::filesystem::absolute(outputDirectory).string().c_str());
    bool ok = true;
    for (const Scenario &scenario : scenarios)
    {
        if (!filter.empty() && std::string(scenario.name).find(filter) == std::string::npos)
        {
            continue;
        }
        Scene scene;
        scenario.body(scene);
        ok = scene.finish(outputDirectory / (std::string(scenario.name) + ".wav")) && ok;
    }
    return ok ? 0 : 1;
}
