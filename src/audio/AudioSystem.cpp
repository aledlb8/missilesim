#include "AudioSystem.h"

#include "audio/engine/AudioEngine.h"
#include "audio/synth/Cues.h"
#include "audio/synth/Voices.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/Atmosphere.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace audio = missilesim::audio;
namespace synth = missilesim::audio::synth;

namespace
{
    // AIM-9 class airframe; the sim does not model the body length.
    constexpr float kMissileBodyLength = 2.9f;
    constexpr float kDefaultNozzleDiameter = 0.1f;
    // Blast-fragmentation warhead, TNT equivalent.
    constexpr float kWarheadChargeKg = 8.0f;
    // A camera moving faster than this between two frames was cut, not flown.
    constexpr float kListenerCutSpeed = 2500.0f;
    constexpr float kListenerVelocitySmoothingSeconds = 0.08f;
    // Engine demand is smoothed so thrust-limited manoeuvring does not flick
    // the afterburner in and out every frame.
    constexpr float kThrottleSmoothingSeconds = 0.6f;

    glm::vec3 directionOr(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        const float length = glm::length(v);
        return (length > 1.0e-4f && std::isfinite(length)) ? v / length : fallback;
    }

    float exponentialBlend(float seconds, float dt)
    {
        return (seconds > 0.0f) ? 1.0f - std::exp(-std::max(dt, 0.0f) / seconds) : 1.0f;
    }

    template <typename Voice>
    struct Emitter
    {
        std::shared_ptr<Voice> voice;
        std::shared_ptr<audio::EmitterControl> control;

        explicit operator bool() const { return voice && control; }

        void release()
        {
            if (control)
            {
                control->release();
            }
            voice.reset();
            control.reset();
        }
    };
}

struct AudioSystem::Impl
{
    audio::AudioEngine engine;
    bool running = false;
    uint64_t nextSeed = 0xA11CE5EEDull;

    AudioWorldState world;
    float wallDeltaSeconds = 1.0f / 60.0f;
    Atmosphere atmosphere;

    bool listenerValid = false;
    glm::vec3 listenerPosition{0.0f};
    glm::vec3 listenerVelocity{0.0f};

    struct TargetVoice
    {
        Emitter<synth::TurbofanVoice> emitter;
        float throttle = 0.7f;
    };

    std::unordered_map<std::uint32_t, Emitter<synth::RocketMotorVoice>> missiles;
    std::unordered_map<std::uint32_t, TargetVoice> targets;
    std::unordered_map<std::uint32_t, Emitter<synth::FlareVoice>> flares;
    std::unordered_map<std::uint32_t, Emitter<synth::ChaffVoice>> chaff;

    // Updates every listed source and releases the voices of sources that
    // are no longer listed.
    template <typename Source, typename Voices, typename Update>
    void syncSources(const std::vector<Source> &sources, Voices &voices, Update update)
    {
        std::unordered_map<std::uint32_t, bool> seen;
        seen.reserve(sources.size());
        for (const Source &source : sources)
        {
            if (source.object == nullptr || source.id == 0)
            {
                continue;
            }
            seen[source.id] = true;
            update(*source.object, voices[source.id]);
        }
        for (auto it = voices.begin(); it != voices.end();)
        {
            if (seen.count(it->first) == 0)
            {
                releaseVoice(it->second);
                it = voices.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    static void releaseVoice(TargetVoice &voice) { voice.emitter.release(); }
    template <typename Voice>
    static void releaseVoice(Emitter<Voice> &voice) { voice.release(); }

    std::shared_ptr<synth::SeekerToneVoice> seeker;
    std::shared_ptr<synth::MissileWarningVoice> missileWarning;
    std::shared_ptr<synth::ListenerWindVoice> wind;

    uint64_t seed()
    {
        nextSeed = nextSeed * 6364136223846793005ull + 1442695040888963407ull;
        return nextSeed;
    }

    // Kinematics are handed to the engine in real (wall-clock) units: the
    // engine scales propagation to the simulation speed itself.
    glm::vec3 wallVelocity(const glm::vec3 &simulationVelocity) const
    {
        return world.paused ? glm::vec3(0.0f) : simulationVelocity * world.timeScale;
    }

    float airDensityAt(float altitude) const
    {
        return atmosphere.sample(altitude).densityKgPerCubicMeter;
    }

    void applyEnvironment()
    {
        const Atmosphere::State ground = atmosphere.sample(world.groundLevel);
        audio::EnvironmentSettings settings = engine.environment();
        settings.temperatureK = ground.temperatureKelvin;
        settings.pressurePa = ground.pressurePascals;
        settings.groundLevel = world.groundLevel;
        settings.groundPresent = world.groundPresent;
        settings.timeScale = world.timeScale;
        settings.paused = world.paused;
        settings.masterVolume = world.masterVolume;
        engine.setEnvironment(settings);
    }

    void updateMissile(const Missile &source, Emitter<synth::RocketMotorVoice> &missile)
    {
        const bool burning = source.isThrustEnabled() && source.getFuel() > 0.0f;
        const glm::vec3 velocity = source.getVelocity();
        const glm::vec3 axis = burning ? directionOr(source.getThrustDirection(), directionOr(velocity, glm::vec3(0, 1, 0)))
                                       : directionOr(velocity, glm::vec3(0, 1, 0));

        synth::RocketMotorParams params;
        params.thrustNewtons = burning ? source.getThrust() * std::clamp(source.getThrottle(), 0.0f, 1.0f) : 0.0f;
        const float exhaustVelocity = source.getExhaustVelocity();
        if (exhaustVelocity > 0.0f)
        {
            params.exhaustVelocity = exhaustVelocity;
        }
        const float nozzleArea = source.getNozzleExitArea();
        params.nozzleDiameter = (nozzleArea > 0.0f) ? std::sqrt(4.0f * nozzleArea / audio::kPi) : kDefaultNozzleDiameter;
        const float frontalArea = source.getCrossSectionalArea();
        if (frontalArea > 0.0f)
        {
            params.bodyDiameter = std::sqrt(4.0f * frontalArea / audio::kPi);
        }
        params.airDensity = airDensityAt(source.getPosition().y);

        if (!missile)
        {
            missile.voice = std::make_shared<synth::RocketMotorVoice>(seed());
            missile.voice->setParams(params);
            missile.control = engine.spawn(synth::RocketMotorVoice::spec(kMissileBodyLength, params.bodyDiameter),
                                           missile.voice,
                                           source.getPosition(),
                                           wallVelocity(velocity),
                                           axis);
            if (!missile.control)
            {
                missile.voice.reset();
            }
            return;
        }

        missile.voice->setParams(params);
        engine.moveEmitter(*missile.control, source.getPosition(), wallVelocity(velocity), axis);
    }

    void updateTarget(const Target &target, TargetVoice &state)
    {
        const glm::vec3 velocity = target.getVelocity();
        const glm::vec3 axis = directionOr(velocity, glm::vec3(0.0f, 0.0f, 1.0f));
        const float demand = std::clamp(target.getThrottle(), 0.0f, 1.0f);

        if (!state.emitter)
        {
            state.throttle = demand;
            state.emitter.voice = std::make_shared<synth::TurbofanVoice>(seed(), demand);
            state.emitter.control = engine.spawn(synth::TurbofanVoice::spec(),
                                                 state.emitter.voice,
                                                 target.getPosition(),
                                                 wallVelocity(velocity),
                                                 axis);
            if (!state.emitter.control)
            {
                state.emitter.voice.reset();
                return;
            }
        }
        else if (!world.paused)
        {
            state.throttle += (demand - state.throttle) * exponentialBlend(kThrottleSmoothingSeconds, wallDeltaSeconds * world.timeScale);
        }

        synth::TurbofanParams params;
        params.throttle = state.throttle;
        params.airDensity = airDensityAt(target.getPosition().y);
        state.emitter.voice->setParams(params);
        engine.moveEmitter(*state.emitter.control, target.getPosition(), wallVelocity(velocity), axis);
    }

    void updateFlare(const Flare &flare, Emitter<synth::FlareVoice> &emitter)
    {
        const glm::vec3 velocity = flare.getVelocity();
        const glm::vec3 axis = directionOr(velocity, glm::vec3(0.0f, -1.0f, 0.0f));
        synth::FlareParams params;
        const float initialHeat = flare.getInitialHeatSignature();
        params.heat = (initialHeat > 1.0e-4f) ? std::clamp(flare.getHeatSignature() / initialHeat, 0.0f, 1.0f) : 0.0f;

        if (!emitter)
        {
            emitter.voice = std::make_shared<synth::FlareVoice>(seed());
            emitter.voice->setParams(params);
            emitter.control = engine.spawn(synth::FlareVoice::spec(), emitter.voice, flare.getPosition(), wallVelocity(velocity), axis);
            if (!emitter.control)
            {
                emitter.voice.reset();
            }
            return;
        }

        emitter.voice->setParams(params);
        engine.moveEmitter(*emitter.control, flare.getPosition(), wallVelocity(velocity), axis);
    }

    void updateChaff(const AudioChaffState &cloud, Emitter<synth::ChaffVoice> &emitter)
    {
        const glm::vec3 velocity = cloud.velocity;
        const glm::vec3 axis = directionOr(velocity, glm::vec3(0.0f, -1.0f, 0.0f));
        synth::ChaffParams params;
        params.bloom = std::clamp(cloud.bloom, 0.0f, 1.0f);

        if (!emitter)
        {
            emitter.voice = std::make_shared<synth::ChaffVoice>(seed());
            emitter.voice->setParams(params);
            emitter.control = engine.spawn(synth::ChaffVoice::spec(), emitter.voice, cloud.position, wallVelocity(velocity), axis);
            if (!emitter.control)
            {
                emitter.voice.reset();
            }
            return;
        }

        emitter.voice->setParams(params);
        engine.moveEmitter(*emitter.control, cloud.position, wallVelocity(velocity), axis);
    }
};

AudioSystem::AudioSystem() : m_impl(std::make_unique<Impl>()) {}

AudioSystem::~AudioSystem()
{
    shutdown();
}

bool AudioSystem::initialize()
{
    Impl &impl = *m_impl;
    if (impl.running)
    {
        return true;
    }

    impl.applyEnvironment();
    if (!impl.engine.start())
    {
        return false;
    }
    impl.running = true;

    impl.seeker = std::make_shared<synth::SeekerToneVoice>(impl.seed());
    impl.missileWarning = std::make_shared<synth::MissileWarningVoice>();
    impl.wind = std::make_shared<synth::ListenerWindVoice>(impl.seed());
    impl.engine.addLocalSource(impl.seeker);
    impl.engine.addLocalSource(impl.missileWarning);
    impl.engine.addLocalSource(impl.wind);
    return true;
}

void AudioSystem::shutdown()
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    stopAllEmitters();
    impl.seeker->release();
    impl.missileWarning->release();
    impl.wind->release();
    impl.engine.stop();
    impl.engine.collectGarbage();
    impl.seeker.reset();
    impl.missileWarning.reset();
    impl.wind.reset();
    impl.running = false;
}

bool AudioSystem::isEnabled() const
{
    return m_impl->running;
}

void AudioSystem::beginFrame(const AudioWorldState &world, float wallDeltaSeconds)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }

    const bool densityChanged = std::abs(world.seaLevelAirDensity - impl.world.seaLevelAirDensity) > 1.0e-4f;
    impl.world = world;
    impl.world.timeScale = std::max(world.timeScale, 1.0e-3f);
    impl.wallDeltaSeconds = (wallDeltaSeconds > 0.0f && std::isfinite(wallDeltaSeconds)) ? wallDeltaSeconds : 1.0f / 60.0f;
    if (densityChanged)
    {
        impl.atmosphere.setDensity(world.seaLevelAirDensity);
    }
    impl.applyEnvironment();
}

void AudioSystem::setListener(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &up)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }

    glm::vec3 velocity(0.0f);
    if (impl.listenerValid && !impl.world.paused)
    {
        const glm::vec3 measured = (position - impl.listenerPosition) / impl.wallDeltaSeconds;
        if (glm::length(measured) < kListenerCutSpeed)
        {
            velocity = impl.listenerVelocity +
                       (measured - impl.listenerVelocity) * exponentialBlend(kListenerVelocitySmoothingSeconds, impl.wallDeltaSeconds);
        }
    }
    impl.listenerPosition = position;
    impl.listenerVelocity = velocity;
    impl.listenerValid = true;

    impl.engine.setListener(position, velocity, directionOr(forward, glm::vec3(0, 0, -1)), directionOr(up, glm::vec3(0, 1, 0)));
    if (impl.wind)
    {
        impl.wind->setAirspeed(glm::length(velocity) / impl.world.timeScale);
    }
}

void AudioSystem::syncMissiles(const std::vector<AudioMissileSource> &missiles)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    impl.syncSources(missiles, impl.missiles,
                     [&impl](const Missile &missile, Emitter<synth::RocketMotorVoice> &voice) { impl.updateMissile(missile, voice); });
}

void AudioSystem::syncTargets(const std::vector<AudioTargetSource> &activeTargets)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    impl.syncSources(activeTargets, impl.targets,
                     [&impl](const Target &target, Impl::TargetVoice &voice) { impl.updateTarget(target, voice); });
}

void AudioSystem::syncFlares(const std::vector<AudioFlareSource> &activeFlares)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    impl.syncSources(activeFlares, impl.flares,
                     [&impl](const Flare &flare, Emitter<synth::FlareVoice> &voice) { impl.updateFlare(flare, voice); });
}

void AudioSystem::syncChaff(const std::vector<AudioChaffSource> &activeChaff)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    impl.syncSources(activeChaff, impl.chaff,
                     [&impl](const AudioChaffState &cloud, Emitter<synth::ChaffVoice> &voice) { impl.updateChaff(cloud, voice); });
}

void AudioSystem::syncCockpitCues(const CockpitCueState &cues)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }

    synth::SeekerToneParams seeker;
    seeker.powered = cues.seekerPowered && !impl.world.paused;
    seeker.locked = cues.seekerLocked;
    seeker.signal = cues.seekerSignal;
    impl.seeker->setParams(seeker);

    synth::MissileWarningParams warning;
    warning.urgency = cues.missileWarningUrgency;
    if (impl.world.paused)
    {
        warning.timbre = synth::WarningTimbre::Off;
    }
    else
    {
        switch (cues.alert)
        {
        case HeadsetAlert::RadarSearch:
            warning.timbre = synth::WarningTimbre::Search;
            break;
        case HeadsetAlert::RadarTrack:
            warning.timbre = synth::WarningTimbre::Track;
            break;
        case HeadsetAlert::RadarLaunch:
            warning.timbre = synth::WarningTimbre::Launch;
            break;
        case HeadsetAlert::MissileSeeker:
            warning.timbre = synth::WarningTimbre::Seeker;
            break;
        case HeadsetAlert::Approach:
            warning.timbre = synth::WarningTimbre::Approach;
            break;
        case HeadsetAlert::None:
            warning.timbre = cues.missileWarning ? synth::WarningTimbre::Approach : synth::WarningTimbre::Off;
            break;
        }
    }
    impl.missileWarning->setParams(warning);
}

void AudioSystem::endFrame()
{
    if (m_impl->running)
    {
        m_impl->engine.collectGarbage();
    }
}

void AudioSystem::playLaunch(const glm::vec3 &position)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    impl.engine.spawn(synth::LaunchEjectVoice::spec(),
                      std::make_shared<synth::LaunchEjectVoice>(impl.seed()),
                      position,
                      glm::vec3(0.0f),
                      glm::vec3(0.0f, 1.0f, 0.0f));
}

void AudioSystem::playExplosion(const glm::vec3 &position)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    const float ambientPressure = impl.atmosphere.sample(position.y).pressurePascals;
    impl.engine.spawn(synth::ExplosionVoice::spec(),
                      std::make_shared<synth::ExplosionVoice>(impl.seed(), kWarheadChargeKg, ambientPressure),
                      position,
                      glm::vec3(0.0f),
                      glm::vec3(0.0f, 1.0f, 0.0f));
}

void AudioSystem::stopAllEmitters()
{
    Impl &impl = *m_impl;
    for (auto &entry : impl.missiles)
    {
        entry.second.release();
    }
    impl.missiles.clear();
    for (auto &entry : impl.targets)
    {
        entry.second.emitter.release();
    }
    impl.targets.clear();
    for (auto &entry : impl.flares)
    {
        entry.second.release();
    }
    impl.flares.clear();
    for (auto &entry : impl.chaff)
    {
        entry.second.release();
    }
    impl.chaff.clear();
}
