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
#include <unordered_set>

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

    Emitter<synth::RocketMotorVoice> missile;
    std::unordered_map<const Target *, TargetVoice> targets;
    std::unordered_map<const Flare *, Emitter<synth::FlareVoice>> flares;
    std::unordered_set<const Flare *> retiredFlares;

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

    void updateMissile(const Missile &source)
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

void AudioSystem::syncMissile(const Missile *missile, bool inFlight)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }
    if (missile == nullptr || !inFlight)
    {
        impl.missile.release();
        return;
    }
    impl.updateMissile(*missile);
}

void AudioSystem::syncTargets(const std::vector<Target *> &activeTargets)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }

    std::unordered_set<const Target *> seen;
    seen.reserve(activeTargets.size());
    for (const Target *target : activeTargets)
    {
        if (target == nullptr)
        {
            continue;
        }
        seen.insert(target);
        impl.updateTarget(*target, impl.targets[target]);
    }

    for (auto it = impl.targets.begin(); it != impl.targets.end();)
    {
        if (seen.count(it->first) == 0)
        {
            it->second.emitter.release();
            it = impl.targets.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void AudioSystem::syncFlares(const std::vector<Flare *> &activeFlares)
{
    Impl &impl = *m_impl;
    if (!impl.running)
    {
        return;
    }

    const std::unordered_set<const Flare *> active(activeFlares.begin(), activeFlares.end());
    for (const Flare *flare : active)
    {
        if (flare != nullptr && impl.retiredFlares.count(flare) == 0)
        {
            impl.updateFlare(*flare, impl.flares[flare]);
        }
    }

    for (auto it = impl.flares.begin(); it != impl.flares.end();)
    {
        if (active.count(it->first) == 0)
        {
            it->second.release();
            it = impl.flares.erase(it);
        }
        else
        {
            ++it;
        }
    }
    // A retired flare's address may be reused by a new flare once the old one
    // has left the active list, so forget it at that point.
    for (auto it = impl.retiredFlares.begin(); it != impl.retiredFlares.end();)
    {
        it = (active.count(*it) == 0) ? impl.retiredFlares.erase(it) : std::next(it);
    }
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
    warning.active = cues.missileWarning && !impl.world.paused;
    warning.urgency = cues.missileWarningUrgency;
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

void AudioSystem::retireFlare(const Flare *flare)
{
    Impl &impl = *m_impl;
    if (!impl.running || flare == nullptr)
    {
        return;
    }
    const auto it = impl.flares.find(flare);
    if (it != impl.flares.end())
    {
        it->second.release();
        impl.flares.erase(it);
    }
    impl.retiredFlares.insert(flare);
}

void AudioSystem::stopMissileEmitters()
{
    m_impl->missile.release();
}

void AudioSystem::stopAllEmitters()
{
    Impl &impl = *m_impl;
    impl.missile.release();
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
    impl.retiredFlares.clear();
}
