#include "AudioSystem.h"
#include "AudioSystemImpl.h"

#include <memory>

AudioSystem::AudioSystem()
    : m_impl(std::make_unique<Impl>())
{
}

AudioSystem::~AudioSystem() = default;

bool AudioSystem::initialize()
{
    return m_impl->initialize();
}

void AudioSystem::shutdown()
{
    m_impl->shutdown();
}

bool AudioSystem::isEnabled() const
{
    return m_impl->m_enabled;
}

void AudioSystem::update(float deltaTime)
{
    m_impl->update(deltaTime);
}

void AudioSystem::setListener(const glm::vec3 &position,
                              const glm::vec3 &forward,
                              const glm::vec3 &up,
                              float deltaTime)
{
    m_impl->setListener(position, forward, up, deltaTime);
}

void AudioSystem::syncMissile(const Missile *missile, bool missileInFlight, float referenceFuel)
{
    m_impl->syncMissile(missile, missileInFlight, referenceFuel);
}

void AudioSystem::syncTargets(const std::vector<Target *> &activeTargets)
{
    m_impl->syncTargets(activeTargets);
}

void AudioSystem::syncFlares(const std::vector<Flare *> &activeFlares)
{
    m_impl->syncFlares(activeFlares);
}

void AudioSystem::syncCockpitCues(bool seekerPowered,
                                  bool seekerLocked,
                                  float seekerSignalStrength,
                                  bool mawsThreatActive,
                                  float mawsUrgency)
{
    m_impl->syncCockpitCues(seekerPowered,
                            seekerLocked,
                            seekerSignalStrength,
                            mawsThreatActive,
                            mawsUrgency);
}

void AudioSystem::playLaunch(const glm::vec3 &position, const glm::vec3 &velocity)
{
    m_impl->playLaunch(position, velocity);
}

void AudioSystem::playFlareLaunch(const glm::vec3 &position, const glm::vec3 &velocity, float heatSignature)
{
    m_impl->playFlareLaunch(position, velocity, heatSignature);
}

void AudioSystem::playExplosion(const glm::vec3 &position, const glm::vec3 &velocity, float intensity)
{
    m_impl->playExplosion(position, velocity, intensity);
}

void AudioSystem::retireFlare(const Flare *flare)
{
    m_impl->retireFlare(flare);
}

void AudioSystem::stopMissileEmitters()
{
    m_impl->stopMissileEmitters();
}

void AudioSystem::stopAllEmitters()
{
    m_impl->stopAllEmitters();
}