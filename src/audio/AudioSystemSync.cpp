#include "AudioSystemImpl.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using missilesim::audio::detail::applyHeadroom;
using missilesim::audio::detail::kExplosionPreset;
using missilesim::audio::detail::kFlareBurnPreset;
using missilesim::audio::detail::kFlareLaunchPreset;
using missilesim::audio::detail::kJetAfterburnerPreset;
using missilesim::audio::detail::kLaunchPreset;
using missilesim::audio::detail::kLoopVoiceHeadroom;
using missilesim::audio::detail::kMasterMixVolume;
using missilesim::audio::detail::kMissileAfterburnerPreset;
using missilesim::audio::detail::kMissileAirframePreset;
using missilesim::audio::detail::kOneShotHeadroom;
using missilesim::audio::detail::kUiLoopHeadroom;
using missilesim::audio::detail::kUiOneShotHeadroom;
using missilesim::audio::detail::resolveAssetPath;
using missilesim::audio::detail::safeNormalize;
using missilesim::audio::detail::saturate;
using missilesim::audio::detail::smoothingFactor;
using missilesim::audio::detail::SpatialSoundPreset;

void AudioSystem::Impl::update(float deltaTime)
{
    if (!m_enabled)
    {
        return;
    }

    const float dt = std::clamp(deltaTime, 0.0f, 0.1f);
    updateLoopEmitter(m_missileAirframe, dt);
    updateLoopEmitter(m_missileAfterburner, dt);
    updateUiLoopEmitter(m_seekerSearch, dt);
    updateUiLoopEmitter(m_seekerLock, dt);
    updateUiLoopEmitter(m_mawsMissileWarning, dt);

    for (auto it = m_targetLoops.begin(); it != m_targetLoops.end();)
    {
        TargetLoopState &state = *it->second;
        updateLoopEmitter(state.afterburner, dt);

        if (state.retireWhenSilent && state.afterburner.currentVolume <= 0.0005f)
        {
            shutdownLoopEmitter(state.afterburner);
            it = m_targetLoops.erase(it);
            continue;
        }

        ++it;
    }

    for (auto it = m_flareLoops.begin(); it != m_flareLoops.end();)
    {
        FlareLoopState &state = *it->second;
        updateLoopEmitter(state.burn, dt);

        if (state.retireWhenSilent && state.burn.currentVolume <= 0.0005f)
        {
            shutdownLoopEmitter(state.burn);
            it = m_flareLoops.erase(it);
            continue;
        }

        ++it;
    }

    pruneOneShots();
}

void AudioSystem::Impl::setListener(const glm::vec3 &position,
                                    const glm::vec3 &forward,
                                    const glm::vec3 &up,
                                    float deltaTime)
{
    if (!m_enabled)
    {
        return;
    }

    glm::vec3 listenerVelocity(0.0f);
    if (m_listenerInitialized && deltaTime > 0.0001f)
    {
        listenerVelocity = (position - m_lastListenerPosition) / deltaTime;
    }

    const glm::vec3 listenerForward = safeNormalize(forward, glm::vec3(0.0f, 0.0f, -1.0f));
    const glm::vec3 listenerUp = safeNormalize(up, glm::vec3(0.0f, 1.0f, 0.0f));

    ma_engine_listener_set_position(&m_engine, 0, position.x, position.y, position.z);
    ma_engine_listener_set_direction(&m_engine, 0, listenerForward.x, listenerForward.y, listenerForward.z);
    ma_engine_listener_set_world_up(&m_engine, 0, listenerUp.x, listenerUp.y, listenerUp.z);
    ma_engine_listener_set_velocity(&m_engine, 0, listenerVelocity.x, listenerVelocity.y, listenerVelocity.z);

    m_lastListenerPosition = position;
    m_listenerInitialized = true;
}

void AudioSystem::Impl::syncMissile(const Missile *missile, bool missileInFlight, float referenceFuel)
{
    if (!m_enabled)
    {
        return;
    }

    if (missile == nullptr || !missileInFlight)
    {
        m_missileAirframe.targetVolume = 0.0f;
        m_missileAfterburner.targetVolume = 0.0f;
        return;
    }

    const glm::vec3 velocity = missile->getVelocity();
    const float speed = glm::length(velocity);
    const glm::vec3 forward = safeNormalize(velocity, missile->getThrustDirection());
    const float speedFraction = saturate(speed / 520.0f);

    m_missileAirframe.position = missile->getPosition();
    m_missileAirframe.velocity = velocity;
    m_missileAirframe.direction = forward;
    m_missileAirframe.targetVolume = (speed > 12.0f) ? (0.10f + (speedFraction * 0.26f)) : 0.0f;
    m_missileAirframe.targetPitch = 0.92f + (speedFraction * 0.24f);

    const bool boosterActive = missile->isThrustEnabled() && missile->getFuel() > 0.0f;
    const float fuelFraction = (referenceFuel > 0.0f)
                                   ? saturate(missile->getFuel() / referenceFuel)
                                   : 1.0f;

    m_missileAfterburner.position = missile->getPosition() - (forward * 1.0f);
    m_missileAfterburner.velocity = velocity;
    m_missileAfterburner.direction = -forward;
    m_missileAfterburner.targetVolume = boosterActive
                                            ? std::clamp(0.34f + (speedFraction * 0.24f) + (fuelFraction * 0.08f), 0.0f, 0.82f)
                                            : 0.0f;
    m_missileAfterburner.targetPitch = 0.98f + (speedFraction * 0.18f);
}

void AudioSystem::Impl::syncTargets(const std::vector<Target *> &activeTargets)
{
    if (!m_enabled)
    {
        return;
    }

    for (auto &entry : m_targetLoops)
    {
        entry.second->seenThisFrame = false;
    }

    for (Target *target : activeTargets)
    {
        if (target == nullptr || !target->isActive())
        {
            continue;
        }

        TargetLoopState *state = getOrCreateTargetState(target);
        if (state == nullptr)
        {
            continue;
        }

        state->seenThisFrame = true;
        state->retireWhenSilent = false;

        const glm::vec3 velocity = target->getVelocity();
        const glm::vec3 forward = safeNormalize(velocity, glm::vec3(0.0f, 0.0f, 1.0f));
        const float radius = std::max(target->getRadius(), 1.0f);
        const TargetAIConfig &config = target->getAIConfig();
        const float speed = glm::length(velocity);
        const float speedBand = std::max(config.maxSpeed - config.minSpeed, 1.0f);
        const float speedFraction = saturate((speed - config.minSpeed) / speedBand);
        const float warningBoost = target->isMissileWarningActive() ? 0.10f : 0.0f;

        state->afterburner.position = target->getPosition() - (forward * radius * 1.25f);
        state->afterburner.velocity = velocity;
        state->afterburner.direction = -forward;
        state->afterburner.targetVolume = std::clamp(0.16f + (speedFraction * 0.32f) + warningBoost, 0.0f, 0.70f);
        state->afterburner.targetPitch = 0.90f + (speedFraction * 0.18f) + (warningBoost * 0.08f);
    }

    for (auto &entry : m_targetLoops)
    {
        TargetLoopState &state = *entry.second;
        if (!state.seenThisFrame)
        {
            state.afterburner.targetVolume = 0.0f;
            state.retireWhenSilent = true;
        }
    }
}

void AudioSystem::Impl::syncFlares(const std::vector<Flare *> &activeFlares)
{
    if (!m_enabled)
    {
        return;
    }

    for (auto &entry : m_flareLoops)
    {
        entry.second->seenThisFrame = false;
    }

    for (Flare *flare : activeFlares)
    {
        if (flare == nullptr || !flare->isActive())
        {
            continue;
        }

        FlareLoopState *state = getOrCreateFlareState(flare);
        if (state == nullptr)
        {
            continue;
        }

        state->seenThisFrame = true;
        state->retireWhenSilent = false;

        const float initialHeat = std::max(flare->getInitialHeatSignature(), 0.0001f);
        const float heatFraction = saturate(flare->getHeatSignature() / initialHeat);
        const glm::vec3 velocity = flare->getVelocity();
        const float speedFraction = saturate(glm::length(velocity) / 180.0f);

        state->burn.position = flare->getPosition();
        state->burn.velocity = velocity;
        state->burn.direction = safeNormalize(-velocity, glm::vec3(0.0f, 1.0f, 0.0f));
        state->burn.targetVolume = (heatFraction > 0.02f)
                                       ? std::clamp(0.08f + (heatFraction * 0.24f) + (speedFraction * 0.06f), 0.0f, 0.38f)
                                       : 0.0f;
        state->burn.targetPitch = 0.94f + (heatFraction * 0.20f) + (speedFraction * 0.06f);
    }

    for (auto &entry : m_flareLoops)
    {
        FlareLoopState &state = *entry.second;
        if (!state.seenThisFrame)
        {
            state.burn.targetVolume = 0.0f;
            state.retireWhenSilent = true;
        }
    }
}

void AudioSystem::Impl::syncCockpitCues(bool seekerPowered,
                                        bool seekerLocked,
                                        float seekerSignalStrength,
                                        bool mawsThreatActive,
                                        float mawsUrgency)
{
    if (!m_enabled)
    {
        return;
    }

    const float seekerStrength = saturate(seekerSignalStrength);
    const float warningUrgency = saturate(mawsUrgency);

    if (seekerPowered && !m_lastSeekerPowered && !m_assets.seekerPowerOn.empty())
    {
        silenceAndStopUiLoopEmitter(m_seekerSearch);
        silenceAndStopUiLoopEmitter(m_seekerLock);
        playUiOneShot(m_assets.seekerPowerOn, 0.56f, 1.0f);
    }

    if (seekerPowered && seekerLocked && !m_lastSeekerLocked && !m_assets.seekerLockAcquire.empty())
    {
        playUiOneShot(m_assets.seekerLockAcquire,
                      0.52f + (seekerStrength * 0.12f),
                      0.98f + (seekerStrength * 0.10f));
    }

    if (mawsThreatActive && !m_lastMawsThreatActive)
    {
        silenceAndStopUiLoopEmitter(m_mawsMissileWarning);
    }

    if (seekerPowered)
    {
        m_seekerSearch.targetVolume = 0.08f + (seekerStrength * 0.14f);
        m_seekerSearch.targetPitch = 0.94f + (seekerStrength * 0.18f);
        if (seekerLocked)
        {
            m_seekerSearch.targetVolume *= 0.14f;
            m_seekerSearch.targetPitch += 0.04f;
        }
    }
    else
    {
        m_seekerSearch.targetVolume = 0.0f;
        m_seekerSearch.targetPitch = 1.0f;
    }

    if (seekerPowered && seekerLocked)
    {
        m_seekerLock.targetVolume = std::clamp(0.18f + (seekerStrength * 0.22f), 0.0f, 0.52f);
        m_seekerLock.targetPitch = 1.02f + (seekerStrength * 0.10f);
    }
    else
    {
        m_seekerLock.targetVolume = 0.0f;
        m_seekerLock.targetPitch = 1.0f;
    }

    if (mawsThreatActive)
    {
        m_mawsMissileWarning.targetVolume = std::clamp(0.22f + (warningUrgency * 0.24f), 0.0f, 0.62f);
        m_mawsMissileWarning.targetPitch = 0.96f + (warningUrgency * 0.14f);
    }
    else
    {
        m_mawsMissileWarning.targetVolume = 0.0f;
        m_mawsMissileWarning.targetPitch = 1.0f;
    }

    m_lastSeekerPowered = seekerPowered;
    m_lastSeekerLocked = seekerPowered && seekerLocked;
    m_lastMawsThreatActive = mawsThreatActive;
}

void AudioSystem::Impl::stopMissileEmitters()
{
    if (!m_enabled)
    {
        return;
    }

    silenceAndStopLoopEmitter(m_missileAirframe);
    silenceAndStopLoopEmitter(m_missileAfterburner);
}

void AudioSystem::Impl::stopAllEmitters()
{
    if (!m_enabled)
    {
        return;
    }

    stopMissileEmitters();
    silenceAndStopUiLoopEmitter(m_seekerSearch);
    silenceAndStopUiLoopEmitter(m_seekerLock);
    silenceAndStopUiLoopEmitter(m_mawsMissileWarning);
    m_lastSeekerPowered = false;
    m_lastSeekerLocked = false;
    m_lastMawsThreatActive = false;

    for (auto &voice : m_oneShots)
    {
        shutdownOneShot(*voice);
    }
    m_oneShots.clear();

    for (auto &entry : m_targetLoops)
    {
        if (entry.second)
        {
            silenceAndStopLoopEmitter(entry.second->afterburner);
            shutdownLoopEmitter(entry.second->afterburner);
        }
    }

    m_targetLoops.clear();

    for (auto &entry : m_flareLoops)
    {
        if (entry.second)
        {
            silenceAndStopLoopEmitter(entry.second->burn);
            shutdownLoopEmitter(entry.second->burn);
        }
    }
    m_flareLoops.clear();
}