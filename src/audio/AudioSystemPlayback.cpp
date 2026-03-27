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

void AudioSystem::Impl::playLaunch(const glm::vec3 &position, const glm::vec3 &velocity)
{
    if (!m_enabled || m_assets.fireLaunch.empty())
    {
        return;
    }

    const float speed = glm::length(velocity);
    const float speedFraction = saturate(speed / 280.0f);
    playOneShot(m_assets.fireLaunch,
                kLaunchPreset,
                position,
                velocity,
                0.96f + (speedFraction * 0.10f),
                0.99f + (speedFraction * 0.04f),
                safeNormalize(-velocity, glm::vec3(0.0f, 0.0f, -1.0f)));
}

void AudioSystem::Impl::playFlareLaunch(const glm::vec3 &position, const glm::vec3 &velocity, float heatSignature)
{
    if (!m_enabled || m_assets.flareLaunch.empty())
    {
        return;
    }

    const float heatFraction = saturate(heatSignature / 6.0f);
    const float speedFraction = saturate(glm::length(velocity) / 140.0f);
    playOneShot(m_assets.flareLaunch,
                kFlareLaunchPreset,
                position,
                velocity,
                std::clamp(0.34f + (heatFraction * 0.18f) + (speedFraction * 0.10f), 0.0f, 0.68f),
                0.96f + (speedFraction * 0.10f) + (heatFraction * 0.04f),
                safeNormalize(-velocity, glm::vec3(0.0f, 1.0f, 0.0f)));
}

void AudioSystem::Impl::playExplosion(const glm::vec3 &position, const glm::vec3 &velocity, float intensity)
{
    if (!m_enabled || m_assets.explosion.empty())
    {
        return;
    }

    const float clampedIntensity = std::clamp(intensity, 0.5f, 2.0f);
    playOneShot(m_assets.explosion,
                kExplosionPreset,
                position,
                velocity,
                0.72f + (clampedIntensity * 0.20f),
                0.88f + (clampedIntensity * 0.05f),
                safeNormalize(velocity, glm::vec3(0.0f, 1.0f, 0.0f)));
}

void AudioSystem::Impl::retireFlare(const Flare *flare)
{
    if (!m_enabled || flare == nullptr)
    {
        return;
    }

    const auto found = m_flareLoops.find(flare);
    if (found == m_flareLoops.end() || !found->second)
    {
        return;
    }

    silenceAndStopLoopEmitter(found->second->burn);
    shutdownLoopEmitter(found->second->burn);
    m_flareLoops.erase(found);
}