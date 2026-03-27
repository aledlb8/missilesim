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

AudioSystem::Impl::~Impl()
{
    shutdown();
}

bool AudioSystem::Impl::initialize()
{
    if (m_enabled)
    {
        return true;
    }

    ma_engine_config engineConfig = ma_engine_config_init();
    engineConfig.listenerCount = 1;

    const ma_result result = ma_engine_init(&engineConfig, &m_engine);
    if (result != MA_SUCCESS)
    {
        std::cerr << "ERROR: Failed to initialize audio engine: " << result << std::endl;
        return false;
    }

    m_engineInitialized = true;
    m_enabled = true;

    const ma_result volumeResult = ma_engine_set_volume(&m_engine, kMasterMixVolume);
    if (volumeResult != MA_SUCCESS)
    {
        std::cerr << "WARNING: Failed to set audio master volume: " << volumeResult << std::endl;
    }

    m_assets.fireLaunch = resolveAssetPath("audio/fire_launch.wav");
    m_assets.flareLaunch = resolveAssetPath("audio/flare_launch.wav");
    m_assets.flareBurnLoop = resolveAssetPath("audio/flare_burn_loop.wav");
    m_assets.explosion = resolveAssetPath("audio/explosion.wav");
    m_assets.missileFlightLoop = resolveAssetPath("audio/missile_flight_loop.wav");
    m_assets.missileAfterburnerLoop = resolveAssetPath("audio/missile_afterburner_loop.wav");
    m_assets.jetAfterburnerLoop = resolveAssetPath("audio/jet_afterburner_loop.wav");
    m_assets.mawsMissileWarningLoop = resolveAssetPath("audio/maws_missile_warning_loop.wav");
    m_assets.seekerPowerOn = resolveAssetPath("audio/seeker_power_on.wav");
    m_assets.seekerSearchLoop = resolveAssetPath("audio/seeker_search_loop.wav");
    m_assets.seekerLockLoop = resolveAssetPath("audio/seeker_lock_loop.wav");
    m_assets.seekerLockAcquire = resolveAssetPath("audio/seeker_lock_acquire.wav");

    logMissingAssets();
    initializeLoopEmitter(m_missileAirframe, m_assets.missileFlightLoop, kMissileAirframePreset);
    initializeLoopEmitter(m_missileAfterburner, m_assets.missileAfterburnerLoop, kMissileAfterburnerPreset);
    initializeUiLoopEmitter(m_seekerSearch, m_assets.seekerSearchLoop);
    initializeUiLoopEmitter(m_seekerLock, m_assets.seekerLockLoop);
    initializeUiLoopEmitter(m_mawsMissileWarning, m_assets.mawsMissileWarningLoop);
    return true;
}

void AudioSystem::Impl::shutdown()
{
    if (!m_engineInitialized)
    {
        return;
    }

    for (auto &voice : m_oneShots)
    {
        shutdownOneShot(*voice);
    }
    m_oneShots.clear();

    for (auto &entry : m_targetLoops)
    {
        if (entry.second)
        {
            shutdownLoopEmitter(entry.second->afterburner);
        }
    }
    m_targetLoops.clear();

    for (auto &entry : m_flareLoops)
    {
        if (entry.second)
        {
            shutdownLoopEmitter(entry.second->burn);
        }
    }
    m_flareLoops.clear();

    shutdownLoopEmitter(m_missileAirframe);
    shutdownLoopEmitter(m_missileAfterburner);
    shutdownUiLoopEmitter(m_seekerSearch);
    shutdownUiLoopEmitter(m_seekerLock);
    shutdownUiLoopEmitter(m_mawsMissileWarning);

    ma_engine_uninit(&m_engine);
    m_engine = {};
    m_engineInitialized = false;
    m_enabled = false;
    m_listenerInitialized = false;
    m_lastSeekerPowered = false;
    m_lastSeekerLocked = false;
    m_lastMawsThreatActive = false;
}

void AudioSystem::Impl::logMissingAssets() const
{
    auto logIfMissing = [](const char *label, const std::filesystem::path &path)
    {
        if (path.empty())
        {
            std::cerr << "WARNING: Missing audio asset: " << label << std::endl;
        }
    };

    logIfMissing("audio/fire_launch.wav", m_assets.fireLaunch);
    logIfMissing("audio/flare_launch.wav", m_assets.flareLaunch);
    logIfMissing("audio/flare_burn_loop.wav", m_assets.flareBurnLoop);
    logIfMissing("audio/explosion.wav", m_assets.explosion);
    logIfMissing("audio/missile_flight_loop.wav", m_assets.missileFlightLoop);
    logIfMissing("audio/missile_afterburner_loop.wav", m_assets.missileAfterburnerLoop);
    logIfMissing("audio/jet_afterburner_loop.wav", m_assets.jetAfterburnerLoop);
    logIfMissing("audio/maws_missile_warning_loop.wav", m_assets.mawsMissileWarningLoop);
    logIfMissing("audio/seeker_power_on.wav", m_assets.seekerPowerOn);
    logIfMissing("audio/seeker_search_loop.wav", m_assets.seekerSearchLoop);
    logIfMissing("audio/seeker_lock_loop.wav", m_assets.seekerLockLoop);
    logIfMissing("audio/seeker_lock_acquire.wav", m_assets.seekerLockAcquire);
}