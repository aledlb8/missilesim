#include "AudioSystem.h"

#include "../objects/Flare.h"
#include "../objects/Missile.h"
#include "../objects/Target.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/gtx/norm.hpp>
#include <miniaudio.h>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

namespace
{
    struct SpatialSoundPreset
    {
        float minDistance = 1.0f;
        float maxDistance = 1000.0f;
        float rolloff = 1.0f;
        float directionalAttenuation = 0.0f;
        float dopplerFactor = 0.0f;
    };

    constexpr SpatialSoundPreset kLaunchPreset{10.0f, 2600.0f, 1.15f, 0.0f};
    constexpr SpatialSoundPreset kFlareLaunchPreset{6.0f, 1600.0f, 1.18f, 0.0f};
    constexpr SpatialSoundPreset kFlareBurnPreset{4.0f, 920.0f, 1.24f, 0.0f};
    constexpr SpatialSoundPreset kExplosionPreset{18.0f, 4200.0f, 1.05f, 0.0f};
    constexpr SpatialSoundPreset kMissileAirframePreset{16.0f, 3200.0f, 1.08f, 0.0f};
    constexpr SpatialSoundPreset kMissileAfterburnerPreset{20.0f, 3600.0f, 1.02f, 0.22f};
    constexpr SpatialSoundPreset kJetAfterburnerPreset{26.0f, 4200.0f, 0.95f, 0.18f};
    constexpr float kMasterMixVolume = 0.72f;
    constexpr float kLoopVoiceHeadroom = 0.72f;
    constexpr float kOneShotHeadroom = 0.84f;
    constexpr float kUiLoopHeadroom = 0.56f;
    constexpr float kUiOneShotHeadroom = 0.72f;

    float saturate(float value)
    {
        return std::clamp(value, 0.0f, 1.0f);
    }

    float applyHeadroom(float normalizedVolume, float headroom)
    {
        return saturate(normalizedVolume) * std::max(headroom, 0.0f);
    }

    glm::vec3 safeNormalize(const glm::vec3 &value, const glm::vec3 &fallback)
    {
        if (glm::length2(value) > 0.0001f)
        {
            return glm::normalize(value);
        }

        if (glm::length2(fallback) > 0.0001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 0.0f, 1.0f);
    }

    float smoothingFactor(float lambda, float deltaTime)
    {
        const float dt = std::max(deltaTime, 0.0f);
        return 1.0f - std::exp(-lambda * dt);
    }

    std::filesystem::path resolveAssetPath(const std::string &relativePath)
    {
        const std::filesystem::path requested(relativePath);
        const std::filesystem::path sourceAssetRoot(MISSILESIM_SOURCE_ASSET_DIR);

        const std::filesystem::path candidates[] = {
            requested,
            std::filesystem::current_path() / requested,
            std::filesystem::current_path() / "assets" / requested,
            sourceAssetRoot.empty() ? std::filesystem::path() : sourceAssetRoot / requested};

        for (const auto &candidate : candidates)
        {
            if (!candidate.empty() && std::filesystem::exists(candidate))
            {
                return candidate;
            }
        }

        return {};
    }
}

struct AudioSystem::Impl
{
    struct LoopEmitter
    {
        ma_sound sound{};
        bool initialized = false;
        float currentVolume = 0.0f;
        float targetVolume = 0.0f;
        float currentPitch = 1.0f;
        float targetPitch = 1.0f;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 direction{0.0f, 0.0f, -1.0f};
    };

    struct UiLoopEmitter
    {
        ma_sound sound{};
        bool initialized = false;
        float currentVolume = 0.0f;
        float targetVolume = 0.0f;
        float currentPitch = 1.0f;
        float targetPitch = 1.0f;
    };

    struct TargetLoopState
    {
        LoopEmitter afterburner;
        bool seenThisFrame = false;
        bool retireWhenSilent = false;
    };

    struct FlareLoopState
    {
        LoopEmitter burn;
        bool seenThisFrame = false;
        bool retireWhenSilent = false;
    };

    struct OneShotVoice
    {
        ma_sound sound{};
        bool initialized = false;
    };

    struct AssetPaths
    {
        std::filesystem::path fireLaunch;
        std::filesystem::path flareLaunch;
        std::filesystem::path flareBurnLoop;
        std::filesystem::path explosion;
        std::filesystem::path missileFlightLoop;
        std::filesystem::path missileAfterburnerLoop;
        std::filesystem::path jetAfterburnerLoop;
        std::filesystem::path mawsMissileWarningLoop;
        std::filesystem::path seekerPowerOn;
        std::filesystem::path seekerSearchLoop;
        std::filesystem::path seekerLockLoop;
        std::filesystem::path seekerLockAcquire;
    };

    ma_engine m_engine{};
    bool m_engineInitialized = false;
    bool m_enabled = false;
    AssetPaths m_assets;
    LoopEmitter m_missileAirframe;
    LoopEmitter m_missileAfterburner;
    UiLoopEmitter m_seekerSearch;
    UiLoopEmitter m_seekerLock;
    UiLoopEmitter m_mawsMissileWarning;
    std::unordered_map<const Target *, std::unique_ptr<TargetLoopState>> m_targetLoops;
    std::unordered_map<const Flare *, std::unique_ptr<FlareLoopState>> m_flareLoops;
    std::vector<std::unique_ptr<OneShotVoice>> m_oneShots;
    glm::vec3 m_lastListenerPosition{0.0f};
    bool m_listenerInitialized = false;
    bool m_lastSeekerPowered = false;
    bool m_lastSeekerLocked = false;
    bool m_lastMawsThreatActive = false;

    ~Impl()
    {
        shutdown();
    }

    bool initialize()
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

    void shutdown()
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

    void update(float deltaTime)
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

    void setListener(const glm::vec3 &position,
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

    void syncMissile(const Missile *missile, bool missileInFlight, float referenceFuel)
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

    void syncTargets(const std::vector<Target *> &activeTargets)
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

    void syncFlares(const std::vector<Flare *> &activeFlares)
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

    void syncCockpitCues(bool seekerPowered,
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

    void playLaunch(const glm::vec3 &position, const glm::vec3 &velocity)
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

    void playFlareLaunch(const glm::vec3 &position, const glm::vec3 &velocity, float heatSignature)
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

    void playExplosion(const glm::vec3 &position, const glm::vec3 &velocity, float intensity)
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

    void retireFlare(const Flare *flare)
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

    void stopMissileEmitters()
    {
        if (!m_enabled)
        {
            return;
        }

        silenceAndStopLoopEmitter(m_missileAirframe);
        silenceAndStopLoopEmitter(m_missileAfterburner);
    }

    void stopAllEmitters()
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

private:
    void logMissingAssets() const
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

    bool initializeLoopEmitter(LoopEmitter &emitter,
                               const std::filesystem::path &assetPath,
                               const SpatialSoundPreset &preset)
    {
        if (!m_engineInitialized || assetPath.empty())
        {
            return false;
        }

        if (emitter.initialized)
        {
            return true;
        }

        const std::wstring widePath = assetPath.wstring();
        const ma_result result = ma_sound_init_from_file_w(&m_engine, widePath.c_str(), 0, nullptr, nullptr, &emitter.sound);
        if (result != MA_SUCCESS)
        {
            std::cerr << "ERROR: Failed to load looping sound '" << assetPath.string() << "': " << result << std::endl;
            return false;
        }

        configureSpatialSound(emitter.sound, preset);
        ma_sound_set_looping(&emitter.sound, MA_TRUE);
        ma_sound_set_volume(&emitter.sound, 0.0f);
        ma_sound_set_pitch(&emitter.sound, 1.0f);

        emitter.initialized = true;
        emitter.currentVolume = 0.0f;
        emitter.targetVolume = 0.0f;
        emitter.currentPitch = 1.0f;
        emitter.targetPitch = 1.0f;
        return true;
    }

    bool initializeUiLoopEmitter(UiLoopEmitter &emitter, const std::filesystem::path &assetPath)
    {
        if (!m_engineInitialized || assetPath.empty())
        {
            return false;
        }

        if (emitter.initialized)
        {
            return true;
        }

        const std::wstring widePath = assetPath.wstring();
        const ma_result result = ma_sound_init_from_file_w(&m_engine,
                                                           widePath.c_str(),
                                                           MA_SOUND_FLAG_NO_SPATIALIZATION,
                                                           nullptr,
                                                           nullptr,
                                                           &emitter.sound);
        if (result != MA_SUCCESS)
        {
            std::cerr << "ERROR: Failed to load interface loop '" << assetPath.string() << "': " << result << std::endl;
            return false;
        }

        configureInterfaceSound(emitter.sound);
        ma_sound_set_looping(&emitter.sound, MA_TRUE);
        ma_sound_set_volume(&emitter.sound, 0.0f);
        ma_sound_set_pitch(&emitter.sound, 1.0f);

        emitter.initialized = true;
        emitter.currentVolume = 0.0f;
        emitter.targetVolume = 0.0f;
        emitter.currentPitch = 1.0f;
        emitter.targetPitch = 1.0f;
        return true;
    }

    void shutdownLoopEmitter(LoopEmitter &emitter)
    {
        if (!emitter.initialized)
        {
            return;
        }

        ma_sound_uninit(&emitter.sound);
        emitter = {};
    }

    void shutdownUiLoopEmitter(UiLoopEmitter &emitter)
    {
        if (!emitter.initialized)
        {
            return;
        }

        ma_sound_uninit(&emitter.sound);
        emitter = {};
    }

    void silenceAndStopLoopEmitter(LoopEmitter &emitter)
    {
        if (!emitter.initialized)
        {
            return;
        }

        emitter.currentVolume = 0.0f;
        emitter.targetVolume = 0.0f;
        ma_sound_set_volume(&emitter.sound, 0.0f);
        ma_sound_stop(&emitter.sound);
        ma_sound_seek_to_pcm_frame(&emitter.sound, 0);
    }

    void silenceAndStopUiLoopEmitter(UiLoopEmitter &emitter)
    {
        if (!emitter.initialized)
        {
            return;
        }

        emitter.currentVolume = 0.0f;
        emitter.targetVolume = 0.0f;
        ma_sound_set_volume(&emitter.sound, 0.0f);
        ma_sound_stop(&emitter.sound);
        ma_sound_seek_to_pcm_frame(&emitter.sound, 0);
    }

    void updateLoopEmitter(LoopEmitter &emitter, float deltaTime)
    {
        if (!emitter.initialized)
        {
            return;
        }

        const float dt = (deltaTime > 0.0f) ? deltaTime : 0.016f;
        const float volumeBlend = smoothingFactor(10.0f, dt);
        const float pitchBlend = smoothingFactor(8.0f, dt);

        emitter.currentVolume += (emitter.targetVolume - emitter.currentVolume) * volumeBlend;
        emitter.currentPitch += (emitter.targetPitch - emitter.currentPitch) * pitchBlend;

        ma_sound_set_position(&emitter.sound, emitter.position.x, emitter.position.y, emitter.position.z);
        ma_sound_set_velocity(&emitter.sound, emitter.velocity.x, emitter.velocity.y, emitter.velocity.z);
        ma_sound_set_direction(&emitter.sound, emitter.direction.x, emitter.direction.y, emitter.direction.z);
        ma_sound_set_volume(&emitter.sound, applyHeadroom(emitter.currentVolume, kLoopVoiceHeadroom));
        ma_sound_set_pitch(&emitter.sound, std::max(emitter.currentPitch, 0.05f));

        if (emitter.targetVolume > 0.002f && !ma_sound_is_playing(&emitter.sound))
        {
            ma_sound_seek_to_pcm_frame(&emitter.sound, 0);
            ma_sound_start(&emitter.sound);
        }
        else if (emitter.targetVolume <= 0.001f &&
                 emitter.currentVolume <= 0.001f &&
                 ma_sound_is_playing(&emitter.sound))
        {
            ma_sound_stop(&emitter.sound);
            ma_sound_seek_to_pcm_frame(&emitter.sound, 0);
        }
    }

    void updateUiLoopEmitter(UiLoopEmitter &emitter, float deltaTime)
    {
        if (!emitter.initialized)
        {
            return;
        }

        const float dt = (deltaTime > 0.0f) ? deltaTime : 0.016f;
        const float volumeBlend = smoothingFactor(12.0f, dt);
        const float pitchBlend = smoothingFactor(10.0f, dt);

        emitter.currentVolume += (emitter.targetVolume - emitter.currentVolume) * volumeBlend;
        emitter.currentPitch += (emitter.targetPitch - emitter.currentPitch) * pitchBlend;

        ma_sound_set_volume(&emitter.sound, applyHeadroom(emitter.currentVolume, kUiLoopHeadroom));
        ma_sound_set_pitch(&emitter.sound, std::max(emitter.currentPitch, 0.05f));

        if (emitter.targetVolume > 0.002f && !ma_sound_is_playing(&emitter.sound))
        {
            ma_sound_seek_to_pcm_frame(&emitter.sound, 0);
            ma_sound_start(&emitter.sound);
        }
        else if (emitter.targetVolume <= 0.001f &&
                 emitter.currentVolume <= 0.001f &&
                 ma_sound_is_playing(&emitter.sound))
        {
            ma_sound_stop(&emitter.sound);
            ma_sound_seek_to_pcm_frame(&emitter.sound, 0);
        }
    }

    TargetLoopState *getOrCreateTargetState(const Target *target)
    {
        auto found = m_targetLoops.find(target);
        if (found != m_targetLoops.end())
        {
            return found->second.get();
        }

        if (m_assets.jetAfterburnerLoop.empty())
        {
            return nullptr;
        }

        auto state = std::make_unique<TargetLoopState>();
        if (!initializeLoopEmitter(state->afterburner, m_assets.jetAfterburnerLoop, kJetAfterburnerPreset))
        {
            return nullptr;
        }

        TargetLoopState *statePtr = state.get();
        m_targetLoops.emplace(target, std::move(state));
        return statePtr;
    }

    FlareLoopState *getOrCreateFlareState(const Flare *flare)
    {
        auto found = m_flareLoops.find(flare);
        if (found != m_flareLoops.end())
        {
            return found->second.get();
        }

        if (m_assets.flareBurnLoop.empty())
        {
            return nullptr;
        }

        auto state = std::make_unique<FlareLoopState>();
        if (!initializeLoopEmitter(state->burn, m_assets.flareBurnLoop, kFlareBurnPreset))
        {
            return nullptr;
        }

        FlareLoopState *statePtr = state.get();
        m_flareLoops.emplace(flare, std::move(state));
        return statePtr;
    }

    void playOneShot(const std::filesystem::path &assetPath,
                     const SpatialSoundPreset &preset,
                     const glm::vec3 &position,
                     const glm::vec3 &velocity,
                     float volume,
                     float pitch,
                     const glm::vec3 &direction)
    {
        if (!m_engineInitialized || assetPath.empty())
        {
            return;
        }

        auto voice = std::make_unique<OneShotVoice>();
        const std::wstring widePath = assetPath.wstring();
        const ma_result result = ma_sound_init_from_file_w(&m_engine, widePath.c_str(), 0, nullptr, nullptr, &voice->sound);
        if (result != MA_SUCCESS)
        {
            std::cerr << "ERROR: Failed to load one-shot sound '" << assetPath.string() << "': " << result << std::endl;
            return;
        }

        voice->initialized = true;
        configureSpatialSound(voice->sound, preset);
        ma_sound_set_position(&voice->sound, position.x, position.y, position.z);
        ma_sound_set_velocity(&voice->sound, velocity.x, velocity.y, velocity.z);
        ma_sound_set_direction(&voice->sound, direction.x, direction.y, direction.z);
        ma_sound_set_volume(&voice->sound, applyHeadroom(volume, kOneShotHeadroom));
        ma_sound_set_pitch(&voice->sound, std::max(pitch, 0.05f));

        const ma_result startResult = ma_sound_start(&voice->sound);
        if (startResult != MA_SUCCESS)
        {
            std::cerr << "ERROR: Failed to start one-shot sound '" << assetPath.string() << "': " << startResult << std::endl;
            shutdownOneShot(*voice);
            return;
        }

        m_oneShots.push_back(std::move(voice));
    }

    void playUiOneShot(const std::filesystem::path &assetPath, float volume, float pitch)
    {
        if (!m_engineInitialized || assetPath.empty())
        {
            return;
        }

        auto voice = std::make_unique<OneShotVoice>();
        const std::wstring widePath = assetPath.wstring();
        const ma_result result = ma_sound_init_from_file_w(&m_engine,
                                                           widePath.c_str(),
                                                           MA_SOUND_FLAG_NO_SPATIALIZATION,
                                                           nullptr,
                                                           nullptr,
                                                           &voice->sound);
        if (result != MA_SUCCESS)
        {
            std::cerr << "ERROR: Failed to load interface one-shot '" << assetPath.string() << "': " << result << std::endl;
            return;
        }

        voice->initialized = true;
        configureInterfaceSound(voice->sound);
        ma_sound_set_volume(&voice->sound, applyHeadroom(volume, kUiOneShotHeadroom));
        ma_sound_set_pitch(&voice->sound, std::max(pitch, 0.05f));

        const ma_result startResult = ma_sound_start(&voice->sound);
        if (startResult != MA_SUCCESS)
        {
            std::cerr << "ERROR: Failed to start interface one-shot '" << assetPath.string() << "': " << startResult << std::endl;
            shutdownOneShot(*voice);
            return;
        }

        m_oneShots.push_back(std::move(voice));
    }

    void pruneOneShots()
    {
        auto it = m_oneShots.begin();
        while (it != m_oneShots.end())
        {
            OneShotVoice &voice = *(*it);
            if (!voice.initialized ||
                (ma_sound_at_end(&voice.sound) && !ma_sound_is_playing(&voice.sound)))
            {
                shutdownOneShot(voice);
                it = m_oneShots.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void shutdownOneShot(OneShotVoice &voice)
    {
        if (!voice.initialized)
        {
            return;
        }

        ma_sound_uninit(&voice.sound);
        voice = {};
    }

    void configureSpatialSound(ma_sound &sound, const SpatialSoundPreset &preset)
    {
        ma_sound_set_spatialization_enabled(&sound, MA_TRUE);
        ma_sound_set_positioning(&sound, ma_positioning_absolute);
        ma_sound_set_attenuation_model(&sound, ma_attenuation_model_inverse);
        ma_sound_set_rolloff(&sound, preset.rolloff);
        ma_sound_set_min_distance(&sound, preset.minDistance);
        ma_sound_set_max_distance(&sound, preset.maxDistance);
        ma_sound_set_directional_attenuation_factor(&sound, preset.directionalAttenuation);
        // MissileSim world velocities are high enough to produce unstable pitch warble, so
        // we keep spatialization but disable Doppler on these synthetic emitters.
        ma_sound_set_doppler_factor(&sound, preset.dopplerFactor);
    }

    void configureInterfaceSound(ma_sound &sound)
    {
        ma_sound_set_spatialization_enabled(&sound, MA_FALSE);
        ma_sound_set_positioning(&sound, ma_positioning_absolute);
        ma_sound_set_pan(&sound, 0.0f);
    }
};

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
