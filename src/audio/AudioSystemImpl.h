#pragma once

#include "AudioSystem.h"
#include "AudioSystemDetail.h"

#include "../objects/Flare.h"
#include "../objects/Missile.h"
#include "../objects/Target.h"

#include <filesystem>
#include <memory>
#include <unordered_map>
#include <vector>

#include <miniaudio.h>

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

    ~Impl();

    bool initialize();
    void shutdown();
    void update(float deltaTime);
    void setListener(const glm::vec3 &position,
                     const glm::vec3 &forward,
                     const glm::vec3 &up,
                     float deltaTime);
    void syncMissile(const Missile *missile, bool missileInFlight, float referenceFuel);
    void syncTargets(const std::vector<Target *> &activeTargets);
    void syncFlares(const std::vector<Flare *> &activeFlares);
    void syncCockpitCues(bool seekerPowered,
                         bool seekerLocked,
                         float seekerSignalStrength,
                         bool mawsThreatActive,
                         float mawsUrgency);
    void playLaunch(const glm::vec3 &position, const glm::vec3 &velocity);
    void playFlareLaunch(const glm::vec3 &position, const glm::vec3 &velocity, float heatSignature);
    void playExplosion(const glm::vec3 &position, const glm::vec3 &velocity, float intensity);
    void retireFlare(const Flare *flare);
    void stopMissileEmitters();
    void stopAllEmitters();

private:
    void logMissingAssets() const;
    bool initializeLoopEmitter(LoopEmitter &emitter,
                               const std::filesystem::path &assetPath,
                               const missilesim::audio::detail::SpatialSoundPreset &preset);
    bool initializeUiLoopEmitter(UiLoopEmitter &emitter, const std::filesystem::path &assetPath);
    void shutdownLoopEmitter(LoopEmitter &emitter);
    void shutdownUiLoopEmitter(UiLoopEmitter &emitter);
    void silenceAndStopLoopEmitter(LoopEmitter &emitter);
    void silenceAndStopUiLoopEmitter(UiLoopEmitter &emitter);
    void updateLoopEmitter(LoopEmitter &emitter, float deltaTime);
    void updateUiLoopEmitter(UiLoopEmitter &emitter, float deltaTime);
    TargetLoopState *getOrCreateTargetState(const Target *target);
    FlareLoopState *getOrCreateFlareState(const Flare *flare);
    void playOneShot(const std::filesystem::path &assetPath,
                     const missilesim::audio::detail::SpatialSoundPreset &preset,
                     const glm::vec3 &position,
                     const glm::vec3 &velocity,
                     float volume,
                     float pitch,
                     const glm::vec3 &direction);
    void playUiOneShot(const std::filesystem::path &assetPath, float volume, float pitch);
    void pruneOneShots();
    void shutdownOneShot(OneShotVoice &voice);
    void configureSpatialSound(ma_sound &sound, const missilesim::audio::detail::SpatialSoundPreset &preset);
    void configureInterfaceSound(ma_sound &sound);
};