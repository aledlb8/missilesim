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

bool AudioSystem::Impl::initializeLoopEmitter(LoopEmitter &emitter,
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

bool AudioSystem::Impl::initializeUiLoopEmitter(UiLoopEmitter &emitter, const std::filesystem::path &assetPath)
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

void AudioSystem::Impl::shutdownLoopEmitter(LoopEmitter &emitter)
{
    if (!emitter.initialized)
    {
        return;
    }

    ma_sound_uninit(&emitter.sound);
    emitter = {};
}

void AudioSystem::Impl::shutdownUiLoopEmitter(UiLoopEmitter &emitter)
{
    if (!emitter.initialized)
    {
        return;
    }

    ma_sound_uninit(&emitter.sound);
    emitter = {};
}

void AudioSystem::Impl::silenceAndStopLoopEmitter(LoopEmitter &emitter)
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

void AudioSystem::Impl::silenceAndStopUiLoopEmitter(UiLoopEmitter &emitter)
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

void AudioSystem::Impl::updateLoopEmitter(LoopEmitter &emitter, float deltaTime)
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

void AudioSystem::Impl::updateUiLoopEmitter(UiLoopEmitter &emitter, float deltaTime)
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

AudioSystem::Impl::TargetLoopState *AudioSystem::Impl::getOrCreateTargetState(const Target *target)
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

AudioSystem::Impl::FlareLoopState *AudioSystem::Impl::getOrCreateFlareState(const Flare *flare)
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

void AudioSystem::Impl::playOneShot(const std::filesystem::path &assetPath,
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

void AudioSystem::Impl::playUiOneShot(const std::filesystem::path &assetPath, float volume, float pitch)
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

void AudioSystem::Impl::pruneOneShots()
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

void AudioSystem::Impl::shutdownOneShot(OneShotVoice &voice)
{
    if (!voice.initialized)
    {
        return;
    }

    ma_sound_uninit(&voice.sound);
    voice = {};
}

void AudioSystem::Impl::configureSpatialSound(ma_sound &sound, const SpatialSoundPreset &preset)
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

void AudioSystem::Impl::configureInterfaceSound(ma_sound &sound)
{
    ma_sound_set_spatialization_enabled(&sound, MA_FALSE);
    ma_sound_set_positioning(&sound, ma_positioning_absolute);
    ma_sound_set_pan(&sound, 0.0f);
}