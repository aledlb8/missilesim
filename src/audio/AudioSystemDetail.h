#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

namespace missilesim::audio::detail
{
    struct SpatialSoundPreset
    {
        float minDistance = 1.0f;
        float maxDistance = 1000.0f;
        float rolloff = 1.0f;
        float directionalAttenuation = 0.0f;
        float dopplerFactor = 0.0f;
    };

    inline constexpr SpatialSoundPreset kLaunchPreset{10.0f, 2600.0f, 1.15f, 0.0f};
    inline constexpr SpatialSoundPreset kFlareLaunchPreset{6.0f, 1600.0f, 1.18f, 0.0f};
    inline constexpr SpatialSoundPreset kFlareBurnPreset{4.0f, 920.0f, 1.24f, 0.0f};
    inline constexpr SpatialSoundPreset kExplosionPreset{18.0f, 4200.0f, 1.05f, 0.0f};
    inline constexpr SpatialSoundPreset kMissileAirframePreset{16.0f, 3200.0f, 1.08f, 0.0f};
    inline constexpr SpatialSoundPreset kMissileAfterburnerPreset{20.0f, 3600.0f, 1.02f, 0.22f};
    inline constexpr SpatialSoundPreset kJetAfterburnerPreset{26.0f, 4200.0f, 0.95f, 0.18f};
    inline constexpr float kMasterMixVolume = 0.72f;
    inline constexpr float kLoopVoiceHeadroom = 0.72f;
    inline constexpr float kOneShotHeadroom = 0.84f;
    inline constexpr float kUiLoopHeadroom = 0.56f;
    inline constexpr float kUiOneShotHeadroom = 0.72f;

    inline float saturate(float value)
    {
        return std::clamp(value, 0.0f, 1.0f);
    }

    inline float applyHeadroom(float normalizedVolume, float headroom)
    {
        return saturate(normalizedVolume) * std::max(headroom, 0.0f);
    }

    inline glm::vec3 safeNormalize(const glm::vec3 &value, const glm::vec3 &fallback)
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

    inline float smoothingFactor(float lambda, float deltaTime)
    {
        const float dt = std::max(deltaTime, 0.0f);
        return 1.0f - std::exp(-lambda * dt);
    }

    inline std::filesystem::path resolveAssetPath(const std::string &relativePath)
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