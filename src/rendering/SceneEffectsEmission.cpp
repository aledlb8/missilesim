#include "SceneEffects.h"
#include "SceneEffectsDetail.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

using missilesim::rendering::detail::kMaxHeatHazeSprites;
using missilesim::rendering::detail::kMaxParticles;
using missilesim::rendering::detail::perpendicularTo;
using missilesim::rendering::detail::safeNormalize;
using missilesim::rendering::detail::saturate;

void SceneEffects::emitMissileExhaust(const glm::vec3 &start,
                                      const glm::vec3 &end,
                                      const glm::vec3 &forward,
                                      const glm::vec3 &carrierVelocity,
                                      float intensity)
{
    emitEngineTrail(start, end, forward, carrierVelocity, glm::clamp(intensity, 0.5f, 1.0f), true);
}

void SceneEffects::emitJetAfterburner(const glm::vec3 &start,
                                      const glm::vec3 &end,
                                      const glm::vec3 &forward,
                                      const glm::vec3 &carrierVelocity,
                                      float intensity)
{
    emitEngineTrail(start, end, forward, carrierVelocity, glm::clamp(intensity, 0.35f, 1.1f), false);
}

void SceneEffects::emitFlareEffect(const glm::vec3 &start,
                                   const glm::vec3 &end,
                                   const glm::vec3 &carrierVelocity,
                                   float heatFraction)
{
    const float intensity = glm::clamp(heatFraction, 0.0f, 1.0f);
    if (intensity <= 0.01f)
    {
        return;
    }

    const glm::vec3 direction = safeNormalize(-carrierVelocity, glm::vec3(0.0f, -1.0f, 0.0f));
    const float segmentLength = glm::length(end - start);
    const int sampleCount = std::max(1, static_cast<int>(std::ceil(segmentLength / 1.5f)));

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        const float interpolation = (sampleCount == 1) ? 1.0f : static_cast<float>(sampleIndex) / static_cast<float>(sampleCount - 1);
        const glm::vec3 center = glm::mix(start, end, interpolation);

        EffectParticle glow{};
        glow.position = center + (randomInUnitSphere() * 0.2f);
        glow.velocity = carrierVelocity * 0.15f;
        glow.axis = direction;
        glow.color = glm::vec4(glm::mix(glm::vec3(1.0f, 0.55f, 0.18f), glm::vec3(1.0f, 0.96f, 0.76f), intensity), 1.0f);
        glow.lifetime = randomRange(0.12f, 0.22f);
        glow.startSize = 0.45f;
        glow.endSize = 1.7f + intensity * 1.1f;
        glow.stretch = 1.2f;
        glow.softness = 1.1f;
        glow.emissive = 1.35f;
        glow.seed = randomRange(0.0f, 1000.0f);
        glow.material = ParticleMaterial::GLOW;
        glow.blendMode = BlendMode::ADDITIVE;
        addParticle(glow);

        EffectParticle smoke{};
        smoke.position = center + (direction * 0.25f) + (randomInUnitSphere() * 0.35f);
        smoke.velocity = (carrierVelocity * 0.35f) + (direction * randomRange(6.0f, 14.0f)) + (randomInUnitSphere() * 5.0f);
        smoke.axis = safeNormalize(smoke.velocity, direction);
        smoke.color = glm::vec4(glm::mix(glm::vec3(0.22f, 0.20f, 0.20f), glm::vec3(0.42f, 0.34f, 0.28f), intensity), 0.6f);
        smoke.lifetime = randomRange(0.8f, 1.35f);
        smoke.startSize = 0.35f;
        smoke.endSize = 2.6f;
        smoke.stretch = 1.1f;
        smoke.softness = 0.8f;
        smoke.emissive = 1.0f;
        smoke.seed = randomRange(0.0f, 1000.0f);
        smoke.drag = 0.35f;
        smoke.upwardAcceleration = 2.4f;
        smoke.material = ParticleMaterial::SMOKE;
        smoke.blendMode = BlendMode::ALPHA;
        addParticle(smoke);
    }

    HeatHazeSprite haze{};
    haze.position = end;
    haze.velocity = carrierVelocity * 0.2f;
    haze.axis = direction;
    haze.lifetime = 0.12f + intensity * 0.08f;
    haze.radius = 1.25f + intensity * 1.1f;
    haze.stretch = 1.4f;
    haze.rotation = randomRange(0.0f, 6.28318f);
    haze.angularVelocity = randomRange(-1.8f, 1.8f);
    haze.strength = 3.2f + intensity * 2.1f;
    haze.seed = randomRange(0.0f, 1000.0f);
    haze.drag = 1.2f;
    addHeatHaze(haze);
}

void SceneEffects::spawnExplosion(const glm::vec3 &position, const glm::vec3 &velocityHint, float intensity)
{
    const float clampedIntensity = glm::clamp(intensity, 0.5f, 2.0f);
    const glm::vec3 forwardBias = safeNormalize(velocityHint, glm::vec3(0.0f, 1.0f, 0.0f));
    const int flameCount = std::clamp(static_cast<int>(std::round(34.0f * clampedIntensity)), 28, 72);
    const int smokeCount = std::clamp(static_cast<int>(std::round(26.0f * clampedIntensity)), 22, 56);
    const int sparkCount = std::clamp(static_cast<int>(std::round(22.0f * clampedIntensity)), 18, 52);
    const int hazeCount = std::clamp(static_cast<int>(std::round(8.0f * clampedIntensity)), 6, 16);

    EffectParticle flash{};
    flash.position = position;
    flash.velocity = velocityHint * 0.08f;
    flash.axis = forwardBias;
    flash.color = glm::vec4(1.0f, 0.82f, 0.42f, 1.0f);
    flash.lifetime = 0.18f;
    flash.startSize = 2.8f * clampedIntensity;
    flash.endSize = 9.5f * clampedIntensity;
    flash.stretch = 1.0f;
    flash.softness = 1.25f;
    flash.emissive = 1.8f;
    flash.seed = randomRange(0.0f, 1000.0f);
    flash.material = ParticleMaterial::GLOW;
    flash.blendMode = BlendMode::ADDITIVE;
    addParticle(flash);

    for (int index = 0; index < flameCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();
        direction = safeNormalize(glm::mix(direction, forwardBias, 0.18f), direction);

        EffectParticle flame{};
        flame.position = position + (direction * randomRange(0.2f, 1.1f) * clampedIntensity);
        flame.velocity = (direction * randomRange(18.0f, 48.0f) * clampedIntensity) + (velocityHint * 0.2f);
        flame.axis = direction;
        flame.color = glm::vec4(glm::mix(glm::vec3(1.0f, 0.42f, 0.12f), glm::vec3(1.0f, 0.90f, 0.62f), randomRange(0.15f, 0.65f)), 1.0f);
        flame.lifetime = randomRange(0.32f, 0.55f);
        flame.startSize = randomRange(0.7f, 1.2f) * clampedIntensity;
        flame.endSize = randomRange(3.4f, 5.8f) * clampedIntensity;
        flame.stretch = randomRange(1.8f, 3.6f);
        flame.rotation = randomRange(0.0f, 6.28318f);
        flame.angularVelocity = randomRange(-5.0f, 5.0f);
        flame.softness = 1.0f;
        flame.emissive = randomRange(1.0f, 1.5f);
        flame.seed = randomRange(0.0f, 1000.0f);
        flame.drag = 1.8f;
        flame.material = ParticleMaterial::FLAME;
        flame.blendMode = BlendMode::ADDITIVE;
        addParticle(flame);
    }

    for (int index = 0; index < smokeCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();

        EffectParticle smoke{};
        smoke.position = position + (direction * randomRange(0.5f, 2.0f) * clampedIntensity);
        smoke.velocity = (direction * randomRange(8.0f, 22.0f) * clampedIntensity) + (velocityHint * 0.12f);
        smoke.axis = direction;
        smoke.color = glm::vec4(glm::vec3(randomRange(0.18f, 0.34f)), 0.72f);
        smoke.lifetime = randomRange(1.1f, 2.2f);
        smoke.startSize = randomRange(0.8f, 1.4f) * clampedIntensity;
        smoke.endSize = randomRange(5.0f, 8.5f) * clampedIntensity;
        smoke.stretch = randomRange(1.0f, 1.6f);
        smoke.rotation = randomRange(0.0f, 6.28318f);
        smoke.angularVelocity = randomRange(-1.8f, 1.8f);
        smoke.softness = 0.9f;
        smoke.emissive = 1.0f;
        smoke.seed = randomRange(0.0f, 1000.0f);
        smoke.drag = 0.28f;
        smoke.upwardAcceleration = randomRange(4.0f, 8.0f);
        smoke.material = ParticleMaterial::SMOKE;
        smoke.blendMode = BlendMode::ALPHA;
        addParticle(smoke);
    }

    for (int index = 0; index < sparkCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();

        EffectParticle spark{};
        spark.position = position;
        spark.velocity = (direction * randomRange(35.0f, 75.0f) * clampedIntensity) + (velocityHint * 0.25f);
        spark.axis = direction;
        spark.color = glm::vec4(glm::vec3(1.0f, randomRange(0.72f, 0.92f), 0.38f), 1.0f);
        spark.lifetime = randomRange(0.18f, 0.34f);
        spark.startSize = randomRange(0.16f, 0.24f) * clampedIntensity;
        spark.endSize = randomRange(1.5f, 2.8f) * clampedIntensity;
        spark.stretch = randomRange(3.6f, 6.5f);
        spark.softness = 1.0f;
        spark.emissive = 1.3f;
        spark.seed = randomRange(0.0f, 1000.0f);
        spark.drag = 3.0f;
        spark.material = ParticleMaterial::SPARK;
        spark.blendMode = BlendMode::ADDITIVE;
        addParticle(spark);
    }

    for (int index = 0; index < hazeCount; ++index)
    {
        HeatHazeSprite haze{};
        haze.position = position + (randomInUnitSphere() * 0.8f * clampedIntensity);
        haze.velocity = randomInUnitSphere() * randomRange(4.0f, 11.0f) + (velocityHint * 0.08f);
        haze.axis = safeNormalize(haze.velocity, forwardBias);
        haze.lifetime = randomRange(0.18f, 0.34f);
        haze.radius = randomRange(3.5f, 6.8f) * clampedIntensity;
        haze.stretch = randomRange(1.0f, 1.4f);
        haze.rotation = randomRange(0.0f, 6.28318f);
        haze.angularVelocity = randomRange(-2.4f, 2.4f);
        haze.strength = randomRange(3.5f, 7.0f) * clampedIntensity;
        haze.seed = randomRange(0.0f, 1000.0f);
        haze.drag = 0.8f;
        addHeatHaze(haze);
    }

    for (int index = 0; index < 3; ++index)
    {
        HeatHazeSprite shockwave{};
        shockwave.position = position + (forwardBias * randomRange(0.2f, 1.2f) * clampedIntensity);
        shockwave.velocity = (randomUnitVector() * randomRange(10.0f, 24.0f) * clampedIntensity) + (velocityHint * 0.12f);
        shockwave.axis = safeNormalize(shockwave.velocity, forwardBias);
        shockwave.lifetime = randomRange(0.20f, 0.32f);
        shockwave.radius = randomRange(5.8f, 9.4f) * clampedIntensity;
        shockwave.stretch = randomRange(1.6f, 2.4f);
        shockwave.rotation = randomRange(0.0f, 6.28318f);
        shockwave.angularVelocity = randomRange(-1.2f, 1.2f);
        shockwave.strength = randomRange(5.6f, 8.8f) * clampedIntensity;
        shockwave.seed = randomRange(0.0f, 1000.0f);
        shockwave.drag = 1.1f;
        addHeatHaze(shockwave);
    }
}

void SceneEffects::addParticle(const EffectParticle &particle)
{
    if (m_particles.size() >= kMaxParticles)
    {
        return;
    }

    m_particles.push_back(particle);
}

void SceneEffects::addHeatHaze(const HeatHazeSprite &sprite)
{
    if (m_heatHazeSprites.size() >= kMaxHeatHazeSprites)
    {
        return;
    }

    m_heatHazeSprites.push_back(sprite);
}

void SceneEffects::emitEngineTrail(const glm::vec3 &start,
                                   const glm::vec3 &end,
                                   const glm::vec3 &forward,
                                   const glm::vec3 &carrierVelocity,
                                   float intensity,
                                   bool missilePreset)
{
    const glm::vec3 exhaustDirection = safeNormalize(-forward, glm::vec3(0.0f, -1.0f, 0.0f));
    const glm::vec3 lateralDirection = perpendicularTo(exhaustDirection);
    const glm::vec3 verticalDirection = safeNormalize(glm::cross(exhaustDirection, lateralDirection), glm::vec3(0.0f, 1.0f, 0.0f));
    const float sweepLength = glm::length(end - start);
    const auto computeSweepCount = [sweepLength](float spacing, int maxCount)
    {
        if (sweepLength <= spacing * 0.35f)
        {
            return 1;
        }

        return std::clamp(static_cast<int>(std::ceil(sweepLength / spacing)) + 1, 2, maxCount);
    };

    // Fill the swept nozzle path so fast movers keep a continuous plume between frames.
    const int plumeSweepCount = computeSweepCount(missilePreset ? 0.45f : 0.60f,
                                                  missilePreset ? 24 : 16);
    const int smokeSweepCount = computeSweepCount(missilePreset ? 1.05f : 1.35f,
                                                  missilePreset ? 8 : 6);
    const int hazeSweepCount = computeSweepCount(missilePreset ? 0.85f : 1.10f,
                                                 missilePreset ? 7 : 5);
    const glm::vec3 advectedVelocity = carrierVelocity * (missilePreset ? 0.07f : 0.05f);
    const glm::vec3 smokeAdvectedVelocity = carrierVelocity * (missilePreset ? 0.14f : 0.08f);
    const glm::vec3 hazeAdvectedVelocity = carrierVelocity * (missilePreset ? 0.08f : 0.06f);
    const int plumeLayerCount = missilePreset ? 2 : 3;
    const float plumeClusterLength = missilePreset ? 0.09f : 0.07f;
    const float smokeClusterLength = missilePreset ? 0.26f : 0.20f;
    const float hazeClusterLength = missilePreset ? 0.16f : 0.12f;

    const glm::vec3 flameColor = missilePreset
                                     ? glm::vec3(1.0f, 0.68f, 0.24f)
                                     : glm::vec3(0.40f, 0.72f, 1.0f);
    const glm::vec3 flameHighlight = missilePreset
                                         ? glm::vec3(1.0f, 0.95f, 0.82f)
                                         : glm::vec3(0.86f, 0.94f, 1.0f);

    for (int sweepIndex = 0; sweepIndex < plumeSweepCount; ++sweepIndex)
    {
        const float sweepInterpolation = (plumeSweepCount == 1) ? 1.0f : static_cast<float>(sweepIndex) / static_cast<float>(plumeSweepCount - 1);
        const float sampleIntensity = intensity * glm::mix(0.84f, 1.0f, sweepInterpolation);
        const float sampleScale = glm::mix(0.90f, 1.0f, sweepInterpolation);
        const glm::vec3 trailEnd = glm::mix(start, end, sweepInterpolation);

        for (int sampleIndex = 0; sampleIndex < plumeLayerCount; ++sampleIndex)
        {
            const float interpolation = (plumeLayerCount == 1) ? 0.0f : static_cast<float>(sampleIndex) / static_cast<float>(plumeLayerCount - 1);
            const glm::vec3 center = trailEnd +
                                     (exhaustDirection * (glm::mix(0.0f, plumeClusterLength, interpolation) +
                                                          randomRange(0.0f, plumeClusterLength * 0.15f)));
            const float jitterSpan = missilePreset ? 0.018f : 0.024f;
            const glm::vec3 nozzleJitter = (lateralDirection * randomRange(-jitterSpan, jitterSpan) * sampleIntensity) +
                                           (verticalDirection * randomRange(-jitterSpan, jitterSpan) * sampleIntensity);

            EffectParticle core{};
            core.position = center + nozzleJitter;
            core.velocity = advectedVelocity +
                            (exhaustDirection * randomRange(missilePreset ? 12.0f : 8.0f, missilePreset ? 22.0f : 14.0f) *
                             sampleIntensity);
            core.axis = exhaustDirection;
            core.color = glm::vec4(flameHighlight, 1.0f);
            core.lifetime = missilePreset ? randomRange(0.08f, 0.12f) : randomRange(0.06f, 0.09f);
            core.startSize = (missilePreset ? 0.09f : 0.11f) * sampleScale;
            core.endSize = missilePreset ? 0.24f * sampleIntensity : 0.30f * sampleIntensity;
            core.stretch = missilePreset ? 1.35f : 1.28f;
            core.softness = 1.0f;
            core.emissive = (missilePreset ? 1.1f : 1.0f) * sampleScale;
            core.seed = randomRange(0.0f, 1000.0f);
            core.drag = 3.0f;
            core.material = ParticleMaterial::GLOW;
            core.blendMode = BlendMode::ADDITIVE;
            addParticle(core);

            EffectParticle flame{};
            flame.position = center + nozzleJitter + (exhaustDirection * randomRange(0.03f, missilePreset ? 0.10f : 0.08f));
            flame.velocity = (advectedVelocity * 1.2f) +
                             (exhaustDirection * randomRange(missilePreset ? 16.0f : 12.0f, missilePreset ? 30.0f : 22.0f) *
                              sampleIntensity) +
                             (randomInUnitSphere() * (missilePreset ? 1.1f : 1.0f));
            flame.axis = exhaustDirection;
            flame.color = glm::vec4(glm::mix(flameColor, flameHighlight, randomRange(0.08f, 0.22f)), 1.0f);
            flame.lifetime = missilePreset ? randomRange(0.11f, 0.18f) : randomRange(0.09f, 0.14f);
            flame.startSize = (missilePreset ? 0.10f : 0.13f) * sampleScale;
            flame.endSize = missilePreset ? randomRange(0.44f, 0.72f) * sampleIntensity : randomRange(0.64f, 0.98f) * sampleIntensity;
            flame.stretch = missilePreset ? randomRange(1.45f, 2.05f) : randomRange(1.35f, 1.85f);
            flame.rotation = randomRange(0.0f, 6.28318f);
            flame.angularVelocity = randomRange(-2.0f, 2.0f);
            flame.softness = 0.92f;
            flame.emissive = (missilePreset ? 0.95f : 0.9f) * sampleScale;
            flame.seed = randomRange(0.0f, 1000.0f);
            flame.drag = 1.6f;
            flame.material = ParticleMaterial::FLAME;
            flame.blendMode = BlendMode::ADDITIVE;
            addParticle(flame);
        }
    }

    const int smokeCount = missilePreset ? 1 : 2;
    for (int sweepIndex = 0; sweepIndex < smokeSweepCount; ++sweepIndex)
    {
        const float sweepInterpolation = (smokeSweepCount == 1) ? 1.0f : static_cast<float>(sweepIndex) / static_cast<float>(smokeSweepCount - 1);
        const float sampleIntensity = intensity * glm::mix(0.78f, 1.0f, sweepInterpolation);
        const glm::vec3 trailEnd = glm::mix(start, end, sweepInterpolation);

        for (int smokeIndex = 0; smokeIndex < smokeCount; ++smokeIndex)
        {
            EffectParticle smoke{};
            smoke.position = trailEnd + (exhaustDirection * randomRange(missilePreset ? 0.14f : 0.10f, smokeClusterLength)) +
                             (randomInUnitSphere() * 0.10f);
            smoke.velocity = smokeAdvectedVelocity +
                             (exhaustDirection * randomRange(missilePreset ? 3.0f : 2.4f, missilePreset ? 7.0f : 5.0f) *
                              sampleIntensity) +
                             (randomInUnitSphere() * (missilePreset ? 1.1f : 0.9f));
            smoke.axis = safeNormalize(smoke.velocity, exhaustDirection);
            smoke.color = missilePreset
                              ? glm::vec4(0.30f, 0.30f, 0.32f, 0.42f)
                              : glm::vec4(0.20f, 0.24f, 0.28f, 0.34f);
            smoke.lifetime = missilePreset ? randomRange(0.42f, 0.68f) : randomRange(0.28f, 0.46f);
            smoke.startSize = missilePreset ? 0.14f : 0.18f;
            smoke.endSize = missilePreset ? randomRange(0.46f, 0.78f) * sampleIntensity : randomRange(0.60f, 0.96f) * sampleIntensity;
            smoke.stretch = randomRange(1.0f, 1.14f);
            smoke.rotation = randomRange(0.0f, 6.28318f);
            smoke.angularVelocity = randomRange(-0.6f, 0.6f);
            smoke.softness = 0.72f;
            smoke.emissive = 1.0f;
            smoke.seed = randomRange(0.0f, 1000.0f);
            smoke.drag = 0.35f;
            smoke.upwardAcceleration = missilePreset ? 1.8f : 0.6f;
            smoke.material = ParticleMaterial::SMOKE;
            smoke.blendMode = BlendMode::ALPHA;
            addParticle(smoke);
        }
    }

    const int hazeCount = missilePreset ? 1 : 2;
    for (int sweepIndex = 0; sweepIndex < hazeSweepCount; ++sweepIndex)
    {
        const float sweepInterpolation = (hazeSweepCount == 1) ? 1.0f : static_cast<float>(sweepIndex) / static_cast<float>(hazeSweepCount - 1);
        const float sampleIntensity = intensity * glm::mix(0.74f, 1.0f, sweepInterpolation);
        const glm::vec3 trailEnd = glm::mix(start, end, sweepInterpolation);

        for (int hazeIndex = 0; hazeIndex < hazeCount; ++hazeIndex)
        {
            const float interpolation = (hazeCount == 1) ? 1.0f : static_cast<float>(hazeIndex) / static_cast<float>(hazeCount - 1);
            HeatHazeSprite haze{};
            haze.position = trailEnd +
                            (exhaustDirection * (glm::mix(0.04f, hazeClusterLength, interpolation) +
                                                 randomRange(0.0f, hazeClusterLength * 0.12f)));
            haze.velocity = hazeAdvectedVelocity +
                            (exhaustDirection * glm::mix(missilePreset ? 6.5f : 3.8f, missilePreset ? 3.0f : 1.8f, interpolation));
            haze.axis = exhaustDirection;
            haze.lifetime = missilePreset ? randomRange(0.07f, 0.11f) : randomRange(0.06f, 0.10f);
            haze.radius = (missilePreset ? randomRange(0.38f, 0.62f) : randomRange(0.40f, 0.66f)) * sampleIntensity;
            haze.stretch = missilePreset ? randomRange(1.15f, 1.45f) : randomRange(1.18f, 1.50f);
            haze.rotation = randomRange(0.0f, 6.28318f);
            haze.angularVelocity = randomRange(-1.4f, 1.4f);
            haze.strength = (missilePreset ? randomRange(1.6f, 2.6f) : randomRange(1.1f, 1.9f)) * sampleIntensity;
            haze.seed = randomRange(0.0f, 1000.0f);
            haze.drag = 1.8f;
            addHeatHaze(haze);
        }
    }
}

float SceneEffects::randomRange(float minimum, float maximum)
{
    std::uniform_real_distribution<float> distribution(minimum, maximum);
    return distribution(m_rng);
}

glm::vec3 SceneEffects::randomInUnitSphere()
{
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        glm::vec3 candidate(randomRange(-1.0f, 1.0f),
                            randomRange(-1.0f, 1.0f),
                            randomRange(-1.0f, 1.0f));
        if (glm::length2(candidate) <= 1.0f)
        {
            return candidate;
        }
    }

    return glm::vec3(0.0f);
}

glm::vec3 SceneEffects::randomUnitVector()
{
    return safeNormalize(randomInUnitSphere(), glm::vec3(0.0f, 1.0f, 0.0f));
}