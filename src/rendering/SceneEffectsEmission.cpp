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

namespace
{
    constexpr float kTwoPi = 6.28318530718f;

    float smooth01(float value)
    {
        const float t = glm::clamp(value, 0.0f, 1.0f);
        return t * t * (3.0f - (2.0f * t));
    }
}

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

void SceneEffects::emitJetWake(const glm::vec3 &start,
                               const glm::vec3 &end,
                               const glm::vec3 &forward,
                               const glm::vec3 &carrierVelocity,
                               float intensity)
{
    const float clampedIntensity = glm::clamp(intensity, 0.0f, 1.0f);
    if (clampedIntensity <= 0.02f)
    {
        return;
    }

    const glm::vec3 wakeDirection = safeNormalize(-forward, glm::vec3(0.0f, 0.0f, -1.0f));
    const glm::vec3 lateralDirection = perpendicularTo(wakeDirection);
    const glm::vec3 verticalDirection = safeNormalize(glm::cross(wakeDirection, lateralDirection), glm::vec3(0.0f, 1.0f, 0.0f));
    const float sweepLength = glm::length(end - start);
    const int sampleCount = std::clamp(static_cast<int>(std::ceil(sweepLength / 1.4f)) + 1, 1, 8);

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        const float interpolation = (sampleCount == 1) ? 1.0f : static_cast<float>(sampleIndex) / static_cast<float>(sampleCount - 1);
        const glm::vec3 base = glm::mix(start, end, interpolation);
        const float swirl = randomRange(0.0f, kTwoPi);
        const glm::vec3 swirlOffset =
            ((lateralDirection * std::cos(swirl)) + (verticalDirection * std::sin(swirl))) *
            randomRange(0.02f, 0.18f) * clampedIntensity;

        EffectParticle vapor{};
        vapor.position = base + (wakeDirection * randomRange(0.05f, 0.55f)) + swirlOffset;
        vapor.velocity = (carrierVelocity * 0.035f) +
                         (wakeDirection * randomRange(6.0f, 16.0f) * clampedIntensity) +
                         (swirlOffset * randomRange(4.0f, 10.0f)) +
                         (randomInUnitSphere() * 0.7f);
        vapor.axis = safeNormalize(vapor.velocity, wakeDirection);
        vapor.color = glm::vec4(glm::mix(glm::vec3(0.66f, 0.76f, 0.86f),
                                         glm::vec3(0.90f, 0.96f, 1.0f),
                                         randomRange(0.0f, 1.0f)),
                                randomRange(0.10f, 0.24f) * clampedIntensity);
        vapor.lifetime = randomRange(0.7f, 1.7f);
        vapor.startSize = randomRange(0.06f, 0.14f);
        vapor.endSize = randomRange(1.2f, 2.8f) * (0.45f + clampedIntensity);
        vapor.stretch = randomRange(1.6f, 2.8f);
        vapor.rotation = randomRange(0.0f, kTwoPi);
        vapor.angularVelocity = randomRange(-1.5f, 1.5f);
        vapor.softness = 0.85f;
        vapor.emissive = 0.92f;
        vapor.seed = randomRange(0.0f, 1000.0f);
        vapor.drag = randomRange(0.22f, 0.55f);
        vapor.upwardAcceleration = randomRange(-0.15f, 0.45f);
        vapor.material = ParticleMaterial::SMOKE;
        vapor.blendMode = BlendMode::ALPHA;
        addParticle(vapor);
    }
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

void SceneEffects::spawnMissileLaunch(const glm::vec3 &position,
                                      const glm::vec3 &forward,
                                      const glm::vec3 &carrierVelocity,
                                      float intensity)
{
    const float clampedIntensity = glm::clamp(intensity, 0.45f, 2.2f);
    const glm::vec3 launchForward = safeNormalize(forward, glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 exhaustDirection = safeNormalize(-launchForward, glm::vec3(0.0f, -1.0f, 0.0f));
    const glm::vec3 lateralDirection = perpendicularTo(exhaustDirection);
    const glm::vec3 verticalDirection = safeNormalize(glm::cross(exhaustDirection, lateralDirection), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::vec3 nozzlePosition = position + (exhaustDirection * 1.05f);

    // Tight, blinding ignition core.
    EffectParticle flashCore{};
    flashCore.position = nozzlePosition;
    flashCore.velocity = (carrierVelocity * 0.04f) + (exhaustDirection * 10.0f);
    flashCore.axis = exhaustDirection;
    flashCore.color = glm::vec4(1.0f, 0.88f, 0.55f, 1.0f);
    flashCore.lifetime = 0.18f;
    flashCore.startSize = 1.4f * clampedIntensity;
    flashCore.endSize = 6.5f * clampedIntensity;
    flashCore.stretch = 1.1f;
    flashCore.softness = 1.2f;
    flashCore.emissive = 2.4f;
    flashCore.seed = randomRange(0.0f, 1000.0f);
    flashCore.material = ParticleMaterial::GLOW;
    flashCore.blendMode = BlendMode::ADDITIVE;
    addParticle(flashCore);

    // Broad soft bloom around the core for a punchy light flash.
    EffectParticle flashBloom = flashCore;
    flashBloom.velocity = carrierVelocity * 0.03f;
    flashBloom.color = glm::vec4(1.0f, 0.7f, 0.34f, 0.8f);
    flashBloom.lifetime = 0.30f;
    flashBloom.startSize = 3.2f * clampedIntensity;
    flashBloom.endSize = 13.0f * clampedIntensity;
    flashBloom.emissive = 1.5f;
    flashBloom.seed = randomRange(0.0f, 1000.0f);
    addParticle(flashBloom);

    const int flameCount = std::clamp(static_cast<int>(std::round(22.0f * clampedIntensity)), 12, 44);
    for (int index = 0; index < flameCount; ++index)
    {
        const glm::vec3 sideJitter = (lateralDirection * randomRange(-0.22f, 0.22f)) +
                                     (verticalDirection * randomRange(-0.22f, 0.22f));

        EffectParticle flame{};
        flame.position = nozzlePosition + sideJitter + (exhaustDirection * randomRange(0.0f, 0.6f));
        flame.velocity = (carrierVelocity * 0.08f) +
                         (exhaustDirection * randomRange(28.0f, 64.0f) * clampedIntensity) +
                         (sideJitter * randomRange(8.0f, 18.0f));
        flame.axis = exhaustDirection;
        flame.color = glm::vec4(glm::mix(glm::vec3(1.0f, 0.44f, 0.12f),
                                         glm::vec3(1.0f, 0.92f, 0.72f),
                                         randomRange(0.12f, 0.48f)),
                                1.0f);
        flame.lifetime = randomRange(0.16f, 0.34f);
        flame.startSize = randomRange(0.22f, 0.42f) * clampedIntensity;
        flame.endSize = randomRange(2.2f, 4.6f) * clampedIntensity;
        flame.stretch = randomRange(2.4f, 5.0f);
        flame.rotation = randomRange(0.0f, 6.28318f);
        flame.angularVelocity = randomRange(-4.0f, 4.0f);
        flame.softness = 0.95f;
        flame.emissive = randomRange(1.2f, 1.7f);
        flame.seed = randomRange(0.0f, 1000.0f);
        flame.drag = 2.2f;
        flame.material = ParticleMaterial::FLAME;
        flame.blendMode = BlendMode::ADDITIVE;
        addParticle(flame);
    }

    // Billowing exhaust smoke. Longer-lived than a blast so it hangs in the
    // air and seeds the base of the climb-out pillar.
    const int smokeCount = std::clamp(static_cast<int>(std::round(56.0f * clampedIntensity)), 40, 120);
    for (int index = 0; index < smokeCount; ++index)
    {
        const float azimuth = randomRange(0.0f, 6.28318f);
        const glm::vec3 radialDirection =
            (lateralDirection * std::cos(azimuth)) + (verticalDirection * std::sin(azimuth));
        const glm::vec3 blastDirection = safeNormalize((radialDirection * randomRange(0.35f, 1.0f)) +
                                                           (exhaustDirection * randomRange(0.55f, 1.45f)) +
                                                           (glm::vec3(0.0f, 1.0f, 0.0f) * randomRange(0.0f, 0.35f)),
                                                       exhaustDirection);

        EffectParticle smoke{};
        smoke.position = nozzlePosition + (blastDirection * randomRange(0.12f, 1.4f) * clampedIntensity);
        smoke.velocity = (carrierVelocity * 0.05f) +
                         (blastDirection * randomRange(16.0f, 40.0f) * clampedIntensity) +
                         (glm::vec3(0.0f, 1.0f, 0.0f) * randomRange(0.0f, 5.0f));
        smoke.axis = safeNormalize(smoke.velocity, blastDirection);
        smoke.color = glm::vec4(glm::vec3(randomRange(0.26f, 0.46f)), randomRange(0.58f, 0.8f));
        smoke.lifetime = randomRange(1.8f, 3.6f);
        smoke.startSize = randomRange(0.5f, 1.0f) * clampedIntensity;
        smoke.endSize = randomRange(5.5f, 11.0f) * clampedIntensity;
        smoke.stretch = randomRange(1.0f, 1.5f);
        smoke.rotation = randomRange(0.0f, 6.28318f);
        smoke.angularVelocity = randomRange(-1.4f, 1.4f);
        smoke.softness = 0.85f;
        smoke.emissive = 1.0f;
        smoke.seed = randomRange(0.0f, 1000.0f);
        smoke.drag = randomRange(0.18f, 0.36f);
        smoke.upwardAcceleration = randomRange(2.5f, 6.5f);
        smoke.material = ParticleMaterial::SMOKE;
        smoke.blendMode = BlendMode::ALPHA;
        addParticle(smoke);
    }

    const int sparkCount = std::clamp(static_cast<int>(std::round(20.0f * clampedIntensity)), 10, 34);
    for (int index = 0; index < sparkCount; ++index)
    {
        glm::vec3 sparkDirection = randomUnitVector();
        sparkDirection = safeNormalize(glm::mix(sparkDirection, exhaustDirection, 0.35f), exhaustDirection);

        EffectParticle spark{};
        spark.position = nozzlePosition;
        spark.velocity = (sparkDirection * randomRange(24.0f, 62.0f) * clampedIntensity) + (carrierVelocity * 0.12f);
        spark.axis = sparkDirection;
        spark.color = glm::vec4(1.0f, randomRange(0.72f, 0.94f), 0.34f, 1.0f);
        spark.lifetime = randomRange(0.14f, 0.28f);
        spark.startSize = randomRange(0.06f, 0.12f) * clampedIntensity;
        spark.endSize = randomRange(0.65f, 1.35f) * clampedIntensity;
        spark.stretch = randomRange(3.5f, 6.5f);
        spark.softness = 1.0f;
        spark.emissive = 1.35f;
        spark.seed = randomRange(0.0f, 1000.0f);
        spark.drag = 3.2f;
        spark.material = ParticleMaterial::SPARK;
        spark.blendMode = BlendMode::ADDITIVE;
        addParticle(spark);
    }

    const int hazeCount = std::clamp(static_cast<int>(std::round(8.0f * clampedIntensity)), 5, 16);
    for (int index = 0; index < hazeCount; ++index)
    {
        HeatHazeSprite haze{};
        haze.position = nozzlePosition + (randomInUnitSphere() * 0.7f * clampedIntensity);
        haze.velocity = (exhaustDirection * randomRange(5.0f, 16.0f) * clampedIntensity) + (randomInUnitSphere() * 4.0f);
        haze.axis = safeNormalize(haze.velocity, exhaustDirection);
        haze.lifetime = randomRange(0.16f, 0.32f);
        haze.radius = randomRange(2.1f, 4.8f) * clampedIntensity;
        haze.stretch = randomRange(1.2f, 1.8f);
        haze.rotation = randomRange(0.0f, 6.28318f);
        haze.angularVelocity = randomRange(-2.2f, 2.2f);
        haze.strength = randomRange(3.2f, 6.8f) * clampedIntensity;
        haze.seed = randomRange(0.0f, 1000.0f);
        haze.drag = 0.8f;
        addHeatHaze(haze);
    }
}

void SceneEffects::spawnLaunchGroundCloud(const glm::vec3 &position, const glm::vec3 &up, float intensity)
{
    const float clampedIntensity = glm::clamp(intensity, 0.5f, 2.2f);
    const glm::vec3 upDirection = safeNormalize(up, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::vec3 groundAxisA = perpendicularTo(upDirection);
    const glm::vec3 groundAxisB = safeNormalize(glm::cross(upDirection, groundAxisA), glm::vec3(1.0f, 0.0f, 0.0f));

    // A big, slow, ground-hugging cloud of ejection gas that rolls outward and
    // lingers for several seconds: the iconic launch-site signature. Fewer,
    // more varied billows read better than a dense wall of identical puffs
    // now that the shader erodes them individually.
    const int billowCount = std::clamp(static_cast<int>(std::round(52.0f * clampedIntensity)), 36, 90);
    for (int index = 0; index < billowCount; ++index)
    {
        const float azimuth = randomRange(0.0f, 6.28318f);
        const glm::vec3 radialDirection =
            (groundAxisA * std::cos(azimuth)) + (groundAxisB * std::sin(azimuth));

        // Slight warm tint variation so the cloud is not uniformly grey.
        const float tone = randomRange(0.30f, 0.56f);
        const glm::vec3 warmDust(tone * 1.12f, tone * 1.02f, tone * 0.88f);
        const glm::vec3 greyGas(tone);

        EffectParticle billow{};
        billow.position = position + (radialDirection * randomRange(0.2f, 3.6f) * clampedIntensity) +
                          (upDirection * randomRange(0.0f, 1.6f));
        billow.velocity = (radialDirection * randomRange(3.0f, 17.0f) * clampedIntensity) +
                          (upDirection * randomRange(1.0f, 6.5f));
        billow.axis = safeNormalize(billow.velocity, radialDirection);
        billow.color = glm::vec4(glm::mix(greyGas, warmDust, randomRange(0.0f, 1.0f)),
                                 randomRange(0.45f, 0.75f));
        billow.lifetime = randomRange(3.2f, 8.5f);
        billow.startSize = randomRange(0.9f, 3.4f) * clampedIntensity;
        billow.endSize = randomRange(6.0f, 19.0f) * clampedIntensity;
        billow.stretch = randomRange(0.85f, 1.6f);
        billow.rotation = randomRange(0.0f, 6.28318f);
        billow.angularVelocity = randomRange(-1.4f, 1.4f);
        billow.softness = randomRange(0.65f, 0.95f);
        billow.emissive = randomRange(0.75f, 0.95f);
        billow.seed = randomRange(0.0f, 1000.0f);
        billow.drag = randomRange(0.45f, 1.25f);
        billow.upwardAcceleration = randomRange(0.4f, 2.6f);
        billow.material = ParticleMaterial::SMOKE;
        billow.blendMode = BlendMode::ALPHA;
        addParticle(billow);
    }

    // A few warm, low dust puffs kicked up at the base.
    const int dustCount = std::clamp(static_cast<int>(std::round(14.0f * clampedIntensity)), 8, 26);
    for (int index = 0; index < dustCount; ++index)
    {
        const float azimuth = randomRange(0.0f, 6.28318f);
        const glm::vec3 radialDirection =
            (groundAxisA * std::cos(azimuth)) + (groundAxisB * std::sin(azimuth));

        EffectParticle dust{};
        dust.position = position + (radialDirection * randomRange(0.1f, 1.8f) * clampedIntensity);
        dust.velocity = (radialDirection * randomRange(8.0f, 22.0f) * clampedIntensity) +
                        (upDirection * randomRange(0.5f, 3.0f));
        dust.axis = safeNormalize(dust.velocity, radialDirection);
        dust.color = glm::vec4(glm::mix(glm::vec3(0.62f, 0.5f, 0.34f),
                                        glm::vec3(0.42f, 0.4f, 0.38f),
                                        randomRange(0.0f, 1.0f)),
                               randomRange(0.42f, 0.62f));
        dust.lifetime = randomRange(1.4f, 3.0f);
        dust.startSize = randomRange(0.6f, 1.2f) * clampedIntensity;
        dust.endSize = randomRange(3.5f, 7.0f) * clampedIntensity;
        dust.stretch = randomRange(1.1f, 1.6f);
        dust.rotation = randomRange(0.0f, 6.28318f);
        dust.angularVelocity = randomRange(-1.6f, 1.6f);
        dust.softness = 0.85f;
        dust.emissive = 0.9f;
        dust.seed = randomRange(0.0f, 1000.0f);
        dust.drag = randomRange(0.9f, 1.6f);
        dust.upwardAcceleration = randomRange(0.0f, 1.0f);
        dust.material = ParticleMaterial::SMOKE;
        dust.blendMode = BlendMode::ALPHA;
        addParticle(dust);
    }
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

    // Expanding blast ring: the quad stays at full blast radius while the
    // shader sweeps a thin ring outward across it over the lifetime.
    EffectParticle shockRing{};
    shockRing.position = position;
    shockRing.velocity = velocityHint * 0.05f;
    shockRing.axis = forwardBias;
    shockRing.color = glm::vec4(1.0f, 0.9f, 0.75f, 1.0f);
    shockRing.lifetime = 0.7f;
    shockRing.startSize = 42.0f * clampedIntensity;
    shockRing.endSize = 42.0f * clampedIntensity;
    shockRing.stretch = 1.0f;
    shockRing.softness = 1.2f;
    shockRing.emissive = 1.6f;
    shockRing.seed = randomRange(0.0f, 1000.0f);
    shockRing.material = ParticleMaterial::SHOCKWAVE;
    shockRing.blendMode = BlendMode::ADDITIVE;
    addParticle(shockRing);

    // Glowing debris fragments: upward-biased cone, pulled down by ~2g so
    // they arc visibly, stretched along their velocity by the update step.
    const int debrisCount = std::clamp(static_cast<int>(std::round(24.0f * clampedIntensity)), 18, 32);
    for (int index = 0; index < debrisCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();
        direction.y = std::abs(direction.y) * 0.85f + 0.15f;
        direction = safeNormalize(direction, glm::vec3(0.0f, 1.0f, 0.0f));

        EffectParticle debris{};
        debris.position = position + (direction * randomRange(0.3f, 1.5f));
        debris.velocity = (direction * randomRange(35.0f, 90.0f) * clampedIntensity) + (velocityHint * 0.3f);
        debris.axis = safeNormalize(debris.velocity, direction);
        debris.color = glm::vec4(1.0f, randomRange(0.55f, 0.8f), 0.28f, 1.0f);
        debris.lifetime = randomRange(1.0f, 2.2f);
        debris.startSize = randomRange(0.35f, 0.7f) * clampedIntensity;
        debris.endSize = randomRange(0.15f, 0.3f) * clampedIntensity;
        debris.stretch = randomRange(3.0f, 6.0f);
        debris.softness = 1.0f;
        debris.emissive = randomRange(1.4f, 2.2f);
        debris.seed = randomRange(0.0f, 1000.0f);
        debris.drag = 0.4f;
        debris.upwardAcceleration = -19.6f;
        debris.material = ParticleMaterial::DEBRIS;
        debris.blendMode = BlendMode::ADDITIVE;
        addParticle(debris);
    }

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
    const float clampedIntensity = glm::clamp(intensity,
                                             missilePreset ? 0.22f : 0.16f,
                                             missilePreset ? 1.35f : 1.25f);
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

    const float speed = glm::length(carrierVelocity);
    const float speedBlend = glm::clamp((speed - (missilePreset ? 80.0f : 140.0f)) /
                                            (missilePreset ? 280.0f : 260.0f),
                                        0.0f,
                                        1.0f);
    const int plumeSweepCount = computeSweepCount(missilePreset ? 0.38f : 0.48f,
                                                  missilePreset ? 30 : 20);
    const int smokeSweepCount = computeSweepCount(missilePreset ? 0.82f : 1.25f,
                                                  missilePreset ? 12 : 7);
    const int hazeSweepCount = computeSweepCount(missilePreset ? 0.58f : 0.72f,
                                                 missilePreset ? 10 : 8);

    const glm::vec3 hotCoreColor = missilePreset
                                       ? glm::vec3(1.0f, 0.95f, 0.80f)
                                       : glm::vec3(0.86f, 0.96f, 1.0f);
    const glm::vec3 flameColor = missilePreset
                                     ? glm::vec3(1.0f, 0.52f, 0.12f)
                                     : glm::vec3(0.22f, 0.56f, 1.0f);
    const glm::vec3 flameEdgeColor = missilePreset
                                         ? glm::vec3(1.0f, 0.25f, 0.06f)
                                         : glm::vec3(0.72f, 0.36f, 1.0f);

    EffectParticle nozzleGlow{};
    nozzleGlow.position = end + (exhaustDirection * (missilePreset ? 0.10f : 0.16f));
    nozzleGlow.velocity = (carrierVelocity * 0.025f) + (exhaustDirection * randomRange(1.5f, 5.0f));
    nozzleGlow.axis = exhaustDirection;
    nozzleGlow.color = glm::vec4(hotCoreColor, missilePreset ? 0.95f : 0.88f);
    nozzleGlow.lifetime = missilePreset ? randomRange(0.055f, 0.085f) : randomRange(0.045f, 0.075f);
    nozzleGlow.startSize = (missilePreset ? 0.20f : 0.28f) * clampedIntensity;
    nozzleGlow.endSize = (missilePreset ? 0.55f : 0.72f) * clampedIntensity;
    nozzleGlow.stretch = missilePreset ? 1.35f : 1.55f;
    nozzleGlow.softness = 1.1f;
    nozzleGlow.emissive = missilePreset ? 1.75f : 1.55f;
    nozzleGlow.seed = randomRange(0.0f, 1000.0f);
    nozzleGlow.material = ParticleMaterial::GLOW;
    nozzleGlow.blendMode = BlendMode::ADDITIVE;
    addParticle(nozzleGlow);

    for (int sweepIndex = 0; sweepIndex < plumeSweepCount; ++sweepIndex)
    {
        const float sweepInterpolation = (plumeSweepCount == 1) ? 1.0f : static_cast<float>(sweepIndex) / static_cast<float>(plumeSweepCount - 1);
        const float sampleIntensity = clampedIntensity * glm::mix(0.80f, 1.0f, smooth01(sweepInterpolation));
        const glm::vec3 trailEnd = glm::mix(start, end, sweepInterpolation);
        const float jitterSpan = (missilePreset ? 0.045f : 0.070f) * (0.6f + sampleIntensity);
        const glm::vec3 nozzleJitter =
            (lateralDirection * randomRange(-jitterSpan, jitterSpan)) +
            (verticalDirection * randomRange(-jitterSpan, jitterSpan));

        EffectParticle core{};
        core.position = trailEnd + nozzleJitter + (exhaustDirection * randomRange(0.03f, missilePreset ? 0.42f : 0.72f));
        core.velocity = (carrierVelocity * (missilePreset ? 0.045f : 0.055f)) +
                        (exhaustDirection * randomRange(missilePreset ? 24.0f : 18.0f,
                                                        missilePreset ? 48.0f : 34.0f) *
                         sampleIntensity);
        core.axis = exhaustDirection;
        core.color = glm::vec4(hotCoreColor, 1.0f);
        core.lifetime = missilePreset ? randomRange(0.075f, 0.125f) : randomRange(0.060f, 0.105f);
        core.startSize = (missilePreset ? 0.09f : 0.13f) * sampleIntensity;
        core.endSize = (missilePreset ? 0.38f : 0.52f) * sampleIntensity;
        core.stretch = missilePreset ? randomRange(2.4f, 3.6f) : randomRange(3.0f, 4.4f);
        core.softness = 1.0f;
        core.emissive = missilePreset ? 1.35f : 1.2f;
        core.seed = randomRange(0.0f, 1000.0f);
        core.drag = missilePreset ? 2.2f : 1.8f;
        core.material = ParticleMaterial::GLOW;
        core.blendMode = BlendMode::ADDITIVE;
        addParticle(core);

        EffectParticle flame{};
        flame.position = trailEnd + nozzleJitter + (exhaustDirection * randomRange(0.10f, missilePreset ? 0.72f : 1.18f));
        flame.velocity = (carrierVelocity * (missilePreset ? 0.055f : 0.060f)) +
                         (exhaustDirection * randomRange(missilePreset ? 32.0f : 24.0f,
                                                         missilePreset ? 76.0f : 48.0f) *
                          sampleIntensity) +
                         (randomInUnitSphere() * (missilePreset ? 1.8f : 1.3f));
        flame.axis = exhaustDirection;
        flame.color = glm::vec4(glm::mix(flameColor, flameEdgeColor, randomRange(0.0f, 0.45f)), 1.0f);
        flame.lifetime = missilePreset ? randomRange(0.13f, 0.24f) : randomRange(0.095f, 0.17f);
        flame.startSize = (missilePreset ? 0.15f : 0.22f) * sampleIntensity;
        flame.endSize = missilePreset
                            ? randomRange(0.82f, 1.55f) * sampleIntensity
                            : randomRange(1.05f, 1.95f) * sampleIntensity;
        flame.stretch = missilePreset ? randomRange(3.4f, 5.4f) : randomRange(4.2f, 7.0f);
        flame.rotation = randomRange(0.0f, kTwoPi);
        flame.angularVelocity = randomRange(-2.8f, 2.8f);
        flame.softness = 0.95f;
        flame.emissive = missilePreset ? 1.08f : 0.98f;
        flame.seed = randomRange(0.0f, 1000.0f);
        flame.drag = missilePreset ? 1.55f : 1.2f;
        flame.material = ParticleMaterial::FLAME;
        flame.blendMode = BlendMode::ADDITIVE;
        addParticle(flame);
    }

    const int diamondCount = missilePreset ? 3 : 5;
    const float diamondSpacing = missilePreset ? 0.42f : 0.58f;
    const float diamondStart = missilePreset ? 0.38f : 0.54f;
    for (int diamondIndex = 0; diamondIndex < diamondCount; ++diamondIndex)
    {
        const float distance = diamondStart + (diamondSpacing * static_cast<float>(diamondIndex)) + randomRange(-0.035f, 0.035f);
        const float fade = 1.0f - (static_cast<float>(diamondIndex) / static_cast<float>(diamondCount));
        const float diamondIntensity = clampedIntensity * glm::mix(0.45f, 1.0f, fade);
        const glm::vec3 diamondColor = missilePreset
                                           ? glm::mix(glm::vec3(1.0f, 0.38f, 0.08f), hotCoreColor, fade)
                                           : ((diamondIndex % 2) == 0
                                                  ? glm::vec3(0.84f, 0.96f, 1.0f)
                                                  : glm::vec3(0.28f, 0.58f, 1.0f));

        EffectParticle diamond{};
        diamond.position = end + (exhaustDirection * distance) +
                           (lateralDirection * randomRange(-0.025f, 0.025f)) +
                           (verticalDirection * randomRange(-0.025f, 0.025f));
        diamond.velocity = (carrierVelocity * 0.035f) + (exhaustDirection * randomRange(3.0f, 10.0f));
        diamond.axis = exhaustDirection;
        diamond.color = glm::vec4(diamondColor, 1.0f);
        diamond.lifetime = missilePreset ? randomRange(0.055f, 0.090f) : randomRange(0.050f, 0.085f);
        diamond.startSize = (missilePreset ? 0.12f : 0.17f) * diamondIntensity;
        diamond.endSize = (missilePreset ? 0.34f : 0.46f) * diamondIntensity;
        diamond.stretch = missilePreset ? randomRange(1.65f, 2.15f) : randomRange(1.85f, 2.55f);
        diamond.rotation = randomRange(-0.12f, 0.12f);
        diamond.softness = 1.0f;
        diamond.emissive = missilePreset ? 1.55f : 1.35f;
        diamond.seed = randomRange(0.0f, 1000.0f);
        diamond.drag = 1.4f;
        diamond.material = ParticleMaterial::SHOCK_DIAMOND;
        diamond.blendMode = BlendMode::ADDITIVE;
        addParticle(diamond);
    }

    const int smokeCount = missilePreset ? 2 : 1;
    for (int sweepIndex = 0; sweepIndex < smokeSweepCount; ++sweepIndex)
    {
        const float sweepInterpolation = (smokeSweepCount == 1) ? 1.0f : static_cast<float>(sweepIndex) / static_cast<float>(smokeSweepCount - 1);
        const float sampleIntensity = clampedIntensity * glm::mix(0.70f, 1.0f, smooth01(sweepInterpolation));
        const glm::vec3 trailEnd = glm::mix(start, end, sweepInterpolation);

        for (int smokeIndex = 0; smokeIndex < smokeCount; ++smokeIndex)
        {
            const float smokeSpread = missilePreset ? randomRange(0.08f, 0.42f) : randomRange(0.04f, 0.18f);
            EffectParticle smoke{};
            smoke.position = trailEnd + (exhaustDirection * randomRange(missilePreset ? 0.45f : 0.35f,
                                                                        missilePreset ? 1.65f : 1.10f)) +
                             (randomInUnitSphere() * smokeSpread);
            smoke.velocity = (carrierVelocity * (missilePreset ? 0.070f : 0.045f)) +
                             (exhaustDirection * randomRange(missilePreset ? 5.0f : 4.0f,
                                                             missilePreset ? 14.0f : 9.0f) *
                              sampleIntensity) +
                             (randomInUnitSphere() * (missilePreset ? 2.6f : 1.1f));
            smoke.axis = safeNormalize(smoke.velocity, exhaustDirection);
            smoke.color = missilePreset
                              ? glm::vec4(glm::mix(glm::vec3(0.30f, 0.29f, 0.28f),
                                                   glm::vec3(0.48f, 0.43f, 0.36f),
                                                   randomRange(0.0f, 1.0f)),
                                          randomRange(0.42f, 0.66f))
                              : glm::vec4(glm::mix(glm::vec3(0.18f, 0.22f, 0.27f),
                                                   glm::vec3(0.42f, 0.50f, 0.58f),
                                                   speedBlend * 0.55f),
                                          randomRange(0.12f, 0.24f));
            smoke.lifetime = missilePreset ? randomRange(1.15f, 2.45f) : randomRange(0.48f, 1.05f);
            smoke.startSize = missilePreset ? randomRange(0.22f, 0.42f) : randomRange(0.10f, 0.20f);
            smoke.endSize = missilePreset
                                ? randomRange(1.5f, 3.6f) * (0.65f + sampleIntensity)
                                : randomRange(0.85f, 1.75f) * (0.55f + sampleIntensity + speedBlend * 0.25f);
            smoke.stretch = missilePreset ? randomRange(1.15f, 1.75f) : randomRange(1.8f, 3.2f);
            smoke.rotation = randomRange(0.0f, kTwoPi);
            smoke.angularVelocity = randomRange(-1.0f, 1.0f);
            smoke.softness = missilePreset ? 0.82f : 0.75f;
            smoke.emissive = missilePreset ? 0.96f : 0.88f;
            smoke.seed = randomRange(0.0f, 1000.0f);
            smoke.drag = missilePreset ? randomRange(0.22f, 0.46f) : randomRange(0.30f, 0.62f);
            smoke.upwardAcceleration = missilePreset ? randomRange(1.4f, 4.6f) : randomRange(-0.2f, 0.8f);
            smoke.material = ParticleMaterial::SMOKE;
            smoke.blendMode = BlendMode::ALPHA;
            addParticle(smoke);
        }
    }

    const int hazeCount = missilePreset ? 2 : 1;
    for (int sweepIndex = 0; sweepIndex < hazeSweepCount; ++sweepIndex)
    {
        const float sweepInterpolation = (hazeSweepCount == 1) ? 1.0f : static_cast<float>(sweepIndex) / static_cast<float>(hazeSweepCount - 1);
        const float sampleIntensity = clampedIntensity * glm::mix(0.72f, 1.0f, smooth01(sweepInterpolation));
        const glm::vec3 trailEnd = glm::mix(start, end, sweepInterpolation);

        for (int hazeIndex = 0; hazeIndex < hazeCount; ++hazeIndex)
        {
            const float interpolation = (hazeCount == 1) ? 1.0f : static_cast<float>(hazeIndex) / static_cast<float>(hazeCount - 1);
            HeatHazeSprite haze{};
            haze.position = trailEnd +
                            (exhaustDirection * (glm::mix(0.10f, missilePreset ? 1.05f : 1.45f, interpolation) +
                                                 randomRange(0.0f, missilePreset ? 0.28f : 0.36f)));
            haze.velocity = (carrierVelocity * (missilePreset ? 0.05f : 0.045f)) +
                            (exhaustDirection * glm::mix(missilePreset ? 13.0f : 9.0f,
                                                         missilePreset ? 5.0f : 4.0f,
                                                         interpolation));
            haze.axis = exhaustDirection;
            haze.lifetime = missilePreset ? randomRange(0.11f, 0.18f) : randomRange(0.10f, 0.17f);
            haze.radius = (missilePreset ? randomRange(0.70f, 1.25f) : randomRange(0.80f, 1.55f)) *
                          (0.65f + sampleIntensity);
            haze.stretch = missilePreset ? randomRange(1.55f, 2.25f) : randomRange(1.85f, 2.85f);
            haze.rotation = randomRange(0.0f, kTwoPi);
            haze.angularVelocity = randomRange(-2.2f, 2.2f);
            haze.strength = (missilePreset ? randomRange(3.0f, 5.4f) : randomRange(2.6f, 5.2f)) *
                            (0.55f + sampleIntensity);
            haze.seed = randomRange(0.0f, 1000.0f);
            haze.drag = missilePreset ? 1.2f : 1.45f;
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
