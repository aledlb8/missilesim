#include "Renderer.h"
#include "SceneEffects.h"
#include "pbr/PBRPipeline.h"

#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

void Renderer::beginSceneFrame(const glm::vec3 &clearColor)
{
    if (isPBRActive())
    {
        // PBR mode: store camera state; actual rendering is deferred to executeRenderPass
        m_pbrPipeline->beginFrame(buildViewMatrix(), buildProjectionMatrix(), m_cameraPosition);

        // SceneEffects still needs camera for particle rendering
        if (m_sceneEffects)
        {
            m_sceneEffects->setViewportSize(m_viewportWidth, m_viewportHeight);
            m_sceneEffects->setCamera(m_cameraPosition, buildViewMatrix(), buildProjectionMatrix());
        }
        return;
    }

    if (m_sceneEffects)
    {
        m_sceneEffects->setViewportSize(m_viewportWidth, m_viewportHeight);
        m_sceneEffects->setCamera(m_cameraPosition, buildViewMatrix(), buildProjectionMatrix());
        m_sceneEffects->beginScene(clearColor);
        return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportWidth, m_viewportHeight);
    glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::renderSceneEffects()
{
    if (isPBRActive())
    {
        // Feed effect lights to the clustered light system before the pass
        // (light culling runs inside executeRenderPass).
        uploadEffectLights();

        // Execute the full PBR render pipeline (shadows → depth → cull → shade → skybox → resolve)
        m_pbrPipeline->executeRenderPass();

        if (m_sceneEffects)
        {
            // Snapshot the opaque depth so soft particles sample a texture
            // that is not attached to the framebuffer they render into.
            m_pbrPipeline->copySceneDepthForEffects();
            m_sceneEffects->setExternalDepthTexture(m_pbrPipeline->sceneCopyDepthTexture());

            // Composite particles into the resolved HDR scene.
            m_pbrPipeline->bindResolvedFBO();
            m_sceneEffects->setCamera(m_cameraPosition, buildViewMatrix(), buildProjectionMatrix());
            m_sceneEffects->renderParticlesToScene();

            // Snapshot the colour (now including particles) and run the heat
            // distortion pass back into the resolve target, pre-bloom/tonemap
            // so the shimmer inherits HDR bloom.
            m_pbrPipeline->copySceneColorForDistortion();
            m_sceneEffects->setExternalSceneColor(m_pbrPipeline->sceneCopyColorTexture());
            m_pbrPipeline->bindResolvedFBO();
            m_sceneEffects->renderHeatHazePass();
            m_sceneEffects->clearExternalSceneColor();
        }
        return;
    }

    if (!m_sceneEffects)
    {
        return;
    }

    m_sceneEffects->setCamera(m_cameraPosition, buildViewMatrix(), buildProjectionMatrix());
    m_sceneEffects->renderParticlesToScene();
}

void Renderer::presentSceneFrame()
{
    if (isPBRActive())
    {
        // Bloom + tone mapping → default framebuffer
        m_pbrPipeline->postProcess();

        // Draw debug primitives on top of tonemapped result
        flushDebugPrimitivesInternal();

        if (m_sceneEffects)
        {
            m_sceneEffects->clearExternalDepthTexture();
        }
        return;
    }

    if (m_sceneEffects)
    {
        m_sceneEffects->setCamera(m_cameraPosition, buildViewMatrix(), buildProjectionMatrix());
        m_sceneEffects->presentScene();
    }
}

void Renderer::updateEffects(float deltaTime)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->update(deltaTime);
    }
    updateEffectLights(deltaTime);
}

// ---------------------------------------------------------------------------
// Effect lights: transient point lights emitted alongside particle effects so
// explosions, launches and engine plumes illuminate nearby geometry.
// ---------------------------------------------------------------------------

namespace {

// Cheap deterministic flicker in [1-amount, 1+amount).
float effectLightFlicker(float seed, float age, float amount)
{
    float t = std::sin(seed * 12.9898f + age * 37.0f) * 43758.5453f;
    float fract = t - std::floor(t);
    return 1.0f + amount * (2.0f * fract - 1.0f);
}

} // namespace

void Renderer::addEffectLight(const EffectLight &light)
{
    // Hard cap on the pool; oldest transient content ages out quickly anyway.
    constexpr std::size_t kMaxEffectLights = 128;
    if (m_effectLights.size() < kMaxEffectLights)
    {
        m_effectLights.push_back(light);
    }
}

void Renderer::updateEffectLights(float deltaTime)
{
    for (EffectLight &light : m_effectLights)
    {
        light.age += deltaTime;
    }
    m_effectLights.erase(
        std::remove_if(m_effectLights.begin(), m_effectLights.end(),
                       [](const EffectLight &l)
                       { return l.lifetime <= 0.0f || l.age >= l.lifetime; }),
        m_effectLights.end());
}

void Renderer::uploadEffectLights()
{
    m_effectLightScratch.clear();

    for (const EffectLight &light : m_effectLights)
    {
        float envelope = 1.0f;
        switch (light.envelope)
        {
        case EffectLight::Envelope::Flash:
            envelope = std::exp(-light.age * 10.0f);
            break;
        case EffectLight::Envelope::Ember:
        {
            float t = light.lifetime > 0.0f ? light.age / light.lifetime : 0.0f;
            float fade = 1.0f - std::min(t, 1.0f);
            envelope = fade * fade;
            break;
        }
        case EffectLight::Envelope::Steady:
            break;
        }

        float strength = light.intensity * envelope *
                         effectLightFlicker(light.seed, light.age, 0.18f);
        if (strength < 0.5f)
            continue;

        pbr::PointLight gpuLight;
        gpuLight.position = light.position;
        gpuLight.color = light.color;
        gpuLight.strength = strength;
        gpuLight.zFar = light.radius;
        m_effectLightScratch.push_back(gpuLight);
    }

    // Keep the strongest lights as seen from the camera; the tile cull's
    // per-cluster list is small, and dozens of lights are plenty visually.
    constexpr std::size_t kMaxUploadedLights = 32;
    if (m_effectLightScratch.size() > kMaxUploadedLights)
    {
        std::partial_sort(
            m_effectLightScratch.begin(),
            m_effectLightScratch.begin() + kMaxUploadedLights,
            m_effectLightScratch.end(),
            [this](const pbr::PointLight &a, const pbr::PointLight &b)
            {
                float da = 1.0f + glm::length2(a.position - m_cameraPosition);
                float db = 1.0f + glm::length2(b.position - m_cameraPosition);
                return a.strength / da > b.strength / db;
            });
        m_effectLightScratch.resize(kMaxUploadedLights);
    }

    m_pbrPipeline->setPointLights(m_effectLightScratch);
}

void Renderer::clearEffects()
{
    if (m_sceneEffects)
    {
        m_sceneEffects->clear();
    }
    m_effectLights.clear();
}

void Renderer::setViewportSize(int width, int height)
{
    m_viewportWidth = width;
    m_viewportHeight = height;

    if (m_sceneEffects)
    {
        m_sceneEffects->setViewportSize(width, height);
    }

    if (isPBRActive())
    {
        m_pbrPipeline->resize(width, height);
    }
}

void Renderer::emitMissileExhaust(const glm::vec3 &start,
                                  const glm::vec3 &end,
                                  const glm::vec3 &forward,
                                  const glm::vec3 &carrierVelocity,
                                  float intensity)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->emitMissileExhaust(start, end, forward, carrierVelocity, intensity);
    }

    // Warm glow tracking the nozzle for a single frame (re-emitted while
    // the motor burns).
    EffectLight light;
    light.position = start;
    light.color = glm::vec3(1.0f, 0.55f, 0.22f);
    light.intensity = 120.0f * intensity;
    light.radius = 25.0f;
    light.seed = start.x + start.z;
    addEffectLight(light);
}

void Renderer::emitJetAfterburner(const glm::vec3 &start,
                                  const glm::vec3 &end,
                                  const glm::vec3 &forward,
                                  const glm::vec3 &carrierVelocity,
                                  float intensity)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->emitJetAfterburner(start, end, forward, carrierVelocity, intensity);
    }

    EffectLight light;
    light.position = start;
    light.color = glm::vec3(0.45f, 0.65f, 1.0f);
    light.intensity = 90.0f * intensity;
    light.radius = 20.0f;
    light.seed = start.x + start.y;
    addEffectLight(light);
}

void Renderer::emitFlareEffect(const glm::vec3 &start,
                               const glm::vec3 &end,
                               const glm::vec3 &carrierVelocity,
                               float heatFraction)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->emitFlareEffect(start, end, carrierVelocity, heatFraction);
    }

    // Burning magnesium: intense warm point light following the flare.
    EffectLight light;
    light.position = start;
    light.color = glm::vec3(1.0f, 0.78f, 0.5f);
    light.intensity = 250.0f * heatFraction;
    light.radius = 35.0f;
    light.seed = start.y + start.z;
    addEffectLight(light);
}

void Renderer::spawnMissileLaunchEffect(const glm::vec3 &position,
                                        const glm::vec3 &forward,
                                        const glm::vec3 &carrierVelocity,
                                        float intensity)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->spawnMissileLaunch(position, forward, carrierVelocity, intensity);
    }

    EffectLight flash;
    flash.position = position;
    flash.color = glm::vec3(1.0f, 0.72f, 0.42f);
    flash.intensity = 600.0f * intensity;
    flash.radius = 50.0f;
    flash.lifetime = 0.5f;
    flash.seed = position.x;
    flash.envelope = EffectLight::Envelope::Flash;
    addEffectLight(flash);
}

void Renderer::spawnLaunchGroundCloudEffect(const glm::vec3 &position,
                                            const glm::vec3 &up,
                                            float intensity)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->spawnLaunchGroundCloud(position, up, intensity);
    }
}

void Renderer::spawnExplosionEffect(const glm::vec3 &position,
                                    const glm::vec3 &velocityHint,
                                    float intensity)
{
    if (m_sceneEffects)
    {
        m_sceneEffects->spawnExplosion(position, velocityHint, intensity);
    }

    // Detonation flash: short, violent, far-reaching.
    EffectLight flash;
    flash.position = position;
    flash.color = glm::vec3(1.0f, 0.75f, 0.45f);
    flash.intensity = 2500.0f * intensity;
    flash.radius = 140.0f;
    flash.lifetime = 0.35f;
    flash.seed = position.x + position.y;
    flash.envelope = EffectLight::Envelope::Flash;
    addEffectLight(flash);

    // Lingering fireball ember glow.
    EffectLight ember;
    ember.position = position;
    ember.color = glm::vec3(1.0f, 0.45f, 0.12f);
    ember.intensity = 400.0f * intensity;
    ember.radius = 60.0f;
    ember.lifetime = 2.2f;
    ember.seed = position.y + position.z;
    ember.envelope = EffectLight::Envelope::Ember;
    addEffectLight(ember);
}
