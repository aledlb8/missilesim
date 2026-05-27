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
        // Execute the full PBR render pipeline (shadows → depth → cull → shade → skybox → resolve)
        m_pbrPipeline->executeRenderPass();

        // Render particles into the PBR resolved FBO
        if (m_sceneEffects)
        {
            m_sceneEffects->setExternalDepthTexture(m_pbrPipeline->getResolvedDepthTexture());
            m_pbrPipeline->bindResolvedFBO();
            m_sceneEffects->setCamera(m_cameraPosition, buildViewMatrix(), buildProjectionMatrix());
            m_sceneEffects->renderParticlesToScene();
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
}

void Renderer::clearEffects()
{
    if (m_sceneEffects)
    {
        m_sceneEffects->clear();
    }
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
}
