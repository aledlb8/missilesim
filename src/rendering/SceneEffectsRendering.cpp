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

void SceneEffects::submitEnginePlume(const glm::vec3 &nozzle, const glm::vec3 &direction,
                                     float radius, float throttle, bool rocket)
{
    if (!std::isfinite(radius) || radius <= 0.0001f || !std::isfinite(throttle) || throttle <= 0.01f ||
        !std::isfinite(glm::length(nozzle)) || !std::isfinite(glm::length(direction)) ||
        m_enginePlumes.size() >= 256) return;
    const float power = rocket ? saturate(throttle) : glm::smoothstep(0.45f, 1.0f, throttle);
    const float length = radius * (rocket ? glm::mix(18.0f, 40.0f, power) : glm::mix(1.6f, 14.0f, power));
    ParticleInstance instance{};
    instance.centerRotation = glm::vec4(nozzle, 0.0f);
    instance.axisSizeX = glm::vec4(safeNormalize(direction, glm::vec3(0, -1, 0)), radius);
    instance.color = glm::vec4(1.0f);
    instance.params0 = glm::vec4(length, m_effectTime, power, 1.0f);
    instance.params1 = glm::vec4(static_cast<float>(rocket ? ParticleMaterial::ROCKET_PLUME : ParticleMaterial::JET_PLUME),
                                radius * 71.0f, 0.0f, 0.0f);
    m_enginePlumes.push_back(instance);
}

void SceneEffects::renderParticlesToScene()
{
    if (!m_initialized || m_particleProgram == 0 || (m_particles.empty() && m_enginePlumes.empty()))
    {
        return;
    }

    std::vector<ParticleInstance> instances;
    instances.reserve(m_particles.size() + m_enginePlumes.size());

    struct SortableParticle
    {
        float viewDepth = 0.0f;
        ParticleInstance instance{};
    };
    std::vector<SortableParticle> sortedParticles;
    sortedParticles.reserve(m_particles.size() + m_enginePlumes.size());
    for (const auto &engine : m_enginePlumes)
    {
        const glm::vec3 center = glm::vec3(engine.centerRotation) + glm::vec3(engine.axisSizeX) * engine.params0.x * 0.5f;
        sortedParticles.push_back({-(m_view * glm::vec4(center, 1.0f)).z, engine});
    }

    for (const EffectParticle &particle : m_particles)
    {
        const float ageNorm = (particle.lifetime > 0.0f) ? saturate(particle.age / particle.lifetime) : 1.0f;
        if (ageNorm >= 1.0f)
        {
            continue;
        }

        const float size = glm::mix(particle.startSize, particle.endSize, ageNorm);
        if (!std::isfinite(size) || size <= 0.0001f)
        {
            continue;
        }

        ParticleInstance instance{};
        const bool axisAligned = particle.material == ParticleMaterial::FLAME ||
                                 particle.material == ParticleMaterial::SPARK ||
                                 particle.material == ParticleMaterial::DEBRIS ||
                                 particle.material == ParticleMaterial::SHOCK_DIAMOND;
        instance.centerRotation = glm::vec4(particle.position, axisAligned ? 0.0f : particle.rotation);
        instance.axisSizeX = glm::vec4(safeNormalize(particle.axis, particle.velocity), size);
        instance.color = particle.color;
        instance.params0 = glm::vec4(size * particle.stretch,
                                     ageNorm,
                                     particle.softness,
                                     particle.emissive);
        // Sort emission and extinction together: foreground smoke must obscure
        // fire behind it. Additive instances output zero alpha in the shader.
        instance.params1 = glm::vec4(static_cast<float>(particle.material), particle.seed,
                                     particle.blendMode == BlendMode::ADDITIVE ? 1.0f : 0.0f, 0.0f);
        const float viewDepth = -(m_view * glm::vec4(particle.position, 1.0f)).z;
        sortedParticles.push_back({viewDepth, instance});
    }

    std::stable_sort(sortedParticles.begin(), sortedParticles.end(), [](const SortableParticle &lhs, const SortableParticle &rhs)
              { return lhs.viewDepth > rhs.viewDepth; });
    for (const SortableParticle &entry : sortedParticles)
    {
        instances.push_back(entry.instance);
    }

    renderParticlePass(instances);
}

void SceneEffects::renderParticlePass(const std::vector<ParticleInstance> &instances)
{
    if (instances.empty())
    {
        return;
    }

    ensureParticleInstanceCapacity(instances.size());

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glUseProgram(m_particleProgram);
    const glm::mat4 inverseView = glm::inverse(m_view);
    const glm::mat4 inverseProjection = glm::inverse(m_projection);
    glUniformMatrix4fv(glGetUniformLocation(m_particleProgram, "inverseView"), 1, GL_FALSE, glm::value_ptr(inverseView));
    glUniformMatrix4fv(glGetUniformLocation(m_particleProgram, "inverseProjection"), 1, GL_FALSE, glm::value_ptr(inverseProjection));
    glUniform1f(glGetUniformLocation(m_particleProgram, "zNear"), m_projection[3][2] / (m_projection[2][2] - 1.0f));
    glUniform1f(glGetUniformLocation(m_particleProgram, "zFar"), m_projection[3][2] / (m_projection[2][2] + 1.0f));
    glUniform2f(glGetUniformLocation(m_particleProgram, "viewportSize"), static_cast<float>(m_viewportWidth), static_cast<float>(m_viewportHeight));
    glUniform3fv(glGetUniformLocation(m_particleProgram, "sunDirection"), 1, glm::value_ptr(m_sunDirection));
    glUniform3fv(glGetUniformLocation(m_particleProgram, "sunRadiance"), 1, glm::value_ptr(m_sunRadiance));
    const GLint viewLoc = glGetUniformLocation(m_particleProgram, "view");
    const GLint projectionLoc = glGetUniformLocation(m_particleProgram, "projection");
    const GLint cameraPosLoc = glGetUniformLocation(m_particleProgram, "cameraPos");
    if (viewLoc != -1)
    {
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_view));
    }
    if (projectionLoc != -1)
    {
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(m_projection));
    }
    if (cameraPosLoc != -1)
    {
        glUniform3fv(cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }

    // Both rendering paths sample detached depth, avoiding attachment feedback.
    GLuint depthTexture = m_externalDepthTexture;
    GLint drawFramebuffer = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFramebuffer);
    if (depthTexture == 0 && m_sceneFramebufferValid &&
        static_cast<GLuint>(drawFramebuffer) == m_sceneFramebuffer)
    {
        GLint readFramebuffer = 0;
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFramebuffer);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, m_sceneFramebuffer);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_sceneDepthSnapshot);
        glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, m_viewportWidth, m_viewportHeight);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(readFramebuffer));
        depthTexture = m_sceneDepthSnapshot;
    }
    const GLint depthFadeLoc = glGetUniformLocation(m_particleProgram, "depthFadeEnabled");
    if (depthTexture != 0)
    {
        // Recover the perspective near/far planes from the projection matrix
        // (glm::perspective: P[2][2] = -(f+n)/(f-n), P[3][2] = -2fn/(f-n)).
        const float p22 = m_projection[2][2];
        const float p32 = m_projection[3][2];
        const float nearPlane = p32 / (p22 - 1.0f);
        const float farPlane = p32 / (p22 + 1.0f);

        const GLint sceneDepthLoc = glGetUniformLocation(m_particleProgram, "sceneDepth");
        const GLint viewportSizeLoc = glGetUniformLocation(m_particleProgram, "viewportSize");
        const GLint zNearLoc = glGetUniformLocation(m_particleProgram, "zNear");
        const GLint zFarLoc = glGetUniformLocation(m_particleProgram, "zFar");

        if (sceneDepthLoc != -1)
        {
            glUniform1i(sceneDepthLoc, 0);
        }
        if (viewportSizeLoc != -1)
        {
            glUniform2f(viewportSizeLoc, static_cast<float>(m_viewportWidth), static_cast<float>(m_viewportHeight));
        }
        if (zNearLoc != -1)
        {
            glUniform1f(zNearLoc, nearPlane);
        }
        if (zFarLoc != -1)
        {
            glUniform1f(zFarLoc, farPlane);
        }
        if (depthFadeLoc != -1)
        {
            glUniform1i(depthFadeLoc, 1);
        }

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, depthTexture);
    }
    else if (depthFadeLoc != -1)
    {
        glUniform1i(depthFadeLoc, 0);
    }

    glBindVertexArray(m_particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_particleInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(ParticleInstance), instances.data());
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(instances.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void SceneEffects::renderHeatHazePass()
{
    const bool haveSceneColor = (m_externalSceneColor != 0) || m_sceneFramebufferValid;
    if (!haveSceneColor || m_hazeProgram == 0 || m_heatHazeSprites.empty())
    {
        return;
    }

    struct SortableHaze
    {
        float distanceSquared = 0.0f;
        HazeInstance instance{};
    };

    std::vector<SortableHaze> sortableInstances;
    sortableInstances.reserve(m_heatHazeSprites.size());

    for (const HeatHazeSprite &sprite : m_heatHazeSprites)
    {
        const float ageNorm = (sprite.lifetime > 0.0f) ? saturate(sprite.age / sprite.lifetime) : 1.0f;
        if (ageNorm >= 1.0f)
        {
            continue;
        }

        HazeInstance instance{};
        instance.centerRotation = glm::vec4(sprite.position, sprite.rotation);
        instance.axisSizeX = glm::vec4(safeNormalize(sprite.axis, sprite.velocity), sprite.radius);
        instance.params0 = glm::vec4(sprite.radius * sprite.stretch, ageNorm, sprite.strength, sprite.seed);
        sortableInstances.push_back({glm::length2(sprite.position - m_cameraPosition), instance});
    }

    if (sortableInstances.empty())
    {
        return;
    }

    std::sort(sortableInstances.begin(), sortableInstances.end(), [](const SortableHaze &lhs, const SortableHaze &rhs)
              { return lhs.distanceSquared > rhs.distanceSquared; });

    std::vector<HazeInstance> instances;
    instances.reserve(sortableInstances.size());
    for (const SortableHaze &entry : sortableInstances)
    {
        instances.push_back(entry.instance);
    }

    if (instances.empty())
    {
        return;
    }

    ensureHazeInstanceCapacity(instances.size());

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glUseProgram(m_hazeProgram);
    glUniform1f(glGetUniformLocation(m_hazeProgram, "zNear"), m_projection[3][2] / (m_projection[2][2] - 1.0f));
    glUniform1f(glGetUniformLocation(m_hazeProgram, "zFar"), m_projection[3][2] / (m_projection[2][2] + 1.0f));
    const GLint viewLoc = glGetUniformLocation(m_hazeProgram, "view");
    const GLint projectionLoc = glGetUniformLocation(m_hazeProgram, "projection");
    const GLint cameraPosLoc = glGetUniformLocation(m_hazeProgram, "cameraPos");
    const GLint sceneColorLoc = glGetUniformLocation(m_hazeProgram, "sceneColor");
    const GLint sceneDepthLoc = glGetUniformLocation(m_hazeProgram, "sceneDepth");
    const GLint viewportSizeLoc = glGetUniformLocation(m_hazeProgram, "viewportSize");

    if (viewLoc != -1)
    {
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_view));
    }
    if (projectionLoc != -1)
    {
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(m_projection));
    }
    if (cameraPosLoc != -1)
    {
        glUniform3fv(cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }
    if (sceneColorLoc != -1)
    {
        glUniform1i(sceneColorLoc, 0);
    }
    if (sceneDepthLoc != -1)
    {
        glUniform1i(sceneDepthLoc, 1);
    }
    if (viewportSizeLoc != -1)
    {
        glUniform2f(viewportSizeLoc, static_cast<float>(m_viewportWidth), static_cast<float>(m_viewportHeight));
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_externalSceneColor != 0 ? m_externalSceneColor : m_sceneColorTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_externalDepthTexture != 0 ? m_externalDepthTexture : m_sceneDepthTexture);

    glBindVertexArray(m_hazeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(HazeInstance), instances.data());
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(instances.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}
