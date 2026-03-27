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

void SceneEffects::renderParticlesToScene()
{
    if (!m_initialized || m_particleProgram == 0 || m_particles.empty())
    {
        return;
    }

    std::vector<ParticleInstance> alphaInstances;
    std::vector<ParticleInstance> additiveInstances;
    alphaInstances.reserve(m_particles.size());
    additiveInstances.reserve(m_particles.size());

    struct SortableParticle
    {
        float distanceSquared = 0.0f;
        ParticleInstance instance{};
    };
    std::vector<SortableParticle> sortableAlpha;
    sortableAlpha.reserve(m_particles.size());

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
        instance.centerRotation = glm::vec4(particle.position, particle.rotation);
        instance.axisSizeX = glm::vec4(safeNormalize(particle.axis, particle.velocity), size);
        instance.color = particle.color;
        instance.params0 = glm::vec4(size * particle.stretch,
                                     ageNorm,
                                     particle.softness,
                                     particle.emissive);
        instance.params1 = glm::vec4(static_cast<float>(particle.material), particle.seed, 0.0f, 0.0f);

        if (particle.blendMode == BlendMode::ALPHA)
        {
            sortableAlpha.push_back({glm::length2(particle.position - m_cameraPosition), instance});
        }
        else
        {
            additiveInstances.push_back(instance);
        }
    }

    std::sort(sortableAlpha.begin(), sortableAlpha.end(), [](const SortableParticle &lhs, const SortableParticle &rhs)
              { return lhs.distanceSquared > rhs.distanceSquared; });
    alphaInstances.reserve(sortableAlpha.size());
    for (const SortableParticle &entry : sortableAlpha)
    {
        alphaInstances.push_back(entry.instance);
    }

    renderParticlePass(alphaInstances, BlendMode::ALPHA);
    renderParticlePass(additiveInstances, BlendMode::ADDITIVE);
}

void SceneEffects::renderParticlePass(const std::vector<ParticleInstance> &instances, BlendMode blendMode)
{
    if (instances.empty())
    {
        return;
    }

    ensureParticleInstanceCapacity(instances.size());

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    if (blendMode == BlendMode::ALPHA)
    {
        glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    }
    else
    {
        glBlendFunc(GL_ONE, GL_ONE);
    }

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glUseProgram(m_particleProgram);
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
    if (!m_sceneFramebufferValid || m_hazeProgram == 0 || m_heatHazeSprites.empty())
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
    glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);

    glBindVertexArray(m_hazeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(HazeInstance), instances.data());
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(instances.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}