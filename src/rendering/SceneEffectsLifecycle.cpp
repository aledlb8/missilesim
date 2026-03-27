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

void SceneEffects::initialize()
{
    if (m_initialized)
    {
        return;
    }

    createShaders();
    createBuffers();
    ensureSceneFramebuffer();
    m_initialized = true;
}

void SceneEffects::shutdown()
{
    destroySceneFramebuffer();
    destroyBuffers();

    if (m_particleProgram != 0)
    {
        glDeleteProgram(m_particleProgram);
        m_particleProgram = 0;
    }
    if (m_hazeProgram != 0)
    {
        glDeleteProgram(m_hazeProgram);
        m_hazeProgram = 0;
    }
    if (m_compositeProgram != 0)
    {
        glDeleteProgram(m_compositeProgram);
        m_compositeProgram = 0;
    }

    m_particles.clear();
    m_heatHazeSprites.clear();
    m_initialized = false;
}

void SceneEffects::setViewportSize(int width, int height)
{
    const int clampedWidth = std::max(width, 1);
    const int clampedHeight = std::max(height, 1);
    if (m_viewportWidth == clampedWidth && m_viewportHeight == clampedHeight)
    {
        return;
    }

    m_viewportWidth = clampedWidth;
    m_viewportHeight = clampedHeight;
    destroySceneFramebuffer();
}

void SceneEffects::setCamera(const glm::vec3 &cameraPosition, const glm::mat4 &view, const glm::mat4 &projection)
{
    m_cameraPosition = cameraPosition;
    m_view = view;
    m_projection = projection;
}

void SceneEffects::beginScene(const glm::vec3 &clearColor)
{
    if (!m_initialized)
    {
        initialize();
    }

    ensureSceneFramebuffer();
    if (m_sceneFramebufferValid)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_sceneFramebuffer);
    }
    else
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    glViewport(0, 0, m_viewportWidth, m_viewportHeight);
    glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void SceneEffects::presentScene()
{
    if (!m_initialized)
    {
        return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportWidth, m_viewportHeight);

    if (m_sceneFramebufferValid && m_compositeProgram != 0)
    {
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        glUseProgram(m_compositeProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
        const GLint sceneSamplerLoc = glGetUniformLocation(m_compositeProgram, "sceneColor");
        if (sceneSamplerLoc != -1)
        {
            glUniform1i(sceneSamplerLoc, 0);
        }

        glBindVertexArray(m_fullscreenVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);

        renderHeatHazePass();

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }
}

void SceneEffects::clear()
{
    m_particles.clear();
    m_heatHazeSprites.clear();
}