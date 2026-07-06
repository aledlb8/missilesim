#include "Renderer.h"
#include "SceneEffects.h"

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

void Renderer::renderLine(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color)
{
    if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.z) ||
        !std::isfinite(end.x) || !std::isfinite(end.y) || !std::isfinite(end.z))
    {
        return;
    }

    m_debugLineVertices.push_back({start, color, 1.0f});
    m_debugLineVertices.push_back({end, color, 1.0f});
}

void Renderer::renderText(const glm::vec3 &position, const std::string &text, const glm::vec3 &color)
{
    // since we don' have a full text rendering system,
    // well use ImGui for this purpose in the UI layer.
}

void Renderer::renderPoint(const glm::vec3 &position, const glm::vec3 &color, float size)
{
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(size) || size <= 0.0f)
    {
        return;
    }

    m_debugPointVertices.push_back({position, color, size});
}

void Renderer::ensureDebugBufferCapacity(std::size_t vertexCount)
{
    if (vertexCount <= m_lineBufferCapacity)
    {
        return;
    }

    std::size_t newCapacity = std::max<std::size_t>(m_lineBufferCapacity, 256);
    while (newCapacity < vertexCount)
    {
        newCapacity *= 2;
    }

    m_lineBufferCapacity = newCapacity;
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, m_lineBufferCapacity * sizeof(DebugVertex), nullptr, GL_DYNAMIC_DRAW);
}

void Renderer::flushDebugPrimitives()
{
    // When PBR is active, defer debug rendering to presentSceneFrame
    // so lines are drawn on top of the tonemapped result.
    if (isPBRActive())
    {
        return;
    }

    flushDebugPrimitivesInternal();
}

void Renderer::flushDebugPrimitivesInternal()
{
    if (m_lineShaderProgram == 0 || (m_debugLineVertices.empty() && m_debugPointVertices.empty()))
    {
        clearDebugPrimitives();
        return;
    }

    // PBR path: draw anti-aliased instanced line quads into the HDR resolve
    // target, depth-tested against the scene so trajectories are occluded by
    // terrain and aircraft. glLineWidth is capped at 1 on core profiles, so
    // screen-space expanded quads are the only way to get wide smooth lines.
    if (isPBRActive() && m_aaLineProgram != 0)
    {
        flushAALinesInternal();
        return;
    }

    try
    {
        const glm::mat4 view = buildViewMatrix();
        const glm::mat4 projection = buildProjectionMatrix();

        glUseProgram(m_lineShaderProgram);
        if (m_lineViewLoc != -1)
        {
            glUniformMatrix4fv(m_lineViewLoc, 1, GL_FALSE, glm::value_ptr(view));
        }
        if (m_lineProjLoc != -1)
        {
            glUniformMatrix4fv(m_lineProjLoc, 1, GL_FALSE, glm::value_ptr(projection));
        }

        glBindVertexArray(m_lineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);

        if (!m_debugLineVertices.empty())
        {
            ensureDebugBufferCapacity(m_debugLineVertices.size());
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            m_debugLineVertices.size() * sizeof(DebugVertex),
                            m_debugLineVertices.data());
            glLineWidth(2.0f);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_debugLineVertices.size()));
            glLineWidth(1.0f);
        }

        if (!m_debugPointVertices.empty())
        {
            ensureDebugBufferCapacity(m_debugPointVertices.size());
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            m_debugPointVertices.size() * sizeof(DebugVertex),
                            m_debugPointVertices.data());
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_debugPointVertices.size()));
        }

        glBindVertexArray(0);
        glUseProgram(0);
    }
    catch (...)
    {
        std::cerr << "Error flushing debug primitives" << std::endl;
    }

    clearDebugPrimitives();
}

void Renderer::flushAALinesInternal()
{
    // Line segments as instanced AA quads.
    if (!m_debugLineVertices.empty())
    {
        m_lineInstances.clear();
        m_lineInstances.reserve(m_debugLineVertices.size() / 2);
        for (std::size_t i = 0; i + 1 < m_debugLineVertices.size(); i += 2)
        {
            LineInstance instance;
            instance.start = glm::vec4(m_debugLineVertices[i].position, 2.5f);
            instance.end = glm::vec4(m_debugLineVertices[i + 1].position, 0.0f);
            instance.color = glm::vec4(m_debugLineVertices[i].color, 1.0f);
            m_lineInstances.push_back(instance);
        }

        if (m_lineInstances.size() > m_aaLineInstanceCapacity)
        {
            std::size_t newCapacity = std::max<std::size_t>(m_aaLineInstanceCapacity, 256);
            while (newCapacity < m_lineInstances.size())
            {
                newCapacity *= 2;
            }
            m_aaLineInstanceCapacity = newCapacity;
            glBindBuffer(GL_ARRAY_BUFFER, m_aaLineInstanceVBO);
            glBufferData(GL_ARRAY_BUFFER, m_aaLineInstanceCapacity * sizeof(LineInstance),
                         nullptr, GL_DYNAMIC_DRAW);
        }

        const glm::mat4 viewProj = buildProjectionMatrix() * buildViewMatrix();

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);  // premultiplied
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);

        glUseProgram(m_aaLineProgram);
        if (m_aaLineViewProjLoc != -1)
        {
            glUniformMatrix4fv(m_aaLineViewProjLoc, 1, GL_FALSE, glm::value_ptr(viewProj));
        }
        if (m_aaLineViewportLoc != -1)
        {
            glUniform2f(m_aaLineViewportLoc,
                        static_cast<float>(m_viewportWidth),
                        static_cast<float>(m_viewportHeight));
        }

        glBindVertexArray(m_aaLineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_aaLineInstanceVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        m_lineInstances.size() * sizeof(LineInstance),
                        m_lineInstances.data());
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4,
                              static_cast<GLsizei>(m_lineInstances.size()));
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
    }

    // Point markers reuse the legacy program, depth-tested into the same target.
    if (!m_debugPointVertices.empty() && m_lineShaderProgram != 0)
    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_PROGRAM_POINT_SIZE);

        glUseProgram(m_lineShaderProgram);
        const glm::mat4 view = buildViewMatrix();
        const glm::mat4 projection = buildProjectionMatrix();
        if (m_lineViewLoc != -1)
        {
            glUniformMatrix4fv(m_lineViewLoc, 1, GL_FALSE, glm::value_ptr(view));
        }
        if (m_lineProjLoc != -1)
        {
            glUniformMatrix4fv(m_lineProjLoc, 1, GL_FALSE, glm::value_ptr(projection));
        }

        glBindVertexArray(m_lineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
        ensureDebugBufferCapacity(m_debugPointVertices.size());
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        m_debugPointVertices.size() * sizeof(DebugVertex),
                        m_debugPointVertices.data());
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_debugPointVertices.size()));
        glBindVertexArray(0);

        glDisable(GL_PROGRAM_POINT_SIZE);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
    }

    glUseProgram(0);
    clearDebugPrimitives();
}

void Renderer::clearDebugPrimitives()
{
    m_debugLineVertices.clear();
    m_debugPointVertices.clear();
}