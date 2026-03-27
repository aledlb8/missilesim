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
    if (m_lineShaderProgram == 0 || (m_debugLineVertices.empty() && m_debugPointVertices.empty()))
    {
        clearDebugPrimitives();
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

void Renderer::clearDebugPrimitives()
{
    m_debugLineVertices.clear();
    m_debugPointVertices.clear();
}