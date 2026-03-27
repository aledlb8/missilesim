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

void SceneEffects::createBuffers()
{
    static const std::array<float, 8> quadCorners = {
        -1.0f, -1.0f,
        1.0f, -1.0f,
        -1.0f, 1.0f,
        1.0f, 1.0f};

    static const std::array<float, 16> fullscreenQuad = {
        -1.0f, -1.0f, 0.0f, 0.0f,
        1.0f, -1.0f, 1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f};

    glGenBuffers(1, &m_quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER, quadCorners.size() * sizeof(float), quadCorners.data(), GL_STATIC_DRAW);

    glGenVertexArrays(1, &m_particleVAO);
    glBindVertexArray(m_particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, (void *)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &m_particleInstanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_particleInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(ParticleInstance) * 256, nullptr, GL_DYNAMIC_DRAW);

    const GLsizei particleStride = static_cast<GLsizei>(sizeof(ParticleInstance));
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, centerRotation));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, axisSizeX));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, color));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, params0));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, params1));
    glEnableVertexAttribArray(5);
    glVertexAttribDivisor(5, 1);

    glGenVertexArrays(1, &m_hazeVAO);
    glBindVertexArray(m_hazeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, (void *)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &m_hazeInstanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(HazeInstance) * 128, nullptr, GL_DYNAMIC_DRAW);

    const GLsizei hazeStride = static_cast<GLsizei>(sizeof(HazeInstance));
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, hazeStride, (void *)offsetof(HazeInstance, centerRotation));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, hazeStride, (void *)offsetof(HazeInstance, axisSizeX));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, hazeStride, (void *)offsetof(HazeInstance, params0));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glGenVertexArrays(1, &m_fullscreenVAO);
    glGenBuffers(1, &m_fullscreenVBO);
    glBindVertexArray(m_fullscreenVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_fullscreenVBO);
    glBufferData(GL_ARRAY_BUFFER, fullscreenQuad.size() * sizeof(float), fullscreenQuad.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void *)(sizeof(float) * 2));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SceneEffects::destroyBuffers()
{
    if (m_fullscreenVAO != 0)
    {
        glDeleteVertexArrays(1, &m_fullscreenVAO);
        m_fullscreenVAO = 0;
    }
    if (m_fullscreenVBO != 0)
    {
        glDeleteBuffers(1, &m_fullscreenVBO);
        m_fullscreenVBO = 0;
    }
    if (m_hazeVAO != 0)
    {
        glDeleteVertexArrays(1, &m_hazeVAO);
        m_hazeVAO = 0;
    }
    if (m_hazeInstanceVBO != 0)
    {
        glDeleteBuffers(1, &m_hazeInstanceVBO);
        m_hazeInstanceVBO = 0;
    }
    if (m_particleVAO != 0)
    {
        glDeleteVertexArrays(1, &m_particleVAO);
        m_particleVAO = 0;
    }
    if (m_particleInstanceVBO != 0)
    {
        glDeleteBuffers(1, &m_particleInstanceVBO);
        m_particleInstanceVBO = 0;
    }
    if (m_quadVBO != 0)
    {
        glDeleteBuffers(1, &m_quadVBO);
        m_quadVBO = 0;
    }
}

void SceneEffects::destroySceneFramebuffer()
{
    if (m_sceneDepthTexture != 0)
    {
        glDeleteTextures(1, &m_sceneDepthTexture);
        m_sceneDepthTexture = 0;
    }
    if (m_sceneColorTexture != 0)
    {
        glDeleteTextures(1, &m_sceneColorTexture);
        m_sceneColorTexture = 0;
    }
    if (m_sceneFramebuffer != 0)
    {
        glDeleteFramebuffers(1, &m_sceneFramebuffer);
        m_sceneFramebuffer = 0;
    }
    m_sceneFramebufferValid = false;
}

void SceneEffects::ensureSceneFramebuffer()
{
    if (m_sceneFramebufferValid || m_viewportWidth <= 0 || m_viewportHeight <= 0)
    {
        return;
    }

    destroySceneFramebuffer();

    glGenFramebuffers(1, &m_sceneFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_sceneFramebuffer);

    glGenTextures(1, &m_sceneColorTexture);
    glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_viewportWidth, m_viewportHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_sceneColorTexture, 0);

    glGenTextures(1, &m_sceneDepthTexture);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_viewportWidth, m_viewportHeight, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_sceneDepthTexture, 0);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);

    const GLenum framebufferStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (framebufferStatus != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "ERROR: Scene framebuffer incomplete: " << framebufferStatus << std::endl;
        destroySceneFramebuffer();
    }
    else
    {
        m_sceneFramebufferValid = true;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SceneEffects::ensureParticleInstanceCapacity(std::size_t instanceCount)
{
    if (instanceCount <= m_particleInstanceCapacity)
    {
        return;
    }

    std::size_t newCapacity = std::max<std::size_t>(m_particleInstanceCapacity, 256);
    while (newCapacity < instanceCount)
    {
        newCapacity *= 2;
    }

    m_particleInstanceCapacity = newCapacity;
    glBindBuffer(GL_ARRAY_BUFFER, m_particleInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, m_particleInstanceCapacity * sizeof(ParticleInstance), nullptr, GL_DYNAMIC_DRAW);
}

void SceneEffects::ensureHazeInstanceCapacity(std::size_t instanceCount)
{
    if (instanceCount <= m_hazeInstanceCapacity)
    {
        return;
    }

    std::size_t newCapacity = std::max<std::size_t>(m_hazeInstanceCapacity, 128);
    while (newCapacity < instanceCount)
    {
        newCapacity *= 2;
    }

    m_hazeInstanceCapacity = newCapacity;
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, m_hazeInstanceCapacity * sizeof(HazeInstance), nullptr, GL_DYNAMIC_DRAW);
}