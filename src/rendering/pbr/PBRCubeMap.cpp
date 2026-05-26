#include "PBRCubeMap.h"
#include "PBRMeshPrimitives.h"
#include "PBRShader.h"

#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace pbr {

// ---------------------------------------------------------------------------
// Static data
// ---------------------------------------------------------------------------

const glm::mat4 CubeMap::captureProjection =
    glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);

const glm::mat4 CubeMap::captureViews[6] = {
    glm::lookAt(glm::vec3(0.0f), glm::vec3( 1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
    glm::lookAt(glm::vec3(0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
    glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
    glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
    glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
    glm::lookAt(glm::vec3(0.0f), glm::vec3( 0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
};

Cube *CubeMap::s_cube = nullptr;

// ---------------------------------------------------------------------------
// RAII
// ---------------------------------------------------------------------------

CubeMap::~CubeMap()
{
    if (m_textureID != 0)
        glDeleteTextures(1, &m_textureID);
}

CubeMap::CubeMap(CubeMap &&other) noexcept
    : m_textureID(other.m_textureID),
      m_width(other.m_width),
      m_height(other.m_height),
      m_maxMipLevels(other.m_maxMipLevels)
{
    other.m_textureID = 0;
    other.m_width = 0;
    other.m_height = 0;
}

CubeMap &CubeMap::operator=(CubeMap &&other) noexcept
{
    if (this != &other)
    {
        if (m_textureID != 0)
            glDeleteTextures(1, &m_textureID);

        m_textureID = other.m_textureID;
        m_width = other.m_width;
        m_height = other.m_height;
        m_maxMipLevels = other.m_maxMipLevels;

        other.m_textureID = 0;
        other.m_width = 0;
        other.m_height = 0;
    }
    return *this;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void CubeMap::ensureCube()
{
    if (s_cube == nullptr)
    {
        static Cube cube;
        cube.setup();
        s_cube = &cube;
    }
}

// ---------------------------------------------------------------------------
// generateCubeMap
// ---------------------------------------------------------------------------

void CubeMap::generateCubeMap(unsigned int w, unsigned int h, CubeMapType type)
{
    m_width = w;
    m_height = h;

    if (m_textureID != 0)
        glDeleteTextures(1, &m_textureID);

    glGenTextures(1, &m_textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_textureID);

    for (unsigned int i = 0; i < 6; ++i)
    {
        switch (type)
        {
        case CubeMapType::Shadow:
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,
                         GL_DEPTH_COMPONENT, w, h, 0,
                         GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
            break;

        case CubeMapType::HDR:
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,
                         GL_RGB16F, w, h, 0,
                         GL_RGB, GL_FLOAT, nullptr);
            break;

        case CubeMapType::Prefilter:
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,
                         GL_RGB16F, w, h, 0,
                         GL_RGB, GL_FLOAT, nullptr);
            break;
        }
    }

    if (type == CubeMapType::Shadow)
    {
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    }
    else
    {
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

        if (type == CubeMapType::Prefilter)
        {
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER,
                            GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
        }
        else
        {
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        }
    }
}

// ---------------------------------------------------------------------------
// convolveCubeMap  --  diffuse irradiance
// ---------------------------------------------------------------------------

void CubeMap::convolveCubeMap(GLuint envMapID, Shader &convolveShader)
{
    ensureCube();

    GLuint captureFBO, captureRBO;
    glGenFramebuffers(1, &captureFBO);
    glGenRenderbuffers(1, &captureRBO);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, captureRBO);

    convolveShader.use();
    convolveShader.setInt("environmentMap", 0);
    convolveShader.setMat4("projection", captureProjection);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, envMapID);

    glViewport(0, 0, m_width, m_height);
    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);

    for (unsigned int i = 0; i < 6; ++i)
    {
        convolveShader.setMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                               m_textureID, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        s_cube->draw();
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteRenderbuffers(1, &captureRBO);
    glDeleteFramebuffers(1, &captureFBO);
}

// ---------------------------------------------------------------------------
// preFilterCubeMap  --  split-sum specular prefilter
// ---------------------------------------------------------------------------

void CubeMap::preFilterCubeMap(GLuint envMapID, GLuint captureRBO,
                               Shader &filterShader)
{
    ensureCube();

    GLuint captureFBO;
    glGenFramebuffers(1, &captureFBO);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, captureRBO);

    filterShader.use();
    filterShader.setInt("environmentMap", 0);
    filterShader.setMat4("projection", captureProjection);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, envMapID);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);

    for (unsigned int mip = 0; mip < m_maxMipLevels; ++mip)
    {
        unsigned int mipWidth  = static_cast<unsigned int>(m_width  * std::pow(0.5, mip));
        unsigned int mipHeight = static_cast<unsigned int>(m_height * std::pow(0.5, mip));
        if (mipWidth  == 0) mipWidth  = 1;
        if (mipHeight == 0) mipHeight = 1;

        glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                              mipWidth, mipHeight);

        glViewport(0, 0, mipWidth, mipHeight);

        float roughness = static_cast<float>(mip) /
                          static_cast<float>(m_maxMipLevels - 1);
        filterShader.setFloat("roughness", roughness);

        for (unsigned int i = 0; i < 6; ++i)
        {
            filterShader.setMat4("view", captureViews[i]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                                   m_textureID, mip);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            s_cube->draw();
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &captureFBO);
}

// ---------------------------------------------------------------------------
// equiRectangularToCubeMap
// ---------------------------------------------------------------------------

void CubeMap::equiRectangularToCubeMap(GLuint equirectMapID,
                                       unsigned int resolution,
                                       Shader &transformShader)
{
    ensureCube();

    GLuint captureFBO, captureRBO;
    glGenFramebuffers(1, &captureFBO);
    glGenRenderbuffers(1, &captureRBO);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                          resolution, resolution);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, captureRBO);

    transformShader.use();
    transformShader.setInt("equirectangularMap", 0);
    transformShader.setMat4("projection", captureProjection);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, equirectMapID);

    glViewport(0, 0, resolution, resolution);
    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);

    for (unsigned int i = 0; i < 6; ++i)
    {
        transformShader.setMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                               m_textureID, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        s_cube->draw();
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteRenderbuffers(1, &captureRBO);
    glDeleteFramebuffers(1, &captureFBO);
}

} // namespace pbr
