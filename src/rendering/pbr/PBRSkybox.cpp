#include "PBRSkybox.h"
#include "PBRShader.h"

#include "stb/stb_image.h"

#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace pbr {

// ---------------------------------------------------------------------------
// RAII
// ---------------------------------------------------------------------------

Skybox::~Skybox()
{
    if (m_equirectangularMapID != 0)
        glDeleteTextures(1, &m_equirectangularMapID);
}

Skybox::Skybox(Skybox &&other) noexcept
    : m_resolution(other.m_resolution),
      m_equirectangularMapID(other.m_equirectangularMapID),
      m_skyBoxCubeMap(std::move(other.m_skyBoxCubeMap)),
      m_cube(std::move(other.m_cube))
{
    other.m_equirectangularMapID = 0;
    other.m_resolution = 0;
}

Skybox &Skybox::operator=(Skybox &&other) noexcept
{
    if (this != &other)
    {
        if (m_equirectangularMapID != 0)
            glDeleteTextures(1, &m_equirectangularMapID);

        m_resolution = other.m_resolution;
        m_equirectangularMapID = other.m_equirectangularMapID;
        m_skyBoxCubeMap = std::move(other.m_skyBoxCubeMap);
        m_cube = std::move(other.m_cube);

        other.m_equirectangularMapID = 0;
        other.m_resolution = 0;
    }
    return *this;
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------

void Skybox::setup(const std::string &skyboxName,
                   bool isHDR,
                   unsigned int resolution,
                   const std::filesystem::path &assetDir)
{
    m_resolution = resolution;

    // Build path: assetDir / skyboxes / skyboxName / (skyboxName + ".hdr")
    std::filesystem::path skyboxPath =
        assetDir / "skyboxes" / skyboxName / (skyboxName + ".hdr");

    // Load HDR equirectangular texture via stb_image.
    stbi_set_flip_vertically_on_load(true);

    int width = 0, height = 0, nrComponents = 0;
    float *data = nullptr;

    if (isHDR)
    {
        data = stbi_loadf(skyboxPath.string().c_str(),
                          &width, &height, &nrComponents, 0);
    }

    if (!data)
    {
        std::cerr << "PBR: Failed to load HDR skybox: " << skyboxPath << std::endl;
        return;
    }

    // Create the equirectangular GL texture.
    if (m_equirectangularMapID != 0)
        glDeleteTextures(1, &m_equirectangularMapID);

    glGenTextures(1, &m_equirectangularMapID);
    glBindTexture(GL_TEXTURE_2D, m_equirectangularMapID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0,
                 GL_RGB, GL_FLOAT, data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);

    // Generate the environment cubemap.
    m_skyBoxCubeMap.generateCubeMap(resolution, resolution, CubeMapType::HDR);

    // Prepare the cube used for rendering.
    m_cube.setup();
}

// ---------------------------------------------------------------------------
// fillCubeMapWithTexture
// ---------------------------------------------------------------------------

void Skybox::fillCubeMapWithTexture(Shader &transformShader)
{
    m_skyBoxCubeMap.equiRectangularToCubeMap(m_equirectangularMapID,
                                             m_resolution,
                                             transformShader);
}

// ---------------------------------------------------------------------------
// draw
// ---------------------------------------------------------------------------

void Skybox::draw(Shader &shader, const glm::mat4 &VP) const
{
    // Render inside the cube with depth <= so existing geometry at max depth
    // does not occlude the skybox.
    GLint prevDepthFunc = 0;
    glGetIntegerv(GL_DEPTH_FUNC, &prevDepthFunc);

    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);

    shader.use();
    shader.setMat4("VP", VP);
    shader.setInt("environmentMap", 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_skyBoxCubeMap.textureID());

    m_cube.draw();

    glDepthMask(GL_TRUE);
    glDepthFunc(static_cast<GLenum>(prevDepthFunc));
}

} // namespace pbr
