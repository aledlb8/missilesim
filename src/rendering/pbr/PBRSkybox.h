#pragma once

#include "PBRCubeMap.h"
#include "PBRMeshPrimitives.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <filesystem>
#include <string>

namespace pbr {

class Shader;

class Skybox
{
public:
    Skybox() = default;
    ~Skybox();

    Skybox(const Skybox &) = delete;
    Skybox &operator=(const Skybox &) = delete;
    Skybox(Skybox &&other) noexcept;
    Skybox &operator=(Skybox &&other) noexcept;

    /// Load an HDR skybox and create the environment cubemap.
    /// @param skyboxName  Folder + base filename under assetDir/skyboxes/
    /// @param isHDR       Must be true for .hdr equirectangular maps
    /// @param resolution  Cubemap face resolution in pixels
    /// @param assetDir    Root asset directory (contains skyboxes/ subfolder)
    void setup(const std::string &skyboxName,
               bool isHDR,
               unsigned int resolution,
               const std::filesystem::path &assetDir);

    /// Convert the loaded equirectangular texture into the cubemap.
    void fillCubeMapWithTexture(Shader &transformShader);

    /// Render the skybox cube. Expects the shader already bound externally
    /// (caller should set the projection * view uniform). Pass the
    /// combined VP matrix (with translation removed) so the skybox stays
    /// at the origin.
    void draw(Shader &shader, const glm::mat4 &VP) const;

    GLuint cubeMapTextureID() const { return m_skyBoxCubeMap.textureID(); }
    const CubeMap &cubeMap() const { return m_skyBoxCubeMap; }

private:
    unsigned int m_resolution = 0;
    GLuint m_equirectangularMapID = 0;
    CubeMap m_skyBoxCubeMap;
    Cube m_cube;
};

} // namespace pbr
