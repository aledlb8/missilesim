#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace pbr {

class Shader;
struct Cube;

enum class CubeMapType
{
    Shadow,
    HDR,
    Prefilter
};

class CubeMap
{
public:
    CubeMap() = default;
    ~CubeMap();

    CubeMap(const CubeMap &) = delete;
    CubeMap &operator=(const CubeMap &) = delete;
    CubeMap(CubeMap &&other) noexcept;
    CubeMap &operator=(CubeMap &&other) noexcept;

    /// Allocate the cubemap texture on the GPU.
    void generateCubeMap(unsigned int w, unsigned int h, CubeMapType type);

    /// Diffuse irradiance convolution of an environment cubemap.
    void convolveCubeMap(GLuint envMapID, Shader &convolveShader);

    /// Pre-filtered specular environment map with roughness mip levels.
    void preFilterCubeMap(GLuint envMapID, GLuint captureRBO,
                          Shader &filterShader);

    /// Convert an equirectangular HDR texture to a cubemap.
    void equiRectangularToCubeMap(GLuint equirectMapID,
                                  unsigned int resolution,
                                  Shader &transformShader);

    GLuint textureID() const { return m_textureID; }
    unsigned int width() const { return m_width; }
    unsigned int height() const { return m_height; }

    static constexpr int numSidesInCube = 6;
    static const glm::mat4 captureViews[6];
    static const glm::mat4 captureProjection;

private:
    void ensureCube();

    GLuint m_textureID = 0;
    unsigned int m_width = 0;
    unsigned int m_height = 0;
    unsigned int m_maxMipLevels = 5;

    static Cube *s_cube;
};

} // namespace pbr
