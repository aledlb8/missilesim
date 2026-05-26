#include "ClusterGrid.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace pbr {

ClusterGrid::~ClusterGrid()
{
    release();
}

ClusterGrid::ClusterGrid(ClusterGrid &&other) noexcept
    : m_gridSizeX(other.m_gridSizeX),
      m_gridSizeY(other.m_gridSizeY),
      m_gridSizeZ(other.m_gridSizeZ),
      m_AABBvolumeGridSSBO(other.m_AABBvolumeGridSSBO),
      m_screenToViewSSBO(other.m_screenToViewSSBO),
      m_lightSSBO(other.m_lightSSBO),
      m_lightIndexListSSBO(other.m_lightIndexListSSBO),
      m_lightGridSSBO(other.m_lightGridSSBO),
      m_lightIndexGlobalCountSSBO(other.m_lightIndexGlobalCountSSBO)
{
    other.m_AABBvolumeGridSSBO = 0;
    other.m_screenToViewSSBO = 0;
    other.m_lightSSBO = 0;
    other.m_lightIndexListSSBO = 0;
    other.m_lightGridSSBO = 0;
    other.m_lightIndexGlobalCountSSBO = 0;
}

ClusterGrid &ClusterGrid::operator=(ClusterGrid &&other) noexcept
{
    if (this != &other)
    {
        release();

        m_gridSizeX = other.m_gridSizeX;
        m_gridSizeY = other.m_gridSizeY;
        m_gridSizeZ = other.m_gridSizeZ;
        m_AABBvolumeGridSSBO = other.m_AABBvolumeGridSSBO;
        m_screenToViewSSBO = other.m_screenToViewSSBO;
        m_lightSSBO = other.m_lightSSBO;
        m_lightIndexListSSBO = other.m_lightIndexListSSBO;
        m_lightGridSSBO = other.m_lightGridSSBO;
        m_lightIndexGlobalCountSSBO = other.m_lightIndexGlobalCountSSBO;

        other.m_AABBvolumeGridSSBO = 0;
        other.m_screenToViewSSBO = 0;
        other.m_lightSSBO = 0;
        other.m_lightIndexListSSBO = 0;
        other.m_lightGridSSBO = 0;
        other.m_lightIndexGlobalCountSSBO = 0;
    }
    return *this;
}

void ClusterGrid::release()
{
    GLuint buffers[] = {
        m_AABBvolumeGridSSBO,
        m_screenToViewSSBO,
        m_lightSSBO,
        m_lightIndexListSSBO,
        m_lightGridSSBO,
        m_lightIndexGlobalCountSSBO};

    for (auto &buf : buffers)
    {
        if (buf != 0)
        {
            glDeleteBuffers(1, &buf);
        }
    }

    m_AABBvolumeGridSSBO = 0;
    m_screenToViewSSBO = 0;
    m_lightSSBO = 0;
    m_lightIndexListSSBO = 0;
    m_lightGridSSBO = 0;
    m_lightIndexGlobalCountSSBO = 0;
}

void ClusterGrid::initialize(unsigned int gridSizeX,
                              unsigned int gridSizeY,
                              unsigned int gridSizeZ,
                              unsigned int screenW,
                              unsigned int screenH,
                              float nearPlane,
                              float farPlane,
                              const glm::mat4 &projectionMatrix)
{
    // Release any previously allocated SSBOs
    release();

    m_gridSizeX = gridSizeX;
    m_gridSizeY = gridSizeY;
    m_gridSizeZ = gridSizeZ;

    unsigned int sizeX = static_cast<unsigned int>(std::ceil(static_cast<float>(screenW) / static_cast<float>(gridSizeX)));
    unsigned int sizeY = static_cast<unsigned int>(std::ceil(static_cast<float>(screenH) / static_cast<float>(gridSizeY)));

    unsigned int numClusters = gridSizeX * gridSizeY * gridSizeZ;

    // 1. AABB volume grid SSBO (binding 1)
    {
        glGenBuffers(1, &m_AABBvolumeGridSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_AABBvolumeGridSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(numClusters * sizeof(VolumeTileAABB)),
                     nullptr,
                     GL_STATIC_COPY);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_AABBvolumeGridSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // 2. Screen-to-view SSBO (binding 2)
    {
        // Compute slice scaling and bias factors for logarithmic depth slicing
        float sliceScalingFactor = static_cast<float>(gridSizeZ) /
                                   std::log2(farPlane / nearPlane);
        float sliceBiasFactor = -(static_cast<float>(gridSizeZ) *
                                  std::log2(nearPlane) /
                                  std::log2(farPlane / nearPlane));

        ScreenToView screenToView{};
        screenToView.inverseProjectionMat = glm::inverse(projectionMatrix);
        screenToView.tileSizes[0] = gridSizeX;
        screenToView.tileSizes[1] = gridSizeY;
        screenToView.tileSizes[2] = gridSizeZ;
        screenToView.tileSizes[3] = sizeX;
        screenToView.screenWidth = screenW;
        screenToView.screenHeight = screenH;
        screenToView.sliceScalingFactor = sliceScalingFactor;
        screenToView.sliceBiasFactor = sliceBiasFactor;

        glGenBuffers(1, &m_screenToViewSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_screenToViewSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(sizeof(ScreenToView)),
                     &screenToView,
                     GL_STATIC_COPY);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_screenToViewSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // 3. Light SSBO (binding 3)
    {
        glGenBuffers(1, &m_lightSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_lightSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(kMaxLights * sizeof(GPULight)),
                     nullptr,
                     GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_lightSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // 4. Light index list SSBO (binding 4)
    {
        glGenBuffers(1, &m_lightIndexListSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_lightIndexListSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(numClusters * kMaxLightsPerTile * sizeof(unsigned int)),
                     nullptr,
                     GL_STATIC_COPY);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, m_lightIndexListSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // 5. Light grid SSBO (binding 5)
    //    Each cluster stores an offset and a count (2 uints)
    {
        glGenBuffers(1, &m_lightGridSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_lightGridSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(numClusters * 2 * sizeof(unsigned int)),
                     nullptr,
                     GL_STATIC_COPY);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, m_lightGridSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // 6. Light index global count SSBO (binding 6)
    {
        glGenBuffers(1, &m_lightIndexGlobalCountSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_lightIndexGlobalCountSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(sizeof(unsigned int)),
                     nullptr,
                     GL_STATIC_COPY);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, m_lightIndexGlobalCountSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }
}

void ClusterGrid::uploadLights(const std::vector<PointLight> &lights)
{
    if (m_lightSSBO == 0)
    {
        return;
    }

    unsigned int numLights = static_cast<unsigned int>(
        std::min(static_cast<size_t>(kMaxLights), lights.size()));

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_lightSSBO);

    GPULight *gpuLights = static_cast<GPULight *>(
        glMapBufferRange(GL_SHADER_STORAGE_BUFFER,
                         0,
                         static_cast<GLsizeiptr>(kMaxLights * sizeof(GPULight)),
                         GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT));

    if (gpuLights)
    {
        for (unsigned int i = 0; i < numLights; ++i)
        {
            gpuLights[i].position = glm::vec4(lights[i].position, 1.0f);
            gpuLights[i].color = glm::vec4(lights[i].color, 1.0f);
            gpuLights[i].enabled = 1;
            gpuLights[i].intensity = lights[i].strength;
            gpuLights[i].range = lights[i].zFar;
            gpuLights[i].padding = 0.0f;
        }

        // Mark remaining lights as disabled
        for (unsigned int i = numLights; i < kMaxLights; ++i)
        {
            gpuLights[i].position = glm::vec4(0.0f);
            gpuLights[i].color = glm::vec4(0.0f);
            gpuLights[i].enabled = 0;
            gpuLights[i].intensity = 0.0f;
            gpuLights[i].range = 0.0f;
            gpuLights[i].padding = 0.0f;
        }

        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void ClusterGrid::shutdown()
{
    release();
}

} // namespace pbr
