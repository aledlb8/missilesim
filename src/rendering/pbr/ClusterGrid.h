#pragma once

#include "PBRLight.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

namespace pbr {

struct VolumeTileAABB
{
    glm::vec4 minPoint;
    glm::vec4 maxPoint;
};

struct ScreenToView
{
    glm::mat4 inverseProjectionMat;
    unsigned int tileSizes[4];
    unsigned int screenWidth;
    unsigned int screenHeight;
    float sliceScalingFactor;
    float sliceBiasFactor;
};

class ClusterGrid
{
public:
    static constexpr unsigned int kGridSizeX = 16;
    static constexpr unsigned int kGridSizeY = 9;
    static constexpr unsigned int kGridSizeZ = 24;
    static constexpr unsigned int kMaxLights = 1000;
    static constexpr unsigned int kMaxLightsPerTile = 50;

    ClusterGrid() = default;
    ~ClusterGrid();

    ClusterGrid(const ClusterGrid &) = delete;
    ClusterGrid &operator=(const ClusterGrid &) = delete;
    ClusterGrid(ClusterGrid &&other) noexcept;
    ClusterGrid &operator=(ClusterGrid &&other) noexcept;

    void initialize(unsigned int gridSizeX,
                    unsigned int gridSizeY,
                    unsigned int gridSizeZ,
                    unsigned int screenW,
                    unsigned int screenH,
                    float nearPlane,
                    float farPlane,
                    const glm::mat4 &projectionMatrix);

    void uploadLights(const std::vector<PointLight> &lights);

    void shutdown();

    unsigned int getGridSizeX() const { return m_gridSizeX; }
    unsigned int getGridSizeY() const { return m_gridSizeY; }
    unsigned int getGridSizeZ() const { return m_gridSizeZ; }
    unsigned int getNumClusters() const { return m_gridSizeX * m_gridSizeY * m_gridSizeZ; }

    GLuint getAABBVolumeGridSSBO() const { return m_AABBvolumeGridSSBO; }
    GLuint getScreenToViewSSBO() const { return m_screenToViewSSBO; }
    GLuint getLightSSBO() const { return m_lightSSBO; }
    GLuint getLightIndexListSSBO() const { return m_lightIndexListSSBO; }
    GLuint getLightGridSSBO() const { return m_lightGridSSBO; }
    GLuint getLightIndexGlobalCountSSBO() const { return m_lightIndexGlobalCountSSBO; }

private:
    void release();

    unsigned int m_gridSizeX = kGridSizeX;
    unsigned int m_gridSizeY = kGridSizeY;
    unsigned int m_gridSizeZ = kGridSizeZ;

    GLuint m_AABBvolumeGridSSBO = 0;
    GLuint m_screenToViewSSBO = 0;
    GLuint m_lightSSBO = 0;
    GLuint m_lightIndexListSSBO = 0;
    GLuint m_lightGridSSBO = 0;
    GLuint m_lightIndexGlobalCountSSBO = 0;
};

} // namespace pbr
