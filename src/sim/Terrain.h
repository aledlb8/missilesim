#pragma once

// One ground surface for everything that touches the ground: physics
// contacts, the fighter's crash test, the AI's height floor, line of sight and
// the rendered mesh. A non-flat terrain is a heightfield: samples on a square
// grid, each cell split into two triangles along the (i,j)-(i+1,j+1) diagonal.
// bedHeightAt() interpolates on those triangles, and the renderer meshes the
// same samples with the same split, so the drawn land is the land bodies meet.
//
// Where the terrain has water, its surface is solid too: heightAt() is the
// higher of the land and the water level, and that is what physics, the
// fighter and the AI fly against. The renderer draws the water as a plane at
// that level over the same land.
//
// Every shape is a fictional landscape for gameplay and tests, not terrain
// data. Each keeps a flat apron at the base height around the origin (the
// launch site).
//   Flat       the base height everywhere.
//   Ridge      one ridge line beyond the launch site; dry. A test fixture.
//   Mountains  a valley ringed by mountains, a lake in the valley, and the
//              sea past the mountains. The default scenery.

#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    enum class TerrainKind : std::uint8_t
    {
        Flat,
        Ridge,
        Mountains,
    };

    const char *terrainKindName(TerrainKind kind);
    // "flat", "ridge" or "mountains"; false (out untouched) for anything else.
    bool parseTerrainKind(const char *name, TerrainKind &out);

    struct TerrainConfig
    {
        TerrainKind kind = TerrainKind::Mountains;
        float baseHeightM = 0.0f;    // the launch apron and the valley floor
        float apronRadiusM = 600.0f; // flat ground kept around the origin
        float hillAmplitudeM = 60.0f; // rolling ground on the valley floor
        std::uint32_t seed = 1;      // varies the mountains, not the layout

        // Ridge. Bearing is measured from +Z toward +X, like a heading.
        float ridgeHeightM = 320.0f;    // tallest crest above the base
        float ridgeDistanceM = 2400.0f; // origin to crest line
        float ridgeHalfWidthM = 700.0f; // crest to foot
        float ridgeBearingDeg = 0.0f;

        // Mountains.
        float mountainHeightM = 2100.0f; // tallest peaks above the base
        float valleyRadiusM = 3200.0f;   // where the mountains start to rise
        float coastRadiusM = 12500.0f;   // where the land starts to sink into the sea
        float waterBelowBaseM = 12.0f;   // the lake and the sea stand this far below the base

        // Heightfield grid.
        float extentHalfM = 16000.0f;
        float cellSizeM = 64.0f;

        bool operator==(const TerrainConfig &other) const;
        bool operator!=(const TerrainConfig &other) const { return !(*this == other); }
    };

    class Terrain
    {
    public:
        // Most cells along one side of the grid; a finer request is coarsened.
        static constexpr int kMaxCellsPerSide = 1024;
        // The two triangles of cell (i, j) as (di, dj) corner offsets. The
        // renderer indexes its mesh with this table and bedHeightAt
        // interpolates on it, so the drawn split is the collision split.
        static constexpr int kCellTriangles[6][2] = {{0, 0}, {0, 1}, {1, 1}, {0, 0}, {1, 1}, {1, 0}};

        Terrain();
        explicit Terrain(const TerrainConfig &config);

        // Building a large heightfield takes a moment, and every world (the
        // game's, each harness run) asks for the same few. This returns a
        // shared, immutable terrain for the config, built once.
        static std::shared_ptr<const Terrain> shared(const TerrainConfig &config);

        const TerrainConfig &config() const { return m_config; }
        TerrainKind kind() const { return m_config.kind; }
        bool isFlat() const { return m_cells == 0 && !m_hasWater; }
        float baseHeight() const { return m_config.baseHeightM; }
        // Highest point of the solid surface (land or water).
        float maxHeight() const { return m_maxHeight; }

        bool hasWater() const { return m_hasWater; }
        float waterLevel() const { return m_waterLevel; }

        // The solid surface: land, or the water above it.
        float heightAt(float x, float z) const;
        float heightAt(const glm::vec3 &position) const { return heightAt(position.x, position.z); }
        // The land alone (the lake and sea bed under water).
        float bedHeightAt(float x, float z) const;
        // Upward unit normal of the solid surface under (x, z).
        glm::vec3 normalAt(float x, float z) const;
        float heightAbove(const glm::vec3 &position) const { return position.y - heightAt(position.x, position.z); }
        bool isWater(float x, float z) const { return m_hasWater && bedHeightAt(x, z) < m_waterLevel; }

        // First point where the segment from a to b meets or enters the
        // solid surface. fraction is along a->b (0 when a is already inside).
        // Exact on the triangulated land and the water plane, not sampled.
        bool segmentHit(const glm::vec3 &a, const glm::vec3 &b, float *fraction = nullptr) const;
        // True when nothing of the terrain lies between the two points.
        bool lineOfSight(const glm::vec3 &a, const glm::vec3 &b) const { return !segmentHit(a, b); }

        // Grid for meshing. Valid when gridCells() > 0. Sample (i, j) sits at
        // x = gridOrigin() + i * cellSize(), z = gridOrigin() + j * cellSize().
        // Beyond the grid the land is outerBedHeight().
        int gridCells() const { return m_cells; }
        float cellSize() const { return m_cellSize; }
        float gridOrigin() const { return m_origin; }
        float sample(int i, int j) const { return m_heights[static_cast<std::size_t>(j) * (m_cells + 1) + i]; }
        const std::vector<float> &samples() const { return m_heights; }
        float outerBedHeight() const { return m_outerBed; }

    private:
        float shapeHeight(float x, float z) const;
        float ridgeShape(float x, float z) const;
        float mountainShape(float x, float z) const;

        TerrainConfig m_config;
        int m_cells = 0;
        float m_cellSize = 0.0f;
        float m_origin = 0.0f;
        float m_maxHeight = 0.0f;
        float m_outerBed = 0.0f;
        bool m_hasWater = false;
        float m_waterLevel = 0.0f;
        std::vector<float> m_heights;
    };
}
