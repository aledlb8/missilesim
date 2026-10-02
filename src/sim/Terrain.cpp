#include "Terrain.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>

namespace missilesim::sim
{
    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;

        float smoothstep(float edge0, float edge1, float x)
        {
            if (edge1 <= edge0)
            {
                return x < edge0 ? 0.0f : 1.0f;
            }
            const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        }

        // Integer hash to [0, 1). Pure integer arithmetic, so a seed gives the
        // same landscape on every machine.
        float hash2(std::int32_t x, std::int32_t y, std::uint32_t seed)
        {
            std::uint32_t h = static_cast<std::uint32_t>(x) * 0x8da6b343u ^ static_cast<std::uint32_t>(y) * 0xd8163841u ^
                              seed * 0xcb1ab31fu;
            h ^= h >> 13;
            h *= 0x5bd1e995u;
            h ^= h >> 15;
            return static_cast<float>(h & 0x00ffffffu) / 16777216.0f;
        }

        // Smooth value noise in [0, 1].
        float valueNoise(float x, float y, std::uint32_t seed)
        {
            const float fx = std::floor(x);
            const float fy = std::floor(y);
            const auto ix = static_cast<std::int32_t>(fx);
            const auto iy = static_cast<std::int32_t>(fy);
            const float tx = x - fx;
            const float ty = y - fy;
            const float ux = tx * tx * tx * (tx * (tx * 6.0f - 15.0f) + 10.0f);
            const float uy = ty * ty * ty * (ty * (ty * 6.0f - 15.0f) + 10.0f);
            const float a = hash2(ix, iy, seed);
            const float b = hash2(ix + 1, iy, seed);
            const float c = hash2(ix, iy + 1, seed);
            const float d = hash2(ix + 1, iy + 1, seed);
            return a + (b - a) * ux + (c - a) * uy + (a - b - c + d) * ux * uy;
        }

        // Fractal sum in [0, 1].
        float fbm(float x, float y, int octaves, std::uint32_t seed)
        {
            float sum = 0.0f;
            float amplitude = 0.5f;
            float norm = 0.0f;
            for (int octave = 0; octave < octaves; ++octave)
            {
                sum += amplitude * valueNoise(x, y, seed + static_cast<std::uint32_t>(octave) * 101u);
                norm += amplitude;
                // Rotate each octave a little so the grid never lines up.
                const float rx = 1.6f * x + 1.2f * y;
                const float ry = -1.2f * x + 1.6f * y;
                x = rx + 17.3f;
                y = ry - 9.1f;
                amplitude *= 0.5f;
            }
            return sum / norm;
        }

        // Ridged multifractal in [0, 1]: sharp crests, smooth valleys, and
        // finer detail where the larger octaves are already high.
        float ridged(float x, float y, int octaves, std::uint32_t seed)
        {
            float sum = 0.0f;
            float amplitude = 0.5f;
            float weight = 1.0f;
            float norm = 0.0f;
            for (int octave = 0; octave < octaves; ++octave)
            {
                float n = 1.0f - std::abs(2.0f * valueNoise(x, y, seed + static_cast<std::uint32_t>(octave) * 131u) - 1.0f);
                n *= n;
                n *= weight;
                weight = std::clamp(n * 1.8f, 0.0f, 1.0f);
                sum += amplitude * n;
                norm += amplitude;
                const float rx = 1.7f * x + 1.1f * y;
                const float ry = -1.1f * x + 1.7f * y;
                x = rx + 5.7f;
                y = ry + 3.3f;
                amplitude *= 0.5f;
            }
            return sum / norm;
        }

        // Adds every t in (0, 1) where s0 + (s1 - s0) t crosses an integer
        // in [lo, hi].
        void addIntegerCrossings(float s0, float s1, float lo, float hi, std::vector<float> &out)
        {
            const float ds = s1 - s0;
            if (std::abs(ds) < 1.0e-9f)
            {
                return;
            }
            const float first = std::max(std::ceil(std::min(s0, s1)), lo);
            const float last = std::min(std::floor(std::max(s0, s1)), hi);
            for (float k = first; k <= last; k += 1.0f)
            {
                const float t = (k - s0) / ds;
                if (t > 0.0f && t < 1.0f)
                {
                    out.push_back(t);
                }
            }
        }

        // The lake: an ellipse in the valley, clear of the launch apron.
        constexpr float kLakeCenterX = 2300.0f;
        constexpr float kLakeCenterZ = -2100.0f;
        constexpr float kLakeRadiusAlong = 1500.0f;
        constexpr float kLakeRadiusAcross = 850.0f;
        constexpr float kLakeAngle = 0.6f;
        constexpr float kLakeDepthM = 40.0f;
        constexpr float kSeaDepthM = 80.0f;
    }

    const char *terrainKindName(TerrainKind kind)
    {
        switch (kind)
        {
        case TerrainKind::Ridge:
            return "ridge";
        case TerrainKind::Mountains:
            return "mountains";
        case TerrainKind::Flat:
            break;
        }
        return "flat";
    }

    bool parseTerrainKind(const char *name, TerrainKind &out)
    {
        if (name == nullptr)
        {
            return false;
        }
        for (const TerrainKind kind : {TerrainKind::Flat, TerrainKind::Ridge, TerrainKind::Mountains})
        {
            if (std::strcmp(name, terrainKindName(kind)) == 0)
            {
                out = kind;
                return true;
            }
        }
        return false;
    }

    bool TerrainConfig::operator==(const TerrainConfig &other) const
    {
        return kind == other.kind && baseHeightM == other.baseHeightM && apronRadiusM == other.apronRadiusM &&
               hillAmplitudeM == other.hillAmplitudeM && seed == other.seed && ridgeHeightM == other.ridgeHeightM &&
               ridgeDistanceM == other.ridgeDistanceM && ridgeHalfWidthM == other.ridgeHalfWidthM &&
               ridgeBearingDeg == other.ridgeBearingDeg && mountainHeightM == other.mountainHeightM &&
               valleyRadiusM == other.valleyRadiusM && coastRadiusM == other.coastRadiusM &&
               waterBelowBaseM == other.waterBelowBaseM && extentHalfM == other.extentHalfM &&
               cellSizeM == other.cellSizeM;
    }

    std::shared_ptr<const Terrain> Terrain::shared(const TerrainConfig &config)
    {
        // A short list is enough: a game switches between a couple of
        // terrains, and the harness reuses one or two.
        static std::mutex mutex;
        static std::vector<std::shared_ptr<const Terrain>> recent;
        constexpr std::size_t kKept = 4;

        const std::lock_guard<std::mutex> lock(mutex);
        for (auto it = recent.begin(); it != recent.end(); ++it)
        {
            if ((*it)->config() == config)
            {
                std::shared_ptr<const Terrain> found = *it;
                recent.erase(it);
                recent.insert(recent.begin(), found);
                return found;
            }
        }
        auto built = std::make_shared<const Terrain>(config);
        recent.insert(recent.begin(), built);
        if (recent.size() > kKept)
        {
            recent.pop_back();
        }
        return built;
    }

    Terrain::Terrain() : Terrain(TerrainConfig{TerrainKind::Flat})
    {
    }

    Terrain::Terrain(const TerrainConfig &config) : m_config(config)
    {
        m_maxHeight = m_config.baseHeightM;
        m_outerBed = m_config.baseHeightM;
        if (m_config.kind == TerrainKind::Flat)
        {
            return;
        }

        if (m_config.kind == TerrainKind::Mountains)
        {
            m_hasWater = true;
            m_waterLevel = m_config.baseHeightM - std::max(m_config.waterBelowBaseM, 0.1f);
            // Past the grid lies open sea.
            m_outerBed = m_waterLevel - kSeaDepthM;
        }

        const float extent = std::max(m_config.extentHalfM, 1.0f);
        const float cellRequest = std::max(m_config.cellSizeM, 1.0f);
        m_cells = std::clamp(static_cast<int>(std::ceil(2.0f * extent / cellRequest)), 2, kMaxCellsPerSide);
        m_cellSize = 2.0f * extent / static_cast<float>(m_cells);
        m_origin = -extent;

        m_heights.resize(static_cast<std::size_t>(m_cells + 1) * (m_cells + 1));
        for (int j = 0; j <= m_cells; ++j)
        {
            for (int i = 0; i <= m_cells; ++i)
            {
                const bool edge = i == 0 || j == 0 || i == m_cells || j == m_cells;
                const float x = m_origin + static_cast<float>(i) * m_cellSize;
                const float z = m_origin + static_cast<float>(j) * m_cellSize;
                // Edge samples are pinned to the land beyond the grid so the
                // two meet without a step.
                const float height = edge ? m_outerBed : shapeHeight(x, z);
                m_heights[static_cast<std::size_t>(j) * (m_cells + 1) + i] = height;
                m_maxHeight = std::max(m_maxHeight, height);
            }
        }
        if (m_hasWater)
        {
            m_maxHeight = std::max(m_maxHeight, m_waterLevel);
        }
    }

    float Terrain::shapeHeight(float x, float z) const
    {
        return m_config.kind == TerrainKind::Mountains ? mountainShape(x, z) : ridgeShape(x, z);
    }

    float Terrain::ridgeShape(float x, float z) const
    {
        const float bearing = m_config.ridgeBearingDeg * (kPi / 180.0f);
        // across: toward the crest; along: down the crest line.
        const float across = x * std::sin(bearing) + z * std::cos(bearing);
        const float along = x * std::cos(bearing) - z * std::sin(bearing);

        // The crest wanders and its height varies along its length.
        const float crestLine = m_config.ridgeDistanceM + 180.0f * std::sin(along / 1300.0f);
        const float crestHeight = m_config.ridgeHeightM *
                                  (0.75f + 0.25f * std::sin(along / 900.0f + 0.7f)) *
                                  (0.88f + 0.12f * std::sin(along / 370.0f));
        const float halfWidth = std::max(m_config.ridgeHalfWidthM, 1.0f);
        const float q = std::abs(across - crestLine) / halfWidth;
        const float ridge = q < 1.0f ? crestHeight * 0.5f * (1.0f + std::cos(kPi * q)) : 0.0f;

        // Rolling ground, never below the base.
        const float roll = 0.6f * std::sin(x / 430.0f) * std::cos(z / 510.0f) + 0.4f * std::sin((x + z) / 260.0f + 1.3f);
        const float hills = m_config.hillAmplitudeM * 0.5f * (1.0f + roll);

        const float apron = std::max(m_config.apronRadiusM, 0.0f);
        const float fromOrigin = std::sqrt(x * x + z * z);
        const float apronMask = smoothstep(apron, apron * 2.0f + 1.0f, fromOrigin);

        const float extent = -m_origin;
        const float fromEdge = extent - std::max(std::abs(x), std::abs(z));
        const float edgeMask = smoothstep(0.0f, std::min(1500.0f, extent * 0.25f), fromEdge);

        return m_config.baseHeightM + apronMask * edgeMask * (ridge + hills);
    }

    float Terrain::mountainShape(float x, float z) const
    {
        const std::uint32_t seed = m_config.seed * 7919u + 13u;
        const float base = m_config.baseHeightM;
        const float fromOrigin = std::sqrt(x * x + z * z);

        // Warp the ring so the valley edge and the coast are not circles.
        const float warpX = 1100.0f * (fbm(x / 5000.0f + 11.3f, z / 5000.0f - 7.1f, 3, seed + 1u) - 0.5f) * 2.0f;
        const float warpZ = 1100.0f * (fbm(x / 5000.0f - 3.7f, z / 5000.0f + 5.2f, 3, seed + 2u) - 0.5f) * 2.0f;
        const float wx = x + warpX;
        const float wz = z + warpZ;
        const float warped = std::sqrt(wx * wx + wz * wz);

        const float coastInner = m_config.coastRadiusM;
        const float coastOuter = m_config.coastRadiusM + 2600.0f;
        const float sea = smoothstep(coastInner, coastOuter, warped);

        // Mountains: rise from the valley edge, crest, and fall to the coast.
        float mountains = 0.0f;
        const float rise = smoothstep(m_config.valleyRadiusM, m_config.valleyRadiusM + 4300.0f, warped);
        if (rise > 0.0f && sea < 1.0f)
        {
            const float peaks = ridged(wx / 5200.0f, wz / 5200.0f, 6, seed + 3u);
            const float massif = fbm(wx / 9000.0f + 2.1f, wz / 9000.0f - 4.4f, 2, seed + 4u);
            mountains = m_config.mountainHeightM * rise * (0.25f + 0.75f * peaks) * (0.55f + 0.65f * massif);
        }

        // Rolling valley floor, flattening onto the launch apron.
        const float apron = std::max(m_config.apronRadiusM, 0.0f);
        const float hills = m_config.hillAmplitudeM * fbm(x / 1500.0f, z / 1500.0f, 4, seed + 5u) *
                            smoothstep(apron, apron * 3.5f + 1.0f, fromOrigin);

        // The lake basin, below the water level at its middle.
        const float c = std::cos(kLakeAngle);
        const float s = std::sin(kLakeAngle);
        const float lx = x - kLakeCenterX;
        const float lz = z - kLakeCenterZ;
        const float along = (lx * c + lz * s) / kLakeRadiusAlong;
        const float across = (-lx * s + lz * c) / kLakeRadiusAcross;
        // Bays and points along the shore so the lake is not an ellipse.
        const float shore = 0.55f * (fbm(x / 1100.0f + 4.2f, z / 1100.0f - 1.7f, 3, seed + 6u) - 0.5f) +
                            0.18f * (fbm(x / 260.0f, z / 260.0f, 2, seed + 7u) - 0.5f);
        const float lakeDistance = std::sqrt(along * along + across * across) + shore;
        const float lake = (kLakeDepthM + m_config.hillAmplitudeM + m_config.waterBelowBaseM) *
                           (1.0f - smoothstep(0.35f, 1.0f, lakeDistance));

        float height = base + (mountains + hills) * (1.0f - sea) - lake;
        // Below the coast the land drops to the sea floor.
        height = height * (1.0f - sea) + (m_waterLevel - kSeaDepthM) * sea;

        // The launch apron stays exactly at the base.
        const float apronMask = smoothstep(apron, apron * 2.0f + 1.0f, fromOrigin);
        return base + (height - base) * apronMask;
    }

    float Terrain::bedHeightAt(float x, float z) const
    {
        if (m_cells == 0)
        {
            return m_config.baseHeightM;
        }
        const float u = (x - m_origin) / m_cellSize;
        const float w = (z - m_origin) / m_cellSize;
        if (!(u >= 0.0f && w >= 0.0f && u <= static_cast<float>(m_cells) && w <= static_cast<float>(m_cells)))
        {
            return m_outerBed;
        }
        const int i = std::min(static_cast<int>(u), m_cells - 1);
        const int j = std::min(static_cast<int>(w), m_cells - 1);
        const float fx = u - static_cast<float>(i);
        const float fz = w - static_cast<float>(j);
        const float h00 = sample(i, j);
        const float h10 = sample(i + 1, j);
        const float h01 = sample(i, j + 1);
        const float h11 = sample(i + 1, j + 1);
        if (fz >= fx)
        {
            // Triangle (i,j) (i,j+1) (i+1,j+1).
            return h00 + fz * (h01 - h00) + fx * (h11 - h01);
        }
        // Triangle (i,j) (i+1,j+1) (i+1,j).
        return h00 + fx * (h10 - h00) + fz * (h11 - h10);
    }

    float Terrain::heightAt(float x, float z) const
    {
        const float bed = bedHeightAt(x, z);
        return m_hasWater ? std::max(bed, m_waterLevel) : bed;
    }

    glm::vec3 Terrain::normalAt(float x, float z) const
    {
        if (m_cells == 0 || isWater(x, z))
        {
            return glm::vec3(0.0f, 1.0f, 0.0f);
        }
        const float u = (x - m_origin) / m_cellSize;
        const float w = (z - m_origin) / m_cellSize;
        if (!(u >= 0.0f && w >= 0.0f && u <= static_cast<float>(m_cells) && w <= static_cast<float>(m_cells)))
        {
            return glm::vec3(0.0f, 1.0f, 0.0f);
        }
        const int i = std::min(static_cast<int>(u), m_cells - 1);
        const int j = std::min(static_cast<int>(w), m_cells - 1);
        const float fx = u - static_cast<float>(i);
        const float fz = w - static_cast<float>(j);
        const float h00 = sample(i, j);
        const float h10 = sample(i + 1, j);
        const float h01 = sample(i, j + 1);
        const float h11 = sample(i + 1, j + 1);
        float slopeX = 0.0f;
        float slopeZ = 0.0f;
        if (fz >= fx)
        {
            slopeX = (h11 - h01) / m_cellSize;
            slopeZ = (h01 - h00) / m_cellSize;
        }
        else
        {
            slopeX = (h10 - h00) / m_cellSize;
            slopeZ = (h11 - h10) / m_cellSize;
        }
        return glm::normalize(glm::vec3(-slopeX, 1.0f, -slopeZ));
    }

    bool Terrain::segmentHit(const glm::vec3 &a, const glm::vec3 &b, float *fraction) const
    {
        // The clearance to the land and to the water are each linear between
        // crossings of grid lines and cell diagonals, so testing both at
        // those breakpoints finds every contact exactly.
        const auto landClearance = [&](float t) {
            const glm::vec3 p = a + (b - a) * t;
            return p.y - bedHeightAt(p.x, p.z);
        };
        const auto waterClearance = [&](float t) { return m_hasWater ? (a.y + (b.y - a.y) * t) - m_waterLevel : 1.0f; };

        float previousLand = landClearance(0.0f);
        float previousWater = waterClearance(0.0f);
        if (previousLand < 0.0f || previousWater < 0.0f)
        {
            if (fraction != nullptr)
            {
                *fraction = 0.0f;
            }
            return true;
        }
        // Wholly above the highest point: nothing to meet.
        if (a.y > m_maxHeight && b.y > m_maxHeight)
        {
            return false;
        }

        std::vector<float> breaks;
        if (m_cells > 0)
        {
            const float cells = static_cast<float>(m_cells);
            const float u0 = (a.x - m_origin) / m_cellSize;
            const float u1 = (b.x - m_origin) / m_cellSize;
            const float w0 = (a.z - m_origin) / m_cellSize;
            const float w1 = (b.z - m_origin) / m_cellSize;
            addIntegerCrossings(u0, u1, 0.0f, cells, breaks);
            addIntegerCrossings(w0, w1, 0.0f, cells, breaks);
            addIntegerCrossings(u0 - w0, u1 - w1, -cells, cells, breaks);
            std::sort(breaks.begin(), breaks.end());
        }
        breaks.push_back(1.0f);

        // Where a linear clearance first goes negative inside [t0, t1].
        const auto crossing = [](float t0, float t1, float c0, float c1) {
            const float span = c0 - c1;
            const float local = span > 0.0f ? c0 / span : 0.0f;
            return t0 + (t1 - t0) * std::clamp(local, 0.0f, 1.0f);
        };

        float previousT = 0.0f;
        for (const float t : breaks)
        {
            const float land = landClearance(t);
            const float water = waterClearance(t);
            if (land < 0.0f || water < 0.0f)
            {
                if (fraction != nullptr)
                {
                    float hit = t;
                    if (land < 0.0f)
                    {
                        hit = std::min(hit, crossing(previousT, t, previousLand, land));
                    }
                    if (water < 0.0f)
                    {
                        hit = std::min(hit, crossing(previousT, t, previousWater, water));
                    }
                    *fraction = hit;
                }
                return true;
            }
            previousT = t;
            previousLand = land;
            previousWater = water;
        }
        return false;
    }
}
