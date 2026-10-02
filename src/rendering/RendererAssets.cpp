#include "Renderer.h"
#include "SceneEffects.h"
#include "pbr/PBRPipeline.h"

#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"
#include "../sim/Terrain.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

namespace
{
    struct ObjVertexRef
    {
        int position = 0;
        int normal = 0;
        bool hasNormal = false;
    };

    int resolveObjIndex(int rawIndex, std::size_t count)
    {
        if (rawIndex > 0)
        {
            return rawIndex - 1;
        }
        if (rawIndex < 0)
        {
            return static_cast<int>(count) + rawIndex;
        }
        return -1;
    }

    bool parseObjVertexRef(const std::string &token, ObjVertexRef &result)
    {
        std::stringstream tokenStream(token);
        std::string positionToken;
        std::string texcoordToken;
        std::string normalToken;

        if (!std::getline(tokenStream, positionToken, '/') || positionToken.empty())
        {
            return false;
        }

        std::getline(tokenStream, texcoordToken, '/');
        std::getline(tokenStream, normalToken, '/');

        try
        {
            result.position = std::stoi(positionToken);
            if (!normalToken.empty())
            {
                result.normal = std::stoi(normalToken);
                result.hasNormal = true;
            }
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    glm::vec3 normalizeOrFallback(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        if (glm::length2(vector) > 0.000001f)
        {
            return glm::normalize(vector);
        }

        if (glm::length2(fallback) > 0.000001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 rotateAroundAxis(const glm::vec3 &vector, const glm::vec3 &axis, float angleRadians)
    {
        const glm::vec3 normalizedAxis = normalizeOrFallback(axis, glm::vec3(0.0f, 1.0f, 0.0f));
        const float cosine = std::cos(angleRadians);
        const float sine = std::sin(angleRadians);
        return (vector * cosine) +
               (glm::cross(normalizedAxis, vector) * sine) +
               (normalizedAxis * glm::dot(normalizedAxis, vector) * (1.0f - cosine));
    }
}

std::filesystem::path Renderer::resolveAssetPath(const std::string &relativePath) const
{
    const std::filesystem::path requested(relativePath);
    const std::filesystem::path sourceAssetRoot(MISSILESIM_SOURCE_ASSET_DIR);

    const std::filesystem::path candidates[] = {
        requested,
        std::filesystem::current_path() / requested,
        std::filesystem::current_path() / "assets" / requested,
        sourceAssetRoot.empty() ? std::filesystem::path() : sourceAssetRoot / requested};

    for (const auto &candidate : candidates)
    {
        if (!candidate.empty() && std::filesystem::exists(candidate))
        {
            return candidate;
        }
    }

    return {};
}

void Renderer::normalizeMesh(std::vector<Vertex> &vertices, float targetExtent,
                             std::vector<ExhaustSocket> *sockets) const
{
    if (vertices.empty() || targetExtent <= 0.0f)
    {
        return;
    }

    glm::vec3 minCorner(std::numeric_limits<float>::max());
    glm::vec3 maxCorner(std::numeric_limits<float>::lowest());

    for (const Vertex &vertex : vertices)
    {
        minCorner = glm::min(minCorner, vertex.position);
        maxCorner = glm::max(maxCorner, vertex.position);
    }

    const glm::vec3 center = (minCorner + maxCorner) * 0.5f;
    const glm::vec3 extents = maxCorner - minCorner;
    const float maxExtent = std::max(extents.x, std::max(extents.y, extents.z));
    if (maxExtent <= 0.0001f)
    {
        return;
    }

    const float scale = targetExtent / maxExtent;
    if (sockets)
    {
        for (auto &socket : *sockets)
        {
            socket.position = (socket.position - center) * scale;
            socket.radius *= scale;
        }
    }
    for (Vertex &vertex : vertices)
    {
        vertex.position = (vertex.position - center) * scale;
        if (glm::length2(vertex.normal) > 0.000001f)
        {
            vertex.normal = glm::normalize(vertex.normal);
        }
        else
        {
            vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }
}

bool Renderer::loadObjModel(const std::string &relativePath,
                            std::vector<Vertex> &vertices,
                            std::vector<unsigned int> &indices,
                            const glm::vec3 &baseColor,
                            const glm::mat4 &preTransform,
                            float targetExtent,
                            std::vector<ExhaustSocket> *sockets)
{
    vertices.clear();
    indices.clear();
    if (sockets) sockets->clear();

    const std::filesystem::path assetPath = resolveAssetPath(relativePath);
    if (assetPath.empty())
    {
        return false;
    }

    std::ifstream input(assetPath);
    if (!input.is_open())
    {
        std::cerr << "Failed to open model asset: " << assetPath << std::endl;
        return false;
    }

    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec3> colors;
    std::vector<glm::vec2> materials;
    glm::vec2 currentMaterial(0.2f, 0.45f);
    const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(preTransform)));

    std::string line;
    while (std::getline(input, line))
    {
        if (sockets && line.rfind("# exhaust ", 0) == 0)
        {
            ExhaustSocket socket;
            std::istringstream stream(line.substr(10));
            if (stream >> socket.position.x >> socket.position.y >> socket.position.z >>
                          socket.direction.x >> socket.direction.y >> socket.direction.z >> socket.radius &&
                std::isfinite(glm::length(socket.position)) && std::isfinite(glm::length(socket.direction)) &&
                std::isfinite(socket.radius) && socket.radius > 0.0f && glm::length2(socket.direction) > 1e-8f &&
                sockets->size() < 8)
            {
                socket.position = glm::vec3(preTransform * glm::vec4(socket.position, 1.0f));
                socket.direction = glm::normalize(glm::mat3(preTransform) * socket.direction);
                socket.radius *= glm::length(glm::vec3(preTransform[0]));
                sockets->push_back(socket);
            }
            continue;
        }
        // Optional material metadata from our Blender exporter. Standard OBJ
        // readers ignore this comment; ordinary OBJ files retain defaults.
        if (line.rfind("# pbr ", 0) == 0)
        {
            std::istringstream materialStream(line.substr(6));
            glm::vec2 value;
            if (materialStream >> value.x >> value.y &&
                std::isfinite(value.x) && std::isfinite(value.y))
            {
                currentMaterial = glm::clamp(value, glm::vec2(0.0f, 0.04f), glm::vec2(1.0f));
            }
        }
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream stream(line);
        std::string prefix;
        stream >> prefix;

        if (prefix == "v")
        {
            glm::vec3 position(0.0f);
            stream >> position.x >> position.y >> position.z;
            positions.push_back(position);
            glm::vec3 color = baseColor;
            glm::vec3 vertexColor;
            if (stream >> vertexColor.r >> vertexColor.g >> vertexColor.b &&
                std::isfinite(vertexColor.r) && std::isfinite(vertexColor.g) && std::isfinite(vertexColor.b))
            {
                color = glm::clamp(vertexColor, glm::vec3(0.0f), glm::vec3(1.0f));
            }
            colors.push_back(color);
            materials.push_back(currentMaterial);
        }
        else if (prefix == "vn")
        {
            glm::vec3 normal(0.0f, 1.0f, 0.0f);
            stream >> normal.x >> normal.y >> normal.z;
            normals.push_back(glm::normalize(normal));
        }
        else if (prefix == "f")
        {
            std::vector<ObjVertexRef> faceRefs;
            std::string token;
            while (stream >> token)
            {
                ObjVertexRef ref;
                if (parseObjVertexRef(token, ref))
                {
                    faceRefs.push_back(ref);
                }
            }

            if (faceRefs.size() < 3)
            {
                continue;
            }

            for (std::size_t i = 1; i + 1 < faceRefs.size(); ++i)
            {
                const ObjVertexRef triangleRefs[3] = {faceRefs[0], faceRefs[i], faceRefs[i + 1]};
                glm::vec3 transformedPositions[3];
                glm::vec3 transformedNormals[3];
                bool validTriangle = true;
                bool hasTriangleNormals = true;

                for (int j = 0; j < 3; ++j)
                {
                    const int positionIndex = resolveObjIndex(triangleRefs[j].position, positions.size());
                    if (positionIndex < 0 || positionIndex >= static_cast<int>(positions.size()))
                    {
                        validTriangle = false;
                        hasTriangleNormals = false;
                        break;
                    }

                    transformedPositions[j] = glm::vec3(preTransform * glm::vec4(positions[static_cast<std::size_t>(positionIndex)], 1.0f));

                    if (triangleRefs[j].hasNormal)
                    {
                        const int normalIndex = resolveObjIndex(triangleRefs[j].normal, normals.size());
                        if (normalIndex >= 0 && normalIndex < static_cast<int>(normals.size()))
                        {
                            transformedNormals[j] = glm::normalize(normalMatrix * normals[static_cast<std::size_t>(normalIndex)]);
                        }
                        else
                        {
                            hasTriangleNormals = false;
                        }
                    }
                    else
                    {
                        hasTriangleNormals = false;
                    }
                }

                if (!validTriangle)
                {
                    continue;
                }

                glm::vec3 faceNormal = glm::cross(transformedPositions[1] - transformedPositions[0],
                                                  transformedPositions[2] - transformedPositions[0]);
                if (glm::length2(faceNormal) <= 0.000001f)
                {
                    faceNormal = glm::vec3(0.0f, 1.0f, 0.0f);
                }
                else
                {
                    faceNormal = glm::normalize(faceNormal);
                }

                for (int j = 0; j < 3; ++j)
                {
                    const auto index = static_cast<std::size_t>(resolveObjIndex(triangleRefs[j].position, positions.size()));
                    vertices.push_back({transformedPositions[j], hasTriangleNormals ? transformedNormals[j] : faceNormal,
                                        colors[index], materials[index]});
                    indices.push_back(static_cast<unsigned int>(vertices.size() - 1));
                }
            }
        }
    }

    if (vertices.empty())
    {
        if (sockets) sockets->clear();
        std::cerr << "Loaded empty model asset: " << assetPath << std::endl;
        return false;
    }

    normalizeMesh(vertices, targetExtent, sockets);
    return true;
}

void Renderer::createSimpleCube()
{
    // Clear previous vertices and indices
    m_vertices.clear();
    m_indices.clear();

    // Generate a simple cube
    m_vertices = {
        // Front face
        {{-0.5f, -0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}}, // 0
        {{0.5f, -0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},  // 1
        {{0.5f, 0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},   // 2
        {{-0.5f, 0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},  // 3

        // Back face
        {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}}, // 4
        {{0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},  // 5
        {{0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},   // 6
        {{-0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},  // 7

        // Top face
        {{-0.5f, 0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, // 8
        {{0.5f, 0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},  // 9
        {{0.5f, 0.5f, 0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},   // 10
        {{-0.5f, 0.5f, 0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},  // 11

        // Bottom face
        {{-0.5f, -0.5f, -0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}}, // 12
        {{0.5f, -0.5f, -0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},  // 13
        {{0.5f, -0.5f, 0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},   // 14
        {{-0.5f, -0.5f, 0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},  // 15

        // Right face
        {{0.5f, -0.5f, 0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}},  // 16
        {{0.5f, -0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}}, // 17
        {{0.5f, 0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}},  // 18
        {{0.5f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}},   // 19

        // Left face
        {{-0.5f, -0.5f, 0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}},  // 20
        {{-0.5f, -0.5f, -0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}}, // 21
        {{-0.5f, 0.5f, -0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}},  // 22
        {{-0.5f, 0.5f, 0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}}    // 23
    };

    // Indices for drawing triangles
    m_indices = {
        // Front face
        0, 1, 2, 2, 3, 0,

        // Back face
        4, 5, 6, 6, 7, 4,

        // Top face
        8, 9, 10, 10, 11, 8,

        // Bottom face
        12, 13, 14, 14, 15, 12,

        // Right face
        16, 17, 18, 18, 19, 16,

        // Left face
        20, 21, 22, 22, 23, 20};
}

void Renderer::createMissileModel()
{
    // Clear previous vertices and indices
    m_vertices.clear();
    m_indices.clear();

    if (loadObjModel("models/missile.obj",
                     m_vertices,
                     m_indices,
                     glm::vec3(0.78f, 0.79f, 0.82f),
                     glm::mat4(1.0f),
                     2.0f, &m_missileExhaustSockets))
    {
        return;
    }

    // Simplified missile model (elongated cylinder with nose cone)
    const int segments = 12;
    const float radius = 0.2f;
    const float height = 1.0f;

    // Colors
    glm::vec3 bodyColor(0.7f, 0.7f, 0.7f);
    glm::vec3 noseColor(0.9f, 0.1f, 0.1f);
    glm::vec3 finColor(0.3f, 0.3f, 0.3f);

    // Generate missile body (cylinder)
    for (int i = 0; i <= segments; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        // Calculate normal
        glm::vec3 normal = glm::normalize(glm::vec3(x, 0.0f, z));

        // Bottom vertex
        m_vertices.push_back({{x, -height, z}, normal, bodyColor});

        // Top vertex
        m_vertices.push_back({{x, height, z}, normal, bodyColor});

        // Create indices for cylinder sides
        if (i < segments)
        {
            unsigned int bottomLeft = i * 2;
            unsigned int bottomRight = (i * 2 + 2) % (segments * 2 + 2);
            unsigned int topLeft = i * 2 + 1;
            unsigned int topRight = (i * 2 + 3) % (segments * 2 + 2);

            // First triangle
            m_indices.push_back(bottomLeft);
            m_indices.push_back(bottomRight);
            m_indices.push_back(topRight);

            // Second triangle
            m_indices.push_back(bottomLeft);
            m_indices.push_back(topRight);
            m_indices.push_back(topLeft);
        }
    }

    // Create top cap (nose cone)
    unsigned int centerIndex = m_vertices.size();
    m_vertices.push_back({{0.0f, height + radius, 0.0f}, {0.0f, 1.0f, 0.0f}, noseColor});

    for (int i = 0; i < segments; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        unsigned int current = i * 2 + 1;
        unsigned int next = ((i + 1) % segments) * 2 + 1;

        m_indices.push_back(centerIndex);
        m_indices.push_back(current);
        m_indices.push_back(next);
    }

    // Create bottom cap
    centerIndex = m_vertices.size();
    m_vertices.push_back({{0.0f, -height, 0.0f}, {0.0f, -1.0f, 0.0f}, bodyColor});

    for (int i = 0; i < segments; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        unsigned int current = i * 2;
        unsigned int next = ((i + 1) % segments) * 2;

        m_indices.push_back(centerIndex);
        m_indices.push_back(next);
        m_indices.push_back(current);
    }

    // Create fins
    // For simplicity, we'll add 4 triangular fins at the bottom
    const int numFins = 4;
    const float finHeight = 0.4f;
    const float finLength = 0.8f;

    for (int i = 0; i < numFins; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(numFins);

        glm::vec3 finNormal = glm::normalize(glm::vec3(cos(angle + glm::pi<float>() / 4.0f), 0.0f, sin(angle + glm::pi<float>() / 4.0f)));

        // Base of the fin at the body
        glm::vec3 base1 = glm::vec3(radius * cos(angle), -height + 0.1f, radius * sin(angle));
        glm::vec3 base2 = glm::vec3(radius * cos(angle), -height + 0.1f + finLength, radius * sin(angle));

        // Tip of the fin
        glm::vec3 tip = glm::vec3(radius * cos(angle) + finHeight * finNormal.x,
                                  -height + 0.1f + finLength / 2.0f,
                                  radius * sin(angle) + finHeight * finNormal.z);

        // Add vertices
        unsigned int baseIndex1 = m_vertices.size();
        m_vertices.push_back({base1, finNormal, finColor});

        unsigned int baseIndex2 = m_vertices.size();
        m_vertices.push_back({base2, finNormal, finColor});

        unsigned int tipIndex = m_vertices.size();
        m_vertices.push_back({tip, finNormal, finColor});

        // Add indices for the triangular fin
        m_indices.push_back(baseIndex1);
        m_indices.push_back(baseIndex2);
        m_indices.push_back(tipIndex);
    }
}

void Renderer::createFloor()
{
    // Clear previous floor vertices and indices
    m_floorVertices.clear();
    m_floorIndices.clear();

    const glm::vec3 scrubColor(0.30f, 0.34f, 0.26f);
    const glm::vec3 dryGrassColor(0.42f, 0.39f, 0.26f);
    const glm::vec3 distantEarthColor(0.26f, 0.28f, 0.23f);
    const glm::vec3 rockColor(0.40f, 0.38f, 0.34f);
    const glm::vec3 crestColor(0.47f, 0.45f, 0.36f);

    auto groundColor = [&](float xPos, float zPos, float fadeExtent)
    {
        const float radialT = glm::clamp(glm::length(glm::vec2(xPos, zPos)) / fadeExtent, 0.0f, 1.0f);

        const float broadNoise = 0.5f + 0.5f * std::sin(xPos * 0.0034f + std::cos(zPos * 0.0017f) * 1.4f);
        const float grassNoise = 0.5f + 0.5f * std::sin(xPos * 0.019f) * std::cos(zPos * 0.014f);
        glm::vec3 color = glm::mix(scrubColor, dryGrassColor, broadNoise * 0.65f + grassNoise * 0.25f);
        color = glm::mix(color, distantEarthColor, radialT * 0.55f);

        // Natural terrain mottling only - no runway/service-lane pavement,
        // which formed a dark cross across the launch site.
        const float mottling = 0.5f + 0.5f * std::sin((xPos - zPos) * 0.006f);
        return glm::mix(color, glm::vec3(0.22f, 0.27f, 0.21f), mottling * 0.16f);
    };

    const missilesim::sim::Terrain *terrain = m_terrain.get();
    const float baseHeight = terrain != nullptr ? terrain->baseHeight() : 0.0f;

    if (terrain == nullptr || terrain->isFlat())
    {
        const float size = m_groundHalfExtent * 2.0f;
        const int gridSize = std::clamp(static_cast<int>(size / 80.0f), 64, 128);
        const float cellSize = size / gridSize;

        for (int z = 0; z <= gridSize; z++)
        {
            for (int x = 0; x <= gridSize; x++)
            {
                const float xPos = -size / 2.0f + x * cellSize;
                const float zPos = -size / 2.0f + z * cellSize;
                m_floorVertices.push_back({{xPos, baseHeight, zPos},
                                           {0.0f, 1.0f, 0.0f},
                                           groundColor(xPos, zPos, size * 0.5f)});
            }
        }

        for (int z = 0; z < gridSize; z++)
        {
            for (int x = 0; x < gridSize; x++)
            {
                unsigned int topLeft = z * (gridSize + 1) + x;
                unsigned int topRight = topLeft + 1;
                unsigned int bottomLeft = (z + 1) * (gridSize + 1) + x;
                unsigned int bottomRight = bottomLeft + 1;

                m_floorIndices.push_back(topLeft);
                m_floorIndices.push_back(bottomLeft);
                m_floorIndices.push_back(bottomRight);

                m_floorIndices.push_back(topLeft);
                m_floorIndices.push_back(bottomRight);
                m_floorIndices.push_back(topRight);
            }
        }
        return;
    }

    // Heightfield: one vertex per terrain sample, and the same diagonal split
    // Terrain::heightAt interpolates on, so the drawn triangles are the
    // collision triangles. Normals are smoothed for shading only.
    const int cells = terrain->gridCells();
    const float cell = terrain->cellSize();
    const float origin = terrain->gridOrigin();
    const float domainHalf = -origin;
    const float fadeExtent = std::max(m_groundHalfExtent, domainHalf);
    const float relief = std::max(terrain->maxHeight() - baseHeight, 1.0f);
    auto sampleClamped = [&](int i, int j)
    {
        return terrain->sample(std::clamp(i, 0, cells), std::clamp(j, 0, cells));
    };

    for (int j = 0; j <= cells; ++j)
    {
        for (int i = 0; i <= cells; ++i)
        {
            const float xPos = origin + static_cast<float>(i) * cell;
            const float zPos = origin + static_cast<float>(j) * cell;
            const float height = terrain->sample(i, j);
            const float slopeX = (sampleClamped(i + 1, j) - sampleClamped(i - 1, j)) / (2.0f * cell);
            const float slopeZ = (sampleClamped(i, j + 1) - sampleClamped(i, j - 1)) / (2.0f * cell);
            const glm::vec3 normal = glm::normalize(glm::vec3(-slopeX, 1.0f, -slopeZ));

            glm::vec3 color = groundColor(xPos, zPos, fadeExtent);
            const float steepness = glm::clamp((1.0f - normal.y) * 6.0f, 0.0f, 1.0f);
            const float elevation = glm::clamp((height - baseHeight) / relief, 0.0f, 1.0f);
            color = glm::mix(color, crestColor, elevation * elevation * 0.45f);
            color = glm::mix(color, rockColor, steepness * 0.7f);
            m_floorVertices.push_back({{xPos, height, zPos}, normal, color});
        }
    }
    for (int j = 0; j < cells; ++j)
    {
        for (int i = 0; i < cells; ++i)
        {
            for (const auto &corner : missilesim::sim::Terrain::kCellTriangles)
            {
                m_floorIndices.push_back(static_cast<unsigned int>((j + corner[1]) * (cells + 1) + (i + corner[0])));
            }
        }
    }

    // Past the grid of a terrain with water lies open sea: the water plane
    // covers it. A dry terrain continues as a flat skirt at the base height
    // out to the rendered ground extent, in four strips.
    const float outer = m_groundHalfExtent;
    if (terrain->hasWater() || outer <= domainHalf + 1.0f)
    {
        return;
    }
    auto addStrip = [&](float x0, float x1, float z0, float z1)
    {
        const int nx = std::max(1, static_cast<int>(std::ceil((x1 - x0) / 240.0f)));
        const int nz = std::max(1, static_cast<int>(std::ceil((z1 - z0) / 240.0f)));
        const unsigned int first = static_cast<unsigned int>(m_floorVertices.size());
        for (int b = 0; b <= nz; ++b)
        {
            for (int a = 0; a <= nx; ++a)
            {
                const float xPos = x0 + (x1 - x0) * static_cast<float>(a) / static_cast<float>(nx);
                const float zPos = z0 + (z1 - z0) * static_cast<float>(b) / static_cast<float>(nz);
                m_floorVertices.push_back({{xPos, baseHeight, zPos}, {0.0f, 1.0f, 0.0f}, groundColor(xPos, zPos, fadeExtent)});
            }
        }
        for (int b = 0; b < nz; ++b)
        {
            for (int a = 0; a < nx; ++a)
            {
                const unsigned int topLeft = first + static_cast<unsigned int>(b * (nx + 1) + a);
                const unsigned int topRight = topLeft + 1;
                const unsigned int bottomLeft = first + static_cast<unsigned int>((b + 1) * (nx + 1) + a);
                const unsigned int bottomRight = bottomLeft + 1;
                m_floorIndices.push_back(topLeft);
                m_floorIndices.push_back(bottomLeft);
                m_floorIndices.push_back(bottomRight);
                m_floorIndices.push_back(topLeft);
                m_floorIndices.push_back(bottomRight);
                m_floorIndices.push_back(topRight);
            }
        }
    };
    addStrip(-outer, outer, -outer, -domainHalf);
    addStrip(-outer, outer, domainHalf, outer);
    addStrip(-outer, -domainHalf, -domainHalf, domainHalf);
    addStrip(domainHalf, outer, -domainHalf, domainHalf);
}

void Renderer::createWaterMesh()
{
    // A grid rather than one quad keeps the interpolated world position
    // accurate for the wave and fog maths across tens of kilometres.
    const int cells = 64;
    const float half = m_sceneFarPlane;
    const float cell = 2.0f * half / static_cast<float>(cells);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(static_cast<std::size_t>(cells + 1) * (cells + 1));
    for (int j = 0; j <= cells; ++j)
    {
        for (int i = 0; i <= cells; ++i)
        {
            vertices.push_back({{-half + static_cast<float>(i) * cell, 0.0f, -half + static_cast<float>(j) * cell},
                                {0.0f, 1.0f, 0.0f},
                                glm::vec3(1.0f)});
        }
    }
    for (int j = 0; j < cells; ++j)
    {
        for (int i = 0; i < cells; ++i)
        {
            for (const auto &corner : missilesim::sim::Terrain::kCellTriangles)
            {
                indices.push_back(static_cast<unsigned int>((j + corner[1]) * (cells + 1) + (i + corner[0])));
            }
        }
    }

    glGenVertexArrays(1, &m_waterVAO);
    glGenBuffers(1, &m_waterVBO);
    glGenBuffers(1, &m_waterEBO);
    glBindVertexArray(m_waterVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_waterVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_waterEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, metalRoughness));
    glEnableVertexAttribArray(3);
    glBindVertexArray(0);
    m_waterIndexCount = static_cast<GLsizei>(indices.size());
}

void Renderer::uploadTerrainSurface()
{
    if (!isPBRActive())
    {
        return;
    }
    pbr::TerrainSurface surface;
    const missilesim::sim::Terrain *terrain = m_terrain.get();
    if (terrain != nullptr && terrain->gridCells() > 0)
    {
        const int samples = terrain->gridCells() + 1;
        if (m_heightmapTexture == 0)
        {
            glGenTextures(1, &m_heightmapTexture);
        }
        glBindTexture(GL_TEXTURE_2D, m_heightmapTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, samples, samples, 0, GL_RED, GL_FLOAT, terrain->samples().data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        surface.heightmap = m_heightmapTexture;
        surface.origin = glm::vec2(terrain->gridOrigin());
        surface.size = terrain->cellSize() * static_cast<float>(terrain->gridCells());
    }
    if (terrain != nullptr)
    {
        surface.outerBed = terrain->outerBedHeight();
        surface.hasWater = terrain->hasWater();
        surface.waterLevel = terrain->hasWater() ? terrain->waterLevel() : terrain->baseHeight();
    }
    m_pbrPipeline->setTerrainSurface(surface);
}

void Renderer::uploadFloorMesh()
{
    if (m_floorVAO == 0 || m_floorVBO == 0 || m_floorEBO == 0)
    {
        return;
    }

    glBindVertexArray(m_floorVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_floorVBO);
    glBufferData(GL_ARRAY_BUFFER, m_floorVertices.size() * sizeof(Vertex), m_floorVertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_floorEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_floorIndices.size() * sizeof(unsigned int), m_floorIndices.data(), GL_STATIC_DRAW);
    glBindVertexArray(0);
}

void Renderer::createTargetModel()
{
    // Clear previous target vertices and indices
    m_targetVertices.clear();
    m_targetIndices.clear();

    const glm::mat4 jetOrientation = glm::rotate(glm::mat4(1.0f), -glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
    if (loadObjModel("models/jet.obj",
                     m_targetVertices,
                     m_targetIndices,
                     glm::vec3(0.70f, 0.74f, 0.78f),
                     jetOrientation,
                     2.0f, &m_targetExhaustSockets))
    {
        return;
    }

    // Create a simple sphere for the target
    const int stacks = 16;
    const int sectors = 16;
    const float radius = 0.5f;

    // Use a bright red color for the target
    const glm::vec3 targetColor(1.0f, 0.2f, 0.2f);

    // Generate vertices
    for (int stack = 0; stack <= stacks; ++stack)
    {
        float stackAngle = glm::pi<float>() * (float)stack / (float)stacks;
        float stackY = radius * cos(stackAngle);
        float stackRadius = radius * sin(stackAngle);

        for (int sector = 0; sector <= sectors; ++sector)
        {
            float sectorAngle = 2.0f * glm::pi<float>() * (float)sector / (float)sectors;
            float x = stackRadius * cos(sectorAngle);
            float z = stackRadius * sin(sectorAngle);

            // Position
            glm::vec3 position(x, stackY, z);

            // Normal (pointing outward from center)
            glm::vec3 normal = glm::normalize(position);

            // Add vertex
            m_targetVertices.push_back({position, normal, targetColor});
        }
    }

    // Generate indices
    for (int stack = 0; stack < stacks; ++stack)
    {
        int stackStart = stack * (sectors + 1);
        int nextStackStart = (stack + 1) * (sectors + 1);

        for (int sector = 0; sector < sectors; ++sector)
        {
            // First triangle
            m_targetIndices.push_back(stackStart + sector);
            m_targetIndices.push_back(stackStart + sector + 1);
            m_targetIndices.push_back(nextStackStart + sector + 1);

            // Second triangle
            m_targetIndices.push_back(stackStart + sector);
            m_targetIndices.push_back(nextStackStart + sector + 1);
            m_targetIndices.push_back(nextStackStart + sector);
        }
    }
}

void Renderer::createExplosionModel()
{
    // Clear previous vertices and indices
    m_explosionVertices.clear();
    m_explosionIndices.clear();

    // Create a simple sphere for the explosion
    // This is similar to the target model but with different colors
    const int stacks = 16;
    const int sectors = 16;
    const float radius = 1.0f; // Base radius of 1, will be scaled when rendering

    // Use an orange/red gradient for the explosion
    const glm::vec3 innerColor(1.0f, 0.6f, 0.0f); // Orange
    const glm::vec3 outerColor(1.0f, 0.2f, 0.0f); // Red

    // Generate vertices
    for (int stack = 0; stack <= stacks; ++stack)
    {
        float stackAngle = glm::pi<float>() * (float)stack / (float)stacks;
        float stackY = radius * cos(stackAngle);
        float stackRadius = radius * sin(stackAngle);

        // Calculate color based on distance from center
        float normalizedRadius = sin(stackAngle); // 0 at poles, 1 at equator
        glm::vec3 vertexColor = glm::mix(innerColor, outerColor, normalizedRadius);

        for (int sector = 0; sector <= sectors; ++sector)
        {
            float sectorAngle = 2.0f * glm::pi<float>() * (float)sector / (float)sectors;
            float x = stackRadius * cos(sectorAngle);
            float z = stackRadius * sin(sectorAngle);

            // Position
            glm::vec3 position(x, stackY, z);

            // Normal (pointing outward from center)
            glm::vec3 normal = glm::normalize(position);

            // Add vertex
            m_explosionVertices.push_back({position, normal, vertexColor});
        }
    }

    // Generate indices
    for (int stack = 0; stack < stacks; ++stack)
    {
        int stackStart = stack * (sectors + 1);
        int nextStackStart = (stack + 1) * (sectors + 1);

        for (int sector = 0; sector < sectors; ++sector)
        {
            // First triangle
            m_explosionIndices.push_back(stackStart + sector);
            m_explosionIndices.push_back(stackStart + sector + 1);
            m_explosionIndices.push_back(nextStackStart + sector + 1);

            // Second triangle
            m_explosionIndices.push_back(stackStart + sector);
            m_explosionIndices.push_back(nextStackStart + sector + 1);
            m_explosionIndices.push_back(nextStackStart + sector);
        }
    }
}
