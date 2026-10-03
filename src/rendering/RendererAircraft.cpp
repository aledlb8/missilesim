#include "Renderer.h"
#include "../flight/AircraftCatalog.h"
#include "../objects/Fighter.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

bool Renderer::hasAircraftModel(const char *aircraftId) const
{
    return aircraftId && m_aircraftMeshes.find(aircraftId) != m_aircraftMeshes.end();
}

const Renderer::AircraftMesh *Renderer::aircraftMesh(const PhysicsObject &object) const
{
    if (object.getType() != "Fighter") return nullptr;
    const auto &fighter = static_cast<const Fighter &>(object);
    const auto found = m_aircraftMeshes.find(fighter.jet().aircraftId());
    return found == m_aircraftMeshes.end() ? nullptr : &found->second;
}

void Renderer::createAircraftModels()
{
    // Load once with a live GL context, never while switching planes or while
    // collecting exhaust sockets. CPU staging arrays are released after upload.
    const auto *catalog = missilesim::flight::aircraftCatalog();
    const glm::mat4 orientation = glm::rotate(glm::mat4(1.0f), -glm::half_pi<float>(), glm::vec3(1, 0, 0));
    for (int i = 0; i < missilesim::flight::aircraftCatalogCount(); ++i)
    {
        const std::string id = catalog[i].id;
        if (hasAircraftModel(id.c_str())) continue;
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        AircraftMesh mesh;
        if (!loadObjModel("models/fighters/" + id + ".obj", vertices, indices,
                          glm::vec3(0.35f), orientation, 0.0f, &mesh.sockets))
        {
            std::cerr << "Aircraft asset missing: " << id << "; using legacy fallback\n";
            continue;
        }

        // Author origin is the nose. Center only along the longitudinal axis;
        // the fuselage datum remains z=0 instead of shifting with fin height.
        float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::lowest();
        for (const auto &v : vertices)
        {
            minY = std::min(minY, v.position.y);
            maxY = std::max(maxY, v.position.y);
        }
        const float centerY = (minY + maxY) * 0.5f;
        for (auto &v : vertices) v.position.y -= centerY;
        for (auto &socket : mesh.sockets) socket.position.y -= centerY;

        glGenVertexArrays(1, &mesh.vao);
        glGenBuffers(1, &mesh.vbo);
        glGenBuffers(1, &mesh.ebo);
        glBindVertexArray(mesh.vao);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, position)));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, normal)));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, color)));
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, metalRoughness)));
        for (GLuint location = 0; location < 4; ++location) glEnableVertexAttribArray(location);
        glBindVertexArray(0);
        mesh.indexCount = static_cast<GLsizei>(indices.size());
        std::cout << "Aircraft mesh " << id << ": " << indices.size()/3 << " triangles, "
                  << mesh.sockets.size() << " exhaust sockets\n";
        m_aircraftMeshes.emplace(id, std::move(mesh));
    }
}
