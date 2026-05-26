#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace pbr {

struct PBRVertex
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 tangent;
    glm::vec3 biTangent;
    glm::vec2 texCoords;
};

class Shader;

class Mesh
{
public:
    Mesh(std::vector<PBRVertex> vertices,
         std::vector<unsigned int> indices,
         std::vector<GLuint> textures);
    ~Mesh();

    Mesh(const Mesh &) = delete;
    Mesh &operator=(const Mesh &) = delete;
    Mesh(Mesh &&other) noexcept;
    Mesh &operator=(Mesh &&other) noexcept;

    void draw(const Shader &shader, bool textured) const;

private:
    void setupMesh();
    void release();

    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    GLuint m_ebo = 0;

    std::vector<PBRVertex> m_vertices;
    std::vector<unsigned int> m_indices;
    std::vector<GLuint> m_textures;
};

} // namespace pbr
