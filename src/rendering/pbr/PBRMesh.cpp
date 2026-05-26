#include "PBRMesh.h"
#include "PBRShader.h"

#include <iostream>

namespace pbr {

Mesh::Mesh(std::vector<PBRVertex> vertices,
           std::vector<unsigned int> indices,
           std::vector<GLuint> textures)
    : m_vertices(std::move(vertices)),
      m_indices(std::move(indices)),
      m_textures(std::move(textures))
{
    setupMesh();
}

Mesh::~Mesh()
{
    release();
}

Mesh::Mesh(Mesh &&other) noexcept
    : m_vao(other.m_vao),
      m_vbo(other.m_vbo),
      m_ebo(other.m_ebo),
      m_vertices(std::move(other.m_vertices)),
      m_indices(std::move(other.m_indices)),
      m_textures(std::move(other.m_textures))
{
    other.m_vao = 0;
    other.m_vbo = 0;
    other.m_ebo = 0;
}

Mesh &Mesh::operator=(Mesh &&other) noexcept
{
    if (this != &other)
    {
        release();

        m_vao = other.m_vao;
        m_vbo = other.m_vbo;
        m_ebo = other.m_ebo;
        m_vertices = std::move(other.m_vertices);
        m_indices = std::move(other.m_indices);
        m_textures = std::move(other.m_textures);

        other.m_vao = 0;
        other.m_vbo = 0;
        other.m_ebo = 0;
    }
    return *this;
}

void Mesh::release()
{
    if (m_vao != 0)
    {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }
    if (m_vbo != 0)
    {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }
    if (m_ebo != 0)
    {
        glDeleteBuffers(1, &m_ebo);
        m_ebo = 0;
    }
}

void Mesh::setupMesh()
{
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(m_vertices.size() * sizeof(PBRVertex)),
                 m_vertices.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(m_indices.size() * sizeof(unsigned int)),
                 m_indices.data(),
                 GL_STATIC_DRAW);

    // Location 0: position (vec3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PBRVertex),
                          reinterpret_cast<void *>(offsetof(PBRVertex, position)));

    // Location 1: normal (vec3)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(PBRVertex),
                          reinterpret_cast<void *>(offsetof(PBRVertex, normal)));

    // Location 2: texCoords (vec2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(PBRVertex),
                          reinterpret_cast<void *>(offsetof(PBRVertex, texCoords)));

    // Location 3: tangent (vec3)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(PBRVertex),
                          reinterpret_cast<void *>(offsetof(PBRVertex, tangent)));

    // Location 4: biTangent (vec3)
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(PBRVertex),
                          reinterpret_cast<void *>(offsetof(PBRVertex, biTangent)));

    glBindVertexArray(0);
}

void Mesh::draw(const Shader &shader, bool textured) const
{
    // Always bind texture[0] as albedoMap at texture unit 0
    if (!m_textures.empty())
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_textures[0]);
        shader.setInt("albedoMap", 0);
    }

    if (textured)
    {
        // Emissive map at unit 1
        if (m_textures.size() > 1)
        {
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, m_textures[1]);
            shader.setInt("emissiveMap", 1);
        }

        // Normal map at unit 2
        if (m_textures.size() > 2)
        {
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, m_textures[2]);
            shader.setInt("normalsMap", 2);

            bool normalMapped = (m_textures[2] != 0);
            shader.setBool("normalMapped", normalMapped);
        }
        else
        {
            shader.setBool("normalMapped", false);
        }

        // AO / lightmap at unit 3
        if (m_textures.size() > 3)
        {
            glActiveTexture(GL_TEXTURE3);
            glBindTexture(GL_TEXTURE_2D, m_textures[3]);
            shader.setInt("lightMap", 3);

            bool aoMapped = (m_textures[3] != 0);
            shader.setBool("aoMapped", aoMapped);
        }
        else
        {
            shader.setBool("aoMapped", false);
        }

        // Metal-roughness map at unit 4
        if (m_textures.size() > 4)
        {
            glActiveTexture(GL_TEXTURE4);
            glBindTexture(GL_TEXTURE_2D, m_textures[4]);
            shader.setInt("metalRoughMap", 4);
        }
    }

    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(m_indices.size()),
                   GL_UNSIGNED_INT,
                   nullptr);
    glBindVertexArray(0);

    glActiveTexture(GL_TEXTURE0);
}

} // namespace pbr
