#pragma once

#include "PBRMesh.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <assimp/scene.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace pbr {

struct TransformParameters
{
    glm::vec3 translation{0.0f};
    float angle = 0.0f;
    glm::vec3 rotationAxis{0.0f, 1.0f, 0.0f};
    glm::vec3 scaling{1.0f};
};

class Shader;

class Model
{
public:
    Model(const std::string &meshPath,
          const TransformParameters &initParameters,
          bool IBL);
    ~Model() = default;

    Model(const Model &) = delete;
    Model &operator=(const Model &) = delete;
    Model(Model &&other) noexcept = default;
    Model &operator=(Model &&other) noexcept = default;

    void draw(const Shader &shader, bool textured) const;

    const glm::mat4 &getModelMatrix() const { return m_modelMatrix; }
    void setModelMatrix(const glm::mat4 &mat) { m_modelMatrix = mat; }
    bool isIBL() const { return m_IBL; }

private:
    void loadModel(const std::string &path);
    void processNode(aiNode *node, const aiScene *scene);
    Mesh processMesh(aiMesh *mesh, const aiScene *scene);
    std::vector<GLuint> processTextures(aiMaterial *material);

    bool m_IBL = false;
    glm::mat4 m_modelMatrix{1.0f};
    std::vector<Mesh> m_meshes;
    std::unordered_map<std::string, GLuint> m_textureAtlas;
    std::string m_directory;
    std::string m_fileExtension;
};

} // namespace pbr
