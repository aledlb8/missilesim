#include "PBRModel.h"
#include "PBRShader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include "stb/stb_image.h"

#include <filesystem>
#include <iostream>

namespace pbr {

// ---------------------------------------------------------------------------
// Local texture loading helper
// ---------------------------------------------------------------------------
static GLuint loadTextureFromFile(const std::string &filePath)
{
    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(false);
    unsigned char *data = stbi_load(filePath.c_str(), &width, &height, &nrChannels, 0);
    if (!data)
    {
        std::cerr << "PBR: Failed to load texture: " << filePath << std::endl;
        return 0;
    }

    GLenum internalFormat = GL_RGB;
    GLenum dataFormat = GL_RGB;
    if (nrChannels == 1)
    {
        internalFormat = GL_RED;
        dataFormat = GL_RED;
    }
    else if (nrChannels == 3)
    {
        internalFormat = GL_RGB;
        dataFormat = GL_RGB;
    }
    else if (nrChannels == 4)
    {
        internalFormat = GL_RGBA;
        dataFormat = GL_RGBA;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(internalFormat),
                 width, height, 0, dataFormat, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
    return textureID;
}

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------
Model::Model(const std::string &meshPath,
             const TransformParameters &initParameters,
             bool IBL)
    : m_IBL(IBL)
{
    // Build model matrix from transform parameters
    m_modelMatrix = glm::mat4(1.0f);
    m_modelMatrix = glm::translate(m_modelMatrix, initParameters.translation);
    if (initParameters.angle != 0.0f)
    {
        m_modelMatrix = glm::rotate(m_modelMatrix,
                                    glm::radians(initParameters.angle),
                                    initParameters.rotationAxis);
    }
    m_modelMatrix = glm::scale(m_modelMatrix, initParameters.scaling);

    loadModel(meshPath);
}

void Model::draw(const Shader &shader, bool textured) const
{
    for (const auto &mesh : m_meshes)
    {
        mesh.draw(shader, textured);
    }
}

void Model::loadModel(const std::string &path)
{
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(
        path,
        aiProcess_Triangulate |
        aiProcess_OptimizeMeshes |
        aiProcess_CalcTangentSpace |
        aiProcess_FlipUVs);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cerr << "PBR: ASSIMP error: " << importer.GetErrorString() << std::endl;
        return;
    }

    // Extract directory and file extension from the path
    std::filesystem::path fsPath(path);
    m_directory = fsPath.parent_path().string();
    m_fileExtension = fsPath.extension().string();

    processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode *node, const aiScene *scene)
{
    // Process all meshes in the current node
    for (unsigned int i = 0; i < node->mNumMeshes; ++i)
    {
        aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        m_meshes.push_back(processMesh(mesh, scene));
    }

    // Recursively process child nodes
    for (unsigned int i = 0; i < node->mNumChildren; ++i)
    {
        processNode(node->mChildren[i], scene);
    }
}

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene)
{
    std::vector<PBRVertex> vertices;
    std::vector<unsigned int> indices;

    vertices.reserve(mesh->mNumVertices);

    for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
    {
        PBRVertex vertex{};

        // Position
        vertex.position.x = mesh->mVertices[i].x;
        vertex.position.y = mesh->mVertices[i].y;
        vertex.position.z = mesh->mVertices[i].z;

        // Normal
        if (mesh->HasNormals())
        {
            vertex.normal.x = mesh->mNormals[i].x;
            vertex.normal.y = mesh->mNormals[i].y;
            vertex.normal.z = mesh->mNormals[i].z;
        }

        // Tangent and bitangent
        if (mesh->HasTangentsAndBitangents())
        {
            vertex.tangent.x = mesh->mTangents[i].x;
            vertex.tangent.y = mesh->mTangents[i].y;
            vertex.tangent.z = mesh->mTangents[i].z;

            vertex.biTangent.x = mesh->mBitangents[i].x;
            vertex.biTangent.y = mesh->mBitangents[i].y;
            vertex.biTangent.z = mesh->mBitangents[i].z;
        }

        // Texture coordinates (first set only)
        if (mesh->mTextureCoords[0])
        {
            vertex.texCoords.x = mesh->mTextureCoords[0][i].x;
            vertex.texCoords.y = mesh->mTextureCoords[0][i].y;
        }
        else
        {
            vertex.texCoords = glm::vec2(0.0f);
        }

        vertices.push_back(vertex);
    }

    // Process indices
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i)
    {
        const aiFace &face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; ++j)
        {
            indices.push_back(face.mIndices[j]);
        }
    }

    // Process textures from material
    std::vector<GLuint> textures;
    if (mesh->mMaterialIndex >= 0)
    {
        aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
        textures = processTextures(material);
    }

    return Mesh(std::move(vertices), std::move(indices), std::move(textures));
}

std::vector<GLuint> Model::processTextures(aiMaterial *material)
{
    // We collect textures in a specific order matching the PBR pipeline:
    // [0] albedo (diffuse), [1] emissive, [2] normals, [3] AO/lightmap, [4] metalRoughness

    // Texture type mapping for iteration
    static const aiTextureType textureTypes[] = {
        aiTextureType_DIFFUSE,          // -> albedo   [0]
        aiTextureType_EMISSIVE,         // -> emissive [1]
        aiTextureType_NORMALS,          // -> normals  [2]
        aiTextureType_LIGHTMAP,         // -> AO/light [3]
        aiTextureType_UNKNOWN,          // -> metalRoughness [4] (glTF stores here)
    };

    std::vector<GLuint> textures;
    textures.reserve(5);

    for (const auto &type : textureTypes)
    {
        GLuint texID = 0;

        if (material->GetTextureCount(type) > 0)
        {
            aiString texPath;
            material->GetTexture(type, 0, &texPath);
            std::string texName(texPath.C_Str());

            // Check if already loaded
            auto it = m_textureAtlas.find(texName);
            if (it != m_textureAtlas.end())
            {
                texID = it->second;
            }
            else
            {
                std::string fullPath = m_directory + "/" + texName;
                texID = loadTextureFromFile(fullPath);
                if (texID != 0)
                {
                    m_textureAtlas[texName] = texID;
                }
            }
        }

        textures.push_back(texID);
    }

    return textures;
}

} // namespace pbr
