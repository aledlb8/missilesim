#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <filesystem>
#include <string>

namespace pbr {

class Shader
{
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader &) = delete;
    Shader &operator=(const Shader &) = delete;
    Shader(Shader &&other) noexcept;
    Shader &operator=(Shader &&other) noexcept;

    bool load(const std::filesystem::path &vertexPath,
              const std::filesystem::path &fragmentPath,
              const std::filesystem::path &geometryPath = "");

    void use() const;

    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec3(const std::string &name, const glm::vec3 &vec) const;
    void setVec4(const std::string &name, const glm::vec4 &vec) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;

    GLuint id() const { return m_id; }
    bool valid() const { return m_id != 0; }

private:
    GLuint m_id = 0;
};

class ComputeShader
{
public:
    ComputeShader() = default;
    ~ComputeShader();

    ComputeShader(const ComputeShader &) = delete;
    ComputeShader &operator=(const ComputeShader &) = delete;
    ComputeShader(ComputeShader &&other) noexcept;
    ComputeShader &operator=(ComputeShader &&other) noexcept;

    bool load(const std::filesystem::path &computePath);

    void use() const;
    void dispatch(unsigned int x, unsigned int y = 1, unsigned int z = 1) const;

    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec3(const std::string &name, const glm::vec3 &vec) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;

    GLuint id() const { return m_id; }
    bool valid() const { return m_id != 0; }

private:
    GLuint m_id = 0;
};

} // namespace pbr
