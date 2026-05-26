#include "PBRShader.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace pbr {

// --- Shader ---

Shader::~Shader()
{
    if (m_id != 0)
    {
        glDeleteProgram(m_id);
    }
}

Shader::Shader(Shader &&other) noexcept : m_id(other.m_id)
{
    other.m_id = 0;
}

Shader &Shader::operator=(Shader &&other) noexcept
{
    if (this != &other)
    {
        if (m_id != 0)
        {
            glDeleteProgram(m_id);
        }
        m_id = other.m_id;
        other.m_id = 0;
    }
    return *this;
}

static bool readFile(const std::filesystem::path &path, std::string &out)
{
    std::ifstream file(path);
    if (!file.good())
    {
        std::cerr << "PBR: Cannot find shader file: " << path << std::endl;
        return false;
    }
    std::stringstream ss;
    ss << file.rdbuf();
    out = ss.str();
    return true;
}

static GLuint compileStage(GLenum type, const char *source, const char *label)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "PBR: " << label << " shader compilation failed:\n"
                  << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool Shader::load(const std::filesystem::path &vertexPath,
                  const std::filesystem::path &fragmentPath,
                  const std::filesystem::path &geometryPath)
{
    std::string vertexCode, fragmentCode, geometryCode;

    if (!readFile(vertexPath, vertexCode))
        return false;
    if (!readFile(fragmentPath, fragmentCode))
        return false;

    bool hasGeometry = !geometryPath.empty() && geometryPath != "";
    if (hasGeometry && !readFile(geometryPath, geometryCode))
        return false;

    GLuint vert = compileStage(GL_VERTEX_SHADER, vertexCode.c_str(), "Vertex");
    if (!vert)
        return false;

    GLuint frag = compileStage(GL_FRAGMENT_SHADER, fragmentCode.c_str(), "Fragment");
    if (!frag)
    {
        glDeleteShader(vert);
        return false;
    }

    GLuint geom = 0;
    if (hasGeometry)
    {
        geom = compileStage(GL_GEOMETRY_SHADER, geometryCode.c_str(), "Geometry");
        if (!geom)
        {
            glDeleteShader(vert);
            glDeleteShader(frag);
            return false;
        }
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vert);
    glAttachShader(program, frag);
    if (hasGeometry)
        glAttachShader(program, geom);
    glLinkProgram(program);

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetProgramInfoLog(program, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "PBR: Shader linking failed:\n"
                  << infoLog << std::endl;
        glDeleteProgram(program);
        glDeleteShader(vert);
        glDeleteShader(frag);
        if (hasGeometry)
            glDeleteShader(geom);
        return false;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);
    if (hasGeometry)
        glDeleteShader(geom);

    if (m_id != 0)
        glDeleteProgram(m_id);
    m_id = program;
    return true;
}

void Shader::use() const { glUseProgram(m_id); }

void Shader::setBool(const std::string &name, bool value) const
{
    glUniform1i(glGetUniformLocation(m_id, name.c_str()), static_cast<int>(value));
}
void Shader::setInt(const std::string &name, int value) const
{
    glUniform1i(glGetUniformLocation(m_id, name.c_str()), value);
}
void Shader::setFloat(const std::string &name, float value) const
{
    glUniform1f(glGetUniformLocation(m_id, name.c_str()), value);
}
void Shader::setVec3(const std::string &name, const glm::vec3 &vec) const
{
    glUniform3fv(glGetUniformLocation(m_id, name.c_str()), 1, &vec[0]);
}
void Shader::setVec4(const std::string &name, const glm::vec4 &vec) const
{
    glUniform4fv(glGetUniformLocation(m_id, name.c_str()), 1, &vec[0]);
}
void Shader::setMat4(const std::string &name, const glm::mat4 &mat) const
{
    glUniformMatrix4fv(glGetUniformLocation(m_id, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

// --- ComputeShader ---

ComputeShader::~ComputeShader()
{
    if (m_id != 0)
    {
        glDeleteProgram(m_id);
    }
}

ComputeShader::ComputeShader(ComputeShader &&other) noexcept : m_id(other.m_id)
{
    other.m_id = 0;
}

ComputeShader &ComputeShader::operator=(ComputeShader &&other) noexcept
{
    if (this != &other)
    {
        if (m_id != 0)
            glDeleteProgram(m_id);
        m_id = other.m_id;
        other.m_id = 0;
    }
    return *this;
}

bool ComputeShader::load(const std::filesystem::path &computePath)
{
    std::string computeCode;
    if (!readFile(computePath, computeCode))
        return false;

    GLuint comp = compileStage(GL_COMPUTE_SHADER, computeCode.c_str(), "Compute");
    if (!comp)
        return false;

    GLuint program = glCreateProgram();
    glAttachShader(program, comp);
    glLinkProgram(program);

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetProgramInfoLog(program, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "PBR: Compute shader linking failed:\n"
                  << infoLog << std::endl;
        glDeleteProgram(program);
        glDeleteShader(comp);
        return false;
    }

    glDeleteShader(comp);

    if (m_id != 0)
        glDeleteProgram(m_id);
    m_id = program;
    return true;
}

void ComputeShader::use() const { glUseProgram(m_id); }

void ComputeShader::dispatch(unsigned int x, unsigned int y, unsigned int z) const
{
    glDispatchCompute(x, y, z);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void ComputeShader::setBool(const std::string &name, bool value) const
{
    glUniform1i(glGetUniformLocation(m_id, name.c_str()), static_cast<int>(value));
}
void ComputeShader::setInt(const std::string &name, int value) const
{
    glUniform1i(glGetUniformLocation(m_id, name.c_str()), value);
}
void ComputeShader::setFloat(const std::string &name, float value) const
{
    glUniform1f(glGetUniformLocation(m_id, name.c_str()), value);
}
void ComputeShader::setVec3(const std::string &name, const glm::vec3 &vec) const
{
    glUniform3fv(glGetUniformLocation(m_id, name.c_str()), 1, &vec[0]);
}
void ComputeShader::setMat4(const std::string &name, const glm::mat4 &mat) const
{
    glUniformMatrix4fv(glGetUniformLocation(m_id, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

} // namespace pbr
