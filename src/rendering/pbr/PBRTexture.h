#pragma once

#include <glad/glad.h>
#include <filesystem>
#include <string>

namespace pbr {

enum class TextureType
{
    MultisampledColor,
    SingleColor,
    MultisampledDepth,
    SingleDepth,
    SingleColorClamp,
    SingleDepthBorder
};

class Texture
{
public:
    Texture() = default;
    ~Texture();

    Texture(const Texture &) = delete;
    Texture &operator=(const Texture &) = delete;
    Texture(Texture &&other) noexcept;
    Texture &operator=(Texture &&other) noexcept;

    bool loadTexture(const std::filesystem::path &filePath, bool sRGB = true);
    bool loadHDRTexture(const std::filesystem::path &filePath);

    static GLuint genTextureDirectlyOnGPU(int width, int height, int attachmentNum, TextureType type);

    GLuint id() const { return m_textureID; }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    int getComponents() const { return m_nComponents; }
    bool valid() const { return m_textureID != 0; }

private:
    GLuint m_textureID = 0;
    int m_width = 0;
    int m_height = 0;
    int m_nComponents = 0;
};

} // namespace pbr
