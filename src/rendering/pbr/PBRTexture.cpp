#include "PBRTexture.h"

#include "stb/stb_image.h"

#include <iostream>

namespace pbr {

// ---------------------------------------------------------------------------
// RAII / move semantics
// ---------------------------------------------------------------------------

Texture::~Texture()
{
    if (m_textureID != 0)
    {
        glDeleteTextures(1, &m_textureID);
    }
}

Texture::Texture(Texture &&other) noexcept
    : m_textureID(other.m_textureID),
      m_width(other.m_width),
      m_height(other.m_height),
      m_nComponents(other.m_nComponents)
{
    other.m_textureID = 0;
    other.m_width = 0;
    other.m_height = 0;
    other.m_nComponents = 0;
}

Texture &Texture::operator=(Texture &&other) noexcept
{
    if (this != &other)
    {
        if (m_textureID != 0)
        {
            glDeleteTextures(1, &m_textureID);
        }
        m_textureID = other.m_textureID;
        m_width = other.m_width;
        m_height = other.m_height;
        m_nComponents = other.m_nComponents;

        other.m_textureID = 0;
        other.m_width = 0;
        other.m_height = 0;
        other.m_nComponents = 0;
    }
    return *this;
}

// ---------------------------------------------------------------------------
// loadTexture  --  LDR images (PNG, JPG, BMP, TGA, etc.) via stb_image
// ---------------------------------------------------------------------------

bool Texture::loadTexture(const std::filesystem::path &filePath, bool sRGB)
{
    stbi_set_flip_vertically_on_load(true);

    unsigned char *data = stbi_load(filePath.string().c_str(),
                                    &m_width, &m_height, &m_nComponents, 0);
    if (!data)
    {
        std::cerr << "PBR: Failed to load texture: " << filePath << std::endl;
        return false;
    }

    GLenum internalFormat = GL_RGBA8;
    GLenum dataFormat = GL_RGBA;

    if (m_nComponents == 1)
    {
        internalFormat = GL_R8;
        dataFormat = GL_RED;
    }
    else if (m_nComponents == 3)
    {
        internalFormat = sRGB ? GL_SRGB8 : GL_RGB8;
        dataFormat = GL_RGB;
    }
    else if (m_nComponents == 4)
    {
        internalFormat = sRGB ? GL_SRGB8_ALPHA8 : GL_RGBA8;
        dataFormat = GL_RGBA;
    }

    if (m_textureID != 0)
    {
        glDeleteTextures(1, &m_textureID);
    }

    glGenTextures(1, &m_textureID);
    glBindTexture(GL_TEXTURE_2D, m_textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(internalFormat),
                 m_width, m_height, 0, dataFormat, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

// ---------------------------------------------------------------------------
// loadHDRTexture  --  HDR radiance images (.hdr) via stb_image
// ---------------------------------------------------------------------------

bool Texture::loadHDRTexture(const std::filesystem::path &filePath)
{
    stbi_set_flip_vertically_on_load(true);

    float *data = stbi_loadf(filePath.string().c_str(),
                             &m_width, &m_height, &m_nComponents, 0);
    if (!data)
    {
        std::cerr << "PBR: Failed to load HDR texture: " << filePath << std::endl;
        return false;
    }

    if (m_textureID != 0)
    {
        glDeleteTextures(1, &m_textureID);
    }

    glGenTextures(1, &m_textureID);
    glBindTexture(GL_TEXTURE_2D, m_textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F,
                 m_width, m_height, 0, GL_RGB, GL_FLOAT, data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

// ---------------------------------------------------------------------------
// genTextureDirectlyOnGPU  --  create FBO attachment textures on the GPU
// ---------------------------------------------------------------------------

GLuint Texture::genTextureDirectlyOnGPU(int width, int height,
                                        int attachmentNum, TextureType type)
{
    GLuint textureID = 0;
    glGenTextures(1, &textureID);

    switch (type)
    {
    case TextureType::MultisampledColor:
    {
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureID);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA16F,
                                width, height, GL_TRUE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + attachmentNum,
                               GL_TEXTURE_2D_MULTISAMPLE, textureID, 0);
        break;
    }

    case TextureType::SingleColor:
    {
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F,
                     width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + attachmentNum,
                               GL_TEXTURE_2D, textureID, 0);
        break;
    }

    case TextureType::MultisampledDepth:
    {
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureID);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_DEPTH_COMPONENT32F,
                                width, height, GL_TRUE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D_MULTISAMPLE, textureID, 0);
        break;
    }

    case TextureType::SingleDepth:
    {
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F,
                     width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D, textureID, 0);
        break;
    }

    case TextureType::SingleColorClamp:
    {
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F,
                     width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + attachmentNum,
                               GL_TEXTURE_2D, textureID, 0);
        break;
    }

    case TextureType::SingleDepthBorder:
    {
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F,
                     width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D, textureID, 0);
        break;
    }
    }

    return textureID;
}

} // namespace pbr
