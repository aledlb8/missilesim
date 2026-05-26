#include "PBRFrameBuffer.h"
#include "PBRTexture.h"

#include <iostream>

namespace pbr {

// ===========================================================================
// FrameBuffer  (base)
// ===========================================================================

FrameBuffer::~FrameBuffer()
{
    destroy();
}

void FrameBuffer::destroy()
{
    if (m_texColorBuffer != 0)
    {
        glDeleteTextures(1, &m_texColorBuffer);
        m_texColorBuffer = 0;
    }
    if (m_depthBuffer != 0)
    {
        glDeleteTextures(1, &m_depthBuffer);
        m_depthBuffer = 0;
    }
    if (m_frameBufferID != 0)
    {
        glDeleteFramebuffers(1, &m_frameBufferID);
        m_frameBufferID = 0;
    }
    m_width = 0;
    m_height = 0;
}

void FrameBuffer::moveFrom(FrameBuffer &other) noexcept
{
    m_frameBufferID = other.m_frameBufferID;
    m_texColorBuffer = other.m_texColorBuffer;
    m_depthBuffer = other.m_depthBuffer;
    m_width = other.m_width;
    m_height = other.m_height;

    other.m_frameBufferID = 0;
    other.m_texColorBuffer = 0;
    other.m_depthBuffer = 0;
    other.m_width = 0;
    other.m_height = 0;
}

FrameBuffer::FrameBuffer(FrameBuffer &&other) noexcept
{
    moveFrom(other);
}

FrameBuffer &FrameBuffer::operator=(FrameBuffer &&other) noexcept
{
    if (this != &other)
    {
        destroy();
        moveFrom(other);
    }
    return *this;
}

void FrameBuffer::bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);
}

void FrameBuffer::clear() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void FrameBuffer::blitTo(const FrameBuffer &target,
                          int srcWidth, int srcHeight,
                          int dstWidth, int dstHeight,
                          GLbitfield mask, GLenum filter) const
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_frameBufferID);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target.m_frameBufferID);
    glBlitFramebuffer(0, 0, srcWidth, srcHeight,
                      0, 0, dstWidth, dstHeight,
                      mask, filter);
}

// ===========================================================================
// FrameBufferMultiSampled  --  4x MSAA colour + depth
// ===========================================================================

FrameBufferMultiSampled::FrameBufferMultiSampled(FrameBufferMultiSampled &&other) noexcept
    : FrameBuffer(std::move(other))
{
}

FrameBufferMultiSampled &FrameBufferMultiSampled::operator=(FrameBufferMultiSampled &&other) noexcept
{
    FrameBuffer::operator=(std::move(other));
    return *this;
}

void FrameBufferMultiSampled::setupFrameBuffer(int width, int height)
{
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    m_texColorBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::MultisampledColor);

    m_depthBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::MultisampledDepth);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: FrameBufferMultiSampled is not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===========================================================================
// ResolveBuffer  --  2 colour attachments (main + bloom high-pass) + depth
// ===========================================================================

ResolveBuffer::~ResolveBuffer()
{
    if (m_texBloomBuffer != 0)
    {
        glDeleteTextures(1, &m_texBloomBuffer);
    }
}

ResolveBuffer::ResolveBuffer(ResolveBuffer &&other) noexcept
    : FrameBuffer(std::move(other)),
      m_texBloomBuffer(other.m_texBloomBuffer)
{
    other.m_texBloomBuffer = 0;
}

ResolveBuffer &ResolveBuffer::operator=(ResolveBuffer &&other) noexcept
{
    if (this != &other)
    {
        if (m_texBloomBuffer != 0)
        {
            glDeleteTextures(1, &m_texBloomBuffer);
        }
        FrameBuffer::operator=(std::move(other));
        m_texBloomBuffer = other.m_texBloomBuffer;
        other.m_texBloomBuffer = 0;
    }
    return *this;
}

void ResolveBuffer::setupFrameBuffer(int width, int height)
{
    if (m_texBloomBuffer != 0)
    {
        glDeleteTextures(1, &m_texBloomBuffer);
        m_texBloomBuffer = 0;
    }
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    // Attachment 0: main resolved colour
    m_texColorBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::SingleColor);

    // Attachment 1: bloom / bright-pass
    m_texBloomBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 1, TextureType::SingleColor);

    // Depth
    m_depthBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::SingleDepth);

    // Tell OpenGL which colour attachments we are drawing into
    GLuint attachments[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    glDrawBuffers(2, attachments);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: ResolveBuffer is not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===========================================================================
// QuadHDRBuffer  --  single HDR colour for ping-pong Gaussian blur
// ===========================================================================

QuadHDRBuffer::QuadHDRBuffer(QuadHDRBuffer &&other) noexcept
    : FrameBuffer(std::move(other))
{
}

QuadHDRBuffer &QuadHDRBuffer::operator=(QuadHDRBuffer &&other) noexcept
{
    FrameBuffer::operator=(std::move(other));
    return *this;
}

void QuadHDRBuffer::setupFrameBuffer(int width, int height)
{
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    m_texColorBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::SingleColorClamp);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: QuadHDRBuffer is not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===========================================================================
// CaptureBuffer  --  renderbuffer depth for IBL environment map capture
// ===========================================================================

CaptureBuffer::~CaptureBuffer()
{
    if (m_rbo != 0)
    {
        glDeleteRenderbuffers(1, &m_rbo);
    }
}

CaptureBuffer::CaptureBuffer(CaptureBuffer &&other) noexcept
    : FrameBuffer(std::move(other)),
      m_rbo(other.m_rbo)
{
    other.m_rbo = 0;
}

CaptureBuffer &CaptureBuffer::operator=(CaptureBuffer &&other) noexcept
{
    if (this != &other)
    {
        if (m_rbo != 0)
        {
            glDeleteRenderbuffers(1, &m_rbo);
        }
        FrameBuffer::operator=(std::move(other));
        m_rbo = other.m_rbo;
        other.m_rbo = 0;
    }
    return *this;
}

void CaptureBuffer::setupFrameBuffer(int width, int height)
{
    if (m_rbo != 0)
    {
        glDeleteRenderbuffers(1, &m_rbo);
        m_rbo = 0;
    }
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    glGenRenderbuffers(1, &m_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, m_rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: CaptureBuffer is not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void CaptureBuffer::resizeFrameBuffer(int width, int height)
{
    m_width = width;
    m_height = height;

    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);
    glBindRenderbuffer(GL_RENDERBUFFER, m_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===========================================================================
// DirShadowBuffer  --  depth-only for directional light shadows
// ===========================================================================

DirShadowBuffer::DirShadowBuffer(DirShadowBuffer &&other) noexcept
    : FrameBuffer(std::move(other))
{
}

DirShadowBuffer &DirShadowBuffer::operator=(DirShadowBuffer &&other) noexcept
{
    FrameBuffer::operator=(std::move(other));
    return *this;
}

void DirShadowBuffer::setupFrameBuffer(int width, int height)
{
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    m_depthBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::SingleDepthBorder);

    // No colour buffer -- depth only
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: DirShadowBuffer is not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===========================================================================
// PointShadowBuffer  --  cubemap depth for omnidirectional point light shadows
// ===========================================================================

PointShadowBuffer::~PointShadowBuffer()
{
    if (m_depthCubemap != 0)
    {
        glDeleteTextures(1, &m_depthCubemap);
    }
}

PointShadowBuffer::PointShadowBuffer(PointShadowBuffer &&other) noexcept
    : FrameBuffer(std::move(other)),
      m_depthCubemap(other.m_depthCubemap)
{
    other.m_depthCubemap = 0;
}

PointShadowBuffer &PointShadowBuffer::operator=(PointShadowBuffer &&other) noexcept
{
    if (this != &other)
    {
        if (m_depthCubemap != 0)
        {
            glDeleteTextures(1, &m_depthCubemap);
        }
        FrameBuffer::operator=(std::move(other));
        m_depthCubemap = other.m_depthCubemap;
        other.m_depthCubemap = 0;
    }
    return *this;
}

void PointShadowBuffer::setupFrameBuffer(int width, int height)
{
    if (m_depthCubemap != 0)
    {
        glDeleteTextures(1, &m_depthCubemap);
        m_depthCubemap = 0;
    }
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    // Create a cubemap depth texture
    glGenTextures(1, &m_depthCubemap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_depthCubemap);
    for (unsigned int i = 0; i < 6; ++i)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,
                     GL_DEPTH_COMPONENT32F,
                     width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_depthCubemap, 0);

    // No colour buffer -- depth only
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: PointShadowBuffer is not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace pbr
