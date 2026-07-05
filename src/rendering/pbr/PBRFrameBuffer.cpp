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
// ResolveBuffer  --  HDR colour + depth target for the MSAA resolve
// ===========================================================================

ResolveBuffer::ResolveBuffer(ResolveBuffer &&other) noexcept
    : FrameBuffer(std::move(other))
{
}

ResolveBuffer &ResolveBuffer::operator=(ResolveBuffer &&other) noexcept
{
    FrameBuffer::operator=(std::move(other));
    return *this;
}

void ResolveBuffer::setupFrameBuffer(int width, int height)
{
    destroy();

    m_width = width;
    m_height = height;

    glGenFramebuffers(1, &m_frameBufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferID);

    m_texColorBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::SingleColor);

    m_depthBuffer = Texture::genTextureDirectlyOnGPU(
        width, height, 0, TextureType::SingleDepth);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "PBR: ResolveBuffer is not complete!" << std::endl;
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
// BloomMipChain  --  descending half-resolution RGBA16F mips for bloom
// ===========================================================================

BloomMipChain::~BloomMipChain()
{
    destroy();
}

void BloomMipChain::destroy()
{
    for (Mip &mip : m_mips)
    {
        if (mip.texture != 0)
            glDeleteTextures(1, &mip.texture);
        if (mip.fbo != 0)
            glDeleteFramebuffers(1, &mip.fbo);
    }
    m_mips.clear();
}

void BloomMipChain::setup(int width, int height)
{
    destroy();

    // Enough mips to reach a very small base level (wide halos) without
    // degenerating below ~10px: 1080p -> 6 mips (960x540 ... 30x17).
    int shortest = width < height ? width : height;
    int count = 3;
    while ((shortest >> (count + 1)) >= 10 && count < 6)
        ++count;

    int mipW = width;
    int mipH = height;
    for (int i = 0; i < count; ++i)
    {
        mipW = mipW > 1 ? mipW / 2 : 1;
        mipH = mipH > 1 ? mipH / 2 : 1;

        Mip mip;
        mip.width = mipW;
        mip.height = mipH;

        glGenTextures(1, &mip.texture);
        glBindTexture(GL_TEXTURE_2D, mip.texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, mipW, mipH, 0,
                     GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glGenFramebuffers(1, &mip.fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, mip.fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, mip.texture, 0);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            std::cerr << "PBR: BloomMipChain mip " << i << " is not complete!" << std::endl;
        }

        m_mips.push_back(mip);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace pbr
