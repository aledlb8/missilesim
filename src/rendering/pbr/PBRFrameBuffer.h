#pragma once

#include <glad/glad.h>

#include <vector>

namespace pbr {

// ---------------------------------------------------------------------------
// FrameBuffer  --  base class owning an FBO, colour attachment and depth
// ---------------------------------------------------------------------------

class FrameBuffer
{
public:
    FrameBuffer() = default;
    virtual ~FrameBuffer();

    FrameBuffer(const FrameBuffer &) = delete;
    FrameBuffer &operator=(const FrameBuffer &) = delete;
    FrameBuffer(FrameBuffer &&other) noexcept;
    FrameBuffer &operator=(FrameBuffer &&other) noexcept;

    void bind() const;
    void clear() const;
    void blitTo(const FrameBuffer &target, int srcWidth, int srcHeight,
                int dstWidth, int dstHeight, GLbitfield mask = GL_COLOR_BUFFER_BIT,
                GLenum filter = GL_NEAREST) const;

    GLuint fbo() const { return m_frameBufferID; }
    GLuint colorTexture() const { return m_texColorBuffer; }
    GLuint depthTexture() const { return m_depthBuffer; }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }

protected:
    GLuint m_frameBufferID = 0;
    GLuint m_texColorBuffer = 0;
    GLuint m_depthBuffer = 0;
    int m_width = 0;
    int m_height = 0;

    void moveFrom(FrameBuffer &other) noexcept;
    void destroy();
};

// ---------------------------------------------------------------------------
// FrameBufferMultiSampled  --  4x MSAA colour + depth
// ---------------------------------------------------------------------------

class FrameBufferMultiSampled : public FrameBuffer
{
public:
    FrameBufferMultiSampled() = default;
    ~FrameBufferMultiSampled() override = default;

    FrameBufferMultiSampled(FrameBufferMultiSampled &&other) noexcept;
    FrameBufferMultiSampled &operator=(FrameBufferMultiSampled &&other) noexcept;

    void setupFrameBuffer(int width, int height);
};

// ---------------------------------------------------------------------------
// ResolveBuffer  --  HDR colour + depth target for the MSAA resolve
// ---------------------------------------------------------------------------

class ResolveBuffer : public FrameBuffer
{
public:
    ResolveBuffer() = default;
    ~ResolveBuffer() override = default;

    ResolveBuffer(ResolveBuffer &&other) noexcept;
    ResolveBuffer &operator=(ResolveBuffer &&other) noexcept;

    void setupFrameBuffer(int width, int height);
};

// ---------------------------------------------------------------------------
// CaptureBuffer  --  renderbuffer depth for IBL environment capture
// ---------------------------------------------------------------------------

class CaptureBuffer : public FrameBuffer
{
public:
    CaptureBuffer() = default;
    ~CaptureBuffer() override;

    CaptureBuffer(CaptureBuffer &&other) noexcept;
    CaptureBuffer &operator=(CaptureBuffer &&other) noexcept;

    void setupFrameBuffer(int width, int height);
    void resizeFrameBuffer(int width, int height);

    GLuint rbo() const { return m_rbo; }

private:
    GLuint m_rbo = 0;
};

// ---------------------------------------------------------------------------
// DirShadowBuffer  --  depth-only for directional light shadows
// ---------------------------------------------------------------------------

class DirShadowBuffer : public FrameBuffer
{
public:
    DirShadowBuffer() = default;
    ~DirShadowBuffer() override = default;

    DirShadowBuffer(DirShadowBuffer &&other) noexcept;
    DirShadowBuffer &operator=(DirShadowBuffer &&other) noexcept;

    void setupFrameBuffer(int width, int height);
};

// ---------------------------------------------------------------------------
// BloomMipChain  --  descending half-resolution RGBA16F mips, each with its
// own FBO, for progressive downsample/upsample bloom
// ---------------------------------------------------------------------------

class BloomMipChain
{
public:
    BloomMipChain() = default;
    ~BloomMipChain();

    BloomMipChain(const BloomMipChain &) = delete;
    BloomMipChain &operator=(const BloomMipChain &) = delete;

    /// (Re)build the chain for the given full-resolution size.
    void setup(int width, int height);
    void destroy();

    int mipCount() const { return static_cast<int>(m_mips.size()); }
    GLuint fbo(int i) const { return m_mips[i].fbo; }
    GLuint texture(int i) const { return m_mips[i].texture; }
    int mipWidth(int i) const { return m_mips[i].width; }
    int mipHeight(int i) const { return m_mips[i].height; }

private:
    struct Mip
    {
        GLuint fbo = 0;
        GLuint texture = 0;
        int width = 0;
        int height = 0;
    };

    std::vector<Mip> m_mips;
};

} // namespace pbr
