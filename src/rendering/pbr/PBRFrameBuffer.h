#pragma once

#include <glad/glad.h>

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
// ResolveBuffer  --  2 colour attachments (main + bloom high-pass) + depth
// ---------------------------------------------------------------------------

class ResolveBuffer : public FrameBuffer
{
public:
    ResolveBuffer() = default;
    ~ResolveBuffer() override;

    ResolveBuffer(ResolveBuffer &&other) noexcept;
    ResolveBuffer &operator=(ResolveBuffer &&other) noexcept;

    void setupFrameBuffer(int width, int height);

    GLuint bloomTexture() const { return m_texBloomBuffer; }

private:
    GLuint m_texBloomBuffer = 0;
};

// ---------------------------------------------------------------------------
// QuadHDRBuffer  --  single HDR colour for ping-pong blur
// ---------------------------------------------------------------------------

class QuadHDRBuffer : public FrameBuffer
{
public:
    QuadHDRBuffer() = default;
    ~QuadHDRBuffer() override = default;

    QuadHDRBuffer(QuadHDRBuffer &&other) noexcept;
    QuadHDRBuffer &operator=(QuadHDRBuffer &&other) noexcept;

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
// PointShadowBuffer  --  cubemap depth for point light shadows
// ---------------------------------------------------------------------------

class PointShadowBuffer : public FrameBuffer
{
public:
    PointShadowBuffer() = default;
    ~PointShadowBuffer() override;

    PointShadowBuffer(PointShadowBuffer &&other) noexcept;
    PointShadowBuffer &operator=(PointShadowBuffer &&other) noexcept;

    void setupFrameBuffer(int width, int height);

    GLuint depthCubemap() const { return m_depthCubemap; }

private:
    GLuint m_depthCubemap = 0;
};

} // namespace pbr
