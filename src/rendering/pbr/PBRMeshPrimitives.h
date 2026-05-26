#pragma once

#include <glad/glad.h>

namespace pbr {

/// Fullscreen quad (2 triangles, 6 vertices) for screen-space passes.
struct Quad
{
    Quad() = default;
    ~Quad();

    Quad(const Quad &) = delete;
    Quad &operator=(const Quad &) = delete;
    Quad(Quad &&other) noexcept;
    Quad &operator=(Quad &&other) noexcept;

    void setup();

    /// Draw the quad, optionally binding up to 3 textures to units 0, 1, 2.
    void draw(GLuint readTex1 = 0, GLuint readTex2 = 0, GLuint readTex3 = 0) const;

private:
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
};

/// Unit cube (36 vertices) for cubemap rendering.
struct Cube
{
    Cube() = default;
    ~Cube();

    Cube(const Cube &) = delete;
    Cube &operator=(const Cube &) = delete;
    Cube(Cube &&other) noexcept;
    Cube &operator=(Cube &&other) noexcept;

    void setup();
    void draw() const;

private:
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
};

} // namespace pbr
