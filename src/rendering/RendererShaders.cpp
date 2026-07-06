#include "Renderer.h"
#include "SceneEffects.h"

#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

// Vertex shader source
const char *vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aNormal;
    layout (location = 2) in vec3 aColor;
    
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    
    out vec3 FragPos;
    out vec3 Normal;
    out vec3 Color;
    
    void main() {
        FragPos = vec3(model * vec4(aPos, 1.0));
        Normal = mat3(transpose(inverse(model))) * aNormal;
        Color = aColor;
        gl_Position = projection * view * model * vec4(aPos, 1.0);
    }
)";

// Fragment shader source
const char *fragmentShaderSource = R"(
    #version 330 core
    in vec3 FragPos;
    in vec3 Normal;
    in vec3 Color;
    
    uniform vec3 cameraPos;
    uniform float fogDensity;
    
    out vec4 FragColor;
    
    void main() {
        vec3 lightDir = normalize(vec3(-0.35, 0.9, -0.2));
        vec3 norm = normalize(Normal);
        float diffuse = max(dot(norm, lightDir), 0.0);
        float skyMix = clamp(norm.y * 0.5 + 0.5, 0.0, 1.0);
        vec3 ambient = mix(vec3(0.12, 0.13, 0.16), vec3(0.36, 0.42, 0.48), skyMix);
        vec3 litColor = Color * (ambient + diffuse * 0.85);

        float viewDistance = length(FragPos - cameraPos);
        float fogFactor = clamp(exp(-viewDistance * fogDensity), 0.0, 1.0);
        vec3 fogColor = vec3(0.58, 0.69, 0.82);
        
        FragColor = vec4(mix(fogColor, litColor, fogFactor), 1.0);
    }
)";

// Add a line shader source at the top of the file after the fragment shader source
const char *lineVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    layout (location = 2) in float aSize;
    
    uniform mat4 view;
    uniform mat4 projection;
    
    out vec3 Color;
    
    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
        gl_PointSize = aSize;
        Color = aColor;
    }
)";

const char *lineFragmentShaderSource = R"(
    #version 330 core
    in vec3 Color;
    out vec4 FragColor;
    
    void main() {
        FragColor = vec4(Color, 1.0);
    }
)";

// Anti-aliased instanced line shader: each segment is a screen-space
// expanded quad (4-vertex strip, one instance per segment). Rendered into
// the HDR resolve target pre-tonemap with depth testing.
const char *aaLineVertexShaderSource = R"(
    #version 430 core
    layout (location = 0) in vec4 iStart;  // xyz world, w = width in pixels
    layout (location = 1) in vec4 iEnd;    // xyz world
    layout (location = 2) in vec4 iColor;

    uniform mat4 viewProj;
    uniform vec2 viewportSize;

    out vec4 vColor;
    out float vEdge;        // -1..1 across the ribbon
    flat out float vWidthPx;

    void main() {
        vec4 clip0 = viewProj * vec4(iStart.xyz, 1.0);
        vec4 clip1 = viewProj * vec4(iEnd.xyz, 1.0);

        // Clip against the near plane in homogeneous space; a fully
        // behind-camera segment is discarded off-screen.
        const float nearEps = 1e-4;
        if (clip0.w < nearEps && clip1.w < nearEps) {
            gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
            vColor = vec4(0.0);
            vEdge = 0.0;
            vWidthPx = 1.0;
            return;
        }
        if (clip0.w < nearEps) {
            float t = (nearEps - clip0.w) / (clip1.w - clip0.w);
            clip0 = mix(clip0, clip1, t);
        } else if (clip1.w < nearEps) {
            float t = (nearEps - clip1.w) / (clip0.w - clip1.w);
            clip1 = mix(clip1, clip0, t);
        }

        // gl_VertexID 0..3 -> (start,-1) (start,+1) (end,-1) (end,+1)
        float endSel = float(gl_VertexID >> 1);
        float side = float((gl_VertexID & 1) * 2 - 1);
        vec4 clip = mix(clip0, clip1, endSel);

        // Screen-space perpendicular of the segment.
        vec2 screen0 = (clip0.xy / clip0.w) * viewportSize;
        vec2 screen1 = (clip1.xy / clip1.w) * viewportSize;
        vec2 dir = screen1 - screen0;
        if (dot(dir, dir) < 1e-8) {
            dir = vec2(1.0, 0.0);
        }
        dir = normalize(dir);
        vec2 perp = vec2(-dir.y, dir.x);

        float widthPx = max(iStart.w, 1.0);
        // Half width plus a 1px feather margin, converted back to NDC.
        vec2 offsetNdc = perp * (0.5 * widthPx + 1.0) * (2.0 / viewportSize);
        clip.xy += offsetNdc * side * clip.w;

        gl_Position = clip;
        vColor = iColor;
        vEdge = side;
        vWidthPx = widthPx;
    }
)";

const char *aaLineFragmentShaderSource = R"(
    #version 430 core
    in vec4 vColor;
    in float vEdge;
    flat in float vWidthPx;

    out vec4 FragColor;

    void main() {
        float distPx = abs(vEdge) * (0.5 * vWidthPx + 1.0);
        float alpha = clamp(0.5 * vWidthPx + 0.5 - distPx, 0.0, 1.0);
        alpha *= vColor.a;
        // Mild HDR lift so overlay lines pick up a whisper of bloom.
        vec3 color = vColor.rgb * 1.5;
        FragColor = vec4(color * alpha, alpha);  // premultiplied
    }
)";

void Renderer::createShaders()
{
    // Vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // Check vertex shader compilation
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "ERROR: Vertex shader compilation failed\n"
                  << infoLog << std::endl;
    }

    // Fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // Check fragment shader compilation
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "ERROR: Fragment shader compilation failed\n"
                  << infoLog << std::endl;
    }

    // Link shaders
    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);

    // Check for linking errors
    glGetProgramiv(m_shaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(m_shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR: Shader program linking failed\n"
                  << infoLog << std::endl;
    }

    // Delete the shaders as they're linked into our program now and no longer needed
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Renderer::createLineRendering()
{
    // Create and compile line vertex shader
    GLuint lineVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(lineVertexShader, 1, &lineVertexShaderSource, NULL);
    glCompileShader(lineVertexShader);

    // Check line vertex shader compile errors
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(lineVertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(lineVertexShader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
                  << infoLog << std::endl;
    }

    // Create and compile line fragment shader
    GLuint lineFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(lineFragmentShader, 1, &lineFragmentShaderSource, NULL);
    glCompileShader(lineFragmentShader);

    // Check line fragment shader compile errors
    glGetShaderiv(lineFragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(lineFragmentShader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
                  << infoLog << std::endl;
    }

    // Create line shader program
    m_lineShaderProgram = glCreateProgram();
    glAttachShader(m_lineShaderProgram, lineVertexShader);
    glAttachShader(m_lineShaderProgram, lineFragmentShader);
    glLinkProgram(m_lineShaderProgram);

    // Check line shader program link errors
    glGetProgramiv(m_lineShaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(m_lineShaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
                  << infoLog << std::endl;
    }

    // Delete line shaders after linking
    glDeleteShader(lineVertexShader);
    glDeleteShader(lineFragmentShader);

    m_lineViewLoc = glGetUniformLocation(m_lineShaderProgram, "view");
    m_lineProjLoc = glGetUniformLocation(m_lineShaderProgram, "projection");

    // Create VAO and VBO for lines
    glGenVertexArrays(1, &m_lineVAO);
    glGenBuffers(1, &m_lineVBO);

    // Setup line VAO and VBO
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);

    m_lineBufferCapacity = 256;
    glBufferData(GL_ARRAY_BUFFER, m_lineBufferCapacity * sizeof(DebugVertex), nullptr, GL_DYNAMIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)offsetof(DebugVertex, position));
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)offsetof(DebugVertex, color));
    glEnableVertexAttribArray(1);

    // Point size attribute
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)offsetof(DebugVertex, size));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
    m_debugLineVertices.reserve(512);
    m_debugPointVertices.reserve(128);

    createAALineRendering();
}

void Renderer::createAALineRendering()
{
    GLint success;
    GLchar infoLog[512];

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &aaLineVertexShaderSource, NULL);
    glCompileShader(vertexShader);
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "ERROR: AA line vertex shader compilation failed\n"
                  << infoLog << std::endl;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &aaLineFragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "ERROR: AA line fragment shader compilation failed\n"
                  << infoLog << std::endl;
    }

    m_aaLineProgram = glCreateProgram();
    glAttachShader(m_aaLineProgram, vertexShader);
    glAttachShader(m_aaLineProgram, fragmentShader);
    glLinkProgram(m_aaLineProgram);
    glGetProgramiv(m_aaLineProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(m_aaLineProgram, 512, NULL, infoLog);
        std::cerr << "ERROR: AA line program linking failed\n"
                  << infoLog << std::endl;
        glDeleteProgram(m_aaLineProgram);
        m_aaLineProgram = 0;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    if (m_aaLineProgram == 0)
    {
        return;
    }

    m_aaLineViewProjLoc = glGetUniformLocation(m_aaLineProgram, "viewProj");
    m_aaLineViewportLoc = glGetUniformLocation(m_aaLineProgram, "viewportSize");

    glGenVertexArrays(1, &m_aaLineVAO);
    glGenBuffers(1, &m_aaLineInstanceVBO);

    glBindVertexArray(m_aaLineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_aaLineInstanceVBO);

    m_aaLineInstanceCapacity = 256;
    glBufferData(GL_ARRAY_BUFFER, m_aaLineInstanceCapacity * sizeof(LineInstance),
                 nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(LineInstance),
                          (void *)offsetof(LineInstance, start));
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0, 1);

    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(LineInstance),
                          (void *)offsetof(LineInstance, end));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(LineInstance),
                          (void *)offsetof(LineInstance, color));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    glBindVertexArray(0);
}