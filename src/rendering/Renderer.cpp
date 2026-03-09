#include "Renderer.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Missile.h"
#include "../objects/Target.h"
#include <algorithm>
#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

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
    
    out vec4 FragColor;
    
    void main() {
        vec3 lightDir = normalize(vec3(-0.35, 0.9, -0.2));
        vec3 norm = normalize(Normal);
        float diffuse = max(dot(norm, lightDir), 0.0);
        float skyMix = clamp(norm.y * 0.5 + 0.5, 0.0, 1.0);
        vec3 ambient = mix(vec3(0.12, 0.13, 0.16), vec3(0.36, 0.42, 0.48), skyMix);
        vec3 litColor = Color * (ambient + diffuse * 0.85);

        float viewDistance = length(FragPos - cameraPos);
        float fogFactor = clamp(exp(-viewDistance * 0.0013), 0.0, 1.0);
        vec3 fogColor = vec3(0.58, 0.69, 0.82);
        
        FragColor = vec4(mix(fogColor, litColor, fogFactor), 1.0);
    }
)";

// Add a line shader source at the top of the file after the fragment shader source
const char *lineVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    
    uniform mat4 view;
    uniform mat4 projection;
    
    out vec3 Color;
    
    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
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

Renderer::Renderer()
    : m_vao(0), m_vbo(0), m_ebo(0), m_shaderProgram(0),
      m_floorVAO(0), m_floorVBO(0), m_floorEBO(0),
      m_targetVAO(0), m_targetVBO(0), m_targetEBO(0),
      m_explosionVAO(0), m_explosionVBO(0), m_explosionEBO(0),
      m_cameraPosition(-120.0f, 78.0f, 180.0f),
      m_cameraTarget(0.0f, 0.0f, 0.0f),
      m_cameraUp(0.0f, 1.0f, 0.0f),
      m_cameraFront(0.0f, 0.0f, -1.0f),
      m_cameraRight(1.0f, 0.0f, 0.0f),
      m_cameraYaw(-126.0f),
      m_cameraPitch(-18.0f),
      m_cameraSpeed(35.0f),
      m_cameraFOV(50.0f)
{
    initialize();
}

Renderer::~Renderer()
{
    // Cleanup OpenGL resources
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
    glDeleteBuffers(1, &m_ebo);

    // Cleanup floor resources
    glDeleteVertexArrays(1, &m_floorVAO);
    glDeleteBuffers(1, &m_floorVBO);
    glDeleteBuffers(1, &m_floorEBO);

    // Cleanup target resources
    glDeleteVertexArrays(1, &m_targetVAO);
    glDeleteBuffers(1, &m_targetVBO);
    glDeleteBuffers(1, &m_targetEBO);

    // Cleanup explosion resources
    glDeleteVertexArrays(1, &m_explosionVAO);
    glDeleteBuffers(1, &m_explosionVBO);
    glDeleteBuffers(1, &m_explosionEBO);

    // Cleanup line resources
    glDeleteVertexArrays(1, &m_lineVAO);
    glDeleteBuffers(1, &m_lineVBO);
    glDeleteProgram(m_lineShaderProgram);

    glDeleteProgram(m_shaderProgram);
}

void Renderer::initialize()
{
    // Create shaders
    createShaders();

    // Create model data
    createMissileModel();
    createFloor();
    createTargetModel();
    createExplosionModel();
    createLineRendering();

    // Store models in the map
    m_modelMeshes["missile"] = std::make_pair(m_vertices, m_indices);
    m_modelMeshes["target"] = std::make_pair(m_targetVertices, m_targetIndices);
    m_modelMeshes["explosion"] = std::make_pair(m_explosionVertices, m_explosionIndices);

    // Set up vertex buffers for missile
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    // Bind vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(Vertex), m_vertices.data(), GL_STATIC_DRAW);

    // Bind element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW);

    // Set vertex attribute pointers
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    // Color attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    // Set up vertex buffers for floor
    glGenVertexArrays(1, &m_floorVAO);
    glGenBuffers(1, &m_floorVBO);
    glGenBuffers(1, &m_floorEBO);

    glBindVertexArray(m_floorVAO);

    // Bind vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_floorVBO);
    glBufferData(GL_ARRAY_BUFFER, m_floorVertices.size() * sizeof(Vertex), m_floorVertices.data(), GL_STATIC_DRAW);

    // Bind element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_floorEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_floorIndices.size() * sizeof(unsigned int), m_floorIndices.data(), GL_STATIC_DRAW);

    // Set vertex attribute pointers
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    // Color attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    // Set up vertex buffers for targets
    glGenVertexArrays(1, &m_targetVAO);
    glGenBuffers(1, &m_targetVBO);
    glGenBuffers(1, &m_targetEBO);

    glBindVertexArray(m_targetVAO);

    // Bind vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_targetVBO);
    glBufferData(GL_ARRAY_BUFFER, m_targetVertices.size() * sizeof(Vertex), m_targetVertices.data(), GL_STATIC_DRAW);

    // Bind element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_targetEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_targetIndices.size() * sizeof(unsigned int), m_targetIndices.data(), GL_STATIC_DRAW);

    // Set vertex attribute pointers
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    // Color attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    // Set up vertex buffers for explosion
    glGenVertexArrays(1, &m_explosionVAO);
    glGenBuffers(1, &m_explosionVBO);
    glGenBuffers(1, &m_explosionEBO);

    glBindVertexArray(m_explosionVAO);

    // Bind vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_explosionVBO);
    glBufferData(GL_ARRAY_BUFFER, m_explosionVertices.size() * sizeof(Vertex), m_explosionVertices.data(), GL_STATIC_DRAW);

    // Bind element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_explosionEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_explosionIndices.size() * sizeof(unsigned int), m_explosionIndices.data(), GL_STATIC_DRAW);

    // Set vertex attribute pointers
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    // Color attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    // Get shader uniform locations
    m_modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    m_viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    m_projLoc = glGetUniformLocation(m_shaderProgram, "projection");
    m_cameraPosLoc = glGetUniformLocation(m_shaderProgram, "cameraPos");

    // Initialize camera vectors
    updateCameraVectors();

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
}

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

void Renderer::createSimpleCube()
{
    // Clear previous vertices and indices
    m_vertices.clear();
    m_indices.clear();

    // Generate a simple cube
    m_vertices = {
        // Front face
        {{-0.5f, -0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}}, // 0
        {{0.5f, -0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},  // 1
        {{0.5f, 0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},   // 2
        {{-0.5f, 0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},  // 3

        // Back face
        {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}}, // 4
        {{0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},  // 5
        {{0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},   // 6
        {{-0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},  // 7

        // Top face
        {{-0.5f, 0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, // 8
        {{0.5f, 0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},  // 9
        {{0.5f, 0.5f, 0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},   // 10
        {{-0.5f, 0.5f, 0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},  // 11

        // Bottom face
        {{-0.5f, -0.5f, -0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}}, // 12
        {{0.5f, -0.5f, -0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},  // 13
        {{0.5f, -0.5f, 0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},   // 14
        {{-0.5f, -0.5f, 0.5f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},  // 15

        // Right face
        {{0.5f, -0.5f, 0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}},  // 16
        {{0.5f, -0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}}, // 17
        {{0.5f, 0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}},  // 18
        {{0.5f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}},   // 19

        // Left face
        {{-0.5f, -0.5f, 0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}},  // 20
        {{-0.5f, -0.5f, -0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}}, // 21
        {{-0.5f, 0.5f, -0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}},  // 22
        {{-0.5f, 0.5f, 0.5f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 1.0f}}    // 23
    };

    // Indices for drawing triangles
    m_indices = {
        // Front face
        0, 1, 2, 2, 3, 0,

        // Back face
        4, 5, 6, 6, 7, 4,

        // Top face
        8, 9, 10, 10, 11, 8,

        // Bottom face
        12, 13, 14, 14, 15, 12,

        // Right face
        16, 17, 18, 18, 19, 16,

        // Left face
        20, 21, 22, 22, 23, 20};
}

void Renderer::createMissileModel()
{
    // Clear previous vertices and indices
    m_vertices.clear();
    m_indices.clear();

    // Simplified missile model (elongated cylinder with nose cone)
    const int segments = 12;
    const float radius = 0.2f;
    const float height = 1.0f;

    // Colors
    glm::vec3 bodyColor(0.7f, 0.7f, 0.7f);
    glm::vec3 noseColor(0.9f, 0.1f, 0.1f);
    glm::vec3 finColor(0.3f, 0.3f, 0.3f);

    // Generate missile body (cylinder)
    for (int i = 0; i <= segments; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        // Calculate normal
        glm::vec3 normal = glm::normalize(glm::vec3(x, 0.0f, z));

        // Bottom vertex
        m_vertices.push_back({{x, -height, z}, normal, bodyColor});

        // Top vertex
        m_vertices.push_back({{x, height, z}, normal, bodyColor});

        // Create indices for cylinder sides
        if (i < segments)
        {
            unsigned int bottomLeft = i * 2;
            unsigned int bottomRight = (i * 2 + 2) % (segments * 2 + 2);
            unsigned int topLeft = i * 2 + 1;
            unsigned int topRight = (i * 2 + 3) % (segments * 2 + 2);

            // First triangle
            m_indices.push_back(bottomLeft);
            m_indices.push_back(bottomRight);
            m_indices.push_back(topRight);

            // Second triangle
            m_indices.push_back(bottomLeft);
            m_indices.push_back(topRight);
            m_indices.push_back(topLeft);
        }
    }

    // Create top cap (nose cone)
    unsigned int centerIndex = m_vertices.size();
    m_vertices.push_back({{0.0f, height + radius, 0.0f}, {0.0f, 1.0f, 0.0f}, noseColor});

    for (int i = 0; i < segments; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        unsigned int current = i * 2 + 1;
        unsigned int next = ((i + 1) % segments) * 2 + 1;

        m_indices.push_back(centerIndex);
        m_indices.push_back(current);
        m_indices.push_back(next);
    }

    // Create bottom cap
    centerIndex = m_vertices.size();
    m_vertices.push_back({{0.0f, -height, 0.0f}, {0.0f, -1.0f, 0.0f}, bodyColor});

    for (int i = 0; i < segments; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        unsigned int current = i * 2;
        unsigned int next = ((i + 1) % segments) * 2;

        m_indices.push_back(centerIndex);
        m_indices.push_back(next);
        m_indices.push_back(current);
    }

    // Create fins
    // For simplicity, we'll add 4 triangular fins at the bottom
    const int numFins = 4;
    const float finHeight = 0.4f;
    const float finLength = 0.8f;

    for (int i = 0; i < numFins; ++i)
    {
        float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(numFins);

        glm::vec3 finNormal = glm::normalize(glm::vec3(cos(angle + glm::pi<float>() / 4.0f), 0.0f, sin(angle + glm::pi<float>() / 4.0f)));

        // Base of the fin at the body
        glm::vec3 base1 = glm::vec3(radius * cos(angle), -height + 0.1f, radius * sin(angle));
        glm::vec3 base2 = glm::vec3(radius * cos(angle), -height + 0.1f + finLength, radius * sin(angle));

        // Tip of the fin
        glm::vec3 tip = glm::vec3(radius * cos(angle) + finHeight * finNormal.x,
                                  -height + 0.1f + finLength / 2.0f,
                                  radius * sin(angle) + finHeight * finNormal.z);

        // Add vertices
        unsigned int baseIndex1 = m_vertices.size();
        m_vertices.push_back({base1, finNormal, finColor});

        unsigned int baseIndex2 = m_vertices.size();
        m_vertices.push_back({base2, finNormal, finColor});

        unsigned int tipIndex = m_vertices.size();
        m_vertices.push_back({tip, finNormal, finColor});

        // Add indices for the triangular fin
        m_indices.push_back(baseIndex1);
        m_indices.push_back(baseIndex2);
        m_indices.push_back(tipIndex);
    }
}

void Renderer::createFloor()
{
    // Clear previous floor vertices and indices
    m_floorVertices.clear();
    m_floorIndices.clear();

    const float size = 2400.0f;
    const float y = 0.0f;
    const int gridSize = 120;
    const float cellSize = size / gridSize;
    const glm::vec3 tarmacColor(0.16f, 0.18f, 0.22f);
    const glm::vec3 terrainColor(0.29f, 0.32f, 0.27f);
    const glm::vec3 accentColor(0.22f, 0.25f, 0.29f);

    for (int z = 0; z <= gridSize; z++)
    {
        for (int x = 0; x <= gridSize; x++)
        {
            float xPos = -size / 2.0f + x * cellSize;
            float zPos = -size / 2.0f + z * cellSize;
            float radialT = glm::clamp(glm::length(glm::vec2(xPos, zPos)) / (size * 0.5f), 0.0f, 1.0f);
            float macroNoise = 0.5f + 0.5f * sin(xPos * 0.008f) * cos(zPos * 0.010f);
            glm::vec3 color = glm::mix(tarmacColor, terrainColor, radialT);
            color = glm::mix(color, accentColor, 0.18f + macroNoise * 0.12f);

            if (glm::abs(xPos) < 18.0f && glm::abs(zPos) < 260.0f)
            {
                color = glm::mix(color, glm::vec3(0.30f, 0.32f, 0.36f), 0.7f);
            }

            if (glm::abs(zPos) < 24.0f && glm::abs(xPos) < 160.0f)
            {
                color = glm::mix(color, glm::vec3(0.24f, 0.27f, 0.32f), 0.35f);
            }

            m_floorVertices.push_back({{xPos, y, zPos}, {0.0f, 1.0f, 0.0f}, color});
        }
    }

    for (int z = 0; z < gridSize; z++)
    {
        for (int x = 0; x < gridSize; x++)
        {
            unsigned int topLeft = z * (gridSize + 1) + x;
            unsigned int topRight = topLeft + 1;
            unsigned int bottomLeft = (z + 1) * (gridSize + 1) + x;
            unsigned int bottomRight = bottomLeft + 1;

            m_floorIndices.push_back(topLeft);
            m_floorIndices.push_back(bottomLeft);
            m_floorIndices.push_back(bottomRight);

            m_floorIndices.push_back(topLeft);
            m_floorIndices.push_back(bottomRight);
            m_floorIndices.push_back(topRight);
        }
    }
}

void Renderer::createTargetModel()
{
    // Clear previous target vertices and indices
    m_targetVertices.clear();
    m_targetIndices.clear();

    // Create a simple sphere for the target
    const int stacks = 16;
    const int sectors = 16;
    const float radius = 0.5f;

    // Use a bright red color for the target
    const glm::vec3 targetColor(1.0f, 0.2f, 0.2f);

    // Generate vertices
    for (int stack = 0; stack <= stacks; ++stack)
    {
        float stackAngle = glm::pi<float>() * (float)stack / (float)stacks;
        float stackY = radius * cos(stackAngle);
        float stackRadius = radius * sin(stackAngle);

        for (int sector = 0; sector <= sectors; ++sector)
        {
            float sectorAngle = 2.0f * glm::pi<float>() * (float)sector / (float)sectors;
            float x = stackRadius * cos(sectorAngle);
            float z = stackRadius * sin(sectorAngle);

            // Position
            glm::vec3 position(x, stackY, z);

            // Normal (pointing outward from center)
            glm::vec3 normal = glm::normalize(position);

            // Add vertex
            m_targetVertices.push_back({position, normal, targetColor});
        }
    }

    // Generate indices
    for (int stack = 0; stack < stacks; ++stack)
    {
        int stackStart = stack * (sectors + 1);
        int nextStackStart = (stack + 1) * (sectors + 1);

        for (int sector = 0; sector < sectors; ++sector)
        {
            // First triangle
            m_targetIndices.push_back(stackStart + sector);
            m_targetIndices.push_back(stackStart + sector + 1);
            m_targetIndices.push_back(nextStackStart + sector + 1);

            // Second triangle
            m_targetIndices.push_back(stackStart + sector);
            m_targetIndices.push_back(nextStackStart + sector + 1);
            m_targetIndices.push_back(nextStackStart + sector);
        }
    }
}

void Renderer::createExplosionModel()
{
    // Clear previous vertices and indices
    m_explosionVertices.clear();
    m_explosionIndices.clear();

    // Create a simple sphere for the explosion
    // This is similar to the target model but with different colors
    const int stacks = 16;
    const int sectors = 16;
    const float radius = 1.0f; // Base radius of 1, will be scaled when rendering

    // Use an orange/red gradient for the explosion
    const glm::vec3 innerColor(1.0f, 0.6f, 0.0f); // Orange
    const glm::vec3 outerColor(1.0f, 0.2f, 0.0f); // Red

    // Generate vertices
    for (int stack = 0; stack <= stacks; ++stack)
    {
        float stackAngle = glm::pi<float>() * (float)stack / (float)stacks;
        float stackY = radius * cos(stackAngle);
        float stackRadius = radius * sin(stackAngle);

        // Calculate color based on distance from center
        float normalizedRadius = sin(stackAngle); // 0 at poles, 1 at equator
        glm::vec3 vertexColor = glm::mix(innerColor, outerColor, normalizedRadius);

        for (int sector = 0; sector <= sectors; ++sector)
        {
            float sectorAngle = 2.0f * glm::pi<float>() * (float)sector / (float)sectors;
            float x = stackRadius * cos(sectorAngle);
            float z = stackRadius * sin(sectorAngle);

            // Position
            glm::vec3 position(x, stackY, z);

            // Normal (pointing outward from center)
            glm::vec3 normal = glm::normalize(position);

            // Add vertex
            m_explosionVertices.push_back({position, normal, vertexColor});
        }
    }

    // Generate indices
    for (int stack = 0; stack < stacks; ++stack)
    {
        int stackStart = stack * (sectors + 1);
        int nextStackStart = (stack + 1) * (sectors + 1);

        for (int sector = 0; sector < sectors; ++sector)
        {
            // First triangle
            m_explosionIndices.push_back(stackStart + sector);
            m_explosionIndices.push_back(stackStart + sector + 1);
            m_explosionIndices.push_back(nextStackStart + sector + 1);

            // Second triangle
            m_explosionIndices.push_back(stackStart + sector);
            m_explosionIndices.push_back(nextStackStart + sector + 1);
            m_explosionIndices.push_back(nextStackStart + sector);
        }
    }
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

    // Create VAO and VBO for lines
    glGenVertexArrays(1, &m_lineVAO);
    glGenBuffers(1, &m_lineVBO);

    // Setup line VAO and VBO
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);

    // Allocate buffer space for lines (we'll update it later)
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 2, nullptr, GL_DYNAMIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void Renderer::renderLine(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color)
{
    try
    {
        // Line vertex data: position (xyz) and color (rgb)
        float lineVertices[12] = {
            start.x, start.y, start.z, color.r, color.g, color.b,
            end.x, end.y, end.z, color.r, color.g, color.b};

        // Use line shader program
        glUseProgram(m_lineShaderProgram);

        glm::mat4 view = buildViewMatrix();
        glm::mat4 projection = buildProjectionMatrix();

        // Set uniform values
        GLuint viewLoc = glGetUniformLocation(m_lineShaderProgram, "view");
        GLuint projLoc = glGetUniformLocation(m_lineShaderProgram, "projection");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

        // Bind VAO and update VBO with line vertices
        glBindVertexArray(m_lineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(lineVertices), lineVertices);

        // Set line width (may be limited by hardware)
        glLineWidth(2.0f);

        // Draw the line
        glDrawArrays(GL_LINES, 0, 2);

        // Reset state
        glLineWidth(1.0f);
        glBindVertexArray(0);
        glUseProgram(0);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error rendering line: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown error rendering line" << std::endl;
    }
}

void Renderer::renderText(const glm::vec3 &position, const std::string &text, const glm::vec3 &color)
{
    // Since we don't have a full text rendering system,
    // we'll use ImGui for this purpose in the UI layer.
    // For now, this is a placeholder that doesn't render anything.

    // In a full implementation, you would:
    // 1. Create a text rendering system with FreeType or similar
    // 2. Generate textures for each glyph
    // 3. Render textured quads for each character

    // For simplicity, we're not implementing full text rendering in 3D space.
    // This function could be extended in the future.
}

// Camera control methods
void Renderer::setCameraPosition(const glm::vec3 &position)
{
    m_cameraPosition = position;
    updateCameraVectors();
}

void Renderer::setCameraTarget(const glm::vec3 &target)
{
    m_cameraTarget = target;

    // Calculate direction vector
    m_cameraFront = glm::normalize(target - m_cameraPosition);

    // Update camera angles based on front vector
    m_cameraPitch = glm::degrees(asin(m_cameraFront.y));
    m_cameraYaw = glm::degrees(atan2(m_cameraFront.z, m_cameraFront.x));

    updateCameraVectors();
}

void Renderer::rotateCameraYaw(float deltaDegrees)
{
    m_cameraYaw += deltaDegrees;
    updateCameraVectors();
}

void Renderer::rotateCameraPitch(float deltaDegrees)
{
    m_cameraPitch += deltaDegrees;

    // Constrain pitch to avoid gimbal lock
    if (m_cameraPitch > 89.0f)
        m_cameraPitch = 89.0f;
    if (m_cameraPitch < -89.0f)
        m_cameraPitch = -89.0f;

    updateCameraVectors();
}

void Renderer::moveCameraForward(float distance)
{
    float scaledDistance = distance * m_cameraSpeed;
    glm::vec3 forward = glm::vec3(m_cameraFront.x, 0.0f, m_cameraFront.z);
    if (glm::length2(forward) < 0.0001f)
    {
        forward = glm::vec3(0.0f, 0.0f, -1.0f);
    }
    forward = glm::normalize(forward);
    m_cameraPosition += forward * scaledDistance;
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::moveCameraRight(float distance)
{
    float scaledDistance = distance * m_cameraSpeed;
    glm::vec3 right = glm::vec3(m_cameraRight.x, 0.0f, m_cameraRight.z);
    if (glm::length2(right) < 0.0001f)
    {
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    right = glm::normalize(right);
    m_cameraPosition += right * scaledDistance;
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::moveCameraUp(float distance)
{
    float scaledDistance = distance * m_cameraSpeed;
    m_cameraPosition += glm::vec3(0.0f, 1.0f, 0.0f) * scaledDistance;
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::updateCameraVectors()
{
    // Calculate front vector from yaw and pitch
    glm::vec3 front;
    front.x = cos(glm::radians(m_cameraYaw)) * cos(glm::radians(m_cameraPitch));
    front.y = sin(glm::radians(m_cameraPitch));
    front.z = sin(glm::radians(m_cameraYaw)) * cos(glm::radians(m_cameraPitch));
    m_cameraFront = glm::normalize(front);

    // Recalculate the right and up vectors
    m_cameraRight = glm::normalize(glm::cross(m_cameraFront, glm::vec3(0.0f, 1.0f, 0.0f)));
    m_cameraUp = glm::normalize(glm::cross(m_cameraRight, m_cameraFront));

    // Update target position based on front vector
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::renderAll(const std::vector<PhysicsObject *> &objects)
{
    renderEnvironment();
    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    // Render all objects
    for (auto *object : objects)
    {
        if (!object)
            continue;

        // Create model matrix for the object
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, object->getPosition());

        // If the object is a missile, orient it along its velocity vector
        if (object->getType() == "Missile")
        {
            // If the object is moving, orient it along the velocity vector
            glm::vec3 velocity = object->getVelocity();
            if (glm::length(velocity) > 0.001f)
            {
                // Rotate code remains the same
                // Use velocity direction for facing the object
                glm::vec3 direction = glm::normalize(velocity);

                // Calculate the rotation axis and angle from default orientation (along y-axis) to velocity direction
                glm::vec3 defaultDir = glm::vec3(0.0f, 1.0f, 0.0f);

                // Calculate the angle between default direction and velocity
                float angle = acos(glm::dot(defaultDir, direction));

                // If angle is not 0 or 180 degrees, we need to rotate
                if (abs(angle) > 0.001f && abs(angle - glm::pi<float>()) > 0.001f)
                {
                    // Calculate rotation axis (perpendicular to both vectors)
                    glm::vec3 rotationAxis = glm::normalize(glm::cross(defaultDir, direction));

                    // Apply rotation
                    model = glm::rotate(model, angle, rotationAxis);
                }
                // Handle special case: if velocity is in opposite direction of default
                else if (abs(angle - glm::pi<float>()) <= 0.001f)
                {
                    // Just rotate 180 degrees around X axis
                    model = glm::rotate(model, glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
                }
            }
        }

        // Scale based on the type of object
        if (object->getType() == "Missile")
        {
            // Scale to make the missile shape visible
            model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));
            glBindVertexArray(m_vao);

            glUseProgram(m_shaderProgram);
            glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
            if (m_cameraPosLoc != -1)
            {
                glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
            }

            // Draw object
            glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
        else if (object->getType() == "Target")
        {
            // Scale for target
            model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));
            glBindVertexArray(m_targetVAO);

            glUseProgram(m_shaderProgram);
            glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
            if (m_cameraPosLoc != -1)
            {
                glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
            }

            // Draw object
            glDrawElements(GL_TRIANGLES, m_targetIndices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }
}

void Renderer::render(PhysicsObject *object)
{
    if (!object)
        return;

    // Create model matrix for the object
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, object->getPosition());

    // If the object is moving, orient it along the velocity vector
    glm::vec3 velocity = object->getVelocity();
    if (glm::length(velocity) > 0.001f)
    {
        // Use velocity direction for facing the object
        glm::vec3 direction = glm::normalize(velocity);

        // Calculate the rotation axis and angle from default orientation (along y-axis) to velocity direction
        glm::vec3 defaultDir = glm::vec3(0.0f, 1.0f, 0.0f);

        // Calculate the angle between default direction and velocity
        float angle = acos(glm::dot(defaultDir, direction));

        // If angle is not 0 or 180 degrees, we need to rotate
        if (abs(angle) > 0.001f && abs(angle - glm::pi<float>()) > 0.001f)
        {
            // Calculate rotation axis (perpendicular to both vectors)
            glm::vec3 rotationAxis = glm::normalize(glm::cross(defaultDir, direction));

            // Apply rotation
            model = glm::rotate(model, angle, rotationAxis);
        }
        // Handle special case: if velocity is in opposite direction of default
        else if (abs(angle - glm::pi<float>()) <= 0.001f)
        {
            // Just rotate 180 degrees around X axis
            model = glm::rotate(model, glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
        }
    }

    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    // Use shader program
    glUseProgram(m_shaderProgram);

    // Pass transformation matrices to the shader
    glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    if (m_cameraPosLoc != -1)
    {
        glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }

    // Draw object based on its type
    if (object->getType() == "Missile")
    {
        // Scale to make the missile shape visible
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));

        // Pass updated model matrix with scale
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        // Draw missile
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    else if (object->getType() == "Target")
    {
        // Scale for target
        model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));

        // Pass updated model matrix with scale
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        // Draw target
        glBindVertexArray(m_targetVAO);
        glDrawElements(GL_TRIANGLES, m_targetIndices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    else
    {
        // Default rendering for other objects
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Renderer::renderFloor()
{
    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();
    glm::mat4 model = glm::mat4(1.0f);

    glUseProgram(m_shaderProgram);

    glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    if (m_cameraPosLoc != -1)
    {
        glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }

    glBindVertexArray(m_floorVAO);
    glDrawElements(GL_TRIANGLES, m_floorIndices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Renderer::renderEnvironment()
{
    renderFloor();
    renderWorldGuides();
}

void Renderer::renderWorldGuides()
{
    const float guideY = 0.08f;
    const float runwayHalfWidth = 14.0f;
    const float runwayHalfLength = 230.0f;
    const float airspaceHalfExtent = 420.0f;
    const float beaconHeight = 220.0f;

    const glm::vec3 runwayColor(0.92f, 0.94f, 0.96f);
    const glm::vec3 runwayAccent(0.72f, 0.82f, 0.90f);
    const glm::vec3 axisXColor(0.82f, 0.40f, 0.38f);
    const glm::vec3 axisZColor(0.36f, 0.70f, 0.82f);
    const glm::vec3 guideColor(0.50f, 0.60f, 0.68f);
    const glm::vec3 beaconColor(0.72f, 0.78f, 0.86f);

    renderLine(glm::vec3(-airspaceHalfExtent, guideY, 0.0f), glm::vec3(airspaceHalfExtent, guideY, 0.0f), axisXColor);
    renderLine(glm::vec3(0.0f, guideY, -airspaceHalfExtent), glm::vec3(0.0f, guideY, airspaceHalfExtent), axisZColor);

    renderGroundCircle(120.0f, glm::vec3(0.40f, 0.48f, 0.54f), 32);
    renderGroundCircle(240.0f, glm::vec3(0.46f, 0.54f, 0.62f), 40);
    renderGroundCircle(360.0f, glm::vec3(0.52f, 0.60f, 0.68f), 48);

    renderLine(glm::vec3(-runwayHalfWidth, guideY, -runwayHalfLength), glm::vec3(runwayHalfWidth, guideY, -runwayHalfLength), runwayColor);
    renderLine(glm::vec3(runwayHalfWidth, guideY, -runwayHalfLength), glm::vec3(runwayHalfWidth, guideY, runwayHalfLength), runwayColor);
    renderLine(glm::vec3(runwayHalfWidth, guideY, runwayHalfLength), glm::vec3(-runwayHalfWidth, guideY, runwayHalfLength), runwayColor);
    renderLine(glm::vec3(-runwayHalfWidth, guideY, runwayHalfLength), glm::vec3(-runwayHalfWidth, guideY, -runwayHalfLength), runwayColor);

    for (float z = -runwayHalfLength + 24.0f; z < runwayHalfLength - 12.0f; z += 34.0f)
    {
        renderLine(glm::vec3(0.0f, guideY, z), glm::vec3(0.0f, guideY, z + 18.0f), runwayAccent);
    }

    const float padHalf = 18.0f;
    renderLine(glm::vec3(-padHalf, guideY, -padHalf), glm::vec3(padHalf, guideY, -padHalf), guideColor);
    renderLine(glm::vec3(padHalf, guideY, -padHalf), glm::vec3(padHalf, guideY, padHalf), guideColor);
    renderLine(glm::vec3(padHalf, guideY, padHalf), glm::vec3(-padHalf, guideY, padHalf), guideColor);
    renderLine(glm::vec3(-padHalf, guideY, padHalf), glm::vec3(-padHalf, guideY, -padHalf), guideColor);
    renderPoint(glm::vec3(0.0f, 0.18f, 0.0f), glm::vec3(0.98f, 0.82f, 0.30f), 6.0f);

    const glm::vec3 corners[4] = {
        glm::vec3(-airspaceHalfExtent, 0.0f, -airspaceHalfExtent),
        glm::vec3(airspaceHalfExtent, 0.0f, -airspaceHalfExtent),
        glm::vec3(airspaceHalfExtent, 0.0f, airspaceHalfExtent),
        glm::vec3(-airspaceHalfExtent, 0.0f, airspaceHalfExtent)};

    for (int i = 0; i < 4; ++i)
    {
        const glm::vec3 currentGround = corners[i] + glm::vec3(0.0f, guideY, 0.0f);
        const glm::vec3 nextGround = corners[(i + 1) % 4] + glm::vec3(0.0f, guideY, 0.0f);
        const glm::vec3 currentTop = corners[i] + glm::vec3(0.0f, beaconHeight, 0.0f);
        const glm::vec3 nextTop = corners[(i + 1) % 4] + glm::vec3(0.0f, beaconHeight, 0.0f);

        renderLine(currentGround, nextGround, guideColor);
        renderLine(currentTop, nextTop, glm::vec3(0.56f, 0.64f, 0.72f));
        renderAirspaceBeacon(corners[i], beaconHeight, beaconColor);
    }

    renderLine(glm::vec3(-airspaceHalfExtent, beaconHeight, -airspaceHalfExtent),
               glm::vec3(airspaceHalfExtent, beaconHeight, airspaceHalfExtent),
               glm::vec3(0.42f, 0.50f, 0.58f));
    renderLine(glm::vec3(airspaceHalfExtent, beaconHeight, -airspaceHalfExtent),
               glm::vec3(-airspaceHalfExtent, beaconHeight, airspaceHalfExtent),
               glm::vec3(0.42f, 0.50f, 0.58f));
}

void Renderer::renderGroundCircle(float radius, const glm::vec3 &color, int segments)
{
    const float guideY = 0.08f;
    const float angleStep = glm::two_pi<float>() / static_cast<float>(segments);

    for (int i = 0; i < segments; ++i)
    {
        float startAngle = angleStep * static_cast<float>(i);
        float endAngle = angleStep * static_cast<float>(i + 1);
        glm::vec3 start(radius * cos(startAngle), guideY, radius * sin(startAngle));
        glm::vec3 end(radius * cos(endAngle), guideY, radius * sin(endAngle));
        renderLine(start, end, color);
    }
}

void Renderer::renderAirspaceBeacon(const glm::vec3 &basePosition, float height, const glm::vec3 &color)
{
    const glm::vec3 base = basePosition + glm::vec3(0.0f, 0.08f, 0.0f);
    const glm::vec3 top = basePosition + glm::vec3(0.0f, height, 0.0f);
    renderLine(base, top, color);

    const glm::vec3 tickColor = glm::mix(color, glm::vec3(1.0f, 1.0f, 1.0f), 0.15f);
    for (float altitude = 55.0f; altitude < height; altitude += 55.0f)
    {
        const glm::vec3 tickCenter = basePosition + glm::vec3(0.0f, altitude, 0.0f);
        renderLine(tickCenter - glm::vec3(8.0f, 0.0f, 0.0f), tickCenter + glm::vec3(8.0f, 0.0f, 0.0f), tickColor);
        renderLine(tickCenter - glm::vec3(0.0f, 0.0f, 8.0f), tickCenter + glm::vec3(0.0f, 0.0f, 8.0f), tickColor);
    }
}

glm::mat4 Renderer::buildViewMatrix() const
{
    return glm::lookAt(m_cameraPosition, m_cameraTarget, m_cameraUp);
}

glm::mat4 Renderer::buildProjectionMatrix() const
{
    const int safeHeight = std::max(m_viewportHeight, 1);
    float aspectRatio = static_cast<float>(m_viewportWidth) / static_cast<float>(safeHeight);
    return glm::perspective(glm::radians(m_cameraFOV), aspectRatio, 0.1f, 10000.0f);
}

void Renderer::renderExplosion(const glm::vec3 &position, float size)
{
    // Safety check - make sure explosion VAO is valid and parameters are valid
    if (m_explosionVAO == 0 || size <= 0.0f)
    {
        return;
    }

    // Safety check for invalid position
    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
    {
        std::cerr << "Invalid explosion position detected" << std::endl;
        return;
    }

    // Safety check for invalid size
    if (std::isnan(size) || std::isinf(size))
    {
        std::cerr << "Invalid explosion size detected" << std::endl;
        return;
    }

    // Save current OpenGL state
    GLint lastBlendSrc = 0, lastBlendDst = 0;
    GLboolean lastBlendEnabled = GL_FALSE;

    try
    {
        glGetIntegerv(GL_BLEND_SRC, &lastBlendSrc);
        glGetIntegerv(GL_BLEND_DST, &lastBlendDst);
        lastBlendEnabled = glIsEnabled(GL_BLEND);
    }
    catch (...)
    {
        std::cerr << "Error saving OpenGL state" << std::endl;
        // Continue anyway, we'll try to restore default state if this fails
    }

    // Enable additive blending for a glowing effect
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // Create model matrix for the explosion
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, glm::vec3(size));

    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    bool success = true;
    try
    {
        // Use shader program
        glUseProgram(m_shaderProgram);

        // Check if shader program is valid
        if (m_shaderProgram == 0)
        {
            std::cerr << "Invalid shader program" << std::endl;
            success = false;
        }
        else
        {
            // Pass transformation matrices to the shader
            if (m_modelLoc != -1)
                glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            if (m_viewLoc != -1)
                glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            if (m_projLoc != -1)
                glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
            if (m_cameraPosLoc != -1)
                glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));

            // Draw explosion
            glBindVertexArray(m_explosionVAO);
            glDrawElements(GL_TRIANGLES, m_explosionIndices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }
    catch (...)
    {
        // In case of an OpenGL error, restore state and continue
        std::cerr << "Error rendering explosion" << std::endl;
        success = false;
    }

    // Restore previous OpenGL state
    try
    {
        if (lastBlendEnabled)
        {
            glEnable(GL_BLEND);
            glBlendFunc(lastBlendSrc, lastBlendDst);
        }
        else
        {
            glDisable(GL_BLEND);
        }
    }
    catch (...)
    {
        std::cerr << "Error restoring OpenGL state" << std::endl;
        // If we can't restore the state, set to default safe values
        glDisable(GL_BLEND);
    }
}

void Renderer::renderPoint(const glm::vec3 &position, const glm::vec3 &color, float size)
{
    try
    {
        // Point vertex data: position (xyz) and color (rgb)
        float pointVertex[6] = {
            position.x, position.y, position.z, color.r, color.g, color.b
        };

        // Use line shader program (works for points too)
        glUseProgram(m_lineShaderProgram);

        glm::mat4 view = buildViewMatrix();
        glm::mat4 projection = buildProjectionMatrix();

        // Set uniform values
        GLuint viewLoc = glGetUniformLocation(m_lineShaderProgram, "view");
        GLuint projLoc = glGetUniformLocation(m_lineShaderProgram, "projection");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

        // Bind VAO and update VBO with point vertex
        glBindVertexArray(m_lineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(pointVertex), pointVertex);

        // Set point size (may be limited by hardware)
        glPointSize(size);

        // Draw the point
        glDrawArrays(GL_POINTS, 0, 1);

        // Reset state
        glPointSize(1.0f);
        glBindVertexArray(0);
        glUseProgram(0);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error rendering point: " << e.what() << std::endl;
    }
}
