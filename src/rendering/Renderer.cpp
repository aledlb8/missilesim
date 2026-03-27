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

Renderer::Renderer()
    : m_vao(0), m_vbo(0), m_ebo(0), m_shaderProgram(0),
      m_lineVAO(0), m_lineVBO(0), m_lineShaderProgram(0),
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
    m_sceneEffects = std::make_unique<SceneEffects>();
    m_sceneEffects->initialize();

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
    m_fogDensityLoc = glGetUniformLocation(m_shaderProgram, "fogDensity");

    // Initialize camera vectors
    updateCameraVectors();

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
}