#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstddef>
#include <filesystem>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

class PhysicsObject;

class Renderer
{
public:
    Renderer();
    ~Renderer();

    void initialize();
    void renderEnvironment();
    void render(PhysicsObject *object);
    void renderAll(const std::vector<PhysicsObject *> &objects);
    void setEnvironmentMetrics(float groundHalfExtent, float airspaceHalfExtent, float airspaceHeight);

    // Visual effects
    void renderExplosion(const glm::vec3 &position, float size);

    // Debug visualization
    void renderLine(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color = glm::vec3(1.0f, 1.0f, 1.0f));
    void renderPoint(const glm::vec3 &position, const glm::vec3 &color = glm::vec3(1.0f, 1.0f, 1.0f), float size = 5.0f);
    void renderText(const glm::vec3 &position, const std::string &text, const glm::vec3 &color = glm::vec3(1.0f, 1.0f, 1.0f));
    void flushDebugPrimitives();
    void clearDebugPrimitives();

    // Camera controls
    void setCameraPosition(const glm::vec3 &position);
    void setCameraTarget(const glm::vec3 &target);
    void rotateCameraYaw(float deltaDegrees);
    void rotateCameraPitch(float deltaDegrees);
    void moveCameraForward(float distance);
    void moveCameraRight(float distance);
    void moveCameraUp(float distance);

    // Camera settings
    void setCameraFOV(float degrees) { m_cameraFOV = degrees; }
    float getCameraFOV() const { return m_cameraFOV; }
    void setCameraSpeed(float speed) { m_cameraSpeed = speed; }
    float getCameraSpeed() const { return m_cameraSpeed; }
    float getSceneFarPlane() const { return m_sceneFarPlane; }

    // Viewport settings
    void setViewportSize(int width, int height)
    {
        m_viewportWidth = width;
        m_viewportHeight = height;
    }

    // Getters for camera properties
    const glm::vec3 &getCameraPosition() const { return m_cameraPosition; }
    const glm::vec3 &getCameraTarget() const { return m_cameraTarget; }
    const glm::vec3 &getCameraUp() const { return m_cameraUp; }
    const glm::vec3 &getCameraFront() const { return m_cameraFront; }
    const glm::vec3 &getCameraRight() const { return m_cameraRight; }

private:
    struct Vertex
    {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec3 color;
    };

    struct DebugVertex
    {
        glm::vec3 position;
        glm::vec3 color;
        float size;
    };

    void createShaders();
    void createSimpleCube();
    void createMissileModel();
    void createFloor();
    void createTargetModel();
    void createExplosionModel();
    void createLineRendering();
    void uploadFloorMesh();
    void renderObject(PhysicsObject *object, const glm::mat4 &modelMatrix);
    void renderFloor();
    void renderWorldGuides();
    void renderGroundCircle(float radius, const glm::vec3 &color, int segments = 48);
    void renderAirspaceBeacon(const glm::vec3 &basePosition, float height, const glm::vec3 &color);
    bool loadObjModel(const std::string &relativePath,
                      std::vector<Vertex> &vertices,
                      std::vector<unsigned int> &indices,
                      const glm::vec3 &baseColor,
                      const glm::mat4 &preTransform,
                      float targetExtent);
    std::filesystem::path resolveAssetPath(const std::string &relativePath) const;
    void normalizeMesh(std::vector<Vertex> &vertices, float targetExtent) const;
    void ensureDebugBufferCapacity(std::size_t vertexCount);
    void updateCameraVectors();
    glm::mat4 buildViewMatrix() const;
    glm::mat4 buildProjectionMatrix() const;

    // OpenGL resources
    GLuint m_vao;           // Vertex Array Object
    GLuint m_vbo;           // Vertex Buffer Object
    GLuint m_ebo;           // Element Buffer Object
    GLuint m_shaderProgram; // Shader program

    // Line rendering resources
    GLuint m_lineVAO;
    GLuint m_lineVBO;
    GLuint m_lineShaderProgram;
    GLint m_lineViewLoc = -1;
    GLint m_lineProjLoc = -1;
    std::size_t m_lineBufferCapacity = 0;
    std::vector<DebugVertex> m_debugLineVertices;
    std::vector<DebugVertex> m_debugPointVertices;

    // Floor resources
    GLuint m_floorVAO;
    GLuint m_floorVBO;
    GLuint m_floorEBO;
    std::vector<Vertex> m_floorVertices;
    std::vector<unsigned int> m_floorIndices;

    // Target resources
    GLuint m_targetVAO;
    GLuint m_targetVBO;
    GLuint m_targetEBO;
    std::vector<Vertex> m_targetVertices;
    std::vector<unsigned int> m_targetIndices;

    // Explosion resources
    GLuint m_explosionVAO;
    GLuint m_explosionVBO;
    GLuint m_explosionEBO;
    std::vector<Vertex> m_explosionVertices;
    std::vector<unsigned int> m_explosionIndices;

    // Model meshes for different object types
    std::unordered_map<std::string, std::pair<std::vector<Vertex>, std::vector<unsigned int>>> m_modelMeshes;

    // Camera settings
    glm::vec3 m_cameraPosition; // Position of the camera
    glm::vec3 m_cameraTarget;   // Point the camera is looking at
    glm::vec3 m_cameraUp;       // Up vector (0,1,0 typically)
    glm::vec3 m_cameraFront;    // Direction vector the camera is facing
    glm::vec3 m_cameraRight;    // Right vector of the camera
    float m_cameraYaw;          // Yaw angle in degrees
    float m_cameraPitch;        // Pitch angle in degrees
    float m_cameraSpeed;        // Movement speed
    float m_cameraFOV;          // Field of view in degrees

    // Viewport dimensions
    int m_viewportWidth = 1280;
    int m_viewportHeight = 720;
    float m_groundHalfExtent = 1200.0f;
    float m_airspaceHalfExtent = 600.0f;
    float m_airspaceHeight = 320.0f;
    float m_sceneFarPlane = 20000.0f;

    // Mesh data
    std::vector<Vertex> m_vertices;
    std::vector<unsigned int> m_indices;

    // Shader locations
    GLint m_modelLoc;
    GLint m_viewLoc;
    GLint m_projLoc;
    GLint m_cameraPosLoc;
    GLint m_fogDensityLoc;
};
