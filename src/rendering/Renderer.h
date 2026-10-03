#pragma once

#include "pbr/PBRLight.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class PhysicsObject;
class SceneEffects;

namespace pbr { class PBRPipeline; }
namespace missilesim::sim { class Terrain; }

class Renderer
{
public:
    // A 1 m near plane: the closest camera (the missile chase) sits 8 m back,
    // and the 24-bit depth buffer keeps metre-level precision at 10 km, so
    // distant slopes and shorelines do not flicker.
    static constexpr float kNearPlane = 1.0f;

    struct ExhaustSocket
    {
        glm::vec3 position{0.0f};
        glm::vec3 direction{0.0f, -1.0f, 0.0f};
        float radius = 0.0f;
    };
    // World-space sockets use the very same matrix as the rendered mesh.
    std::vector<ExhaustSocket> getExhaustSockets(const PhysicsObject &object) const;
    bool hasAircraftModel(const char *aircraftId) const;
    Renderer();
    ~Renderer();

    void initialize();
    void beginSceneFrame(const glm::vec3 &clearColor);
    void renderSceneEffects();
    void presentSceneFrame();
    void updateEffects(float deltaTime);
    void clearEffects();
    void renderEnvironment();
    void render(PhysicsObject *object);
    void renderAll(const std::vector<PhysicsObject *> &objects);
    // The ground mesh is built from this terrain's own samples, so the drawn
    // surface is the one the simulation collides with. Null draws flat ground.
    void setTerrain(std::shared_ptr<const missilesim::sim::Terrain> terrain);

    // Visual effects
    void renderExplosion(const glm::vec3 &position, float size);
    void emitMissileExhaust(const glm::vec3 &start,
                            const glm::vec3 &end,
                            const glm::vec3 &forward,
                            const glm::vec3 &carrierVelocity,
                            float intensity);
    void emitJetAfterburner(const glm::vec3 &start,
                            const glm::vec3 &end,
                            const glm::vec3 &forward,
                            const glm::vec3 &carrierVelocity,
                            float intensity);
    void emitJetWake(const glm::vec3 &start,
                     const glm::vec3 &end,
                     const glm::vec3 &forward,
                     const glm::vec3 &carrierVelocity,
                     float intensity);
    void emitFlareEffect(const glm::vec3 &start,
                         const glm::vec3 &end,
                         const glm::vec3 &carrierVelocity,
                         float heatFraction);
    void spawnMissileLaunchEffect(const glm::vec3 &position,
                                  const glm::vec3 &forward,
                                  const glm::vec3 &carrierVelocity,
                                  float intensity = 1.0f);
    void spawnLaunchGroundCloudEffect(const glm::vec3 &position,
                                      const glm::vec3 &up,
                                      float intensity = 1.0f);
    void spawnExplosionEffect(const glm::vec3 &position,
                              const glm::vec3 &velocityHint = glm::vec3(0.0f),
                              float intensity = 1.0f);

    // Distance from the missile mesh centre to its tail tip along the body
    // axis, so a vertically-staged round can rest its base exactly on the
    // ground (centre height = ground + this offset).
    float getMissileGroundRestOffset() const { return m_missileGroundRestOffset; }

    // Debug visualization
    void renderLine(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color = glm::vec3(1.0f, 1.0f, 1.0f));
    void renderPoint(const glm::vec3 &position, const glm::vec3 &color = glm::vec3(1.0f, 1.0f, 1.0f), float size = 5.0f);
    void renderText(const glm::vec3 &position, const std::string &text, const glm::vec3 &color = glm::vec3(1.0f, 1.0f, 1.0f));
    void flushDebugPrimitives();
    void clearDebugPrimitives();

    // Camera controls
    void setCameraPosition(const glm::vec3 &position);
    void setCameraTarget(const glm::vec3 &target);
    // Full orientation for the chase and mouse-aim rigs. Unlike the yaw/pitch
    // path it keeps the given up vector, so the view can roll and can look
    // straight up or down without flipping.
    void setCameraView(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &up);
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
    void setViewportSize(int width, int height);

    // PBR rendering controls
    bool hasPBR() const;
    void setPBRExposure(float exposure);
    float getPBRExposure() const;
    void setPBRBloomPasses(int passes);
    int getPBRBloomPasses() const;
    void setPBRBloomStrength(float strength);
    float getPBRBloomStrength() const;
    void setPBRShadowsEnabled(bool enabled);
    bool getPBRShadowsEnabled() const;
    void setPBRFogDensityScale(float scale);
    float getPBRFogDensityScale() const;
    void setEffectLightsEnabled(bool enabled);
    bool getEffectLightsEnabled() const;
    void setSunOrientation(float azimuthDeg, float elevationDeg, float intensity);
    void getSunOrientation(float &azimuthDeg, float &elevationDeg, float &intensity) const;

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
        glm::vec2 metalRoughness{0.2f, 0.45f};
    };

    struct DebugVertex
    {
        glm::vec3 position;
        glm::vec3 color;
        float size;
    };

    // One anti-aliased line segment instance (screen-space expanded quad).
    struct LineInstance
    {
        glm::vec4 start;  // xyz world, w = width in pixels
        glm::vec4 end;    // xyz world
        glm::vec4 color;  // rgb + alpha
    };

    // Transient dynamic light emitted by a visual effect (explosion flash,
    // launch plume, engine exhaust); fed into the PBR clustered light system
    // so effects illuminate nearby geometry.
    struct EffectLight
    {
        enum class Envelope
        {
            Flash,  // sharp exponential decay
            Ember,  // quadratic fade over lifetime
            Steady  // constant while alive
        };

        glm::vec3 position{0.0f};
        glm::vec3 color{1.0f};
        float intensity = 0.0f;  // radiance scale at 1 m (pre-attenuation)
        float radius = 50.0f;    // attenuation window (m)
        float age = 0.0f;
        float lifetime = 0.0f;   // seconds; <= 0 lives for a single frame
        float seed = 0.0f;       // flicker phase offset
        Envelope envelope = Envelope::Steady;
    };

    void createShaders();
    void createSimpleCube();
    void createMissileModel();
    void createFloor();
    void createTargetModel();
    void createAircraftModels();
    void createExplosionModel();
    void createLineRendering();
    void createAALineRendering();
    void uploadFloorMesh();
    void renderObject(PhysicsObject *object, const glm::mat4 &modelMatrix);
    void renderFloor();
    void renderWater();
    void createWaterMesh();
    void uploadTerrainSurface();
    glm::vec3 keepCameraAboveGround(const glm::vec3 &position) const;
    static constexpr float kCameraGroundClearance = 3.0f;
    bool loadObjModel(const std::string &relativePath,
                      std::vector<Vertex> &vertices,
                      std::vector<unsigned int> &indices,
                      const glm::vec3 &baseColor,
                      const glm::mat4 &preTransform,
                      float targetExtent,
                      std::vector<ExhaustSocket> *sockets = nullptr);
    std::filesystem::path resolveAssetPath(const std::string &relativePath) const;
    void normalizeMesh(std::vector<Vertex> &vertices, float targetExtent,
                       std::vector<ExhaustSocket> *sockets = nullptr) const;
    glm::mat4 buildObjectModelMatrix(const PhysicsObject &object) const;
    void submitEnginePlumes(const PhysicsObject &object);
    void ensureDebugBufferCapacity(std::size_t vertexCount);
    void flushDebugPrimitivesInternal();
    void flushAALinesInternal();
    void addEffectLight(const EffectLight &light);
    void updateEffectLights(float deltaTime);
    void uploadEffectLights();
    void updateCameraVectors();
    glm::mat4 buildViewMatrix() const;
    glm::mat4 buildProjectionMatrix() const;
    bool isPBRActive() const;

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

    // Anti-aliased instanced line rendering (PBR path)
    GLuint m_aaLineVAO = 0;
    GLuint m_aaLineInstanceVBO = 0;
    GLuint m_aaLineProgram = 0;
    GLint m_aaLineViewProjLoc = -1;
    GLint m_aaLineViewportLoc = -1;
    std::size_t m_aaLineInstanceCapacity = 0;
    std::vector<LineInstance> m_lineInstances;

    // Floor resources
    GLuint m_floorVAO;
    GLuint m_floorVBO;
    GLuint m_floorEBO;
    // Water: a plane at the terrain's water level that follows the camera,
    // and the land heights the water shader reads for depth and shoreline.
    GLuint m_waterVAO = 0;
    GLuint m_waterVBO = 0;
    GLuint m_waterEBO = 0;
    GLsizei m_waterIndexCount = 0;
    GLuint m_heightmapTexture = 0;
    std::chrono::steady_clock::time_point m_startTime = std::chrono::steady_clock::now();
    std::vector<Vertex> m_floorVertices;
    std::vector<unsigned int> m_floorIndices;

    // Target resources
    GLuint m_targetVAO;
    GLuint m_targetVBO;
    GLuint m_targetEBO;
    std::vector<Vertex> m_targetVertices;
    std::vector<unsigned int> m_targetIndices;

    struct AircraftMesh
    {
        GLuint vao = 0, vbo = 0, ebo = 0;
        GLsizei indexCount = 0;
        std::vector<ExhaustSocket> sockets;
    };
    // Immutable GPU meshes, keyed by the same id as the flight catalog. Both
    // render paths and exhaust effects resolve the current object's id.
    std::unordered_map<std::string, AircraftMesh> m_aircraftMeshes;
    const AircraftMesh *aircraftMesh(const PhysicsObject &object) const;

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
    // The world has a fixed size: the flat floor (or the skirt past a dry
    // heightfield) reaches this far, and the far plane sees the horizon.
    float m_groundHalfExtent = 40000.0f;
    std::shared_ptr<const missilesim::sim::Terrain> m_terrain;
    float m_sceneFarPlane = 60000.0f;

    std::unique_ptr<SceneEffects> m_sceneEffects;
    std::vector<ExhaustSocket> m_missileExhaustSockets;
    std::vector<ExhaustSocket> m_targetExhaustSockets;
    std::unique_ptr<pbr::PBRPipeline> m_pbrPipeline;
    bool m_usePBR = true;

    // Dynamic effect lights (aged in updateEffects, uploaded each frame)
    std::vector<EffectLight> m_effectLights;
    std::vector<pbr::PointLight> m_effectLightScratch;
    bool m_effectLightsEnabled = true;

    // Mesh data
    std::vector<Vertex> m_vertices;
    std::vector<unsigned int> m_indices;
    float m_missileGroundRestOffset = 1.0f;

    // Shader locations
    GLint m_modelLoc;
    GLint m_viewLoc;
    GLint m_projLoc;
    GLint m_cameraPosLoc;
    GLint m_fogDensityLoc;
};
