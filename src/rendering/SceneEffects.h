#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <random>
#include <vector>

class SceneEffects
{
public:
    SceneEffects();
    ~SceneEffects();

    void initialize();
    void shutdown();

    void setViewportSize(int width, int height);
    void setCamera(const glm::vec3 &cameraPosition, const glm::mat4 &view, const glm::mat4 &projection);

    void beginScene(const glm::vec3 &clearColor);
    void renderParticlesToScene();
    void presentScene();

    void update(float deltaTime);
    void clear();

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
    void emitFlareEffect(const glm::vec3 &start,
                         const glm::vec3 &end,
                         const glm::vec3 &carrierVelocity,
                         float heatFraction);
    void spawnExplosion(const glm::vec3 &position,
                        const glm::vec3 &velocityHint = glm::vec3(0.0f),
                        float intensity = 1.0f);

private:
    enum class ParticleMaterial
    {
        FLAME = 0,
        SMOKE = 1,
        SPARK = 2,
        GLOW = 3
    };

    enum class BlendMode
    {
        ALPHA,
        ADDITIVE
    };

    struct EffectParticle
    {
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 axis{0.0f, 1.0f, 0.0f};
        glm::vec4 color{1.0f};
        float age = 0.0f;
        float lifetime = 1.0f;
        float startSize = 1.0f;
        float endSize = 1.0f;
        float stretch = 1.0f;
        float rotation = 0.0f;
        float angularVelocity = 0.0f;
        float softness = 0.5f;
        float emissive = 1.0f;
        float seed = 0.0f;
        float drag = 0.0f;
        float upwardAcceleration = 0.0f;
        ParticleMaterial material = ParticleMaterial::SMOKE;
        BlendMode blendMode = BlendMode::ALPHA;
    };

    struct HeatHazeSprite
    {
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 axis{0.0f, 1.0f, 0.0f};
        float age = 0.0f;
        float lifetime = 1.0f;
        float radius = 1.0f;
        float stretch = 1.0f;
        float rotation = 0.0f;
        float angularVelocity = 0.0f;
        float strength = 0.0f;
        float seed = 0.0f;
        float drag = 0.0f;
    };

    struct ParticleInstance
    {
        glm::vec4 centerRotation;
        glm::vec4 axisSizeX;
        glm::vec4 color;
        glm::vec4 params0;
        glm::vec4 params1;
    };

    struct HazeInstance
    {
        glm::vec4 centerRotation;
        glm::vec4 axisSizeX;
        glm::vec4 params0;
    };

    void createShaders();
    void createBuffers();
    void destroyBuffers();
    void destroySceneFramebuffer();
    void ensureSceneFramebuffer();
    void ensureParticleInstanceCapacity(std::size_t instanceCount);
    void ensureHazeInstanceCapacity(std::size_t instanceCount);
    void renderParticlePass(const std::vector<ParticleInstance> &instances, BlendMode blendMode);
    void renderHeatHazePass();

    void addParticle(const EffectParticle &particle);
    void addHeatHaze(const HeatHazeSprite &sprite);
    void emitEngineTrail(const glm::vec3 &start,
                         const glm::vec3 &end,
                         const glm::vec3 &forward,
                         const glm::vec3 &carrierVelocity,
                         float intensity,
                         bool missilePreset);
    float randomRange(float minimum, float maximum);
    glm::vec3 randomInUnitSphere();
    glm::vec3 randomUnitVector();

    bool m_initialized = false;

    int m_viewportWidth = 1280;
    int m_viewportHeight = 720;
    glm::vec3 m_cameraPosition{0.0f};
    glm::mat4 m_view{1.0f};
    glm::mat4 m_projection{1.0f};

    GLuint m_particleProgram = 0;
    GLuint m_hazeProgram = 0;
    GLuint m_compositeProgram = 0;

    GLuint m_quadVBO = 0;
    GLuint m_particleVAO = 0;
    GLuint m_particleInstanceVBO = 0;
    GLuint m_hazeVAO = 0;
    GLuint m_hazeInstanceVBO = 0;
    GLuint m_fullscreenVAO = 0;
    GLuint m_fullscreenVBO = 0;

    GLuint m_sceneFramebuffer = 0;
    GLuint m_sceneColorTexture = 0;
    GLuint m_sceneDepthTexture = 0;
    bool m_sceneFramebufferValid = false;
    std::size_t m_particleInstanceCapacity = 0;
    std::size_t m_hazeInstanceCapacity = 0;

    std::vector<EffectParticle> m_particles;
    std::vector<HeatHazeSprite> m_heatHazeSprites;
    std::mt19937 m_rng;
};
