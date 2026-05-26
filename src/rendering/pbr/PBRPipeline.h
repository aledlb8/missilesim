#pragma once

#include "PBRShader.h"
#include "PBRFrameBuffer.h"
#include "PBRSkybox.h"
#include "PBRCubeMap.h"
#include "PBRLight.h"
#include "PBRMeshPrimitives.h"
#include "PBRModel.h"
#include "ClusterGrid.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <filesystem>
#include <string>
#include <vector>

namespace pbr {

class PBRPipeline
{
public:
    PBRPipeline() = default;
    ~PBRPipeline();

    PBRPipeline(const PBRPipeline &) = delete;
    PBRPipeline &operator=(const PBRPipeline &) = delete;

    /// Initialize the entire PBR pipeline: shaders, FBOs, SSBOs, IBL.
    bool initialize(int width, int height,
                    float nearPlane, float farPlane,
                    const std::filesystem::path &assetDir);
    void shutdown();

    /// Rebuild FBOs after window resize.
    void resize(int width, int height);

    // ---- Scene configuration ----

    void setSkybox(const std::string &skyboxName, int resolution = 512);
    void setDirectionalLight(const DirectionalLight &light);
    void setPointLights(const std::vector<PointLight> &lights);
    void setExposure(float exposure) { m_exposure = exposure; }
    void setBloomPasses(int passes) { m_bloomPasses = passes; }
    float getExposure() const { return m_exposure; }
    int getBloomPasses() const { return m_bloomPasses; }
    DirectionalLight &directionalLight() { return m_dirLight; }
    const DirectionalLight &directionalLight() const { return m_dirLight; }

    // ---- Per-frame rendering ----

    /// Call at the start of each frame with current camera state.
    void beginFrame(const glm::mat4 &view, const glm::mat4 &projection,
                    const glm::vec3 &cameraPos);

    /// Queue a textured PBR model for rendering.
    void submitModel(Model &model, const glm::mat4 &modelMatrix);

    /// Queue a legacy (untextured) mesh for PBR rendering.
    void submitLegacyMesh(GLuint vao, GLsizei indexCount,
                          const glm::mat4 &modelMatrix,
                          const glm::vec3 &albedo,
                          float metallic, float roughness);

    /// Execute shadows, depth prepass, light culling, shading, skybox, MSAA resolve.
    void executeRenderPass();

    /// Access resolved FBO for particle compositing (between execute and postProcess).
    GLuint getResolvedColorTexture() const;
    GLuint getResolvedDepthTexture() const;
    void bindResolvedFBO();

    /// Bloom + tone mapping to default framebuffer.
    void postProcess();

    bool isValid() const { return m_initialized; }

private:
    // Draw call structures
    struct ModelDrawCall
    {
        Model *model;
        glm::mat4 modelMatrix;
    };

    struct LegacyDrawCall
    {
        GLuint vao;
        GLsizei indexCount;
        glm::mat4 modelMatrix;
        glm::vec3 albedo;
        float metallic;
        float roughness;
    };

    // Initialization stages
    bool loadShaders();
    bool initFBOs();
    void initIBL();
    void initShadowMaps();

    // Render passes
    void shadowPass();
    void depthPrePass();
    void lightCulling();
    void mainShadingPass();
    void renderSkybox();

    // Shared setup for PBR shaders
    void bindPBRUniforms(Shader &shader);
    void bindShadowTextures(Shader &shader, int startUnit);

    // Dimensions
    int m_width = 0;
    int m_height = 0;
    float m_nearPlane = 0.1f;
    float m_farPlane = 2000.0f;
    std::filesystem::path m_assetDir;

    // Rendering parameters
    float m_exposure = 1.0f;
    int m_bloomPasses = 5;

    // Per-frame camera state
    glm::mat4 m_viewMatrix{1.0f};
    glm::mat4 m_projectionMatrix{1.0f};
    glm::vec3 m_cameraPos{0.0f};

    // --- Shaders ---
    // Pre-processing
    Shader m_fillCubeMapShader;
    Shader m_convolveShader;
    Shader m_preFilterShader;
    Shader m_brdfIntegralShader;
    ComputeShader m_buildGridShader;
    ComputeShader m_cullLightsShader;
    // Rendering
    Shader m_pbrShader;
    Shader m_pbrSimpleShader;
    Shader m_depthPassShader;
    Shader m_skyboxShader;
    Shader m_screenShader;
    // Shadows
    Shader m_dirShadowShader;
    Shader m_pointShadowShader;
    // Post-processing
    Shader m_highPassShader;
    Shader m_blurShader;

    // --- FBOs ---
    FrameBufferMultiSampled m_multisampledFBO;
    ResolveBuffer m_resolveFBO;
    QuadHDRBuffer m_pingPongFBO;
    QuadHDRBuffer m_simpleFBO;
    CaptureBuffer m_captureFBO;
    DirShadowBuffer m_dirShadowFBO;
    std::vector<PointShadowBuffer> m_pointShadowFBOs;

    // --- Scene resources ---
    Skybox m_skybox;
    CubeMap m_irradianceMap;
    CubeMap m_specFilteredMap;
    GLuint m_brdfLUTTexture = 0;
    Quad m_canvas;

    // --- Lighting ---
    DirectionalLight m_dirLight;
    std::vector<PointLight> m_pointLights;
    ClusterGrid m_clusterGrid;

    // --- Draw call queues ---
    std::vector<ModelDrawCall> m_modelDrawCalls;
    std::vector<LegacyDrawCall> m_legacyDrawCalls;

    bool m_initialized = false;
    bool m_skyboxReady = false;
};

} // namespace pbr
