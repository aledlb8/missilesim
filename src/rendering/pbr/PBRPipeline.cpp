#include "PBRPipeline.h"

#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace pbr {

// ===========================================================================
// Lifetime
// ===========================================================================

PBRPipeline::~PBRPipeline()
{
    shutdown();
}

void PBRPipeline::shutdown()
{
    if (m_brdfLUTTexture != 0)
    {
        glDeleteTextures(1, &m_brdfLUTTexture);
        m_brdfLUTTexture = 0;
    }
    m_clusterGrid.shutdown();
    m_initialized = false;
}

// ===========================================================================
// Initialize
// ===========================================================================

bool PBRPipeline::initialize(int width, int height,
                              float nearPlane, float farPlane,
                              const std::filesystem::path &assetDir)
{
    m_width = width;
    m_height = height;
    m_nearPlane = nearPlane;
    m_farPlane = farPlane;
    m_assetDir = assetDir;

    std::cout << "PBR: Initializing pipeline (" << width << "x" << height << ")" << std::endl;

    if (!loadShaders())
        return false;

    if (!initFBOs())
        return false;

    // Initialize canvas quad for screen-space passes
    m_canvas.setup();

    // Initialize cluster grid SSBOs
    glm::mat4 defaultProj = glm::perspective(
        glm::radians(60.0f),
        static_cast<float>(width) / static_cast<float>(height),
        nearPlane, farPlane);

    m_clusterGrid.initialize(
        ClusterGrid::kGridSizeX,
        ClusterGrid::kGridSizeY,
        ClusterGrid::kGridSizeZ,
        static_cast<unsigned int>(width),
        static_cast<unsigned int>(height),
        nearPlane, farPlane,
        defaultProj);

    // Build cluster AABB grid via compute shader
    m_buildGridShader.use();
    m_buildGridShader.setFloat("zNear", nearPlane);
    m_buildGridShader.setFloat("zFar", farPlane);
    m_buildGridShader.dispatch(
        ClusterGrid::kGridSizeX,
        ClusterGrid::kGridSizeY,
        ClusterGrid::kGridSizeZ);

    m_initialized = true;
    std::cout << "PBR: Pipeline initialized successfully." << std::endl;
    return true;
}

// ===========================================================================
// Shader loading
// ===========================================================================

bool PBRPipeline::loadShaders()
{
    std::filesystem::path shaderDir = m_assetDir / "shaders" / "pbr";
    std::filesystem::path compDir = shaderDir / "ComputeShaders";

    bool ok = true;

    // Pre-processing shaders
    ok &= m_buildGridShader.load(compDir / "clusterShader.comp");
    ok &= m_cullLightsShader.load(compDir / "clusterCullLightShader.comp");
    ok &= m_fillCubeMapShader.load(shaderDir / "cubeMapShader.vert",
                                    shaderDir / "buildCubeMapShader.frag");
    ok &= m_convolveShader.load(shaderDir / "cubeMapShader.vert",
                                 shaderDir / "convolveCubemapShader.frag");
    ok &= m_preFilterShader.load(shaderDir / "cubeMapShader.vert",
                                  shaderDir / "preFilteringShader.frag");
    ok &= m_brdfIntegralShader.load(shaderDir / "screenShader.vert",
                                     shaderDir / "brdfIntegralShader.frag");

    if (!ok)
    {
        std::cerr << "PBR: Failed to load pre-processing shaders!" << std::endl;
        return false;
    }

    // Rendering shaders
    ok &= m_pbrShader.load(shaderDir / "PBRClusteredShader.vert",
                            shaderDir / "PBRClusteredShader.frag");
    ok &= m_pbrSimpleShader.load(shaderDir / "PBRSimpleShader.vert",
                                  shaderDir / "PBRSimpleShader.frag");
    ok &= m_depthPassShader.load(shaderDir / "depthPassShader.vert",
                                  shaderDir / "depthPassShader.frag");
    ok &= m_skyboxShader.load(shaderDir / "skyboxShader.vert",
                               shaderDir / "skyboxShader.frag");
    ok &= m_screenShader.load(shaderDir / "screenShader.vert",
                               shaderDir / "screenShader.frag");

    if (!ok)
    {
        std::cerr << "PBR: Failed to load rendering shaders!" << std::endl;
        return false;
    }

    // Shadow shaders
    ok &= m_dirShadowShader.load(shaderDir / "shadowShader.vert",
                                  shaderDir / "shadowShader.frag");
    ok &= m_pointShadowShader.load(shaderDir / "pointShadowShader.vert",
                                    shaderDir / "pointShadowShader.frag",
                                    shaderDir / "pointShadowShader.geom");

    if (!ok)
    {
        std::cerr << "PBR: Failed to load shadow shaders!" << std::endl;
        return false;
    }

    // Post-processing shaders
    ok &= m_highPassShader.load(shaderDir / "splitHighShader.vert",
                                 shaderDir / "splitHighShader.frag");
    ok &= m_blurShader.load(shaderDir / "blurShader.vert",
                              shaderDir / "blurShader.frag");

    if (!ok)
    {
        std::cerr << "PBR: Failed to load post-processing shaders!" << std::endl;
        return false;
    }

    std::cout << "PBR: All shaders loaded successfully." << std::endl;
    return true;
}

// ===========================================================================
// FBO initialization
// ===========================================================================

bool PBRPipeline::initFBOs()
{
    m_multisampledFBO.setupFrameBuffer(m_width, m_height);
    m_resolveFBO.setupFrameBuffer(m_width, m_height);
    m_pingPongFBO.setupFrameBuffer(m_width, m_height);
    m_simpleFBO.setupFrameBuffer(m_width, m_height);

    std::cout << "PBR: FBOs initialized." << std::endl;
    return true;
}

// ===========================================================================
// Resize
// ===========================================================================

void PBRPipeline::resize(int width, int height)
{
    if (width <= 0 || height <= 0)
        return;

    m_width = width;
    m_height = height;

    // Rebuild rendering FBOs
    m_multisampledFBO.setupFrameBuffer(width, height);
    m_resolveFBO.setupFrameBuffer(width, height);
    m_pingPongFBO.setupFrameBuffer(width, height);
    m_simpleFBO.setupFrameBuffer(width, height);

    // Rebuild cluster grid
    glm::mat4 proj = glm::perspective(
        glm::radians(60.0f),
        static_cast<float>(width) / static_cast<float>(height),
        m_nearPlane, m_farPlane);

    m_clusterGrid.initialize(
        ClusterGrid::kGridSizeX,
        ClusterGrid::kGridSizeY,
        ClusterGrid::kGridSizeZ,
        static_cast<unsigned int>(width),
        static_cast<unsigned int>(height),
        m_nearPlane, m_farPlane,
        proj);

    m_buildGridShader.use();
    m_buildGridShader.setFloat("zNear", m_nearPlane);
    m_buildGridShader.setFloat("zFar", m_farPlane);
    m_buildGridShader.dispatch(
        ClusterGrid::kGridSizeX,
        ClusterGrid::kGridSizeY,
        ClusterGrid::kGridSizeZ);
}

// ===========================================================================
// Scene configuration
// ===========================================================================

void PBRPipeline::setSkybox(const std::string &skyboxName, int resolution)
{
    m_skybox.setup(skyboxName, true, static_cast<unsigned int>(resolution), m_assetDir);

    // Set up capture FBO for IBL
    m_captureFBO.setupFrameBuffer(resolution, resolution);
    m_captureFBO.bind();

    // Convert equirectangular → cubemap
    m_skybox.fillCubeMapWithTexture(m_fillCubeMapShader);

    // Generate irradiance map (diffuse IBL)
    unsigned int irradianceRes = 32;
    m_irradianceMap.generateCubeMap(irradianceRes, irradianceRes, CubeMapType::HDR);
    m_captureFBO.resizeFrameBuffer(irradianceRes, irradianceRes);
    m_captureFBO.bind();
    m_irradianceMap.convolveCubeMap(m_skybox.cubeMapTextureID(), m_convolveShader);

    // Generate pre-filtered specular map
    unsigned int prefilterRes = 128;
    m_specFilteredMap.generateCubeMap(prefilterRes, prefilterRes, CubeMapType::Prefilter);
    m_captureFBO.bind();
    m_specFilteredMap.preFilterCubeMap(m_skybox.cubeMapTextureID(),
                                       m_captureFBO.rbo(),
                                       m_preFilterShader);

    // Generate BRDF LUT
    unsigned int brdfRes = 512;
    if (m_brdfLUTTexture != 0)
        glDeleteTextures(1, &m_brdfLUTTexture);

    glGenTextures(1, &m_brdfLUTTexture);
    glBindTexture(GL_TEXTURE_2D, m_brdfLUTTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, brdfRes, brdfRes, 0, GL_RG, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    m_captureFBO.resizeFrameBuffer(brdfRes, brdfRes);
    m_captureFBO.bind();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           m_brdfLUTTexture, 0);
    glViewport(0, 0, brdfRes, brdfRes);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_brdfIntegralShader.use();
    m_canvas.draw();

    // Restore viewport
    glViewport(0, 0, m_width, m_height);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    m_skyboxReady = true;
    std::cout << "PBR: Skybox '" << skyboxName << "' loaded with IBL maps." << std::endl;
}

void PBRPipeline::setDirectionalLight(const DirectionalLight &light)
{
    m_dirLight = light;

    // Set up directional shadow FBO
    m_dirShadowFBO.setupFrameBuffer(m_dirLight.shadowRes, m_dirLight.shadowRes);
}

void PBRPipeline::setPointLights(const std::vector<PointLight> &lights)
{
    m_pointLights = lights;

    // Set up point shadow FBOs
    m_pointShadowFBOs.clear();
    m_pointShadowFBOs.resize(lights.size());
    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        m_pointShadowFBOs[i].setupFrameBuffer(
            lights[i].shadowRes, lights[i].shadowRes);
    }

    // Upload to cluster grid SSBO
    m_clusterGrid.uploadLights(m_pointLights);
}

// ===========================================================================
// Per-frame rendering
// ===========================================================================

void PBRPipeline::beginFrame(const glm::mat4 &view,
                              const glm::mat4 &projection,
                              const glm::vec3 &cameraPos)
{
    m_viewMatrix = view;
    m_projectionMatrix = projection;
    m_cameraPos = cameraPos;
    m_modelDrawCalls.clear();
    m_legacyDrawCalls.clear();
}

void PBRPipeline::submitModel(Model &model, const glm::mat4 &modelMatrix)
{
    m_modelDrawCalls.push_back({&model, modelMatrix});
}

void PBRPipeline::submitLegacyMesh(GLuint vao, GLsizei indexCount,
                                    const glm::mat4 &modelMatrix,
                                    const glm::vec3 &albedo,
                                    float metallic, float roughness)
{
    m_legacyDrawCalls.push_back({vao, indexCount, modelMatrix, albedo, metallic, roughness});
}

// ===========================================================================
// Execute render pass
// ===========================================================================

void PBRPipeline::executeRenderPass()
{
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    shadowPass();
    depthPrePass();
    lightCulling();
    mainShadingPass();

    if (m_skyboxReady)
        renderSkybox();

    // MSAA resolve: blit multisampled → resolve FBO
    m_multisampledFBO.blitTo(m_resolveFBO,
                              m_width, m_height,
                              m_width, m_height,
                              GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

// ---------------------------------------------------------------------------
// Shadow pass
// ---------------------------------------------------------------------------

void PBRPipeline::shadowPass()
{
    // Directional light shadow
    {
        float boxSize = m_dirLight.orthoBoxSize;
        m_dirLight.shadowProjectionMat = glm::ortho(-boxSize, boxSize,
                                                     -boxSize, boxSize,
                                                     m_dirLight.zNear, m_dirLight.zFar);
        m_dirLight.lightView = glm::lookAt(
            m_dirLight.distance * glm::normalize(-m_dirLight.direction),
            glm::vec3(0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f));
        m_dirLight.lightSpaceMatrix = m_dirLight.shadowProjectionMat * m_dirLight.lightView;

        m_dirShadowFBO.bind();
        glClear(GL_DEPTH_BUFFER_BIT);

        m_dirShadowShader.use();

        glm::mat4 VP = m_dirLight.lightSpaceMatrix;

        for (auto &dc : m_modelDrawCalls)
        {
            m_dirShadowShader.setMat4("lightSpaceMatrix", VP * dc.modelMatrix);
            dc.model->draw(m_dirShadowShader, false);
        }
        for (auto &dc : m_legacyDrawCalls)
        {
            m_dirShadowShader.setMat4("lightSpaceMatrix", VP * dc.modelMatrix);
            glBindVertexArray(dc.vao);
            glDrawElements(GL_TRIANGLES, dc.indexCount, GL_UNSIGNED_INT, nullptr);
        }
    }

    // Point light shadows
    for (std::size_t li = 0; li < m_pointLights.size() && li < 4; ++li)
    {
        PointLight &light = m_pointLights[li];

        // Build shadow projection
        float aspect = 1.0f;
        light.shadowProjectionMat = glm::perspective(
            glm::radians(90.0f), aspect, light.zNear, light.zFar);

        // Build lookAt matrices for 6 faces
        glm::vec3 pos = light.position;
        light.lookAtPerFace[0] = glm::lookAt(pos, pos + glm::vec3(1, 0, 0), glm::vec3(0, -1, 0));
        light.lookAtPerFace[1] = glm::lookAt(pos, pos + glm::vec3(-1, 0, 0), glm::vec3(0, -1, 0));
        light.lookAtPerFace[2] = glm::lookAt(pos, pos + glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
        light.lookAtPerFace[3] = glm::lookAt(pos, pos + glm::vec3(0, -1, 0), glm::vec3(0, 0, -1));
        light.lookAtPerFace[4] = glm::lookAt(pos, pos + glm::vec3(0, 0, 1), glm::vec3(0, -1, 0));
        light.lookAtPerFace[5] = glm::lookAt(pos, pos + glm::vec3(0, 0, -1), glm::vec3(0, -1, 0));

        light.depthMapTextureID = m_pointShadowFBOs[li].depthCubemap();

        m_pointShadowFBOs[li].bind();
        glClear(GL_DEPTH_BUFFER_BIT);

        m_pointShadowShader.use();
        m_pointShadowShader.setVec3("lightPos", light.position);
        m_pointShadowShader.setFloat("far_plane", light.zFar);

        for (unsigned int face = 0; face < 6; ++face)
        {
            glm::mat4 lightMatrix = light.shadowProjectionMat * light.lookAtPerFace[face];
            m_pointShadowShader.setMat4(
                ("shadowMatrices[" + std::to_string(face) + "]"),
                lightMatrix);
        }

        for (auto &dc : m_modelDrawCalls)
        {
            m_pointShadowShader.setMat4("M", dc.modelMatrix);
            dc.model->draw(m_pointShadowShader, false);
        }
        for (auto &dc : m_legacyDrawCalls)
        {
            m_pointShadowShader.setMat4("M", dc.modelMatrix);
            glBindVertexArray(dc.vao);
            glDrawElements(GL_TRIANGLES, dc.indexCount, GL_UNSIGNED_INT, nullptr);
        }
    }
}

// ---------------------------------------------------------------------------
// Depth pre-pass
// ---------------------------------------------------------------------------

void PBRPipeline::depthPrePass()
{
    m_multisampledFBO.bind();
    glViewport(0, 0, m_width, m_height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 VP = m_projectionMatrix * m_viewMatrix;

    m_depthPassShader.use();

    for (auto &dc : m_modelDrawCalls)
    {
        glm::mat4 MVP = VP * dc.modelMatrix;
        m_depthPassShader.setMat4("MVP", MVP);
        dc.model->draw(m_depthPassShader, false);
    }

    for (auto &dc : m_legacyDrawCalls)
    {
        glm::mat4 MVP = VP * dc.modelMatrix;
        m_depthPassShader.setMat4("MVP", MVP);
        glBindVertexArray(dc.vao);
        glDrawElements(GL_TRIANGLES, dc.indexCount, GL_UNSIGNED_INT, nullptr);
    }
}

// ---------------------------------------------------------------------------
// Light culling (compute)
// ---------------------------------------------------------------------------

void PBRPipeline::lightCulling()
{
    m_cullLightsShader.use();
    m_cullLightsShader.setMat4("viewMatrix", m_viewMatrix);
    m_cullLightsShader.dispatch(1, 1, 6);
}

// ---------------------------------------------------------------------------
// Main shading pass
// ---------------------------------------------------------------------------

void PBRPipeline::bindPBRUniforms(Shader &shader)
{
    glm::mat4 VP = m_projectionMatrix * m_viewMatrix;

    shader.setVec3("dirLight.direction", m_dirLight.direction);
    shader.setVec3("dirLight.color", m_dirLight.strength * m_dirLight.color);
    shader.setMat4("lightSpaceMatrix", m_dirLight.lightSpaceMatrix);
    shader.setVec3("cameraPos_wS", m_cameraPos);
    shader.setFloat("zFar", m_farPlane);
    shader.setFloat("zNear", m_nearPlane);

    // Bind point light shadow cubemaps (texture units 5..8)
    constexpr unsigned int numMaterialTexUnits = 5;
    for (std::size_t i = 0; i < m_pointLights.size() && i < 4; ++i)
    {
        glActiveTexture(GL_TEXTURE0 + numMaterialTexUnits + static_cast<GLenum>(i));
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_pointLights[i].depthMapTextureID);
        shader.setInt("depthMaps[" + std::to_string(i) + "]",
                      static_cast<int>(numMaterialTexUnits + i));
        shader.setFloat("far_plane", m_pointLights[i].zFar);
    }

    // Directional shadow map
    unsigned int shadowUnit = numMaterialTexUnits + static_cast<unsigned int>(
                                  std::min(m_pointLights.size(), std::size_t(4)));
    glActiveTexture(GL_TEXTURE0 + shadowUnit);
    glBindTexture(GL_TEXTURE_2D, m_dirShadowFBO.depthTexture());
    shader.setInt("shadowMap", static_cast<int>(shadowUnit));

    // IBL textures
    if (m_skyboxReady)
    {
        unsigned int iblBase = shadowUnit + 1;
        glActiveTexture(GL_TEXTURE0 + iblBase);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_irradianceMap.textureID());
        shader.setInt("irradianceMap", static_cast<int>(iblBase));

        glActiveTexture(GL_TEXTURE0 + iblBase + 1);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_specFilteredMap.textureID());
        shader.setInt("prefilterMap", static_cast<int>(iblBase + 1));

        glActiveTexture(GL_TEXTURE0 + iblBase + 2);
        glBindTexture(GL_TEXTURE_2D, m_brdfLUTTexture);
        shader.setInt("brdfLUT", static_cast<int>(iblBase + 2));
    }
}

void PBRPipeline::mainShadingPass()
{
    // Draw into multisampled FBO with depth <= (use depth prepass z-buffer)
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);

    glm::mat4 VP = m_projectionMatrix * m_viewMatrix;

    // --- Textured PBR models ---
    if (!m_modelDrawCalls.empty())
    {
        m_pbrShader.use();
        bindPBRUniforms(m_pbrShader);
        m_pbrShader.setBool("slices", false);

        for (auto &dc : m_modelDrawCalls)
        {
            glm::mat4 MVP = VP * dc.modelMatrix;
            m_pbrShader.setMat4("MVP", MVP);
            m_pbrShader.setMat4("M", dc.modelMatrix);
            m_pbrShader.setBool("IBL", dc.model->isIBL());
            dc.model->draw(m_pbrShader, true);
        }
    }

    // --- Legacy untextured meshes ---
    if (!m_legacyDrawCalls.empty())
    {
        m_pbrSimpleShader.use();
        bindPBRUniforms(m_pbrSimpleShader);
        m_pbrSimpleShader.setBool("IBL", m_skyboxReady);

        for (auto &dc : m_legacyDrawCalls)
        {
            glm::mat4 MVP = VP * dc.modelMatrix;
            m_pbrSimpleShader.setMat4("MVP", MVP);
            m_pbrSimpleShader.setMat4("M", dc.modelMatrix);
            m_pbrSimpleShader.setVec3("u_albedo", dc.albedo);
            m_pbrSimpleShader.setFloat("u_metallic", dc.metallic);
            m_pbrSimpleShader.setFloat("u_roughness", dc.roughness);

            glBindVertexArray(dc.vao);
            glDrawElements(GL_TRIANGLES, dc.indexCount, GL_UNSIGNED_INT, nullptr);
        }
    }

    glBindVertexArray(0);

    // Restore defaults
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
}

// ---------------------------------------------------------------------------
// Skybox
// ---------------------------------------------------------------------------

void PBRPipeline::renderSkybox()
{
    glm::mat4 viewNoTranslation = glm::mat4(glm::mat3(m_viewMatrix));
    glm::mat4 VPCubeMap = m_projectionMatrix * viewNoTranslation;
    m_skybox.draw(m_skyboxShader, VPCubeMap);
}

// ===========================================================================
// Resolved FBO access (for particle compositing)
// ===========================================================================

GLuint PBRPipeline::getResolvedColorTexture() const
{
    return m_resolveFBO.colorTexture();
}

GLuint PBRPipeline::getResolvedDepthTexture() const
{
    return m_resolveFBO.depthTexture();
}

void PBRPipeline::bindResolvedFBO()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_resolveFBO.fbo());
    glViewport(0, 0, m_width, m_height);
}

// ===========================================================================
// Post-processing: bloom + tone mapping
// ===========================================================================

void PBRPipeline::postProcess()
{
    glDisable(GL_DEPTH_TEST);

    // High-pass filter on resolved color
    m_pingPongFBO.bind();
    glClear(GL_COLOR_BUFFER_BIT);

    if (m_bloomPasses > 0)
    {
        m_highPassShader.use();
        m_canvas.draw(m_resolveFBO.colorTexture());
    }

    // Gaussian blur ping-pong between pingPong and simpleFBO's bloom attachment
    m_blurShader.use();
    for (int i = 0; i < m_bloomPasses; ++i)
    {
        // Horizontal pass → simpleFBO bloom attachment
        glBindFramebuffer(GL_FRAMEBUFFER, m_simpleFBO.fbo());
        glViewport(0, 0, m_width, m_height);
        m_blurShader.setBool("horizontal", true);
        m_canvas.draw(m_pingPongFBO.colorTexture());

        // Vertical pass → pingPong
        m_pingPongFBO.bind();
        m_blurShader.setBool("horizontal", false);
        m_canvas.draw(m_simpleFBO.colorTexture());
    }

    // Final compositing: tone mapping + bloom merge → default framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_width, m_height);

    m_screenShader.use();
    m_screenShader.setFloat("exposure", m_exposure);
    m_screenShader.setInt("screenTexture", 0);
    m_screenShader.setInt("bloomBlur", 1);
    m_canvas.draw(m_resolveFBO.colorTexture(), m_pingPongFBO.colorTexture());

    glEnable(GL_DEPTH_TEST);
}

} // namespace pbr
