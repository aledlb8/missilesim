#include "PBRPipeline.h"

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
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
    m_clusterGrid.uploadLights(m_pointLights);

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

    if (!ok)
    {
        std::cerr << "PBR: Failed to load shadow shaders!" << std::endl;
        return false;
    }

    // Post-processing shaders
    ok &= m_bloomDownShader.load(shaderDir / "screenShader.vert",
                                  shaderDir / "bloomDownsampleShader.frag");
    ok &= m_bloomUpShader.load(shaderDir / "screenShader.vert",
                                shaderDir / "bloomUpsampleShader.frag");

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
    m_sceneCopyFBO.setupFrameBuffer(m_width, m_height);
    m_bloomChain.setup(m_width, m_height);

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
    m_sceneCopyFBO.setupFrameBuffer(width, height);
    m_bloomChain.setup(width, height);

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
    m_clusterGrid.uploadLights(m_pointLights);

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
                                    float metallic, float roughness,
                                    bool useVertexColor)
{
    m_legacyDrawCalls.push_back({vao, indexCount, modelMatrix, albedo, metallic, roughness, useVertexColor});
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

        // Centre the shadow frustum on the ground beneath the camera so shadows
        // render wherever we are looking (and track the missile in flight),
        // instead of a fixed box pinned to the world origin. Snap the centre to
        // shadow-texel steps to stop the shadow edges crawling as we move.
        const float texelWorld = (2.0f * boxSize) / static_cast<float>(m_dirLight.shadowRes);
        glm::vec3 focus(m_cameraPos.x, 0.0f, m_cameraPos.z);
        focus.x = std::floor(focus.x / texelWorld) * texelWorld;
        focus.z = std::floor(focus.z / texelWorld) * texelWorld;

        m_dirLight.shadowProjectionMat = glm::ortho(-boxSize, boxSize,
                                                     -boxSize, boxSize,
                                                     m_dirLight.zNear, m_dirLight.zFar);
        m_dirLight.lightView = glm::lookAt(
            focus + m_dirLight.distance * glm::normalize(-m_dirLight.direction),
            focus,
            glm::vec3(0.0f, 1.0f, 0.0f));
        m_dirLight.lightSpaceMatrix = m_dirLight.shadowProjectionMat * m_dirLight.lightView;

        m_dirShadowFBO.bind();
        // The shadow map is shadowRes x shadowRes; without this the depth pass
        // renders at the window viewport and only fills a corner of the map,
        // so almost every fragment samples "no shadow".
        glViewport(0, 0, static_cast<GLsizei>(m_dirLight.shadowRes), static_cast<GLsizei>(m_dirLight.shadowRes));
        glClear(GL_DEPTH_BUFFER_BIT);

        // Slope-scaled depth offset during rasterization; paired with the
        // receiver-side normal-offset bias in the shading shaders.
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(2.0f, 4.0f);

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

        glDisable(GL_POLYGON_OFFSET_FILL);
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
        m_depthPassShader.setBool("alphaTest", false);
        dc.model->draw(m_depthPassShader, false);
    }

    for (auto &dc : m_legacyDrawCalls)
    {
        glm::mat4 MVP = VP * dc.modelMatrix;
        m_depthPassShader.setMat4("MVP", MVP);
        m_depthPassShader.setBool("alphaTest", false);
        glBindVertexArray(dc.vao);
        glDrawElements(GL_TRIANGLES, dc.indexCount, GL_UNSIGNED_INT, nullptr);
    }
}

// ---------------------------------------------------------------------------
// Light culling (compute)
// ---------------------------------------------------------------------------

void PBRPipeline::lightCulling()
{
    const unsigned int zero = 0;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_clusterGrid.getLightIndexGlobalCountSSBO());
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(zero), &zero);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

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
    shader.setFloat("shadowTexelWorld",
                    2.0f * m_dirLight.orthoBoxSize /
                        static_cast<float>(m_dirLight.shadowRes));
    shader.setVec3("cameraPos_wS", m_cameraPos);
    shader.setFloat("zFar", m_farPlane);
    shader.setFloat("zNear", m_nearPlane);
    shader.setVec3("fogColor", m_fogColor);
    shader.setFloat("fogDensity", m_fogDensity > 0.0f
                                      ? m_fogDensity
                                      : 1.0f / std::max(m_farPlane * 0.6f, 9000.0f));
    shader.setFloat("fogHeightFalloff", m_fogHeightFalloff);

    // Directional shadow map (unit 5, after the material texture units)
    constexpr unsigned int shadowUnit = 5;
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
            m_pbrSimpleShader.setBool("u_useVertexColor", dc.useVertexColor);

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

    m_skyboxShader.use();
    m_skyboxShader.setVec3("sunDirection", glm::normalize(m_dirLight.direction));
    m_skyboxShader.setVec3("sunColor", m_dirLight.strength * m_dirLight.color);

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

void PBRPipeline::copySceneDepthForEffects()
{
    m_resolveFBO.blitTo(m_sceneCopyFBO, m_width, m_height, m_width, m_height,
                        GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

void PBRPipeline::copySceneColorForDistortion()
{
    m_resolveFBO.blitTo(m_sceneCopyFBO, m_width, m_height, m_width, m_height,
                        GL_COLOR_BUFFER_BIT, GL_NEAREST);
}

GLuint PBRPipeline::sceneCopyColorTexture() const
{
    return m_sceneCopyFBO.colorTexture();
}

GLuint PBRPipeline::sceneCopyDepthTexture() const
{
    return m_sceneCopyFBO.depthTexture();
}

// ===========================================================================
// Post-processing: bloom + tone mapping
// ===========================================================================

void PBRPipeline::postProcess()
{
    glDisable(GL_DEPTH_TEST);

    const int levels = m_bloomPasses <= 0
                           ? 0
                           : std::min(m_bloomPasses, m_bloomChain.mipCount());
    const bool bloomEnabled = levels > 0;

    if (bloomEnabled)
    {
        // Downsample chain: resolve -> mip0 -> ... -> mipN. The first pass
        // applies the soft-knee bright pass and Karis firefly suppression.
        m_bloomDownShader.use();
        GLuint srcTex = m_resolveFBO.colorTexture();
        for (int i = 0; i < levels; ++i)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, m_bloomChain.fbo(i));
            glViewport(0, 0, m_bloomChain.mipWidth(i), m_bloomChain.mipHeight(i));
            m_bloomDownShader.setBool("firstPass", i == 0);
            m_canvas.draw(srcTex);
            srcTex = m_bloomChain.texture(i);
        }

        // Upsample: tent-filter each mip additively into the next larger one,
        // accumulating progressively wider halos on the way back to mip0.
        m_bloomUpShader.use();
        m_bloomUpShader.setFloat("filterRadius", 0.005f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glBlendEquation(GL_FUNC_ADD);
        for (int i = levels - 1; i > 0; --i)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, m_bloomChain.fbo(i - 1));
            glViewport(0, 0, m_bloomChain.mipWidth(i - 1), m_bloomChain.mipHeight(i - 1));
            m_canvas.draw(m_bloomChain.texture(i));
        }
        glDisable(GL_BLEND);
    }

    // Final compositing: bloom merge + tone mapping + sRGB encode → default FB
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_width, m_height);

    m_screenShader.use();
    m_screenShader.setFloat("exposure", m_exposure);
    m_screenShader.setFloat("bloomStrength", bloomEnabled ? m_bloomStrength : 0.0f);
    m_screenShader.setInt("screenTexture", 0);
    m_screenShader.setInt("bloomBlur", 1);
    m_canvas.draw(m_resolveFBO.colorTexture(),
                  bloomEnabled ? m_bloomChain.texture(0) : m_resolveFBO.colorTexture());

    glEnable(GL_DEPTH_TEST);
}

} // namespace pbr
