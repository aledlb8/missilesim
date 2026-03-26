#include "SceneEffects.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

namespace
{
    constexpr std::size_t kMaxParticles = 8192;
    constexpr std::size_t kMaxHeatHazeSprites = 1024;

    glm::vec3 safeNormalize(const glm::vec3 &value, const glm::vec3 &fallback)
    {
        if (glm::length2(value) > 0.0001f)
        {
            return glm::normalize(value);
        }

        if (glm::length2(fallback) > 0.0001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 perpendicularTo(const glm::vec3 &direction)
    {
        const glm::vec3 upReference = (std::abs(direction.y) < 0.95f)
                                          ? glm::vec3(0.0f, 1.0f, 0.0f)
                                          : glm::vec3(1.0f, 0.0f, 0.0f);
        return safeNormalize(glm::cross(direction, upReference), glm::vec3(1.0f, 0.0f, 0.0f));
    }

    float saturate(float value)
    {
        return glm::clamp(value, 0.0f, 1.0f);
    }

    GLuint compileShader(GLenum type, const char *source, const char *label)
    {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint success = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (success == GL_TRUE)
        {
            return shader;
        }

        char infoLog[1024] = {};
        glGetShaderInfoLog(shader, static_cast<GLsizei>(sizeof(infoLog)), nullptr, infoLog);
        std::cerr << "ERROR: " << label << " shader compilation failed\n"
                  << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }

    GLuint linkProgram(GLuint vertexShader, GLuint fragmentShader, const char *label)
    {
        if (vertexShader == 0 || fragmentShader == 0)
        {
            return 0;
        }

        const GLuint program = glCreateProgram();
        glAttachShader(program, vertexShader);
        glAttachShader(program, fragmentShader);
        glLinkProgram(program);

        GLint success = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (success == GL_TRUE)
        {
            return program;
        }

        char infoLog[1024] = {};
        glGetProgramInfoLog(program, static_cast<GLsizei>(sizeof(infoLog)), nullptr, infoLog);
        std::cerr << "ERROR: " << label << " program link failed\n"
                  << infoLog << std::endl;
        glDeleteProgram(program);
        return 0;
    }

    const char *particleVertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec2 aCorner;
        layout (location = 1) in vec4 iCenterRotation;
        layout (location = 2) in vec4 iAxisSizeX;
        layout (location = 3) in vec4 iColor;
        layout (location = 4) in vec4 iParams0;
        layout (location = 5) in vec4 iParams1;

        uniform mat4 view;
        uniform mat4 projection;
        uniform vec3 cameraPos;

        out vec2 vLocalUv;
        out vec4 vColor;
        out vec4 vParams0;
        flat out vec2 vParams1;

        void main()
        {
            vec3 center = iCenterRotation.xyz;
            float rotation = iCenterRotation.w;
            vec3 axis = iAxisSizeX.xyz;
            float halfWidth = iAxisSizeX.w;
            float halfHeight = iParams0.x;

            vec3 viewDirection = normalize(cameraPos - center);
            vec3 projectedAxis = axis - viewDirection * dot(axis, viewDirection);
            if (dot(projectedAxis, projectedAxis) < 1e-5)
            {
                projectedAxis = cross(viewDirection, vec3(0.0, 1.0, 0.0));
                if (dot(projectedAxis, projectedAxis) < 1e-5)
                {
                    projectedAxis = cross(viewDirection, vec3(1.0, 0.0, 0.0));
                }
            }

            vec3 tangent = normalize(projectedAxis);
            vec3 bitangent = normalize(cross(viewDirection, tangent));

            float sineValue = sin(rotation);
            float cosineValue = cos(rotation);
            vec2 rotatedCorner = vec2(
                (aCorner.x * cosineValue) - (aCorner.y * sineValue),
                (aCorner.x * sineValue) + (aCorner.y * cosineValue));

            vec3 worldOffset = (bitangent * rotatedCorner.x * halfWidth) +
                               (tangent * rotatedCorner.y * halfHeight);
            vec4 worldPosition = vec4(center + worldOffset, 1.0);

            gl_Position = projection * view * worldPosition;
            vLocalUv = rotatedCorner;
            vColor = iColor;
            vParams0 = iParams0;
            vParams1 = iParams1.xy;
        }
    )";

    const char *particleFragmentShaderSource = R"(
        #version 330 core
        in vec2 vLocalUv;
        in vec4 vColor;
        in vec4 vParams0;
        flat in vec2 vParams1;

        out vec4 FragColor;

        float hash12(vec2 value)
        {
            vec3 p3 = fract(vec3(value.xyx) * 0.1031);
            p3 += dot(p3, p3.yzx + 33.33);
            return fract((p3.x + p3.y) * p3.z);
        }

        float noise(vec2 value)
        {
            vec2 i = floor(value);
            vec2 f = fract(value);
            float a = hash12(i);
            float b = hash12(i + vec2(1.0, 0.0));
            float c = hash12(i + vec2(0.0, 1.0));
            float d = hash12(i + vec2(1.0, 1.0));
            vec2 u = f * f * (3.0 - 2.0 * f);
            return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
        }

        void main()
        {
            float ageNorm = clamp(vParams0.y, 0.0, 1.0);
            float softness = clamp(vParams0.z, 0.05, 2.0);
            float emissive = max(vParams0.w, 0.0);
            float material = vParams1.x;
            float seed = vParams1.y;

            vec2 uv = vLocalUv;
            float radial = length(uv);
            float alpha = 0.0;
            vec3 color = vColor.rgb;

            if (material < 0.5)
            {
                float flicker = noise(vec2(uv.x * 4.6 + seed * 3.2, uv.y * 2.8 - ageNorm * 2.4));
                float body = exp(-4.0 * abs(uv.x));
                float tail = smoothstep(1.15, -0.30, uv.y);
                float core = exp(-18.0 * uv.x * uv.x) * smoothstep(0.8, -0.1, uv.y);
                alpha = body * tail * mix(0.84, 1.02, flicker) * pow(1.0 - ageNorm, 1.55);
                color = mix(vColor.rgb * 0.55, vec3(1.0, 0.96, 0.88), core * 0.75) * (0.9 + core * 0.45);
            }
            else if (material < 1.5)
            {
                float puff = smoothstep(1.12, 0.1, radial * mix(0.96, 1.06, noise((uv * 2.2) + seed * 2.7)));
                alpha = puff * pow(1.0 - ageNorm, 1.15) * (1.0 - ageNorm * 0.35);
                color = mix(vColor.rgb * 0.72, vec3(0.92), 0.14);
            }
            else if (material < 2.5)
            {
                float streak = exp(-20.0 * uv.x * uv.x) * exp(-2.8 * max(uv.y + 0.12, 0.0));
                float tip = smoothstep(1.08, 0.0, uv.y);
                alpha = streak * tip * pow(1.0 - ageNorm, 2.0);
                color = mix(vColor.rgb, vec3(1.0, 0.98, 0.82), 0.35);
            }
            else
            {
                float glow = smoothstep(1.06, 0.0, radial);
                alpha = glow * pow(1.0 - ageNorm, 1.8);
                color = mix(vColor.rgb, vec3(1.0, 0.98, 0.88), 0.25);
            }

            alpha = clamp(alpha * softness, 0.0, 1.0);
            vec3 premultiplied = color * alpha * max(emissive, 0.0);
            FragColor = vec4(premultiplied, alpha);
        }
    )";

    const char *hazeVertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec2 aCorner;
        layout (location = 1) in vec4 iCenterRotation;
        layout (location = 2) in vec4 iAxisSizeX;
        layout (location = 3) in vec4 iParams0;

        uniform mat4 view;
        uniform mat4 projection;
        uniform vec3 cameraPos;

        out vec2 vLocalUv;
        out vec2 vScreenUv;
        out vec3 vParams;

        void main()
        {
            vec3 center = iCenterRotation.xyz;
            float rotation = iCenterRotation.w;
            vec3 axis = iAxisSizeX.xyz;
            float halfWidth = iAxisSizeX.w;
            float halfHeight = iParams0.x;

            vec3 viewDirection = normalize(cameraPos - center);
            vec3 projectedAxis = axis - viewDirection * dot(axis, viewDirection);
            if (dot(projectedAxis, projectedAxis) < 1e-5)
            {
                projectedAxis = cross(viewDirection, vec3(0.0, 1.0, 0.0));
                if (dot(projectedAxis, projectedAxis) < 1e-5)
                {
                    projectedAxis = cross(viewDirection, vec3(1.0, 0.0, 0.0));
                }
            }

            vec3 tangent = normalize(projectedAxis);
            vec3 bitangent = normalize(cross(viewDirection, tangent));

            float sineValue = sin(rotation);
            float cosineValue = cos(rotation);
            vec2 rotatedCorner = vec2(
                (aCorner.x * cosineValue) - (aCorner.y * sineValue),
                (aCorner.x * sineValue) + (aCorner.y * cosineValue));

            vec3 worldOffset = (bitangent * rotatedCorner.x * halfWidth) +
                               (tangent * rotatedCorner.y * halfHeight);
            vec4 clipPosition = projection * view * vec4(center + worldOffset, 1.0);

            gl_Position = clipPosition;
            vLocalUv = rotatedCorner;
            vScreenUv = (clipPosition.xy / clipPosition.w) * 0.5 + 0.5;
            vParams = vec3(iParams0.y, iParams0.z, iParams0.w);
        }
    )";

    const char *hazeFragmentShaderSource = R"(
        #version 330 core
        in vec2 vLocalUv;
        in vec2 vScreenUv;
        in vec3 vParams;

        uniform sampler2D sceneColor;
        uniform sampler2D sceneDepth;
        uniform vec2 viewportSize;

        out vec4 FragColor;

        float hash12(vec2 value)
        {
            vec3 p3 = fract(vec3(value.xyx) * 0.1031);
            p3 += dot(p3, p3.yzx + 33.33);
            return fract((p3.x + p3.y) * p3.z);
        }

        float noise(vec2 value)
        {
            vec2 i = floor(value);
            vec2 f = fract(value);
            float a = hash12(i);
            float b = hash12(i + vec2(1.0, 0.0));
            float c = hash12(i + vec2(0.0, 1.0));
            float d = hash12(i + vec2(1.0, 1.0));
            vec2 u = f * f * (3.0 - 2.0 * f);
            return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
        }

        void main()
        {
            float ageNorm = clamp(vParams.x, 0.0, 1.0);
            float strength = max(vParams.y, 0.0);
            float seed = vParams.z;

            float radial = length(vLocalUv);
            float edgeMask = smoothstep(1.05, 0.02, radial);
            if (edgeMask <= 0.001)
            {
                discard;
            }

            float depthAtPixel = texture(sceneDepth, vScreenUv).r;
            float fragmentDepth = gl_FragCoord.z;
            if (fragmentDepth > depthAtPixel + 0.00035)
            {
                discard;
            }

            float depthFade = clamp(((depthAtPixel - fragmentDepth) * 1800.0) + 0.35, 0.0, 1.0);
            float axialMask = smoothstep(1.18, -0.22, vLocalUv.y);
            float lifeFade = pow(1.0 - ageNorm, 1.18);
            float distortionMask = edgeMask * mix(0.65, 1.0, axialMask) * lifeFade * depthFade;

            vec2 radialDirection = (radial > 0.001) ? (vLocalUv / radial) : vec2(0.0, 1.0);
            vec2 flowUv0 = (vLocalUv * vec2(4.8, 8.4)) + vec2(seed * 5.3, seed * 7.1) + vec2(ageNorm * 2.8, -ageNorm * 4.6);
            vec2 flowUv1 = (flowUv0 * 1.85) + vec2(11.7, -7.9);
            vec2 flowUv2 = (flowUv0 * 3.25) + vec2(-5.3, 13.4);

            float macroNoise = noise(flowUv0);
            float detailNoise = noise(flowUv1);
            float filamentNoise = noise(flowUv2);
            vec2 offsetDirection = vec2(
                (macroNoise + filamentNoise) - 1.0,
                (detailNoise + filamentNoise) - 1.0);

            float pulse = 0.7 + (0.3 * sin((ageNorm * 11.0) + (seed * 6.28318) + (macroNoise * 4.0)));
            offsetDirection += radialDirection * (0.14 + axialMask * 0.22);

            vec2 distortion = offsetDirection * strength * distortionMask * pulse / max(viewportSize, vec2(1.0));
            vec2 primaryUv = clamp(vScreenUv + distortion, vec2(0.001), vec2(0.999));
            vec2 secondaryUv = clamp(vScreenUv + (distortion * 0.55), vec2(0.001), vec2(0.999));
            vec2 tertiaryUv = clamp(vScreenUv - (distortion * 0.45), vec2(0.001), vec2(0.999));

            vec3 baseColor = texture(sceneColor, clamp(vScreenUv + (distortion * 0.18), vec2(0.001), vec2(0.999))).rgb;
            vec3 refracted = vec3(
                texture(sceneColor, primaryUv).r,
                texture(sceneColor, secondaryUv).g,
                texture(sceneColor, tertiaryUv).b);
            refracted = mix(baseColor, refracted, 0.78);

            float alpha = clamp((0.14 + (strength * 0.035)) * distortionMask, 0.0, 0.55);
            FragColor = vec4(refracted, alpha);
        }
    )";

    const char *compositeVertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec2 aPosition;
        layout (location = 1) in vec2 aTexCoord;

        out vec2 vTexCoord;

        void main()
        {
            vTexCoord = aTexCoord;
            gl_Position = vec4(aPosition, 0.0, 1.0);
        }
    )";

    const char *compositeFragmentShaderSource = R"(
        #version 330 core
        in vec2 vTexCoord;

        uniform sampler2D sceneColor;

        out vec4 FragColor;

        void main()
        {
            FragColor = texture(sceneColor, vTexCoord);
        }
    )";
} // namespace

SceneEffects::SceneEffects()
{
    std::random_device randomDevice;
    m_rng = std::mt19937(randomDevice());
}

SceneEffects::~SceneEffects()
{
    shutdown();
}

void SceneEffects::initialize()
{
    if (m_initialized)
    {
        return;
    }

    createShaders();
    createBuffers();
    ensureSceneFramebuffer();
    m_initialized = true;
}

void SceneEffects::shutdown()
{
    destroySceneFramebuffer();
    destroyBuffers();

    if (m_particleProgram != 0)
    {
        glDeleteProgram(m_particleProgram);
        m_particleProgram = 0;
    }
    if (m_hazeProgram != 0)
    {
        glDeleteProgram(m_hazeProgram);
        m_hazeProgram = 0;
    }
    if (m_compositeProgram != 0)
    {
        glDeleteProgram(m_compositeProgram);
        m_compositeProgram = 0;
    }

    m_particles.clear();
    m_heatHazeSprites.clear();
    m_initialized = false;
}

void SceneEffects::setViewportSize(int width, int height)
{
    const int clampedWidth = std::max(width, 1);
    const int clampedHeight = std::max(height, 1);
    if (m_viewportWidth == clampedWidth && m_viewportHeight == clampedHeight)
    {
        return;
    }

    m_viewportWidth = clampedWidth;
    m_viewportHeight = clampedHeight;
    destroySceneFramebuffer();
}

void SceneEffects::setCamera(const glm::vec3 &cameraPosition, const glm::mat4 &view, const glm::mat4 &projection)
{
    m_cameraPosition = cameraPosition;
    m_view = view;
    m_projection = projection;
}

void SceneEffects::beginScene(const glm::vec3 &clearColor)
{
    if (!m_initialized)
    {
        initialize();
    }

    ensureSceneFramebuffer();
    if (m_sceneFramebufferValid)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_sceneFramebuffer);
    }
    else
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    glViewport(0, 0, m_viewportWidth, m_viewportHeight);
    glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void SceneEffects::renderParticlesToScene()
{
    if (!m_initialized || m_particleProgram == 0 || m_particles.empty())
    {
        return;
    }

    std::vector<ParticleInstance> alphaInstances;
    std::vector<ParticleInstance> additiveInstances;
    alphaInstances.reserve(m_particles.size());
    additiveInstances.reserve(m_particles.size());

    struct SortableParticle
    {
        float distanceSquared = 0.0f;
        ParticleInstance instance{};
    };
    std::vector<SortableParticle> sortableAlpha;
    sortableAlpha.reserve(m_particles.size());

    for (const EffectParticle &particle : m_particles)
    {
        const float ageNorm = (particle.lifetime > 0.0f) ? saturate(particle.age / particle.lifetime) : 1.0f;
        if (ageNorm >= 1.0f)
        {
            continue;
        }

        const float size = glm::mix(particle.startSize, particle.endSize, ageNorm);
        if (!std::isfinite(size) || size <= 0.0001f)
        {
            continue;
        }

        ParticleInstance instance{};
        instance.centerRotation = glm::vec4(particle.position, particle.rotation);
        instance.axisSizeX = glm::vec4(safeNormalize(particle.axis, particle.velocity), size);
        instance.color = particle.color;
        instance.params0 = glm::vec4(size * particle.stretch,
                                     ageNorm,
                                     particle.softness,
                                     particle.emissive);
        instance.params1 = glm::vec4(static_cast<float>(particle.material), particle.seed, 0.0f, 0.0f);

        if (particle.blendMode == BlendMode::ALPHA)
        {
            sortableAlpha.push_back({glm::length2(particle.position - m_cameraPosition), instance});
        }
        else
        {
            additiveInstances.push_back(instance);
        }
    }

    std::sort(sortableAlpha.begin(), sortableAlpha.end(), [](const SortableParticle &lhs, const SortableParticle &rhs)
              { return lhs.distanceSquared > rhs.distanceSquared; });
    alphaInstances.reserve(sortableAlpha.size());
    for (const SortableParticle &entry : sortableAlpha)
    {
        alphaInstances.push_back(entry.instance);
    }

    renderParticlePass(alphaInstances, BlendMode::ALPHA);
    renderParticlePass(additiveInstances, BlendMode::ADDITIVE);
}

void SceneEffects::presentScene()
{
    if (!m_initialized)
    {
        return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportWidth, m_viewportHeight);

    if (m_sceneFramebufferValid && m_compositeProgram != 0)
    {
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        glUseProgram(m_compositeProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
        const GLint sceneSamplerLoc = glGetUniformLocation(m_compositeProgram, "sceneColor");
        if (sceneSamplerLoc != -1)
        {
            glUniform1i(sceneSamplerLoc, 0);
        }

        glBindVertexArray(m_fullscreenVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);

        renderHeatHazePass();

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }
}

void SceneEffects::update(float deltaTime)
{
    if (!m_initialized)
    {
        return;
    }

    const float dt = glm::clamp(deltaTime, 0.0f, 0.05f);
    if (dt <= 0.0f)
    {
        return;
    }

    for (EffectParticle &particle : m_particles)
    {
        particle.age += dt;
        particle.rotation += particle.angularVelocity * dt;
        particle.velocity += glm::vec3(0.0f, particle.upwardAcceleration, 0.0f) * dt;
        particle.velocity *= (1.0f / (1.0f + (particle.drag * dt)));
        particle.position += particle.velocity * dt;
        particle.axis = safeNormalize(glm::mix(particle.axis, particle.velocity, glm::clamp(dt * 4.0f, 0.0f, 1.0f)),
                                      particle.axis);
    }

    m_particles.erase(std::remove_if(m_particles.begin(), m_particles.end(),
                                     [](const EffectParticle &particle)
                                     {
                                         return particle.age >= particle.lifetime;
                                     }),
                      m_particles.end());

    for (HeatHazeSprite &sprite : m_heatHazeSprites)
    {
        sprite.age += dt;
        sprite.rotation += sprite.angularVelocity * dt;
        sprite.velocity *= (1.0f / (1.0f + (sprite.drag * dt)));
        sprite.position += sprite.velocity * dt;
        sprite.axis = safeNormalize(glm::mix(sprite.axis, sprite.velocity, glm::clamp(dt * 3.0f, 0.0f, 1.0f)),
                                    sprite.axis);
    }

    m_heatHazeSprites.erase(std::remove_if(m_heatHazeSprites.begin(), m_heatHazeSprites.end(),
                                           [](const HeatHazeSprite &sprite)
                                           {
                                               return sprite.age >= sprite.lifetime;
                                           }),
                            m_heatHazeSprites.end());
}

void SceneEffects::clear()
{
    m_particles.clear();
    m_heatHazeSprites.clear();
}

void SceneEffects::emitMissileExhaust(const glm::vec3 &start,
                                      const glm::vec3 &end,
                                      const glm::vec3 &forward,
                                      const glm::vec3 &carrierVelocity,
                                      float intensity)
{
    emitEngineTrail(start, end, forward, carrierVelocity, glm::clamp(intensity, 0.5f, 1.0f), true);
}

void SceneEffects::emitJetAfterburner(const glm::vec3 &start,
                                      const glm::vec3 &end,
                                      const glm::vec3 &forward,
                                      const glm::vec3 &carrierVelocity,
                                      float intensity)
{
    emitEngineTrail(start, end, forward, carrierVelocity, glm::clamp(intensity, 0.35f, 1.1f), false);
}

void SceneEffects::emitFlareEffect(const glm::vec3 &start,
                                   const glm::vec3 &end,
                                   const glm::vec3 &carrierVelocity,
                                   float heatFraction)
{
    const float intensity = glm::clamp(heatFraction, 0.0f, 1.0f);
    if (intensity <= 0.01f)
    {
        return;
    }

    const glm::vec3 direction = safeNormalize(-carrierVelocity, glm::vec3(0.0f, -1.0f, 0.0f));
    const float segmentLength = glm::length(end - start);
    const int sampleCount = std::max(1, static_cast<int>(std::ceil(segmentLength / 1.5f)));

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        const float interpolation = (sampleCount == 1) ? 1.0f : static_cast<float>(sampleIndex) / static_cast<float>(sampleCount - 1);
        const glm::vec3 center = glm::mix(start, end, interpolation);

        EffectParticle glow{};
        glow.position = center + (randomInUnitSphere() * 0.2f);
        glow.velocity = carrierVelocity * 0.15f;
        glow.axis = direction;
        glow.color = glm::vec4(glm::mix(glm::vec3(1.0f, 0.55f, 0.18f), glm::vec3(1.0f, 0.96f, 0.76f), intensity), 1.0f);
        glow.lifetime = randomRange(0.12f, 0.22f);
        glow.startSize = 0.45f;
        glow.endSize = 1.7f + intensity * 1.1f;
        glow.stretch = 1.2f;
        glow.softness = 1.1f;
        glow.emissive = 1.35f;
        glow.seed = randomRange(0.0f, 1000.0f);
        glow.material = ParticleMaterial::GLOW;
        glow.blendMode = BlendMode::ADDITIVE;
        addParticle(glow);

        EffectParticle smoke{};
        smoke.position = center + (direction * 0.25f) + (randomInUnitSphere() * 0.35f);
        smoke.velocity = (carrierVelocity * 0.35f) + (direction * randomRange(6.0f, 14.0f)) + (randomInUnitSphere() * 5.0f);
        smoke.axis = safeNormalize(smoke.velocity, direction);
        smoke.color = glm::vec4(glm::mix(glm::vec3(0.22f, 0.20f, 0.20f), glm::vec3(0.42f, 0.34f, 0.28f), intensity), 0.6f);
        smoke.lifetime = randomRange(0.8f, 1.35f);
        smoke.startSize = 0.35f;
        smoke.endSize = 2.6f;
        smoke.stretch = 1.1f;
        smoke.softness = 0.8f;
        smoke.emissive = 1.0f;
        smoke.seed = randomRange(0.0f, 1000.0f);
        smoke.drag = 0.35f;
        smoke.upwardAcceleration = 2.4f;
        smoke.material = ParticleMaterial::SMOKE;
        smoke.blendMode = BlendMode::ALPHA;
        addParticle(smoke);
    }

    HeatHazeSprite haze{};
    haze.position = end;
    haze.velocity = carrierVelocity * 0.2f;
    haze.axis = direction;
    haze.lifetime = 0.12f + intensity * 0.08f;
    haze.radius = 1.25f + intensity * 1.1f;
    haze.stretch = 1.4f;
    haze.rotation = randomRange(0.0f, 6.28318f);
    haze.angularVelocity = randomRange(-1.8f, 1.8f);
    haze.strength = 3.2f + intensity * 2.1f;
    haze.seed = randomRange(0.0f, 1000.0f);
    haze.drag = 1.2f;
    addHeatHaze(haze);
}

void SceneEffects::spawnExplosion(const glm::vec3 &position, const glm::vec3 &velocityHint, float intensity)
{
    const float clampedIntensity = glm::clamp(intensity, 0.5f, 2.0f);
    const glm::vec3 forwardBias = safeNormalize(velocityHint, glm::vec3(0.0f, 1.0f, 0.0f));
    const int flameCount = std::clamp(static_cast<int>(std::round(34.0f * clampedIntensity)), 28, 72);
    const int smokeCount = std::clamp(static_cast<int>(std::round(26.0f * clampedIntensity)), 22, 56);
    const int sparkCount = std::clamp(static_cast<int>(std::round(22.0f * clampedIntensity)), 18, 52);
    const int hazeCount = std::clamp(static_cast<int>(std::round(8.0f * clampedIntensity)), 6, 16);

    EffectParticle flash{};
    flash.position = position;
    flash.velocity = velocityHint * 0.08f;
    flash.axis = forwardBias;
    flash.color = glm::vec4(1.0f, 0.82f, 0.42f, 1.0f);
    flash.lifetime = 0.18f;
    flash.startSize = 2.8f * clampedIntensity;
    flash.endSize = 9.5f * clampedIntensity;
    flash.stretch = 1.0f;
    flash.softness = 1.25f;
    flash.emissive = 1.8f;
    flash.seed = randomRange(0.0f, 1000.0f);
    flash.material = ParticleMaterial::GLOW;
    flash.blendMode = BlendMode::ADDITIVE;
    addParticle(flash);

    for (int index = 0; index < flameCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();
        direction = safeNormalize(glm::mix(direction, forwardBias, 0.18f), direction);

        EffectParticle flame{};
        flame.position = position + (direction * randomRange(0.2f, 1.1f) * clampedIntensity);
        flame.velocity = (direction * randomRange(18.0f, 48.0f) * clampedIntensity) + (velocityHint * 0.2f);
        flame.axis = direction;
        flame.color = glm::vec4(glm::mix(glm::vec3(1.0f, 0.42f, 0.12f), glm::vec3(1.0f, 0.90f, 0.62f), randomRange(0.15f, 0.65f)), 1.0f);
        flame.lifetime = randomRange(0.32f, 0.55f);
        flame.startSize = randomRange(0.7f, 1.2f) * clampedIntensity;
        flame.endSize = randomRange(3.4f, 5.8f) * clampedIntensity;
        flame.stretch = randomRange(1.8f, 3.6f);
        flame.rotation = randomRange(0.0f, 6.28318f);
        flame.angularVelocity = randomRange(-5.0f, 5.0f);
        flame.softness = 1.0f;
        flame.emissive = randomRange(1.0f, 1.5f);
        flame.seed = randomRange(0.0f, 1000.0f);
        flame.drag = 1.8f;
        flame.material = ParticleMaterial::FLAME;
        flame.blendMode = BlendMode::ADDITIVE;
        addParticle(flame);
    }

    for (int index = 0; index < smokeCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();

        EffectParticle smoke{};
        smoke.position = position + (direction * randomRange(0.5f, 2.0f) * clampedIntensity);
        smoke.velocity = (direction * randomRange(8.0f, 22.0f) * clampedIntensity) + (velocityHint * 0.12f);
        smoke.axis = direction;
        smoke.color = glm::vec4(glm::vec3(randomRange(0.18f, 0.34f)), 0.72f);
        smoke.lifetime = randomRange(1.1f, 2.2f);
        smoke.startSize = randomRange(0.8f, 1.4f) * clampedIntensity;
        smoke.endSize = randomRange(5.0f, 8.5f) * clampedIntensity;
        smoke.stretch = randomRange(1.0f, 1.6f);
        smoke.rotation = randomRange(0.0f, 6.28318f);
        smoke.angularVelocity = randomRange(-1.8f, 1.8f);
        smoke.softness = 0.9f;
        smoke.emissive = 1.0f;
        smoke.seed = randomRange(0.0f, 1000.0f);
        smoke.drag = 0.28f;
        smoke.upwardAcceleration = randomRange(4.0f, 8.0f);
        smoke.material = ParticleMaterial::SMOKE;
        smoke.blendMode = BlendMode::ALPHA;
        addParticle(smoke);
    }

    for (int index = 0; index < sparkCount; ++index)
    {
        glm::vec3 direction = randomUnitVector();

        EffectParticle spark{};
        spark.position = position;
        spark.velocity = (direction * randomRange(35.0f, 75.0f) * clampedIntensity) + (velocityHint * 0.25f);
        spark.axis = direction;
        spark.color = glm::vec4(glm::vec3(1.0f, randomRange(0.72f, 0.92f), 0.38f), 1.0f);
        spark.lifetime = randomRange(0.18f, 0.34f);
        spark.startSize = randomRange(0.16f, 0.24f) * clampedIntensity;
        spark.endSize = randomRange(1.5f, 2.8f) * clampedIntensity;
        spark.stretch = randomRange(3.6f, 6.5f);
        spark.softness = 1.0f;
        spark.emissive = 1.3f;
        spark.seed = randomRange(0.0f, 1000.0f);
        spark.drag = 3.0f;
        spark.material = ParticleMaterial::SPARK;
        spark.blendMode = BlendMode::ADDITIVE;
        addParticle(spark);
    }

    for (int index = 0; index < hazeCount; ++index)
    {
        HeatHazeSprite haze{};
        haze.position = position + (randomInUnitSphere() * 0.8f * clampedIntensity);
        haze.velocity = randomInUnitSphere() * randomRange(4.0f, 11.0f) + (velocityHint * 0.08f);
        haze.axis = safeNormalize(haze.velocity, forwardBias);
        haze.lifetime = randomRange(0.18f, 0.34f);
        haze.radius = randomRange(3.5f, 6.8f) * clampedIntensity;
        haze.stretch = randomRange(1.0f, 1.4f);
        haze.rotation = randomRange(0.0f, 6.28318f);
        haze.angularVelocity = randomRange(-2.4f, 2.4f);
        haze.strength = randomRange(3.5f, 7.0f) * clampedIntensity;
        haze.seed = randomRange(0.0f, 1000.0f);
        haze.drag = 0.8f;
        addHeatHaze(haze);
    }

    for (int index = 0; index < 3; ++index)
    {
        HeatHazeSprite shockwave{};
        shockwave.position = position + (forwardBias * randomRange(0.2f, 1.2f) * clampedIntensity);
        shockwave.velocity = (randomUnitVector() * randomRange(10.0f, 24.0f) * clampedIntensity) + (velocityHint * 0.12f);
        shockwave.axis = safeNormalize(shockwave.velocity, forwardBias);
        shockwave.lifetime = randomRange(0.20f, 0.32f);
        shockwave.radius = randomRange(5.8f, 9.4f) * clampedIntensity;
        shockwave.stretch = randomRange(1.6f, 2.4f);
        shockwave.rotation = randomRange(0.0f, 6.28318f);
        shockwave.angularVelocity = randomRange(-1.2f, 1.2f);
        shockwave.strength = randomRange(5.6f, 8.8f) * clampedIntensity;
        shockwave.seed = randomRange(0.0f, 1000.0f);
        shockwave.drag = 1.1f;
        addHeatHaze(shockwave);
    }
}

void SceneEffects::createShaders()
{
    GLuint particleVertexShader = compileShader(GL_VERTEX_SHADER, particleVertexShaderSource, "particle vertex");
    GLuint particleFragmentShader = compileShader(GL_FRAGMENT_SHADER, particleFragmentShaderSource, "particle fragment");
    m_particleProgram = linkProgram(particleVertexShader, particleFragmentShader, "particle");
    if (particleVertexShader != 0)
    {
        glDeleteShader(particleVertexShader);
    }
    if (particleFragmentShader != 0)
    {
        glDeleteShader(particleFragmentShader);
    }

    GLuint hazeVertexShader = compileShader(GL_VERTEX_SHADER, hazeVertexShaderSource, "haze vertex");
    GLuint hazeFragmentShader = compileShader(GL_FRAGMENT_SHADER, hazeFragmentShaderSource, "haze fragment");
    m_hazeProgram = linkProgram(hazeVertexShader, hazeFragmentShader, "haze");
    if (hazeVertexShader != 0)
    {
        glDeleteShader(hazeVertexShader);
    }
    if (hazeFragmentShader != 0)
    {
        glDeleteShader(hazeFragmentShader);
    }

    GLuint compositeVertexShader = compileShader(GL_VERTEX_SHADER, compositeVertexShaderSource, "composite vertex");
    GLuint compositeFragmentShader = compileShader(GL_FRAGMENT_SHADER, compositeFragmentShaderSource, "composite fragment");
    m_compositeProgram = linkProgram(compositeVertexShader, compositeFragmentShader, "composite");
    if (compositeVertexShader != 0)
    {
        glDeleteShader(compositeVertexShader);
    }
    if (compositeFragmentShader != 0)
    {
        glDeleteShader(compositeFragmentShader);
    }
}

void SceneEffects::createBuffers()
{
    static const std::array<float, 8> quadCorners = {
        -1.0f, -1.0f,
        1.0f, -1.0f,
        -1.0f, 1.0f,
        1.0f, 1.0f};

    static const std::array<float, 16> fullscreenQuad = {
        -1.0f, -1.0f, 0.0f, 0.0f,
        1.0f, -1.0f, 1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f};

    glGenBuffers(1, &m_quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER, quadCorners.size() * sizeof(float), quadCorners.data(), GL_STATIC_DRAW);

    glGenVertexArrays(1, &m_particleVAO);
    glBindVertexArray(m_particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, (void *)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &m_particleInstanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_particleInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(ParticleInstance) * 256, nullptr, GL_DYNAMIC_DRAW);

    const GLsizei particleStride = static_cast<GLsizei>(sizeof(ParticleInstance));
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, centerRotation));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, axisSizeX));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, color));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, params0));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, particleStride, (void *)offsetof(ParticleInstance, params1));
    glEnableVertexAttribArray(5);
    glVertexAttribDivisor(5, 1);

    glGenVertexArrays(1, &m_hazeVAO);
    glBindVertexArray(m_hazeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, (void *)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &m_hazeInstanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(HazeInstance) * 128, nullptr, GL_DYNAMIC_DRAW);

    const GLsizei hazeStride = static_cast<GLsizei>(sizeof(HazeInstance));
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, hazeStride, (void *)offsetof(HazeInstance, centerRotation));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, hazeStride, (void *)offsetof(HazeInstance, axisSizeX));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, hazeStride, (void *)offsetof(HazeInstance, params0));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glGenVertexArrays(1, &m_fullscreenVAO);
    glGenBuffers(1, &m_fullscreenVBO);
    glBindVertexArray(m_fullscreenVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_fullscreenVBO);
    glBufferData(GL_ARRAY_BUFFER, fullscreenQuad.size() * sizeof(float), fullscreenQuad.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void *)(sizeof(float) * 2));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SceneEffects::destroyBuffers()
{
    if (m_fullscreenVAO != 0)
    {
        glDeleteVertexArrays(1, &m_fullscreenVAO);
        m_fullscreenVAO = 0;
    }
    if (m_fullscreenVBO != 0)
    {
        glDeleteBuffers(1, &m_fullscreenVBO);
        m_fullscreenVBO = 0;
    }
    if (m_hazeVAO != 0)
    {
        glDeleteVertexArrays(1, &m_hazeVAO);
        m_hazeVAO = 0;
    }
    if (m_hazeInstanceVBO != 0)
    {
        glDeleteBuffers(1, &m_hazeInstanceVBO);
        m_hazeInstanceVBO = 0;
    }
    if (m_particleVAO != 0)
    {
        glDeleteVertexArrays(1, &m_particleVAO);
        m_particleVAO = 0;
    }
    if (m_particleInstanceVBO != 0)
    {
        glDeleteBuffers(1, &m_particleInstanceVBO);
        m_particleInstanceVBO = 0;
    }
    if (m_quadVBO != 0)
    {
        glDeleteBuffers(1, &m_quadVBO);
        m_quadVBO = 0;
    }
}

void SceneEffects::destroySceneFramebuffer()
{
    if (m_sceneDepthTexture != 0)
    {
        glDeleteTextures(1, &m_sceneDepthTexture);
        m_sceneDepthTexture = 0;
    }
    if (m_sceneColorTexture != 0)
    {
        glDeleteTextures(1, &m_sceneColorTexture);
        m_sceneColorTexture = 0;
    }
    if (m_sceneFramebuffer != 0)
    {
        glDeleteFramebuffers(1, &m_sceneFramebuffer);
        m_sceneFramebuffer = 0;
    }
    m_sceneFramebufferValid = false;
}

void SceneEffects::ensureSceneFramebuffer()
{
    if (m_sceneFramebufferValid || m_viewportWidth <= 0 || m_viewportHeight <= 0)
    {
        return;
    }

    destroySceneFramebuffer();

    glGenFramebuffers(1, &m_sceneFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_sceneFramebuffer);

    glGenTextures(1, &m_sceneColorTexture);
    glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_viewportWidth, m_viewportHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_sceneColorTexture, 0);

    glGenTextures(1, &m_sceneDepthTexture);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_viewportWidth, m_viewportHeight, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_sceneDepthTexture, 0);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);

    const GLenum framebufferStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (framebufferStatus != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "ERROR: Scene framebuffer incomplete: " << framebufferStatus << std::endl;
        destroySceneFramebuffer();
    }
    else
    {
        m_sceneFramebufferValid = true;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SceneEffects::ensureParticleInstanceCapacity(std::size_t instanceCount)
{
    if (instanceCount <= m_particleInstanceCapacity)
    {
        return;
    }

    std::size_t newCapacity = std::max<std::size_t>(m_particleInstanceCapacity, 256);
    while (newCapacity < instanceCount)
    {
        newCapacity *= 2;
    }

    m_particleInstanceCapacity = newCapacity;
    glBindBuffer(GL_ARRAY_BUFFER, m_particleInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, m_particleInstanceCapacity * sizeof(ParticleInstance), nullptr, GL_DYNAMIC_DRAW);
}

void SceneEffects::ensureHazeInstanceCapacity(std::size_t instanceCount)
{
    if (instanceCount <= m_hazeInstanceCapacity)
    {
        return;
    }

    std::size_t newCapacity = std::max<std::size_t>(m_hazeInstanceCapacity, 128);
    while (newCapacity < instanceCount)
    {
        newCapacity *= 2;
    }

    m_hazeInstanceCapacity = newCapacity;
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, m_hazeInstanceCapacity * sizeof(HazeInstance), nullptr, GL_DYNAMIC_DRAW);
}

void SceneEffects::renderParticlePass(const std::vector<ParticleInstance> &instances, BlendMode blendMode)
{
    if (instances.empty())
    {
        return;
    }

    ensureParticleInstanceCapacity(instances.size());

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    if (blendMode == BlendMode::ALPHA)
    {
        glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    }
    else
    {
        glBlendFunc(GL_ONE, GL_ONE);
    }

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glUseProgram(m_particleProgram);
    const GLint viewLoc = glGetUniformLocation(m_particleProgram, "view");
    const GLint projectionLoc = glGetUniformLocation(m_particleProgram, "projection");
    const GLint cameraPosLoc = glGetUniformLocation(m_particleProgram, "cameraPos");
    if (viewLoc != -1)
    {
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_view));
    }
    if (projectionLoc != -1)
    {
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(m_projection));
    }
    if (cameraPosLoc != -1)
    {
        glUniform3fv(cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }

    glBindVertexArray(m_particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_particleInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(ParticleInstance), instances.data());
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(instances.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void SceneEffects::renderHeatHazePass()
{
    if (!m_sceneFramebufferValid || m_hazeProgram == 0 || m_heatHazeSprites.empty())
    {
        return;
    }

    struct SortableHaze
    {
        float distanceSquared = 0.0f;
        HazeInstance instance{};
    };

    std::vector<SortableHaze> sortableInstances;
    sortableInstances.reserve(m_heatHazeSprites.size());

    for (const HeatHazeSprite &sprite : m_heatHazeSprites)
    {
        const float ageNorm = (sprite.lifetime > 0.0f) ? saturate(sprite.age / sprite.lifetime) : 1.0f;
        if (ageNorm >= 1.0f)
        {
            continue;
        }

        HazeInstance instance{};
        instance.centerRotation = glm::vec4(sprite.position, sprite.rotation);
        instance.axisSizeX = glm::vec4(safeNormalize(sprite.axis, sprite.velocity), sprite.radius);
        instance.params0 = glm::vec4(sprite.radius * sprite.stretch, ageNorm, sprite.strength, sprite.seed);
        sortableInstances.push_back({glm::length2(sprite.position - m_cameraPosition), instance});
    }

    if (sortableInstances.empty())
    {
        return;
    }

    std::sort(sortableInstances.begin(), sortableInstances.end(), [](const SortableHaze &lhs, const SortableHaze &rhs)
              { return lhs.distanceSquared > rhs.distanceSquared; });

    std::vector<HazeInstance> instances;
    instances.reserve(sortableInstances.size());
    for (const SortableHaze &entry : sortableInstances)
    {
        instances.push_back(entry.instance);
    }

    if (instances.empty())
    {
        return;
    }

    ensureHazeInstanceCapacity(instances.size());

    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glUseProgram(m_hazeProgram);
    const GLint viewLoc = glGetUniformLocation(m_hazeProgram, "view");
    const GLint projectionLoc = glGetUniformLocation(m_hazeProgram, "projection");
    const GLint cameraPosLoc = glGetUniformLocation(m_hazeProgram, "cameraPos");
    const GLint sceneColorLoc = glGetUniformLocation(m_hazeProgram, "sceneColor");
    const GLint sceneDepthLoc = glGetUniformLocation(m_hazeProgram, "sceneDepth");
    const GLint viewportSizeLoc = glGetUniformLocation(m_hazeProgram, "viewportSize");

    if (viewLoc != -1)
    {
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_view));
    }
    if (projectionLoc != -1)
    {
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(m_projection));
    }
    if (cameraPosLoc != -1)
    {
        glUniform3fv(cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }
    if (sceneColorLoc != -1)
    {
        glUniform1i(sceneColorLoc, 0);
    }
    if (sceneDepthLoc != -1)
    {
        glUniform1i(sceneDepthLoc, 1);
    }
    if (viewportSizeLoc != -1)
    {
        glUniform2f(viewportSizeLoc, static_cast<float>(m_viewportWidth), static_cast<float>(m_viewportHeight));
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);

    glBindVertexArray(m_hazeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_hazeInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(HazeInstance), instances.data());
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(instances.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void SceneEffects::addParticle(const EffectParticle &particle)
{
    if (m_particles.size() >= kMaxParticles)
    {
        return;
    }

    m_particles.push_back(particle);
}

void SceneEffects::addHeatHaze(const HeatHazeSprite &sprite)
{
    if (m_heatHazeSprites.size() >= kMaxHeatHazeSprites)
    {
        return;
    }

    m_heatHazeSprites.push_back(sprite);
}

void SceneEffects::emitEngineTrail(const glm::vec3 &start,
                                   const glm::vec3 &end,
                                   const glm::vec3 &forward,
                                   const glm::vec3 &carrierVelocity,
                                   float intensity,
                                   bool missilePreset)
{
    const glm::vec3 exhaustDirection = safeNormalize(-forward, glm::vec3(0.0f, -1.0f, 0.0f));
    const glm::vec3 lateralDirection = perpendicularTo(exhaustDirection);
    const glm::vec3 verticalDirection = safeNormalize(glm::cross(exhaustDirection, lateralDirection), glm::vec3(0.0f, 1.0f, 0.0f));
    const float spacing = missilePreset ? 1.25f : 1.35f;
    const float segmentLength = glm::length(end - start);
    const int sampleCount = std::max(1, static_cast<int>(std::ceil(segmentLength / spacing)) + 1);

    const glm::vec3 flameColor = missilePreset
                                     ? glm::vec3(1.0f, 0.68f, 0.24f)
                                     : glm::vec3(0.40f, 0.72f, 1.0f);
    const glm::vec3 flameHighlight = missilePreset
                                         ? glm::vec3(1.0f, 0.95f, 0.82f)
                                         : glm::vec3(0.86f, 0.94f, 1.0f);

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        const float interpolation = (sampleCount == 1) ? 1.0f : static_cast<float>(sampleIndex) / static_cast<float>(sampleCount - 1);
        const glm::vec3 center = glm::mix(start, end, interpolation);
        const float jitterSpan = missilePreset ? 0.035f : 0.05f;
        const glm::vec3 nozzleJitter = (lateralDirection * randomRange(-jitterSpan, jitterSpan) * intensity) +
                                       (verticalDirection * randomRange(-jitterSpan, jitterSpan) * intensity);

        EffectParticle core{};
        core.position = center + nozzleJitter;
        core.velocity = (carrierVelocity * 0.18f) + (exhaustDirection * randomRange(18.0f, 34.0f) * intensity);
        core.axis = exhaustDirection;
        core.color = glm::vec4(flameHighlight, 1.0f);
        core.lifetime = missilePreset ? randomRange(0.12f, 0.18f) : randomRange(0.08f, 0.12f);
        core.startSize = missilePreset ? 0.14f : 0.10f;
        core.endSize = missilePreset ? 0.42f * intensity : 0.34f * intensity;
        core.stretch = missilePreset ? 1.8f : 1.5f;
        core.softness = 1.0f;
        core.emissive = missilePreset ? 1.1f : 1.0f;
        core.seed = randomRange(0.0f, 1000.0f);
        core.drag = 3.0f;
        core.material = ParticleMaterial::GLOW;
        core.blendMode = BlendMode::ADDITIVE;
        addParticle(core);

        EffectParticle flame{};
        flame.position = center + nozzleJitter + (exhaustDirection * randomRange(0.12f, 0.26f));
        flame.velocity = (carrierVelocity * 0.25f) + (exhaustDirection * randomRange(24.0f, missilePreset ? 48.0f : 38.0f) * intensity) +
                         (randomInUnitSphere() * (missilePreset ? 2.4f : 2.0f));
        flame.axis = exhaustDirection;
        flame.color = glm::vec4(glm::mix(flameColor, flameHighlight, randomRange(0.08f, 0.22f)), 1.0f);
        flame.lifetime = missilePreset ? randomRange(0.18f, 0.28f) : randomRange(0.14f, 0.22f);
        flame.startSize = missilePreset ? 0.18f : 0.14f;
        flame.endSize = missilePreset ? randomRange(0.9f, 1.5f) * intensity : randomRange(0.7f, 1.1f) * intensity;
        flame.stretch = missilePreset ? randomRange(2.4f, 3.4f) : randomRange(2.1f, 3.0f);
        flame.rotation = randomRange(0.0f, 6.28318f);
        flame.angularVelocity = randomRange(-2.0f, 2.0f);
        flame.softness = 0.92f;
        flame.emissive = missilePreset ? 0.95f : 0.9f;
        flame.seed = randomRange(0.0f, 1000.0f);
        flame.drag = 1.6f;
        flame.material = ParticleMaterial::FLAME;
        flame.blendMode = BlendMode::ADDITIVE;
        addParticle(flame);

        if ((missilePreset && sampleIndex % 3 == 0) || (!missilePreset && sampleIndex % 2 == 0))
        {
            EffectParticle smoke{};
            smoke.position = center + (exhaustDirection * randomRange(0.4f, 0.8f)) + (randomInUnitSphere() * 0.2f);
            smoke.velocity = (carrierVelocity * 0.45f) + (exhaustDirection * randomRange(4.0f, missilePreset ? 10.0f : 7.0f) * intensity) +
                             (randomInUnitSphere() * 2.2f);
            smoke.axis = safeNormalize(smoke.velocity, exhaustDirection);
            smoke.color = missilePreset
                              ? glm::vec4(0.30f, 0.30f, 0.32f, 0.42f)
                              : glm::vec4(0.20f, 0.24f, 0.28f, 0.30f);
            smoke.lifetime = missilePreset ? randomRange(0.9f, 1.35f) : randomRange(0.6f, 0.95f);
            smoke.startSize = missilePreset ? 0.22f : 0.20f;
            smoke.endSize = missilePreset ? randomRange(1.0f, 1.8f) * intensity : randomRange(0.8f, 1.4f) * intensity;
            smoke.stretch = randomRange(1.0f, 1.2f);
            smoke.rotation = randomRange(0.0f, 6.28318f);
            smoke.angularVelocity = randomRange(-0.6f, 0.6f);
            smoke.softness = 0.72f;
            smoke.emissive = 1.0f;
            smoke.seed = randomRange(0.0f, 1000.0f);
            smoke.drag = 0.35f;
            smoke.upwardAcceleration = missilePreset ? 1.8f : 0.6f;
            smoke.material = ParticleMaterial::SMOKE;
            smoke.blendMode = BlendMode::ALPHA;
            addParticle(smoke);
        }
    }

    const int hazeCount = missilePreset ? 3 : 2;
    for (int hazeIndex = 0; hazeIndex < hazeCount; ++hazeIndex)
    {
        const float interpolation = (hazeCount == 1) ? 1.0f : static_cast<float>(hazeIndex) / static_cast<float>(hazeCount - 1);
        HeatHazeSprite haze{};
        haze.position = glm::mix(start, end, interpolation) +
                        (exhaustDirection * glm::mix(0.12f, missilePreset ? 0.42f : 0.24f, interpolation));
        haze.velocity = (carrierVelocity * 0.22f) +
                        (exhaustDirection * glm::mix(missilePreset ? 12.0f : 6.0f, missilePreset ? 6.0f : 2.5f, interpolation));
        haze.axis = exhaustDirection;
        haze.lifetime = missilePreset ? randomRange(0.10f, 0.16f) : randomRange(0.08f, 0.13f);
        haze.radius = (missilePreset ? randomRange(0.85f, 1.35f) : randomRange(0.55f, 0.95f)) * intensity;
        haze.stretch = missilePreset ? randomRange(1.7f, 2.4f) : randomRange(1.35f, 1.9f);
        haze.rotation = randomRange(0.0f, 6.28318f);
        haze.angularVelocity = randomRange(-1.4f, 1.4f);
        haze.strength = (missilePreset ? randomRange(2.4f, 3.8f) : randomRange(1.6f, 2.6f)) * intensity;
        haze.seed = randomRange(0.0f, 1000.0f);
        haze.drag = 1.8f;
        addHeatHaze(haze);
    }
}

float SceneEffects::randomRange(float minimum, float maximum)
{
    std::uniform_real_distribution<float> distribution(minimum, maximum);
    return distribution(m_rng);
}

glm::vec3 SceneEffects::randomInUnitSphere()
{
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        glm::vec3 candidate(randomRange(-1.0f, 1.0f),
                            randomRange(-1.0f, 1.0f),
                            randomRange(-1.0f, 1.0f));
        if (glm::length2(candidate) <= 1.0f)
        {
            return candidate;
        }
    }

    return glm::vec3(0.0f);
}

glm::vec3 SceneEffects::randomUnitVector()
{
    return safeNormalize(randomInUnitSphere(), glm::vec3(0.0f, 1.0f, 0.0f));
}
