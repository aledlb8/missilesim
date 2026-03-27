#include "SceneEffects.h"
#include "SceneEffectsDetail.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

using missilesim::rendering::detail::kMaxHeatHazeSprites;
using missilesim::rendering::detail::kMaxParticles;
using missilesim::rendering::detail::perpendicularTo;
using missilesim::rendering::detail::safeNormalize;
using missilesim::rendering::detail::saturate;

namespace
{
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