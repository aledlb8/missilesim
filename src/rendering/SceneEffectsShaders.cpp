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
        uniform vec3 sunDirection;
        uniform mat4 inverseView;
        uniform mat4 inverseProjection;

        out vec2 vLocalUv;
        out vec4 vColor;
        out vec4 vParams0;
        flat out vec3 vParams1;
        flat out vec3 vLightLocal;
        flat out vec4 vEngineNozzleRadius;
        flat out vec4 vEngineAxisLength;
        out vec3 vWorldRay;

        void main()
        {
            vEngineNozzleRadius = vec4(0.0);
            vEngineAxisLength = vec4(0.0);
            vWorldRay = vec3(0.0);
            if (iParams1.x > 7.5)
            {
                // Project the volume bounds, including end-on and inside views.
                vec3 nozzle = iCenterRotation.xyz;
                vec3 axis = normalize(iAxisSizeX.xyz);
                float radius = iAxisSizeX.w * 2.0;
                vec3 reference = abs(axis.y) < 0.95 ? vec3(0,1,0) : vec3(1,0,0);
                vec3 right = normalize(cross(axis, reference));
                vec3 up = cross(right, axis);
                vec2 lower = vec2(1e6), upper = vec2(-1e6);
                bool crossesEye = false;
                for (int i = 0; i < 8; ++i)
                {
                    vec3 p = nozzle + axis * ((i & 1) == 0 ? 0.0 : iParams0.x)
                           + right * ((i & 2) == 0 ? -radius : radius)
                           + up * ((i & 4) == 0 ? -radius : radius);
                    vec4 clip = projection * view * vec4(p, 1.0);
                    if (clip.w <= 0.001) crossesEye = true;
                    vec2 ndc = clip.xy / max(clip.w, 0.001);
                    lower = min(lower, ndc); upper = max(upper, ndc);
                }
                if (crossesEye) { lower = vec2(-1); upper = vec2(1); }
                vec2 ndc = mix(clamp(lower, -1.0, 1.0), clamp(upper, -1.0, 1.0), aCorner * 0.5 + 0.5);
                gl_Position = vec4(ndc, 0.0, 1.0);
                vec4 ray = inverseProjection * vec4(ndc, -1.0, 1.0);
                vWorldRay = mat3(inverseView) * ray.xyz;
                vEngineNozzleRadius = vec4(nozzle, iAxisSizeX.w);
                vEngineAxisLength = vec4(axis, iParams0.x);
                vParams0 = iParams0; vParams1 = iParams1.xyz;
                vColor = iColor; vLocalUv = aCorner; vLightLocal = vec3(0.0);
                return;
            }
            vec3 center = iCenterRotation.xyz;
            float rotation = iCenterRotation.w;
            vec3 axis = iAxisSizeX.xyz;
            float halfWidth = iAxisSizeX.w;
            float halfHeight = iParams0.x;

            vec3 toCamera = cameraPos - center;
            vec3 viewDirection = dot(toCamera, toCamera) > 1e-8 ? normalize(toCamera) : vec3(0.0, 0.0, 1.0);
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
            vec3 localRight = bitangent * cosineValue + tangent * sineValue;
            vec3 localUp = tangent * cosineValue - bitangent * sineValue;
            vec3 worldOffset = localRight * aCorner.x * halfWidth + localUp * aCorner.y * halfHeight;
            vec4 worldPosition = vec4(center + worldOffset, 1.0);

            gl_Position = projection * view * worldPosition;
            vLocalUv = aCorner;
            vColor = iColor;
            vParams0 = iParams0;
            vParams1 = iParams1.xyz;
            vec3 lightDirection = normalize(-sunDirection);
            vLightLocal = vec3(dot(lightDirection, localRight), dot(lightDirection, localUp), dot(lightDirection, viewDirection));
        }
    )";

    const char *particleFragmentShaderSource = R"(
        #version 330 core
        in vec2 vLocalUv;
        in vec4 vColor;
        in vec4 vParams0;
        flat in vec3 vParams1;
        flat in vec3 vLightLocal;
        uniform vec3 sunRadiance;
        flat in vec4 vEngineNozzleRadius;
        flat in vec4 vEngineAxisLength;
        in vec3 vWorldRay;
        uniform vec3 cameraPos;
        uniform mat4 view;
        uniform mat4 projection;

        uniform sampler2D sceneDepth;
        uniform vec2 viewportSize;
        uniform float zNear;
        uniform float zFar;
        uniform bool depthFadeEnabled;

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

        float linearizeDepth(float depthSample)
        {
            float ndc = 2.0 * depthSample - 1.0;
            return 2.0 * zNear * zFar / (zFar + zNear - ndc * (zFar - zNear));
        }

        float turbulence(vec2 p)
        {
            mat2 turn = mat2(0.8, -0.6, 0.6, 0.8);
            float result = noise(p) * 0.57;
            p = turn * p * 2.03 + vec2(7.1, 3.7);
            result += noise(p) * 0.28;
            p = turn * p * 2.01 + vec2(2.3, 9.2);
            return result + noise(p) * 0.15;
        }

        // Finite-cylinder intersection followed by emission/absorption integration.
        // Geometry depth truncates the ray, so the aircraft can occlude its plume.
        void renderEngineVolume()
        {
            vec3 axis = vEngineAxisLength.xyz;
            float plumeLength = vEngineAxisLength.w;
            float nozzleRadius = vEngineNozzleRadius.w;
            vec3 ray = normalize(vWorldRay);
            vec3 origin = cameraPos - vEngineNozzleRadius.xyz;
            float axialOrigin = dot(origin, axis), axialRay = dot(ray, axis);
            vec3 radialOrigin = origin - axis * axialOrigin;
            vec3 radialRay = ray - axis * axialRay;
            float a = dot(radialRay, radialRay);
            float b = dot(radialOrigin, radialRay);
            float c = dot(radialOrigin, radialOrigin) - 4.0 * nozzleRadius * nozzleRadius;
            float enter = 0.0, leave = 1e8;
            if (a > 1e-8)
            {
                float discriminant = b * b - a * c;
                if (discriminant <= 0.0) discard;
                float root = sqrt(discriminant);
                enter = max(enter, (-b - root) / a);
                leave = min(leave, (-b + root) / a);
            }
            else if (c > 0.0) discard;
            if (abs(axialRay) > 1e-6)
            {
                float t0 = -axialOrigin / axialRay;
                float t1 = (plumeLength - axialOrigin) / axialRay;
                enter = max(enter, min(t0, t1));
                leave = min(leave, max(t0, t1));
            }
            else if (axialOrigin < 0.0 || axialOrigin > plumeLength) discard;
            float viewRayDepth = max(-(view * vec4(ray, 0.0)).z, 1e-5);
            enter = max(enter, zNear / viewRayDepth);
            if (depthFadeEnabled)
                leave = min(leave, linearizeDepth(texture(sceneDepth, gl_FragCoord.xy / viewportSize).r) / viewRayDepth);
            if (leave <= enter) discard;

            bool rocket = vParams1.x > 8.5;
            float power = vParams0.z;
            float clock = vParams0.y;
            vec3 reference = abs(axis.y) < 0.95 ? vec3(0,1,0) : vec3(1,0,0);
            vec3 right = normalize(cross(axis, reference));
            vec3 up = cross(right, axis);
            float stepLength = (leave - enter) / 40.0;
            vec3 radiance = vec3(0.0);
            float transmittance = 1.0;
            for (int i = 0; i < 40; ++i)
            {
                vec3 p = origin + ray * (enter + (float(i) + 0.5) * stepLength);
                float distance = dot(p, axis);
                float t = clamp(distance / plumeLength, 0.0, 1.0);
                vec2 crossSection = vec2(dot(p, right), dot(p, up)) / nozzleRadius;
                float cells = distance / (nozzleRadius * (rocket ? 4.6 : 3.3));
                float cellPhase = 6.2831853 * cells;
                float envelope = rocket ? (0.88 + 0.85 * (1.0 - exp(-t * 9.0))) : 0.91;
                envelope *= 1.0 - pow(t, rocket ? 2.2 : 1.8) * 0.96;
                envelope *= 1.0 + 0.10 * sin(cellPhase);
                // Low-amplitude advected filaments preserve a stable silhouette.
                float flow = sin(crossSection.x * 8.0 + cells * 7.0 - clock * 28.0)
                           * sin(crossSection.y * 7.0 - cells * 4.0 + clock * 21.0);
                float radius = length(crossSection + vec2(sin(cells * 4.0 - clock * 14.0),
                                      cos(cells * 5.0 - clock * 17.0)) * (0.015 + t * 0.04));
                float normalizedRadius = radius / max(envelope, 0.025);
                float body = 1.0 - smoothstep(0.45, 1.02, normalizedRadius);
                float tip = 1.0 - smoothstep(0.65, 1.0, t);
                float cell = pow(max(0.0, cos(cellPhase)), 10.0) * exp(-t * 2.0)
                           * exp(-normalizedRadius * normalizedRadius * 5.0);
                float core = exp(-normalizedRadius * normalizedRadius * 7.0);
                float density = body * tip * (0.42 + core * 0.48 + cell * 0.7);
                density *= (0.92 + flow * 0.08) * (rocket ? 0.9 : mix(0.22, 0.65, power));
                float absorption = 1.0 - exp(-density * stepLength / nozzleRadius * 0.8);
                vec3 edgeColor = rocket ? vec3(2.6, 0.38, 0.035) : vec3(0.18, 0.24, 1.15);
                vec3 coreColor = rocket ? vec3(5.5, 3.3, 1.15) : vec3(4.2, 1.9, 0.55);
                vec3 emission = mix(edgeColor, coreColor, clamp(core * 0.65 + cell * 0.7, 0.0, 1.0));
                emission *= (rocket ? 1.0 : mix(0.35, 1.0, power)) * (1.0 - t * 0.5);
                radiance += transmittance * absorption * emission;
                transmittance *= 1.0 - absorption;
                if (transmittance < 0.015) break;
            }
            vec4 entryClip = projection * view * vec4(cameraPos + ray * enter, 1.0);
            gl_FragDepth = clamp(entryClip.z / entryClip.w * 0.5 + 0.5, 0.0, 1.0);
            FragColor = vec4(radiance, 1.0 - transmittance);
        }

        void main()
        {
            gl_FragDepth = gl_FragCoord.z;
            if (vParams1.x > 7.5)
            {
                renderEngineVolume();
                return;
            }
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
                // Advected filaments with a compact hot core and cooling edges.
                vec2 flow = uv * vec2(3.2, 2.1) + vec2(seed * 0.73, -ageNorm * 3.5);
                float billow = turbulence(flow);
                float bend = (noise(flow * 0.65 + 8.7) - 0.5) * 0.28;
                float width = mix(0.48, 0.10, smoothstep(-0.65, 0.95, uv.y));
                float crossSection = (uv.x + bend) / width;
                float body = exp(-2.5 * crossSection * crossSection);
                float ends = smoothstep(-1.0, -0.72, uv.y) * (1.0 - smoothstep(0.55, 1.0, uv.y));
                float core = exp(-20.0 * (uv.x + bend) * (uv.x + bend)) * (1.0 - smoothstep(-0.4, 0.8, uv.y));
                float erosion = smoothstep(ageNorm * 0.65, ageNorm * 0.65 + 0.32, billow);
                alpha = body * ends * erosion * pow(1.0 - ageNorm, 1.35);
                float heat = clamp(core * 0.8 + billow * 0.35 - ageNorm * 0.45, 0.0, 1.0);
                color = mix(vColor.rgb * vec3(0.95, 0.48, 0.22), vec3(1.0, 0.94, 0.78), heat);
                color *= 1.1 + heat * heat * 2.3;
            }
            else if (material < 1.5)
            {
                // Approximate a lit volume using turbulent optical thickness.
                // Low-frequency lobes carry the silhouette; finer eddies erode it.
                vec2 flow = uv * 2.6 + vec2(seed * 0.31, seed * 0.17 - ageNorm * 0.65);
                vec2 warp = vec2(noise(flow + 4.7), noise(flow - 8.3)) - 0.5;
                vec2 p = flow + warp * 0.85;
                float density = turbulence(p);
                float envelope = 1.0 - smoothstep(0.18, 1.0, radial + (density - 0.5) * 0.25);
                float breakup = smoothstep(0.06 + ageNorm * 0.3, 0.52 + ageNorm * 0.2, density);
                float thickness = envelope * (0.35 + density * 2.8) * breakup;
                float life = smoothstep(0.0, 0.08, ageNorm) * (1.0 - smoothstep(0.35, 1.0, ageNorm));
                alpha = (1.0 - exp(-thickness * 2.2)) * life;

                float coarse = noise(p);
                vec2 slope = vec2(noise(p + vec2(0.12, 0.0)) - coarse,
                                  noise(p + vec2(0.0, 0.12)) - coarse) / 0.12;
                vec3 normal = normalize(vec3(uv - slope * 0.65, sqrt(max(0.08, 1.0 - dot(uv, uv)))));
                float diffuse = clamp(dot(normal, vLightLocal) * 0.55 + 0.45, 0.0, 1.0);
                float transmission = exp(-thickness * 1.4);
                float silver = pow(max(-vLightLocal.z, 0.0), 4.0) * transmission * 0.45;
                vec3 ambient = vec3(0.32, 0.39, 0.48);
                color = vColor.rgb * (ambient + sunRadiance * (diffuse * 0.38 + silver))
                        * mix(0.62, 1.0, transmission);
            }
            else if (material < 2.5)
            {
                // SPARK: short-lived hot streak.
                float streak = exp(-20.0 * uv.x * uv.x) * exp(-2.8 * max(uv.y + 0.12, 0.0));
                float tip = 1.0 - smoothstep(0.0, 1.08, uv.y);
                alpha = streak * tip * pow(1.0 - ageNorm, 2.0);
                color = mix(vColor.rgb, vec3(1.0, 0.98, 0.82), 0.35);
            }
            else if (material < 3.5)
            {
                // GLOW: smooth radial flash.
                float glow = exp(-5.0 * radial * radial) * (1.0 - smoothstep(0.65, 1.0, radial));
                alpha = glow * pow(1.0 - ageNorm, 1.8);
                color = mix(vColor.rgb, vec3(1.0, 0.98, 0.88), 0.25);
            }
            else if (material < 4.5)
            {
                // SHOCKWAVE: thin expanding blast ring; radius sweeps outward
                // with sqrt(age) (fast then decelerating, like a real front).
                float ringRadius = 0.12 + 0.82 * sqrt(ageNorm);
                float ringDist = abs(radial - ringRadius);
                float fade = (1.0 - ageNorm) * (1.0 - ageNorm);
                float width = max(0.012, fwidth(radial) * 1.5);
                alpha = exp(-ringDist * ringDist / (width * width)) * fade * 0.025;
                alpha *= 1.0 - smoothstep(0.92, 1.0, radial);
                color = mix(vColor.rgb, vec3(1.0, 0.92, 0.8), 0.5);
            }
            else if (material < 5.5)
            {
                // DEBRIS: gravity-arcing fragment - glowing head, fading tail.
                float streak = exp(-16.0 * uv.x * uv.x);
                float tail = 1.0 - smoothstep(-0.6, 1.15, uv.y);
                float headDist = (uv.y - 0.55) * (uv.y - 0.55) + uv.x * uv.x * 4.0;
                float head = exp(-9.0 * headDist);
                alpha = streak * tail * 0.55 * pow(1.0 - ageNorm, 1.5) +
                        head * pow(1.0 - ageNorm, 1.1);
                color = mix(vColor.rgb, vec3(1.0, 0.88, 0.62), head * 0.8);
            }
            else if (material < 6.5)
            {
                // SHOCK_DIAMOND: compact supersonic plume pulse. The diamond
                // mask gives afterburners a standing-wave pattern without a mesh.
                float diamond = abs(uv.x) * 1.42 + abs(uv.y) * 0.78;
                float body = 1.0 - smoothstep(0.18, 1.05, diamond);
                float waist = exp(-14.0 * uv.x * uv.x) * (1.0 - smoothstep(-0.15, 1.0, abs(uv.y)));
                float core = exp(-8.0 * (uv.x * uv.x + uv.y * uv.y * 0.55));
                float shimmer = 0.82 + 0.18 * noise(uv * 6.0 + vec2(seed * 2.3, ageNorm * 5.0));
                alpha = (body * 0.74 + waist * 0.32 + core * 0.45) * shimmer * pow(1.0 - ageNorm, 1.25);
                color = mix(vColor.rgb, vec3(1.0, 0.97, 0.86), core * 0.65);
            }
            else
            {
                // FIREBALL: thick rolling lobes cool from yellow-orange to soot.
                // Alpha compositing preserves the internal texture under overlap.
                vec2 p = uv * 2.8 + vec2(seed * 0.19, ageNorm * -1.2);
                vec2 warp = vec2(noise(p + 3.1), noise(p - 7.4)) - 0.5;
                float billow = turbulence(p + warp * 1.1);
                float silhouette = 1.0 - smoothstep(0.25, 0.95, radial + (billow - 0.5) * 0.38);
                float density = silhouette * (0.6 + billow * 2.4);
                alpha = (1.0 - exp(-density * 2.0)) * (1.0 - smoothstep(0.65, 1.0, ageNorm));
                float heat = clamp(billow * 1.4 + (1.0 - radial) * 0.3 - ageNorm * 0.85, 0.0, 1.0);
                vec3 ember = mix(vec3(0.10, 0.055, 0.035), vec3(1.8, 0.23, 0.025), smoothstep(0.08, 0.42, heat));
                color = mix(ember, vec3(3.8, 2.2, 0.75), smoothstep(0.45, 0.95, heat));
                color *= mix(vec3(1.0), vColor.rgb, 0.2);
            }

            // Authored opacity controls wispy wakes and dense blast clouds.
            float edge = 1.0 - smoothstep(0.84, 1.0, max(abs(uv.x), abs(uv.y)));
            alpha = clamp(alpha * softness * vColor.a * edge, 0.0, 1.0);

            if (depthFadeEnabled)
            {
                vec2 screenUv = gl_FragCoord.xy / viewportSize;
                float sceneLinear = linearizeDepth(texture(sceneDepth, screenUv).r);
                float fragLinear = linearizeDepth(gl_FragCoord.z);

                // Soft particles: fade out where the billboard approaches
                // scene geometry. Fade distance scales with particle size so
                // large smoke fades over metres and sparks stay crisp.
                float fadeDistance = clamp(vParams0.x * 0.5, 0.3, 8.0);
                alpha *= clamp((sceneLinear - fragLinear) / fadeDistance, 0.0, 1.0);

                // Fade near the camera so flying through a plume doesn't pop.
                alpha *= clamp((fragLinear - zNear * 2.0) / 1.5, 0.0, 1.0);
            }

            vec3 premultiplied = color * alpha * max(emissive, 0.0);
            FragColor = vec4(premultiplied, alpha * (1.0 - vParams1.z));
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
        out vec3 vParams;
        flat out float vFadeDistance;

        void main()
        {
            vec3 center = iCenterRotation.xyz;
            float rotation = iCenterRotation.w;
            vec3 axis = iAxisSizeX.xyz;
            float halfWidth = iAxisSizeX.w;
            float halfHeight = iParams0.x;

            vec3 toCamera = cameraPos - center;
            vec3 viewDirection = dot(toCamera, toCamera) > 1e-8 ? normalize(toCamera) : vec3(0.0, 0.0, 1.0);
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
            vec3 localRight = bitangent * cosineValue + tangent * sineValue;
            vec3 localUp = tangent * cosineValue - bitangent * sineValue;
            vec3 worldOffset = localRight * aCorner.x * halfWidth + localUp * aCorner.y * halfHeight;
            vec4 clipPosition = projection * view * vec4(center + worldOffset, 1.0);

            gl_Position = clipPosition;
            vLocalUv = aCorner;
            vParams = vec3(iParams0.y, iParams0.z, iParams0.w);
            vFadeDistance = clamp(halfWidth * 0.5, 0.3, 8.0);
        }
    )";

    const char *hazeFragmentShaderSource = R"(
        #version 330 core
        in vec2 vLocalUv;
        in vec3 vParams;
        flat in float vFadeDistance;

        uniform sampler2D sceneColor;
        uniform sampler2D sceneDepth;
        uniform vec2 viewportSize;
        uniform float zNear;
        uniform float zFar;

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

        float linearizeDepth(float depthSample)
        {
            float ndc = 2.0 * depthSample - 1.0;
            return 2.0 * zNear * zFar / (zFar + zNear - ndc * (zFar - zNear));
        }

        void main()
        {
            vec2 screenUv = gl_FragCoord.xy / viewportSize;
            float ageNorm = clamp(vParams.x, 0.0, 1.0);
            float strength = max(vParams.y, 0.0);
            float seed = vParams.z;

            float radial = length(vLocalUv);
            float edgeMask = 1.0 - smoothstep(0.02, 1.0, radial);
            if (edgeMask <= 0.001)
            {
                discard;
            }

            float depthAtPixel = linearizeDepth(texture(sceneDepth, screenUv).r);
            float fragmentDepth = linearizeDepth(gl_FragCoord.z);
            if (fragmentDepth >= depthAtPixel)
            {
                discard;
            }

            float depthFade = smoothstep(0.0, vFadeDistance, depthAtPixel - fragmentDepth);
            depthFade *= smoothstep(zNear, zNear + 1.5, fragmentDepth);
            float axialMask = 1.0 - smoothstep(-0.22, 1.18, vLocalUv.y);
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
            vec2 texelMargin = 0.5 / max(viewportSize, vec2(1.0));
            vec2 refractedUv = clamp(screenUv + distortion, texelMargin, vec2(1.0) - texelMargin);
            // Do not pull foreground geometry into a plume behind it.
            float refractedDepth = linearizeDepth(texture(sceneDepth, refractedUv).r);
            float refractionVisibility = smoothstep(0.0, vFadeDistance, refractedDepth - fragmentDepth);
            refractedUv = mix(screenUv, refractedUv, refractionVisibility);
            vec3 refracted = texture(sceneColor, refractedUv).rgb;

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
