#version 430 core

out vec4 FragColor;

in VS_OUT {
    vec3 fragPos_wS;
    vec4 fragPos_lS;
    vec3 N;
    vec3 vertexColor;
    vec2 metalRoughness;
} fs_in;

// Directional light
struct DirLight {
    vec3 direction;
    vec3 color;
};
uniform DirLight dirLight;

// Material uniforms (instead of textures)
uniform vec3  u_albedo;
uniform float u_metallic;
uniform float u_roughness;
uniform bool  u_useVertexColor;
uniform bool  u_useVertexMaterial;

// Shadow map
uniform sampler2D shadowMap;
uniform mat4 lightSpaceMatrix;
uniform float shadowTexelWorld;  // world-space size of one shadow texel

// IBL textures
uniform samplerCube irradianceMap;
uniform samplerCube prefilterMap;
uniform sampler2D brdfLUT;
uniform bool IBL;

uniform vec3 cameraPos_wS;
uniform vec3 fogColor;
uniform float fogDensity;
uniform float fogHeightFalloff;

#define M_PI 3.1415926535897932384626433832795

// Clustered shading structures
struct PointLight {
    vec4 position;
    vec4 color;
    bool enabled;
    float intensity;
    float range;
};
struct LightGrid {
    uint offset;
    uint count;
};
layout (std430, binding = 2) buffer screenToView {
    mat4 inverseProjection;
    uvec4 tileSizes;
    uvec2 screenDimensions;
    float scale;
    float bias;
};
layout (std430, binding = 3) buffer lightSSBO {
    PointLight pointLight[];
};
layout (std430, binding = 4) buffer lightIndexSSBO {
    uint globalLightIndexList[];
};
layout (std430, binding = 5) buffer lightGridSSBO {
    LightGrid lightGrid[];
};

uniform float zFar;
uniform float zNear;

// Surface treatment: 0 authored mesh, 1 terrain, 2 water.
uniform int u_surface;
uniform float u_time;
// Terrain and water share the heightfield: the land height under any point.
uniform sampler2D u_heightmap;
uniform bool u_hasHeightmap;
uniform vec2 u_heightmapOrigin; // world x, z of sample (0, 0)
uniform float u_heightmapSize;  // world span of the grid
uniform float u_outerBed;       // land height past the grid
uniform bool u_hasWater;
uniform float u_waterLevel;
uniform float u_snowLine;       // metres above the water/base where snow settles

#include "sky.glsl"

// Value noise for procedural ground detail
float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float valueNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    float a = hash12(i);
    float b = hash12(i + vec2(1.0, 0.0));
    float c = hash12(i + vec2(0.0, 1.0));
    float d = hash12(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float fbm4(vec2 p) {
    return valueNoise(p) * 0.5 + valueNoise(p * 2.03 + 7.1) * 0.25 +
           valueNoise(p * 4.01 - 3.7) * 0.125 + valueNoise(p * 8.05 + 1.3) * 0.0625;
}

float terrainBed(vec2 xz) {
    if (!u_hasHeightmap) {
        return u_outerBed;
    }
    vec2 uv = (xz - u_heightmapOrigin) / u_heightmapSize;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
        return u_outerBed;
    }
    vec2 texels = vec2(textureSize(u_heightmap, 0));
    uv = uv * (texels - 1.0) / texels + 0.5 / texels;
    return texture(u_heightmap, uv).r;
}

// Ground materials blended by height, slope and noise: valley grass with
// darker woods, dry meadow, bare earth on steeper ground, layered rock on
// cliffs, snow on high gentle slopes and sand at the waterline. Detail
// noise fades with distance so far slopes stay calm instead of sparkling.
void terrainMaterial(vec3 p, vec3 geometric, float viewDistance,
                     out vec3 albedo, out float roughness, out vec3 normal) {
    float near = clamp(1.0 - viewDistance / 1800.0, 0.0, 1.0);
    float mid = clamp(1.0 - viewDistance / 9000.0, 0.0, 1.0);
    vec2 xz = p.xz;
    float fine = mix(0.5, valueNoise(xz / 3.5) * 0.6 + valueNoise(xz / 11.0) * 0.4, near);
    float patchy = mix(0.5, fbm4(xz / 60.0), mid);
    float region = fbm4(xz / 900.0);

    float above = p.y - (u_hasWater ? u_waterLevel : 0.0);
    float slope = 1.0 - clamp(geometric.y, 0.0, 1.0);

    vec3 lushGrass = vec3(0.070, 0.135, 0.040);
    vec3 dryGrass = vec3(0.200, 0.190, 0.085);
    vec3 woods = vec3(0.030, 0.062, 0.030);
    vec3 earth = vec3(0.180, 0.140, 0.095);
    vec3 rockDark = vec3(0.165, 0.155, 0.145);
    vec3 rockLight = vec3(0.360, 0.335, 0.300);
    vec3 snow = vec3(0.86, 0.89, 0.93);
    vec3 sand = vec3(0.50, 0.44, 0.32);

    vec3 grass = mix(lushGrass, dryGrass, smoothstep(0.35, 0.70, region * 0.7 + patchy * 0.3));
    grass *= 0.78 + 0.44 * fine;
    float forest = smoothstep(0.52, 0.66, patchy * 0.65 + region * 0.35) *
                   (1.0 - smoothstep(0.10, 0.28, slope)) * (1.0 - smoothstep(700.0, 1050.0, above));
    grass = mix(grass, woods * (0.8 + 0.4 * fine), forest);

    // Faint layering in the rock, broken up and gone at range, where it
    // would only read as stripes.
    float layers = clamp(1.0 - viewDistance / 3500.0, 0.0, 1.0);
    float strata = valueNoise(vec2(p.y / 16.0 + patchy * 3.0 + fine * 0.6, (xz.x + xz.y) / 700.0));
    float tone = mix(0.5, strata, 0.55 * layers) * 0.6 + patchy * 0.4;
    vec3 rock = mix(rockDark, rockLight, tone) * (0.80 + 0.40 * fine);

    float alpine = smoothstep(550.0, 1000.0, above + (region - 0.5) * 300.0);
    float bare = clamp(smoothstep(0.06, 0.16, slope + (patchy - 0.5) * 0.08) * 0.6 + alpine * 0.8, 0.0, 1.0);
    float cliff = smoothstep(0.11, 0.26, slope + (patchy - 0.5) * 0.10 + alpine * 0.05);
    float snowy = smoothstep(u_snowLine - 150.0, u_snowLine + 150.0, above + (region - 0.5) * 400.0) *
                  (1.0 - smoothstep(0.32, 0.55, slope));
    float shore = u_hasWater ? (1.0 - smoothstep(1.5, 7.0, above + (fine - 0.5) * 2.0)) * (1.0 - cliff) : 0.0;
    float wet = u_hasWater ? 1.0 - smoothstep(0.0, 1.2, above) : 0.0;

    albedo = mix(grass, earth * (0.8 + 0.4 * fine), bare);
    albedo = mix(albedo, rock, cliff);
    albedo = mix(albedo, sand * (0.85 + 0.3 * fine), shore);
    albedo = mix(albedo, snow * (0.92 + 0.08 * fine), snowy);
    albedo *= 1.0 - wet * 0.45;

    roughness = mix(0.93, 0.82, cliff);
    roughness = mix(roughness, 0.88, shore);
    roughness = mix(roughness, 0.42, snowy);
    roughness = mix(roughness, 0.35, wet);

    // Mid-scale relief: gullies and outcrops on the slopes, out to ~7 km.
    normal = geometric;
    float relief = clamp(1.0 - viewDistance / 7000.0, 0.0, 1.0) * (0.35 + 0.65 * max(cliff, bare));
    if (relief > 0.001) {
        const float eps = 6.0;
        float scale = 1.0 / 70.0;
        float r0 = fbm4(xz * scale);
        float rx = fbm4((xz + vec2(eps, 0.0)) * scale);
        float rz = fbm4((xz + vec2(0.0, eps)) * scale);
        vec3 gradient = vec3(rx - r0, 0.0, rz - r0) * (9.0 * relief);
        normal = normalize(normal - gradient);
    }

    // Bump from the detail noise, stronger on rock, gone by ~1.8 km.
    if (near > 0.001) {
        const float eps = 0.4;
        float scale = 1.0 / 5.0;
        float h0 = valueNoise(xz * scale);
        float hx = valueNoise((xz + vec2(eps, 0.0)) * scale);
        float hz = valueNoise((xz + vec2(0.0, eps)) * scale);
        vec3 gradient = vec3(hx - h0, 0.0, hz - h0) * (mix(1.0, 2.6, cliff) * (1.0 - snowy * 0.7) * near);
        normal = normalize(normal - gradient);
    }
}

// Height of the animated sea surface, for its normal.
// ripples (0..1) brings in the short waves near the camera only; at range
// they would alias into sparkle.
float waveHeight(vec2 xz, float ripples) {
    float t = u_time;
    return valueNoise(xz / 160.0 + vec2(t * 0.018, t * 0.011)) * 1.1 +
           valueNoise(xz / 47.0 + vec2(-t * 0.045, t * 0.030)) * 0.40 +
           ripples * valueNoise(xz / 13.0 + vec2(t * 0.10, -t * 0.065)) * 0.10;
}

vec3 applyFog(vec3 color, vec3 viewDir, float viewDistance) {
    // Height fog with sun forward-scatter: extinction thins with altitude,
    // and fog looking toward the sun warms up (cheap aerial perspective).
    float fogHeight = mix(fs_in.fragPos_wS.y, cameraPos_wS.y, 0.5);
    float sigma = fogDensity * exp(-max(fogHeight, 0.0) * fogHeightFalloff);
    float fogAmount = 1.0 - clamp(exp(-viewDistance * sigma), 0.0, 1.0);
    float sunAmount = pow(max(dot(-viewDir, normalize(-dirLight.direction)), 0.0), 8.0);
    vec3 scatterColor = mix(fogColor, fogColor * vec3(1.35, 1.15, 0.85), sunAmount * 0.6);
    return mix(color, scatterColor, fogAmount);
}

// PBR functions
vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    float val = 1.0 - cosTheta;
    return F0 + (1.0 - F0) * (val*val*val*val*val);
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    float val = 1.0 - cosTheta;
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * (val*val*val*val*val);
}

float distributionGGX(vec3 N, vec3 H, float rough) {
    float a  = rough * rough;
    float a2 = a * a;
    float nDotH = max(dot(N, H), 0.0);
    float nDotH2 = nDotH * nDotH;
    float denom = (nDotH2 * (a2 - 1.0) + 1.0);
    return a2 / (M_PI * denom * denom);
}

float geometrySchlickGGX(float nDotV, float rough) {
    float r = (rough + 1.0);
    float k = r*r / 8.0;
    return nDotV / (nDotV * (1.0 - k) + k);
}

float geometrySmith(float nDotV, float nDotL, float rough) {
    return geometrySchlickGGX(nDotV, rough) * geometrySchlickGGX(nDotL, rough);
}

float linearDepth(float depthSample) {
    float depthRange = 2.0 * depthSample - 1.0;
    return 2.0 * zNear * zFar / (zFar + zNear - depthRange * (zFar - zNear));
}

// Per-pixel dither angle for rotating the PCF sample disk; breaks the
// banding a fixed kernel produces at soft shadow edges.
float interleavedGradientNoise(vec2 pixel) {
    return fract(52.9829189 * fract(dot(pixel, vec2(0.06711056, 0.00583715))));
}

float calcDirShadow(vec3 fragPos, vec3 normal) {
    // Normal-offset bias: sample the shadow map as if the receiver sat a
    // texel and a half above its surface, escaping acne on slopes.
    vec3 offsetPos = fragPos + normal * shadowTexelWorld * 1.5;
    vec4 fragPosLightSpace = lightSpaceMatrix * vec4(offsetPos, 1.0);
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    if (projCoords.z > 1.0) {
        return 0.0;
    }

    float nDotL = max(dot(normal, normalize(-dirLight.direction)), 0.0);
    float bias = max(0.0006, 0.0035 * (1.0 - nDotL));

    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    float angle = interleavedGradientNoise(gl_FragCoord.xy) * 6.2831853;
    float s = sin(angle);
    float c = cos(angle);
    mat2 rotation = mat2(c, s, -s, c);

    // 16-tap Vogel disk, radius 2 texels.
    const int SAMPLE_COUNT = 16;
    const float GOLDEN_ANGLE = 2.39996323;
    float shadow = 0.0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        float r = sqrt((float(i) + 0.5) / float(SAMPLE_COUNT)) * 2.0;
        float theta = float(i) * GOLDEN_ANGLE;
        vec2 offset = rotation * (vec2(cos(theta), sin(theta)) * r) * texelSize;
        float pcfDepth = texture(shadowMap, projCoords.xy + offset).r;
        shadow += (projCoords.z - bias > pcfDepth) ? 1.0 : 0.0;
    }
    shadow /= float(SAMPLE_COUNT);

    // Fade out at the moving shadow-frustum border instead of hard-cutting
    // (the map clamps to a white border beyond it).
    vec2 border = abs(projCoords.xy * 2.0 - 1.0);
    shadow *= 1.0 - smoothstep(0.9, 1.0, max(border.x, border.y));
    return shadow;
}

// Lake and sea: sky reflection by Fresnel, a sun glint, colour that deepens
// with the water under it, and foam where it meets the shore.
vec4 shadeWater(vec3 p, vec3 viewDir, float viewDistance) {
    float depth = u_waterLevel - terrainBed(p.xz);
    if (depth <= 0.0) {
        discard;
    }

    // Short waves fade with distance; the long swell stays, flattening
    // toward the horizon.
    float detail = clamp(1.0 - viewDistance / 1200.0, 0.0, 1.0);
    const float eps = 1.5;
    float h0 = waveHeight(p.xz, detail);
    float hx = waveHeight(p.xz + vec2(eps, 0.0), detail);
    float hz = waveHeight(p.xz + vec2(0.0, eps), detail);
    float strength = mix(0.35, 1.0, detail) * mix(1.0, 0.4, clamp(viewDistance / 15000.0, 0.0, 1.0));
    vec3 normal = normalize(vec3(-(hx - h0) / eps * strength, 1.0, -(hz - h0) / eps * strength));

    float nDotV = max(dot(normal, viewDir), 0.0);
    float fresnel = 0.02 + 0.98 * pow(1.0 - nDotV, 5.0);
    vec3 reflected = reflect(-viewDir, normal);
    reflected.y = abs(reflected.y);
    vec3 skyColor = skyRadiance(reflected, true);

    vec3 lightDir = normalize(-dirLight.direction);
    vec3 halfway = normalize(lightDir + viewDir);
    float glint = pow(max(dot(normal, halfway), 0.0), mix(250.0, 900.0, detail)) * mix(1.5, 5.0, detail);
    float shadow = calcDirShadow(p, vec3(0.0, 1.0, 0.0));

    float clarity = smoothstep(0.0, 14.0, depth);
    vec3 deep = vec3(0.004, 0.020, 0.032);
    vec3 shallow = vec3(0.020, 0.100, 0.090);
    vec3 ambient = skyHorizonColor * 0.9 + dirLight.color * max(lightDir.y, 0.0) * 0.06;
    vec3 body = mix(shallow, deep, clarity) * ambient * 3.0;

    vec3 color = mix(body, skyColor, fresnel) + dirLight.color * glint * (1.0 - shadow);

    // Foam along the shore, broken up and drifting.
    float foamNoise = valueNoise(p.xz / 2.5 + vec2(u_time * 0.25, -u_time * 0.18));
    float foam = (1.0 - smoothstep(0.0, 1.4, depth)) * smoothstep(0.35, 0.75, foamNoise * 0.7 + 0.3 * detail);
    vec3 foamColor = skyHorizonColor * 1.4 + dirLight.color * max(lightDir.y, 0.0) * 0.25 * (1.0 - shadow);
    color = mix(color, foamColor, foam * 0.8);

    // Shallow water lets the bed show through; deep water and grazing views do not.
    float alpha = mix(0.45, 1.0, smoothstep(0.0, 5.0, depth));
    alpha = max(alpha, fresnel);
    alpha = max(alpha, foam * 0.8);

    return vec4(applyFog(color, viewDir, viewDistance), alpha);
}

void main() {
    vec3 albedo    = u_useVertexColor ? fs_in.vertexColor : u_albedo;
    float metallic = u_useVertexMaterial ? clamp(fs_in.metalRoughness.x, 0.0, 1.0) : u_metallic;
    float roughness = u_useVertexMaterial ? clamp(fs_in.metalRoughness.y, 0.04, 1.0) : u_roughness;

    vec3 norm = normalize(fs_in.N);
    vec3 viewDir = normalize(cameraPos_wS - fs_in.fragPos_wS);
    float viewDistance = length(cameraPos_wS - fs_in.fragPos_wS);

    if (u_surface == 2) {
        FragColor = shadeWater(fs_in.fragPos_wS, viewDir, viewDistance);
        return;
    }
    if (u_surface == 1) {
        terrainMaterial(fs_in.fragPos_wS, norm, viewDistance, albedo, roughness, norm);
        metallic = 0.0;
    }

    // Procedural terrain detail. Authored vehicle materials bypass the ground
    // treatment: multi-octave albedo
    // variation plus a distance-faded normal perturbation so the terrain
    // reads as scrubland at close range and stays calm at distance.
    if (u_surface == 0 && u_useVertexColor && !u_useVertexMaterial) {
        vec2 groundUv = fs_in.fragPos_wS.xz;
        float detailNoise = valueNoise(groundUv * (1.0 / 7.0)) * 0.5 +
                            valueNoise(groundUv * (1.0 / 29.0)) * 0.3 +
                            valueNoise(groundUv * (1.0 / 450.0)) * 0.2;
        albedo *= 0.82 + 0.36 * detailNoise;

        // Macro patchiness: hue drift toward dry grass over hundreds of metres.
        float macro = valueNoise(groundUv * (1.0 / 450.0) + 17.3);
        albedo = mix(albedo, albedo * vec3(1.12, 1.05, 0.78), macro * 0.45);

        // Bump from the noise gradient, fading out by ~900 m so the far
        // field doesn't sparkle under the sun.
        float detailFade = clamp(1.0 - viewDistance / 900.0, 0.0, 1.0);
        if (detailFade > 0.001) {
            const float eps = 0.35;
            const float freq = 1.0 / 7.0;
            float h0 = valueNoise(groundUv * freq);
            float hx = valueNoise((groundUv + vec2(eps, 0.0)) * freq);
            float hz = valueNoise((groundUv + vec2(0.0, eps)) * freq);
            vec3 gradient = vec3(hx - h0, 0.0, hz - h0) * (1.4 * detailFade);
            norm = normalize(norm - gradient);
        }
    }

    vec3 R = reflect(-viewDir, norm);

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo, metallic);

    // Cluster tile lookup
    float tileDepth = max(linearDepth(gl_FragCoord.z), zNear);
    uint zTile      = uint(clamp(log2(tileDepth) * scale + bias, 0.0, float(tileSizes.z - 1u)));
    uvec2 xyTile    = min(uvec2(gl_FragCoord.xy / float(tileSizes.w)), tileSizes.xy - uvec2(1u));
    uvec3 tiles     = uvec3(xyTile, zTile);
    uint tileIndex = tiles.x +
                     tileSizes.x * tiles.y +
                     (tileSizes.x * tileSizes.y) * tiles.z;

    vec3 radianceOut = vec3(0.0);

    // Directional light with shadow
    float shadow = calcDirShadow(fs_in.fragPos_wS, norm);
    {
        vec3 lightDir = normalize(-dirLight.direction);
        vec3 halfway  = normalize(lightDir + viewDir);
        float nDotV = max(dot(norm, viewDir), 0.0);
        float nDotL = max(dot(norm, lightDir), 0.0);

        float NDF = distributionGGX(norm, halfway, roughness);
        float G   = geometrySmith(nDotV, nDotL, roughness);
        vec3  F   = fresnelSchlick(max(dot(halfway, viewDir), 0.0), F0);

        vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
        vec3 specular = (NDF * G * F) / max(4.0 * nDotV * nDotL, 0.0001);
        radianceOut = (kD * (albedo / M_PI) + specular) * dirLight.color * nDotL * (1.0 - shadow);
    }

    // Clustered point lights
    uint lightCount       = lightGrid[tileIndex].count;
    uint lightIndexOffset = lightGrid[tileIndex].offset;

    for (uint i = 0; i < lightCount; i++) {
        uint idx = globalLightIndexList[lightIndexOffset + i];
        vec3 position = pointLight[idx].position.xyz;
        vec3 color    = pointLight[idx].color.rgb * pointLight[idx].intensity;
        float radius  = pointLight[idx].range;

        vec3 lightDir = normalize(position - fs_in.fragPos_wS);
        vec3 halfway  = normalize(lightDir + viewDir);
        float nDotV = max(dot(norm, viewDir), 0.0);
        float nDotL = max(dot(norm, lightDir), 0.0);

        float distance    = length(position - fs_in.fragPos_wS);
        float attenuation = pow(clamp(1.0 - pow(distance / radius, 4.0), 0.0, 1.0), 2.0)
                          / (1.0 + distance * distance);
        vec3 radianceIn = color * attenuation;

        float NDF = distributionGGX(norm, halfway, roughness);
        float G   = geometrySmith(nDotV, nDotL, roughness);
        vec3  F   = fresnelSchlick(max(dot(halfway, viewDir), 0.0), F0);

        vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
        vec3 specular = (NDF * G * F) / max(4.0 * nDotV * nDotL, 0.0001);
        radianceOut += (kD * (albedo / M_PI) + specular) * radianceIn * nDotL;
    }

    // Ambient / IBL
    vec3 ambient = vec3(0.025) * albedo;
    if (IBL) {
        vec3  kS = fresnelSchlickRoughness(max(dot(norm, viewDir), 0.0), F0, roughness);
        vec3  kD = (1.0 - kS) * (1.0 - metallic);
        vec3 irradiance = texture(irradianceMap, norm).rgb;
        vec3 diffuse    = irradiance * albedo;

        const float MAX_REFLECTION_LOD = 4.0;
        vec3 prefilteredColor = textureLod(prefilterMap, R, roughness * MAX_REFLECTION_LOD).rgb;
        vec2 envBRDF = texture(brdfLUT, vec2(max(dot(norm, viewDir), 0.0), roughness)).rg;
        vec3 specular = prefilteredColor * (kS * envBRDF.x + envBRDF.y);
        ambient = kD * diffuse + specular;
    }
    radianceOut += ambient;

    FragColor = vec4(applyFog(radianceOut, viewDir, viewDistance), 1.0);
}
