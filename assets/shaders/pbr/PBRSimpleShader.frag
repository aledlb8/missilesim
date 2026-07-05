#version 430 core

out vec4 FragColor;

in VS_OUT {
    vec3 fragPos_wS;
    vec4 fragPos_lS;
    vec3 N;
    vec3 vertexColor;
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

void main() {
    vec3 albedo    = u_useVertexColor ? fs_in.vertexColor : u_albedo;
    float metallic = u_metallic;
    float roughness = u_roughness;

    vec3 norm = normalize(fs_in.N);
    vec3 viewDir = normalize(cameraPos_wS - fs_in.fragPos_wS);
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
    float viewDistance = length(cameraPos_wS - fs_in.fragPos_wS);
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

    float fogFactor = clamp(exp(-viewDistance * fogDensity), 0.0, 1.0);
    radianceOut = mix(fogColor, radianceOut, fogFactor);

    FragColor = vec4(radianceOut, 1.0);
}
