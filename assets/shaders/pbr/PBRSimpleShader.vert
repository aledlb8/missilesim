#version 430 core

// Vertex layout matching MissileSim's legacy Renderer::Vertex
layout (location = 0) in vec3 vertexPos_mS;
layout (location = 1) in vec3 normal_mS;
layout (location = 2) in vec3 vertexColor;
layout (location = 3) in vec2 metalRoughness;

out VS_OUT {
    vec3 fragPos_wS;
    vec4 fragPos_lS;
    vec3 N;
    vec3 vertexColor;
    vec2 metalRoughness;
} vs_out;

uniform mat4 MVP;
uniform mat4 M;
uniform mat4 lightSpaceMatrix;

void main() {
    vec4 worldPos = M * vec4(vertexPos_mS, 1.0);
    gl_Position = MVP * vec4(vertexPos_mS, 1.0);

    vs_out.fragPos_wS = worldPos.xyz;
    vs_out.N = normalize(mat3(transpose(inverse(M))) * normal_mS);
    vs_out.vertexColor = vertexColor;
    vs_out.metalRoughness = metalRoughness;
    vs_out.fragPos_lS = lightSpaceMatrix * worldPos;
}
