#version 430 core

// Vertex layout matching MissileSim's legacy Renderer::Vertex
layout (location = 0) in vec3 vertexPos_mS;
layout (location = 1) in vec3 normal_mS;
layout (location = 2) in vec3 vertexColor;  // unused in PBR, but present in legacy VAO

out VS_OUT {
    vec3 fragPos_wS;
    vec4 fragPos_lS;
    vec3 N;
} vs_out;

uniform mat4 MVP;
uniform mat4 M;
uniform mat4 lightSpaceMatrix;

void main() {
    gl_Position = MVP * vec4(vertexPos_mS, 1.0);

    vs_out.fragPos_wS = mat3(M) * vertexPos_mS;
    vs_out.N = normalize(mat3(M) * normal_mS);
    vs_out.fragPos_lS = lightSpaceMatrix * vec4(vs_out.fragPos_wS, 1.0);
}
