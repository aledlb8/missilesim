#pragma once

#include <glm/glm.hpp>

namespace pbr {

struct DirectionalLight
{
    glm::vec3 direction{-0.35f, -0.9f, 0.2f};
    glm::vec3 color{1.0f};
    float strength = 2.5f;
    float distance = 200.0f;
    float orthoBoxSize = 200.0f;
    float zNear = 1.0f;
    float zFar = 2000.0f;
    unsigned int shadowRes = 2048;

    glm::mat4 shadowProjectionMat{0.0f};
    glm::mat4 lightView{0.0f};
    glm::mat4 lightSpaceMatrix{0.0f};
    unsigned int depthMapTextureID = 0;
};

struct PointLight
{
    glm::vec3 position{0.0f};
    glm::vec3 color{1.0f};
    float strength = 1.0f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    unsigned int shadowRes = 2048;

    glm::mat4 shadowProjectionMat{0.0f};
    glm::mat4 lookAtPerFace[6];
    unsigned int depthMapTextureID = 0;
};

struct GPULight
{
    glm::vec4 position;
    glm::vec4 color;
    unsigned int enabled;
    float intensity;
    float range;
    float padding;
};

} // namespace pbr
