#include "Renderer.h"
#include "SceneEffects.h"

#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"
#include "../sim/Terrain.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

glm::vec3 Renderer::keepCameraAboveGround(const glm::vec3 &position) const
{
    // No camera looks out from inside a hill or under the water.
    if (!m_terrain)
    {
        return position;
    }
    glm::vec3 kept = position;
    kept.y = std::max(kept.y, m_terrain->heightAt(kept.x, kept.z) + kCameraGroundClearance);
    return kept;
}

void Renderer::setCameraPosition(const glm::vec3 &position)
{
    m_cameraPosition = keepCameraAboveGround(position);
    updateCameraVectors();
}

void Renderer::setCameraTarget(const glm::vec3 &target)
{
    m_cameraTarget = target;

    // Calculate direction vector
    m_cameraFront = glm::normalize(target - m_cameraPosition);

    // Update camera angles based on front vector
    m_cameraPitch = glm::degrees(asin(m_cameraFront.y));
    m_cameraYaw = glm::degrees(atan2(m_cameraFront.z, m_cameraFront.x));

    updateCameraVectors();
}

void Renderer::setCameraView(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &up)
{
    if (glm::length2(forward) < 1.0e-10f)
    {
        return;
    }
    const glm::vec3 front = glm::normalize(forward);
    glm::vec3 orthoUp = up - front * glm::dot(up, front);
    if (glm::length2(orthoUp) < 1.0e-10f)
    {
        return;
    }
    orthoUp = glm::normalize(orthoUp);

    m_cameraPosition = keepCameraAboveGround(position);
    m_cameraFront = front;
    m_cameraUp = orthoUp;
    m_cameraRight = glm::normalize(glm::cross(front, orthoUp));
    m_cameraTarget = m_cameraPosition + front;
    // Keep yaw/pitch in step so the free camera resumes from this view.
    m_cameraPitch = glm::degrees(std::asin(glm::clamp(front.y, -1.0f, 1.0f)));
    m_cameraYaw = glm::degrees(std::atan2(front.z, front.x));
}

void Renderer::rotateCameraYaw(float deltaDegrees)
{
    m_cameraYaw += deltaDegrees;
    updateCameraVectors();
}

void Renderer::rotateCameraPitch(float deltaDegrees)
{
    m_cameraPitch += deltaDegrees;

    // Constrain pitch to avoid gimbal lock
    if (m_cameraPitch > 89.0f)
        m_cameraPitch = 89.0f;
    if (m_cameraPitch < -89.0f)
        m_cameraPitch = -89.0f;

    updateCameraVectors();
}

void Renderer::moveCameraForward(float distance)
{
    float scaledDistance = distance * m_cameraSpeed;
    glm::vec3 forward = glm::vec3(m_cameraFront.x, 0.0f, m_cameraFront.z);
    if (glm::length2(forward) < 0.0001f)
    {
        forward = glm::vec3(0.0f, 0.0f, -1.0f);
    }
    forward = glm::normalize(forward);
    m_cameraPosition += forward * scaledDistance;
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::moveCameraRight(float distance)
{
    float scaledDistance = distance * m_cameraSpeed;
    glm::vec3 right = glm::vec3(m_cameraRight.x, 0.0f, m_cameraRight.z);
    if (glm::length2(right) < 0.0001f)
    {
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    right = glm::normalize(right);
    m_cameraPosition += right * scaledDistance;
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::moveCameraUp(float distance)
{
    float scaledDistance = distance * m_cameraSpeed;
    m_cameraPosition += glm::vec3(0.0f, 1.0f, 0.0f) * scaledDistance;
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

void Renderer::setTerrain(std::shared_ptr<const missilesim::sim::Terrain> terrain)
{
    if (terrain == m_terrain)
    {
        return;
    }
    m_terrain = std::move(terrain);
    createFloor();
    uploadFloorMesh();
    uploadTerrainSurface();
}

void Renderer::updateCameraVectors()
{
    // Calculate front vector from yaw and pitch
    glm::vec3 front;
    front.x = cos(glm::radians(m_cameraYaw)) * cos(glm::radians(m_cameraPitch));
    front.y = sin(glm::radians(m_cameraPitch));
    front.z = sin(glm::radians(m_cameraYaw)) * cos(glm::radians(m_cameraPitch));
    m_cameraFront = glm::normalize(front);

    // Recalculate the right and up vectors
    m_cameraRight = glm::normalize(glm::cross(m_cameraFront, glm::vec3(0.0f, 1.0f, 0.0f)));
    m_cameraUp = glm::normalize(glm::cross(m_cameraRight, m_cameraFront));

    // Update target position based on front vector
    m_cameraTarget = m_cameraPosition + m_cameraFront;
}

glm::mat4 Renderer::buildViewMatrix() const
{
    return glm::lookAt(m_cameraPosition, m_cameraTarget, m_cameraUp);
}

glm::mat4 Renderer::buildProjectionMatrix() const
{
    const int safeHeight = std::max(m_viewportHeight, 1);
    float aspectRatio = static_cast<float>(m_viewportWidth) / static_cast<float>(safeHeight);
    return glm::perspective(glm::radians(m_cameraFOV), aspectRatio, kNearPlane, m_sceneFarPlane);
}