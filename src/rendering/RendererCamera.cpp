#include "Renderer.h"
#include "SceneEffects.h"

#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"

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

namespace
{
    constexpr float kEnvironmentShrinkHysteresis = 0.72f;

    float quantizeEnvironmentMetric(float value, float minimumStep)
    {
        const float step = std::max(minimumStep, value * 0.12f);
        return std::ceil(value / step) * step;
    }

    float adoptQuantizedEnvironmentMetric(float currentValue, float requestedValue)
    {
        if (requestedValue > currentValue + 0.5f)
        {
            return requestedValue;
        }

        if (requestedValue < currentValue * kEnvironmentShrinkHysteresis)
        {
            return requestedValue;
        }

        return currentValue;
    }
}

void Renderer::setCameraPosition(const glm::vec3 &position)
{
    m_cameraPosition = position;
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

void Renderer::setEnvironmentMetrics(float groundHalfExtent, float airspaceHalfExtent, float airspaceHeight)
{
    const float requestedGroundHalfExtent = quantizeEnvironmentMetric(std::max(groundHalfExtent, 1200.0f), 240.0f);
    const float requestedAirspaceHalfExtent = quantizeEnvironmentMetric(std::max(airspaceHalfExtent, 600.0f), 120.0f);
    const float requestedAirspaceHeight = quantizeEnvironmentMetric(std::max(airspaceHeight, 320.0f), 80.0f);

    const float updatedGroundHalfExtent = adoptQuantizedEnvironmentMetric(m_groundHalfExtent, requestedGroundHalfExtent);
    const float updatedAirspaceHalfExtent = adoptQuantizedEnvironmentMetric(m_airspaceHalfExtent, requestedAirspaceHalfExtent);
    const float updatedAirspaceHeight = adoptQuantizedEnvironmentMetric(m_airspaceHeight, requestedAirspaceHeight);

    const bool groundExtentChanged = std::abs(updatedGroundHalfExtent - m_groundHalfExtent) >= 0.5f;
    const bool airspaceExtentChanged = std::abs(updatedAirspaceHalfExtent - m_airspaceHalfExtent) >= 0.5f;
    const bool airspaceHeightChanged = std::abs(updatedAirspaceHeight - m_airspaceHeight) >= 0.5f;

    if (!groundExtentChanged && !airspaceExtentChanged && !airspaceHeightChanged)
    {
        return;
    }

    m_groundHalfExtent = updatedGroundHalfExtent;
    m_airspaceHalfExtent = updatedAirspaceHalfExtent;
    m_airspaceHeight = updatedAirspaceHeight;
    m_sceneFarPlane = std::max(20000.0f, (m_groundHalfExtent * 3.5f) + (m_airspaceHeight * 2.0f));

    if (groundExtentChanged || airspaceExtentChanged)
    {
        createFloor();
        uploadFloorMesh();
    }
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
    return glm::perspective(glm::radians(m_cameraFOV), aspectRatio, 0.1f, m_sceneFarPlane);
}