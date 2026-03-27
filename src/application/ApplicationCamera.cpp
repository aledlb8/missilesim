#include "Application.h"
#include "ApplicationDetail.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <unordered_map>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

#include "audio/AudioSystem.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/Atmosphere.h"
#include "physics/PhysicsEngine.h"
#include "physics/forces/Drag.h"
#include "physics/forces/Lift.h"
#include "rendering/Renderer.h"

using missilesim::application::detail::formatBoolValue;
using missilesim::application::detail::formatVec3Value;
using missilesim::application::detail::parseBoolValue;
using missilesim::application::detail::parseFloatValue;
using missilesim::application::detail::parseIntValue;
using missilesim::application::detail::parseVec3Value;
using missilesim::application::detail::safeNormalize;
using missilesim::application::detail::trimWhitespace;

float Application::computeEngagementRadius() const
{
    float engagementRadius = std::max(400.0f, m_targetAIConfig.preferredDistance * 1.6f);

    if (m_missile)
    {
        engagementRadius = std::max(engagementRadius, glm::length(glm::vec2(m_missile->getPosition().x, m_missile->getPosition().z)) + 150.0f);
    }

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        engagementRadius = std::max(engagementRadius, glm::length(glm::vec2(target->getPosition().x, target->getPosition().z)) + (target->getRadius() * 8.0f));
    }

    return engagementRadius;
}

void Application::updateEnvironmentScale()
{
    if (!m_renderer)
    {
        return;
    }

    const float engagementRadius = computeEngagementRadius();
    float maxAltitude = std::max(320.0f, glm::clamp(m_targetAIConfig.preferredDistance * 0.22f, 180.0f, 900.0f));

    if (m_missile)
    {
        maxAltitude = std::max(maxAltitude, m_missile->getPosition().y + 150.0f);
    }

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        maxAltitude = std::max(maxAltitude, target->getPosition().y + std::max(120.0f, m_targetAIConfig.preferredDistance * 0.18f));
    }

    const float airspaceHalfExtent = std::max(600.0f, engagementRadius * 1.35f);
    const float groundHalfExtent = std::max(1200.0f, airspaceHalfExtent * 2.4f);
    const float airspaceHeight = std::max(320.0f, std::max(maxAltitude * 1.6f, engagementRadius * 0.45f));
    m_renderer->setEnvironmentMetrics(groundHalfExtent, airspaceHalfExtent, airspaceHeight);
}

void Application::frameEngagementCamera()
{
    if (!m_renderer)
    {
        return;
    }

    std::vector<glm::vec3> points;
    points.reserve(m_targets.size() + 2);
    points.push_back(glm::vec3(0.0f, 0.0f, 0.0f));

    if (m_missile)
    {
        points.push_back(m_missile->getPosition());
    }

    for (const auto &target : m_targets)
    {
        if (target && target->isActive())
        {
            points.push_back(target->getPosition());
        }
    }

    glm::vec3 minPoint = points.front();
    glm::vec3 maxPoint = points.front();
    for (const glm::vec3 &point : points)
    {
        minPoint = glm::min(minPoint, point);
        maxPoint = glm::max(maxPoint, point);
    }

    const glm::vec3 center = 0.5f * (minPoint + maxPoint);
    const float horizontalSpan = glm::length(glm::vec2(maxPoint.x - minPoint.x, maxPoint.z - minPoint.z));
    const float verticalSpan = maxPoint.y - minPoint.y;
    const float framingRadius = std::max({horizontalSpan * 0.60f, verticalSpan * 1.35f, computeEngagementRadius() * 0.65f, 150.0f});

    glm::vec3 cameraDirection = glm::normalize(glm::vec3(-1.15f, 0.60f, 1.05f));
    glm::vec3 cameraPosition = center + cameraDirection * (framingRadius * 1.85f);
    cameraPosition.y = std::max(cameraPosition.y, center.y + framingRadius * 0.60f);

    glm::vec3 lookTarget = center;
    lookTarget.y = std::max(18.0f, center.y + verticalSpan * 0.20f);

    m_renderer->setCameraSpeed(std::clamp(framingRadius * 0.22f, 30.0f, 800.0f));
    m_renderer->setCameraFOV(50.0f);
    m_renderer->setCameraPosition(cameraPosition);
    m_renderer->setCameraTarget(lookTarget);

    if (m_cameraMode == CameraMode::FREE)
    {
        captureFreeCameraState();
    }
}

void Application::setCameraMode(CameraMode mode, bool frameFreeCamera)
{
    if (!m_renderer)
    {
        m_cameraMode = mode;
        return;
    }

    const CameraMode previousMode = m_cameraMode;
    if (mode == previousMode && !(mode == CameraMode::FREE && frameFreeCamera))
    {
        if (mode != CameraMode::FREE)
        {
            updateActiveCameraMode();
        }
        return;
    }

    if (previousMode == CameraMode::FREE)
    {
        captureFreeCameraState();
    }

    if (mode != CameraMode::FREE)
    {
        releaseMouseCameraCapture();
    }

    if (mode != previousMode)
    {
        resetChaseCameraState();
    }

    m_cameraMode = mode;

    if (mode == CameraMode::FREE)
    {
        if (frameFreeCamera || !m_freeCameraState.valid)
        {
            frameEngagementCamera();
        }
        else
        {
            restoreFreeCameraState();
        }
        return;
    }

    updateActiveCameraMode();
}

void Application::updateActiveCameraMode()
{
    switch (m_cameraMode)
    {
    case CameraMode::FREE:
        return;
    case CameraMode::MISSILE:
        updateMissileCamera();
        return;
    case CameraMode::FIGHTER_JET:
        updateFighterJetCamera();
        return;
    }
}

void Application::updateMissileCamera()
{
    if (!m_renderer || !m_missile)
    {
        return;
    }

    const glm::vec3 fallbackForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::vec3 forward = safeNormalize(m_missile->getVelocity(), glm::vec3(0.0f));
    if (glm::length2(forward) < 0.0001f)
    {
        forward = safeNormalize(m_missile->getThrustDirection(), fallbackForward);
    }

    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 missilePosition = m_missile->getPosition();
    const float missileSpeed = glm::length(m_missile->getVelocity());
    const float chaseDistance = std::clamp(8.0f + missileSpeed * 0.04f, 8.0f, 30.0f);
    const float chaseHeight = std::clamp(1.4f + missileSpeed * 0.005f, 1.4f, 7.0f);
    const float lookAhead = std::clamp(25.0f + missileSpeed * 0.12f, 25.0f, 120.0f);

    const glm::vec3 focusPoint = missilePosition + worldUp * 0.8f;
    const glm::vec3 cameraPosition = missilePosition - forward * chaseDistance + worldUp * chaseHeight;
    const glm::vec3 lookTarget = missilePosition + forward * lookAhead + worldUp * 0.8f;

    applyChaseCamera(focusPoint, cameraPosition, lookTarget);
}

void Application::updateFighterJetCamera()
{
    if (!m_renderer)
    {
        return;
    }

    Target *focusTarget = getTrackedMissileTarget();
    if (focusTarget == nullptr)
    {
        focusTarget = findBestTarget();
    }

    if (focusTarget == nullptr)
    {
        return;
    }

    const glm::vec3 fallbackForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::vec3 forward = safeNormalize(focusTarget->getVelocity(), glm::vec3(0.0f));
    if (glm::length2(forward) < 0.0001f && m_missile)
    {
        forward = safeNormalize(focusTarget->getPosition() - m_missile->getPosition(), fallbackForward);
    }
    if (glm::length2(forward) < 0.0001f)
    {
        forward = fallbackForward;
    }

    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 targetPosition = focusTarget->getPosition();
    const float targetSpeed = glm::length(focusTarget->getVelocity());
    const float targetRadius = std::max(focusTarget->getRadius(), 1.0f);
    const float chaseDistance = std::clamp(targetRadius * 4.0f + targetSpeed * 0.05f, 12.0f, 40.0f);
    const float chaseHeight = std::clamp(targetRadius * 1.5f + targetSpeed * 0.008f, 3.0f, 12.0f);
    const float lookAhead = std::clamp(targetRadius * 10.0f + targetSpeed * 0.15f, 20.0f, 150.0f);

    const glm::vec3 focusPoint = targetPosition + worldUp * (targetRadius * 0.35f);
    const glm::vec3 cameraPosition = targetPosition - forward * chaseDistance + worldUp * chaseHeight;
    const glm::vec3 lookTarget = targetPosition + forward * lookAhead + worldUp * (targetRadius * 0.35f);

    applyChaseCamera(focusPoint, cameraPosition, lookTarget);
}

void Application::captureFreeCameraState()
{
    if (!m_renderer)
    {
        return;
    }

    m_freeCameraState.position = m_renderer->getCameraPosition();
    m_freeCameraState.target = m_renderer->getCameraTarget();
    m_freeCameraState.fov = m_renderer->getCameraFOV();
    m_freeCameraState.speed = m_renderer->getCameraSpeed();
    m_freeCameraState.valid = true;
}

void Application::restoreFreeCameraState()
{
    if (!m_renderer || !m_freeCameraState.valid)
    {
        return;
    }

    m_renderer->setCameraFOV(m_freeCameraState.fov);
    m_renderer->setCameraSpeed(m_freeCameraState.speed);
    m_renderer->setCameraPosition(m_freeCameraState.position);
    m_renderer->setCameraTarget(m_freeCameraState.target);
}

void Application::resetChaseCameraState()
{
    m_chaseCameraState = {};
}

void Application::primeChaseCameraState(const glm::vec3 &focusPoint)
{
    if (!m_renderer)
    {
        return;
    }

    glm::vec3 offset = m_renderer->getCameraPosition() - focusPoint;
    float distance = glm::length(offset);
    if (distance <= 0.0001f)
    {
        offset = -safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f)) * 12.0f;
        distance = glm::length(offset);
    }

    const glm::vec3 direction = offset / std::max(distance, 0.0001f);
    m_chaseCameraState.yaw = glm::degrees(std::atan2(direction.z, direction.x));
    m_chaseCameraState.pitch = glm::degrees(std::asin(glm::clamp(direction.y, -1.0f, 1.0f)));
    m_chaseCameraState.distance = distance;
    m_chaseCameraState.returnBlend = 1.0f;
    m_chaseCameraState.initialized = true;
}

void Application::updateChaseOrbit(float yawDeltaDegrees, float pitchDeltaDegrees)
{
    if (m_cameraMode == CameraMode::FREE)
    {
        return;
    }

    if (!m_chaseCameraState.initialized)
    {
        return;
    }

    m_chaseCameraState.yaw += yawDeltaDegrees;
    m_chaseCameraState.pitch = glm::clamp(m_chaseCameraState.pitch + pitchDeltaDegrees, -80.0f, 80.0f);
    m_chaseCameraState.returnBlend = 1.0f;
}

void Application::applyChaseCamera(const glm::vec3 &focusPoint,
                                   const glm::vec3 &defaultPosition,
                                   const glm::vec3 &defaultTarget)
{
    if (!m_renderer)
    {
        return;
    }

    auto buildOrbitPosition = [&]() -> glm::vec3
    {
        const float yaw = glm::radians(m_chaseCameraState.yaw);
        const float pitch = glm::radians(m_chaseCameraState.pitch);
        const float cosPitch = std::cos(pitch);
        glm::vec3 offset(cosPitch * std::cos(yaw),
                         std::sin(pitch),
                         cosPitch * std::sin(yaw));

        if (glm::length2(offset) <= 0.0001f)
        {
            offset = glm::vec3(0.0f, 0.0f, 1.0f);
        }
        else
        {
            offset = glm::normalize(offset);
        }

        return focusPoint + (offset * std::max(m_chaseCameraState.distance, 1.0f));
    };

    if (m_enableMouseCamera)
    {
        if (!m_chaseCameraState.initialized)
        {
            primeChaseCameraState(focusPoint);
        }

        if (m_chaseCameraState.initialized)
        {
            m_renderer->setCameraPosition(buildOrbitPosition());
            m_renderer->setCameraTarget(focusPoint);
            m_chaseCameraState.returnBlend = 1.0f;
            return;
        }
    }

    if (m_chaseCameraState.initialized && m_chaseCameraState.returnBlend > 0.001f)
    {
        const glm::vec3 orbitPosition = buildOrbitPosition();
        const float blend = glm::clamp(m_chaseCameraState.returnBlend, 0.0f, 1.0f);
        m_renderer->setCameraPosition(glm::mix(defaultPosition, orbitPosition, blend));
        m_renderer->setCameraTarget(glm::mix(defaultTarget, focusPoint, blend));

        const float blendDecay = glm::clamp(m_lastFrameDeltaTime, 0.0f, 0.05f) * 3.5f;
        m_chaseCameraState.returnBlend = std::max(0.0f, blend - blendDecay);
        if (m_chaseCameraState.returnBlend <= 0.0f)
        {
            m_chaseCameraState.initialized = false;
        }
        return;
    }

    m_renderer->setCameraPosition(defaultPosition);
    m_renderer->setCameraTarget(defaultTarget);
}

void Application::releaseMouseCameraCapture()
{
    const bool wasCapturing = m_enableMouseCamera;
    m_enableMouseCamera = false;
    m_firstMouse = true;

    if (wasCapturing && m_cameraMode != CameraMode::FREE && m_chaseCameraState.initialized)
    {
        m_chaseCameraState.returnBlend = 1.0f;
    }

    if (m_window)
    {
        glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
}

const char *Application::getCameraModeLabel() const
{
    switch (m_cameraMode)
    {
    case CameraMode::FREE:
        return "Free";
    case CameraMode::MISSILE:
        return "Missile";
    case CameraMode::FIGHTER_JET:
        return "Fighter Jet";
    }

    return "Free";
}