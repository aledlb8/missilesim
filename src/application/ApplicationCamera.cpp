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
#include "objects/Fighter.h"
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
    // While holding on a detonation, keep onboard cameras pointed at the blast
    // rather than chasing the (now inert) missile or target.
    if (m_detonationHoldActive && m_cameraMode != CameraMode::FREE)
    {
        frameDetonationCamera();
        return;
    }

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

void Application::frameDetonationCamera()
{
    if (!m_renderer)
    {
        return;
    }

    // Freeze the camera where the chase left it and ease round to the impact
    // point so the explosion stays centred for the hold, keeping the current
    // roll instead of snapping it level.
    const glm::vec3 position = m_renderer->getCameraPosition();
    const glm::vec3 front = m_renderer->getCameraFront();
    const glm::vec3 toBlast = safeNormalize(m_detonationHoldPosition - position, front);
    const float blend = 1.0f - std::exp(-6.0f * std::max(m_lastFrameDeltaTime, 0.0f));
    const glm::vec3 forward = safeNormalize(front + (toBlast - front) * blend, toBlast);
    m_renderer->setCameraView(position, forward, m_renderer->getCameraUp());
}

void Application::updateMissileCamera()
{
    if (!m_renderer || !m_missile)
    {
        return;
    }

    glm::vec3 heading = m_missile->getVelocity();
    if (glm::length2(heading) < 1.0f)
    {
        heading = m_missile->getThrustDirection();
    }
    const float speed = glm::length(m_missile->getVelocity());
    const float distance = std::clamp(8.0f + speed * 0.04f, 8.0f, 30.0f);
    const float height = std::clamp(1.4f + speed * 0.005f, 1.4f, 7.0f);
    const float lookAhead = std::clamp(25.0f + speed * 0.12f, 25.0f, 120.0f);
    m_chaseCamera.setOrbiting(m_enableMouseCamera);
    m_chaseCamera.update(m_lastFrameDeltaTime, m_missile->getRenderPosition(), heading, distance, height, lookAhead, m_savedCameraFOV);
    applyCameraPose(m_chaseCamera.pose());
}

void Application::updateFighterJetCamera()
{
    if (!m_renderer)
    {
        return;
    }

    if (m_playerRole == PlayerRole::Fighter && m_fighter)
    {
        m_aimCamera.update(m_lastFrameDeltaTime, m_fighter->getRenderPosition(), m_fighter->getRadius(), m_savedCameraFOV,
                           m_cameraSmoothing);
        applyCameraPose(m_aimCamera.pose());
        return;
    }

    // SAM role: ride along behind the target the missile is after.
    Target *focusTarget = getTrackedMissileTarget();
    if (focusTarget == nullptr)
    {
        focusTarget = findBestTarget();
    }
    if (focusTarget == nullptr)
    {
        return;
    }

    glm::vec3 heading = focusTarget->getVelocity();
    if (glm::length2(heading) < 1.0f && m_missile)
    {
        heading = focusTarget->getPosition() - m_missile->getPosition();
    }
    const float speed = glm::length(focusTarget->getVelocity());
    const float radius = std::max(focusTarget->getRadius(), 1.0f);
    const float distance = std::clamp(radius * 4.0f + speed * 0.05f, 12.0f, 40.0f);
    const float height = std::clamp(radius * 1.5f + speed * 0.008f, 3.0f, 12.0f);
    const float lookAhead = std::clamp(radius * 10.0f + speed * 0.15f, 20.0f, 150.0f);
    m_chaseCamera.setOrbiting(m_enableMouseCamera);
    m_chaseCamera.update(m_lastFrameDeltaTime, focusTarget->getRenderPosition(), heading, distance, height, lookAhead, m_savedCameraFOV);
    applyCameraPose(m_chaseCamera.pose());
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
    m_chaseCamera.reset();
}

void Application::applyCameraPose(const CameraPose &pose)
{
    if (!m_renderer)
    {
        return;
    }
    m_renderer->setCameraView(pose.position, pose.forward, pose.up);
    m_renderer->setCameraFOV(pose.fov);
}

void Application::resetAimCamera()
{
    const glm::vec3 nose = m_fighter ? m_fighter->getNose() : glm::vec3(0.0f, 0.0f, 1.0f);
    m_aimCamera.reset(nose);
    m_pendingMouseDelta = glm::vec2(0.0f);
}

bool Application::mouseAimActive() const
{
    return m_playerRole == PlayerRole::Fighter && m_fighter && m_cameraMode == CameraMode::FIGHTER_JET &&
           m_screen == Screen::Playing && m_overlay == Overlay::None && !m_showUI;
}

void Application::updateCursorCapture()
{
    if (!m_window)
    {
        return;
    }
    const bool wanted = mouseAimActive() || m_enableMouseCamera;
    if (wanted == m_cursorCaptured)
    {
        return;
    }
    m_cursorCaptured = wanted;
    m_firstMouse = true;
    m_pendingMouseDelta = glm::vec2(0.0f);
    glfwSetInputMode(m_window, GLFW_CURSOR, wanted ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    // Raw (unaccelerated, unscaled) motion while captured, where supported:
    // the aim then moves exactly with the hand, as a shooter's does.
    if (glfwRawMouseMotionSupported())
    {
        glfwSetInputMode(m_window, GLFW_RAW_MOUSE_MOTION, wanted ? GLFW_TRUE : GLFW_FALSE);
    }
}

void Application::releaseMouseCameraCapture()
{
    m_enableMouseCamera = false;
    m_firstMouse = true;
    m_freeLookMouseHeld = false;
    m_freeLookHeld = false;
    m_chaseCamera.setOrbiting(false);
    m_aimCamera.setFreeLook(false);
    if (m_window && m_cursorCaptured)
    {
        m_cursorCaptured = false;
        glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        if (glfwRawMouseMotionSupported())
        {
            glfwSetInputMode(m_window, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
        }
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