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
#include <iterator>
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

void Application::processInput(float deltaTime)
{
    if (!m_window || !m_renderer)
    {
        return;
    }

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        deltaTime = 0.016f;
    }

    // Edge-triggered keys. The latches keep tracking while menus own the
    // keyboard, so a key still held as a menu closes (Enter on "Resume") does
    // not also fire its gameplay action on the next frame.
    static constexpr int kLatchedKeys[] = {GLFW_KEY_TAB, GLFW_KEY_H, GLFW_KEY_V, GLFW_KEY_ENTER,
                                           GLFW_KEY_KP_ENTER, GLFW_KEY_C, GLFW_KEY_R, GLFW_KEY_F};
    static bool keyHeld[std::size(kLatchedKeys)] = {};
    bool keyPressed[std::size(kLatchedKeys)] = {};
    for (size_t i = 0; i < std::size(kLatchedKeys); ++i)
    {
        const bool down = glfwGetKey(m_window, kLatchedKeys[i]) == GLFW_PRESS;
        keyPressed[i] = down && !keyHeld[i];
        keyHeld[i] = down;
    }
    auto pressed = [&](int key)
    {
        for (size_t i = 0; i < std::size(kLatchedKeys); ++i)
        {
            if (kLatchedKeys[i] == key)
            {
                return keyPressed[i];
            }
        }
        return false;
    };

    // Title screen and menus own the keyboard (handled in ApplicationMenus.cpp).
    if (!gameplayInputEnabled())
    {
        return;
    }

    // Typing into a field (e.g. an exact slider value) must not fly the camera.
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantTextInput)
    {
        updatePreLaunchSeekerLock();
        return;
    }

    if (pressed(GLFW_KEY_TAB))
    {
        m_showUI = !m_showUI;
    }
    if (pressed(GLFW_KEY_H))
    {
        m_hudVisible = !m_hudVisible;
    }
    if (pressed(GLFW_KEY_V))
    {
        cycleCameraMode();
    }
    if (pressed(GLFW_KEY_ENTER) || pressed(GLFW_KEY_KP_ENTER))
    {
        m_isPaused = !m_isPaused;
    }
    if (pressed(GLFW_KEY_C))
    {
        setCameraMode(CameraMode::FREE, true);
    }

    if (m_cameraMode == CameraMode::FREE)
    {
        const bool speedBoost = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                                glfwGetKey(m_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        const float cameraStep = deltaTime * (speedBoost ? 2.8f : 1.0f);

        if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
        {
            m_renderer->moveCameraForward(cameraStep);
        }
        if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS)
        {
            m_renderer->moveCameraForward(-cameraStep);
        }
        if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS)
        {
            m_renderer->moveCameraRight(-cameraStep);
        }
        if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS)
        {
            m_renderer->moveCameraRight(cameraStep);
        }
        if (glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS)
        {
            m_renderer->moveCameraUp(cameraStep);
        }
        if (glfwGetKey(m_window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
            glfwGetKey(m_window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS)
        {
            m_renderer->moveCameraUp(-cameraStep);
        }
    }

    if (pressed(GLFW_KEY_R))
    {
        m_seekerCueEnabled = !m_seekerCueEnabled;
        if (!m_seekerCueEnabled && !m_missileInFlight && m_missile)
        {
            m_missile->clearTarget();
        }
    }

    updatePreLaunchSeekerLock();

    if (pressed(GLFW_KEY_F))
    {
        launchMissile();
    }
}

void Application::cycleCameraMode()
{
    switch (m_cameraMode)
    {
    case CameraMode::FREE:
        setCameraMode(CameraMode::MISSILE);
        break;
    case CameraMode::MISSILE:
        setCameraMode(CameraMode::FIGHTER_JET);
        break;
    case CameraMode::FIGHTER_JET:
        setCameraMode(CameraMode::FREE);
        break;
    }
}

void Application::mouseCallback(double xpos, double ypos)
{
    // Skip if camera rotation is disabled or imgui has focus
    if (!m_enableMouseCamera || !gameplayInputEnabled() || ImGui::GetIO().WantCaptureMouse)
        return;

    const float mouseX = static_cast<float>(xpos);
    const float mouseY = static_cast<float>(ypos);
    if (m_firstMouse)
    {
        m_lastMouseX = mouseX;
        m_lastMouseY = mouseY;
        m_firstMouse = false;
        return;
    }

    // Calculate mouse movement
    float xoffset = mouseX - m_lastMouseX;
    float yoffset = m_lastMouseY - mouseY; // Reversed since y-coordinates go from bottom to top

    m_lastMouseX = mouseX;
    m_lastMouseY = mouseY;

    // Apply sensitivity factor
    const float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    if (m_cameraMode == CameraMode::FREE)
    {
        m_renderer->rotateCameraYaw(xoffset);
        m_renderer->rotateCameraPitch(yoffset);
        return;
    }

    updateChaseOrbit(xoffset, -yoffset);
}

void Application::mouseButtonCallback(int button, int action)
{
    // Check if the right mouse button is pressed or released
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (action == GLFW_PRESS && gameplayInputEnabled() && !ImGui::GetIO().WantCaptureMouse)
        {
            // Enable camera rotation and hide cursor
            m_enableMouseCamera = true;
            glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            m_firstMouse = true; // Reset first mouse flag to avoid jumps
            if (m_cameraMode != CameraMode::FREE)
            {
                m_chaseCameraState.initialized = false;
                m_chaseCameraState.returnBlend = 1.0f;
            }
        }
        else if (action == GLFW_RELEASE)
        {
            releaseMouseCameraCapture();
        }
    }
}