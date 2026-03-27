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

    static bool tabPressed = false;
    if (glfwGetKey(m_window, GLFW_KEY_TAB) == GLFW_PRESS)
    {
        if (!tabPressed)
        {
            m_showUI = !m_showUI;
            tabPressed = true;
        }
    }
    else
    {
        tabPressed = false;
    }

    const bool uiCapturesKeyboard = m_showUI &&
                                    ImGui::GetCurrentContext() != nullptr &&
                                    ImGui::GetIO().WantCaptureKeyboard;

    if (uiCapturesKeyboard)
    {
        updatePreLaunchSeekerLock();
        return;
    }

    static bool enterPressed = false;
    if (glfwGetKey(m_window, GLFW_KEY_ENTER) == GLFW_PRESS ||
        glfwGetKey(m_window, GLFW_KEY_KP_ENTER) == GLFW_PRESS)
    {
        if (!enterPressed)
        {
            m_isPaused = !m_isPaused;
            enterPressed = true;
        }
    }
    else
    {
        enterPressed = false;
    }

    static bool focusPressed = false;
    if (glfwGetKey(m_window, GLFW_KEY_C) == GLFW_PRESS)
    {
        if (!focusPressed)
        {
            setCameraMode(CameraMode::FREE, true);
            focusPressed = true;
        }
    }
    else
    {
        focusPressed = false;
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

    static bool seekerCuePressed = false;
    if (glfwGetKey(m_window, GLFW_KEY_R) == GLFW_PRESS)
    {
        if (!seekerCuePressed)
        {
            m_seekerCueEnabled = !m_seekerCueEnabled;
            if (!m_seekerCueEnabled && !m_missileInFlight && m_missile)
            {
                m_missile->clearTarget();
            }
            seekerCuePressed = true;
        }
    }
    else
    {
        seekerCuePressed = false;
    }

    updatePreLaunchSeekerLock();

    static bool launchKeyPressed = false;
    if (glfwGetKey(m_window, GLFW_KEY_F) == GLFW_PRESS)
    {
        if (!launchKeyPressed)
        {
            launchMissile();
            launchKeyPressed = true;
        }
    }
    else
    {
        launchKeyPressed = false;
    }
}

void Application::mouseCallback(double xpos, double ypos)
{
    // Skip if camera rotation is disabled or imgui has focus
    if (!m_enableMouseCamera ||
        (m_showUI && ImGui::GetIO().WantCaptureMouse))
        return;

    if (m_firstMouse)
    {
        m_lastMouseX = xpos;
        m_lastMouseY = ypos;
        m_firstMouse = false;
        return;
    }

    // Calculate mouse movement
    float xoffset = xpos - m_lastMouseX;
    float yoffset = m_lastMouseY - ypos; // Reversed since y-coordinates go from bottom to top

    m_lastMouseX = xpos;
    m_lastMouseY = ypos;

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
        if (action == GLFW_PRESS)
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