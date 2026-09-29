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
                                           GLFW_KEY_KP_ENTER, GLFW_KEY_C, GLFW_KEY_R, GLFW_KEY_F,
                                           GLFW_KEY_G, GLFW_KEY_X};
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
        updateCursorCapture();
        sampleFighterControls(deltaTime);
        return;
    }

    // Typing into a field (e.g. an exact slider value) must not fly the camera.
    if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantTextInput)
    {
        updateCursorCapture();
        sampleFighterControls(deltaTime);
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
    // C frames the engagement, except in mouse aim where holding it is free look.
    if (pressed(GLFW_KEY_C) && !mouseAimActive())
    {
        setCameraMode(CameraMode::FREE, true);
    }

    updateCursorCapture();
    if (mouseAimActive())
    {
        m_freeLookHeld = m_freeLookMouseHeld || glfwGetKey(m_window, GLFW_KEY_C) == GLFW_PRESS;
        m_aimCamera.setFreeLook(m_freeLookHeld);
        const float radiansPerPixel = glm::radians(m_mouseAimSensitivity);
        const float pitchSign = m_invertMouseY ? 1.0f : -1.0f; // screen y grows downward
        m_aimCamera.turn(m_pendingMouseDelta.x * radiansPerPixel, pitchSign * m_pendingMouseDelta.y * radiansPerPixel);
    }
    else
    {
        m_freeLookHeld = false;
        m_freeLookMouseHeld = false;
        m_aimCamera.setFreeLook(false);
    }
    m_pendingMouseDelta = glm::vec2(0.0f);

    sampleFighterControls(deltaTime);

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
        if (!m_seekerCueEnabled && !m_missileInFlight && m_missile && !m_missile->isFox2())
        {
            m_missile->clearTarget();
        }
    }

    updatePreLaunchSeekerLock();

    const bool flyingFighter = m_playerRole == PlayerRole::Fighter && m_cameraMode == CameraMode::FIGHTER_JET;
    if (pressed(GLFW_KEY_X) && flyingFighter && m_fighter)
    {
        m_fighter->toggleAfterburner();
    }
    if (pressed(GLFW_KEY_G) && flyingFighter)
    {
        rearmFighter();
    }

    if (pressed(GLFW_KEY_F))
    {
        if (m_playerRole != PlayerRole::Fighter || flyingFighter)
        {
            launchMissile();
        }
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
    const float mouseX = static_cast<float>(xpos);
    const float mouseY = static_cast<float>(ypos);
    // Only a captured (hidden) cursor steers anything; a visible one belongs
    // to the interface.
    if (!m_cursorCaptured || !gameplayInputEnabled())
    {
        m_firstMouse = true;
        return;
    }
    if (m_firstMouse)
    {
        m_lastMouseX = mouseX;
        m_lastMouseY = mouseY;
        m_firstMouse = false;
        return;
    }

    const float dx = mouseX - m_lastMouseX;
    const float dy = mouseY - m_lastMouseY;
    m_lastMouseX = mouseX;
    m_lastMouseY = mouseY;

    if (mouseAimActive())
    {
        // Consumed once per frame by processInput.
        m_pendingMouseDelta += glm::vec2(dx, dy);
        return;
    }
    if (!m_enableMouseCamera)
    {
        return;
    }

    constexpr float kLookDegreesPerPixel = 0.1f;
    if (m_cameraMode == CameraMode::FREE)
    {
        m_renderer->rotateCameraYaw(dx * kLookDegreesPerPixel);
        m_renderer->rotateCameraPitch(-dy * kLookDegreesPerPixel);
        return;
    }
    m_chaseCamera.orbit(glm::radians(dx * kLookDegreesPerPixel), glm::radians(-dy * kLookDegreesPerPixel));
}

void Application::mouseButtonCallback(int button, int action)
{
    if (button != GLFW_MOUSE_BUTTON_RIGHT)
    {
        return;
    }

    if (action == GLFW_PRESS && gameplayInputEnabled() && !ImGui::GetIO().WantCaptureMouse)
    {
        if (mouseAimActive())
        {
            // Mouse aim: right button holds free look. The aim freezes, so the
            // jet keeps flying where it was pointed while the mouse looks round.
            m_freeLookMouseHeld = true;
            return;
        }
        // Free camera: look around. Chase cameras: orbit the subject.
        m_enableMouseCamera = true;
        m_chaseCamera.setOrbiting(true);
        updateCursorCapture();
    }
    else if (action == GLFW_RELEASE)
    {
        m_freeLookMouseHeld = false;
        if (m_enableMouseCamera)
        {
            m_enableMouseCamera = false;
            m_chaseCamera.setOrbiting(false);
            updateCursorCapture();
        }
    }
}

void Application::scrollCallback(double yoffset)
{
    if (!mouseAimActive() || ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }
    // Wheel pulls the chase camera in or out.
    const float scale = m_aimCamera.distanceScale() * std::pow(0.9f, static_cast<float>(yoffset));
    m_aimCamera.setDistanceScale(std::clamp(scale, 0.6f, 2.5f));
}
