#include "Application.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>

#include "objects/Fighter.h"
#include "objects/Missile.h"
#include "rendering/Renderer.h"
#include "sim/Fox2Catalog.h"

void Application::sampleFighterControls(float deltaTime)
{
    Fighter *jet = fighter();
    if (!jet)
    {
        return;
    }

    missilesim::flight::InstructorInput input;
    input.aimDirection = m_aimCamera.aimDirection();

    const bool typing = ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantTextInput;
    const bool flying = m_playerRole == PlayerRole::Fighter && m_cameraMode == CameraMode::FIGHTER_JET &&
                        gameplayInputEnabled() && !typing && m_window != nullptr;
    if (flying)
    {
        const auto axis = [&](int negative, int positive) {
            float value = 0.0f;
            value += glfwGetKey(m_window, positive) == GLFW_PRESS ? 1.0f : 0.0f;
            value -= glfwGetKey(m_window, negative) == GLFW_PRESS ? 1.0f : 0.0f;
            return value;
        };
        // Keyboard assists (stick convention: W pushes the nose down).
        input.pitchKey = axis(GLFW_KEY_W, GLFW_KEY_S);
        input.rollKey = axis(GLFW_KEY_A, GLFW_KEY_D);
        input.yawKey = axis(GLFW_KEY_Q, GLFW_KEY_E);

        const float throttleKey = axis(GLFW_KEY_LEFT_CONTROL, GLFW_KEY_LEFT_SHIFT) + axis(GLFW_KEY_RIGHT_CONTROL, GLFW_KEY_RIGHT_SHIFT);
        if (throttleKey != 0.0f)
        {
            constexpr float kThrottleRatePerSecond = 0.5f;
            jet->adjustThrottle(std::clamp(throttleKey, -1.0f, 1.0f) * kThrottleRatePerSecond * std::max(deltaTime, 0.0f));
        }
    }
    // Aim velocity belongs to the input sampling clock, not the fixed-step
    // clock. Rebase while paused or outside mouse aim to avoid a resume kick.
    const float sampleTime = flying && mouseAimActive() && !m_isPaused ? deltaTime * m_simulationSpeed : 0.0f;
    jet->setInstructorInput(input, sampleTime);
}

void Application::renderFighter()
{
    Fighter *jet = fighter();
    if (m_playerRole != PlayerRole::Fighter || !jet || !m_renderer)
    {
        return;
    }
    m_renderer->render(jet);
}

void Application::emitFighterVisuals()
{
    Fighter *jet = fighter();
    if (m_playerRole != PlayerRole::Fighter || !jet || !m_renderer)
    {
        return;
    }

    const float throttle = glm::clamp(jet->getThrottle(), 0.0f, 1.0f);
    const float intensity = jet->isAfterburner()
                                ? glm::clamp(0.55f + 0.45f * throttle, 0.55f, 1.0f)
                                : glm::clamp(0.16f + 0.32f * throttle, 0.12f, 0.48f);
    for (const auto &socket : m_renderer->getExhaustSockets(*jet))
    {
        const glm::vec3 previous = socket.position + jet->getPreviousRenderPosition() - jet->getRenderPosition();
        m_renderer->emitJetAfterburner(previous, socket.position, -socket.direction, jet->getVelocity(), intensity);
    }
}

float Application::fox2SeekerCueRadiusPixels() const
{
    const float cap = 0.46f * static_cast<float>(std::min(std::max(m_width, 1), std::max(m_height, 1)));
    const Missile *round = m_world ? m_world->readyRound() : nullptr;
    if (round == nullptr || !round->isFox2() || round->fox2Spec() == nullptr || m_renderer == nullptr)
    {
        return m_seekerCueRadiusPixels;
    }

    const missilesim::fox2::Spec &spec = *round->fox2Spec();
    if (spec.rearHemisphereDesignation)
    {
        return cap;
    }

    float angle = spec.gimbalDeg;
    if (spec.homing == missilesim::fox2::LaunchHoming::LockAfterLaunch && spec.cueDeg > 0.0f)
    {
        angle = spec.cueDeg;
    }
    angle = std::min(angle, 89.0f);
    const float halfFov = glm::radians(m_renderer->getCameraFOV() * 0.5f);
    const float tangent = std::tan(halfFov);
    if (tangent <= 1.0e-4f)
    {
        return m_seekerCueRadiusPixels;
    }
    const float radius = std::tan(glm::radians(angle)) / tangent * (static_cast<float>(std::max(m_height, 1)) * 0.5f);
    return std::min(std::max(radius, 8.0f), cap);
}
