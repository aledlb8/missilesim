#include "Application.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <glm/gtx/norm.hpp>

#include "audio/AudioSystem.h"
#include "objects/Fighter.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "sim/Fox2Catalog.h"

namespace
{
    // Wingtip launch rails, in multiples of the fighter's draw radius (the
    // renderer scales assets/models/jet.obj to it, see buildObjectModelMatrix).
    // Measured from that mesh: the tips end 0.75 radii out, the wing sits 0.14
    // below the model centre and the tip trailing edge is 0.57 aft. The round
    // hangs just outboard of and under the tip, tail at the trailing edge.
    constexpr float kRailOutboard = 0.762f;
    constexpr float kRailBelow = 0.16f;
    constexpr float kRailAft = 0.36f;

    glm::vec3 unitOr(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        if (glm::length2(vector) > 1.0e-8f)
        {
            return glm::normalize(vector);
        }
        return fallback;
    }
}

void Application::setPlayerRole(PlayerRole role)
{
    if (role == m_playerRole)
    {
        return;
    }

    m_playerRole = role;
    if (role == PlayerRole::Sam)
    {
        m_fox2Id = "custom";
        if (!m_missileInFlight && !m_detonationHoldActive)
        {
            resetMissile();
        }
        return;
    }

    if (missilesim::fox2::find(m_fox2Id.c_str()) == nullptr)
    {
        m_fox2Id = "aim-9x-blk2";
    }
    m_fox2Rounds = 2;
    m_railSign = 1;
    placeFighterAtEngagement();
    setCameraMode(CameraMode::FIGHTER_JET);
    resetAimCamera();
    if (!m_missileInFlight && !m_detonationHoldActive)
    {
        resetMissile();
    }
}

void Application::selectFox2(const char *id)
{
    if (m_playerRole != PlayerRole::Fighter || id == nullptr)
    {
        return;
    }
    if (missilesim::fox2::find(id) == nullptr)
    {
        return;
    }

    m_fox2Id = id;
    if (!m_missileInFlight && !m_detonationHoldActive)
    {
        resetMissile();
    }
}

void Application::placeFighterAtEngagement()
{
    // Spawn at the origin at the first target's altitude, pointed at it, at
    // a medium-altitude cruise speed (a scenario choice, not a published
    // employment figure; see research_notes flight-model.md).
    float altitude = 1500.0f;
    glm::vec3 heading(0.0f, 0.0f, 1.0f);
    for (const auto &target : m_targets)
    {
        if (target && target->isActive())
        {
            altitude = target->getPosition().y;
            const glm::vec3 flat(target->getPosition().x, 0.0f, target->getPosition().z);
            heading = unitOr(flat, heading);
            break;
        }
    }
    altitude = std::max(altitude, 600.0f);

    if (!m_fighter)
    {
        m_fighter = std::make_unique<Fighter>();
    }
    m_fighter->place(glm::vec3(0.0f, altitude, 0.0f), heading * 250.0f, heading);
}

void Application::sampleFighterControls(float deltaTime)
{
    if (!m_fighter)
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
            m_fighter->adjustThrottle(std::clamp(throttleKey, -1.0f, 1.0f) * kThrottleRatePerSecond * std::max(deltaTime, 0.0f));
        }
    }
    m_fighter->setInstructorInput(input);
}

void Application::handleFighterCrash()
{
    if (!m_fighter)
    {
        return;
    }
    const glm::vec3 impact = m_fighter->getPosition();
    createExplosion(impact);
    placeFighterAtEngagement();
    resetAimCamera();
}

void Application::beginFixedStepForAll()
{
    if (m_fighter)
    {
        m_fighter->beginFixedStep();
    }
    if (m_missile)
    {
        m_missile->beginFixedStep();
    }
    for (const auto &target : m_targets)
    {
        if (target)
        {
            target->beginFixedStep();
        }
    }
    for (const auto &flare : m_flares)
    {
        if (flare)
        {
            flare->beginFixedStep();
        }
    }
}

void Application::setRenderBlendForAll(float alpha)
{
    if (m_fighter)
    {
        m_fighter->setRenderBlend(alpha);
    }
    if (m_missile)
    {
        m_missile->setRenderBlend(alpha);
    }
    for (const auto &target : m_targets)
    {
        if (target)
        {
            target->setRenderBlend(alpha);
        }
    }
    for (const auto &flare : m_flares)
    {
        if (flare)
        {
            flare->setRenderBlend(alpha);
        }
    }
}

void Application::updateFighter(float deltaTime)
{
    if (m_playerRole != PlayerRole::Fighter || !m_fighter || !m_physicsEngine)
    {
        return;
    }

    const Atmosphere::State air = m_physicsEngine->getAtmosphereState(m_fighter->getPosition().y);
    const float gravity = std::max(m_physicsEngine->getGravity(), 0.0f);
    m_fighter->updateFlight(deltaTime, air.densityKgPerCubicMeter, air.speedOfSoundMetersPerSecond, gravity);

    // No gear, no runway: touching the ground is a crash.
    const float ground = m_physicsEngine->getGroundLevel();
    const glm::vec3 position = m_fighter->getPosition();
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || position.y <= ground + 1.5f)
    {
        handleFighterCrash();
    }
}

void Application::stageFox2OnRail()
{
    if (m_playerRole != PlayerRole::Fighter || m_missileInFlight || !m_missile || !m_fighter)
    {
        return;
    }
    if (!m_missile->isFox2() || m_fox2Rounds <= 0)
    {
        return;
    }

    if (m_physicsEngine)
    {
        m_physicsEngine->removeObject(m_missile.get());
    }

    const glm::vec3 nose = m_fighter->getNose();
    const glm::vec3 right = m_fighter->getRight();
    const glm::vec3 up = m_fighter->getUp();
    const float sign = m_railSign >= 0 ? 1.0f : -1.0f;
    const float scale = std::max(m_fighter->getRadius(), 1.0f);
    const glm::vec3 position = m_fighter->getPosition() +
                               scale * (right * (sign * kRailOutboard) - up * kRailBelow - nose * kRailAft);
    m_missile->setPosition(position);
    m_missile->setVelocity(m_fighter->getVelocity());
    m_missile->setBodyForward(nose);
    m_missile->setThrustDirection(nose);
    m_missile->setThrustEnabled(false);
    m_missile->setThrottle(1.0f);
}

void Application::refreshFox2Prelaunch()
{
    if (m_playerRole != PlayerRole::Fighter || m_missileInFlight || !m_missile || !m_missile->isFox2() || !m_fighter)
    {
        return;
    }

    if (!m_seekerCueEnabled)
    {
        m_missile->clearTarget();
        m_missile->clearFox2Lock();
        return;
    }

    std::vector<Target *> targets;
    targets.reserve(m_targets.size());
    for (const auto &target : m_targets)
    {
        if (target && target->isActive())
        {
            targets.push_back(target.get());
        }
    }
    m_missile->updateFox2Prelaunch(targets, m_fighter->getNose(), m_fighter->getPosition());
}

void Application::launchFox2FromRail()
{
    if (m_missileInFlight || m_detonationHoldActive)
    {
        return;
    }
    if (!m_fighter || m_fox2Rounds <= 0)
    {
        std::cout << "Launch blocked: wingtip rails are empty" << std::endl;
        return;
    }
    if (!m_missile)
    {
        resetMissile();
    }

    stageFox2OnRail();
    refreshFox2Prelaunch();
    if (!m_missile || !m_missile->isFox2() || m_missile->fox2Spec() == nullptr)
    {
        return;
    }

    const missilesim::fox2::Spec &spec = *m_missile->fox2Spec();
    Target *designated = getTrackedMissileTarget();
    const bool lockAfterLaunch = spec.homing == missilesim::fox2::LaunchHoming::LockAfterLaunch;
    if (designated == nullptr)
    {
        std::cout << "Launch blocked: seeker has no designation" << std::endl;
        return;
    }
    if (!lockAfterLaunch && !m_missile->hasFox2InfraredLock())
    {
        std::cout << "Launch blocked: this round needs an infrared lock before launch" << std::endl;
        return;
    }

    const glm::vec3 nose = m_fighter->getNose();
    const glm::vec3 velocity = m_fighter->getVelocity();
    const bool infrared = m_missile->hasFox2InfraredLock();
    m_missile->setVelocity(velocity);
    m_missile->beginFox2Flight(nose, infrared);
    if (m_physicsEngine)
    {
        m_physicsEngine->addObject(m_missile.get());
    }

    const glm::vec3 launchPosition = m_missile->getPosition();
    const float flash = m_missile->isReducedSmoke() ? 0.35f : 1.0f;
    if (m_renderer)
    {
        m_renderer->spawnMissileLaunchEffect(launchPosition, nose, velocity, flash);
    }
    if (m_audioSystem)
    {
        m_audioSystem->playLaunch(launchPosition);
    }

    m_fox2Rounds = std::max(0, m_fox2Rounds - 1);
    m_railSign = -m_railSign;
    m_seekerCueEnabled = false;
    m_missileInFlight = true;
    m_missileFlightTime = 0.0f;
    m_closestTargetDistance = 1000000.0f;
    invalidateTrajectoryPreviewCache();
    std::cout << "Rail launch " << spec.displayName << " at fighter speed, motor " << m_missile->getThrust() << " N" << std::endl;
}

void Application::rearmFighter()
{
    if (m_playerRole != PlayerRole::Fighter)
    {
        resetMissile();
        return;
    }

    m_fox2Rounds = 2;
    m_railSign = 1;
    if (!m_fighter)
    {
        placeFighterAtEngagement();
    }
    resetMissile();
}

void Application::renderFighter()
{
    if (m_playerRole != PlayerRole::Fighter || !m_fighter || !m_renderer)
    {
        return;
    }
    m_renderer->render(m_fighter.get());
}

void Application::emitFighterVisuals()
{
    if (m_playerRole != PlayerRole::Fighter || !m_fighter || !m_renderer)
    {
        return;
    }

    const float throttle = glm::clamp(m_fighter->getThrottle(), 0.0f, 1.0f);
    const float intensity = m_fighter->isAfterburner()
                                ? glm::clamp(0.55f + 0.45f * throttle, 0.55f, 1.0f)
                                : glm::clamp(0.16f + 0.32f * throttle, 0.12f, 0.48f);
    for (const auto &socket : m_renderer->getExhaustSockets(*m_fighter))
    {
        const glm::vec3 previous = socket.position + m_fighter->getPreviousRenderPosition() - m_fighter->getRenderPosition();
        m_renderer->emitJetAfterburner(previous, socket.position, -socket.direction, m_fighter->getVelocity(), intensity);
    }
}

float Application::fox2SeekerCueRadiusPixels() const
{
    const float cap = 0.46f * static_cast<float>(std::min(std::max(m_width, 1), std::max(m_height, 1)));
    if (!m_missile || !m_missile->isFox2() || m_missile->fox2Spec() == nullptr || m_renderer == nullptr)
    {
        return m_seekerCueRadiusPixels;
    }

    const missilesim::fox2::Spec &spec = *m_missile->fox2Spec();
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
