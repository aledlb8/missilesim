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

void Application::createExplosion(const glm::vec3 &position)
{
    const glm::vec3 velocityHint = m_missile ? m_missile->getVelocity() : glm::vec3(0.0f);

    if (m_renderer)
    {
        m_renderer->spawnExplosionEffect(position, velocityHint, 1.0f);
    }

    if (m_audioSystem)
    {
        m_audioSystem->playExplosion(position, velocityHint, 1.0f);
    }
}

void Application::updateExplosions(float deltaTime)
{
    (void)deltaTime;
}

void Application::renderExplosions()
{
}

void Application::updateAudioFrame(float deltaTime)
{
    if (!m_audioSystem || !m_renderer)
    {
        return;
    }

    const float dt = (deltaTime > 0.0f && std::isfinite(deltaTime)) ? deltaTime : 0.016f;
    std::vector<Target *> activeTargets;
    activeTargets.reserve(m_targets.size());
    for (const auto &target : m_targets)
    {
        if (target && target->isActive())
        {
            activeTargets.push_back(target.get());
        }
    }

    std::vector<Flare *> activeFlares;
    activeFlares.reserve(m_flares.size());
    for (const auto &flare : m_flares)
    {
        if (flare && flare->isActive())
        {
            activeFlares.push_back(flare.get());
        }
    }

    const bool seekerPowered = !m_missileInFlight &&
                               m_missile != nullptr &&
                               m_guidanceEnabled &&
                               m_missile->isGuidanceEnabled() &&
                               m_seekerCueEnabled;
    Target *lockedTarget = seekerPowered ? getTrackedMissileTarget() : nullptr;
    const bool seekerLocked = seekerPowered && lockedTarget != nullptr;

    float seekerSignalStrength = 0.0f;
    if (seekerPowered)
    {
        const float searchCueRadiusPixels = std::max(m_seekerCueRadiusPixels * 3.0f, m_seekerCueRadiusPixels + 1.0f);
        float bestPixelDistance = std::numeric_limits<float>::max();

        for (Target *target : activeTargets)
        {
            ImVec2 screenPosition(0.0f, 0.0f);
            float pixelDistance = 0.0f;
            if (!projectTargetToSeekerScreen(target, screenPosition, &pixelDistance))
            {
                continue;
            }

            if (pixelDistance > searchCueRadiusPixels)
            {
                continue;
            }

            bestPixelDistance = std::min(bestPixelDistance, pixelDistance);
        }

        if (bestPixelDistance < std::numeric_limits<float>::max())
        {
            const float normalizedSignal = 1.0f - glm::clamp(bestPixelDistance / searchCueRadiusPixels, 0.0f, 1.0f);
            seekerSignalStrength = normalizedSignal * normalizedSignal;
        }

        if (lockedTarget != nullptr)
        {
            ImVec2 lockScreenPosition(0.0f, 0.0f);
            float lockPixelDistance = 0.0f;
            if (projectTargetToSeekerScreen(lockedTarget, lockScreenPosition, &lockPixelDistance))
            {
                const float lockRadius = std::max(m_seekerCueRadiusPixels, 1.0f);
                const float lockSignal = 1.0f - glm::clamp(lockPixelDistance / lockRadius, 0.0f, 1.0f);
                seekerSignalStrength = std::max(seekerSignalStrength, 0.66f + (lockSignal * 0.34f));
            }
            else
            {
                seekerSignalStrength = std::max(seekerSignalStrength, 0.72f);
            }
        }
    }

    Target *cockpitTarget = nullptr;
    if (m_cameraMode == CameraMode::FIGHTER_JET)
    {
        cockpitTarget = getTrackedMissileTarget();
        if (cockpitTarget == nullptr)
        {
            cockpitTarget = findBestTarget();
        }
    }

    bool mawsThreatActive = false;
    float mawsUrgency = 0.0f;
    if (cockpitTarget != nullptr && cockpitTarget->isMissileWarningActive())
    {
        const MAWSConfig mawsDefaults{};
        const float threatWindow = std::max(mawsDefaults.reactionTimeWindow, 0.1f);
        const float cpaThreshold = std::max(mawsDefaults.closestApproachThreshold, 1.0f);
        const float detectionRange = std::max(mawsDefaults.detectionRange, 1.0f);
        const float tcaBlend = 1.0f - glm::clamp(cockpitTarget->getThreatTimeToClosestApproach() / threatWindow, 0.0f, 1.0f);
        const float cpaBlend = 1.0f - glm::clamp(cockpitTarget->getThreatClosestApproachDistance() / cpaThreshold, 0.0f, 1.0f);
        const float rangeBlend = 1.0f - glm::clamp(cockpitTarget->getThreatDistance() / detectionRange, 0.0f, 1.0f);

        mawsThreatActive = true;
        mawsUrgency = glm::clamp((tcaBlend * 0.55f) + (cpaBlend * 0.25f) + (rangeBlend * 0.20f), 0.0f, 1.0f);
    }

    m_audioSystem->setListener(m_renderer->getCameraPosition(),
                               m_renderer->getCameraFront(),
                               m_renderer->getCameraUp(),
                               dt);
    m_audioSystem->syncMissile(m_missile.get(), m_missileInFlight, m_missileFuel);
    m_audioSystem->syncTargets(activeTargets);
    m_audioSystem->syncFlares(activeFlares);
    m_audioSystem->syncCockpitCues(seekerPowered,
                                   seekerLocked,
                                   seekerSignalStrength,
                                   mawsThreatActive,
                                   mawsUrgency);
    m_audioSystem->update(dt);
}

void Application::emitFrameVisualEffects(float deltaTime)
{
    if (!m_renderer || m_isPaused)
    {
        return;
    }

    if (glm::clamp(deltaTime, 0.0f, 0.05f) <= 0.0f)
    {
        return;
    }

    if (m_missile && m_missile->isThrustEnabled() && m_missile->getFuel() > 0.0f)
    {
        const glm::vec3 missileForward = safeNormalize(m_missile->getVelocity(), m_missile->getThrustDirection());
        const glm::vec3 currentEmitter = m_missile->getPosition() - (missileForward * 1.12f);
        const glm::vec3 previousEmitter = m_missile->getPreviousPosition() - (missileForward * 1.12f);
        const float fuelFraction = (m_missileFuel > 0.0f)
                                       ? glm::clamp(m_missile->getFuel() / m_missileFuel, 0.0f, 1.0f)
                                       : 1.0f;
        const float plumeIntensity = glm::clamp(glm::mix(0.38f, 0.68f, fuelFraction), 0.36f, 0.74f);
        m_renderer->emitMissileExhaust(previousEmitter, currentEmitter, missileForward, m_missile->getVelocity(), plumeIntensity);
    }

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        const glm::vec3 forward = safeNormalize(target->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::vec3 right = safeNormalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::vec3 up = safeNormalize(glm::cross(right, forward), glm::vec3(0.0f, 1.0f, 0.0f));
        const float radius = std::max(target->getRadius(), 1.0f);
        const glm::vec3 engineOffset = (-forward * radius * 1.42f) - (up * radius * 0.05f);
        const glm::vec3 lateralOffset = right * radius * 0.26f;
        const glm::vec3 currentBase = target->getPosition() + engineOffset;
        const glm::vec3 previousBase = target->getPreviousPosition() + engineOffset;

        const TargetAIConfig &config = target->getAIConfig();
        const float speed = glm::length(target->getVelocity());
        const float speedBand = std::max(config.maxSpeed - config.minSpeed, 1.0f);
        const float speedFraction = glm::clamp((speed - config.minSpeed) / speedBand, 0.0f, 1.0f);
        const float afterburnerIntensity = glm::clamp(0.46f + (speedFraction * 0.26f) + (target->isMissileWarningActive() ? 0.08f : 0.0f),
                                                      0.40f, 0.82f);

        m_renderer->emitJetAfterburner(previousBase - lateralOffset,
                                       currentBase - lateralOffset,
                                       forward,
                                       target->getVelocity(),
                                       afterburnerIntensity);
        m_renderer->emitJetAfterburner(previousBase + lateralOffset,
                                       currentBase + lateralOffset,
                                       forward,
                                       target->getVelocity(),
                                       afterburnerIntensity);
    }

    for (const auto &flare : m_flares)
    {
        if (!flare || !flare->isActive())
        {
            continue;
        }

        const float heatFraction = (flare->getInitialHeatSignature() > 0.0f)
                                       ? glm::clamp(flare->getHeatSignature() / flare->getInitialHeatSignature(), 0.0f, 1.0f)
                                       : 0.0f;
        m_renderer->emitFlareEffect(flare->getPreviousPosition(),
                                    flare->getPosition(),
                                    flare->getVelocity(),
                                    heatFraction);
    }
}