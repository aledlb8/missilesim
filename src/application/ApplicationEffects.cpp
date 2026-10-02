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

namespace
{
    float urgencyBlend(float value, float limit)
    {
        if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0f)
        {
            return 0.0f;
        }
        return 1.0f - glm::clamp(value / limit, 0.0f, 1.0f);
    }

    float computeMawsUrgency(const Target &target)
    {
        const MAWSConfig &config = target.getMAWSConfig();
        if (!config.enabled)
        {
            return 0.0f;
        }

        const float threatWindow = std::max(config.reactionTimeWindow, 0.1f);
        const float closestApproachThreshold = std::max(config.closestApproachThreshold, 1.0f);
        const float detectionRange = std::max(config.detectionRange, 1.0f);

        const float tcaBlend = urgencyBlend(target.getThreatTimeToClosestApproach(), threatWindow);
        const float cpaBlend = urgencyBlend(target.getThreatClosestApproachDistance(), closestApproachThreshold);
        const float rangeBlend = urgencyBlend(target.getThreatDistance(), detectionRange);

        return glm::clamp((tcaBlend * 0.55f) + (cpaBlend * 0.25f) + (rangeBlend * 0.20f), 0.0f, 1.0f);
    }
}

void Application::createExplosion(const glm::vec3 &position, const glm::vec3 &velocityHint)
{
    if (m_renderer)
    {
        m_renderer->spawnExplosionEffect(position, velocityHint, 1.0f);
    }

    if (m_audioSystem)
    {
        m_audioSystem->playExplosion(position);
    }
}

void Application::updateAudioFrame(float deltaTime)
{
    if (!m_audioSystem || !m_renderer)
    {
        return;
    }

    const float dt = (deltaTime > 0.0f && std::isfinite(deltaTime)) ? deltaTime : 0.016f;
    std::vector<AudioTargetSource> activeTargets;
    activeTargets.reserve(targets().size());
    for (const auto &target : targets())
    {
        if (target->isActive())
        {
            activeTargets.push_back(AudioTargetSource{target->getEntityId().value, target.get()});
        }
    }

    std::vector<AudioFlareSource> activeFlares;
    activeFlares.reserve(flares().size());
    for (const auto &flare : flares())
    {
        if (flare->isActive())
        {
            activeFlares.push_back(AudioFlareSource{flare->getEntityId().value, flare.get()});
        }
    }

    std::vector<AudioMissileSource> shotsInFlight;
    if (m_world)
    {
        shotsInFlight.reserve(m_world->shots().size());
        for (const auto &shot : m_world->shots())
        {
            shotsInFlight.push_back(AudioMissileSource{shot->id.value, shot->missile.get()});
        }
    }

    // The seeker tone belongs to the uncaged round waiting to fire.
    const Missile *readyRound = m_world ? m_world->readyRound() : nullptr;
    const bool seekerPowered = readyRound != nullptr &&
                               m_guidanceEnabled &&
                               readyRound->isGuidanceEnabled() &&
                               seekerUncaged();
    Target *lockedTarget = seekerPowered ? readyRound->getTargetObject() : nullptr;
    if (lockedTarget != nullptr && !lockedTarget->isActive())
    {
        lockedTarget = nullptr;
    }
    const bool seekerLocked = seekerPowered && lockedTarget != nullptr;

    float seekerSignalStrength = 0.0f;
    if (seekerPowered)
    {
        const float searchCueRadiusPixels = std::max(m_seekerCueRadiusPixels * 3.0f, m_seekerCueRadiusPixels + 1.0f);
        float bestPixelDistance = std::numeric_limits<float>::max();

        for (const AudioTargetSource &source : activeTargets)
        {
            ImVec2 screenPosition(0.0f, 0.0f);
            float pixelDistance = 0.0f;
            if (!projectTargetToSeekerScreen(source.object, screenPosition, &pixelDistance))
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
    if (m_cameraMode == CameraMode::FIGHTER_JET && m_playerRole != PlayerRole::Fighter)
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
        mawsThreatActive = true;
        mawsUrgency = computeMawsUrgency(*cockpitTarget);
    }

    AudioWorldState world;
    world.paused = m_isPaused;
    world.timeScale = m_simulationSpeed;
    world.seaLevelAirDensity = physics() ? physics()->getAirDensity() : world.seaLevelAirDensity;
    world.groundPresent = m_groundEnabled;
    world.groundLevel = physics() ? physics()->getGroundLevel() : 0.0f;
    // Duck the mix behind the title screen so the flyby is atmosphere, not a
    // roar; ease back in when the engagement starts.
    const float menuGainTarget = (m_screen == Screen::Title) ? 0.45f : 1.0f;
    m_menuAudioGain += (menuGainTarget - m_menuAudioGain) * (1.0f - std::exp(-2.5f * std::max(deltaTime, 0.0f)));
    world.masterVolume = m_audioVolume * m_menuAudioGain;

    CockpitCueState cues;
    cues.seekerPowered = seekerPowered;
    cues.seekerLocked = seekerLocked;
    cues.seekerSignal = seekerSignalStrength;
    cues.missileWarning = mawsThreatActive;
    cues.missileWarningUrgency = mawsUrgency;

    m_audioSystem->beginFrame(world, dt);
    m_audioSystem->setListener(m_renderer->getCameraPosition(),
                               m_renderer->getCameraFront(),
                               m_renderer->getCameraUp());
    m_audioSystem->syncMissiles(shotsInFlight);
    m_audioSystem->syncTargets(activeTargets);
    m_audioSystem->syncFlares(activeFlares);
    m_audioSystem->syncCockpitCues(cues);
    m_audioSystem->endFrame();
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

    if (m_world)
    {
        for (const auto &shot : m_world->shots())
        {
            const Missile &missile = *shot->missile;
            if (!missile.isThrustEnabled() || missile.getFuel() <= 0.0f || missile.getThrottle() <= 0.01f)
            {
                continue;
            }
            const float fuelCapacity = missile.getFuelCapacity();
            const float fuelFraction = (fuelCapacity > 0.0f) ? glm::clamp(missile.getFuel() / fuelCapacity, 0.0f, 1.0f) : 1.0f;
            // The booster burns at elevated thrust; lay a noticeably thicker,
            // brighter plume for its duration so the climb-out reads as a hard boost.
            const bool boosting = shot->coldLaunch.motorIgnited && !shot->coldLaunch.boostComplete;
            const float boostPlume = boosting ? 0.34f : 0.0f;
            const float throttle = glm::clamp(missile.getThrottle(), 0.0f, 1.0f);
            const float smokeScale = missile.isReducedSmoke() ? 0.35f : 1.0f;
            const float plumeIntensity = glm::clamp((glm::mix(0.38f, 0.68f, fuelFraction) + boostPlume) * throttle * smokeScale,
                                                    0.08f,
                                                    1.0f);
            for (const auto &socket : m_renderer->getExhaustSockets(missile))
            {
                const glm::vec3 previous = socket.position + missile.getPreviousRenderPosition() - missile.getRenderPosition();
                m_renderer->emitMissileExhaust(previous, socket.position, -socket.direction, missile.getVelocity(), plumeIntensity);
            }
        }
    }

    emitFighterVisuals();

    for (const auto &target : targets())
    {
        if (!target->isActive())
        {
            continue;
        }

        const glm::vec3 forward = safeNormalize(target->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::vec3 right = safeNormalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::vec3 up = safeNormalize(glm::cross(right, forward), glm::vec3(0.0f, 1.0f, 0.0f));
        const float radius = std::max(target->getRadius(), 1.0f);
        const TargetAIConfig &config = target->getAIConfig();
        const float speed = glm::length(target->getVelocity());
        const float speedBand = std::max(config.maxSpeed - config.minSpeed, 1.0f);
        const float speedFraction = glm::clamp((speed - config.minSpeed) / speedBand, 0.0f, 1.0f);
        const float afterburnerIntensity = glm::clamp(target->getThrottle(), 0.0f, 1.0f);
        for (const auto &socket : m_renderer->getExhaustSockets(*target))
        {
            const glm::vec3 previous = socket.position + target->getPreviousRenderPosition() - target->getRenderPosition();
            m_renderer->emitJetAfterburner(previous, socket.position, -socket.direction,
                                           target->getVelocity(), afterburnerIntensity);
        }

        const float turnLoad = glm::length(target->getRenderAcceleration()) / 9.81f;
        const float maneuverVapor = glm::clamp((turnLoad - 1.2f) / 4.0f, 0.0f, 1.0f);
        const float wakeIntensity = glm::clamp((speedFraction * 0.28f) +
                                                   (maneuverVapor * 0.52f) +
                                                   (target->isMissileWarningActive() ? 0.16f : 0.0f),
                                               0.0f,
                                               0.85f);
        if (wakeIntensity > 0.04f)
        {
            const glm::vec3 wingOffset = (right * radius * 0.95f) + (up * radius * 0.03f) - (forward * radius * 0.08f);
            m_renderer->emitJetWake(target->getPreviousRenderPosition() - wingOffset,
                                    target->getRenderPosition() - wingOffset,
                                    forward,
                                    target->getVelocity(),
                                    wakeIntensity);
            m_renderer->emitJetWake(target->getPreviousRenderPosition() + wingOffset,
                                    target->getRenderPosition() + wingOffset,
                                    forward,
                                    target->getVelocity(),
                                    wakeIntensity);
        }
    }

    for (const auto &flare : flares())
    {
        if (!flare->isActive())
        {
            continue;
        }

        const float heatFraction = (flare->getInitialHeatSignature() > 0.0f)
                                       ? glm::clamp(flare->getHeatSignature() / flare->getInitialHeatSignature(), 0.0f, 1.0f)
                                       : 0.0f;
        m_renderer->emitFlareEffect(flare->getPreviousRenderPosition(),
                                    flare->getRenderPosition(),
                                    flare->getVelocity(),
                                    heatFraction);
    }
}
