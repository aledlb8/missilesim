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

void Application::launchMissile()
{
    try
    {
        if (m_missileInFlight)
        {
            return;
        }

        if (!m_missile)
        {
            resetMissile();
        }

        if (!m_missile)
        {
            std::cerr << "ERROR: Missile is null in launchMissile()" << std::endl;
            return;
        }

        Target *lockedTarget = getTrackedMissileTarget();
        if (!lockedTarget)
        {
            std::cout << "Launch blocked: seeker has no target lock" << std::endl;
            return;
        }

        const glm::vec3 stagedVelocity(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]);
        const glm::vec3 cameraForward = m_renderer
                                            ? safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f))
                                            : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 thrustDirection = safeNormalize(cameraForward, safeNormalize(stagedVelocity, glm::vec3(0.0f, 0.0f, 1.0f)));

        // Set thrust parameters and preserve the pre-launch lock.
        m_missile->setGuidanceEnabled(m_guidanceEnabled);
        m_missile->setThrust(m_missileThrust);
        m_missile->setThrustDirection(thrustDirection);
        m_missile->setFuel(m_missileFuel);
        m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);

        // Enable thrust engine
        m_missile->setThrustEnabled(true);

        // Fire in the player's aim direction after a valid seeker lock.
        float initialSpeed = glm::length(stagedVelocity);
        if (initialSpeed < 0.1f)
        {
            initialSpeed = std::max(m_missileThrust / 50.0f, 40.0f);
        }
        m_missile->setVelocity(thrustDirection * initialSpeed);

        if (m_audioSystem)
        {
            m_audioSystem->playLaunch(m_missile->getPosition(), m_missile->getVelocity());
        }

        // Log launch
        std::cout << "Missile launched with thrust: " << m_missileThrust
                  << " N, fuel: " << m_missileFuel << " kg, target lock retained" << std::endl;

        m_seekerCueEnabled = false;
        m_missileInFlight = true;
        m_missileFlightTime = 0.0f;
        m_closestTargetDistance = 1000000.0f;
        invalidateTrajectoryPreviewCache();
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in launchMissile: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in launchMissile" << std::endl;
    }
}

void Application::resetMissile()
{
    try
    {
        m_missileInFlight = false;
        m_missileFlightTime = 0.0f;
        m_closestTargetDistance = 1000000.0f;
        m_explosions.clear();
        if (m_renderer)
        {
            m_renderer->clearEffects();
        }
        if (m_audioSystem)
        {
            m_audioSystem->stopAllEmitters();
        }
        invalidateTrajectoryPreviewCache();

        // First, if there's an existing missile, remove it from physics engine
        if (m_missile)
        {
            m_physicsEngine->removeObject(m_missile.get());
        }

        // Validate initial parameters to prevent crashes
        for (int i = 0; i < 3; i++)
        {
            // Check for NaN or inf in initial position and velocity
            if (std::isnan(m_initialPosition[i]) || std::isinf(m_initialPosition[i]))
            {
                m_initialPosition[i] = 0.0f;
            }
            if (std::isnan(m_initialVelocity[i]) || std::isinf(m_initialVelocity[i]))
            {
                m_initialVelocity[i] = 0.0f;
            }
        }

        // Ensure mass is valid
        if (m_mass <= 0.0f || std::isnan(m_mass) || std::isinf(m_mass))
        {
            m_mass = 100.0f; // Default safe value
        }

        // Ensure drag coefficient is valid
        if (m_dragCoefficient < 0.0f || std::isnan(m_dragCoefficient) || std::isinf(m_dragCoefficient))
        {
            m_dragCoefficient = 0.1f; // Default safe value
        }

        // Ensure cross-sectional area is valid
        if (m_crossSectionalArea <= 0.0f || std::isnan(m_crossSectionalArea) || std::isinf(m_crossSectionalArea))
        {
            m_crossSectionalArea = 0.1f; // Default safe value
        }

        // Ensure lift coefficient is valid
        if (m_liftCoefficient < 0.0f || std::isnan(m_liftCoefficient) || std::isinf(m_liftCoefficient))
        {
            m_liftCoefficient = 0.1f; // Default safe value
        }

        // Create missile with validated parameters
        m_missile = std::make_unique<Missile>(
            glm::vec3(m_initialPosition[0], m_initialPosition[1], m_initialPosition[2]),
            glm::vec3(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]),
            m_mass, m_dragCoefficient, m_crossSectionalArea, m_liftCoefficient);

        // Validate guidance parameters
        if (std::isnan(m_navigationGain) || std::isinf(m_navigationGain))
        {
            m_navigationGain = 4.0f;
        }
        m_navigationGain = std::clamp(m_navigationGain, 1.0f, 4.0f);

        if (m_maxSteeringForce <= 0.0f || std::isnan(m_maxSteeringForce) || std::isinf(m_maxSteeringForce))
        {
            m_maxSteeringForce = 20000.0f;
        }

        if (std::isnan(m_trackingAngle) || std::isinf(m_trackingAngle))
        {
            m_trackingAngle = 85.0f;
        }
        m_trackingAngle = std::clamp(m_trackingAngle, 5.0f, 180.0f);

        if (std::isnan(m_proximityFuseRadius) || std::isinf(m_proximityFuseRadius) || m_proximityFuseRadius < 0.0f)
        {
            m_proximityFuseRadius = 18.0f;
        }

        if (std::isnan(m_countermeasureResistance) || std::isinf(m_countermeasureResistance))
        {
            m_countermeasureResistance = 0.65f;
        }
        m_countermeasureResistance = glm::clamp(m_countermeasureResistance, 0.0f, 1.0f);

        if (std::isnan(m_terrainClearance) || std::isinf(m_terrainClearance) || m_terrainClearance < 0.0f)
        {
            m_terrainClearance = 90.0f;
        }

        if (std::isnan(m_terrainLookAheadTime) || std::isinf(m_terrainLookAheadTime) || m_terrainLookAheadTime < 0.5f)
        {
            m_terrainLookAheadTime = 6.0f;
        }

        // Set guidance parameters
        m_missile->setGuidanceEnabled(m_guidanceEnabled);
        m_missile->setNavigationGain(m_navigationGain);
        m_missile->setMaxSteeringForce(m_maxSteeringForce);
        m_missile->setTrackingAngle(m_trackingAngle);
        m_missile->setProximityFuseRadius(m_proximityFuseRadius);
        m_missile->setCountermeasureResistance(m_countermeasureResistance);
        m_missile->setTerrainAvoidanceEnabled(m_terrainAvoidanceEnabled);
        m_missile->setTerrainClearance(m_terrainClearance);
        m_missile->setTerrainLookAheadTime(m_terrainLookAheadTime);
        m_missile->setGroundReferenceAltitude(m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f);

        // Set thrust parameters but disable thrust until launch
        m_missile->setThrust(m_missileThrust);
        m_missile->setThrustEnabled(false);
        m_missile->setFuel(m_missileFuel);
        m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);

        // Add missile to physics engine
        m_physicsEngine->addObject(m_missile.get());
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in resetMissile: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in resetMissile" << std::endl;
    }
}

Target *Application::findBestTarget()
{
    // Find the first active target
    for (const auto &target : m_targets)
    {
        if (target->isActive())
        {
            return target.get();
        }
    }

    // No active targets found
    return nullptr;
}

bool Application::projectTargetToSeekerScreen(const Target *target, ImVec2 &screenPosition, float *pixelDistanceFromCenter) const
{
    if (!m_renderer || !target || !target->isActive())
    {
        return false;
    }

    const int safeHeight = std::max(m_height, 1);
    const float aspectRatio = static_cast<float>(m_width) / static_cast<float>(safeHeight);
    const float tanHalfFovY = std::tan(glm::radians(m_renderer->getCameraFOV() * 0.5f));
    if (tanHalfFovY <= 0.0f)
    {
        return false;
    }

    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    const glm::vec3 cameraForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 cameraRight = safeNormalize(m_renderer->getCameraRight(), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 cameraUp = safeNormalize(m_renderer->getCameraUp(), glm::vec3(0.0f, 1.0f, 0.0f));

    const glm::vec3 aimPoint = target->getPosition() + glm::vec3(0.0f, target->getRadius() * 0.3f, 0.0f);
    const glm::vec3 toTarget = aimPoint - cameraPosition;
    const float forwardDepth = glm::dot(toTarget, cameraForward);
    if (forwardDepth <= 0.1f)
    {
        return false;
    }

    const float ndcX = glm::dot(toTarget, cameraRight) / (forwardDepth * tanHalfFovY * aspectRatio);
    const float ndcY = glm::dot(toTarget, cameraUp) / (forwardDepth * tanHalfFovY);
    if (std::abs(ndcX) > 1.0f || std::abs(ndcY) > 1.0f)
    {
        return false;
    }

    screenPosition.x = (ndcX + 1.0f) * 0.5f * m_width;
    screenPosition.y = (1.0f - ((ndcY + 1.0f) * 0.5f)) * safeHeight;

    if (pixelDistanceFromCenter != nullptr)
    {
        const float offsetX = screenPosition.x - (m_width * 0.5f);
        const float offsetY = screenPosition.y - (safeHeight * 0.5f);
        *pixelDistanceFromCenter = std::sqrt((offsetX * offsetX) + (offsetY * offsetY));
    }

    return true;
}

Target *Application::findSeekerCueTarget() const
{
    if (!m_renderer || !m_missile || m_missileInFlight || !m_seekerCueEnabled || !m_guidanceEnabled)
    {
        return nullptr;
    }

    Target *lockedTarget = getTrackedMissileTarget();
    ImVec2 lockedScreenPosition(0.0f, 0.0f);
    if (lockedTarget != nullptr && projectTargetToSeekerScreen(lockedTarget, lockedScreenPosition, nullptr))
    {
        return lockedTarget;
    }

    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    Target *bestTarget = nullptr;
    float bestPixelDistance = std::numeric_limits<float>::max();
    float bestRange = std::numeric_limits<float>::max();

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        ImVec2 screenPosition(0.0f, 0.0f);
        float pixelDistance = 0.0f;
        if (!projectTargetToSeekerScreen(target.get(), screenPosition, &pixelDistance))
        {
            continue;
        }

        if (pixelDistance > m_seekerCueRadiusPixels)
        {
            continue;
        }

        const float targetRange = glm::length(target->getPosition() - cameraPosition);
        if (pixelDistance < bestPixelDistance || (std::abs(pixelDistance - bestPixelDistance) < 0.5f && targetRange < bestRange))
        {
            bestTarget = target.get();
            bestPixelDistance = pixelDistance;
            bestRange = targetRange;
        }
    }

    return bestTarget;
}

Target *Application::getTrackedMissileTarget() const
{
    if (!m_missile || !m_missile->hasTarget())
    {
        return nullptr;
    }

    Target *trackedTarget = m_missile->getTargetObject();
    if (trackedTarget == nullptr)
    {
        return nullptr;
    }

    for (const auto &target : m_targets)
    {
        if (target.get() == trackedTarget)
        {
            return target->isActive() ? target.get() : nullptr;
        }
    }

    return nullptr;
}

const char *Application::getMissileSeekerStateLabel() const
{
    if (!m_missile || !m_missile->isGuidanceEnabled())
    {
        return "Disabled";
    }

    if (!m_missileInFlight && !m_seekerCueEnabled)
    {
        return "Standby";
    }

    if (!m_missile->hasTarget())
    {
        return "Searching";
    }

    return m_missile->isTrackingDecoy() ? "Tracking flare" : "Tracking airframe";
}

const char *Application::getMissileSeekerTrackLabel() const
{
    if (!m_missile || !m_missile->isGuidanceEnabled())
    {
        return "DISABLED";
    }

    if (!m_missileInFlight && !m_seekerCueEnabled)
    {
        return "STBY";
    }

    if (!m_missile->hasTarget())
    {
        return "SEARCH";
    }

    return m_missile->isTrackingDecoy() ? "FLARE" : "AIRFRAME";
}

void Application::updatePreLaunchSeekerLock()
{
    if (!m_missile || m_missileInFlight)
    {
        return;
    }

    if (!m_seekerCueEnabled || !m_guidanceEnabled)
    {
        m_missile->clearTarget();
        return;
    }

    Target *cueTarget = findSeekerCueTarget();
    if (cueTarget)
    {
        m_missile->setTargetObject(cueTarget);
    }
    else
    {
        m_missile->clearTarget();
    }
}

void Application::renderPreLaunchSeekerCue() const
{
    if (!m_seekerCueEnabled || !m_renderer || !m_missile || m_missileInFlight || ImGui::GetCurrentContext() == nullptr)
    {
        return;
    }

    ImVec2 cueCenter(m_width * 0.5f, m_height * 0.5f);
    bool hasLock = false;
    if (Target *trackedTarget = getTrackedMissileTarget())
    {
        hasLock = projectTargetToSeekerScreen(trackedTarget, cueCenter, nullptr);
    }

    const ImU32 ringColor = hasLock ? IM_COL32(255, 76, 76, 255) : IM_COL32(255, 255, 255, 240);
    ImDrawList *drawList = ImGui::GetForegroundDrawList();
    drawList->AddCircle(cueCenter, m_seekerCueRadiusPixels, ringColor, 64, 2.2f);
    drawList->AddLine(ImVec2(cueCenter.x - 7.0f, cueCenter.y), ImVec2(cueCenter.x + 7.0f, cueCenter.y), ringColor, 1.2f);
    drawList->AddLine(ImVec2(cueCenter.x, cueCenter.y - 7.0f), ImVec2(cueCenter.x, cueCenter.y + 7.0f), ringColor, 1.2f);
}

void Application::terminateMissileFlight(const glm::vec3 &position, bool createEffect)
{
    invalidateTrajectoryPreviewCache();
    if (createEffect)
    {
        createExplosion(position);
    }

    if (m_missile)
    {
        m_missile->setThrustEnabled(false);
        m_missile->setGuidanceEnabled(false);
        m_missile->clearTarget();
        m_missile->setVelocity(glm::vec3(0.0f));
    }

    resetMissile();
}