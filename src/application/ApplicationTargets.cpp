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
    bool isFinite(float value)
    {
        return !std::isnan(value) && !std::isinf(value);
    }

    float sanitizePositive(float value, float fallback)
    {
        return (isFinite(value) && value > 0.0f) ? value : fallback;
    }

    glm::vec3 sanitizePosition(const glm::vec3 &position, const glm::vec3 &fallback)
    {
        glm::vec3 result = position;
        if (!isFinite(result.x))
        {
            result.x = fallback.x;
        }
        if (!isFinite(result.y))
        {
            result.y = fallback.y;
        }
        if (!isFinite(result.z))
        {
            result.z = fallback.z;
        }
        return result;
    }

    float computeMinimumSpawnAltitude(const missilesim::sim::TargetSpawnConfig &config, float spawnDistance)
    {
        return std::max(config.minimumAltitudeFloor,
                        std::min(spawnDistance * config.minimumAltitudeDistanceFraction,
                                 config.minimumAltitudeCeiling));
    }

    float computeMaximumSpawnAltitude(const missilesim::sim::TargetSpawnConfig &config,
                                      float spawnDistance,
                                      float minimumAltitude)
    {
        return std::max(minimumAltitude + config.minimumAltitudeBand,
                        std::min(spawnDistance * config.maximumAltitudeDistanceFraction,
                                 config.maximumAltitudeCeiling));
    }

    glm::vec3 fallbackTargetPosition(const missilesim::sim::TargetSpawnConfig &config, int targetIndex)
    {
        const float safeIndex = static_cast<float>(std::max(targetIndex, 0));
        return config.fallbackPosition + glm::vec3(config.fallbackSpacing * safeIndex, 0.0f, 0.0f);
    }

    MAWSConfig toRuntimeMAWSConfig(const missilesim::sim::TargetMawsConfig &config)
    {
        MAWSConfig result;
        result.enabled = config.enabled;
        result.detectionRange = config.detectionRange;
        result.reactionTimeWindow = config.reactionTimeWindow;
        result.closestApproachThreshold = config.closestApproachThreshold;
        return result;
    }

    FlareDispenserConfig toRuntimeFlareDispenserConfig(const missilesim::sim::TargetFlareConfig &config)
    {
        FlareDispenserConfig result;
        result.enabled = config.enabled;
        result.inventory = config.inventory;
        result.burstSize = config.burstSize;
        result.burstInterval = config.burstInterval;
        result.cooldown = config.cooldown;
        result.ejectSpeed = config.ejectSpeed;
        result.aftLaunchOffset = config.aftLaunchOffset;
        result.lateralLaunchOffset = config.lateralLaunchOffset;
        result.lateralEjectFraction = config.lateralEjectFraction;
        result.downwardEjectFraction = config.downwardEjectFraction;
        result.lifetime = config.lifetime;
        result.heatSignature = config.heatSignature;
        result.heatDecayRate = config.heatDecayRate;
        result.mass = config.mass;
        result.dragCoefficient = config.dragCoefficient;
        result.crossSectionalArea = config.crossSectionalArea;
        return result;
    }

    EvasiveManeuverConfig toRuntimeEvasiveManeuverConfig(const missilesim::sim::TargetEvasiveConfig &config)
    {
        EvasiveManeuverConfig result;
        result.cruisePitchRateDegrees = config.cruisePitchRateDegrees;
        result.defensivePitchRateDegrees = config.defensivePitchRateDegrees;
        result.defensiveBreakWeight = config.defensiveBreakWeight;
        result.defensiveAwayWeight = config.defensiveAwayWeight;
        result.defensiveThreatAwayWeight = config.defensiveThreatAwayWeight;
        result.defensiveIncomingWeight = config.defensiveIncomingWeight;
        result.altitudeCorrectionRange = config.altitudeCorrectionRange;
        result.maxAltitudePitchBias = config.maxAltitudePitchBias;
        result.innerDistanceAltitudeOffset = config.innerDistanceAltitudeOffset;
        result.outerDistanceAltitudeOffset = config.outerDistanceAltitudeOffset;
        result.defensiveLowEnergyAltitudeOffset = config.defensiveLowEnergyAltitudeOffset;
        result.defensiveThreatBelowAltitudeOffset = config.defensiveThreatBelowAltitudeOffset;
        result.defensiveThreatAboveAltitudeOffset = config.defensiveThreatAboveAltitudeOffset;
        result.defensiveSpeedBlend = config.defensiveSpeedBlend;
        result.defensiveSpeedUrgencyBlend = config.defensiveSpeedUrgencyBlend;
        result.nearDistanceSpeedupThreshold = config.nearDistanceSpeedupThreshold;
        result.farDistanceSlowdownThreshold = config.farDistanceSlowdownThreshold;
        result.farDistanceSpeedFloor = config.farDistanceSpeedFloor;
        result.recoveryEnergyThreshold = config.recoveryEnergyThreshold;
        result.repositionDistanceThreshold = config.repositionDistanceThreshold;
        return result;
    }

    void applyTargetRuntimeConfig(Target &target,
                                  const TargetAIConfig &aiConfig,
                                  const missilesim::sim::TargetGroupConfig &targetConfig)
    {
        target.setAIConfig(aiConfig);
        target.setMAWSConfig(toRuntimeMAWSConfig(targetConfig.maws));
        target.setFlareDispenserConfig(toRuntimeFlareDispenserConfig(targetConfig.flares));
        target.setEvasiveManeuverConfig(toRuntimeEvasiveManeuverConfig(targetConfig.evasive));
    }
}

void Application::createTarget(const glm::vec3 &position, float radius)
{
    try
    {
        const missilesim::sim::TargetSpawnConfig &spawnConfig = m_simulationConfig.targets.spawn;
        const glm::vec3 validPosition = sanitizePosition(position, spawnConfig.fallbackPosition);
        const float validRadius = sanitizePositive(radius, spawnConfig.fallbackRadius);

        // Create target with validated parameters
        auto target = std::make_unique<Target>(validPosition, validRadius);
        applyTargetRuntimeConfig(*target, m_targetAIConfig, m_simulationConfig.targets);

        // Safety check before adding to physics engine
        if (target && m_physicsEngine)
        {
            m_physicsEngine->addTarget(target.get());
            m_targets.push_back(std::move(target));
            invalidateTrajectoryPreviewCache();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in createTarget: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in createTarget" << std::endl;
    }
}

void Application::createRandomTarget()
{
    try
    {
        const missilesim::sim::TargetSpawnConfig &spawnConfig = m_simulationConfig.targets.spawn;
        std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * glm::pi<float>());
        std::uniform_real_distribution<float> radiusDist(spawnConfig.radiusMin, spawnConfig.radiusMax);
        std::uniform_real_distribution<float> distanceScale(spawnConfig.distanceScaleMin, spawnConfig.distanceScaleMax);

        if (m_targetAIConfig.preferredDistance <= 0.0f || std::isnan(m_targetAIConfig.preferredDistance) || std::isinf(m_targetAIConfig.preferredDistance))
        {
            m_targetAIConfig.preferredDistance = m_simulationConfig.targets.preferredDistance;
        }

        const float spawnDistance = m_targetAIConfig.preferredDistance * distanceScale(m_rng);
        const float minimumSpawnAltitude = computeMinimumSpawnAltitude(spawnConfig, spawnDistance);
        const float maximumSpawnAltitude = computeMaximumSpawnAltitude(spawnConfig, spawnDistance, minimumSpawnAltitude);
        std::uniform_real_distribution<float> heightDist(minimumSpawnAltitude, maximumSpawnAltitude);

        // Generate random spherical coordinates
        float angle = angleDist(m_rng);
        float height = heightDist(m_rng);
        float radius = radiusDist(m_rng);

        // Convert to Cartesian coordinates
        float x = spawnDistance * std::cos(angle);
        float z = spawnDistance * std::sin(angle);

        // Validate generated coordinates
        if (std::isnan(x) || std::isinf(x))
            x = spawnConfig.fallbackPosition.x;
        if (std::isnan(height) || std::isinf(height))
            height = spawnConfig.fallbackPosition.y;
        if (std::isnan(z) || std::isinf(z))
            z = spawnConfig.fallbackPosition.z;
        if (std::isnan(radius) || std::isinf(radius) || radius <= 0.0f)
            radius = spawnConfig.fallbackRadius;

        // Create target at this position
        auto target = std::make_unique<Target>(glm::vec3(x, height, z), radius);
        applyTargetRuntimeConfig(*target, m_targetAIConfig, m_simulationConfig.targets);

        // Safety check before adding to physics engine
        if (target && m_physicsEngine)
        {
            m_physicsEngine->addTarget(target.get());
            m_targets.push_back(std::move(target));
            invalidateTrajectoryPreviewCache();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in createRandomTarget: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in createRandomTarget" << std::endl;
    }
}

void Application::resetTargets()
{
    try
    {
        // Validate target count
        const int maximumTargetCount = std::max(m_simulationConfig.targets.maxCount, 1);
        if (m_targetCount <= 0 || m_targetCount > maximumTargetCount)
        {
            m_targetCount = std::clamp(m_targetCount, 1, maximumTargetCount);
        }

        // First remove all targets from physics engine
        for (auto &target : m_targets)
        {
            if (target)
            {
                m_physicsEngine->removeTarget(target.get());
            }
        }

        // Clear existing targets
        if (m_missile)
        {
            m_missile->clearTarget();
        }
        m_targets.clear();
        clearFlares();
        m_explosions.clear();
        if (m_renderer)
        {
            m_renderer->clearEffects();
        }
        if (m_audioSystem)
        {
            m_audioSystem->stopMissileEmitters();
        }
        invalidateTrajectoryPreviewCache();

        // Create new random targets
        for (int i = 0; i < m_targetCount; i++)
        {
            createRandomTarget();

            // Failsafe - if target creation failed and we still have no targets, create a default one
            if (m_targets.empty())
            {
                createTarget(fallbackTargetPosition(m_simulationConfig.targets.spawn, i),
                             m_simulationConfig.targets.spawn.fallbackRadius);
            }
        }

        // Final check - make sure we have at least one target
        if (m_targets.empty())
        {
            createTarget(fallbackTargetPosition(m_simulationConfig.targets.spawn, 0),
                         m_simulationConfig.targets.spawn.fallbackRadius);
        }

        if (m_physicsEngine)
        {
            for (auto &target : m_targets)
            {
                if (target && target->isActive())
                {
                    // Apply a small delta time to initialize movement
                    target->update(m_simulationConfig.targets.spawn.warmupTimeStep);
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in resetTargets: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in resetTargets" << std::endl;
    }
}

void Application::createFlare(const FlareLaunchRequest &request)
{
    try
    {
        auto flare = std::make_unique<Flare>(request);
        if (!flare || !flare->isActive() || !m_physicsEngine)
        {
            return;
        }

        m_physicsEngine->addFlare(flare.get());
        if (m_audioSystem)
        {
            m_audioSystem->playFlareLaunch(request.position, request.velocity, request.heatSignature);
        }
        m_flares.push_back(std::move(flare));
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in createFlare: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in createFlare" << std::endl;
    }
}

void Application::collectPendingTargetFlares()
{
    for (const auto &target : m_targets)
    {
        if (!target)
        {
            continue;
        }

        for (const FlareLaunchRequest &request : target->consumePendingFlareLaunches())
        {
            createFlare(request);
        }
    }
}

void Application::removeInactiveFlares()
{
    if (!m_physicsEngine)
    {
        if (m_audioSystem)
        {
            for (const auto &flare : m_flares)
            {
                if (flare)
                {
                    m_audioSystem->retireFlare(flare.get());
                }
            }
        }
        m_flares.clear();
        return;
    }

    auto it = m_flares.begin();
    while (it != m_flares.end())
    {
        if (!(*it) || !(*it)->isActive())
        {
            if (*it)
            {
                if (m_audioSystem)
                {
                    m_audioSystem->retireFlare(it->get());
                }
                m_physicsEngine->removeFlare(it->get());
            }
            it = m_flares.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void Application::clearFlares()
{
    for (auto &flare : m_flares)
    {
        if (!flare)
        {
            continue;
        }

        if (m_audioSystem)
        {
            m_audioSystem->retireFlare(flare.get());
        }

        if (m_physicsEngine)
        {
            m_physicsEngine->removeFlare(flare.get());
        }
    }

    m_flares.clear();
}
