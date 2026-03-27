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

void Application::createTarget(const glm::vec3 &position, float radius)
{
    try
    {
        // Validate target parameters
        glm::vec3 validPosition = position;
        float validRadius = radius;

        // Check position values
        if (std::isnan(validPosition.x) || std::isinf(validPosition.x))
            validPosition.x = 100.0f;
        if (std::isnan(validPosition.y) || std::isinf(validPosition.y))
            validPosition.y = 100.0f;
        if (std::isnan(validPosition.z) || std::isinf(validPosition.z))
            validPosition.z = 100.0f;

        // Check radius
        if (validRadius <= 0.0f || std::isnan(validRadius) || std::isinf(validRadius))
        {
            validRadius = 5.0f; // Default safe value
        }

        // Create target with validated parameters
        auto target = std::make_unique<Target>(validPosition, validRadius);
        target->setAIConfig(m_targetAIConfig);

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
        std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * glm::pi<float>());
        std::uniform_real_distribution<float> radiusDist(3.0f, 7.0f);
        std::uniform_real_distribution<float> distanceScale(0.82f, 1.18f);

        if (m_targetAIConfig.preferredDistance <= 0.0f || std::isnan(m_targetAIConfig.preferredDistance) || std::isinf(m_targetAIConfig.preferredDistance))
        {
            m_targetAIConfig.preferredDistance = 1500.0f;
        }

        const float spawnDistance = m_targetAIConfig.preferredDistance * distanceScale(m_rng);
        const float minimumSpawnAltitude = std::max(80.0f, std::min(spawnDistance * 0.08f, 500.0f));
        const float maximumSpawnAltitude = std::max(minimumSpawnAltitude + 60.0f, std::min(spawnDistance * 0.28f, 3200.0f));
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
            x = 100.0f;
        if (std::isnan(height) || std::isinf(height))
            height = 100.0f;
        if (std::isnan(z) || std::isinf(z))
            z = 100.0f;
        if (std::isnan(radius) || std::isinf(radius) || radius <= 0.0f)
            radius = 5.0f;

        // Create target at this position
        auto target = std::make_unique<Target>(glm::vec3(x, height, z), radius);
        target->setAIConfig(m_targetAIConfig);

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
        if (m_targetCount <= 0 || m_targetCount > 20)
        {
            m_targetCount = 1;
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
                createTarget(glm::vec3(100.0f * (i + 1), 100.0f, 100.0f), 5.0f);
            }
        }

        // Final check - make sure we have at least one target
        if (m_targets.empty())
        {
            createTarget(glm::vec3(100.0f, 100.0f, 100.0f), 5.0f);
        }

        if (m_physicsEngine)
        {
            for (auto &target : m_targets)
            {
                if (target && target->isActive())
                {
                    // Apply a small delta time to initialize movement
                    target->update(0.016f);
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