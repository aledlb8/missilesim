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

bool Application::loadSettings()
{
    std::ifstream input(m_settingsPath);
    if (!input.is_open())
    {
        return false;
    }

    std::unordered_map<std::string, std::string> values;
    std::string line;
    while (std::getline(input, line))
    {
        const std::string trimmed = trimWhitespace(line);
        if (trimmed.empty() || trimmed[0] == '#')
        {
            continue;
        }

        const size_t delimiter = trimmed.find('=');
        if (delimiter == std::string::npos)
        {
            continue;
        }

        const std::string key = trimWhitespace(trimmed.substr(0, delimiter));
        const std::string value = trimWhitespace(trimmed.substr(delimiter + 1));
        if (!key.empty())
        {
            values[key] = value;
        }
    }

    auto readString = [&](const char *key, const std::string &fallback) -> std::string
    {
        const auto found = values.find(key);
        return (found != values.end()) ? found->second : fallback;
    };

    auto readBool = [&](const char *key, bool fallback) -> bool
    {
        return parseBoolValue(readString(key, formatBoolValue(fallback)), fallback);
    };

    auto readFloat = [&](const char *key, float fallback) -> float
    {
        return parseFloatValue(readString(key, ""), fallback);
    };

    auto readInt = [&](const char *key, int fallback) -> int
    {
        return parseIntValue(readString(key, ""), fallback);
    };

    auto readVec3 = [&](const char *key, const glm::vec3 &fallback) -> glm::vec3
    {
        return parseVec3Value(readString(key, ""), fallback);
    };

    m_showTrajectory = readBool("show_trajectory", m_showTrajectory);
    m_showTargetInfo = readBool("show_target_info", m_showTargetInfo);
    m_showPredictedTargetPath = readBool("show_predicted_target_path", m_showPredictedTargetPath);
    m_showInterceptPoint = readBool("show_intercept_point", m_showInterceptPoint);
    m_trajectoryPoints = std::clamp(readInt("trajectory_points", m_trajectoryPoints), 10, 600);
    m_trajectoryTime = std::clamp(readFloat("trajectory_time", m_trajectoryTime), 0.5f, 60.0f);
    m_simulationSpeed = std::clamp(readFloat("simulation_speed", m_simulationSpeed), 0.1f, 10.0f);

    m_groundEnabled = readBool("ground_enabled", m_groundEnabled);
    m_groundRestitution = std::clamp(readFloat("ground_restitution", m_groundRestitution), 0.0f, 1.0f);

    glm::vec3 initialPosition = readVec3("initial_position", glm::vec3(m_initialPosition[0], m_initialPosition[1], m_initialPosition[2]));
    glm::vec3 initialVelocity = readVec3("initial_velocity", glm::vec3(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]));
    m_initialPosition[0] = initialPosition.x;
    m_initialPosition[1] = initialPosition.y;
    m_initialPosition[2] = initialPosition.z;
    m_initialVelocity[0] = initialVelocity.x;
    m_initialVelocity[1] = initialVelocity.y;
    m_initialVelocity[2] = initialVelocity.z;

    m_mass = std::max(readFloat("missile_mass", m_mass), 0.01f);
    m_dragCoefficient = std::max(readFloat("missile_drag_coefficient", m_dragCoefficient), 0.0f);
    m_crossSectionalArea = std::max(readFloat("missile_cross_sectional_area", m_crossSectionalArea), 0.0001f);
    m_liftCoefficient = std::max(readFloat("missile_lift_coefficient", m_liftCoefficient), 0.0f);
    m_guidanceEnabled = readBool("guidance_enabled", m_guidanceEnabled);
    m_navigationGain = std::clamp(readFloat("navigation_gain", m_navigationGain), 1.0f, 4.0f);
    m_maxSteeringForce = std::max(readFloat("max_steering_force", m_maxSteeringForce), 1000.0f);
    m_trackingAngle = std::clamp(readFloat("tracking_angle", m_trackingAngle), 5.0f, 180.0f);
    m_proximityFuseRadius = std::max(readFloat("proximity_fuse_radius", m_proximityFuseRadius), 0.0f);
    m_countermeasureResistance = glm::clamp(readFloat("countermeasure_resistance", m_countermeasureResistance), 0.0f, 1.0f);
    m_terrainAvoidanceEnabled = readBool("terrain_avoidance_enabled", m_terrainAvoidanceEnabled);
    m_terrainClearance = std::max(readFloat("terrain_clearance", m_terrainClearance), 0.0f);
    m_terrainLookAheadTime = std::max(readFloat("terrain_lookahead_time", m_terrainLookAheadTime), 0.5f);

    m_missileThrust = std::max(readFloat("missile_thrust", m_missileThrust), 0.0f);
    m_missileFuel = std::max(readFloat("missile_fuel", m_missileFuel), 0.0f);
    m_missileFuelConsumptionRate = std::max(readFloat("missile_fuel_consumption_rate", m_missileFuelConsumptionRate), 0.0f);

    m_targetCount = std::clamp(readInt("target_count", m_targetCount), 1, 20);
    m_targetAIConfig.preferredDistance = std::max(readFloat("target_average_distance",
                                                            readFloat("target_spawn_distance", m_targetAIConfig.preferredDistance)),
                                                  250.0f);
    m_targetAIConfig.minSpeed = std::max(readFloat("target_min_speed", m_targetAIConfig.minSpeed), 40.0f);
    m_targetAIConfig.maxSpeed = std::max(readFloat("target_max_speed", m_targetAIConfig.maxSpeed), m_targetAIConfig.minSpeed + 10.0f);

    m_savedGravity = std::max(readFloat("gravity", m_savedGravity), 0.0f);
    m_savedAirDensity = std::max(readFloat("air_density", m_savedAirDensity), 0.0f);
    m_savedCameraFOV = std::clamp(readFloat("camera_fov", m_savedCameraFOV), 10.0f, 120.0f);
    m_savedCameraSpeed = std::max(readFloat("camera_speed", m_savedCameraSpeed), 0.1f);

    if (m_physicsEngine)
    {
        m_physicsEngine->setGroundEnabled(m_groundEnabled);
        m_physicsEngine->setGroundRestitution(m_groundRestitution);
        m_physicsEngine->setGravity(m_savedGravity);
        m_physicsEngine->setAirDensity(m_savedAirDensity);
    }

    if (m_renderer)
    {
        m_renderer->setCameraFOV(m_savedCameraFOV);
        m_renderer->setCameraSpeed(m_savedCameraSpeed);
    }

    m_settingsDirty = false;
    m_settingsAutosaveDelay = 0.0f;
    m_lastSettingsSnapshot = buildSettingsSnapshot();
    return true;
}

std::string Application::buildSettingsSnapshot() const
{
    std::ostringstream output;

    const float gravity = m_physicsEngine ? m_physicsEngine->getGravity() : m_savedGravity;
    const float airDensity = m_physicsEngine ? m_physicsEngine->getAirDensity() : m_savedAirDensity;
    const float cameraFOV = m_renderer ? m_renderer->getCameraFOV() : m_savedCameraFOV;
    const float cameraSpeed = m_renderer ? m_renderer->getCameraSpeed() : m_savedCameraSpeed;

    output << "show_trajectory=" << formatBoolValue(m_showTrajectory) << "\n";
    output << "show_target_info=" << formatBoolValue(m_showTargetInfo) << "\n";
    output << "show_predicted_target_path=" << formatBoolValue(m_showPredictedTargetPath) << "\n";
    output << "show_intercept_point=" << formatBoolValue(m_showInterceptPoint) << "\n";
    output << "trajectory_points=" << m_trajectoryPoints << "\n";
    output << "trajectory_time=" << m_trajectoryTime << "\n";
    output << "simulation_speed=" << m_simulationSpeed << "\n";
    output << "gravity=" << gravity << "\n";
    output << "air_density=" << airDensity << "\n";
    output << "ground_enabled=" << formatBoolValue(m_groundEnabled) << "\n";
    output << "ground_restitution=" << m_groundRestitution << "\n";
    output << "initial_position=" << formatVec3Value(glm::vec3(m_initialPosition[0], m_initialPosition[1], m_initialPosition[2])) << "\n";
    output << "initial_velocity=" << formatVec3Value(glm::vec3(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2])) << "\n";
    output << "missile_mass=" << m_mass << "\n";
    output << "missile_drag_coefficient=" << m_dragCoefficient << "\n";
    output << "missile_cross_sectional_area=" << m_crossSectionalArea << "\n";
    output << "missile_lift_coefficient=" << m_liftCoefficient << "\n";
    output << "guidance_enabled=" << formatBoolValue(m_guidanceEnabled) << "\n";
    output << "navigation_gain=" << m_navigationGain << "\n";
    output << "max_steering_force=" << m_maxSteeringForce << "\n";
    output << "tracking_angle=" << m_trackingAngle << "\n";
    output << "proximity_fuse_radius=" << m_proximityFuseRadius << "\n";
    output << "countermeasure_resistance=" << m_countermeasureResistance << "\n";
    output << "terrain_avoidance_enabled=" << formatBoolValue(m_terrainAvoidanceEnabled) << "\n";
    output << "terrain_clearance=" << m_terrainClearance << "\n";
    output << "terrain_lookahead_time=" << m_terrainLookAheadTime << "\n";
    output << "missile_thrust=" << m_missileThrust << "\n";
    output << "missile_fuel=" << m_missileFuel << "\n";
    output << "missile_fuel_consumption_rate=" << m_missileFuelConsumptionRate << "\n";
    output << "target_count=" << m_targetCount << "\n";
    output << "target_average_distance=" << m_targetAIConfig.preferredDistance << "\n";
    output << "target_min_speed=" << m_targetAIConfig.minSpeed << "\n";
    output << "target_max_speed=" << m_targetAIConfig.maxSpeed << "\n";
    output << "camera_fov=" << cameraFOV << "\n";
    output << "camera_speed=" << cameraSpeed << "\n";

    return output.str();
}

void Application::saveSettings()
{
    try
    {
        std::filesystem::path settingsPath(m_settingsPath);
        if (settingsPath.has_parent_path())
        {
            std::filesystem::create_directories(settingsPath.parent_path());
        }

        std::ofstream output(settingsPath, std::ios::trunc);
        if (!output.is_open())
        {
            std::cerr << "ERROR: Failed to open settings file for writing: " << settingsPath << std::endl;
            return;
        }

        const float gravity = m_physicsEngine ? m_physicsEngine->getGravity() : m_savedGravity;
        const float airDensity = m_physicsEngine ? m_physicsEngine->getAirDensity() : m_savedAirDensity;
        const float cameraFOV = m_renderer ? m_renderer->getCameraFOV() : m_savedCameraFOV;
        const float cameraSpeed = m_renderer ? m_renderer->getCameraSpeed() : m_savedCameraSpeed;

        m_savedGravity = gravity;
        m_savedAirDensity = airDensity;
        m_savedCameraFOV = cameraFOV;
        m_savedCameraSpeed = cameraSpeed;

        const std::string snapshot = buildSettingsSnapshot();

        output << "# MissileSim user settings\n";
        output << snapshot;
        m_lastSettingsSnapshot = snapshot;
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Failed to save settings: " << e.what() << std::endl;
    }
}

void Application::scheduleSettingsSave()
{
    m_settingsDirty = true;
    m_settingsAutosaveDelay = 0.75f;
}

void Application::flushSettingsAutosave(float deltaTime)
{
    if (!m_settingsDirty)
    {
        return;
    }

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        deltaTime = 0.016f;
    }

    m_settingsAutosaveDelay = std::max(0.0f, m_settingsAutosaveDelay - deltaTime);
    if (m_settingsAutosaveDelay <= 0.0f)
    {
        saveSettings();
        m_settingsDirty = false;
    }
}