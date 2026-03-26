#include "Application.h"
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <glm/gtx/norm.hpp>
#include "physics/Atmosphere.h"
#include "physics/forces/Drag.h"
#include "physics/forces/Lift.h"

namespace
{
    std::string trimWhitespace(const std::string &value)
    {
        size_t start = 0;
        while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])))
        {
            ++start;
        }

        size_t end = value.size();
        while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])))
        {
            --end;
        }

        return value.substr(start, end - start);
    }

    bool parseBoolValue(const std::string &value, bool fallback)
    {
        const std::string normalized = trimWhitespace(value);
        if (normalized == "true" || normalized == "1")
        {
            return true;
        }
        if (normalized == "false" || normalized == "0")
        {
            return false;
        }
        return fallback;
    }

    float parseFloatValue(const std::string &value, float fallback)
    {
        try
        {
            return std::stof(trimWhitespace(value));
        }
        catch (...)
        {
            return fallback;
        }
    }

    int parseIntValue(const std::string &value, int fallback)
    {
        try
        {
            return std::stoi(trimWhitespace(value));
        }
        catch (...)
        {
            return fallback;
        }
    }

    glm::vec3 parseVec3Value(const std::string &value, const glm::vec3 &fallback)
    {
        std::stringstream stream(value);
        std::string component;
        glm::vec3 parsed = fallback;

        if (std::getline(stream, component, ','))
        {
            parsed.x = parseFloatValue(component, fallback.x);
        }
        if (std::getline(stream, component, ','))
        {
            parsed.y = parseFloatValue(component, fallback.y);
        }
        if (std::getline(stream, component, ','))
        {
            parsed.z = parseFloatValue(component, fallback.z);
        }

        return parsed;
    }

    std::string formatBoolValue(bool value)
    {
        return value ? "true" : "false";
    }

    std::string formatVec3Value(const glm::vec3 &value)
    {
        std::ostringstream stream;
        stream << value.x << "," << value.y << "," << value.z;
        return stream.str();
    }

    glm::vec3 safeNormalize(const glm::vec3 &value, const glm::vec3 &fallback)
    {
        if (glm::length2(value) > 0.0001f)
        {
            return glm::normalize(value);
        }
        return fallback;
    }
}

Application::Application(int width, int height, const std::string &title)
    : m_width(width), m_height(height), m_title(title), m_window(nullptr),
      m_lastMouseX(width / 2.0f), m_lastMouseY(height / 2.0f), m_firstMouse(true)
{
    // Initialize random number generator with a time-based seed
    std::random_device rd;
    m_rng = std::mt19937(rd());
}

Application::~Application()
{
    shutdown();
}

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

void Application::initialize()
{
    try
    {
        // Initialize GLFW
        if (!glfwInit())
        {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return;
        }

        // Configure GLFW
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        // Create window
        m_window = glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
        if (!m_window)
        {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(m_window);

        // Set up window resize callback
        glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow *window, int width, int height)
                                       {
            // Update viewport
            glViewport(0, 0, width, height);
            
            // Update application instance's window dimensions
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            if (app && app->m_renderer) {
                app->m_width = width;
                app->m_height = height;
                app->m_renderer->setViewportSize(width, height);
            } });

        // Store pointer to application instance for callbacks
        glfwSetWindowUserPointer(m_window, this);

        // Set up mouse cursor callbacks
        glfwSetCursorPosCallback(m_window, [](GLFWwindow *window, double xpos, double ypos)
                                 {
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            app->mouseCallback(xpos, ypos); });

        // Set up mouse button callbacks
        glfwSetMouseButtonCallback(m_window, [](GLFWwindow *window, int button, int action, int mods)
                                   {
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            app->mouseButtonCallback(button, action); });

        // Initialize GLAD
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cerr << "Failed to initialize GLAD" << std::endl;
            return;
        }

        // Setup ImGui with error handling
        try
        {
            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGuiIO &io = ImGui::GetIO();
            (void)io;

            // Initialize ImGui GLFW integration
            if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true))
            {
                std::cerr << "Failed to initialize ImGui GLFW backend" << std::endl;
                ImGui::DestroyContext();
                return;
            }

            // Initialize ImGui OpenGL3 renderer
            if (!ImGui_ImplOpenGL3_Init("#version 330"))
            {
                std::cerr << "Failed to initialize ImGui OpenGL3 backend" << std::endl;
                ImGui_ImplGlfw_Shutdown();
                ImGui::DestroyContext();
                return;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception during ImGui initialization: " << e.what() << std::endl;
            return;
        }
        catch (...)
        {
            std::cerr << "Unknown exception during ImGui initialization" << std::endl;
            return;
        }

        // Create simulation components
        m_physicsEngine = std::make_unique<PhysicsEngine>();
        m_renderer = std::make_unique<Renderer>();

        const bool loadedSettings = loadSettings();

        // Set ground collision properties
        m_physicsEngine->setGroundEnabled(m_groundEnabled);
        m_physicsEngine->setGroundRestitution(m_groundRestitution);
        m_physicsEngine->setGravity(m_savedGravity);
        m_physicsEngine->setAirDensity(m_savedAirDensity);
        m_renderer->setCameraFOV(m_savedCameraFOV);
        m_renderer->setCameraSpeed(m_savedCameraSpeed);

        // Create missile with default parameters
        resetMissile();

        // Create targets first, before setting up missile guidance
        resetTargets();

        // Run a small update so the autonomous target controller starts from a live state.
        if (m_physicsEngine && !m_targets.empty())
        {
            // Update targets to initialize movement patterns
            for (auto &target : m_targets)
            {
                if (target && target->isActive())
                {
                    // Apply a small delta time to initialize movement
                    target->update(0.016f);
                }
            }
        }

        frameEngagementCamera();
        if (loadedSettings)
        {
            m_renderer->setCameraFOV(m_savedCameraFOV);
            m_renderer->setCameraSpeed(m_savedCameraSpeed);
        }

        updateEnvironmentScale();
        m_lastSettingsSnapshot = buildSettingsSnapshot();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        shutdown(); // Attempt to clean up if initialization fails
    }
    catch (...)
    {
        std::cerr << "Unknown exception during initialization" << std::endl;
        shutdown(); // Attempt to clean up if initialization fails
    }
}

void Application::shutdown()
{
    try
    {
        if (!m_window && !m_renderer && !m_physicsEngine)
        {
            return;
        }

        saveSettings();
        m_settingsDirty = false;

        // First clear objects that might be using the renderer or physics
        m_missile.reset(); // Release missile before targets to avoid invalid target references
        clearFlares();

        // Clean up targets
        for (auto &target : m_targets)
        {
            if (target && m_physicsEngine)
            {
                m_physicsEngine->removeTarget(target.get());
            }
        }
        m_targets.clear();

        // Clean up physics engine and renderer before ImGui
        m_physicsEngine.reset();
        m_renderer.reset();

        // Cleanup ImGui - use explicit checks to avoid calling shutdown on null pointers
        if (ImGui::GetCurrentContext() != nullptr)
        {
            // Check if ImGui is properly initialized before shutting down
            ImGuiIO &io = ImGui::GetIO();

            // Cleanup ImGui OpenGL renderer
            if (io.BackendRendererUserData != nullptr)
            {
                ImGui_ImplOpenGL3_Shutdown();
            }

            // Cleanup ImGui GLFW integration
            if (io.BackendPlatformUserData != nullptr)
            {
                ImGui_ImplGlfw_Shutdown();
            }

            ImGui::DestroyContext();
        }

        // Cleanup GLFW
        if (m_window)
        {
            glfwDestroyWindow(m_window);
            m_window = nullptr;
        }

        // Finally terminate GLFW
        glfwTerminate();
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception during shutdown: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception during shutdown" << std::endl;
    }
}

void Application::run()
{
    try
    {
        initialize();

        // If initialization failed, exit
        if (!m_window)
        {
            std::cerr << "ERROR: Failed to initialize window, cannot run application" << std::endl;
            return;
        }

        // Set initial viewport size to match window
        int width, height;
        glfwGetFramebufferSize(m_window, &width, &height);
        m_width = width;
        m_height = height;
        m_renderer->setViewportSize(width, height);

        auto lastTime = std::chrono::high_resolution_clock::now();

        // Main loop
        while (!glfwWindowShouldClose(m_window))
        {
            try
            {
                // Calculate delta time
                auto currentTime = std::chrono::high_resolution_clock::now();
                float deltaTime = std::chrono::duration<float>(currentTime - lastTime).count();
                lastTime = currentTime;

                // Cap extremely large deltaTime values (e.g., after debugger pause)
                if (deltaTime > 0.5f)
                {
                    deltaTime = 0.016f; // ~60 FPS
                }

                m_lastFrameDeltaTime = deltaTime;

                // Process input
                try
                {
                    processInput(deltaTime);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "ERROR: Exception in processInput: " << e.what() << std::endl;
                }
                catch (...)
                {
                    std::cerr << "ERROR: Unknown exception in processInput" << std::endl;
                }

                // Update physics
                if (!m_isPaused)
                {
                    update(deltaTime);
                }

                // Render
                render();
                flushSettingsAutosave(deltaTime);

                // Poll events and swap buffers
                try
                {
                    glfwPollEvents();
                    glfwSwapBuffers(m_window);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "ERROR: Exception in GLFW event handling: " << e.what() << std::endl;
                }
                catch (...)
                {
                    std::cerr << "ERROR: Unknown exception in GLFW event handling" << std::endl;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "ERROR: Exception in main loop: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "ERROR: Unknown exception in main loop" << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Fatal exception in run: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown fatal exception in run" << std::endl;
    }

    // Always attempt shutdown, even if we had an exception
    try
    {
        shutdown();
    }
    catch (...)
    {
        std::cerr << "ERROR: Exception during shutdown" << std::endl;
    }
}

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

void Application::update(float deltaTime)
{
    try
    {
        // Validate deltaTime to prevent instability
        if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
        {
            deltaTime = 0.016f; // Default to ~60 FPS
        }

        // Cap excessively large deltaTime values which could cause instability
        if (deltaTime > 0.1f)
        {
            deltaTime = 0.1f;
        }

        // Safety check for null missile (in case it wasn't initialized)
        if (!m_missile)
        {
            resetMissile();
            return;
        }

        updateExplosions(deltaTime);

        // Accumulated time approach for fixed time step
        static float accumulator = 0.0f;
        accumulator += deltaTime * m_simulationSpeed;

        // Cap accumulated time to prevent "spiral of death" if game lags
        if (accumulator > 0.5f)
        {
            accumulator = 0.5f;
        }

        // Validate timeStep
        if (m_timeStep <= 0.0f || std::isnan(m_timeStep) || std::isinf(m_timeStep))
        {
            m_timeStep = 0.01f; // Default to sensible value
        }

        // Update physics in fixed time steps
        int maxSteps = 5; // Prevent too many updates in a single frame
        int stepCount = 0;

        while (accumulator >= m_timeStep && stepCount < maxSteps)
        {
            try
            {
                // Store previous position of missile for hit detection
                glm::vec3 prevMissilePos = m_missile->getPosition();

                // Update physics
                m_physicsEngine->update(m_timeStep);

                // Check if a new hit occurred this frame
                bool missileFlightEnded = false;
                for (const auto &target : m_targets)
                {
                    if (target && !target->isActive() && target->getPosition() != glm::vec3(0))
                    {
                        createExplosion(target->getPosition());
                        // This target was just hit (it's inactive but not yet reset)
                        m_score += 100; // Add points for hitting a target
                        m_targetHits++; // Increment hit counter

                        // Reset target position to prevent multiple explosions
                        target->setPosition(glm::vec3(0));
                        missileFlightEnded = true;
                    }
                }

                if (missileFlightEnded && m_missileInFlight)
                {
                    terminateMissileFlight(prevMissilePos, false);
                    accumulator -= m_timeStep;
                    stepCount++;
                    break;
                }

                if (m_missileInFlight && m_missile)
                {
                    m_missileFlightTime += m_timeStep;

                    const glm::vec3 missilePos = m_missile->getPosition();
                    const glm::vec3 missileVel = m_missile->getVelocity();
                    const float missileSpeed = glm::length(missileVel);

                    bool terminateFlight = m_missile->consumeSelfDestructRequest();

                    // Treat ground contact as impact rather than allowing endless bouncing/guidance loops.
                    if (!terminateFlight && m_groundEnabled && prevMissilePos.y > 0.05f && missilePos.y <= 0.01f)
                    {
                        terminateFlight = true;
                    }

                    // Kill bad flights that leave the playable space.
                    const float engagementRadius = computeEngagementRadius();
                    const float maxFlightRadius = std::max(5000.0f, engagementRadius * 5.0f);
                    const float maxFlightAltitude = std::max(3000.0f, engagementRadius * 1.8f);
                    const float missileHorizontalDistance = glm::length(glm::vec2(missilePos.x, missilePos.z));
                    if (!terminateFlight &&
                        (missileHorizontalDistance > maxFlightRadius || missilePos.y > maxFlightAltitude))
                    {
                        terminateFlight = true;
                    }

                    Target *trackedTarget = m_missile->getTargetObject();
                    if (!terminateFlight && trackedTarget && trackedTarget->isActive())
                    {
                        const glm::vec3 toTarget = trackedTarget->getPosition() - missilePos;
                        const float distanceToTarget = glm::length(toTarget);
                        m_closestTargetDistance = std::min(m_closestTargetDistance, distanceToTarget);

                        if (m_missileFlightTime > 0.75f &&
                            distanceToTarget > (m_closestTargetDistance + std::max(trackedTarget->getRadius() * 4.0f, 25.0f)) &&
                            glm::length2(toTarget) > 0.0001f &&
                            missileSpeed > 1.0f)
                        {
                            const glm::vec3 lineOfSight = glm::normalize(toTarget);
                            const glm::vec3 relativeVelocity = trackedTarget->getVelocity() - missileVel;
                            const float rangeRate = glm::dot(relativeVelocity, lineOfSight);
                            const float aspect = glm::dot(glm::normalize(missileVel), lineOfSight);

                            // If the target is behind the missile and range is opening, the shot is spent.
                            if (rangeRate > 15.0f && aspect < -0.15f)
                            {
                                terminateFlight = true;
                            }
                        }
                    }

                    // Stop dead-stick missiles from wandering forever after they have burned out.
                    if (!terminateFlight &&
                        !m_missile->isThrustEnabled() &&
                        m_missileFlightTime > 2.0f &&
                        missileSpeed < 15.0f)
                    {
                        terminateFlight = true;
                    }

                    if (terminateFlight)
                    {
                        terminateMissileFlight(missilePos, true);
                        accumulator -= m_timeStep;
                        stepCount++;
                        break;
                    }
                }

                accumulator -= m_timeStep;
                stepCount++;
            }
            catch (const std::exception &e)
            {
                std::cerr << "ERROR: Exception during physics update step: " << e.what() << std::endl;
                accumulator = 0.0f; // Reset accumulator to prevent more steps this frame
                break;
            }
            catch (...)
            {
                std::cerr << "ERROR: Unknown exception during physics update step" << std::endl;
                accumulator = 0.0f; // Reset accumulator to prevent more steps this frame
                break;
            }
        }

        collectPendingTargetFlares();
        removeInactiveFlares();

        // Check if all targets are inactive, and create new ones if needed
        bool allTargetsInactive = true;
        for (const auto &target : m_targets)
        {
            if (target && target->isActive())
            {
                allTargetsInactive = false;
                break;
            }
        }

        if (allTargetsInactive && !m_targets.empty())
        {
            resetTargets();
            resetMissile();
        }

        // If no targets exist but we should have some, create them
        if (m_targets.empty() && m_targetCount > 0)
        {
            resetTargets();
        }

        // Update targets for movement (in case they need updating outside physics engine)
        for (const auto &target : m_targets)
        {
            if (target && target->isActive())
            {
                // Additional target-specific updates could be added here
                // Physics engine already calls target->update()
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in update: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in update" << std::endl;
    }
}

void Application::render()
{
    try
    {
        // Clear the screen
        glClearColor(0.58f, 0.69f, 0.82f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Safety check for renderer
        if (!m_renderer)
        {
            std::cerr << "ERROR: Renderer is null in render()" << std::endl;
            return;
        }

        updateActiveCameraMode();
        m_renderer->clearDebugPrimitives();
        updateEnvironmentScale();
        m_renderer->renderEnvironment();

        // Safety check for null missile
        if (!m_missile)
        {
            resetMissile();
        }

        // Render missile
        if (m_missile)
        {
            try
            {
                m_renderer->render(m_missile.get());

                // Render predicted trajectory if enabled
                if (m_showTrajectory)
                {
                    renderPredictedTrajectory();
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "ERROR: Exception rendering missile: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "ERROR: Unknown exception rendering missile" << std::endl;
            }
        }

        struct WorldLabelLine
        {
            std::string text;
            ImU32 color;
            float screenYOffset;
        };

        struct WorldLabel
        {
            glm::vec3 position;
            std::vector<WorldLabelLine> lines;
        };

        // Collect target positions for UI display
        std::vector<WorldLabel> targetLabels;
        targetLabels.reserve(m_targets.size());

        // Render targets
        for (const auto &target : m_targets)
        {
            try
            {
                if (target && target->isActive())
                {
                    m_renderer->render(target.get());

                    // Render target debug info if enabled, or always in hidden HUD mode.
                    if (m_missile && (m_showTargetInfo || !m_showUI))
                    {
                        const float distance = glm::length(target->getPosition() - m_missile->getPosition());
                        const float targetAltitude = std::max(target->getPosition().y, 0.0f);
                        const float targetSpeed = glm::length(target->getVelocity());
                        WorldLabel label;
                        label.position = target->getPosition();
                        label.lines.push_back({"Target: " + std::to_string(static_cast<int>(distance)) + "m",
                                               IM_COL32(255, 234, 120, 255), 0.0f});

                        char buffer[96];
                        std::snprintf(buffer, sizeof(buffer), "ALT %.0f m", targetAltitude);
                        label.lines.push_back({buffer, IM_COL32(255, 214, 132, 255), 16.0f});

                        std::snprintf(buffer, sizeof(buffer), "SPD %.0f m/s", targetSpeed);
                        label.lines.push_back({buffer, IM_COL32(255, 196, 142, 255), 32.0f});

                        if (target->isMissileWarningActive())
                        {
                            label.lines.push_back({"MAWS | Flares " + std::to_string(target->getRemainingFlares()),
                                                   IM_COL32(255, 164, 124, 255), 48.0f});
                        }
                        targetLabels.push_back(std::move(label));
                    }
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "ERROR: Exception rendering target: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "ERROR: Unknown exception rendering target" << std::endl;
            }
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
            const glm::vec3 flareColor = glm::mix(glm::vec3(0.9f, 0.35f, 0.1f), glm::vec3(1.0f, 0.95f, 0.6f), heatFraction);
            const float flareSize = glm::mix(3.0f, 8.0f, heatFraction);
            m_renderer->renderPoint(flare->getPosition(), flareColor, flareSize);
        }

        m_renderer->flushDebugPrimitives();
        renderExplosions();

        // Setup ImGui frame - wrap in try-catch for safety
        try
        {
            if (ImGui::GetCurrentContext() != nullptr)
            {
                ImGui_ImplOpenGL3_NewFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();

                if (m_showUI)
                {
                    setupUI();
                }
                else
                {
                    renderMinimalHUD();
                }

                // Draw world labels using ImGui in screen space projected from the scene.
                if (m_window && m_missile && ((m_showUI && m_showTargetInfo) || !m_showUI))
                {
                    glm::mat4 view = glm::lookAt(m_renderer->getCameraPosition(),
                                                 m_renderer->getCameraPosition() + m_renderer->getCameraFront(),
                                                 m_renderer->getCameraUp());
                    glm::mat4 projection = glm::perspective(glm::radians(m_renderer->getCameraFOV()),
                                                            (float)m_width / (float)m_height,
                                                            0.1f, m_renderer->getSceneFarPlane());
                    ImDrawList *drawList = ImGui::GetBackgroundDrawList();

                    auto drawWorldText = [&](const glm::vec3 &worldPosition, const std::string &text, ImU32 color, float screenYOffset = 0.0f)
                    {
                        glm::vec4 clipSpace = projection * view * glm::vec4(worldPosition, 1.0f);

                        if (clipSpace.w <= 0.0f)
                        {
                            return;
                        }

                        glm::vec3 ndcSpace = glm::vec3(clipSpace) / clipSpace.w;
                        ImVec2 screenPos;
                        screenPos.x = (ndcSpace.x + 1.0f) * 0.5f * m_width;
                        screenPos.y = (1.0f - (ndcSpace.y + 1.0f) * 0.5f) * m_height + screenYOffset;

                        if (screenPos.x >= 0.0f && screenPos.x < m_width &&
                            screenPos.y >= 0.0f && screenPos.y < m_height)
                        {
                            drawList->AddText(screenPos, color, text.c_str());
                        }
                    };

                    for (const auto &targetLabel : targetLabels)
                    {
                        const glm::vec3 labelAnchor = targetLabel.position + glm::vec3(0.0f, 3.0f, 0.0f);
                        for (const auto &line : targetLabel.lines)
                        {
                            drawWorldText(labelAnchor, line.text, line.color, line.screenYOffset);
                        }
                    }

                    if (!m_showUI)
                    {
                        const glm::vec3 missileLabelAnchor = m_missile->getPosition() + glm::vec3(0.0f, 3.6f, 0.0f);
                        char buffer[96];
                        std::snprintf(buffer, sizeof(buffer), "ALT %.0f m", std::max(m_missile->getPosition().y, 0.0f));
                        drawWorldText(missileLabelAnchor, buffer, IM_COL32(152, 220, 255, 255), 0.0f);

                        std::snprintf(buffer, sizeof(buffer), "SPD %.0f m/s", glm::length(m_missile->getVelocity()));
                        drawWorldText(missileLabelAnchor, buffer, IM_COL32(200, 245, 255, 255), 16.0f);
                    }
                }

                ImGui::Render();
                ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "ERROR: Exception in ImGui rendering: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "ERROR: Unknown exception in ImGui rendering" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in render: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in render" << std::endl;
    }
}

void Application::renderMinimalHUD()
{
    if (!m_missile || !m_renderer)
    {
        return;
    }

    const float fuel = m_missile->getFuel();
    const float fuelPercent = (m_missileFuel > 0.0f) ? glm::clamp(fuel / m_missileFuel, 0.0f, 1.0f) : 0.0f;
    int activeFlares = 0;
    for (const auto &flare : m_flares)
    {
        if (flare && flare->isActive())
        {
            ++activeFlares;
        }
    }

    Target *trackedTarget = m_missile->getTargetObject();
    if ((trackedTarget == nullptr || !trackedTarget->isActive()) && m_cameraMode == CameraMode::FIGHTER_JET)
    {
        trackedTarget = findBestTarget();
    }

    const bool validTrackedTarget = (trackedTarget != nullptr && trackedTarget->isActive());
    const bool missileWarning = validTrackedTarget && trackedTarget->isMissileWarningActive();
    const char *seekerTrack = m_missile->isTrackingDecoy() ? "FLARE" : "AIRFRAME";

    ImGui::SetNextWindowSize(ImVec2(320.0f, 230.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Flight HUD"))
    {
        ImGui::Text("Fuel: %.1f kg", fuel);
        ImGui::ProgressBar(fuelPercent, ImVec2(-1.0f, 8.0f), "");
        ImGui::Spacing();
        ImGui::Text("Seeker: %s", seekerTrack);
        ImGui::Text("Flares: %d active", activeFlares);
        if (validTrackedTarget)
        {
            ImGui::Text("Defense: %s", missileWarning ? "MAWS active" : "No cue");
            ImGui::Text("Target flares: %d", trackedTarget->getRemainingFlares());
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Camera: %s", getCameraModeLabel());
        if (ImGui::Button("Free"))
        {
            setCameraMode(CameraMode::FREE);
        }
        ImGui::SameLine();
        if (ImGui::Button("Missile"))
        {
            setCameraMode(CameraMode::MISSILE);
        }
        ImGui::SameLine();
        if (ImGui::Button("Fighter Jet"))
        {
            setCameraMode(CameraMode::FIGHTER_JET);
        }
        ImGui::TextDisabled(m_cameraMode == CameraMode::FREE
                                ? "Free cam: RMB look, WASD move, C frame."
                                : "Chase cam: hold RMB to orbit, release to recenter.");
    }
    ImGui::End();

    if (m_cameraMode == CameraMode::FIGHTER_JET)
    {
        constexpr float rwrWindowWidth = 268.0f;
        constexpr float rwrWindowHeight = 340.0f;
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImVec2 windowPos(20.0f, 20.0f);
        if (viewport != nullptr)
        {
            windowPos.x = viewport->WorkPos.x + viewport->WorkSize.x - rwrWindowWidth - 20.0f;
            windowPos.y = viewport->WorkPos.y + ((viewport->WorkSize.y - rwrWindowHeight) * 0.5f);
        }

        ImGui::SetNextWindowPos(windowPos, ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(rwrWindowWidth, rwrWindowHeight), ImGuiCond_Always);
        const ImGuiWindowFlags rwrFlags = ImGuiWindowFlags_NoMove |
                                          ImGuiWindowFlags_NoResize |
                                          ImGuiWindowFlags_NoCollapse |
                                          ImGuiWindowFlags_NoSavedSettings;
        if (ImGui::Begin("RWR", nullptr, rwrFlags))
        {
            const float availableWidth = std::max(ImGui::GetContentRegionAvail().x, 220.0f);
            const float scopeSize = std::min(availableWidth, 220.0f);
            const float scopeOffsetX = std::max(0.0f, (availableWidth - scopeSize) * 0.5f);
            if (scopeOffsetX > 0.0f)
            {
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + scopeOffsetX);
            }

            const ImVec2 scopeOrigin = ImGui::GetCursorScreenPos();
            const ImVec2 scopeDimensions(scopeSize, scopeSize);
            ImGui::InvisibleButton("FighterJetRwrScope", scopeDimensions);

            ImDrawList *drawList = ImGui::GetWindowDrawList();
            const ImVec2 center(scopeOrigin.x + (scopeDimensions.x * 0.5f),
                                scopeOrigin.y + (scopeDimensions.y * 0.5f));
            const float radius = scopeSize * 0.40f;
            const ImU32 scopeBackground = IM_COL32(8, 16, 24, 210);
            const ImU32 ringColor = IM_COL32(84, 132, 156, 235);
            const ImU32 axisColor = IM_COL32(66, 96, 116, 220);
            const ImU32 labelColor = IM_COL32(166, 212, 232, 255);
            const ImU32 cueColor = missileWarning ? IM_COL32(255, 92, 92, 255) : IM_COL32(110, 224, 154, 255);

            drawList->AddCircleFilled(center, radius + 12.0f, scopeBackground, 64);
            drawList->AddCircle(center, radius, ringColor, 64, 2.0f);
            drawList->AddCircle(center, radius * 0.58f, axisColor, 64, 1.0f);
            drawList->AddLine(ImVec2(center.x - radius, center.y), ImVec2(center.x + radius, center.y), axisColor, 1.0f);
            drawList->AddLine(ImVec2(center.x, center.y - radius), ImVec2(center.x, center.y + radius), axisColor, 1.0f);
            drawList->AddTriangleFilled(ImVec2(center.x, center.y - radius - 10.0f),
                                        ImVec2(center.x - 6.0f, center.y - radius + 2.0f),
                                        ImVec2(center.x + 6.0f, center.y - radius + 2.0f),
                                        labelColor);
            drawList->AddText(ImVec2(center.x - 4.0f, center.y - radius - 26.0f), labelColor, "F");
            drawList->AddText(ImVec2(center.x - 4.0f, center.y + radius + 8.0f), labelColor, "B");
            drawList->AddText(ImVec2(center.x - radius - 16.0f, center.y - 6.0f), labelColor, "L");
            drawList->AddText(ImVec2(center.x + radius + 8.0f, center.y - 6.0f), labelColor, "R");

            float bearingDegrees = 0.0f;
            bool hasRwrThreat = validTrackedTarget && trackedTarget->hasThreatAssessment();
            if (hasRwrThreat)
            {
                const glm::vec3 targetForward = safeNormalize(trackedTarget->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
                const glm::vec3 flatForward = safeNormalize(glm::vec3(targetForward.x, 0.0f, targetForward.z), glm::vec3(0.0f, 0.0f, 1.0f));
                const glm::vec3 targetRight = safeNormalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), flatForward), glm::vec3(1.0f, 0.0f, 0.0f));
                const glm::vec3 incomingOffset = trackedTarget->getThreatMissilePosition() - trackedTarget->getPosition();
                glm::vec2 planarCue(-glm::dot(incomingOffset, targetRight), glm::dot(incomingOffset, flatForward));
                const float planarCueLength = glm::length(planarCue);
                if (planarCueLength > 0.001f)
                {
                    planarCue /= planarCueLength;
                }
                else
                {
                    planarCue = glm::vec2(0.0f, 1.0f);
                }

                bearingDegrees = std::fmod(glm::degrees(std::atan2(planarCue.x, planarCue.y)) + 360.0f, 360.0f);
                const float cueRadius = radius * 0.78f;
                const ImVec2 cuePosition(center.x + (planarCue.x * cueRadius),
                                         center.y - (planarCue.y * cueRadius));
                drawList->AddLine(center, cuePosition, IM_COL32(255, 118, 118, 180), 1.5f);
                drawList->AddCircleFilled(cuePosition, 6.0f, cueColor, 18);
                drawList->AddCircle(cuePosition, 12.0f, IM_COL32(255, 160, 160, 160), 24, 1.5f);
            }

            ImGui::Spacing();
            if (hasRwrThreat)
            {
                char buffer[96];
                ImGui::TextColored(ImVec4(1.0f, 0.42f, 0.42f, 1.0f), "MAWS: INBOUND");
                std::snprintf(buffer, sizeof(buffer), "Bearing: %.0f deg", bearingDegrees);
                ImGui::TextUnformatted(buffer);
                std::snprintf(buffer, sizeof(buffer), "Range: %.0f m", trackedTarget->getThreatDistance());
                ImGui::TextUnformatted(buffer);
                std::snprintf(buffer, sizeof(buffer), "TCA: %.1f s  CPA: %.0f m",
                              trackedTarget->getThreatTimeToClosestApproach(),
                              trackedTarget->getThreatClosestApproachDistance());
                ImGui::TextUnformatted(buffer);
            }
            else
            {
                ImGui::TextColored(ImVec4(0.48f, 0.92f, 0.68f, 1.0f), "MAWS: CLEAR");
                ImGui::TextDisabled("No inbound missile inside the MAWS cue window.");
            }
        }
        ImGui::End();
    }
}

float Application::computeEngagementRadius() const
{
    float engagementRadius = std::max(400.0f, m_targetAIConfig.preferredDistance * 1.6f);

    if (m_missile)
    {
        engagementRadius = std::max(engagementRadius, glm::length(glm::vec2(m_missile->getPosition().x, m_missile->getPosition().z)) + 150.0f);
    }

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        engagementRadius = std::max(engagementRadius, glm::length(glm::vec2(target->getPosition().x, target->getPosition().z)) + (target->getRadius() * 8.0f));
    }

    return engagementRadius;
}

void Application::updateEnvironmentScale()
{
    if (!m_renderer)
    {
        return;
    }

    const float engagementRadius = computeEngagementRadius();
    float maxAltitude = std::max(320.0f, glm::clamp(m_targetAIConfig.preferredDistance * 0.22f, 180.0f, 900.0f));

    if (m_missile)
    {
        maxAltitude = std::max(maxAltitude, m_missile->getPosition().y + 150.0f);
    }

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        maxAltitude = std::max(maxAltitude, target->getPosition().y + std::max(120.0f, m_targetAIConfig.preferredDistance * 0.18f));
    }

    const float airspaceHalfExtent = std::max(600.0f, engagementRadius * 1.35f);
    const float groundHalfExtent = std::max(1200.0f, airspaceHalfExtent * 2.4f);
    const float airspaceHeight = std::max(320.0f, std::max(maxAltitude * 1.6f, engagementRadius * 0.45f));
    m_renderer->setEnvironmentMetrics(groundHalfExtent, airspaceHalfExtent, airspaceHeight);
}

void Application::frameEngagementCamera()
{
    if (!m_renderer)
    {
        return;
    }

    std::vector<glm::vec3> points;
    points.reserve(m_targets.size() + 2);
    points.push_back(glm::vec3(0.0f, 0.0f, 0.0f));

    if (m_missile)
    {
        points.push_back(m_missile->getPosition());
    }

    for (const auto &target : m_targets)
    {
        if (target && target->isActive())
        {
            points.push_back(target->getPosition());
        }
    }

    glm::vec3 minPoint = points.front();
    glm::vec3 maxPoint = points.front();
    for (const glm::vec3 &point : points)
    {
        minPoint = glm::min(minPoint, point);
        maxPoint = glm::max(maxPoint, point);
    }

    const glm::vec3 center = 0.5f * (minPoint + maxPoint);
    const float horizontalSpan = glm::length(glm::vec2(maxPoint.x - minPoint.x, maxPoint.z - minPoint.z));
    const float verticalSpan = maxPoint.y - minPoint.y;
    const float framingRadius = std::max({horizontalSpan * 0.60f, verticalSpan * 1.35f, computeEngagementRadius() * 0.65f, 150.0f});

    glm::vec3 cameraDirection = glm::normalize(glm::vec3(-1.15f, 0.60f, 1.05f));
    glm::vec3 cameraPosition = center + cameraDirection * (framingRadius * 1.85f);
    cameraPosition.y = std::max(cameraPosition.y, center.y + framingRadius * 0.60f);

    glm::vec3 lookTarget = center;
    lookTarget.y = std::max(18.0f, center.y + verticalSpan * 0.20f);

    m_renderer->setCameraSpeed(std::clamp(framingRadius * 0.22f, 30.0f, 800.0f));
    m_renderer->setCameraFOV(50.0f);
    m_renderer->setCameraPosition(cameraPosition);
    m_renderer->setCameraTarget(lookTarget);

    if (m_cameraMode == CameraMode::FREE)
    {
        captureFreeCameraState();
    }
}

void Application::setCameraMode(CameraMode mode, bool frameFreeCamera)
{
    if (!m_renderer)
    {
        m_cameraMode = mode;
        return;
    }

    const CameraMode previousMode = m_cameraMode;
    if (mode == previousMode && !(mode == CameraMode::FREE && frameFreeCamera))
    {
        if (mode != CameraMode::FREE)
        {
            updateActiveCameraMode();
        }
        return;
    }

    if (previousMode == CameraMode::FREE)
    {
        captureFreeCameraState();
    }

    if (mode != CameraMode::FREE)
    {
        releaseMouseCameraCapture();
    }

    if (mode != previousMode)
    {
        resetChaseCameraState();
    }

    m_cameraMode = mode;

    if (mode == CameraMode::FREE)
    {
        if (frameFreeCamera || !m_freeCameraState.valid)
        {
            frameEngagementCamera();
        }
        else
        {
            restoreFreeCameraState();
        }
        return;
    }

    updateActiveCameraMode();
}

void Application::updateActiveCameraMode()
{
    switch (m_cameraMode)
    {
    case CameraMode::FREE:
        return;
    case CameraMode::MISSILE:
        updateMissileCamera();
        return;
    case CameraMode::FIGHTER_JET:
        updateFighterJetCamera();
        return;
    }
}

void Application::updateMissileCamera()
{
    if (!m_renderer || !m_missile)
    {
        return;
    }

    const glm::vec3 fallbackForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::vec3 forward = safeNormalize(m_missile->getVelocity(), glm::vec3(0.0f));
    if (glm::length2(forward) < 0.0001f)
    {
        forward = safeNormalize(m_missile->getThrustDirection(), fallbackForward);
    }

    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 missilePosition = m_missile->getPosition();
    const float missileSpeed = glm::length(m_missile->getVelocity());
    const float chaseDistance = std::clamp(8.0f + missileSpeed * 0.04f, 8.0f, 30.0f);
    const float chaseHeight = std::clamp(1.4f + missileSpeed * 0.005f, 1.4f, 7.0f);
    const float lookAhead = std::clamp(25.0f + missileSpeed * 0.12f, 25.0f, 120.0f);

    const glm::vec3 focusPoint = missilePosition + worldUp * 0.8f;
    const glm::vec3 cameraPosition = missilePosition - forward * chaseDistance + worldUp * chaseHeight;
    const glm::vec3 lookTarget = missilePosition + forward * lookAhead + worldUp * 0.8f;

    applyChaseCamera(focusPoint, cameraPosition, lookTarget);
}

void Application::updateFighterJetCamera()
{
    if (!m_renderer)
    {
        return;
    }

    Target *trackedTarget = m_missile ? m_missile->getTargetObject() : nullptr;
    if (trackedTarget == nullptr || !trackedTarget->isActive())
    {
        trackedTarget = findBestTarget();
    }

    if (trackedTarget == nullptr || !trackedTarget->isActive())
    {
        return;
    }

    const glm::vec3 fallbackForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::vec3 forward = safeNormalize(trackedTarget->getVelocity(), glm::vec3(0.0f));
    if (glm::length2(forward) < 0.0001f && m_missile)
    {
        forward = safeNormalize(trackedTarget->getPosition() - m_missile->getPosition(), fallbackForward);
    }
    if (glm::length2(forward) < 0.0001f)
    {
        forward = fallbackForward;
    }

    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 targetPosition = trackedTarget->getPosition();
    const float targetSpeed = glm::length(trackedTarget->getVelocity());
    const float targetRadius = std::max(trackedTarget->getRadius(), 1.0f);
    const float chaseDistance = std::clamp(targetRadius * 4.0f + targetSpeed * 0.05f, 12.0f, 40.0f);
    const float chaseHeight = std::clamp(targetRadius * 1.5f + targetSpeed * 0.008f, 3.0f, 12.0f);
    const float lookAhead = std::clamp(targetRadius * 10.0f + targetSpeed * 0.15f, 20.0f, 150.0f);

    const glm::vec3 focusPoint = targetPosition + worldUp * (targetRadius * 0.35f);
    const glm::vec3 cameraPosition = targetPosition - forward * chaseDistance + worldUp * chaseHeight;
    const glm::vec3 lookTarget = targetPosition + forward * lookAhead + worldUp * (targetRadius * 0.35f);

    applyChaseCamera(focusPoint, cameraPosition, lookTarget);
}

void Application::captureFreeCameraState()
{
    if (!m_renderer)
    {
        return;
    }

    m_freeCameraState.position = m_renderer->getCameraPosition();
    m_freeCameraState.target = m_renderer->getCameraTarget();
    m_freeCameraState.fov = m_renderer->getCameraFOV();
    m_freeCameraState.speed = m_renderer->getCameraSpeed();
    m_freeCameraState.valid = true;
}

void Application::restoreFreeCameraState()
{
    if (!m_renderer || !m_freeCameraState.valid)
    {
        return;
    }

    m_renderer->setCameraFOV(m_freeCameraState.fov);
    m_renderer->setCameraSpeed(m_freeCameraState.speed);
    m_renderer->setCameraPosition(m_freeCameraState.position);
    m_renderer->setCameraTarget(m_freeCameraState.target);
}

void Application::resetChaseCameraState()
{
    m_chaseCameraState = {};
}

void Application::primeChaseCameraState(const glm::vec3 &focusPoint)
{
    if (!m_renderer)
    {
        return;
    }

    glm::vec3 offset = m_renderer->getCameraPosition() - focusPoint;
    float distance = glm::length(offset);
    if (distance <= 0.0001f)
    {
        offset = -safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f)) * 12.0f;
        distance = glm::length(offset);
    }

    const glm::vec3 direction = offset / std::max(distance, 0.0001f);
    m_chaseCameraState.yaw = glm::degrees(std::atan2(direction.z, direction.x));
    m_chaseCameraState.pitch = glm::degrees(std::asin(glm::clamp(direction.y, -1.0f, 1.0f)));
    m_chaseCameraState.distance = distance;
    m_chaseCameraState.returnBlend = 1.0f;
    m_chaseCameraState.initialized = true;
}

void Application::updateChaseOrbit(float yawDeltaDegrees, float pitchDeltaDegrees)
{
    if (m_cameraMode == CameraMode::FREE)
    {
        return;
    }

    if (!m_chaseCameraState.initialized)
    {
        return;
    }

    m_chaseCameraState.yaw += yawDeltaDegrees;
    m_chaseCameraState.pitch = glm::clamp(m_chaseCameraState.pitch + pitchDeltaDegrees, -80.0f, 80.0f);
    m_chaseCameraState.returnBlend = 1.0f;
}

void Application::applyChaseCamera(const glm::vec3 &focusPoint,
                                   const glm::vec3 &defaultPosition,
                                   const glm::vec3 &defaultTarget)
{
    if (!m_renderer)
    {
        return;
    }

    auto buildOrbitPosition = [&]() -> glm::vec3
    {
        const float yaw = glm::radians(m_chaseCameraState.yaw);
        const float pitch = glm::radians(m_chaseCameraState.pitch);
        const float cosPitch = std::cos(pitch);
        glm::vec3 offset(cosPitch * std::cos(yaw),
                         std::sin(pitch),
                         cosPitch * std::sin(yaw));

        if (glm::length2(offset) <= 0.0001f)
        {
            offset = glm::vec3(0.0f, 0.0f, 1.0f);
        }
        else
        {
            offset = glm::normalize(offset);
        }

        return focusPoint + (offset * std::max(m_chaseCameraState.distance, 1.0f));
    };

    if (m_enableMouseCamera)
    {
        if (!m_chaseCameraState.initialized)
        {
            primeChaseCameraState(focusPoint);
        }

        if (m_chaseCameraState.initialized)
        {
            m_renderer->setCameraPosition(buildOrbitPosition());
            m_renderer->setCameraTarget(focusPoint);
            m_chaseCameraState.returnBlend = 1.0f;
            return;
        }
    }

    if (m_chaseCameraState.initialized && m_chaseCameraState.returnBlend > 0.001f)
    {
        const glm::vec3 orbitPosition = buildOrbitPosition();
        const float blend = glm::clamp(m_chaseCameraState.returnBlend, 0.0f, 1.0f);
        m_renderer->setCameraPosition(glm::mix(defaultPosition, orbitPosition, blend));
        m_renderer->setCameraTarget(glm::mix(defaultTarget, focusPoint, blend));

        const float blendDecay = glm::clamp(m_lastFrameDeltaTime, 0.0f, 0.05f) * 3.5f;
        m_chaseCameraState.returnBlend = std::max(0.0f, blend - blendDecay);
        if (m_chaseCameraState.returnBlend <= 0.0f)
        {
            m_chaseCameraState.initialized = false;
        }
        return;
    }

    m_renderer->setCameraPosition(defaultPosition);
    m_renderer->setCameraTarget(defaultTarget);
}

void Application::releaseMouseCameraCapture()
{
    const bool wasCapturing = m_enableMouseCamera;
    m_enableMouseCamera = false;
    m_firstMouse = true;

    if (wasCapturing && m_cameraMode != CameraMode::FREE && m_chaseCameraState.initialized)
    {
        m_chaseCameraState.returnBlend = 1.0f;
    }

    if (m_window)
    {
        glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
}

const char *Application::getCameraModeLabel() const
{
    switch (m_cameraMode)
    {
    case CameraMode::FREE:
        return "Free";
    case CameraMode::MISSILE:
        return "Missile";
    case CameraMode::FIGHTER_JET:
        return "Fighter Jet";
    }

    return "Free";
}

Application::TrajectoryPreviewConfig Application::captureTrajectoryPreviewConfig() const
{
    TrajectoryPreviewConfig config;
    if (!m_missile)
    {
        return config;
    }

    config.thrustDirection = m_missile->getThrustDirection();
    config.dryMass = m_missile->getDryMass();
    config.dragCoefficient = m_missile->getDragCoefficient();
    config.crossSectionalArea = m_missile->getCrossSectionalArea();
    config.liftCoefficient = m_missile->getLiftCoefficient();
    config.guidanceEnabled = m_missile->isGuidanceEnabled();
    config.navigationGain = m_missile->getNavigationGain();
    config.maxSteeringForce = m_missile->getMaxSteeringForce();
    config.trackingAngle = m_missile->getTrackingAngle();
    config.proximityFuseRadius = m_missile->getProximityFuseRadius();
    config.countermeasureResistance = m_missile->getCountermeasureResistance();
    config.terrainAvoidanceEnabled = m_missile->isTerrainAvoidanceEnabled();
    config.terrainClearance = m_missile->getTerrainClearance();
    config.terrainLookAheadTime = m_missile->getTerrainLookAheadTime();
    config.thrust = m_missile->getThrust();
    config.thrustEnabled = m_missile->isThrustEnabled();
    config.fuel = m_missile->getFuel();
    config.fuelConsumptionRate = m_missile->getFuelConsumptionRate();
    config.trajectoryPoints = m_trajectoryPoints;
    config.trajectoryTime = m_trajectoryTime;
    config.gravityMagnitude = m_physicsEngine ? m_physicsEngine->getGravity() : 9.81f;
    config.airDensity = m_physicsEngine ? m_physicsEngine->getAirDensity() : m_savedAirDensity;
    return config;
}

bool Application::shouldRefreshTrajectoryPreviewCache(Target *target, const TrajectoryPreviewConfig &config) const
{
    if (!m_missile || !target)
    {
        return false;
    }

    if (!m_trajectoryPreviewCache.valid ||
        m_trajectoryPreviewCache.target != target ||
        m_trajectoryPreviewCache.missilePoints.empty())
    {
        return true;
    }

    const TrajectoryPreviewConfig &cachedConfig = m_trajectoryPreviewCache.config;
    const bool configChanged =
        cachedConfig.thrustDirection.x != config.thrustDirection.x ||
        cachedConfig.thrustDirection.y != config.thrustDirection.y ||
        cachedConfig.thrustDirection.z != config.thrustDirection.z ||
        cachedConfig.dryMass != config.dryMass ||
        cachedConfig.dragCoefficient != config.dragCoefficient ||
        cachedConfig.crossSectionalArea != config.crossSectionalArea ||
        cachedConfig.liftCoefficient != config.liftCoefficient ||
        cachedConfig.guidanceEnabled != config.guidanceEnabled ||
        cachedConfig.navigationGain != config.navigationGain ||
        cachedConfig.maxSteeringForce != config.maxSteeringForce ||
        cachedConfig.trackingAngle != config.trackingAngle ||
        cachedConfig.proximityFuseRadius != config.proximityFuseRadius ||
        cachedConfig.countermeasureResistance != config.countermeasureResistance ||
        cachedConfig.terrainAvoidanceEnabled != config.terrainAvoidanceEnabled ||
        cachedConfig.terrainClearance != config.terrainClearance ||
        cachedConfig.terrainLookAheadTime != config.terrainLookAheadTime ||
        cachedConfig.thrust != config.thrust ||
        cachedConfig.thrustEnabled != config.thrustEnabled ||
        cachedConfig.fuel != config.fuel ||
        cachedConfig.fuelConsumptionRate != config.fuelConsumptionRate ||
        cachedConfig.trajectoryPoints != config.trajectoryPoints ||
        cachedConfig.trajectoryTime != config.trajectoryTime ||
        cachedConfig.gravityMagnitude != config.gravityMagnitude ||
        cachedConfig.airDensity != config.airDensity;

    if (configChanged)
    {
        return true;
    }

    const auto cacheAge = std::chrono::steady_clock::now() - m_trajectoryPreviewCache.lastRefresh;
    if (cacheAge < m_trajectoryPreviewRefreshInterval)
    {
        return false;
    }

    const glm::vec3 missilePos = m_missile->getPosition();
    const glm::vec3 missileVel = m_missile->getVelocity();
    const glm::vec3 targetPos = target->getPosition();
    const glm::vec3 targetVel = target->getVelocity();

    const float positionThresholdSq = 4.0f;
    const float velocityThresholdSq = 16.0f;
    const bool missileMoved = glm::distance2(missilePos, m_trajectoryPreviewCache.lastMissilePosition) > positionThresholdSq ||
                              glm::distance2(missileVel, m_trajectoryPreviewCache.lastMissileVelocity) > velocityThresholdSq;
    const bool targetMoved = glm::distance2(targetPos, m_trajectoryPreviewCache.lastTargetPosition) > positionThresholdSq ||
                             glm::distance2(targetVel, m_trajectoryPreviewCache.lastTargetVelocity) > velocityThresholdSq;

    return missileMoved || targetMoved;
}

void Application::updateTrajectoryPreviewCache(Target *target, const TrajectoryPreviewConfig &config)
{
    invalidateTrajectoryPreviewCache();
    if (!m_missile || !target || config.trajectoryPoints < 2 || config.trajectoryTime <= 0.0f)
    {
        return;
    }

    const glm::vec3 missilePos = m_missile->getPosition();
    const glm::vec3 missileVel = m_missile->getVelocity();
    const glm::vec3 targetPos = target->getPosition();
    const glm::vec3 targetVel = target->getVelocity();

    m_trajectoryPreviewCache.missilePoints.reserve(static_cast<std::size_t>(config.trajectoryPoints));
    m_trajectoryPreviewCache.targetPoints.reserve(static_cast<std::size_t>(std::max(1, config.trajectoryPoints / 5)));
    m_trajectoryPreviewCache.missilePoints.push_back(missilePos);
    m_trajectoryPreviewCache.interceptPoint = predictInterceptPoint(missilePos, missileVel, targetPos, targetVel);

    Missile simMissile(missilePos, missileVel, config.dryMass, config.dragCoefficient,
                       config.crossSectionalArea, config.liftCoefficient);
    simMissile.setGuidanceEnabled(config.guidanceEnabled);
    simMissile.setNavigationGain(config.navigationGain);
    simMissile.setMaxSteeringForce(config.maxSteeringForce);
    simMissile.setTrackingAngle(config.trackingAngle);
    simMissile.setProximityFuseRadius(config.proximityFuseRadius);
    simMissile.setCountermeasureResistance(config.countermeasureResistance);
    simMissile.setTerrainAvoidanceEnabled(config.terrainAvoidanceEnabled);
    simMissile.setTerrainClearance(config.terrainClearance);
    simMissile.setTerrainLookAheadTime(config.terrainLookAheadTime);
    simMissile.setGroundReferenceAltitude(m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f);
    simMissile.setThrust(config.thrust);
    simMissile.setThrustEnabled(config.thrustEnabled);
    simMissile.setFuel(config.fuel);
    simMissile.setFuelConsumptionRate(config.fuelConsumptionRate);
    simMissile.setThrustDirection(config.thrustDirection);

    const float dt = config.trajectoryTime / static_cast<float>(config.trajectoryPoints);
    Atmosphere atmosphere(config.airDensity);
    Drag drag(&atmosphere);
    Lift lift(&atmosphere);

    Target simTarget = *target;
    std::vector<Missile *> simulatedMissiles = {&simMissile};

    for (int i = 1; i < config.trajectoryPoints; ++i)
    {
        simMissile.resetForces();
        simMissile.applyForce(glm::vec3(0.0f, -config.gravityMagnitude * simMissile.getMass(), 0.0f));
        drag.applyTo(&simMissile);
        lift.applyTo(&simMissile);
        simTarget.updateThreatAssessment(simulatedMissiles);
        simTarget.update(dt);

        if (i % 5 == 0)
        {
            m_trajectoryPreviewCache.targetPoints.push_back(simTarget.getPosition());
        }

        simMissile.setTargetObject(&simTarget);
        if (simMissile.isGuidanceEnabled() && simMissile.hasTarget())
        {
            simMissile.applyGuidance(dt);
            if (simMissile.consumeSelfDestructRequest())
            {
                m_trajectoryPreviewCache.missilePoints.push_back(simMissile.getPosition());
                break;
            }
        }

        simMissile.update(dt);
        m_trajectoryPreviewCache.missilePoints.push_back(simMissile.getPosition());
    }

    m_trajectoryPreviewCache.lastMissilePosition = missilePos;
    m_trajectoryPreviewCache.lastMissileVelocity = missileVel;
    m_trajectoryPreviewCache.lastTargetPosition = targetPos;
    m_trajectoryPreviewCache.lastTargetVelocity = targetVel;
    m_trajectoryPreviewCache.config = config;
    m_trajectoryPreviewCache.target = target;
    m_trajectoryPreviewCache.lastRefresh = std::chrono::steady_clock::now();
    m_trajectoryPreviewCache.valid = !m_trajectoryPreviewCache.missilePoints.empty();
}

void Application::invalidateTrajectoryPreviewCache()
{
    m_trajectoryPreviewCache.missilePoints.clear();
    m_trajectoryPreviewCache.targetPoints.clear();
    m_trajectoryPreviewCache.valid = false;
    m_trajectoryPreviewCache.target = nullptr;
    m_trajectoryPreviewCache.lastRefresh = std::chrono::steady_clock::time_point{};
}

void Application::renderPredictedTrajectory()
{
    if (!m_missile || !m_renderer)
    {
        return;
    }

    try
    {
        Target *target = findBestTarget();
        if (!target)
        {
            invalidateTrajectoryPreviewCache();
            return;
        }

        const TrajectoryPreviewConfig config = captureTrajectoryPreviewConfig();
        if (shouldRefreshTrajectoryPreviewCache(target, config))
        {
            updateTrajectoryPreviewCache(target, config);
        }

        if (!m_trajectoryPreviewCache.valid || m_trajectoryPreviewCache.missilePoints.empty())
        {
            return;
        }

        if (m_showInterceptPoint)
        {
            const glm::vec3 &startPoint = m_trajectoryPreviewCache.missilePoints.front();
            const glm::vec3 &targetPoint = m_trajectoryPreviewCache.lastTargetPosition;
            m_renderer->renderLine(startPoint, m_trajectoryPreviewCache.interceptPoint, glm::vec3(1.0f, 0.0f, 0.0f));
            m_renderer->renderLine(m_trajectoryPreviewCache.interceptPoint, targetPoint, glm::vec3(0.0f, 1.0f, 0.0f));
            m_renderer->renderPoint(m_trajectoryPreviewCache.interceptPoint, glm::vec3(1.0f, 1.0f, 0.0f), 7.0f);
        }

        if (m_showPredictedTargetPath)
        {
            for (const glm::vec3 &predictedTargetPoint : m_trajectoryPreviewCache.targetPoints)
            {
                m_renderer->renderPoint(predictedTargetPoint, glm::vec3(0.0f, 1.0f, 0.0f), 3.0f);
            }
        }

        for (std::size_t i = 1; i < m_trajectoryPreviewCache.missilePoints.size(); ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(m_trajectoryPreviewCache.missilePoints.size());
            const glm::vec3 color(1.0f, 1.0f - t, 0.0f);
            m_renderer->renderLine(m_trajectoryPreviewCache.missilePoints[i - 1], m_trajectoryPreviewCache.missilePoints[i], color);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in renderPredictedTrajectory: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in renderPredictedTrajectory" << std::endl;
    }
}

glm::vec3 Application::predictInterceptPoint(const glm::vec3 &missilePos, const glm::vec3 &missileVel,
                                             const glm::vec3 &targetPos, const glm::vec3 &targetVel)
{
    try
    {
        float missileSpeed = glm::length(missileVel);
        if (missileSpeed < 0.1f)
        {
            return targetPos;
        }

        const glm::vec3 relativePosition = targetPos - missilePos;
        const float missileSpeedSq = missileSpeed * missileSpeed;
        const float a = glm::dot(targetVel, targetVel) - missileSpeedSq;
        const float b = 2.0f * glm::dot(relativePosition, targetVel);
        const float c = glm::dot(relativePosition, relativePosition);
        const float epsilon = 0.0001f;

        float timeToIntercept = 0.0f;
        bool hasInterceptSolution = false;

        if (std::abs(a) < epsilon)
        {
            if (std::abs(b) > epsilon)
            {
                const float candidate = -c / b;
                if (candidate > 0.0f)
                {
                    timeToIntercept = candidate;
                    hasInterceptSolution = true;
                }
            }
        }
        else
        {
            const float discriminant = (b * b) - (4.0f * a * c);
            if (discriminant >= 0.0f)
            {
                const float sqrtDiscriminant = std::sqrt(discriminant);
                const float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
                const float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

                if (t1 > 0.0f && t2 > 0.0f)
                {
                    timeToIntercept = std::min(t1, t2);
                    hasInterceptSolution = true;
                }
                else if (t1 > 0.0f)
                {
                    timeToIntercept = t1;
                    hasInterceptSolution = true;
                }
                else if (t2 > 0.0f)
                {
                    timeToIntercept = t2;
                    hasInterceptSolution = true;
                }
            }
        }

        if (!hasInterceptSolution)
        {
            timeToIntercept = glm::length(relativePosition) / missileSpeed;
        }

        timeToIntercept = glm::clamp(timeToIntercept, 0.0f, 20.0f);
        return targetPos + targetVel * timeToIntercept;
    }
    catch (...)
    {
        return targetPos;
    }
}

void Application::setupUI()
{
    if (!m_missile || !m_renderer || !m_physicsEngine)
    {
        return;
    }

    const ImGuiTreeNodeFlags openByDefault = ImGuiTreeNodeFlags_DefaultOpen;
    const ImGuiTableFlags readoutTableFlags = ImGuiTableFlags_SizingStretchProp |
                                              ImGuiTableFlags_BordersInnerV |
                                              ImGuiTableFlags_RowBg;

    auto aiStateName = [](TargetAIState state) -> const char *
    {
        switch (state)
        {
        case TargetAIState::PATROL:
            return "Patrol";
        case TargetAIState::REPOSITION:
            return "Reposition";
        case TargetAIState::DEFENSIVE:
            return "Defensive";
        case TargetAIState::RECOVERING:
            return "Recover";
        default:
            return "Unknown";
        }
    };

    auto applyLiveMissileConfig = [&]()
    {
        m_missile->setMass(m_mass);
        m_missile->setDragCoefficient(m_dragCoefficient);
        m_missile->setCrossSectionalArea(m_crossSectionalArea);
        m_missile->setLiftCoefficient(m_liftCoefficient);
        m_missile->setGuidanceEnabled(m_guidanceEnabled);
        m_missile->setNavigationGain(m_navigationGain);
        m_missile->setMaxSteeringForce(m_maxSteeringForce);
        m_missile->setTrackingAngle(m_trackingAngle);
        m_missile->setProximityFuseRadius(m_proximityFuseRadius);
        m_missile->setCountermeasureResistance(m_countermeasureResistance);
        m_missile->setTerrainAvoidanceEnabled(m_terrainAvoidanceEnabled);
        m_missile->setTerrainClearance(m_terrainClearance);
        m_missile->setTerrainLookAheadTime(m_terrainLookAheadTime);
        m_missile->setGroundReferenceAltitude(m_physicsEngine->getGroundLevel());
        m_missile->setThrust(m_missileThrust);
        m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);
        if (!m_missileInFlight)
        {
            m_missile->setFuel(m_missileFuel);
        }
    };

    auto applyLiveTargetAIConfig = [&]()
    {
        for (const auto &target : m_targets)
        {
            if (target)
            {
                target->setAIConfig(m_targetAIConfig);
            }
        }
    };

    auto countActiveTargets = [&]() -> int
    {
        int count = 0;
        for (const auto &target : m_targets)
        {
            if (target && target->isActive())
            {
                ++count;
            }
        }
        return count;
    };

    const int activeTargets = countActiveTargets();
    const glm::vec3 missilePosition = m_missile->getPosition();
    const glm::vec3 missileVelocity = m_missile->getVelocity();
    const glm::vec3 missileAcceleration = m_missile->getAcceleration();
    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    const float missileSpeed = glm::length(missileVelocity);
    const float missileAltitude = std::max(missilePosition.y, 0.0f);
    const float terrainClearance = missilePosition.y - m_physicsEngine->getGroundLevel();
    const float missileMass = m_missile->getMass();
    const float missileDryMass = m_missile->getDryMass();
    const float fuel = m_missile->getFuel();
    const float fuelPercent = (m_missileFuel > 0.0f) ? glm::clamp(fuel / m_missileFuel, 0.0f, 1.0f) : 0.0f;
    const bool thrustEnabled = m_missile->isThrustEnabled();
    const bool guidanceEnabled = m_missile->isGuidanceEnabled();
    const bool boosterBurnedOut = !thrustEnabled && fuel <= 0.0f;
    const char *seekerState = m_missile->isTrackingDecoy() ? "Tracking flare" : "Tracking airframe";
    const Atmosphere::State missileAtmosphere = m_physicsEngine->getAtmosphereState(missileAltitude);
    const float missileMach = (missileAtmosphere.speedOfSoundMetersPerSecond > 0.0f)
                                  ? (missileSpeed / missileAtmosphere.speedOfSoundMetersPerSecond)
                                  : 0.0f;

    Target *trackedTarget = m_missile->getTargetObject();
    if ((trackedTarget == nullptr || !trackedTarget->isActive()) && activeTargets > 0)
    {
        trackedTarget = findBestTarget();
    }

    int trackedTargetIndex = -1;
    if (trackedTarget != nullptr)
    {
        for (size_t i = 0; i < m_targets.size(); ++i)
        {
            if (m_targets[i].get() == trackedTarget)
            {
                trackedTargetIndex = static_cast<int>(i) + 1;
                break;
            }
        }
    }

    const bool guidanceLocked = guidanceEnabled && trackedTarget != nullptr && trackedTarget->isActive();
    const bool missileWarning = guidanceLocked && trackedTarget->isMissileWarningActive();
    const float trackedTargetRange = guidanceLocked ? glm::distance(missilePosition, trackedTarget->getPosition()) : 0.0f;

    const char *missionState = "Standby";
    if (m_isPaused)
    {
        missionState = "Paused";
    }
    else if (m_missileInFlight && guidanceLocked && thrustEnabled)
    {
        missionState = "Intercept";
    }
    else if (m_missileInFlight && guidanceLocked)
    {
        missionState = "Glide Track";
    }
    else if (m_missileInFlight && thrustEnabled)
    {
        missionState = "Boost";
    }
    else if (m_missileInFlight)
    {
        missionState = "Ballistic";
    }

    auto drawReadoutRow = [](const char *label, const char *value)
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(value);
    };

    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360.0f, 250.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Simulation"))
    {
        if (ImGui::Button(m_isPaused ? "Resume" : "Pause"))
        {
            m_isPaused = !m_isPaused;
        }
        ImGui::SameLine();
        if (ImGui::Button("Launch"))
        {
            launchMissile();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Missile"))
        {
            resetMissile();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Targets"))
        {
            resetTargets();
        }
        if (ImGui::Button("Frame Camera"))
        {
            setCameraMode(CameraMode::FREE, true);
        }

        ImGui::Separator();
        ImGui::SliderFloat("Simulation speed", &m_simulationSpeed, 0.1f, 10.0f, "%.1fx");

        float gravity = m_physicsEngine->getGravity();
        if (ImGui::SliderFloat("Gravity", &gravity, 0.0f, 20.0f, "%.2f m/s^2"))
        {
            m_physicsEngine->setGravity(gravity);
        }

        float airDensity = m_physicsEngine->getAirDensity();
        if (ImGui::SliderFloat("Sea-level density", &airDensity, 0.0f, 2.0f, "%.3f kg/m^3"))
        {
            m_physicsEngine->setAirDensity(airDensity);
        }

        if (ImGui::Checkbox("Ground collision enabled", &m_groundEnabled))
        {
            m_physicsEngine->setGroundEnabled(m_groundEnabled);
        }

        if (m_groundEnabled)
        {
            if (ImGui::SliderFloat("Ground restitution", &m_groundRestitution, 0.0f, 1.0f, "%.2f"))
            {
                m_physicsEngine->setGroundRestitution(m_groundRestitution);
            }
        }
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(20.0f, 290.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(420.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Missile Config"))
    {
        ImGui::InputFloat3("Spawn position", m_initialPosition);
        ImGui::InputFloat3("Launch velocity", m_initialVelocity);
        ImGui::Separator();
        ImGui::SliderFloat("Dry mass", &m_mass, 10.0f, 1000.0f, "%.1f kg");
        ImGui::SliderFloat("Drag coefficient", &m_dragCoefficient, 0.01f, 1.0f, "%.3f");
        ImGui::SliderFloat("Cross-sectional area", &m_crossSectionalArea, 0.01f, 1.0f, "%.3f m^2");
        ImGui::SliderFloat("Lift coefficient", &m_liftCoefficient, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Thrust output", &m_missileThrust, 1000.0f, 50000.0f, "%.0f N");
        ImGui::SliderFloat("Fuel load", &m_missileFuel, 10.0f, 1000.0f, "%.1f kg");
        ImGui::SliderFloat("Fuel burn rate", &m_missileFuelConsumptionRate, 0.1f, 10.0f, "%.2f kg/s");
        ImGui::Checkbox("Guidance enabled", &m_guidanceEnabled);
        ImGui::SliderFloat("Lead aggressiveness", &m_navigationGain, 1.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Max steering force", &m_maxSteeringForce, 1000.0f, 50000.0f, "%.0f N");
        ImGui::SliderFloat("Tracking angle", &m_trackingAngle, 5.0f, 180.0f, "%.0f deg");
        ImGui::SliderFloat("Proximity fuse", &m_proximityFuseRadius, 0.0f, 75.0f, "%.1f m");
        ImGui::SliderFloat("IRCCM resistance", &m_countermeasureResistance, 0.0f, 1.0f, "%.2f");
        ImGui::Checkbox("Terrain avoidance", &m_terrainAvoidanceEnabled);
        ImGui::SliderFloat("Terrain clearance", &m_terrainClearance, 0.0f, 400.0f, "%.1f m");
        ImGui::SliderFloat("Terrain look-ahead", &m_terrainLookAheadTime, 0.5f, 12.0f, "%.1f s");

        if (ImGui::Button("Apply To Live Missile"))
        {
            applyLiveMissileConfig();
        }
        ImGui::SameLine();
        if (ImGui::Button("Rearm Missile"))
        {
            resetMissile();
        }
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(460.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 420.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Targets"))
    {
        ImGui::Text("Active targets: %d / %zu", activeTargets, m_targets.size());
        ImGui::SliderInt("Target count", &m_targetCount, 1, 20);
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            resetTargets();
        }

        ImGui::SliderFloat("Average distance", &m_targetAIConfig.preferredDistance, 300.0f, 20000.0f, "%.0f m");
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            resetTargets();
        }

        ImGui::SliderFloat("Minimum speed", &m_targetAIConfig.minSpeed, 60.0f, 450.0f, "%.0f m/s");
        m_targetAIConfig.maxSpeed = std::max(m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f);
        ImGui::SliderFloat("Maximum speed", &m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f, 600.0f, "%.0f m/s");

        if (ImGui::Button("Apply Target AI"))
        {
            applyLiveTargetAIConfig();
        }
        ImGui::SameLine();
        if (ImGui::Button("Rebuild Targets"))
        {
            resetTargets();
        }

        if (!m_targets.empty() && ImGui::BeginTable("TargetRosterWindowTable", 7, readoutTableFlags))
        {
            ImGui::TableSetupColumn("ID");
            ImGui::TableSetupColumn("State");
            ImGui::TableSetupColumn("AI");
            ImGui::TableSetupColumn("Altitude");
            ImGui::TableSetupColumn("Speed");
            ImGui::TableSetupColumn("Range");
            ImGui::TableSetupColumn("Flares");
            ImGui::TableHeadersRow();

            for (size_t i = 0; i < m_targets.size(); ++i)
            {
                const auto &target = m_targets[i];
                if (!target)
                {
                    continue;
                }

                const bool isActive = target->isActive();
                const float targetAltitude = std::max(target->getPosition().y, 0.0f);
                const float targetSpeed = glm::length(target->getVelocity());
                const float targetRange = glm::distance(missilePosition, target->getPosition());

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", i + 1);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(isActive ? "Active" : "Destroyed");
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(aiStateName(target->getAIState()));
                ImGui::TableSetColumnIndex(3);
                if (isActive)
                {
                    ImGui::Text("%.0f m", targetAltitude);
                }
                else
                {
                    ImGui::TextDisabled("--");
                }
                ImGui::TableSetColumnIndex(4);
                if (isActive)
                {
                    ImGui::Text("%.0f m/s", targetSpeed);
                }
                else
                {
                    ImGui::TextDisabled("--");
                }
                ImGui::TableSetColumnIndex(5);
                if (isActive)
                {
                    ImGui::Text("%.1f m", targetRange);
                }
                else
                {
                    ImGui::TextDisabled("--");
                }
                ImGui::TableSetColumnIndex(6);
                if (isActive)
                {
                    ImGui::Text("%d", target->getRemainingFlares());
                }
                else
                {
                    ImGui::TextDisabled("--");
                }
            }

            ImGui::EndTable();
        }
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(460.0f, 460.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360.0f, 260.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("View"))
    {
        float fov = m_renderer->getCameraFOV();
        if (ImGui::SliderFloat("Field of view", &fov, 10.0f, 120.0f, "%.1f deg"))
        {
            m_renderer->setCameraFOV(fov);
        }

        float cameraSpeed = m_renderer->getCameraSpeed();
        if (ImGui::SliderFloat("Camera speed", &cameraSpeed, 1.0f, 800.0f, "%.0f"))
        {
            m_renderer->setCameraSpeed(cameraSpeed);
        }

        ImGui::Checkbox("Show predicted trajectory", &m_showTrajectory);
        ImGui::Checkbox("Show target labels", &m_showTargetInfo);
        ImGui::Checkbox("Show target prediction path", &m_showPredictedTargetPath);
        ImGui::Checkbox("Show intercept point", &m_showInterceptPoint);
        ImGui::SliderInt("Trajectory detail", &m_trajectoryPoints, 10, 600);
        ImGui::SliderFloat("Trajectory horizon", &m_trajectoryTime, 0.5f, 60.0f, "%.1f s");

        ImGui::TextDisabled("Camera: %.1f, %.1f, %.1f", cameraPosition.x, cameraPosition.y, cameraPosition.z);
        ImGui::TextDisabled("Controls: RMB look, WASD move (Free cam), Space/Ctrl vertical, Enter pause, F launch, C frame free cam, Tab toggle UI");
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(840.0f, 460.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(420.0f, 360.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Telemetry"))
    {
        char buffer[128];

        if (ImGui::BeginTable("MissionTelemetryWindowTable", 2, readoutTableFlags))
        {
            drawReadoutRow("Mission state", missionState);
            std::snprintf(buffer, sizeof(buffer), "%d / %zu", activeTargets, m_targets.size());
            drawReadoutRow("Targets active", buffer);
            if (trackedTargetIndex > 0)
            {
                std::snprintf(buffer, sizeof(buffer), "Target %d", trackedTargetIndex);
                drawReadoutRow("Tracked target", buffer);
            }
            else
            {
                drawReadoutRow("Tracked target", "None");
            }
            if (guidanceLocked)
            {
                std::snprintf(buffer, sizeof(buffer), "%.1f m", trackedTargetRange);
                drawReadoutRow("Target range", buffer);
            }
            else
            {
                drawReadoutRow("Target range", "No lock");
            }
            if (m_closestTargetDistance < 999999.0f)
            {
                std::snprintf(buffer, sizeof(buffer), "%.1f m", m_closestTargetDistance);
                drawReadoutRow("Closest pass", buffer);
            }
            else
            {
                drawReadoutRow("Closest pass", "Not available");
            }
            std::snprintf(buffer, sizeof(buffer), "%.1f s", m_missileFlightTime);
            drawReadoutRow("Flight time", buffer);
            ImGui::EndTable();
        }

        if (ImGui::BeginTable("MissileTelemetryWindowTable", 2, readoutTableFlags))
        {
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m", missilePosition.x, missilePosition.y, missilePosition.z);
            drawReadoutRow("Position", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m/s", missileVelocity.x, missileVelocity.y, missileVelocity.z);
            drawReadoutRow("Velocity", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m/s^2", missileAcceleration.x, missileAcceleration.y, missileAcceleration.z);
            drawReadoutRow("Acceleration", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f m/s", missileSpeed);
            drawReadoutRow("Speed", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.2f", missileMach);
            drawReadoutRow("Mach", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f m", missileAltitude);
            drawReadoutRow("Altitude", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f m", terrainClearance);
            drawReadoutRow("Terrain clearance", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileMass);
            drawReadoutRow("Current mass", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileDryMass);
            drawReadoutRow("Dry mass", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f kg", fuel);
            drawReadoutRow("Fuel remaining", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.0f%%", fuelPercent * 100.0f);
            drawReadoutRow("Fuel percent", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.3f kg/m^3", missileAtmosphere.densityKgPerCubicMeter);
            drawReadoutRow("Ambient density", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.2f kPa", missileAtmosphere.pressurePascals * 0.001f);
            drawReadoutRow("Ambient pressure", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.2f K", missileAtmosphere.temperatureKelvin);
            drawReadoutRow("Air temperature", buffer);
            ImGui::EndTable();
        }

        if (ImGui::BeginTable("SystemTelemetryWindowTable", 2, readoutTableFlags))
        {
            drawReadoutRow("Booster", thrustEnabled ? "Active" : (boosterBurnedOut ? "Burned out" : "Off"));
            drawReadoutRow("Guidance", guidanceLocked ? "Locked" : (guidanceEnabled ? "Searching" : "Disabled"));
            drawReadoutRow("Seeker", seekerState);
            drawReadoutRow("Target defense", missileWarning ? "MAWS active" : "No cue");
            std::snprintf(buffer, sizeof(buffer), "%.0f N", m_missile->getThrust());
            drawReadoutRow("Thrust command", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.2f kg/s", m_missile->getFuelConsumptionRate());
            drawReadoutRow("Burn rate", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f", cameraPosition.x, cameraPosition.y, cameraPosition.z);
            drawReadoutRow("Camera position", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f deg", m_renderer->getCameraFOV());
            drawReadoutRow("Camera FOV", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.1f", m_renderer->getCameraSpeed());
            drawReadoutRow("Camera speed", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.2f m/s^2", m_physicsEngine->getGravity());
            drawReadoutRow("Gravity", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.3f kg/m^3", m_physicsEngine->getAirDensity());
            drawReadoutRow("Sea-level density", buffer);
            ImGui::EndTable();
        }
    }
    ImGui::End();

    const std::string currentSettingsSnapshot = buildSettingsSnapshot();
    if (currentSettingsSnapshot != m_lastSettingsSnapshot)
    {
        scheduleSettingsSave();
        m_lastSettingsSnapshot = currentSettingsSnapshot;
    }
    return;
#if 0
    ImGui::SetNextWindowSize(ImVec2(540.0f, 760.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("MissileSim Debug"))
    {
        if (ImGui::Button(m_isPaused ? "Resume" : "Pause"))
        {
            m_isPaused = !m_isPaused;
        }
        ImGui::SameLine();
        if (ImGui::Button("Launch"))
        {
            launchMissile();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Missile"))
        {
            resetMissile();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Targets"))
        {
            resetTargets();
        }
        ImGui::SameLine();
        if (ImGui::Button("Frame Camera"))
        {
            setCameraMode(CameraMode::FREE, true);
        }

        ImGui::Separator();

        if (ImGui::CollapsingHeader("World", openByDefault))
        {
            ImGui::SliderFloat("Simulation speed", &m_simulationSpeed, 0.1f, 10.0f, "%.1fx");

            float gravity = m_physicsEngine->getGravity();
            if (ImGui::SliderFloat("Gravity", &gravity, 0.0f, 20.0f, "%.2f m/s^2"))
            {
                m_physicsEngine->setGravity(gravity);
            }

            float airDensity = m_physicsEngine->getAirDensity();
            if (ImGui::SliderFloat("Sea-level density", &airDensity, 0.0f, 2.0f, "%.3f kg/m^3"))
            {
                m_physicsEngine->setAirDensity(airDensity);
            }

            if (ImGui::Checkbox("Ground collision enabled", &m_groundEnabled))
            {
                m_physicsEngine->setGroundEnabled(m_groundEnabled);
            }

            if (m_groundEnabled)
            {
                if (ImGui::SliderFloat("Ground restitution", &m_groundRestitution, 0.0f, 1.0f, "%.2f"))
                {
                    m_physicsEngine->setGroundRestitution(m_groundRestitution);
                }
            }
        }

        if (ImGui::CollapsingHeader("Missile Config", openByDefault))
        {
            ImGui::InputFloat3("Spawn position", m_initialPosition);
            ImGui::InputFloat3("Launch velocity", m_initialVelocity);
            ImGui::Separator();
            ImGui::SliderFloat("Dry mass", &m_mass, 10.0f, 1000.0f, "%.1f kg");
            ImGui::SliderFloat("Drag coefficient", &m_dragCoefficient, 0.01f, 1.0f, "%.3f");
            ImGui::SliderFloat("Cross-sectional area", &m_crossSectionalArea, 0.01f, 1.0f, "%.3f m^2");
            ImGui::SliderFloat("Lift coefficient", &m_liftCoefficient, 0.0f, 1.0f, "%.3f");
            ImGui::SliderFloat("Thrust output", &m_missileThrust, 1000.0f, 50000.0f, "%.0f N");
            ImGui::SliderFloat("Fuel load", &m_missileFuel, 10.0f, 1000.0f, "%.1f kg");
            ImGui::SliderFloat("Fuel burn rate", &m_missileFuelConsumptionRate, 0.1f, 10.0f, "%.2f kg/s");
            ImGui::Checkbox("Guidance enabled", &m_guidanceEnabled);
            ImGui::SliderFloat("Lead aggressiveness", &m_navigationGain, 1.0f, 4.0f, "%.2f");
            ImGui::SliderFloat("Max steering force", &m_maxSteeringForce, 1000.0f, 50000.0f, "%.0f N");
            ImGui::SliderFloat("Tracking angle", &m_trackingAngle, 5.0f, 180.0f, "%.0f deg");
            ImGui::SliderFloat("Proximity fuse", &m_proximityFuseRadius, 0.0f, 75.0f, "%.1f m");
            ImGui::SliderFloat("IRCCM resistance", &m_countermeasureResistance, 0.0f, 1.0f, "%.2f");
            ImGui::Checkbox("Terrain avoidance", &m_terrainAvoidanceEnabled);
            ImGui::SliderFloat("Terrain clearance", &m_terrainClearance, 0.0f, 400.0f, "%.1f m");
            ImGui::SliderFloat("Terrain look-ahead", &m_terrainLookAheadTime, 0.5f, 12.0f, "%.1f s");

            if (ImGui::Button("Apply To Live Missile"))
            {
                applyLiveMissileConfig();
            }
            ImGui::SameLine();
            if (ImGui::Button("Rearm Missile"))
            {
                resetMissile();
            }
        }

        if (ImGui::CollapsingHeader("Targets", openByDefault))
        {
            const int activeTargets = countActiveTargets();
            ImGui::Text("Active targets: %d / %zu", activeTargets, m_targets.size());

            ImGui::SliderInt("Target count", &m_targetCount, 1, 20);
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                resetTargets();
            }

            ImGui::SliderFloat("Average distance", &m_targetAIConfig.preferredDistance, 300.0f, 20000.0f, "%.0f m");
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                resetTargets();
            }

            ImGui::SliderFloat("Minimum speed", &m_targetAIConfig.minSpeed, 60.0f, 450.0f, "%.0f m/s");
            m_targetAIConfig.maxSpeed = std::max(m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f);
            ImGui::SliderFloat("Maximum speed", &m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f, 600.0f, "%.0f m/s");

            if (ImGui::Button("Apply Target AI"))
            {
                applyLiveTargetAIConfig();
            }
            ImGui::SameLine();
            if (ImGui::Button("Rebuild Targets"))
            {
                resetTargets();
            }

            if (!m_targets.empty())
            {
                const glm::vec3 missilePosition = m_missile->getPosition();
                if (ImGui::BeginTable("TargetRosterTable", 7, readoutTableFlags))
                {
                    ImGui::TableSetupColumn("ID");
                    ImGui::TableSetupColumn("State");
                    ImGui::TableSetupColumn("AI");
                    ImGui::TableSetupColumn("Altitude");
                    ImGui::TableSetupColumn("Speed");
                    ImGui::TableSetupColumn("Range");
                    ImGui::TableSetupColumn("Flares");
                    ImGui::TableHeadersRow();

                    for (size_t i = 0; i < m_targets.size(); ++i)
                    {
                        const auto &target = m_targets[i];
                        if (!target)
                        {
                            continue;
                        }

                        const bool isActive = target->isActive();
                        const float targetAltitude = std::max(target->getPosition().y, 0.0f);
                        const float targetSpeed = glm::length(target->getVelocity());
                        const float targetRange = glm::distance(missilePosition, target->getPosition());

                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%zu", i + 1);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(isActive ? "Active" : "Destroyed");
                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextUnformatted(aiStateName(target->getAIState()));
                        ImGui::TableSetColumnIndex(3);
                        if (isActive) { ImGui::Text("%.0f m", targetAltitude); } else { ImGui::TextDisabled("--"); }
                        ImGui::TableSetColumnIndex(4);
                        if (isActive) { ImGui::Text("%.0f m/s", targetSpeed); } else { ImGui::TextDisabled("--"); }
                        ImGui::TableSetColumnIndex(5);
                        if (isActive) { ImGui::Text("%.1f m", targetRange); } else { ImGui::TextDisabled("--"); }
                        ImGui::TableSetColumnIndex(6);
                        if (isActive) { ImGui::Text("%d", target->getRemainingFlares()); } else { ImGui::TextDisabled("--"); }
                    }

                    ImGui::EndTable();
                }
            }
        }

        if (ImGui::CollapsingHeader("View", openByDefault))
        {
            float fov = m_renderer->getCameraFOV();
            if (ImGui::SliderFloat("Field of view", &fov, 10.0f, 120.0f, "%.1f deg"))
            {
                m_renderer->setCameraFOV(fov);
            }

            float cameraSpeed = m_renderer->getCameraSpeed();
            if (ImGui::SliderFloat("Camera speed", &cameraSpeed, 1.0f, 800.0f, "%.0f"))
            {
                m_renderer->setCameraSpeed(cameraSpeed);
            }

            ImGui::Checkbox("Show predicted trajectory", &m_showTrajectory);
            ImGui::Checkbox("Show target labels", &m_showTargetInfo);
            ImGui::Checkbox("Show target prediction path", &m_showPredictedTargetPath);
            ImGui::Checkbox("Show intercept point", &m_showInterceptPoint);
            ImGui::SliderInt("Trajectory detail", &m_trajectoryPoints, 10, 600);
            ImGui::SliderFloat("Trajectory horizon", &m_trajectoryTime, 0.5f, 60.0f, "%.1f s");

            const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
            ImGui::TextDisabled("Camera: %.1f, %.1f, %.1f", cameraPosition.x, cameraPosition.y, cameraPosition.z);
            ImGui::TextDisabled("Controls: RMB look, WASD move (Free cam), Space/Ctrl vertical, Enter pause, F launch, C frame free cam, Tab toggle UI");
        }

        const int activeTargets = countActiveTargets();
        const glm::vec3 missilePosition = m_missile->getPosition();
        const glm::vec3 missileVelocity = m_missile->getVelocity();
        const glm::vec3 missileAcceleration = m_missile->getAcceleration();
        const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
        const float missileSpeed = glm::length(missileVelocity);
        const float missileAltitude = std::max(missilePosition.y, 0.0f);
        const float terrainClearance = missilePosition.y - m_physicsEngine->getGroundLevel();
        const float missileMass = m_missile->getMass();
        const float missileDryMass = m_missile->getDryMass();
        const float fuel = m_missile->getFuel();
        const float fuelPercent = (m_missileFuel > 0.0f) ? glm::clamp(fuel / m_missileFuel, 0.0f, 1.0f) : 0.0f;
        const bool thrustEnabled = m_missile->isThrustEnabled();
        const bool guidanceEnabled = m_missile->isGuidanceEnabled();
        const bool boosterBurnedOut = !thrustEnabled && fuel <= 0.0f;
        const char *seekerState = m_missile->isTrackingDecoy() ? "Tracking flare" : "Tracking airframe";
        Atmosphere::State missileAtmosphere = m_physicsEngine->getAtmosphereState(missileAltitude);
        const float missileMach = (missileAtmosphere.speedOfSoundMetersPerSecond > 0.0f)
                                      ? (missileSpeed / missileAtmosphere.speedOfSoundMetersPerSecond)
                                      : 0.0f;

        Target *trackedTarget = m_missile->getTargetObject();
        if ((trackedTarget == nullptr || !trackedTarget->isActive()) && activeTargets > 0)
        {
            trackedTarget = findBestTarget();
        }

        int trackedTargetIndex = -1;
        if (trackedTarget != nullptr)
        {
            for (size_t i = 0; i < m_targets.size(); ++i)
            {
                if (m_targets[i].get() == trackedTarget)
                {
                    trackedTargetIndex = static_cast<int>(i) + 1;
                    break;
                }
            }
        }

        const bool guidanceLocked = guidanceEnabled && trackedTarget != nullptr && trackedTarget->isActive();
        const bool missileWarning = guidanceLocked && trackedTarget->isMissileWarningActive();
        const float trackedTargetRange = guidanceLocked ? glm::distance(missilePosition, trackedTarget->getPosition()) : 0.0f;

        const char *missionState = "Standby";
        if (m_isPaused) { missionState = "Paused"; }
        else if (m_missileInFlight && guidanceLocked && thrustEnabled) { missionState = "Intercept"; }
        else if (m_missileInFlight && guidanceLocked) { missionState = "Glide Track"; }
        else if (m_missileInFlight && thrustEnabled) { missionState = "Boost"; }
        else if (m_missileInFlight) { missionState = "Ballistic"; }

        auto drawReadoutRow = [](const char *label, const char *value)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(value);
        };

        if (ImGui::CollapsingHeader("Telemetry", openByDefault))
        {
            char buffer[128];

            if (ImGui::BeginTable("MissionTelemetryTable", 2, readoutTableFlags))
            {
                drawReadoutRow("Mission state", missionState);
                std::snprintf(buffer, sizeof(buffer), "%d / %zu", activeTargets, m_targets.size());
                drawReadoutRow("Targets active", buffer);
                if (trackedTargetIndex > 0) { std::snprintf(buffer, sizeof(buffer), "Target %d", trackedTargetIndex); drawReadoutRow("Tracked target", buffer); }
                else { drawReadoutRow("Tracked target", "None"); }
                if (guidanceLocked) { std::snprintf(buffer, sizeof(buffer), "%.1f m", trackedTargetRange); drawReadoutRow("Target range", buffer); }
                else { drawReadoutRow("Target range", "No lock"); }
                if (m_closestTargetDistance < 999999.0f) { std::snprintf(buffer, sizeof(buffer), "%.1f m", m_closestTargetDistance); drawReadoutRow("Closest pass", buffer); }
                else { drawReadoutRow("Closest pass", "Not available"); }
                std::snprintf(buffer, sizeof(buffer), "%.1f s", m_missileFlightTime);
                drawReadoutRow("Flight time", buffer);
                ImGui::EndTable();
            }

            if (ImGui::BeginTable("MissileTelemetryTable", 2, readoutTableFlags))
            {
                std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m", missilePosition.x, missilePosition.y, missilePosition.z);
                drawReadoutRow("Position", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m/s", missileVelocity.x, missileVelocity.y, missileVelocity.z);
                drawReadoutRow("Velocity", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m/s^2", missileAcceleration.x, missileAcceleration.y, missileAcceleration.z);
                drawReadoutRow("Acceleration", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f m/s", missileSpeed);
                drawReadoutRow("Speed", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.2f", missileMach);
                drawReadoutRow("Mach", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f m", missileAltitude);
                drawReadoutRow("Altitude", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f m", terrainClearance);
                drawReadoutRow("Terrain clearance", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileMass);
                drawReadoutRow("Current mass", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileDryMass);
                drawReadoutRow("Dry mass", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f kg", fuel);
                drawReadoutRow("Fuel remaining", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.0f%%", fuelPercent * 100.0f);
                drawReadoutRow("Fuel percent", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.3f kg/m^3", missileAtmosphere.densityKgPerCubicMeter);
                drawReadoutRow("Ambient density", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.2f kPa", missileAtmosphere.pressurePascals * 0.001f);
                drawReadoutRow("Ambient pressure", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.2f K", missileAtmosphere.temperatureKelvin);
                drawReadoutRow("Air temperature", buffer);
                ImGui::EndTable();
            }

            if (ImGui::BeginTable("SystemTelemetryTable", 2, readoutTableFlags))
            {
                drawReadoutRow("Booster", thrustEnabled ? "Active" : (boosterBurnedOut ? "Burned out" : "Off"));
                drawReadoutRow("Guidance", guidanceLocked ? "Locked" : (guidanceEnabled ? "Searching" : "Disabled"));
                drawReadoutRow("Seeker", seekerState);
                drawReadoutRow("Target defense", missileWarning ? "MAWS active" : "No cue");
                std::snprintf(buffer, sizeof(buffer), "%.0f N", m_missile->getThrust());
                drawReadoutRow("Thrust command", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.2f kg/s", m_missile->getFuelConsumptionRate());
                drawReadoutRow("Burn rate", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f", cameraPosition.x, cameraPosition.y, cameraPosition.z);
                drawReadoutRow("Camera position", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f deg", m_renderer->getCameraFOV());
                drawReadoutRow("Camera FOV", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.1f", m_renderer->getCameraSpeed());
                drawReadoutRow("Camera speed", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.2f m/s^2", m_physicsEngine->getGravity());
                drawReadoutRow("Gravity", buffer);
                std::snprintf(buffer, sizeof(buffer), "%.3f kg/m^3", m_physicsEngine->getAirDensity());
                drawReadoutRow("Sea-level density", buffer);
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();

    const std::string currentSettingsSnapshot = buildSettingsSnapshot();
    if (currentSettingsSnapshot != m_lastSettingsSnapshot)
    {
        scheduleSettingsSave();
        m_lastSettingsSnapshot = currentSettingsSnapshot;
    }
#endif
    return;
#if 0
    {
        static bool themeInitialized = false;

        ImGuiStyle &deckStyle = ImGui::GetStyle();
        if (!themeInitialized)
        {
            ImGui::StyleColorsDark();

            deckStyle.WindowPadding = ImVec2(14.0f, 14.0f);
            deckStyle.FramePadding = ImVec2(12.0f, 7.0f);
            deckStyle.CellPadding = ImVec2(8.0f, 6.0f);
            deckStyle.ItemSpacing = ImVec2(10.0f, 9.0f);
            deckStyle.ItemInnerSpacing = ImVec2(8.0f, 6.0f);
            deckStyle.IndentSpacing = 18.0f;
            deckStyle.ScrollbarSize = 14.0f;
            deckStyle.GrabMinSize = 10.0f;
            deckStyle.WindowRounding = 12.0f;
            deckStyle.ChildRounding = 10.0f;
            deckStyle.FrameRounding = 8.0f;
            deckStyle.PopupRounding = 10.0f;
            deckStyle.ScrollbarRounding = 10.0f;
            deckStyle.GrabRounding = 8.0f;
            deckStyle.TabRounding = 8.0f;
            deckStyle.WindowBorderSize = 1.0f;
            deckStyle.ChildBorderSize = 1.0f;
            deckStyle.FrameBorderSize = 1.0f;
            deckStyle.TabBorderSize = 0.0f;
            deckStyle.WindowTitleAlign = ImVec2(0.02f, 0.5f);

            ImVec4 *colors = deckStyle.Colors;
            colors[ImGuiCol_Text] = ImVec4(0.93f, 0.95f, 0.98f, 1.0f);
            colors[ImGuiCol_TextDisabled] = ImVec4(0.56f, 0.63f, 0.71f, 1.0f);
            colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.07f, 0.10f, 0.97f);
            colors[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.12f, 0.16f, 1.0f);
            colors[ImGuiCol_PopupBg] = ImVec4(0.07f, 0.09f, 0.13f, 0.98f);
            colors[ImGuiCol_Border] = ImVec4(0.17f, 0.24f, 0.31f, 1.0f);
            colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
            colors[ImGuiCol_FrameBg] = ImVec4(0.11f, 0.15f, 0.20f, 1.0f);
            colors[ImGuiCol_FrameBgHovered] = ImVec4(0.15f, 0.21f, 0.28f, 1.0f);
            colors[ImGuiCol_FrameBgActive] = ImVec4(0.18f, 0.25f, 0.34f, 1.0f);
            colors[ImGuiCol_TitleBg] = ImVec4(0.07f, 0.10f, 0.14f, 1.0f);
            colors[ImGuiCol_TitleBgActive] = ImVec4(0.09f, 0.13f, 0.18f, 1.0f);
            colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.05f, 0.07f, 0.10f, 0.90f);
            colors[ImGuiCol_MenuBarBg] = ImVec4(0.08f, 0.11f, 0.15f, 1.0f);
            colors[ImGuiCol_ScrollbarBg] = ImVec4(0.05f, 0.07f, 0.09f, 0.70f);
            colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.22f, 0.31f, 0.41f, 1.0f);
            colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.29f, 0.40f, 0.52f, 1.0f);
            colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.35f, 0.47f, 0.60f, 1.0f);
            colors[ImGuiCol_CheckMark] = ImVec4(0.38f, 0.72f, 0.94f, 1.0f);
            colors[ImGuiCol_SliderGrab] = ImVec4(0.33f, 0.66f, 0.88f, 1.0f);
            colors[ImGuiCol_SliderGrabActive] = ImVec4(0.43f, 0.77f, 0.97f, 1.0f);
            colors[ImGuiCol_Button] = ImVec4(0.13f, 0.20f, 0.27f, 1.0f);
            colors[ImGuiCol_ButtonHovered] = ImVec4(0.18f, 0.28f, 0.37f, 1.0f);
            colors[ImGuiCol_ButtonActive] = ImVec4(0.22f, 0.34f, 0.45f, 1.0f);
            colors[ImGuiCol_Header] = ImVec4(0.12f, 0.18f, 0.24f, 1.0f);
            colors[ImGuiCol_HeaderHovered] = ImVec4(0.18f, 0.25f, 0.33f, 1.0f);
            colors[ImGuiCol_HeaderActive] = ImVec4(0.22f, 0.31f, 0.40f, 1.0f);
            colors[ImGuiCol_Separator] = ImVec4(0.16f, 0.23f, 0.30f, 1.0f);
            colors[ImGuiCol_SeparatorHovered] = ImVec4(0.29f, 0.49f, 0.68f, 1.0f);
            colors[ImGuiCol_SeparatorActive] = ImVec4(0.35f, 0.58f, 0.79f, 1.0f);
            colors[ImGuiCol_ResizeGrip] = ImVec4(0.20f, 0.32f, 0.44f, 0.30f);
            colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.30f, 0.49f, 0.67f, 0.70f);
            colors[ImGuiCol_ResizeGripActive] = ImVec4(0.37f, 0.60f, 0.82f, 0.90f);
            colors[ImGuiCol_Tab] = ImVec4(0.08f, 0.12f, 0.16f, 1.0f);
            colors[ImGuiCol_TabHovered] = ImVec4(0.18f, 0.28f, 0.38f, 1.0f);
            colors[ImGuiCol_TabActive] = ImVec4(0.12f, 0.20f, 0.28f, 1.0f);
            colors[ImGuiCol_TabUnfocused] = ImVec4(0.06f, 0.09f, 0.13f, 1.0f);
            colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.09f, 0.14f, 0.19f, 1.0f);
            colors[ImGuiCol_TableHeaderBg] = ImVec4(0.10f, 0.14f, 0.18f, 1.0f);
            colors[ImGuiCol_TableBorderStrong] = ImVec4(0.14f, 0.20f, 0.27f, 1.0f);
            colors[ImGuiCol_TableBorderLight] = ImVec4(0.11f, 0.16f, 0.21f, 1.0f);
            colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
            colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.07f, 0.10f, 0.13f, 0.75f);
            colors[ImGuiCol_TextSelectedBg] = ImVec4(0.21f, 0.47f, 0.70f, 0.45f);
            colors[ImGuiCol_NavHighlight] = ImVec4(0.36f, 0.71f, 0.95f, 1.0f);
            colors[ImGuiCol_PlotHistogram] = ImVec4(0.38f, 0.72f, 0.94f, 1.0f);
            colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.50f, 0.81f, 0.98f, 1.0f);

            themeInitialized = true;
        }

        const ImVec4 textDim(0.58f, 0.67f, 0.74f, 1.0f);
        const ImVec4 textBright(0.94f, 0.96f, 0.98f, 1.0f);
        const ImVec4 accentBlue(0.33f, 0.66f, 0.88f, 1.0f);
        const ImVec4 accentBlueHover(0.40f, 0.73f, 0.94f, 1.0f);
        const ImVec4 accentBlueActive(0.27f, 0.56f, 0.78f, 1.0f);
        const ImVec4 accentGreen(0.22f, 0.70f, 0.47f, 1.0f);
        const ImVec4 accentGreenHover(0.27f, 0.77f, 0.53f, 1.0f);
        const ImVec4 accentGreenActive(0.18f, 0.61f, 0.41f, 1.0f);
        const ImVec4 accentAmber(0.91f, 0.58f, 0.20f, 1.0f);
        const ImVec4 accentAmberHover(0.96f, 0.65f, 0.28f, 1.0f);
        const ImVec4 accentAmberActive(0.84f, 0.50f, 0.14f, 1.0f);
        const ImVec4 accentRed(0.82f, 0.31f, 0.28f, 1.0f);
        const ImVec4 accentRedHover(0.88f, 0.38f, 0.34f, 1.0f);
        const ImVec4 accentRedActive(0.74f, 0.26f, 0.24f, 1.0f);
        const ImVec4 cardColor(0.09f, 0.12f, 0.16f, 1.0f);

        const auto *viewport = ImGui::GetMainViewport();
        const ImVec2 workPos = viewport->WorkPos;
        const ImVec2 workSize = viewport->WorkSize;
        const float outerPadding = 18.0f;
        const float commandBarHeight = 96.0f;
        const float leftPanelWidth = std::clamp(workSize.x * 0.29f, 360.0f, 470.0f);
        const float rightPanelWidth = std::clamp(workSize.x * 0.24f, 320.0f, 410.0f);
        const float panelTop = workPos.y + outerPadding + commandBarHeight;
        const float panelHeight = std::max(300.0f, workSize.y - commandBarHeight - (outerPadding * 2.0f));
        const ImGuiWindowFlags panelFlags = ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoResize |
                                            ImGuiWindowFlags_NoCollapse |
                                            ImGuiWindowFlags_NoSavedSettings;
        const ImGuiWindowFlags toolbarFlags = panelFlags |
                                            ImGuiWindowFlags_NoTitleBar |
                                            ImGuiWindowFlags_NoScrollbar |
                                            ImGuiWindowFlags_NoScrollWithMouse;
        const ImGuiTableFlags readoutTableFlags = ImGuiTableFlags_SizingStretchProp |
                                                  ImGuiTableFlags_BordersInnerV |
                                                  ImGuiTableFlags_RowBg;

        auto pushButtonPalette = [](const ImVec4 &base, const ImVec4 &hovered, const ImVec4 &active)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, base);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hovered);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, active);
        };

        auto popButtonPalette = []()
        {
            ImGui::PopStyleColor(3);
        };

        bool cardOpen = false;

        auto beginCard = [&](const char *id, float)
        {
            ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(12.0f, 10.0f));
            ImGui::PushStyleColor(ImGuiCol_TableRowBg, cardColor);
            ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, cardColor);
            ImGui::PushStyleColor(ImGuiCol_TableBorderStrong, ImVec4(0.17f, 0.24f, 0.31f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_TableBorderLight, ImVec4(0.17f, 0.24f, 0.31f, 1.0f));

            cardOpen = ImGui::BeginTable(id, 1, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingStretchSame);
            if (cardOpen)
            {
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(cardColor));
                ImGui::TableSetColumnIndex(0);
            }
        };

        auto endCard = [&]()
        {
            if (cardOpen)
            {
                ImGui::EndTable();
                cardOpen = false;
            }
            ImGui::PopStyleColor(4);
            ImGui::PopStyleVar();
            ImGui::Dummy(ImVec2(0.0f, 10.0f));
        };

        auto drawCardHeader = [&](const char *title, const char *subtitle)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, textBright);
            ImGui::TextUnformatted(title);
            ImGui::PopStyleColor();
            if (subtitle != nullptr && subtitle[0] != '\0')
            {
                ImGui::PushStyleColor(ImGuiCol_Text, textDim);
                ImGui::TextWrapped("%s", subtitle);
                ImGui::PopStyleColor();
            }
            ImGui::Dummy(ImVec2(0.0f, 4.0f));
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0.0f, 2.0f));
        };

        auto drawReadoutRow = [&](const char *label, const char *value, const ImVec4 &valueColor)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::PushStyleColor(ImGuiCol_Text, textDim);
            ImGui::TextUnformatted(label);
            ImGui::PopStyleColor();
            ImGui::TableSetColumnIndex(1);
            ImGui::PushStyleColor(ImGuiCol_Text, valueColor);
            ImGui::TextUnformatted(value);
            ImGui::PopStyleColor();
        };

        auto drawToolbarPill = [&](const char *id, const char *label, const char *value, const ImVec4 &accent, float height)
        {
            const float width = std::max(1.0f, ImGui::GetContentRegionAvail().x);
            const ImVec2 pos = ImGui::GetCursorScreenPos();
            const ImVec2 size(width, height);
            ImDrawList *drawList = ImGui::GetWindowDrawList();
            const ImU32 bg = ImGui::ColorConvertFloat4ToU32(ImVec4(0.10f, 0.14f, 0.19f, 1.0f));
            const ImU32 border = ImGui::ColorConvertFloat4ToU32(ImVec4(0.17f, 0.24f, 0.31f, 1.0f));
            const ImU32 accentLine = ImGui::ColorConvertFloat4ToU32(ImVec4(accent.x, accent.y, accent.z, 0.95f));
            const ImU32 labelColor = ImGui::ColorConvertFloat4ToU32(textDim);
            const ImU32 valueColor = ImGui::ColorConvertFloat4ToU32(textBright);

            drawList->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), bg, 10.0f);
            drawList->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), border, 10.0f, 0, 1.0f);
            drawList->AddRectFilled(pos, ImVec2(pos.x + 4.0f, pos.y + size.y), accentLine, 10.0f, ImDrawFlags_RoundCornersLeft);
            drawList->AddText(ImVec2(pos.x + 14.0f, pos.y + 8.0f), labelColor, label);
            drawList->AddText(ImVec2(pos.x + 14.0f, pos.y + 23.0f), valueColor, value);

            ImGui::PushID(id);
            ImGui::InvisibleButton("##toolbar_pill", size);
            ImGui::PopID();
        };

        auto aiStateName = [](TargetAIState state) -> const char *
        {
            switch (state)
            {
            case TargetAIState::PATROL:
                return "Patrol";
            case TargetAIState::REPOSITION:
                return "Reposition";
            case TargetAIState::DEFENSIVE:
                return "Defensive";
            case TargetAIState::RECOVERING:
                return "Recover";
            default:
                return "Unknown";
            }
        };

        auto applyLiveMissileConfig = [&]()
        {
            if (!m_missile)
            {
                return;
            }

            m_missile->setMass(m_mass);
            m_missile->setDragCoefficient(m_dragCoefficient);
            m_missile->setCrossSectionalArea(m_crossSectionalArea);
            m_missile->setLiftCoefficient(m_liftCoefficient);
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
            m_missile->setThrust(m_missileThrust);
            m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);

            if (!m_missileInFlight)
            {
                m_missile->setFuel(m_missileFuel);
            }
        };

        auto applyLiveTargetAIConfig = [&]()
        {
            for (const auto &target : m_targets)
            {
                if (!target)
                {
                    continue;
                }

                target->setAIConfig(m_targetAIConfig);
            }
        };

        int activeTargets = 0;
        for (const auto &target : m_targets)
        {
            if (target && target->isActive())
            {
                activeTargets++;
            }
        }

        glm::vec3 missilePosition = m_missile->getPosition();
        glm::vec3 missileVelocity = m_missile->getVelocity();
        glm::vec3 missileAcceleration = m_missile->getAcceleration();
        glm::vec3 cameraPosition = m_renderer->getCameraPosition();

        const float missileSpeed = glm::length(missileVelocity);
        const float missileAltitude = std::max(missilePosition.y, 0.0f);
        const float terrainClearance = missilePosition.y - (m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f);
        const float missileMass = m_missile->getMass();
        const float missileDryMass = m_missile->getDryMass();
        Atmosphere::State missileAtmosphere;
        if (m_physicsEngine)
        {
            missileAtmosphere = m_physicsEngine->getAtmosphereState(missileAltitude);
        }
        else
        {
            Atmosphere fallbackAtmosphere(m_savedAirDensity);
            missileAtmosphere = fallbackAtmosphere.sample(missileAltitude);
        }
        const float missileMach =
            (missileAtmosphere.speedOfSoundMetersPerSecond > 0.0f)
                ? (missileSpeed / missileAtmosphere.speedOfSoundMetersPerSecond)
                : 0.0f;
        const bool thrustEnabled = m_missile->isThrustEnabled();
        const bool guidanceEnabled = m_missile->isGuidanceEnabled();
        const float fuel = m_missile->getFuel();
        const bool boosterBurnedOut = !thrustEnabled && fuel <= 0.0f;
        const float fuelPercent = (m_missileFuel > 0.0f) ? glm::clamp(fuel / m_missileFuel, 0.0f, 1.0f) : 0.0f;
        ImVec4 fuelColor = accentRed;
        if (fuelPercent > 0.50f)
        {
            fuelColor = accentGreen;
        }
        else if (fuelPercent > 0.25f)
        {
            fuelColor = accentAmber;
        }

        Target *trackedTarget = m_missile->getTargetObject();
        if ((trackedTarget == nullptr || !trackedTarget->isActive()) && activeTargets > 0)
        {
            trackedTarget = findBestTarget();
        }

        int trackedTargetIndex = -1;
        if (trackedTarget != nullptr)
        {
            for (size_t i = 0; i < m_targets.size(); ++i)
            {
                if (m_targets[i].get() == trackedTarget)
                {
                    trackedTargetIndex = static_cast<int>(i) + 1;
                    break;
                }
            }
        }

        const bool guidanceLocked = guidanceEnabled && trackedTarget != nullptr && trackedTarget->isActive();
        const float trackedTargetRange = guidanceLocked ? glm::distance(missilePosition, trackedTarget->getPosition()) : 0.0f;

        const char *missionState = "Standby";
        ImVec4 missionStateColor = accentBlue;
        if (m_isPaused)
        {
            missionState = "Paused";
            missionStateColor = accentAmber;
        }
        else if (m_missileInFlight && guidanceLocked && thrustEnabled)
        {
            missionState = "Intercept";
            missionStateColor = accentGreen;
        }
        else if (m_missileInFlight && guidanceLocked)
        {
            missionState = "Glide Track";
            missionStateColor = accentBlue;
        }
        else if (m_missileInFlight && thrustEnabled)
        {
            missionState = "Boost";
            missionStateColor = accentAmber;
        }
        else if (m_missileInFlight)
        {
            missionState = "Ballistic";
            missionStateColor = accentRed;
        }

        ImGui::SetNextWindowPos(ImVec2(workPos.x + outerPadding, workPos.y + outerPadding), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(workSize.x - (outerPadding * 2.0f), commandBarHeight), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 8.0f));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.09f, 0.13f, 0.98f));
        ImGui::Begin("##CommandDeck", nullptr, toolbarFlags);
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(8.0f, 4.0f));
        if (ImGui::BeginTable("CommandDeckLayout", 3, ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Brand", ImGuiTableColumnFlags_WidthStretch, 1.20f);
            ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 1.45f);
            ImGui::TableSetupColumn("Actions", ImGuiTableColumnFlags_WidthStretch, 1.10f);
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImDrawList *drawList = ImGui::GetWindowDrawList();
            const ImVec2 brandPos = ImGui::GetCursorScreenPos();
            const ImVec2 brandSize(ImGui::GetContentRegionAvail().x, 56.0f);
            drawList->AddRectFilled(brandPos, ImVec2(brandPos.x + brandSize.x, brandPos.y + brandSize.y),
                                    ImGui::ColorConvertFloat4ToU32(ImVec4(0.09f, 0.13f, 0.18f, 1.0f)), 12.0f);
            drawList->AddRect(brandPos, ImVec2(brandPos.x + brandSize.x, brandPos.y + brandSize.y),
                              ImGui::ColorConvertFloat4ToU32(ImVec4(0.19f, 0.29f, 0.39f, 1.0f)), 12.0f, 0, 1.0f);
            drawList->AddRectFilled(brandPos, ImVec2(brandPos.x + 6.0f, brandPos.y + brandSize.y),
                                    ImGui::ColorConvertFloat4ToU32(ImVec4(0.33f, 0.66f, 0.88f, 1.0f)), 12.0f, ImDrawFlags_RoundCornersLeft);
            drawList->AddText(ImVec2(brandPos.x + 18.0f, brandPos.y + 10.0f),
                              ImGui::ColorConvertFloat4ToU32(textDim), "TACTICAL SIMULATION");
            drawList->AddText(ImVec2(brandPos.x + 18.0f, brandPos.y + 29.0f),
                              ImGui::ColorConvertFloat4ToU32(textBright), "MISSILESIM COMMAND DECK");
            ImGui::Dummy(brandSize);

            ImGui::TableSetColumnIndex(1);
            if (ImGui::BeginTable("CommandDeckStats", 4, ImGuiTableFlags_SizingStretchSame))
            {
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                drawToolbarPill("state", "STATE", missionState, missionStateColor, 56.0f);

                char statBuffer[32];

                ImGui::TableSetColumnIndex(1);
                std::snprintf(statBuffer, sizeof(statBuffer), "%d", m_score);
                drawToolbarPill("score", "SCORE", statBuffer, accentBlue, 56.0f);

                ImGui::TableSetColumnIndex(2);
                std::snprintf(statBuffer, sizeof(statBuffer), "%d", m_targetHits);
                drawToolbarPill("hits", "HITS", statBuffer, accentGreen, 56.0f);

                ImGui::TableSetColumnIndex(3);
                std::snprintf(statBuffer, sizeof(statBuffer), "%d", activeTargets);
                drawToolbarPill("targets", "ACTIVE", statBuffer, accentAmber, 56.0f);

                ImGui::EndTable();
            }

            ImGui::TableSetColumnIndex(2);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 10.0f);
            const float actionWidth = ImGui::GetContentRegionAvail().x;
            const float buttonWidth = std::max(0.0f, (actionWidth - 10.0f) * 0.5f);

            pushButtonPalette(accentBlue, accentBlueHover, accentBlueActive);
            if (ImGui::Button(m_isPaused ? "Resume" : "Pause", ImVec2(buttonWidth, 36.0f)))
            {
                m_isPaused = !m_isPaused;
            }
            popButtonPalette();

            ImGui::SameLine();

            pushButtonPalette(accentAmber, accentAmberHover, accentAmberActive);
            if (ImGui::Button("Launch", ImVec2(buttonWidth, 36.0f)))
            {
                launchMissile();
            }
            popButtonPalette();

            ImGui::PushStyleColor(ImGuiCol_Text, textDim);
            ImGui::TextUnformatted(m_missileInFlight ? "Live flight in progress" : "Launch hotkey: F");
            ImGui::PopStyleColor();

            ImGui::EndTable();
        }
        ImGui::PopStyleVar();
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();

        ImGui::SetNextWindowPos(ImVec2(workPos.x + outerPadding, panelTop), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(leftPanelWidth, panelHeight), ImGuiCond_Always);
        ImGui::Begin("Mission Control", nullptr, panelFlags);

        if (ImGui::BeginTabBar("WorkspaceTabs", ImGuiTabBarFlags_FittingPolicyResizeDown))
        {
            if (ImGui::BeginTabItem("Scenario"))
            {
                beginCard("ScenarioSummaryCard", 150.0f);
                drawCardHeader("Scenario Pace", "Primary runtime controls and launch-stage propulsion settings.");
                if (ImGui::BeginTable("ScenarioSummaryTable", 2, readoutTableFlags))
                {
                    char buffer[96];
                    std::snprintf(buffer, sizeof(buffer), "%.1fx", m_simulationSpeed);
                    drawReadoutRow("Simulation speed", buffer, textBright);
                    std::snprintf(buffer, sizeof(buffer), "%.1f s", m_missileFlightTime);
                    drawReadoutRow("Flight timer", buffer, textBright);
                    if (m_closestTargetDistance < 999999.0f)
                    {
                        std::snprintf(buffer, sizeof(buffer), "%.1f m", m_closestTargetDistance);
                        drawReadoutRow("Closest pass", buffer, textBright);
                    }
                    else
                    {
                        drawReadoutRow("Closest pass", "Not available", textDim);
                    }
                    std::snprintf(buffer, sizeof(buffer), "%d active", activeTargets);
                    drawReadoutRow("Target group", buffer, textBright);
                    ImGui::EndTable();
                }
                ImGui::SliderFloat("Simulation speed", &m_simulationSpeed, 0.1f, 10.0f, "%.1fx");
                endCard();

                beginCard("PropulsionCard", 184.0f);
                drawCardHeader("Propulsion Profile", "Fuel is propellant mass added on top of dry airframe mass.");
                ImGui::SliderFloat("Thrust output", &m_missileThrust, 1000.0f, 50000.0f, "%.0f N");
                ImGui::SliderFloat("Fuel load", &m_missileFuel, 10.0f, 1000.0f, "%.1f kg");
                ImGui::SliderFloat("Fuel burn rate", &m_missileFuelConsumptionRate, 0.1f, 10.0f, "%.2f kg/s");
                ImGui::TextDisabled("Launch mass: %.1f kg", m_mass + m_missileFuel);
                endCard();

                beginCard("EnvironmentCard", 200.0f);
                drawCardHeader("World Settings", "Gravity, atmosphere, and ground interaction affect every flight.");
                float gravity = m_physicsEngine->getGravity();
                float airDensity = m_physicsEngine->getAirDensity();
                if (ImGui::SliderFloat("Gravity", &gravity, 0.0f, 20.0f, "%.2f m/s^2"))
                {
                    m_physicsEngine->setGravity(gravity);
                }
                if (ImGui::SliderFloat("Sea-level density", &airDensity, 0.0f, 2.0f, "%.3f kg/m^3"))
                {
                    m_physicsEngine->setAirDensity(airDensity);
                }
                if (ImGui::Checkbox("Ground collision enabled", &m_groundEnabled))
                {
                    m_physicsEngine->setGroundEnabled(m_groundEnabled);
                }
                if (m_groundEnabled)
                {
                    if (ImGui::SliderFloat("Ground restitution", &m_groundRestitution, 0.0f, 1.0f, "%.2f"))
                    {
                        m_physicsEngine->setGroundRestitution(m_groundRestitution);
                    }
                    ImGui::TextDisabled("0.0 is fully deadened, 1.0 is a perfect bounce.");
                }
                else
                {
                    ImGui::TextDisabled("Ground interaction is disabled.");
                }
                endCard();

                beginCard("RecoveryCard", 100.0f);
                drawCardHeader("Recovery", "Quick resets without leaving the control deck.");
                const float recoveryWidth = (ImGui::GetContentRegionAvail().x - deckStyle.ItemSpacing.x) * 0.5f;
                pushButtonPalette(accentRed, accentRedHover, accentRedActive);
                if (ImGui::Button("Reset Missile", ImVec2(recoveryWidth, 36.0f)))
                {
                    resetMissile();
                }
                popButtonPalette();
                ImGui::SameLine();
                if (ImGui::Button("Reset Targets", ImVec2(recoveryWidth, 36.0f)))
                {
                    resetTargets();
                }
                endCard();

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Missile"))
            {
                beginCard("MissileLaunchCard", 170.0f);
                drawCardHeader("Launch State", "Edit the staged missile position and initial velocity vectors.");
                ImGui::InputFloat3("Spawn position", m_initialPosition);
                ImGui::InputFloat3("Launch velocity", m_initialVelocity);
                ImGui::TextDisabled("Rearm to rebuild the missile with these staged values.");
                endCard();

                beginCard("MissileAirframeCard", 188.0f);
                drawCardHeader("Airframe", "Dry mass and aerodynamic coefficients determine stability and energy retention.");
                ImGui::SliderFloat("Dry mass", &m_mass, 10.0f, 1000.0f, "%.1f kg");
                ImGui::SliderFloat("Drag coefficient", &m_dragCoefficient, 0.01f, 1.0f, "%.3f");
                ImGui::SliderFloat("Cross-sectional area", &m_crossSectionalArea, 0.01f, 1.0f, "%.3f m^2");
                ImGui::SliderFloat("Lift coefficient", &m_liftCoefficient, 0.0f, 1.0f, "%.3f");
                endCard();

                beginCard("MissileGuidanceCard", 336.0f);
                drawCardHeader("Guidance", "Heat-seeker tuning, intercept steering, and seeker resistance to countermeasures.");
                ImGui::Checkbox("Guidance enabled", &m_guidanceEnabled);
                ImGui::SliderFloat("Lead aggressiveness", &m_navigationGain, 1.0f, 4.0f, "%.2f");
                ImGui::TextDisabled("1.0 tracks the target directly, 4.0 steers to the full first-order intercept.");
                ImGui::SliderFloat("Max steering force", &m_maxSteeringForce, 1000.0f, 50000.0f, "%.0f N");
                ImGui::SliderFloat("Tracking angle", &m_trackingAngle, 5.0f, 180.0f, "%.0f deg");
                ImGui::SliderFloat("Proximity fuse", &m_proximityFuseRadius, 0.0f, 75.0f, "%.1f m");
                ImGui::SliderFloat("IRCCM resistance", &m_countermeasureResistance, 0.0f, 1.0f, "%.2f");
                ImGui::TextDisabled("0.0 is flare-hungry, 1.0 strongly favors kinematically consistent targets.");
                ImGui::Checkbox("Terrain avoidance", &m_terrainAvoidanceEnabled);
                ImGui::SliderFloat("Terrain clearance", &m_terrainClearance, 0.0f, 400.0f, "%.1f m");
                ImGui::SliderFloat("Terrain look-ahead", &m_terrainLookAheadTime, 0.5f, 12.0f, "%.1f s");
                endCard();

                beginCard("MissileApplyCard", 112.0f);
                drawCardHeader("Apply", "Push staged values into the current missile or rebuild it cleanly.");
                pushButtonPalette(accentGreen, accentGreenHover, accentGreenActive);
                if (ImGui::Button("Apply To Live Missile", ImVec2(-1.0f, 36.0f)))
                {
                    applyLiveMissileConfig();
                }
                popButtonPalette();
                if (ImGui::Button("Rearm Missile From Staged Config", ImVec2(-1.0f, 32.0f)))
                {
                    resetMissile();
                }
                endCard();

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Targets"))
            {
                beginCard("TargetAICard", 214.0f);
                drawCardHeader("Target AI", "Autonomous aircraft manage speed, spacing, altitude, and countermeasures internally.");
                char targetStatus[96];
                std::snprintf(targetStatus, sizeof(targetStatus), "%d active of %zu total", activeTargets, m_targets.size());
                ImGui::TextDisabled("%s", targetStatus);

                ImGui::SliderInt("Target count", &m_targetCount, 1, 20);
                if (ImGui::IsItemDeactivatedAfterEdit())
                {
                    resetTargets();
                }

                ImGui::SliderFloat("Average distance", &m_targetAIConfig.preferredDistance, 300.0f, 20000.0f, "%.0f m");
                if (ImGui::IsItemDeactivatedAfterEdit())
                {
                    resetTargets();
                }
                ImGui::SliderFloat("Minimum speed", &m_targetAIConfig.minSpeed, 60.0f, 450.0f, "%.0f m/s");
                m_targetAIConfig.maxSpeed = std::max(m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f);
                ImGui::SliderFloat("Maximum speed", &m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f, 600.0f, "%.0f m/s");
                pushButtonPalette(accentBlue, accentBlueHover, accentBlueActive);
                if (ImGui::Button("Apply AI Profile", ImVec2(-1.0f, 34.0f)))
                {
                    applyLiveTargetAIConfig();
                }
                popButtonPalette();
                endCard();

                beginCard("TargetRosterCard", 460.0f);
                drawCardHeader("Live Roster", "Read-only status table for every spawned target and its autonomous behavior.");
                if (m_targets.empty())
                {
                    ImGui::TextDisabled("No targets are currently loaded.");
                }
                else if (ImGui::BeginTable("TargetRosterTable", 7, readoutTableFlags))
                {
                    ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 42.0f);
                    ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                    ImGui::TableSetupColumn("AI", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableSetupColumn("Speed", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                    ImGui::TableSetupColumn("Flares", ImGuiTableColumnFlags_WidthStretch, 0.8f);
                    ImGui::TableSetupColumn("Range", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                    ImGui::TableSetupColumn("CPA", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                    ImGui::TableHeadersRow();

                    for (size_t i = 0; i < m_targets.size(); ++i)
                    {
                        const auto &target = m_targets[i];
                        if (!target)
                        {
                            continue;
                        }

                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%zu", i + 1);

                        ImGui::TableSetColumnIndex(1);
                        const bool isActive = target->isActive();
                        ImGui::TextColored(isActive ? accentGreen : textDim, "%s", isActive ? "Active" : "Destroyed");

                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextColored(target->isMissileWarningActive() ? accentAmber : textBright,
                                           "%s", aiStateName(target->getAIState()));

                        ImGui::TableSetColumnIndex(3);
                        if (isActive)
                        {
                            ImGui::Text("%.0f m/s", target->getCommandedSpeed());
                        }
                        else
                        {
                            ImGui::TextDisabled("--");
                        }

                        ImGui::TableSetColumnIndex(4);
                        if (isActive)
                        {
                            ImGui::Text("%d", target->getRemainingFlares());
                        }
                        else
                        {
                            ImGui::TextDisabled("--");
                        }

                        ImGui::TableSetColumnIndex(5);
                        if (isActive)
                        {
                            ImGui::Text("%.1f m", glm::distance(missilePosition, target->getPosition()));
                        }
                        else
                        {
                            ImGui::TextDisabled("--");
                        }

                        ImGui::TableSetColumnIndex(6);
                        if (isActive && target->isMissileWarningActive())
                        {
                            ImGui::Text("%.0f m", target->getThreatClosestApproachDistance());
                        }
                        else
                        {
                            ImGui::TextDisabled("--");
                        }
                    }

                    ImGui::EndTable();
                }

                pushButtonPalette(accentRed, accentRedHover, accentRedActive);
                if (ImGui::Button("Rebuild Target Group", ImVec2(-1.0f, 34.0f)))
                {
                    resetTargets();
                }
                popButtonPalette();
                endCard();

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("View"))
            {
                beginCard("CameraCard", 150.0f);
                drawCardHeader("Camera", "Viewport look controls and movement speed.");
                float fov = m_renderer->getCameraFOV();
                if (ImGui::SliderFloat("Field of view", &fov, 10.0f, 120.0f, "%.1f deg"))
                {
                    m_renderer->setCameraFOV(fov);
                }

                float cameraSpeed = m_renderer->getCameraSpeed();
                if (ImGui::SliderFloat("Camera speed", &cameraSpeed, 1.0f, 800.0f, "%.0f"))
                {
                    m_renderer->setCameraSpeed(cameraSpeed);
                }

                ImGui::TextDisabled("Camera position: %.1f, %.1f, %.1f", cameraPosition.x, cameraPosition.y, cameraPosition.z);
                endCard();

                beginCard("OverlayCard", 230.0f);
                drawCardHeader("Overlays", "Enable debug visuals without digging through the runtime inspector.");
                ImGui::Checkbox("Show predicted trajectory", &m_showTrajectory);
                ImGui::Checkbox("Show target labels", &m_showTargetInfo);
                ImGui::Checkbox("Show target prediction path", &m_showPredictedTargetPath);
                ImGui::Checkbox("Show intercept point", &m_showInterceptPoint);
                ImGui::SliderInt("Trajectory detail", &m_trajectoryPoints, 10, 600);
                ImGui::SliderFloat("Trajectory horizon", &m_trajectoryTime, 0.5f, 60.0f, "%.1f s");
                endCard();

                beginCard("HelpCard", 118.0f);
                drawCardHeader("Controls", "Keep the simulation visible while still knowing how to drive the camera.");
                ImGui::TextUnformatted("Right mouse hold: free look or orbit tracked object");
                ImGui::TextUnformatted("W / A / S / D: move camera in Free mode");
                ImGui::TextUnformatted("Space / Ctrl: move up or down");
                ImGui::TextUnformatted("Enter: pause or resume simulation");
                ImGui::TextDisabled(m_cameraMode == CameraMode::FREE
                                        ? (m_enableMouseCamera ? "Free camera active. Mouse capture is active."
                                                               : "Free camera active. Mouse is currently in UI mode.")
                                        : (m_enableMouseCamera ? "Chase camera active. Orbit is active."
                                                               : "Chase camera active. Release RMB recenters smoothly."));
                endCard();

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(workPos.x + workSize.x - rightPanelWidth - outerPadding, panelTop), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(rightPanelWidth, panelHeight), ImGuiCond_Always);
        ImGui::Begin("Flight Inspector", nullptr, panelFlags);

        beginCard("EngagementOverviewCard", 158.0f);
        drawCardHeader("Engagement Overview", "High-level mission state, target lock, and current fight geometry.");
        if (ImGui::BeginTable("EngagementOverviewTable", 2, readoutTableFlags))
        {
            char buffer[96];
            drawReadoutRow("Mission state", missionState, missionStateColor);
            std::snprintf(buffer, sizeof(buffer), "%d / %zu", activeTargets, m_targets.size());
            drawReadoutRow("Targets active", buffer, textBright);
            if (trackedTargetIndex > 0)
            {
                std::snprintf(buffer, sizeof(buffer), "Target %d", trackedTargetIndex);
                drawReadoutRow("Tracked target", buffer, guidanceLocked ? accentGreen : textBright);
            }
            else
            {
                drawReadoutRow("Tracked target", "None", textDim);
            }
            if (guidanceLocked)
            {
                std::snprintf(buffer, sizeof(buffer), "%.1f m", trackedTargetRange);
                drawReadoutRow("Target range", buffer, textBright);
            }
            else
            {
                drawReadoutRow("Target range", "No lock", textDim);
            }
            if (m_closestTargetDistance < 999999.0f)
            {
                std::snprintf(buffer, sizeof(buffer), "%.1f m", m_closestTargetDistance);
                drawReadoutRow("Closest pass", buffer, textBright);
            }
            else
            {
                drawReadoutRow("Closest pass", "Not available", textDim);
            }
            ImGui::EndTable();
        }
        endCard();

        beginCard("TelemetryCard", 264.0f);
        drawCardHeader("Missile Telemetry", "Live position, kinematics, altitude, and local atmosphere readout for the current missile object.");
        if (ImGui::BeginTable("TelemetryTable", 2, readoutTableFlags))
        {
            char buffer[128];
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m", missilePosition.x, missilePosition.y, missilePosition.z);
            drawReadoutRow("Position", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m/s", missileVelocity.x, missileVelocity.y, missileVelocity.z);
            drawReadoutRow("Velocity", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f m/s^2", missileAcceleration.x, missileAcceleration.y, missileAcceleration.z);
            drawReadoutRow("Acceleration", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f m/s", missileSpeed);
            drawReadoutRow("Speed", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.2f", missileMach);
            drawReadoutRow("Mach", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f m", missileAltitude);
            drawReadoutRow("Altitude", buffer, missileAltitude > 0.0f ? textBright : accentAmber);
            std::snprintf(buffer, sizeof(buffer), "%.1f m", terrainClearance);
            drawReadoutRow("Terrain clearance", buffer, terrainClearance > 0.0f ? textBright : accentAmber);
            std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileMass);
            drawReadoutRow("Current mass", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileDryMass);
            drawReadoutRow("Dry mass", buffer, textDim);
            std::snprintf(buffer, sizeof(buffer), "%.3f kg/m^3", missileAtmosphere.densityKgPerCubicMeter);
            drawReadoutRow("Ambient density", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.2f kPa", missileAtmosphere.pressurePascals * 0.001f);
            drawReadoutRow("Ambient pressure", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.2f K", missileAtmosphere.temperatureKelvin);
            drawReadoutRow("Air temperature", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f s", m_missileFlightTime);
            drawReadoutRow("Flight time", buffer, textBright);
            ImGui::EndTable();
        }
        endCard();

        beginCard("EngineCard", 228.0f);
        drawCardHeader("Engine And Guidance", "Propulsion reserve, seeker state, and live control authority.");
        ImGui::TextColored(
            thrustEnabled ? accentGreen : (boosterBurnedOut ? accentAmber : textDim),
            "%s",
            thrustEnabled ? "BOOSTER ACTIVE" : (boosterBurnedOut ? "BOOSTER BURNED OUT" : "BOOSTER OFF"));
        ImGui::TextColored(guidanceLocked ? accentGreen : (guidanceEnabled ? accentAmber : textDim),
                           "%s",
                           guidanceLocked ? "SEEKER LOCKED" : (guidanceEnabled ? "SEEKER SEARCHING" : "GUIDANCE DISABLED"));
        ImGui::Text("Fuel remaining: %.1f kg", fuel);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, fuelColor);
        ImGui::ProgressBar(fuelPercent, ImVec2(-1.0f, 10.0f), "");
        ImGui::PopStyleColor();

        if (ImGui::BeginTable("EngineTable", 2, readoutTableFlags))
        {
            char buffer[96];
            std::snprintf(buffer, sizeof(buffer), "%.0f N", m_missile->getThrust());
            drawReadoutRow("Thrust command", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.2f kg/s", m_missile->getFuelConsumptionRate());
            drawReadoutRow("Burn rate", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f kg", missileMass);
            drawReadoutRow("Wet mass", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.2f", m_navigationGain);
            drawReadoutRow("Lead aggressiveness", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.0f N", m_maxSteeringForce);
            drawReadoutRow("Max steering", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.0f deg", m_trackingAngle);
            drawReadoutRow("Tracking cone", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f m", m_proximityFuseRadius);
            drawReadoutRow("Proximity fuse", buffer, textBright);
            drawReadoutRow("Terrain mode", m_missile->isTerrainAvoidanceEnabled() ? "Clearance hold" : "Direct pursuit", m_missile->isTerrainAvoidanceEnabled() ? accentGreen : textDim);
            std::snprintf(buffer, sizeof(buffer), "%.1f m", m_missile->getTerrainClearance());
            drawReadoutRow("Terrain floor", buffer, textBright);
            ImGui::EndTable();
        }
        endCard();

        beginCard("SceneCard", 162.0f);
        drawCardHeader("Scene Monitor", "Quick camera and environment readout while the simulation keeps running.");
        if (ImGui::BeginTable("SceneMonitorTable", 2, readoutTableFlags))
        {
            char buffer[128];
            std::snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f", cameraPosition.x, cameraPosition.y, cameraPosition.z);
            drawReadoutRow("Camera position", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f deg", m_renderer->getCameraFOV());
            drawReadoutRow("Camera FOV", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.1f", m_renderer->getCameraSpeed());
            drawReadoutRow("Camera speed", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.2f m/s^2", m_physicsEngine->getGravity());
            drawReadoutRow("Gravity", buffer, textBright);
            std::snprintf(buffer, sizeof(buffer), "%.3f kg/m^3", m_physicsEngine->getAirDensity());
            drawReadoutRow("Sea-level density", buffer, textBright);
            ImGui::EndTable();
        }
        ImGui::TextDisabled(m_cameraMode == CameraMode::FREE
                                ? (m_enableMouseCamera ? "Free camera: mouse capture enabled."
                                                       : "Free camera: mouse capture released.")
                                : (m_enableMouseCamera ? "Chase camera: orbit enabled."
                                                       : "Chase camera: centered."));
        endCard();

        ImGui::End();

        const std::string currentSettingsSnapshot = buildSettingsSnapshot();
        if (currentSettingsSnapshot != m_lastSettingsSnapshot)
        {
            scheduleSettingsSave();
            m_lastSettingsSnapshot = currentSettingsSnapshot;
        }
    }
#endif
}

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
        m_targets.clear();
        clearFlares();
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
    if (m_physicsEngine)
    {
        for (auto &flare : m_flares)
        {
            if (flare)
            {
                m_physicsEngine->removeFlare(flare.get());
            }
        }
    }

    m_flares.clear();
}

void Application::launchMissile()
{
    try
    {
        // Reset missile position and velocity
        resetMissile();

        // Safety check for null missile
        if (!m_missile)
        {
            std::cerr << "ERROR: Missile is null in launchMissile()" << std::endl;
            return;
        }

        // Debug target count
        std::cout << "Target count: " << m_targets.size() << std::endl;

        // Validate targets before finding the best one
        if (m_targets.empty())
        {
            std::cout << "No targets available, creating new ones" << std::endl;
            resetTargets();
            if (m_targets.empty())
            {
                std::cerr << "ERROR: Failed to create targets" << std::endl;
                return;
            }
        }

        // Find the best target to track
        Target *target = findBestTarget();

        // Debug target found
        if (target)
        {
            std::cout << "Found target at position: ("
                      << target->getPosition().x << ", "
                      << target->getPosition().y << ", "
                      << target->getPosition().z << ")" << std::endl;

            // Validate target position
            const glm::vec3 &targetPos = target->getPosition();
            if (std::isnan(targetPos.x) || std::isnan(targetPos.y) || std::isnan(targetPos.z) ||
                std::isinf(targetPos.x) || std::isinf(targetPos.y) || std::isinf(targetPos.z))
            {
                std::cerr << "ERROR: Target position contains invalid values" << std::endl;
                return;
            }
        }
        else
        {
            std::cout << "No active target found, creating new targets" << std::endl;
            resetTargets();
            target = findBestTarget();
            if (!target)
            {
                std::cerr << "ERROR: Still no active target found after reset" << std::endl;
                return;
            }
        }

        // Set the target for guidance if a valid target was found
        if (target)
        {
            m_physicsEngine->setMissileTarget(m_missile.get(), target);
        }

        // Apply initial thrust to the missile
        glm::vec3 thrustDirection;

        // If we have a target, aim initial thrust toward it
        if (target)
        {
            thrustDirection = glm::normalize(target->getPosition() - m_missile->getPosition());
        }
        // Otherwise launch upward with a slight forward tilt
        else
        {
            thrustDirection = glm::normalize(glm::vec3(0.0f, 0.5f, 1.0f));
        }

        // Set thrust parameters
        m_missile->setThrust(m_missileThrust);
        m_missile->setThrustDirection(thrustDirection);
        m_missile->setFuel(m_missileFuel);
        m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);

        // Enable thrust engine
        m_missile->setThrustEnabled(true);

        // Apply an initial impulse in the thrust direction
        float initialSpeed = m_missileThrust / 50.0f; // Scale down for reasonable initial velocity
        m_missile->setVelocity(thrustDirection * initialSpeed);

        // Log launch
        std::cout << "Missile launched with thrust: " << m_missileThrust
                  << " N, fuel: " << m_missileFuel << " kg" << std::endl;

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
            m_countermeasureResistance = 0.35f;
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

void Application::createExplosion(const glm::vec3 &position)
{
    // Create new explosion effect at the given position
    ExplosionEffect explosion;
    explosion.position = position;
    explosion.timeRemaining = m_explosionDuration;
    explosion.size = 0.0f; // Start with size 0, will grow and then shrink

    // Add to explosion list
    m_explosions.push_back(explosion);
}

void Application::updateExplosions(float deltaTime)
{
    // Update all active explosion effects
    for (auto &explosion : m_explosions)
    {
        // Decrease remaining time
        explosion.timeRemaining -= deltaTime;

        // Update size - grow quickly then shrink
        float normalizedTime = 1.0f - (explosion.timeRemaining / m_explosionDuration);
        if (normalizedTime < 0.3f)
        {
            // Initial growth phase
            explosion.size = (normalizedTime / 0.3f) * m_explosionMaxSize;
        }
        else
        {
            // Shrinking phase
            explosion.size = ((1.0f - normalizedTime) / 0.7f) * m_explosionMaxSize;
        }
    }

    // Remove expired explosions
    while (!m_explosions.empty() && m_explosions.front().timeRemaining <= 0)
    {
        m_explosions.pop_front();
    }
}

void Application::renderExplosions()
{
    // Call renderer to draw each active explosion
    for (const auto &explosion : m_explosions)
    {
        m_renderer->renderExplosion(explosion.position, explosion.size);
    }
}