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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
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
#include "ui/Theme.h"
#include "rendering/Renderer.h"

using missilesim::application::detail::formatBoolValue;
using missilesim::application::detail::formatVec3Value;
using missilesim::application::detail::parseBoolValue;
using missilesim::application::detail::parseFloatValue;
using missilesim::application::detail::parseIntValue;
using missilesim::application::detail::parseVec3Value;
using missilesim::application::detail::safeNormalize;
using missilesim::application::detail::trimWhitespace;

void Application::applySimulationConfigDefaults()
{
    const missilesim::sim::SimulationConfig &config = m_simulationConfig;

    m_clock.setStep(config.environment.fixedTimeStep);
    m_simulationSpeed = config.environment.simulationSpeed;
    m_groundEnabled = config.environment.groundCollisionEnabled;
    m_groundRestitution = config.environment.groundRestitution;
    m_savedGravity = config.environment.gravity;
    m_savedAirDensity = config.environment.seaLevelAirDensity;
    m_terrainKind = config.terrain.kind;

    m_showTrajectory = config.visualization.showTrajectory;
    m_showTargetInfo = config.visualization.showTargetInfo;
    m_showPredictedTargetPath = config.visualization.showPredictedTargetPath;
    m_showInterceptPoint = config.visualization.showInterceptPoint;
    m_trajectoryPoints = config.visualization.trajectoryPoints;
    m_trajectoryTime = config.visualization.trajectoryTime;

    m_savedCameraFOV = config.camera.fov;
    m_savedCameraSpeed = config.camera.speed;

    const missilesim::sim::MissileAirframeConfig &airframe = config.missile.airframe;
    m_initialPosition[0] = airframe.initialPosition.x;
    m_initialPosition[1] = airframe.initialPosition.y;
    m_initialPosition[2] = airframe.initialPosition.z;
    m_initialVelocity[0] = airframe.initialVelocity.x;
    m_initialVelocity[1] = airframe.initialVelocity.y;
    m_initialVelocity[2] = airframe.initialVelocity.z;
    m_mass = airframe.dryMass;
    m_dragCoefficient = airframe.dragCoefficient;
    m_crossSectionalArea = airframe.crossSectionalArea;
    m_liftCoefficient = airframe.liftCoefficient;

    const missilesim::sim::MissileMotorConfig &motor = config.missile.motor;
    m_missileThrust = motor.thrust;
    m_missileFuel = motor.fuelMass;
    m_missileFuelConsumptionRate = motor.fuelConsumptionRate;

    const missilesim::sim::MissileGuidanceConfig &guidance = config.missile.guidance;
    m_guidanceEnabled = guidance.enabled;
    m_navigationGain = guidance.navigationGain;
    m_maxSteeringForce = guidance.maxSteeringForce;
    m_trackingAngle = guidance.trackingAngle;
    m_proximityFuseRadius = guidance.proximityFuseRadius;
    m_countermeasureResistance = guidance.countermeasureResistance;
    m_terrainAvoidanceEnabled = guidance.terrainAvoidanceEnabled;
    m_terrainClearance = guidance.terrainClearance;
    m_terrainLookAheadTime = guidance.terrainLookAheadTime;
    m_seekerCueRadiusPixels = guidance.seekerCueRadiusPixels;

    m_targetCount = config.targets.count;
    m_targetAIConfig.minSpeed = config.targets.minSpeed;
    m_targetAIConfig.maxSpeed = config.targets.maxSpeed;
    m_targetAIConfig.preferredDistance = config.targets.preferredDistance;
}

// Startup failures throw (after cleaning up) so they reach main(), which shows
// them to the player; the messages are written for someone without the source.
void Application::initialize()
{
    try
    {
        // Initialize GLFW
        if (!glfwInit())
        {
            throw std::runtime_error("Could not initialize the windowing system (GLFW).");
        }

        // Hidden, centred, context current; throws if OpenGL 4.5 is unavailable.
        createMainWindow();

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

        glfwSetScrollCallback(m_window, [](GLFWwindow *window, double xoffset, double yoffset)
                              {
            (void)xoffset;
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            app->scrollCallback(yoffset); });

        // Initialize GLAD
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            throw std::runtime_error("Could not load the OpenGL functions from the graphics driver (GLAD).");
        }

        // Check OpenGL version
        if (!GLAD_GL_VERSION_4_5)
        {
            std::cerr << "WARNING: OpenGL 4.5 not available. PBR rendering will be disabled." << std::endl;
            std::cerr << "         Detected: " << glGetString(GL_VERSION) << std::endl;
        }

        // Enable core OpenGL features for PBR pipeline
        glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

        // Setup ImGui (a failed backend is torn down by shutdown() via the catch below)
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true))
        {
            throw std::runtime_error("Could not initialize the user interface (ImGui GLFW backend).");
        }

        if (!ImGui_ImplOpenGL3_Init("#version 450"))
        {
            throw std::runtime_error("Could not initialize the user interface (ImGui OpenGL backend).");
        }

        missilesim::ui::initializeTheme(windowContentScale());

        // Create simulation components
        m_renderer = std::make_unique<Renderer>();
        m_audioSystem = std::make_unique<AudioSystem>();
        m_audioSystem->initialize();

        const missilesim::sim::SimulationConfigLoadResult configLoad = missilesim::sim::loadDefaultSimulationConfig();
        m_simulationConfig = configLoad.config;
        if (!configLoad.loaded)
        {
            std::cerr << "WARNING: Using built-in simulation defaults. " << configLoad.error << std::endl;
        }
        for (const std::string &warning : configLoad.warnings)
        {
            std::cerr << "WARNING: " << configLoad.path.string() << ": " << warning << std::endl;
        }
        applySimulationConfigDefaults();

        m_seedSource.seed(std::random_device{}());
        m_world = std::make_unique<missilesim::sim::World>(m_simulationConfig, m_seedSource());

        const bool loadedSettings = loadSettings();

        // User settings override the config's environment.
        m_world->physics().setGroundEnabled(m_groundEnabled);
        m_world->physics().setGroundRestitution(m_groundRestitution);
        m_world->physics().setGravity(m_savedGravity);
        m_world->physics().setAirDensity(m_savedAirDensity);
        if (m_world->terrain().kind() != m_terrainKind)
        {
            missilesim::sim::TerrainConfig terrain = m_simulationConfig.terrain;
            terrain.kind = m_terrainKind;
            m_world->setTerrain(terrain);
        }
        m_renderer->setTerrain(m_world->sharedTerrain());
        m_renderer->setCameraFOV(m_savedCameraFOV);
        m_renderer->setCameraSpeed(m_savedCameraSpeed);

        // Targets first: the fighter spawns pointed at the first of them.
        restartWorld();
        if (m_playerRole == PlayerRole::Fighter)
        {
            setCameraMode(CameraMode::FIGHTER_JET);
            resetAimCamera();
        }

        frameEngagementCamera();
        if (loadedSettings)
        {
            m_renderer->setCameraFOV(m_savedCameraFOV);
            m_renderer->setCameraSpeed(m_savedCameraSpeed);
        }

        m_lastSettingsSnapshot = buildSettingsSnapshot();
        m_initialized = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        shutdown();
        throw;
    }
    catch (...)
    {
        std::cerr << "Unknown exception during initialization" << std::endl;
        shutdown();
        throw;
    }
}

void Application::shutdown()
{
    try
    {
        if (!m_window && !m_renderer && !m_world && !m_audioSystem)
        {
            return;
        }

        if (m_initialized)
        {
            saveSettings();
            m_settingsDirty = false;
        }

        // Audio holds pointers into the simulation; stop it first.
        if (m_audioSystem)
        {
            m_audioSystem->shutdown();
            m_audioSystem.reset();
        }
        m_world.reset();
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
    // Outside the try below on purpose: startup errors propagate to the caller
    // (initialize() has already cleaned up) instead of being logged and swallowed.
    initialize();

    try
    {
        // Set initial viewport size to match window
        int width, height;
        glfwGetFramebufferSize(m_window, &width, &height);
        m_width = width;
        m_height = height;
        m_renderer->setViewportSize(width, height);

        auto lastTime = std::chrono::high_resolution_clock::now();
        const char *frameStatsFlag = std::getenv("MISSILESIM_FRAME_STATS");
        const bool frameStats = frameStatsFlag != nullptr && frameStatsFlag[0] != '\0' && frameStatsFlag[0] != '0';
        float frameStatSeconds = 0.0f;
        float frameStatWorst = 0.0f;
        int frameStatCount = 0;

        // Main loop
        while (!glfwWindowShouldClose(m_window))
        {
            try
            {
                // Deliver input before sampling controls and simulating this
                // frame, including events accumulated during the V-sync wait.
                glfwPollEvents();
                if (glfwWindowShouldClose(m_window))
                {
                    break;
                }
                if (isWindowMinimized())
                {
                    // Nothing to draw into: sleep until the window is restored.
                    glfwWaitEvents();
                    lastTime = std::chrono::high_resolution_clock::now();
                    continue;
                }

                updateWindowFrame();

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

                // MISSILESIM_FRAME_STATS=1 prints the frame-time spread every five
                // seconds: a quick check for hitches without a profiler.
                if (frameStats)
                {
                    frameStatSeconds += deltaTime;
                    frameStatWorst = std::max(frameStatWorst, deltaTime);
                    ++frameStatCount;
                    if (frameStatSeconds >= 5.0f)
                    {
                        std::cout << "Frames: " << frameStatCount << " in " << frameStatSeconds << " s, mean "
                                  << 1000.0f * frameStatSeconds / static_cast<float>(frameStatCount) << " ms, worst "
                                  << 1000.0f * frameStatWorst << " ms" << std::endl;
                        frameStatSeconds = 0.0f;
                        frameStatWorst = 0.0f;
                        frameStatCount = 0;
                    }
                }

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
                else
                {
                    // A launch while paused still shows its effects.
                    processSimEvents();
                }

                // Render
                render();
                if (m_window != nullptr)
                {
                    const bool screenshotKey = glfwGetKey(m_window, GLFW_KEY_F12) == GLFW_PRESS;
                    if (screenshotKey && !m_screenshotKeyHeld)
                    {
                        saveScreenshot();
                    }
                    m_screenshotKeyHeld = screenshotKey;
                }
                flushSettingsAutosave(deltaTime);

                // Present the completed frame.
                try
                {
                    glfwSwapBuffers(m_window);
                    revealWindowAfterFirstFrame();
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

void Application::update(float deltaTime)
{
    try
    {
        if (!m_world)
        {
            return;
        }

        const float frame = (deltaTime > 0.0f && std::isfinite(deltaTime)) ? deltaTime : 0.016f;
        if (m_renderer)
        {
            m_renderer->updateEffects(frame);
        }

        // Whole fixed steps only; the clock owns the backlog policy.
        const int steps = m_clock.advance(frame, m_simulationSpeed);
        for (int step = 0; step < steps; ++step)
        {
            m_world->step();
        }
        // Leftover fraction of a step: rendering blends the last two step
        // states by it so motion is smooth at any frame rate.
        m_renderAlpha = m_clock.alpha();
        processSimEvents();

        m_launchNoticeTimer = std::max(m_launchNoticeTimer - frame, 0.0f);
        m_shotEndNoticeTimer = std::max(m_shotEndNoticeTimer - frame, 0.0f);

        // The hold is presentation: wall-clock, independent of the sim speed.
        if (m_detonationHoldActive)
        {
            m_detonationHoldTimer += frame;
            if (m_detonationHoldTimer >= m_detonationHoldDuration)
            {
                finishDetonationHold();
            }
        }
        else if (!m_followedShot.valid() && !m_world->shots().empty())
        {
            m_followedShot = m_world->shots().back()->id;
        }

        // Sandbox rule: once every aircraft is down and nothing is still in
        // the air (or on camera), a new formation takes off.
        const bool cleared = !targets().empty() && !m_world->anyTargetAlive();
        if ((cleared || targets().empty()) && m_world->shots().empty() && !m_detonationHoldActive && m_targetCount > 0)
        {
            resetTargets();
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
        // Safety check for renderer
        if (!m_renderer)
        {
            std::cerr << "ERROR: Renderer is null in render()" << std::endl;
            return;
        }

        const bool inEngagement = m_screen == Screen::Playing;
        if (m_world)
        {
            m_world->setRenderBlend(m_renderAlpha);
        }
        if (inEngagement)
        {
            updateActiveCameraMode();
        }
        else
        {
            updateTitleCamera(m_lastFrameDeltaTime);
        }
        updateAudioFrame(m_lastFrameDeltaTime);
        m_renderer->beginSceneFrame(glm::vec3(0.58f, 0.69f, 0.82f));
        m_renderer->clearDebugPrimitives();
        m_renderer->renderEnvironment();

        // Loaded rounds on their rails or in the cell, then every shot in the
        // air. A shot that has ended is gone: the blast consumed it.
        if (m_world)
        {
            try
            {
                for (const missilesim::sim::Station &station : m_world->stations())
                {
                    if (station.round)
                    {
                        m_renderer->render(station.round.get());
                    }
                }
                for (const auto &shot : m_world->shots())
                {
                    m_renderer->render(shot->missile.get());
                }

                // Predicted trajectory of the SAM round (tactical overlay: not behind the title).
                const Missile *focus = focusMissile();
                if (m_showTrajectory && inEngagement && focus != nullptr && !focus->isFox2())
                {
                    renderPredictedTrajectory();
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "ERROR: Exception rendering missiles: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "ERROR: Unknown exception rendering missiles" << std::endl;
            }
        }

        renderFighter();

        // Render targets (their screen markers are drawn by the HUD)
        for (const auto &target : targets())
        {
            try
            {
                if (target && target->isActive())
                {
                    m_renderer->render(target.get());
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

        emitFrameVisualEffects(m_lastFrameDeltaTime);
        m_renderer->flushDebugPrimitives();
        m_renderer->renderSceneEffects();
        m_renderer->presentSceneFrame();

        // Setup ImGui frame - wrap in try-catch for safety
        try
        {
            if (ImGui::GetCurrentContext() != nullptr)
            {
                ImGui_ImplOpenGL3_NewFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();

                if (!inEngagement)
                {
                    renderTitleScreen();
                }
                else
                {
                    // The tracker runs even with the HUD hidden so events and
                    // the result card stay consistent when it is shown again.
                    updateHudTracker(ImGui::GetIO().DeltaTime);
                    if (m_hudVisible && m_overlay == Overlay::None)
                    {
                        renderHud();
                        renderPreLaunchSeekerCue();
                        renderSeekerXrayOverlay();
                    }
                    if (m_showUI && m_overlay == Overlay::None)
                    {
                        setupUI();
                    }
                }

                renderMenuOverlays();
                renderScreenFade();
                advanceMenuClocks(ImGui::GetIO().DeltaTime);

                // Persist any setting changed by any screen this frame.
                const std::string settingsSnapshot = buildSettingsSnapshot();
                if (settingsSnapshot != m_lastSettingsSnapshot)
                {
                    scheduleSettingsSave();
                    m_lastSettingsSnapshot = settingsSnapshot;
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
