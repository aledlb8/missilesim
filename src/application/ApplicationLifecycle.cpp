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
        m_audioSystem = std::make_unique<AudioSystem>();
        m_audioSystem->initialize();

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
        if (!m_window && !m_renderer && !m_physicsEngine && !m_audioSystem)
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
        if (m_audioSystem)
        {
            m_audioSystem->shutdown();
            m_audioSystem.reset();
        }
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

        if (m_renderer)
        {
            m_renderer->updateEffects(deltaTime);
        }

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
        // Safety check for renderer
        if (!m_renderer)
        {
            std::cerr << "ERROR: Renderer is null in render()" << std::endl;
            return;
        }

        updateActiveCameraMode();
        updateAudioFrame(m_lastFrameDeltaTime);
        m_renderer->beginSceneFrame(glm::vec3(0.58f, 0.69f, 0.82f));
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

                renderPreLaunchSeekerCue();

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