#include "Application.h"
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
#include <algorithm>
#include <glm/gtx/norm.hpp>
#include "physics/Atmosphere.h"
#include "physics/forces/Drag.h"
#include "physics/forces/Lift.h"

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
            ImGui::StyleColorsDark();

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

        // Set ground collision properties
        m_physicsEngine->setGroundEnabled(m_groundEnabled);
        m_physicsEngine->setGroundRestitution(m_groundRestitution);

        // Create missile with default parameters
        resetMissile();

        // Create targets first, before setting up missile guidance
        resetTargets();

        // Run a small physics update to make sure targets start moving immediately
        if (m_physicsEngine && !m_targets.empty() && m_targetsMove)
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
        // First clear objects that might be using the renderer or physics
        m_missile.reset(); // Release missile before targets to avoid invalid target references

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

                // Process input
                try
                {
                    processInput();
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

void Application::processInput()
{
    // Safety check for window
    if (!m_window)
    {
        return;
    }

    // Handle ESC key to exit
    // static bool escapePressed = false;
    // if (glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    // {
    //     if (!escapePressed)
    //     {
    //         glfwSetWindowShouldClose(m_window, true);
    //         escapePressed = true;
    //     }
    // }
    // else
    // {
    //     escapePressed = false;
    // }

    // Handle SPACE key for pause
    static bool spacePressed = false;
    if (glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS)
    {
        if (!spacePressed)
        {
            m_isPaused = !m_isPaused;
            spacePressed = true;
        }
    }
    else
    {
        spacePressed = false;
    }

    // Camera movement controls with WASD
    const float cameraSpeed = 1.0f;

    if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
    {
        m_renderer->moveCameraForward(cameraSpeed);
    }
    if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS)
    {
        m_renderer->moveCameraForward(-cameraSpeed);
    }
    if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS)
    {
        m_renderer->moveCameraRight(-cameraSpeed);
    }
    if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS)
    {
        m_renderer->moveCameraRight(cameraSpeed);
    }
    if (glfwGetKey(m_window, GLFW_KEY_Q) == GLFW_PRESS)
    {
        m_renderer->moveCameraUp(-cameraSpeed);
    }
    if (glfwGetKey(m_window, GLFW_KEY_E) == GLFW_PRESS)
    {
        m_renderer->moveCameraUp(cameraSpeed);
    }

    // Launch missile with F key
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
    if (!m_enableMouseCamera || ImGui::GetIO().WantCaptureMouse)
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

    // Apply rotation to camera
    m_renderer->rotateCameraYaw(xoffset);
    m_renderer->rotateCameraPitch(yoffset);
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
        }
        else if (action == GLFW_RELEASE)
        {
            // Disable camera rotation and show cursor
            m_enableMouseCamera = false;
            glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
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

                    bool terminateFlight = false;

                    // Treat ground contact as impact rather than allowing endless bouncing/guidance loops.
                    if (m_groundEnabled && prevMissilePos.y > 0.05f && missilePos.y <= 0.01f)
                    {
                        terminateFlight = true;
                    }

                    // Kill bad flights that leave the playable space.
                    if (!terminateFlight &&
                        (glm::length(missilePos) > 5000.0f || missilePos.y > 3000.0f))
                    {
                        terminateFlight = true;
                    }

                    Target* trackedTarget = m_missile->getTargetObject();
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
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Safety check for renderer
        if (!m_renderer)
        {
            std::cerr << "ERROR: Renderer is null in render()" << std::endl;
            return;
        }

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

        // Collect target positions for UI display
        std::vector<std::pair<glm::vec3, std::string>> targetLabels;

        // Render targets
        for (const auto &target : m_targets)
        {
            try
            {
                if (target && target->isActive())
                {
                    m_renderer->render(target.get());

                    // Render target debug info if enabled
                    if (m_showTargetInfo && m_missile)
                    {
                        // Calculate distance to target
                        float distance = glm::length(target->getPosition() - m_missile->getPosition());

                        // Store target position and label for later rendering
                        std::string label = "Target: " + std::to_string(static_cast<int>(distance)) + "m";
                        targetLabels.push_back(std::make_pair(target->getPosition(), label));
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

        renderExplosions();

        // Setup ImGui frame - wrap in try-catch for safety
        try
        {
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // Create UI
            setupUI();

            // Draw target labels using ImGui in world space
            if (m_showTargetInfo && m_window && m_missile)
            {
                glm::mat4 view = glm::lookAt(m_renderer->getCameraPosition(),
                                             m_renderer->getCameraPosition() + m_renderer->getCameraFront(),
                                             m_renderer->getCameraUp());
                glm::mat4 projection = glm::perspective(glm::radians(45.0f),
                                                        (float)m_width / (float)m_height,
                                                        0.1f, 1000.0f);

                for (const auto &targetLabel : targetLabels)
                {
                    // Project 3D world position to screen space
                    glm::vec4 clipSpace = projection * view * glm::vec4(targetLabel.first, 1.0f);

                    if (clipSpace.w > 0)
                    { // Only if it's in front of the camera
                        glm::vec3 ndcSpace = glm::vec3(clipSpace) / clipSpace.w;

                        // Convert from NDC space [-1,1] to screen space [0,width/height]
                        ImVec2 screenPos;
                        screenPos.x = (ndcSpace.x + 1.0f) * 0.5f * m_width;
                        screenPos.y = (1.0f - (ndcSpace.y + 1.0f) * 0.5f) * m_height;

                        // Only draw if it's on screen
                        if (screenPos.x >= 0 && screenPos.x < m_width &&
                            screenPos.y >= 0 && screenPos.y < m_height)
                        {
                            // Add a background rectangle for better visibility
                            ImGui::GetBackgroundDrawList()->AddText(
                                screenPos,
                                IM_COL32(255, 255, 0, 255), // Yellow
                                targetLabel.second.c_str());
                        }
                    }
                }
            }

            // Render ImGui
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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

void Application::renderPredictedTrajectory()
{
    if (!m_missile || !m_renderer)
    {
        return;
    }

    try
    {
        glm::vec3 missilePos = m_missile->getPosition();
        glm::vec3 missileVel = m_missile->getVelocity();

        Target *target = findBestTarget();
        if (!target)
        {
            return;
        }

        glm::vec3 targetPos = target->getPosition();

        glm::vec3 interceptPoint = predictInterceptPoint(missilePos, missileVel, targetPos, target->getVelocity());

        if (m_showInterceptPoint)
        {
            m_renderer->renderLine(missilePos, interceptPoint, glm::vec3(1.0f, 0.0f, 0.0f));
            m_renderer->renderLine(interceptPoint, targetPos, glm::vec3(0.0f, 1.0f, 0.0f));
            m_renderer->renderPoint(interceptPoint, glm::vec3(1.0f, 1.0f, 0.0f), 7.0f);
        }

        std::vector<glm::vec3> trajectoryPoints;
        trajectoryPoints.push_back(missilePos);

        Missile simMissile(missilePos, missileVel, m_missile->getMass(),
                           m_missile->getDragCoefficient(), m_missile->getCrossSectionalArea(),
                           m_missile->getLiftCoefficient());

        simMissile.setGuidanceEnabled(m_missile->isGuidanceEnabled());
        simMissile.setNavigationGain(m_missile->getNavigationGain());
        simMissile.setMaxSteeringForce(m_missile->getMaxSteeringForce());

        simMissile.setThrust(m_missile->getThrust());
        simMissile.setThrustEnabled(m_missile->isThrustEnabled());
        simMissile.setFuel(m_missile->getFuel());
        simMissile.setFuelConsumptionRate(m_missile->getFuelConsumptionRate());
        simMissile.setThrustDirection(m_missile->getThrustDirection());

        float dt = m_trajectoryTime / m_trajectoryPoints;
        Atmosphere atmosphere;
        Drag drag(&atmosphere);
        Lift lift(&atmosphere);

        // Simulate a moving target for the prediction
        Target simTarget = *target;

        for (int i = 1; i < m_trajectoryPoints; ++i)
        {
            simMissile.resetForces();
            simMissile.applyForce(glm::vec3(0.0f, -9.81f * simMissile.getMass(), 0.0f));
            drag.applyTo(&simMissile);
            lift.applyTo(&simMissile);

            // Always update simTarget for every step
            if (target->getMovementPattern() != TargetMovementPattern::STATIONARY)
            {
                simTarget.update(dt);

                if (m_showPredictedTargetPath && i % 5 == 0)
                {
                    m_renderer->renderPoint(simTarget.getPosition(), glm::vec3(0.0f, 1.0f, 0.0f), 3.0f);
                }

                // Set the simulated missile's target to the simulated target's *current* position
                simMissile.setTargetObject(&simTarget);
            }
            else
            {
                simMissile.setTargetObject(target);
            }

            // Apply guidance (will use the updated simTarget position)
            if (simMissile.isGuidanceEnabled() && simMissile.hasTarget())
            {
                simMissile.applyGuidance(dt);
            }

            simMissile.update(dt);
            trajectoryPoints.push_back(simMissile.getPosition());
        }

        for (size_t i = 1; i < trajectoryPoints.size(); ++i)
        {
            float t = static_cast<float>(i) / trajectoryPoints.size();
            glm::vec3 color(1.0f, 1.0f - t, 0.0f);
            m_renderer->renderLine(trajectoryPoints[i - 1], trajectoryPoints[i], color);
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

        timeToIntercept = glm::clamp(timeToIntercept, 0.0f, 10.0f);
        return targetPos + targetVel * timeToIntercept;
    }
    catch (...)
    {
        return targetPos;
    }
}

void Application::setupUI()
{
    ImGuiStyle &style = ImGui::GetStyle();

    // Main control window
    ImGui::SetNextWindowSize(ImVec2(400, 580), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
    ImGui::Begin("Missile Simulator Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Top status bar with score and main controls
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
    ImGui::BeginChild("StatusBar", ImVec2(0, 80), true);

    // Status information
    ImGui::Columns(2);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Score: %d", m_score);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Targets Hit: %d", m_targetHits);
    ImGui::NextColumn();

    // Main action buttons
    if (ImGui::Button(m_isPaused ? "Resume" : "Pause", ImVec2(120, 30)))
    {
        m_isPaused = !m_isPaused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Launch", ImVec2(120, 30)))
    {
        launchMissile();
    }

    ImGui::Columns(1);
    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImGui::Spacing();

    // Create tabbed interface for better organization
    if (ImGui::BeginTabBar("ControlTabs", ImGuiTabBarFlags_None))
    {
        // Simulation tab
        if (ImGui::BeginTabItem("Simulation"))
        {
            ImGui::BeginChild("SimulationPanel", ImVec2(0, 400), true);

            // Simulation speed control
            ImGui::SliderFloat("Simulation Speed", &m_simulationSpeed, 0.1f, 10.0f, "%.1fx");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Adjust simulation speed (1.0 = real time)");

            ImGui::Separator();
            ImGui::Text("Missile Engine Controls:");

            // Display thrust controls with better grouping
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            ImGui::SliderFloat("Thrust Power (N)", &m_missileThrust, 1000.0f, 50000.0f, "%.0f N");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Thrust force in Newtons (higher = more powerful)");

            ImGui::SliderFloat("Fuel Amount (kg)", &m_missileFuel, 10.0f, 1000.0f, "%.1f kg");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Total fuel capacity");

            ImGui::SliderFloat("Fuel Usage (kg/s)", &m_missileFuelConsumptionRate, 0.1f, 10.0f, "%.1f kg/s");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Fuel consumption rate when engine is active");
            ImGui::PopStyleColor();

            ImGui::Separator();
            ImGui::Text("Environment Properties:");

            // Environment properties in a nicer group
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            float gravity = m_physicsEngine->getGravity();
            float airDensity = m_physicsEngine->getAirDensity();

            if (ImGui::SliderFloat("Gravity (m/s²)", &gravity, 0.0f, 20.0f, "%.1f m/s²"))
            {
                m_physicsEngine->setGravity(gravity);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Earth gravity = 9.81 m/s²");

            if (ImGui::SliderFloat("Air Density (kg/m³)", &airDensity, 0.0f, 2.0f, "%.3f kg/m³"))
            {
                m_physicsEngine->setAirDensity(airDensity);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Sea level = 1.225 kg/m³");
            ImGui::PopStyleColor();

            // Ground settings with indentation
            if (ImGui::Checkbox("Enable Ground Collision", &m_groundEnabled))
            {
                m_physicsEngine->setGroundEnabled(m_groundEnabled);
            }

            if (m_groundEnabled)
            {
                ImGui::Indent(20);
                if (ImGui::SliderFloat("Ground Bounciness", &m_groundRestitution, 0.0f, 1.0f, "%.2f"))
                {
                    m_physicsEngine->setGroundRestitution(m_groundRestitution);
                }
                ImGui::Text("0.0 = No bounce, 1.0 = Perfect bounce");
                ImGui::Unindent(20);
            }

            ImGui::Separator();

            // Reset buttons in a horizontal layout
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.1f, 0.1f, 0.8f));
            if (ImGui::Button("Reset Missile"))
            {
                resetMissile();
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset Targets"))
            {
                resetTargets();
            }
            ImGui::PopStyleColor();

            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        // Missile tab
        if (ImGui::BeginTabItem("Missile Properties"))
        {
            ImGui::BeginChild("MissilePanel", ImVec2(0, 400), true);

            // Initial conditions with better formatting
            ImGui::Text("Initial Conditions:");
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            ImGui::InputFloat3("Initial Position (m)", m_initialPosition);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Starting position [x, y, z] in meters");

            ImGui::InputFloat3("Initial Velocity (m/s)", m_initialVelocity);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Starting velocity [x, y, z] in meters/second");
            ImGui::PopStyleColor();

            ImGui::Separator();
            ImGui::Text("Physical Properties:");

            // Physical properties with better formatting
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            ImGui::SliderFloat("Mass (kg)", &m_mass, 10.0f, 1000.0f, "%.1f kg");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Missile mass affects acceleration and momentum");

            ImGui::SliderFloat("Drag Coefficient", &m_dragCoefficient, 0.01f, 1.0f, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Air resistance factor (lower = more aerodynamic)");

            ImGui::SliderFloat("Cross-Sectional Area (m²)", &m_crossSectionalArea, 0.01f, 1.0f, "%.3f m²");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Frontal area exposed to airflow");

            ImGui::SliderFloat("Lift Coefficient", &m_liftCoefficient, 0.0f, 1.0f, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Aerodynamic lift generation capability");
            ImGui::PopStyleColor();

            ImGui::Separator();
            ImGui::Text("Guidance System:");

            // Guidance parameters with better formatting
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            ImGui::Checkbox("Enable Guidance", &m_guidanceEnabled);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Turn on/off target tracking system");

            ImGui::SliderFloat("Navigation Gain", &m_navigationGain, 0.1f, 10.0f, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("How aggressively missile corrects course (higher = more responsive)");

            ImGui::SliderFloat("Max Steering Force (N)", &m_maxSteeringForce, 1000.0f, 50000.0f, "%.0f N");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Maximum lateral force available for seeker/autopilot corrections");
            ImGui::PopStyleColor();

            // Apply button with color - always show regardless of property changes
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.5f, 0.2f, 0.8f));
            if (ImGui::Button("Apply Properties", ImVec2(-1, 30)))
            {
                // Update missile with new properties
                m_missile->setMass(m_mass);
                m_missile->setDragCoefficient(m_dragCoefficient);
                m_missile->setCrossSectionalArea(m_crossSectionalArea);
                m_missile->setLiftCoefficient(m_liftCoefficient);
                m_missile->setGuidanceEnabled(m_guidanceEnabled);
                m_missile->setNavigationGain(m_navigationGain);
                m_missile->setMaxSteeringForce(m_maxSteeringForce);
            }
            ImGui::PopStyleColor();

            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        // Targets tab
        if (ImGui::BeginTabItem("Targets"))
        {
            ImGui::BeginChild("TargetsPanel", ImVec2(0, 400), true);

            // Target count with display of active targets
            int activeTargets = 0;
            for (const auto &target : m_targets)
            {
                if (target && target->isActive())
                {
                    activeTargets++;
                }
            }

            ImGui::Text("Active Targets: %d/%d", activeTargets, (int)m_targets.size());

            if (ImGui::SliderInt("Target Count", &m_targetCount, 1, 10, "%d"))
            {
                resetTargets();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Number of targets to create");

            ImGui::SliderFloat("Target Spawn Distance", &m_targetSpawnDistance, 50.0f, 500.0f, "%.0f m");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("How far targets spawn from origin");

            ImGui::Separator();
            ImGui::Text("Target Movement:");

            // Target movement controls with better organization
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            if (ImGui::Checkbox("Moving Targets", &m_targetsMove))
            {
                for (const auto &target : m_targets)
                {
                    if (target && !m_targetsMove)
                    {
                        target->setMovementPattern(TargetMovementPattern::STATIONARY);
                    }
                }
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Enable/disable target movement");

            if (m_targetsMove)
            {
                ImGui::Indent(20);

                ImGui::Checkbox("Randomize Patterns", &m_randomizeTargetMovement);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Each target gets a random movement pattern");

                if (!m_randomizeTargetMovement)
                {
                    const char *patternItems[] = {
                        "Stationary", "Linear", "Circular", "Sinusoidal", "Random"};

                    int currentPattern = static_cast<int>(m_targetMovementPattern);
                    if (ImGui::Combo("Movement Pattern", &currentPattern, patternItems, IM_ARRAYSIZE(patternItems)))
                    {
                        m_targetMovementPattern = static_cast<TargetMovementPattern>(currentPattern);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Select how targets will move");
                }

                ImGui::SliderFloat("Movement Speed", &m_targetMovementSpeed, 1.0f, 50.0f, "%.1f m/s");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("How fast targets move");

                ImGui::SliderFloat("Movement Range", &m_targetMovementAmplitude, 10.0f, 200.0f, "%.1f m");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Maximum distance targets move from their center point");

                ImGui::SliderFloat("Movement Period", &m_targetMovementPeriod, 2.0f, 30.0f, "%.1f s");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Time to complete one movement cycle");

                // Apply button
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.5f, 0.2f, 0.8f));
                if (ImGui::Button("Apply Movement Settings", ImVec2(-1, 25)))
                {
                    // Update all targets with new settings
                    for (const auto &target : m_targets)
                    {
                        if (target)
                        {
                            if (m_targetsMove)
                            {
                                // Only change pattern if not randomizing
                                if (!m_randomizeTargetMovement)
                                {
                                    target->setMovementPattern(m_targetMovementPattern);
                                }

                                target->setMovementSpeed(m_targetMovementSpeed);
                                target->setMovementAmplitude(m_targetMovementAmplitude);
                                target->setMovementPeriod(m_targetMovementPeriod);
                            }
                            else
                            {
                                target->setMovementPattern(TargetMovementPattern::STATIONARY);
                            }
                        }
                    }
                }
                ImGui::PopStyleColor();

                ImGui::Unindent(20);
            }
            ImGui::PopStyleColor();

            ImGui::Separator();

            // Target status display
            if (ImGui::CollapsingHeader("Target Status"))
            {
                for (size_t i = 0; i < m_targets.size(); i++)
                {
                    const auto &target = m_targets[i];
                    const glm::vec3 &pos = target->getPosition();

                    ImGui::PushID(static_cast<int>(i));
                    if (target->isActive())
                    {
                        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f),
                                           "Target %d: (%.1f, %.1f, %.1f)",
                                           (int)i + 1, pos.x, pos.y, pos.z);

                        ImGui::SameLine();
                        ImGui::TextDisabled("(?)");
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::BeginTooltip();
                            ImGui::Text("Pattern: %d, Speed: %.1f",
                                        static_cast<int>(target->getMovementPattern()),
                                        target->getMovementSpeed());
                            ImGui::EndTooltip();
                        }
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f),
                                           "Target %d: Destroyed", (int)i + 1);
                    }
                    ImGui::PopID();
                }
            }

            ImGui::Separator();

            // Reset button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.1f, 0.1f, 0.8f));
            if (ImGui::Button("Reset All Targets", ImVec2(-1, 30)))
            {
                resetTargets();
            }
            ImGui::PopStyleColor();

            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        // Visualization tab
        if (ImGui::BeginTabItem("Visualization"))
        {
            ImGui::BeginChild("VisualizationPanel", ImVec2(0, 400), true);

            // Camera controls
            ImGui::Text("Camera Controls:");
            ImGui::TextWrapped("Right-click and drag to look around. WASD to move.");

            ImGui::Separator();

            // Camera settings
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            float fov = m_renderer->getCameraFOV();
            if (ImGui::SliderFloat("Field of View", &fov, 10.0f, 120.0f, "%.1f°"))
            {
                m_renderer->setCameraFOV(fov);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Camera field of view (zoom level)");

            float cameraSpeed = m_renderer->getCameraSpeed();
            if (ImGui::SliderFloat("Camera Speed", &cameraSpeed, 0.1f, 10.0f, "%.1f"))
            {
                m_renderer->setCameraSpeed(cameraSpeed);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Movement speed when using WASD keys");
            ImGui::PopStyleColor();

            // Display current camera position
            glm::vec3 cameraPos = m_renderer->getCameraPosition();
            ImGui::Text("Camera Position: (%.1f, %.1f, %.1f)", cameraPos.x, cameraPos.y, cameraPos.z);

            ImGui::Separator();
            ImGui::Text("Visualization Options:");

            // Visualization options
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.2f, 0.8f));
            ImGui::Checkbox("Show Trajectory", &m_showTrajectory);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Display predicted missile path");

            ImGui::Checkbox("Show Target Info", &m_showTargetInfo);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Display distance labels on targets");

            ImGui::SliderInt("Trajectory Points", &m_trajectoryPoints, 10, 300);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Number of points to calculate (higher = more detail but slower)");

            ImGui::SliderFloat("Trajectory Time (s)", &m_trajectoryTime, 0.5f, 10.0f, "%.1f s");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("How far into the future to predict");

            // Add guidance visualization options
            ImGui::Separator();
            ImGui::Text("Guidance Visualization:");

            static bool showPredictedTargetPath = true;
            if (ImGui::Checkbox("Show Target Prediction", &showPredictedTargetPath))
            {
                // Update the flag in the trajectory renderer
                m_showPredictedTargetPath = showPredictedTargetPath;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show dots indicating predicted target positions");

            static bool showInterceptPoint = true;
            if (ImGui::Checkbox("Show Intercept Point", &showInterceptPoint))
            {
                // Update the flag in the trajectory renderer
                m_showInterceptPoint = showInterceptPoint;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show the predicted missile-target intercept point");
            ImGui::PopStyleColor();

            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        // Status tab for missile telemetry
        if (ImGui::BeginTabItem("Telemetry"))
        {
            ImGui::BeginChild("TelemetryPanel", ImVec2(0, 400), true);

            // Position and velocity data
            ImGui::Text("Missile Data:");

            glm::vec3 position = m_missile->getPosition();
            glm::vec3 velocity = m_missile->getVelocity();
            glm::vec3 acceleration = m_missile->getAcceleration();

            ImGui::Columns(2);

            // Position data
            ImGui::Text("Position:");
            ImGui::Indent(10);
            ImGui::Text("X: %.2f m", position.x);
            ImGui::Text("Y: %.2f m", position.y);
            ImGui::Text("Z: %.2f m", position.z);
            ImGui::Unindent(10);

            ImGui::NextColumn();

            // Velocity data
            ImGui::Text("Velocity:");
            ImGui::Indent(10);
            ImGui::Text("X: %.2f m/s", velocity.x);
            ImGui::Text("Y: %.2f m/s", velocity.y);
            ImGui::Text("Z: %.2f m/s", velocity.z);
            ImGui::Text("Speed: %.2f m/s", glm::length(velocity));
            ImGui::Unindent(10);

            ImGui::Columns(1);

            // Acceleration vector
            ImGui::Text("Acceleration: (%.2f, %.2f, %.2f) m/s²",
                        acceleration.x, acceleration.y, acceleration.z);

            ImGui::Separator();

            // Thrust information in a styled box
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.2f, 0.7f));
            ImGui::BeginChild("ThrustInfo", ImVec2(0, 120), true);

            ImGui::Text("Engine Status:");

            bool thrustEnabled = m_missile->isThrustEnabled();
            float fuel = m_missile->getFuel();
            float thrust = m_missile->getThrust();

            // Engine status with appropriate colors
            if (thrustEnabled)
            {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "● ENGINE ACTIVE");
            }
            else
            {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "○ ENGINE OFF");
            }

            // Thrust force
            ImGui::Text("Thrust Force: %.0f N", thrust);

            // Fuel gauge with color gradient
            float fuelPercent = fuel / m_missileFuel;
            ImGui::Text("Fuel Remaining: %.1f kg (%.1f%%)", fuel, fuelPercent * 100.0f);

            // Color the progress bar based on remaining fuel
            ImVec4 fuelColor;
            if (fuelPercent > 0.5f)
            {
                fuelColor = ImVec4(0.0f, 0.8f, 0.0f, 1.0f);
            }
            else if (fuelPercent > 0.25f)
            {
                fuelColor = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);
            }
            else
            {
                fuelColor = ImVec4(0.8f, 0.0f, 0.0f, 1.0f);
            }

            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, fuelColor);
            ImGui::ProgressBar(fuelPercent, ImVec2(-1, 0), "");
            ImGui::PopStyleColor();

            ImGui::EndChild();
            ImGui::PopStyleColor();

            ImGui::Spacing();

            // Status information
            if (position.y <= 0.001f && m_groundEnabled)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "⚠ Missile is on the ground");
            }

            if (m_missile->isGuidanceEnabled() && m_missile->hasTarget())
            {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ Guidance active - Target locked");
            }
            else if (m_missile->isGuidanceEnabled())
            {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "⚠ Guidance active - No target");
            }

            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
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

        // Set target movement properties if enabled
        if (m_targetsMove)
        {
            // Set movement pattern (either the specified one or random)
            TargetMovementPattern pattern = m_targetMovementPattern;
            if (m_randomizeTargetMovement)
            {
                std::uniform_int_distribution<int> patternDist(0, 4);
                pattern = static_cast<TargetMovementPattern>(patternDist(m_rng));
            }
            target->setMovementPattern(pattern);

            // Set movement speed
            target->setMovementSpeed(m_targetMovementSpeed);

            // Set movement amplitude
            target->setMovementAmplitude(m_targetMovementAmplitude);

            // Set movement center
            target->setMovementCenter(validPosition);

            // Set random movement direction
            std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * glm::pi<float>());
            glm::vec3 direction;
            direction.x = std::cos(angleDist(m_rng));
            direction.y = 0.1f; // Small vertical component
            direction.z = std::sin(angleDist(m_rng));
            target->setMovementDirection(glm::normalize(direction));

            // Set movement period
            target->setMovementPeriod(m_targetMovementPeriod);
        }

        // Safety check before adding to physics engine
        if (target && m_physicsEngine)
        {
            m_physicsEngine->addTarget(target.get());
            m_targets.push_back(std::move(target));
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
        // Create a random distribution for angles and height
        std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * 3.14159f); // 0 to 2π
        std::uniform_real_distribution<float> heightDist(50.0f, 150.0f);        // 50 to 150 meters
        std::uniform_real_distribution<float> radiusDist(3.0f, 7.0f);           // 3 to 7 meters radius
        std::uniform_int_distribution<int> movementPatternDist(0, 4);           // Random movement pattern

        // Validate targetSpawnDistance
        if (m_targetSpawnDistance <= 0.0f || std::isnan(m_targetSpawnDistance) || std::isinf(m_targetSpawnDistance))
        {
            m_targetSpawnDistance = 200.0f; // Reset to default
        }

        // Generate random spherical coordinates
        float angle = angleDist(m_rng);
        float height = heightDist(m_rng);
        float radius = radiusDist(m_rng);

        // Convert to Cartesian coordinates
        float x = m_targetSpawnDistance * cos(angle);
        float z = m_targetSpawnDistance * sin(angle);

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

        // Set target movement properties if enabled
        if (m_targetsMove)
        {
            // Select a random movement pattern or use the specified one
            TargetMovementPattern pattern;
            if (m_randomizeTargetMovement)
            {
                int patternIndex = movementPatternDist(m_rng);
                pattern = static_cast<TargetMovementPattern>(patternIndex);
            }
            else
            {
                pattern = m_targetMovementPattern;
            }

            // Set movement pattern
            target->setMovementPattern(pattern);

            // Set movement speed (slightly randomized)
            std::uniform_real_distribution<float> speedVariation(0.8f, 1.2f);
            target->setMovementSpeed(m_targetMovementSpeed * speedVariation(m_rng));

            // Set movement amplitude
            std::uniform_real_distribution<float> ampVariation(0.7f, 1.3f);
            target->setMovementAmplitude(m_targetMovementAmplitude * ampVariation(m_rng));

            // Set movement center to initial position
            target->setMovementCenter(target->getPosition());

            // Set random movement direction
            glm::vec3 direction;
            direction.x = angleDist(m_rng) - glm::pi<float>();
            direction.y = (angleDist(m_rng) - glm::pi<float>()) * 0.3f; // Less vertical movement
            direction.z = angleDist(m_rng) - glm::pi<float>();
            target->setMovementDirection(glm::normalize(direction));

            // Set movement period
            std::uniform_real_distribution<float> periodDist(m_targetMovementPeriod * 0.7f, m_targetMovementPeriod * 1.3f);
            target->setMovementPeriod(periodDist(m_rng));
        }

        // Safety check before adding to physics engine
        if (target && m_physicsEngine)
        {
            m_physicsEngine->addTarget(target.get());
            m_targets.push_back(std::move(target));
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
        {                      // Set reasonable limits
            m_targetCount = 3; // Default to a sensible number
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

        // Give each target an initial update to make sure they start moving right away
        // This ensures movement begins immediately without waiting for the next physics tick
        if (m_physicsEngine && m_targetsMove)
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
        if (m_navigationGain <= 0.0f || std::isnan(m_navigationGain) || std::isinf(m_navigationGain))
        {
            m_navigationGain = 4.0f;
        }

        if (m_maxSteeringForce <= 0.0f || std::isnan(m_maxSteeringForce) || std::isinf(m_maxSteeringForce))
        {
            m_maxSteeringForce = 20000.0f;
        }

        // Set guidance parameters
        m_missile->setGuidanceEnabled(m_guidanceEnabled);
        m_missile->setNavigationGain(m_navigationGain);
        m_missile->setMaxSteeringForce(m_maxSteeringForce);

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
    for (const auto& explosion : m_explosions) {
        m_renderer->renderExplosion(explosion.position, explosion.size);
    }
}
