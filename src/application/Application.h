#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <deque>

#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "objects/Missile.h"
#include "objects/Target.h"

// Structure to represent a visual explosion effect
struct ExplosionEffect {
    glm::vec3 position;   // Position of the explosion
    float timeRemaining;  // Time remaining for the effect to display
    float size;           // Current size of the explosion
};

class Application {
public:
    Application(int width = 1280, int height = 720, const std::string& title = "Missile Simulator");
    ~Application();

    void run();
    void initialize();
    void shutdown();
    
private:
    void processInput();
    void update(float deltaTime);
    void render();
    void setupUI();
    
    // Mouse control functions
    void mouseCallback(double xpos, double ypos);
    void mouseButtonCallback(int button, int action);
    
    // Target functions
    void createTarget(const glm::vec3& position, float radius = 5.0f);
    void createRandomTarget();
    void resetTargets();
    
    // Missile functions
    void launchMissile();
    void resetMissile();
    Target* findBestTarget();
    void terminateMissileFlight(const glm::vec3& position, bool createEffect = true);
    
    // Visual effects
    void createExplosion(const glm::vec3& position);
    void updateExplosions(float deltaTime);
    void renderExplosions();
    
    // visualization
    void renderPredictedTrajectory();
    glm::vec3 predictInterceptPoint(const glm::vec3& missilePos, const glm::vec3& missileVel,
                                   const glm::vec3& targetPos, const glm::vec3& targetVel);
    
    // Window properties
    int m_width;
    int m_height;
    std::string m_title;
    GLFWwindow* m_window;
    
    // Mouse camera control properties
    float m_lastMouseX;
    float m_lastMouseY;
    bool m_firstMouse;
    bool m_enableMouseCamera = false;
    
    // Simulation components
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<Missile> m_missile;
    std::vector<std::unique_ptr<Target>> m_targets;
    
    // Visual effects
    std::deque<ExplosionEffect> m_explosions;
    float m_explosionDuration = 1.0f;  // Duration of explosion effect in seconds
    float m_explosionMaxSize = 10.0f;  // Maximum size of explosion
    
    // visualization options
    bool m_showTrajectory = true;     // Whether to show predicted trajectory
    bool m_showTargetInfo = true;     // Whether to show target information
    bool m_showPredictedTargetPath = true;  // Whether to show predicted target path
    bool m_showInterceptPoint = true;       // Whether to show intercept point
    int m_trajectoryPoints = 100;     // Number of points in trajectory visualization
    float m_trajectoryTime = 5.0f;    // Time in seconds to predict trajectory
    
    // Simulation properties
    float m_timeStep = 0.01f;  // Physics time step in seconds
    float m_simulationSpeed = 1.0f;
    bool m_isPaused = false;
    
    // Ground properties
    bool m_groundEnabled = true;
    float m_groundRestitution = 0.5f;
    
    // UI properties
    float m_initialVelocity[3] = {0.0f, 0.0f, 50.0f};  // Initial velocity in m/s
    float m_initialPosition[3] = {0.0f, 0.0f, 0.0f};   // Initial position in m
    float m_mass = 100.0f;                            // Mass in kg
    float m_dragCoefficient = 0.1f;                   // Drag coefficient
    float m_crossSectionalArea = 0.1f;                // Cross-sectional area in m²
    float m_liftCoefficient = 0.1f;                   // Lift coefficient
    
    // Missile guidance properties
    bool m_guidanceEnabled = true;
    float m_navigationGain = 4.0f;
    float m_maxSteeringForce = 20000.0f;
    
    // Missile thrust properties
    float m_missileThrust = 10000.0f;  // Thrust force in Newtons
    float m_missileFuel = 100.0f;      // Fuel amount in kg
    float m_missileFuelConsumptionRate = 0.5f;  // Fuel consumption in kg/second
    
    // Target properties
    float m_targetSpawnDistance = 200.0f;  // Distance from origin to spawn targets
    int m_targetCount = 1;                // Number of targets to create
    
    // Target movement properties
    bool m_targetsMove = true;                               // Whether targets should move
    bool m_randomizeTargetMovement = true;                   // Whether to randomize movement patterns
    TargetMovementPattern m_targetMovementPattern = TargetMovementPattern::CIRCULAR; // Default movement pattern
    float m_targetMovementSpeed = 10.0f;                     // Movement speed in m/s
    float m_targetMovementAmplitude = 50.0f;                 // Movement amplitude/range in m
    float m_targetMovementPeriod = 10.0f;                    // Time to complete one movement cycle in s
    
    // Score tracking
    int m_score = 0;                      // Player's score
    int m_targetHits = 0;                 // Number of targets hit

    // Missile flight state
    bool m_missileInFlight = false;
    float m_missileFlightTime = 0.0f;
    float m_closestTargetDistance = 1000000.0f;
    
    // Random number generator
    std::mt19937 m_rng;
}; 
