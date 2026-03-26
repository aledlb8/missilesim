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
#include <chrono>

#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "audio/AudioSystem.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"

// Structure to represent a visual explosion effect
struct ExplosionEffect
{
    glm::vec3 position;  // Position of the explosion
    float timeRemaining; // Time remaining for the effect to display
    float size;          // Current size of the explosion
};

class Application
{
public:
    Application(int width = 1280, int height = 720, const std::string &title = "Missile Simulator");
    ~Application();

    void run();
    void initialize();
    void shutdown();

private:
    enum class CameraMode
    {
        FREE,
        MISSILE,
        FIGHTER_JET
    };

    struct FreeCameraState
    {
        glm::vec3 position{0.0f};
        glm::vec3 target{0.0f};
        float fov = 50.0f;
        float speed = 35.0f;
        bool valid = false;
    };

    struct ChaseCameraState
    {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float distance = 0.0f;
        float returnBlend = 0.0f;
        bool initialized = false;
    };

    struct TrajectoryPreviewConfig
    {
        glm::vec3 thrustDirection{0.0f};
        float dryMass = 0.0f;
        float dragCoefficient = 0.0f;
        float crossSectionalArea = 0.0f;
        float liftCoefficient = 0.0f;
        bool guidanceEnabled = false;
        float navigationGain = 0.0f;
        float maxSteeringForce = 0.0f;
        float trackingAngle = 0.0f;
        float proximityFuseRadius = 0.0f;
        float countermeasureResistance = 0.0f;
        bool terrainAvoidanceEnabled = false;
        float terrainClearance = 0.0f;
        float terrainLookAheadTime = 0.0f;
        float thrust = 0.0f;
        bool thrustEnabled = false;
        float fuel = 0.0f;
        float fuelConsumptionRate = 0.0f;
        int trajectoryPoints = 0;
        float trajectoryTime = 0.0f;
        float gravityMagnitude = 0.0f;
        float airDensity = 0.0f;
    };

    struct TrajectoryPreviewCache
    {
        std::vector<glm::vec3> missilePoints;
        std::vector<glm::vec3> targetPoints;
        glm::vec3 interceptPoint{0.0f};
        glm::vec3 lastMissilePosition{0.0f};
        glm::vec3 lastMissileVelocity{0.0f};
        glm::vec3 lastTargetPosition{0.0f};
        glm::vec3 lastTargetVelocity{0.0f};
        TrajectoryPreviewConfig config;
        const Target *target = nullptr;
        std::chrono::steady_clock::time_point lastRefresh{};
        bool valid = false;
    };

    void processInput(float deltaTime);
    void update(float deltaTime);
    void render();
    void setupUI();
    void renderMinimalHUD();
    void frameEngagementCamera();
    void setCameraMode(CameraMode mode, bool frameFreeCamera = false);
    void updateActiveCameraMode();
    void updateMissileCamera();
    void updateFighterJetCamera();
    void captureFreeCameraState();
    void restoreFreeCameraState();
    void resetChaseCameraState();
    void primeChaseCameraState(const glm::vec3 &focusPoint);
    void updateChaseOrbit(float yawDeltaDegrees, float pitchDeltaDegrees);
    void applyChaseCamera(const glm::vec3 &focusPoint,
                          const glm::vec3 &defaultPosition,
                          const glm::vec3 &defaultTarget);
    void releaseMouseCameraCapture();
    const char *getCameraModeLabel() const;
    void updateEnvironmentScale();
    float computeEngagementRadius() const;
    std::string buildSettingsSnapshot() const;
    bool loadSettings();
    void saveSettings();
    void scheduleSettingsSave();
    void flushSettingsAutosave(float deltaTime);

    // Mouse control functions
    void mouseCallback(double xpos, double ypos);
    void mouseButtonCallback(int button, int action);

    // Target functions
    void createTarget(const glm::vec3 &position, float radius = 5.0f);
    void createRandomTarget();
    void resetTargets();
    void createFlare(const FlareLaunchRequest &request);
    void collectPendingTargetFlares();
    void removeInactiveFlares();
    void clearFlares();

    // Missile functions
    void launchMissile();
    void resetMissile();
    Target *findBestTarget();
    bool projectTargetToSeekerScreen(const Target *target, ImVec2 &screenPosition, float *pixelDistanceFromCenter = nullptr) const;
    Target *findSeekerCueTarget() const;
    Target *getTrackedMissileTarget() const;
    const char *getMissileSeekerStateLabel() const;
    const char *getMissileSeekerTrackLabel() const;
    void updatePreLaunchSeekerLock();
    void renderPreLaunchSeekerCue() const;
    void terminateMissileFlight(const glm::vec3 &position, bool createEffect = true);

    // Visual effects
    void createExplosion(const glm::vec3 &position);
    void updateExplosions(float deltaTime);
    void renderExplosions();
    void emitFrameVisualEffects(float deltaTime);
    void updateAudioFrame(float deltaTime);

    // visualization
    void renderPredictedTrajectory();
    TrajectoryPreviewConfig captureTrajectoryPreviewConfig() const;
    bool shouldRefreshTrajectoryPreviewCache(Target *target, const TrajectoryPreviewConfig &config) const;
    void updateTrajectoryPreviewCache(Target *target, const TrajectoryPreviewConfig &config);
    void invalidateTrajectoryPreviewCache();
    glm::vec3 predictInterceptPoint(const glm::vec3 &missilePos, const glm::vec3 &missileVel,
                                    const glm::vec3 &targetPos, const glm::vec3 &targetVel);

    // Window properties
    int m_width;
    int m_height;
    std::string m_title;
    GLFWwindow *m_window;

    // Mouse camera control properties
    float m_lastMouseX;
    float m_lastMouseY;
    bool m_firstMouse;
    bool m_enableMouseCamera = false;
    CameraMode m_cameraMode = CameraMode::FREE;
    FreeCameraState m_freeCameraState;
    ChaseCameraState m_chaseCameraState;
    float m_lastFrameDeltaTime = 0.016f;

    // Simulation components
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<AudioSystem> m_audioSystem;
    std::unique_ptr<Missile> m_missile;
    std::vector<std::unique_ptr<Target>> m_targets;
    std::vector<std::unique_ptr<Flare>> m_flares;

    // Visual effects
    std::deque<ExplosionEffect> m_explosions;
    float m_explosionDuration = 1.0f; // Duration of explosion effect in seconds
    float m_explosionMaxSize = 10.0f; // Maximum size of explosion

    // visualization options
    bool m_showTrajectory = true;          // Whether to show predicted trajectory
    bool m_showTargetInfo = true;          // Whether to show target information
    bool m_showPredictedTargetPath = true; // Whether to show predicted target path
    bool m_showInterceptPoint = true;      // Whether to show intercept point
    int m_trajectoryPoints = 140;          // Number of points in trajectory visualization
    float m_trajectoryTime = 12.0f;        // Time in seconds to predict trajectory
    TrajectoryPreviewCache m_trajectoryPreviewCache;
    std::chrono::milliseconds m_trajectoryPreviewRefreshInterval{100};
    bool m_seekerCueEnabled = false;
    float m_seekerCueRadiusPixels = 44.0f;

    // Simulation properties
    float m_timeStep = 0.01f; // Physics time step in seconds
    float m_simulationSpeed = 1.0f;
    bool m_isPaused = false;

    // Ground properties
    bool m_groundEnabled = true;
    float m_groundRestitution = 0.5f;

    // UI properties
    bool m_showUI = true;
    float m_initialVelocity[3] = {0.0f, 0.0f, 50.0f}; // Initial velocity in m/s
    float m_initialPosition[3] = {0.0f, 0.0f, 0.0f};  // Initial position in m
    float m_mass = 100.0f;                            // Dry mass in kg
    float m_dragCoefficient = 0.1f;                   // Drag coefficient
    float m_crossSectionalArea = 0.1f;                // Cross-sectional area in m²
    float m_liftCoefficient = 0.1f;                   // Lift coefficient

    // Missile guidance properties
    bool m_guidanceEnabled = true;
    float m_navigationGain = 4.0f;
    float m_maxSteeringForce = 20000.0f;
    float m_trackingAngle = 85.0f;
    float m_proximityFuseRadius = 18.0f;
    float m_countermeasureResistance = 0.65f;
    bool m_terrainAvoidanceEnabled = true;
    float m_terrainClearance = 90.0f;
    float m_terrainLookAheadTime = 6.0f;

    // Missile thrust properties
    float m_missileThrust = 10000.0f;          // Thrust force in Newtons
    float m_missileFuel = 100.0f;              // Fuel amount in kg
    float m_missileFuelConsumptionRate = 0.5f; // Fuel consumption in kg/second

    // Target properties
    int m_targetCount = 1; // Number of targets to create
    TargetAIConfig m_targetAIConfig;

    // Score tracking
    int m_score = 0;      // Player's score
    int m_targetHits = 0; // Number of targets hit

    // Missile flight state
    bool m_missileInFlight = false;
    float m_missileFlightTime = 0.0f;
    float m_closestTargetDistance = 1000000.0f;

    // Random number generator
    std::mt19937 m_rng;

    // Autosaved user settings
    std::string m_settingsPath = "config/user_settings.ini";
    bool m_settingsDirty = false;
    float m_settingsAutosaveDelay = 0.0f;
    std::string m_lastSettingsSnapshot;
    float m_savedGravity = 9.81f;
    float m_savedAirDensity = 1.225f;
    float m_savedCameraFOV = 50.0f;
    float m_savedCameraSpeed = 35.0f;
};
