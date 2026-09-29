#pragma once

#include <glm/glm.hpp>
#include <chrono>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include "objects/Target.h"
#include "sim/SimulationConfig.h"
#include "CameraRig.h"

class AudioSystem;
class Fighter;
class Flare;
struct FlareLaunchRequest;
struct GLFWwindow;
struct ImVec2;
class Missile;
class PhysicsEngine;
class Renderer;

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

    enum class PlayerRole
    {
        Sam,
        Fighter
    };

    enum class DisplayMode
    {
        Windowed,   // decorated window, centred on first launch
        Borderless, // undecorated window covering the monitor (F11)
        Fullscreen  // exclusive fullscreen at the monitor's current video mode
    };

    // Front-end flow: the title screen runs over the live scene; overlays sit
    // on top of whichever screen is active.
    enum class Screen
    {
        Title,
        Playing
    };

    enum class Overlay
    {
        None,
        Pause,
        Settings
    };

    enum class SettingsPage
    {
        Display,
        Graphics,
        Audio,
        Controls
    };

    // HUD bookkeeping (ApplicationHUD.cpp).
    struct HudTracker
    {
        float engagementTime = 0.0f;
    };

    // Last windowed client rect, restored when leaving borderless/fullscreen.
    struct WindowedPlacement
    {
        int x = 0;
        int y = 0;
        int width = 0;
        int height = 0;
        bool valid = false;
    };

    struct FreeCameraState
    {
        glm::vec3 position{0.0f};
        glm::vec3 target{0.0f};
        float fov = 50.0f;
        float speed = 35.0f;
        bool valid = false;
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

    // Drives a cold-launch profile: a soft ejection charge lobs the round
    // vertically clear of the cell, the main motor lights in mid-air, then the
    // missile pitches over toward the target before guidance takes the steer.
    struct MissileLaunchSequence
    {
        bool active = false;
        bool motorIgnited = false;
        bool guidanceArmed = false;
        bool boostComplete = false;
        bool ignitionEffectEmitted = false;
        bool restoreGuidanceEnabled = true;
        float elapsed = 0.0f;
        float ignitionDelay = 0.85f;                 // eject coast time before the motor lights
        float thrustRampDuration = 0.30f;            // ignition -> full throttle (punchy)
        float guidanceArmDelay = 1.30f;              // backstop: hand off to guidance by here
        float boostDuration = 1.5f;                  // high-thrust booster burn after ignition
        float sustainThrust = 10000.0f;              // configured motor thrust, restored post-boost
        glm::vec3 ejectDirection{0.0f, 1.0f, 0.0f};  // near-vertical ejection vector
        glm::vec3 launchDirection{0.0f, 0.0f, 1.0f}; // fallback boost aim toward target
        glm::vec3 aimDirection{0.0f, 1.0f, 0.0f};    // live pitch-over aim, rate-limited
    };

    // Window management (ApplicationWindow.cpp)
    void createMainWindow();
    void setDisplayMode(DisplayMode mode);
    void toggleFullscreen();
    // F12: the finished frame (scene + HUD) to screenshots/<timestamp>.png.
    void saveScreenshot();
    void updateWindowFrame();
    void revealWindowAfterFirstFrame();
    bool isWindowMinimized() const;
    float windowContentScale() const;
    void setVsyncEnabled(bool enabled);
    static const char *displayModeName(DisplayMode mode);

    // Menus and screen flow (ApplicationMenus.cpp)
    bool gameplayInputEnabled() const;
    void renderTitleScreen();
    void renderMenuOverlays();
    void renderPauseMenu();
    void renderSettingsScreen();
    void renderSettingsPage(SettingsPage page);
    void renderScreenFade();
    void advanceMenuClocks(float deltaTime);
    void updateTitleCamera(float deltaTime);
    void startEngagement();
    void returnToTitle();
    void openPauseMenu();
    void closePauseMenu();
    void openSettings(SettingsPage page);
    void closeSettings();
    void beginScreenFade(float duration);
    void cycleCameraMode();

    // In-engagement HUD (ApplicationHUD.cpp)
    void renderHud();
    void updateHudTracker(float deltaTime);
    const char *missionStateLabel() const;
    float hudRightInset() const;

    void processInput(float deltaTime);
    void update(float deltaTime);
    void render();
    void setupUI();
    void frameEngagementCamera();
    void setCameraMode(CameraMode mode, bool frameFreeCamera = false);
    void updateActiveCameraMode();
    void updateMissileCamera();
    void updateFighterJetCamera();
    void captureFreeCameraState();
    void restoreFreeCameraState();
    void resetChaseCameraState();
    void applyCameraPose(const CameraPose &pose);
    void releaseMouseCameraCapture();
    // Mouse aim owns the (hidden, raw) cursor while flying the fighter from
    // its own camera with no panel or menu open.
    bool mouseAimActive() const;
    void updateCursorCapture();
    void resetAimCamera();
    const char *getCameraModeLabel() const;
    void updateEnvironmentScale();
    float computeEngagementRadius() const;
    std::string buildSettingsSnapshot() const;
    bool loadSettings();
    void saveSettings();
    void scheduleSettingsSave();
    void flushSettingsAutosave(float deltaTime);
    void applySimulationConfigDefaults();

    // Mouse control functions
    void mouseCallback(double xpos, double ypos);
    void mouseButtonCallback(int button, int action);
    void scrollCallback(double yoffset);

    // Render interpolation across fixed physics steps (see PhysicsObject).
    void beginFixedStepForAll();
    void setRenderBlendForAll(float alpha);

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
    void setPlayerRole(PlayerRole role);
    void selectFox2(const char *id);
    void launchFox2FromRail();
    void stageFox2OnRail();
    void rearmFighter();
    void updateFighter(float deltaTime);
    void placeFighterAtEngagement();
    void sampleFighterControls(float deltaTime);
    void handleFighterCrash();
    void refreshFox2Prelaunch();
    void renderFighter();
    void emitFighterVisuals();
    float fox2SeekerCueRadiusPixels() const;
    glm::vec3 computeMissileLaunchDirection(Target *lockedTarget,
                                            const glm::vec3 &cameraForward,
                                            const glm::vec3 &stagedVelocity) const;
    glm::vec3 computeColdLaunchEjectDirection(const glm::vec3 &launchDirection) const;
    void resetMissileLaunchSequence();
    void updateMissileLaunchSequence(float deltaTime);
    void updatePreLaunchMissileAim(Target *trackedTarget);
    Target *findBestTarget();
    bool projectWorldPointToScreen(const glm::vec3 &worldPosition, ImVec2 &screenPosition, float *pixelDistanceFromCenter = nullptr) const;
    bool projectTargetToSeekerScreen(const Target *target, ImVec2 &screenPosition, float *pixelDistanceFromCenter = nullptr) const;
    Target *findSeekerCueTarget() const;
    Target *getTrackedMissileTarget() const;
    const char *getMissileSeekerStateLabel() const;
    const char *getMissileSeekerTrackLabel() const;
    void updatePreLaunchSeekerLock();
    void renderPreLaunchSeekerCue() const;
    void renderSeekerXrayOverlay() const;

    // Interception / detonation handling. Instead of resetting the instant a
    // missile detonates, we spawn the explosion and hold the scene for a few
    // seconds so the impact is visible (especially from onboard cameras),
    // then reset.
    void beginDetonationHold(const glm::vec3 &position);
    void finishDetonationHold();
    void frameDetonationCamera();

    // Visual effects
    void createExplosion(const glm::vec3 &position);
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
    DisplayMode m_displayMode = DisplayMode::Windowed;
    WindowedPlacement m_windowedPlacement;
    bool m_vsyncEnabled = true;
    bool m_windowRevealed = false;
    bool m_fullscreenKeyHeld = false;
    bool m_screenshotKeyHeld = false;

    // Screen flow and menu state
    Screen m_screen = Screen::Title;
    Overlay m_overlay = Overlay::None;
    Overlay m_overlayReturn = Overlay::None; // where Settings goes back to
    SettingsPage m_settingsPage = SettingsPage::Display;
    int m_menuSelection = 0;
    float m_screenTime = 0.0f;  // seconds since the current screen was entered
    float m_overlayTime = 0.0f; // seconds since the current overlay opened
    float m_fadeAlpha = 1.0f;   // full-screen fade; starts black for the intro
    float m_fadeDuration = 1.4f;
    bool m_pausedBeforeMenu = false;
    float m_titleOrbitAngle = 2.2f;
    glm::vec3 m_titleCameraForward{0.0f, 0.0f, 1.0f};
    float m_uiScale = 1.0f; // user interface size on top of the monitor DPI
    float m_pendingUiScalePercent = 100.0f;
    bool m_uiScaleSliderHeld = false;
    bool m_hudVisible = true;
    float m_menuAudioGain = 0.45f; // smoothed title-screen duck applied to the master volume
    HudTracker m_hud;
    int m_controlPanelTab = 0;
    float m_controlPanelAppear = 0.0f; // 0..1 slide-in, restarts whenever the panel reopens
    int m_controlPanelLastFrame = -2;
    static constexpr float kControlPanelWidth = 392.0f; // at 100% scale; the HUD keeps clear of it

    // Mouse camera control properties
    float m_lastMouseX;
    float m_lastMouseY;
    bool m_firstMouse;
    bool m_enableMouseCamera = false; // right mouse held: free-camera look or chase orbit
    bool m_cursorCaptured = false;
    glm::vec2 m_pendingMouseDelta{0.0f}; // pixels since the last frame, for mouse aim
    bool m_freeLookHeld = false;
    bool m_freeLookMouseHeld = false; // right mouse held in mouse aim
    CameraMode m_cameraMode = CameraMode::FREE;
    FreeCameraState m_freeCameraState;
    ChaseCamera m_chaseCamera;
    MouseAimCamera m_aimCamera;
    float m_lastFrameDeltaTime = 0.016f;

    // Mouse aim settings (Settings > Controls, autosaved).
    float m_mouseAimSensitivity = 0.06f; // degrees of aim per pixel of mouse travel
    bool m_invertMouseY = false;
    float m_cameraSmoothing = 12.0f;     // mouse-aim camera follow rate, 1/s

    // Fixed-step physics bookkeeping for render interpolation.
    float m_physicsAccumulator = 0.0f;
    float m_renderAlpha = 1.0f;

    // Simulation components
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<AudioSystem> m_audioSystem;
    std::unique_ptr<Missile> m_missile;
    std::unique_ptr<Fighter> m_fighter;
    std::vector<std::unique_ptr<Target>> m_targets;
    std::vector<std::unique_ptr<Flare>> m_flares;
    missilesim::sim::SimulationConfig m_simulationConfig;

    // Visual effects
    std::deque<ExplosionEffect> m_explosions;
    float m_explosionDuration = 1.0f; // Duration of explosion effect in seconds
    float m_explosionMaxSize = 10.0f; // Maximum size of explosion

    // Post-interception hold: keep the explosion on screen before resetting.
    bool m_detonationHoldActive = false;
    float m_detonationHoldTimer = 0.0f;
    float m_detonationHoldDuration = 3.0f; // wall-clock seconds to view the blast
    glm::vec3 m_detonationHoldPosition{0.0f};

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
    bool m_seekerXrayEnabled = false; // In-flight seeker x-ray overlay toggle

    // Simulation properties
    float m_timeStep = 0.01f; // Physics time step in seconds
    float m_simulationSpeed = 1.0f;
    bool m_isPaused = false;
    float m_audioVolume = 1.0f; // master output gain, 0..1

    // Ground properties
    bool m_groundEnabled = true;
    float m_groundRestitution = 0.5f;

    // UI properties
    bool m_showUI = false; // control panel (Tab)
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

    // SAM keeps the custom round. Fighter carries one catalog Fox 2 on the wingtip.
    PlayerRole m_playerRole = PlayerRole::Sam;
    std::string m_fox2Id = "custom";
    int m_fox2Rounds = 2;
    int m_railSign = 1;

    // Missile flight state
    bool m_missileInFlight = false;
    float m_missileFlightTime = 0.0f;
    float m_closestTargetDistance = 1000000.0f;
    MissileLaunchSequence m_launchSequence;

    // Random number generator
    std::mt19937 m_rng;

    // Autosaved user settings
    std::string m_settingsPath = "config/user_settings.ini";
    // Set once initialize() completes; a failed startup must not overwrite saved settings.
    bool m_initialized = false;
    bool m_settingsDirty = false;
    float m_settingsAutosaveDelay = 0.0f;
    std::string m_lastSettingsSnapshot;
    float m_savedGravity = 9.81f;
    float m_savedAirDensity = 1.225f;
    float m_savedCameraFOV = 50.0f;
    float m_savedCameraSpeed = 35.0f;
};
