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

    Target *trackedTarget = getTrackedMissileTarget();
    const bool validTrackedTarget = (trackedTarget != nullptr);
    const bool missileWarning = validTrackedTarget && trackedTarget->isMissileWarningActive();
    const char *seekerTrack = getMissileSeekerTrackLabel();

    ImGui::SetNextWindowSize(ImVec2(320.0f, 270.0f), ImGuiCond_FirstUseEver);
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
        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Reset Missile", ImVec2(-1.0f, 0.0f)))
        {
            resetMissile();
        }
        if (ImGui::Button("Reset Target", ImVec2(-1.0f, 0.0f)))
        {
            resetTargets();
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
    const char *seekerState = getMissileSeekerStateLabel();
    const Atmosphere::State missileAtmosphere = m_physicsEngine->getAtmosphereState(missileAltitude);
    const float missileMach = (missileAtmosphere.speedOfSoundMetersPerSecond > 0.0f)
                                  ? (missileSpeed / missileAtmosphere.speedOfSoundMetersPerSecond)
                                  : 0.0f;

    Target *trackedTarget = getTrackedMissileTarget();

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

    const bool guidanceLocked = guidanceEnabled && trackedTarget != nullptr;
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
        ImGui::TextDisabled("Controls: RMB look, WASD move (Free cam), Space/Ctrl vertical, Enter pause, R seeker cue, F fire on lock, C frame free cam, Tab toggle UI");
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
            ImGui::TextDisabled("Controls: RMB look, WASD move (Free cam), Space/Ctrl vertical, Enter pause, R seeker cue, F fire on lock, C frame free cam, Tab toggle UI");
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
        const char *seekerState = getMissileSeekerStateLabel();
        Atmosphere::State missileAtmosphere = m_physicsEngine->getAtmosphereState(missileAltitude);
        const float missileMach = (missileAtmosphere.speedOfSoundMetersPerSecond > 0.0f)
                                      ? (missileSpeed / missileAtmosphere.speedOfSoundMetersPerSecond)
                                      : 0.0f;

        Target *trackedTarget = getTrackedMissileTarget();

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

        const bool guidanceLocked = guidanceEnabled && trackedTarget != nullptr;
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

        Target *trackedTarget = getTrackedMissileTarget();

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

        const bool guidanceLocked = guidanceEnabled && trackedTarget != nullptr;
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
            ImGui::TextUnformatted(m_missileInFlight ? "Live flight in progress" : "Press R to arm seeker, center target, then press F");
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