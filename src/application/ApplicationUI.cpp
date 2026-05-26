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

        if (m_renderer->hasPBR())
        {
            ImGui::Separator();
            float exposure = m_renderer->getPBRExposure();
            if (ImGui::SliderFloat("Exposure", &exposure, 0.1f, 5.0f, "%.2f"))
            {
                m_renderer->setPBRExposure(exposure);
            }
            int bloomPasses = m_renderer->getPBRBloomPasses();
            if (ImGui::SliderInt("Bloom passes", &bloomPasses, 0, 10))
            {
                m_renderer->setPBRBloomPasses(bloomPasses);
            }
        }
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
}
