// Player weapon presentation: launch requests, the SAM's seeker cue, seeker
// labels, the in-flight seeker x-ray and the camera hold when the followed shot
// ends. Launch, staging and flight rules live in sim::World. The fighter's
// seeker circle and radar lock are part of the HUD (ApplicationHUD.cpp).
#include "Application.h"
#include "ApplicationDetail.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>

#include <glm/gtx/norm.hpp>

#include "objects/Missile.h"
#include "objects/Target.h"
#include "rendering/Renderer.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

using missilesim::application::detail::safeNormalize;

void Application::launchMissile()
{
    if (!m_world)
    {
        return;
    }

    const glm::vec3 fallbackAim = m_renderer ? safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f))
                                             : glm::vec3(0.0f, 0.0f, 1.0f);
    const missilesim::sim::LaunchResult result = m_world->launch(fallbackAim);
    if (!result.launched())
    {
        m_launchNotice = result.block;
        m_launchNoticeTimer = 2.5f;
        std::cout << "Launch blocked: " << missilesim::sim::launchBlockMessage(result.block) << std::endl;
        return;
    }

    m_launchNotice = missilesim::sim::LaunchBlock::None;
    m_launchNoticeTimer = 0.0f;
    // The camera and HUD follow the newest shot; a hold on an earlier
    // shot's end point gives way to it.
    m_followedShot = result.shot;
    m_detonationHoldActive = false;
    invalidateTrajectoryPreviewCache();
}

Target *Application::findBestTarget() const
{
    for (const auto &target : targets())
    {
        if (target->isActive())
        {
            return target.get();
        }
    }
    return nullptr;
}

bool Application::projectWorldPointToScreen(const glm::vec3 &worldPosition, ImVec2 &screenPosition, float *pixelDistanceFromCenter) const
{
    if (!m_renderer)
    {
        return false;
    }

    const int safeHeight = std::max(m_height, 1);
    const float aspectRatio = static_cast<float>(m_width) / static_cast<float>(safeHeight);
    const float tanHalfFovY = std::tan(glm::radians(m_renderer->getCameraFOV() * 0.5f));
    if (tanHalfFovY <= 0.0f)
    {
        return false;
    }

    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    const glm::vec3 cameraForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 cameraRight = safeNormalize(m_renderer->getCameraRight(), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 cameraUp = safeNormalize(m_renderer->getCameraUp(), glm::vec3(0.0f, 1.0f, 0.0f));

    const glm::vec3 toPoint = worldPosition - cameraPosition;
    const float forwardDepth = glm::dot(toPoint, cameraForward);
    if (forwardDepth <= 0.1f)
    {
        return false;
    }

    const float ndcX = glm::dot(toPoint, cameraRight) / (forwardDepth * tanHalfFovY * aspectRatio);
    const float ndcY = glm::dot(toPoint, cameraUp) / (forwardDepth * tanHalfFovY);
    if (std::abs(ndcX) > 1.0f || std::abs(ndcY) > 1.0f)
    {
        return false;
    }

    screenPosition.x = (ndcX + 1.0f) * 0.5f * m_width;
    screenPosition.y = (1.0f - ((ndcY + 1.0f) * 0.5f)) * safeHeight;

    if (pixelDistanceFromCenter != nullptr)
    {
        const float offsetX = screenPosition.x - (m_width * 0.5f);
        const float offsetY = screenPosition.y - (safeHeight * 0.5f);
        *pixelDistanceFromCenter = std::sqrt((offsetX * offsetX) + (offsetY * offsetY));
    }

    return true;
}

bool Application::projectTargetToSeekerScreen(const Target *target, ImVec2 &screenPosition, float *pixelDistanceFromCenter) const
{
    if (!target || !target->isActive())
    {
        return false;
    }

    const glm::vec3 aimPoint = target->getPosition() + glm::vec3(0.0f, target->getRadius() * 0.3f, 0.0f);
    return projectWorldPointToScreen(aimPoint, screenPosition, pixelDistanceFromCenter);
}

Target *Application::findSeekerCueTarget() const
{
    const Missile *round = m_world ? m_world->readyRound() : nullptr;
    if (!m_renderer || round == nullptr || !seekerUncaged() || !m_guidanceEnabled)
    {
        return nullptr;
    }

    // Keep the current lock while it stays on screen.
    Target *lockedTarget = round->getTargetObject();
    ImVec2 lockedScreenPosition(0.0f, 0.0f);
    if (lockedTarget != nullptr && projectTargetToSeekerScreen(lockedTarget, lockedScreenPosition, nullptr))
    {
        return lockedTarget;
    }

    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    Target *bestTarget = nullptr;
    float bestPixelDistance = std::numeric_limits<float>::max();
    float bestRange = std::numeric_limits<float>::max();

    for (const auto &target : targets())
    {
        if (!target->isActive())
        {
            continue;
        }

        ImVec2 screenPosition(0.0f, 0.0f);
        float pixelDistance = 0.0f;
        if (!projectTargetToSeekerScreen(target.get(), screenPosition, &pixelDistance))
        {
            continue;
        }

        if (pixelDistance > m_seekerCueRadiusPixels)
        {
            continue;
        }

        const float targetRange = glm::length(target->getPosition() - cameraPosition);
        if (pixelDistance < bestPixelDistance || (std::abs(pixelDistance - bestPixelDistance) < 0.5f && targetRange < bestRange))
        {
            bestTarget = target.get();
            bestPixelDistance = pixelDistance;
            bestRange = targetRange;
        }
    }

    return bestTarget;
}

Target *Application::getTrackedMissileTarget() const
{
    const Missile *missile = focusMissile();
    if (!missile || !missile->hasTarget())
    {
        return nullptr;
    }

    Target *trackedTarget = missile->getTargetObject();
    return (trackedTarget != nullptr && trackedTarget->isActive()) ? trackedTarget : nullptr;
}

const char *Application::getMissileSeekerStateLabel() const
{
    const Missile *missile = focusMissile();
    const bool inFlight = focusInFlight();
    if (missile && missile->isFox2())
    {
        if (!inFlight && !seekerUncaged())
        {
            return "Caged";
        }
        if (missile->isTrackingDecoy())
        {
            return "Tracking flare";
        }
        if (missile->hasFox2InfraredLock())
        {
            return "Infrared lock";
        }
        if (inFlight && missile->fox2OnTrackMemory())
        {
            return "Memory";
        }
        if (inFlight && (missile->fox2GuidanceExpired() || missile->fox2HadLock()))
        {
            return "Ballistic";
        }
        if (inFlight && missile->hasTarget())
        {
            return "Inertial";
        }
        if (missile->hasTarget())
        {
            return "Designated";
        }
        return "Searching";
    }

    if (!missile || !missile->isGuidanceEnabled())
    {
        const missilesim::sim::Shot *shot = followedShot();
        if (shot != nullptr && shot->coldLaunch.active)
        {
            return "Launch program";
        }
        return "Disabled";
    }

    if (!inFlight && !seekerUncaged())
    {
        return "Standby";
    }

    if (!missile->hasTarget())
    {
        return "Searching";
    }

    return missile->isTrackingDecoy() ? "Tracking flare" : "Tracking airframe";
}

const char *Application::getMissileSeekerTrackLabel() const
{
    const Missile *missile = focusMissile();
    if (!missile || !missile->isGuidanceEnabled())
    {
        return "DISABLED";
    }

    if (!focusInFlight() && !seekerUncaged())
    {
        return "STBY";
    }

    if (!missile->hasTarget())
    {
        return "SEARCH";
    }

    return missile->isTrackingDecoy() ? "FLARE" : "AIRFRAME";
}

void Application::updatePreLaunchSeekerLock()
{
    // A catalog Fox 2 designates with its own seeker on every simulation
    // step; only the SAM launcher uses the screen cue.
    if (!m_world || m_world->role() != missilesim::sim::PlayerRole::Sam || m_world->readyRound() == nullptr)
    {
        return;
    }

    const glm::vec3 cameraForward = m_renderer ? safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f))
                                               : glm::vec3(0.0f, 0.0f, 1.0f);
    if (!seekerUncaged() || !m_guidanceEnabled)
    {
        m_world->designate(missilesim::sim::kNoEntity);
        m_world->aimReadyRound(cameraForward);
        return;
    }

    Target *cueTarget = findSeekerCueTarget();
    m_world->designate(cueTarget != nullptr ? cueTarget->getEntityId() : missilesim::sim::kNoEntity);
    m_world->aimReadyRound(cameraForward);
}

void Application::renderPreLaunchSeekerCue() const
{
    // The SAM aims with the middle of the view. The fighter's seeker looks
    // where its head points, and the HUD draws that circle.
    const Missile *round = m_world ? m_world->readyRound() : nullptr;
    if (m_playerRole != PlayerRole::Sam || !seekerUncaged() || !m_renderer || round == nullptr ||
        ImGui::GetCurrentContext() == nullptr)
    {
        return;
    }

    ImVec2 cueCenter(m_width * 0.5f, m_height * 0.5f);
    bool hasLock = false;
    Target *designated = round->getTargetObject();
    if (designated != nullptr && designated->isActive())
    {
        hasLock = projectTargetToSeekerScreen(designated, cueCenter, nullptr);
    }

    namespace ui = missilesim::ui;
    const ImU32 ringColor = hasLock ? ui::toU32(ui::color::danger) : ui::toU32(ui::color::text, 0.85f);
    ImDrawList *drawList = ImGui::GetBackgroundDrawList();
    const float tick = ui::px(6.0f);
    const float cueRadius = m_seekerCueRadiusPixels;
    drawList->AddCircle(cueCenter, cueRadius, ringColor, 64, ui::px(1.8f));
    drawList->AddLine(ImVec2(cueCenter.x - tick, cueCenter.y), ImVec2(cueCenter.x + tick, cueCenter.y), ringColor, ui::px(1.2f));
    drawList->AddLine(ImVec2(cueCenter.x, cueCenter.y - tick), ImVec2(cueCenter.x, cueCenter.y + tick), ringColor, ui::px(1.2f));
    const char *label = hasLock ? "SEEKER LOCK" : "SEEKER SEARCH";
    const float labelWidth = ui::measureTracked(ui::fonts().display, ui::px(12.5f), label, 0.16f).x;
    ui::drawTracked(drawList, ui::fonts().display, ui::px(12.5f),
                    ImVec2(cueCenter.x - labelWidth * 0.5f, cueCenter.y + cueRadius + ui::px(8.0f)),
                    ringColor, label, 0.16f);
}

void Application::renderSeekerXrayOverlay() const
{
    const missilesim::sim::Shot *shot = followedShot();
    if (!m_seekerXrayEnabled || !m_renderer || shot == nullptr || ImGui::GetCurrentContext() == nullptr)
    {
        return;
    }

    // The x-ray is the autonomous "bulldog" view: it only has meaning once the
    // seeker is actually powered and guiding.
    const Missile &missile = *shot->missile;
    if (!missile.isGuidanceEnabled())
    {
        return;
    }

    // Corner brackets are drawn on the background draw list after the scene,
    // so they sit on top of it regardless of depth - that "through walls /
    // terrain" behaviour is the whole point of the x-ray.
    namespace ui = missilesim::ui;
    const auto drawBracket = [](ImDrawList *drawList, const ImVec2 &center, float half, ImU32 color, float thickness) {
        ui::drawCornerBrackets(drawList, center, half, color, thickness, 0.45f);
    };
    ImDrawList *drawList = ImGui::GetBackgroundDrawList();

    // The bold marker tracks the missile's actual aimpoint - the airframe while
    // locked, or the flare the moment the seeker is decoyed. Once the seeker
    // is spoofed, the real jet drops back to a faint cue so you can see the
    // seeker chasing the decoy past the live target.
    const bool hasAimpoint = missile.hasTarget();
    const bool trackingDecoy = hasAimpoint && missile.isTrackingDecoy();
    const Target *boldAirframe = (hasAimpoint && !trackingDecoy) ? missile.getTargetObject() : nullptr;

    const ImU32 ambientColor = ui::toU32(ui::color::info, 0.55f);
    for (const auto &target : targets())
    {
        if (!target->isActive() || target.get() == boldAirframe)
        {
            continue;
        }

        ImVec2 screenPosition(0.0f, 0.0f);
        if (projectTargetToSeekerScreen(target.get(), screenPosition, nullptr))
        {
            drawBracket(drawList, screenPosition, ui::px(12.0f), ambientColor, ui::px(1.4f));
        }
    }

    if (!hasAimpoint)
    {
        return;
    }

    const glm::vec3 aimWorld = missile.getTargetPosition();
    ImVec2 lockScreen(0.0f, 0.0f);
    if (!projectWorldPointToScreen(aimWorld, lockScreen, nullptr))
    {
        return;
    }

    const ImVec4 &lockTone = trackingDecoy ? ui::color::accent : ui::color::danger;
    const ImU32 lockColor = ui::toU32(lockTone);
    const ImU32 leadColor = ui::toU32(lockTone, 0.4f);

    const float lockHalf = ui::px(24.0f);
    drawBracket(drawList, lockScreen, lockHalf, lockColor, ui::px(2.2f));
    drawList->AddCircleFilled(lockScreen, ui::px(2.4f), lockColor, 8);

    const ImVec2 boresight(m_width * 0.5f, m_height * 0.5f);
    drawList->AddLine(boresight, lockScreen, leadColor, ui::px(1.4f));

    const float range = glm::distance(missile.getPosition(), aimWorld);
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.0f m", range);
    const float textX = lockScreen.x + lockHalf + ui::px(8.0f);
    ui::drawTracked(drawList, ui::fonts().display, ui::px(13.0f), ImVec2(textX, lockScreen.y - ui::px(15.0f)), lockColor,
                    trackingDecoy ? "FLARE" : "LOCK", 0.14f);
    drawList->AddText(ui::fonts().mono, ui::px(12.5f), ImVec2(textX, lockScreen.y + ui::px(1.0f)), lockColor, buffer);
}

void Application::beginDetonationHold(const glm::vec3 &position)
{
    invalidateTrajectoryPreviewCache();
    m_detonationHoldActive = true;
    m_detonationHoldTimer = 0.0f;
    m_detonationHoldPosition = position;
}

void Application::finishDetonationHold()
{
    m_detonationHoldActive = false;
    m_detonationHoldTimer = 0.0f;

    // Pick up the newest shot still in the air, if any.
    m_followedShot = missilesim::sim::kNoEntity;
    if (m_world && !m_world->shots().empty())
    {
        m_followedShot = m_world->shots().back()->id;
    }
}
