// In-engagement HUD: mission status, camera selector, target roster, flight
// data strip, target markers with off-screen arrows, event feed, RWR scope and
// the engagement result card. Everything is drawn on the background draw list,
// so the HUD never captures the mouse and always sits beneath panels and menus.
#include "Application.h"
#include "ApplicationDetail.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <string>

#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/Atmosphere.h"
#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

using missilesim::application::detail::safeNormalize;
namespace ui = missilesim::ui;

namespace
{
    constexpr float kToastLifetime = 3.0f;
    constexpr size_t kMaxToasts = 4;
    constexpr float kResultDuration = 4.8f;
    constexpr float kHintsVisibleSeconds = 12.0f;
    constexpr float kGravity = 9.80665f;

    ImVec4 mix(const ImVec4 &a, const ImVec4 &b, float t)
    {
        return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
    }

    float fadeWindow(float age, float lifetime, float fadeIn, float fadeOut)
    {
        const float in = fadeIn > 0.0f ? std::clamp(age / fadeIn, 0.0f, 1.0f) : 1.0f;
        const float out = fadeOut > 0.0f ? std::clamp((lifetime - age) / fadeOut, 0.0f, 1.0f) : 1.0f;
        return std::min(in, out);
    }

    // Translucent HUD panel: glass fill plus a hairline edge.
    void drawGlass(ImDrawList *drawList, ImVec2 min, ImVec2 max, float alpha, float rounding)
    {
        drawList->AddRectFilled(min, max, ui::toU32(ImVec4(0.035f, 0.043f, 0.059f, 0.66f), alpha), rounding);
        drawList->AddRect(min, max, ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.07f), alpha), rounding);
    }

    float textWidth(ImFont *font, float size, const char *text)
    {
        return font->CalcTextSizeA(size, FLT_MAX, 0.0f, text).x;
    }

    std::string formatRange(float metres)
    {
        char buffer[32];
        if (metres < 1000.0f)
        {
            std::snprintf(buffer, sizeof(buffer), "%.0f m", metres);
        }
        else
        {
            std::snprintf(buffer, sizeof(buffer), "%.2f km", metres / 1000.0f);
        }
        return buffer;
    }

    const char *aiStateLabel(TargetAIState state)
    {
        switch (state)
        {
        case TargetAIState::PATROL:
            return "PATROL";
        case TargetAIState::REPOSITION:
            return "REPOSITION";
        case TargetAIState::DEFENSIVE:
            return "DEFENSIVE";
        case TargetAIState::RECOVERING:
            return "RECOVER";
        default:
            return "UNKNOWN";
        }
    }

    void drawBrackets(ImDrawList *drawList, ImVec2 centre, float half, ImU32 colour, float thickness)
    {
        const float leg = half * 0.42f;
        const float l = centre.x - half, r = centre.x + half, t = centre.y - half, b = centre.y + half;
        drawList->AddLine(ImVec2(l, t), ImVec2(l + leg, t), colour, thickness);
        drawList->AddLine(ImVec2(l, t), ImVec2(l, t + leg), colour, thickness);
        drawList->AddLine(ImVec2(r, t), ImVec2(r - leg, t), colour, thickness);
        drawList->AddLine(ImVec2(r, t), ImVec2(r, t + leg), colour, thickness);
        drawList->AddLine(ImVec2(l, b), ImVec2(l + leg, b), colour, thickness);
        drawList->AddLine(ImVec2(l, b), ImVec2(l, b - leg), colour, thickness);
        drawList->AddLine(ImVec2(r, b), ImVec2(r - leg, b), colour, thickness);
        drawList->AddLine(ImVec2(r, b), ImVec2(r, b - leg), colour, thickness);
    }
}

const char *Application::missionStateLabel() const
{
    if (m_isPaused)
    {
        return "PAUSED";
    }
    if (m_detonationHoldActive)
    {
        return "DETONATION";
    }
    if (m_launchSequence.active && !m_launchSequence.motorIgnited)
    {
        return "EJECT";
    }
    if (m_launchSequence.active && !m_launchSequence.guidanceArmed)
    {
        return "IGNITION";
    }
    if (!m_missileInFlight || !m_missile)
    {
        return "READY";
    }
    const bool locked = m_missile->isGuidanceEnabled() && getTrackedMissileTarget() != nullptr;
    const bool thrust = m_missile->isThrustEnabled();
    if (locked && thrust)
    {
        return "INTERCEPT";
    }
    if (locked)
    {
        return "GLIDE";
    }
    return thrust ? "BOOST" : "BALLISTIC";
}

float Application::hudRightInset() const
{
    return m_showUI ? ui::px(kControlPanelWidth) + ui::px(16.0f) : 0.0f;
}

void Application::pushHudToast(std::string text, HudTone tone)
{
    m_hud.toasts.push_front(HudToast{std::move(text), tone, 0.0f});
    while (m_hud.toasts.size() > kMaxToasts)
    {
        m_hud.toasts.pop_back();
    }
}

void Application::updateHudTracker(float deltaTime)
{
    HudTracker &hud = m_hud;
    deltaTime = std::max(deltaTime, 0.0f);
    if (!m_isPaused)
    {
        hud.engagementTime += deltaTime;
    }

    for (HudToast &toast : hud.toasts)
    {
        toast.age += deltaTime;
    }
    while (!hud.toasts.empty() && hud.toasts.back().age > kToastLifetime)
    {
        hud.toasts.pop_back();
    }
    if (hud.resultVisible)
    {
        hud.resultAge += deltaTime;
        hud.resultVisible = hud.resultAge < kResultDuration;
    }

    auto targetIndex = [this](const Target *target) -> int
    {
        for (size_t i = 0; i < m_targets.size(); ++i)
        {
            if (m_targets[i].get() == target)
            {
                return static_cast<int>(i) + 1;
            }
        }
        return 0;
    };

    // resetTargets() rebuilds the roster; resync silently rather than report it.
    bool rosterChanged = !hud.initialized || hud.targets.size() != m_targets.size();
    for (size_t i = 0; !rosterChanged && i < m_targets.size(); ++i)
    {
        rosterChanged = hud.targets[i] != m_targets[i].get();
    }
    if (rosterChanged)
    {
        hud.targets.assign(m_targets.size(), nullptr);
        hud.targetActive.assign(m_targets.size(), false);
        hud.targetFlares.assign(m_targets.size(), 0);
        hud.flareToastCooldown.assign(m_targets.size(), 0.0f);
        for (size_t i = 0; i < m_targets.size(); ++i)
        {
            hud.targets[i] = m_targets[i].get();
            hud.targetActive[i] = m_targets[i] && m_targets[i]->isActive();
            hud.targetFlares[i] = m_targets[i] ? m_targets[i]->getRemainingFlares() : 0;
        }
        hud.initialized = true;
    }
    else
    {
        for (size_t i = 0; i < m_targets.size(); ++i)
        {
            const Target *target = m_targets[i].get();
            const bool active = target != nullptr && target->isActive();
            const int flares = target != nullptr ? target->getRemainingFlares() : 0;
            hud.flareToastCooldown[i] -= deltaTime;
            if (active && flares < hud.targetFlares[i] && hud.flareToastCooldown[i] <= 0.0f)
            {
                pushHudToast("T" + std::to_string(i + 1) + "  Flares", HudTone::Accent);
                hud.flareToastCooldown[i] = 2.5f;
            }
            hud.targetActive[i] = active;
            hud.targetFlares[i] = flares;
        }
    }

    const bool launchActive = m_launchSequence.active || m_missileInFlight;
    if (launchActive && !hud.launchWasActive)
    {
        pushHudToast("Missile away", HudTone::Accent);
        hud.hitsAtLaunch = m_targetHits;
        hud.peakSpeed = 0.0f;
        hud.peakMach = 0.0f;
        hud.lastLock = nullptr;
        hud.wasDecoyed = false;
        hud.resultVisible = false;
    }
    hud.launchWasActive = launchActive;

    if (m_missileInFlight && m_missile && m_physicsEngine)
    {
        const float speed = glm::length(m_missile->getVelocity());
        const Atmosphere::State air = m_physicsEngine->getAtmosphereState(std::max(m_missile->getPosition().y, 0.0f));
        hud.peakSpeed = std::max(hud.peakSpeed, speed);
        if (air.speedOfSoundMetersPerSecond > 0.0f)
        {
            hud.peakMach = std::max(hud.peakMach, speed / air.speedOfSoundMetersPerSecond);
        }

        const Target *lock = getTrackedMissileTarget();
        const bool decoyed = m_missile->hasTarget() && m_missile->isTrackingDecoy();
        if (lock != nullptr && lock != hud.lastLock && !decoyed)
        {
            pushHudToast("Seeker lock  T" + std::to_string(targetIndex(lock)), HudTone::Accent);
        }
        if (decoyed && !hud.wasDecoyed)
        {
            pushHudToast("Seeker decoyed", HudTone::Danger);
        }
        hud.lastLock = lock;
        hud.wasDecoyed = decoyed;
    }

    if (m_detonationHoldActive && !hud.holdWasActive)
    {
        hud.resultVisible = true;
        hud.resultAge = 0.0f;
        hud.resultHit = m_targetHits > hud.hitsAtLaunch;
        hud.resultFlightTime = m_missileFlightTime;
        hud.resultClosestPass = m_closestTargetDistance;
        hud.resultPeakMach = hud.peakMach;
    }
    hud.holdWasActive = m_detonationHoldActive;
}

void Application::renderHud()
{
    if (!m_renderer || !m_missile)
    {
        return;
    }

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    const float width = viewport->Size.x;
    const float height = viewport->Size.y;
    const ImVec2 origin = viewport->Pos;
    ImDrawList *drawList = ImGui::GetBackgroundDrawList();
    const float margin = ui::px(26.0f);
    const float time = static_cast<float>(ImGui::GetTime());
    const ui::Fonts &font = ui::fonts();

    auto toneColour = [](HudTone tone)
    {
        switch (tone)
        {
        case HudTone::Accent:
            return ui::color::accent;
        case HudTone::Positive:
            return ui::color::positive;
        case HudTone::Danger:
            return ui::color::danger;
        case HudTone::Neutral:
            break;
        }
        return ui::color::text;
    };

    Target *trackedTarget = getTrackedMissileTarget();
    auto targetIndex = [this](const Target *target) -> int
    {
        for (size_t i = 0; i < m_targets.size(); ++i)
        {
            if (m_targets[i].get() == target)
            {
                return static_cast<int>(i) + 1;
            }
        }
        return 0;
    };

    // ---- Projection (keeps off-screen and behind-camera directions) --------
    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    const glm::vec3 cameraForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 cameraRight = safeNormalize(m_renderer->getCameraRight(), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 cameraUp = safeNormalize(m_renderer->getCameraUp(), glm::vec3(0.0f, 1.0f, 0.0f));
    const float tanHalfFov = std::tan(glm::radians(m_renderer->getCameraFOV() * 0.5f));
    const float aspect = width / std::max(height, 1.0f);
    struct Projected
    {
        ImVec2 screen;
        float ndcX = 0.0f;
        float ndcY = 0.0f;
        bool onScreen = false;
    };
    auto project = [&](const glm::vec3 &world)
    {
        Projected p;
        const glm::vec3 toPoint = world - cameraPosition;
        const float depth = glm::dot(toPoint, cameraForward);
        const float safeDepth = std::max(std::abs(depth), 0.01f);
        p.ndcX = glm::dot(toPoint, cameraRight) / (safeDepth * tanHalfFov * aspect);
        p.ndcY = glm::dot(toPoint, cameraUp) / (safeDepth * tanHalfFov);
        p.onScreen = depth > 0.1f && std::abs(p.ndcX) <= 1.0f && std::abs(p.ndcY) <= 1.0f;
        p.screen = ImVec2(origin.x + (p.ndcX + 1.0f) * 0.5f * width, origin.y + (1.0f - (p.ndcY + 1.0f) * 0.5f) * height);
        if (depth <= 0.1f)
        {
            // Behind the camera: push the direction outward so the edge arrow
            // points the right way; dead astern points down.
            const bool deadAstern = std::abs(p.ndcX) < 0.001f && std::abs(p.ndcY) < 0.001f;
            p.ndcX = deadAstern ? 0.0f : p.ndcX * 1000.0f;
            p.ndcY = deadAstern ? -1000.0f : p.ndcY * 1000.0f;
        }
        return p;
    };

    // ---- Target markers ------------------------------------------------------
    if (m_showTargetInfo)
    {
        const Target *chaseSubject = (m_cameraMode == CameraMode::FIGHTER_JET)
                                         ? (trackedTarget != nullptr ? trackedTarget : findBestTarget())
                                         : nullptr;
        const float edgeInset = ui::px(44.0f);
        const ImVec2 centre(origin.x + width * 0.5f, origin.y + height * 0.5f);

        for (size_t i = 0; i < m_targets.size(); ++i)
        {
            const Target *target = m_targets[i].get();
            if (target == nullptr || !target->isActive() || target == chaseSubject)
            {
                continue;
            }
            const bool isLocked = target == trackedTarget;
            const bool warning = target->isMissileWarningActive();
            const ImVec4 colour = isLocked ? ui::color::accent : ui::withAlpha(ui::color::text, 0.88f);
            const float range = glm::distance(target->getPosition(), m_missile->getPosition());
            const std::string rangeText = formatRange(range);
            char idText[8];
            std::snprintf(idText, sizeof(idText), "T%zu", i + 1);

            const Projected p = project(target->getPosition());
            if (p.onScreen)
            {
                const float half = ui::px(13.0f) + ui::px(12.0f) * std::clamp(900.0f / std::max(range, 1.0f), 0.0f, 1.0f);
                drawBrackets(drawList, p.screen, half, ui::toU32(colour), isLocked ? ui::px(2.0f) : ui::px(1.4f));
                if (isLocked)
                {
                    drawList->AddCircleFilled(p.screen, ui::px(2.2f), ui::toU32(colour), 10);
                }

                float labelX = p.screen.x - half;
                const float labelY = p.screen.y + half + ui::px(6.0f);
                ui::drawTracked(drawList, font.display, ui::px(14.5f), ImVec2(labelX, labelY), ui::toU32(colour), idText, 0.06f);
                labelX += ui::measureTracked(font.display, ui::px(14.5f), idText, 0.06f).x + ui::px(8.0f);
                drawList->AddText(font.mono, ui::px(12.5f), ImVec2(labelX, labelY + ui::px(1.5f)),
                                  ui::toU32(ui::color::textMuted), rangeText.c_str());
                if (isLocked || warning)
                {
                    const char *tag = warning ? "MAWS" : "LOCK";
                    const ImVec4 tagColour = warning ? ui::color::danger : ui::color::accent;
                    ui::drawTracked(drawList, font.display, ui::px(12.5f), ImVec2(p.screen.x - half, labelY + ui::px(18.0f)),
                                    ui::toU32(tagColour), tag, 0.14f);
                }
            }
            else
            {
                // Edge arrow pointing at the off-screen target.
                float dx = p.ndcX;
                float dy = -p.ndcY;
                const float length = std::sqrt(dx * dx + dy * dy);
                if (length < 0.0001f)
                {
                    continue;
                }
                dx /= length;
                dy /= length;
                const float reachX = width * 0.5f - edgeInset;
                const float reachY = height * 0.5f - edgeInset;
                const float t = std::min(reachX / std::max(std::abs(dx), 0.0001f), reachY / std::max(std::abs(dy), 0.0001f));
                const ImVec2 point(centre.x + dx * t, centre.y + dy * t);
                const ImVec2 perp(-dy, dx);
                const float tip = ui::px(9.0f), back = ui::px(5.0f), wing = ui::px(7.0f);
                drawList->AddTriangleFilled(ImVec2(point.x + dx * tip, point.y + dy * tip),
                                            ImVec2(point.x - dx * back + perp.x * wing, point.y - dy * back + perp.y * wing),
                                            ImVec2(point.x - dx * back - perp.x * wing, point.y - dy * back - perp.y * wing),
                                            ui::toU32(colour));
                const std::string label = std::string(idText) + "  " + rangeText;
                const float labelWidth = textWidth(font.mono, ui::px(12.5f), label.c_str());
                const ImVec2 labelCentre(point.x - dx * ui::px(30.0f), point.y - dy * ui::px(24.0f));
                drawList->AddText(font.mono, ui::px(12.5f), ImVec2(labelCentre.x - labelWidth * 0.5f, labelCentre.y - ui::px(7.0f)),
                                  ui::toU32(colour), label.c_str());
            }
        }

        // Missile marker (when not riding it).
        if (m_missileInFlight && m_cameraMode != CameraMode::MISSILE)
        {
            const Projected p = project(m_missile->getPosition());
            if (p.onScreen)
            {
                const float s = ui::px(6.0f);
                const ImU32 colour = ui::toU32(ui::color::info);
                drawList->AddQuad(ImVec2(p.screen.x, p.screen.y - s), ImVec2(p.screen.x + s, p.screen.y),
                                  ImVec2(p.screen.x, p.screen.y + s), ImVec2(p.screen.x - s, p.screen.y), colour, ui::px(1.6f));
                drawList->AddText(font.monoMedium, ui::px(11.5f), ImVec2(p.screen.x + s + ui::px(5.0f), p.screen.y - ui::px(6.5f)),
                                  colour, "MSL");
            }
        }
    }

    // ---- Mission status (top left) ---------------------------------------------
    {
        const char *state = missionStateLabel();
        ImVec4 stateColour = ui::color::text;
        bool pulsing = false;
        if (std::strcmp(state, "PAUSED") == 0)
        {
            stateColour = ui::color::info;
        }
        else if (std::strcmp(state, "READY") == 0)
        {
            stateColour = ui::color::positive;
        }
        else if (std::strcmp(state, "BALLISTIC") == 0)
        {
            stateColour = ui::color::textMuted;
        }
        else if (std::strcmp(state, "GLIDE") == 0)
        {
            stateColour = ui::color::info;
            pulsing = true;
        }
        else
        {
            stateColour = std::strcmp(state, "DETONATION") == 0 ? ui::color::danger : ui::color::accent;
            pulsing = true;
        }

        const float x = origin.x + margin;
        const float y = origin.y + margin;
        const float dotAlpha = pulsing ? 0.55f + 0.45f * (0.5f + 0.5f * std::sin(time * 6.0f)) : 1.0f;
        drawList->AddCircleFilled(ImVec2(x + ui::px(5.0f), y + ui::px(13.0f)), ui::px(4.5f), ui::toU32(stateColour, dotAlpha), 16);
        ui::drawTracked(drawList, font.display, ui::px(25.0f), ImVec2(x + ui::px(18.0f), y), ui::toU32(stateColour), state, 0.08f);

        std::string detail = std::string("Seeker ") + getMissileSeekerStateLabel();
        std::transform(detail.begin() + 7, detail.end(), detail.begin() + 7,
                       [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });
        if (trackedTarget != nullptr && (m_missileInFlight || m_seekerCueEnabled))
        {
            detail += "  \xC2\xB7  T" + std::to_string(targetIndex(trackedTarget));
        }
        if (m_missileInFlight)
        {
            char flight[32];
            std::snprintf(flight, sizeof(flight), "  \xC2\xB7  %.1f s", m_missileFlightTime);
            detail += flight;
        }
        drawList->AddText(font.body, ui::px(14.5f), ImVec2(x + ui::px(18.0f), y + ui::px(31.0f)),
                          ui::toU32(ui::color::textMuted), detail.c_str());
    }

    // ---- Camera selector (top centre) --------------------------------------------
    float belowSelectorY = origin.y + margin;
    {
        const char *const labels[] = {"FREE", "MISSILE", "FIGHTER"};
        const int active = m_cameraMode == CameraMode::FREE ? 0 : (m_cameraMode == CameraMode::MISSILE ? 1 : 2);
        const float size = ui::px(13.5f);
        const float tracking = 0.16f;
        const float gap = ui::px(20.0f);
        float labelsWidth = 0.0f;
        float widths[3];
        for (int i = 0; i < 3; ++i)
        {
            widths[i] = ui::measureTracked(font.display, size, labels[i], tracking).x;
            labelsWidth += widths[i];
        }
        labelsWidth += gap * 2.0f;
        const float capWidth = ui::px(22.0f);
        const float pad = ui::px(14.0f);
        const float pillHeight = ui::px(36.0f);
        const float pillWidth = pad + capWidth + ui::px(14.0f) + labelsWidth + pad;
        const float usableWidth = width - hudRightInset();
        const ImVec2 pillMin(std::round(origin.x + (usableWidth - pillWidth) * 0.5f), origin.y + margin - ui::px(4.0f));
        const ImVec2 pillMax(pillMin.x + pillWidth, pillMin.y + pillHeight);
        drawGlass(drawList, pillMin, pillMax, 1.0f, pillHeight * 0.5f);
        ui::drawKeycap(drawList, ImVec2(pillMin.x + pad, pillMin.y + (pillHeight - ui::px(22.0f)) * 0.5f), "V");

        float x = pillMin.x + pad + capWidth + ui::px(14.0f);
        const float textY = pillMin.y + (pillHeight - size) * 0.5f - ui::px(1.0f);
        for (int i = 0; i < 3; ++i)
        {
            const bool selected = i == active;
            ui::drawTracked(drawList, font.display, size, ImVec2(x, textY),
                            ui::toU32(selected ? ui::color::text : ui::color::textFaint), labels[i], tracking);
            if (selected)
            {
                drawList->AddRectFilled(ImVec2(x, pillMax.y - ui::px(7.0f)), ImVec2(x + widths[i] - size * tracking, pillMax.y - ui::px(5.0f)),
                                        ui::toU32(ui::color::accent));
            }
            x += widths[i] + gap;
        }
        belowSelectorY = pillMax.y + ui::px(12.0f);

        // Paused chip.
        if (m_isPaused && m_overlay == Overlay::None)
        {
            const char *label = "PAUSED";
            const char *hint = "Resume";
            const float labelWidth = ui::measureTracked(font.display, ui::px(15.0f), label, 0.14f).x;
            const float hintWidth = textWidth(font.body, ui::px(14.0f), hint);
            const float enterCap = textWidth(font.monoMedium, ui::px(12.5f), "Enter") + ui::px(14.0f);
            const float chipWidth = ui::px(18.0f) + labelWidth + ui::px(18.0f) + enterCap + ui::px(8.0f) + hintWidth + ui::px(18.0f);
            const ImVec2 chipMin(std::round(origin.x + (usableWidth - chipWidth) * 0.5f), belowSelectorY);
            const ImVec2 chipMax(chipMin.x + chipWidth, chipMin.y + ui::px(36.0f));
            drawGlass(drawList, chipMin, chipMax, 1.0f, ui::px(18.0f));
            float cx = chipMin.x + ui::px(18.0f);
            ui::drawTracked(drawList, font.display, ui::px(15.0f), ImVec2(cx, chipMin.y + ui::px(9.0f)), ui::toU32(ui::color::info), label, 0.14f);
            cx += labelWidth + ui::px(18.0f);
            cx += ui::drawKeycap(drawList, ImVec2(cx, chipMin.y + ui::px(7.0f)), "Enter") + ui::px(8.0f);
            drawList->AddText(font.body, ui::px(14.0f), ImVec2(cx, chipMin.y + ui::px(10.0f)), ui::toU32(ui::color::textMuted), hint);
            belowSelectorY = chipMax.y + ui::px(10.0f);
        }
    }

    // ---- Event feed (under the selector) ----------------------------------------
    {
        float y = belowSelectorY;
        const float usableWidth = width - hudRightInset();
        for (const HudToast &toast : m_hud.toasts)
        {
            const float alpha = fadeWindow(toast.age, kToastLifetime, 0.16f, 0.5f);
            const float slide = (1.0f - std::clamp(toast.age / 0.16f, 0.0f, 1.0f)) * ui::px(-8.0f);
            std::string upper = toast.text;
            std::transform(upper.begin(), upper.end(), upper.begin(),
                           [](unsigned char c)
                           { return static_cast<char>(std::toupper(c)); });
            const float size = ui::px(16.0f);
            const float labelWidth = ui::measureTracked(font.display, size, upper.c_str(), 0.12f).x;
            const float pillWidth = labelWidth + ui::px(44.0f);
            const float pillHeight = ui::px(34.0f);
            const ImVec2 min(std::round(origin.x + (usableWidth - pillWidth) * 0.5f), y + slide);
            const ImVec2 max(min.x + pillWidth, min.y + pillHeight);
            const ImVec4 tone = toneColour(toast.tone);
            drawGlass(drawList, min, max, alpha, ui::px(6.0f));
            drawList->AddRectFilled(ImVec2(min.x + ui::px(10.0f), min.y + ui::px(10.0f)), ImVec2(min.x + ui::px(13.0f), max.y - ui::px(10.0f)),
                                    ui::toU32(tone, alpha), ui::px(1.5f));
            ui::drawTracked(drawList, font.display, size, ImVec2(min.x + ui::px(26.0f), min.y + (pillHeight - size) * 0.5f),
                            ui::toU32(mix(tone, ui::color::text, 0.35f), alpha), upper.c_str(), 0.12f);
            y += (pillHeight + ui::px(8.0f)) * alpha;
        }
    }

    // ---- Target roster (top right) -----------------------------------------------
    if (!m_targets.empty())
    {
        const size_t rows = std::min<size_t>(m_targets.size(), 8);
        const float panelWidth = ui::px(296.0f);
        const float rowHeight = ui::px(26.0f);
        const float right = origin.x + width - margin - hudRightInset();
        const ImVec2 min(right - panelWidth, origin.y + margin - ui::px(4.0f));
        const ImVec2 max(right, min.y + ui::px(42.0f) + rowHeight * static_cast<float>(rows) + ui::px(8.0f));
        drawGlass(drawList, min, max, 1.0f, ui::px(8.0f));

        int activeCount = 0;
        for (const auto &target : m_targets)
        {
            activeCount += (target && target->isActive()) ? 1 : 0;
        }
        ui::drawTracked(drawList, font.display, ui::px(13.0f), ImVec2(min.x + ui::px(16.0f), min.y + ui::px(14.0f)),
                        ui::toU32(ui::color::textMuted), "TARGETS", 0.16f);
        char count[32];
        std::snprintf(count, sizeof(count), "%d / %zu", activeCount, m_targets.size());
        drawList->AddText(font.monoMedium, ui::px(13.0f), ImVec2(max.x - ui::px(16.0f) - textWidth(font.monoMedium, ui::px(13.0f), count), min.y + ui::px(14.0f)),
                          ui::toU32(ui::color::text), count);

        for (size_t i = 0; i < rows; ++i)
        {
            const Target *target = m_targets[i].get();
            if (target == nullptr)
            {
                continue;
            }
            const float rowY = min.y + ui::px(42.0f) + rowHeight * static_cast<float>(i);
            const float textY = rowY + (rowHeight - ui::px(15.0f)) * 0.5f;
            const bool active = target->isActive();
            const bool isLocked = target == trackedTarget;
            const float rowAlpha = active ? 1.0f : 0.4f;
            if (isLocked)
            {
                drawList->AddRectFilled(ImVec2(min.x + ui::px(6.0f), rowY + ui::px(5.0f)), ImVec2(min.x + ui::px(8.5f), rowY + rowHeight - ui::px(5.0f)),
                                        ui::toU32(ui::color::accent), ui::px(1.0f));
            }
            char idText[8];
            std::snprintf(idText, sizeof(idText), "T%zu", i + 1);
            ui::drawTracked(drawList, font.display, ui::px(15.0f), ImVec2(min.x + ui::px(16.0f), textY),
                            ui::toU32(isLocked ? ui::color::accent : ui::color::text, rowAlpha), idText, 0.06f);

            const TargetAIState aiState = target->getAIState();
            const ImVec4 stateColour = !active ? ui::color::textFaint
                                               : (aiState == TargetAIState::DEFENSIVE ? ui::color::danger
                                                                                     : (aiState == TargetAIState::RECOVERING ? ui::color::accent : ui::color::textMuted));
            ui::drawTracked(drawList, font.displayMedium, ui::px(13.0f), ImVec2(min.x + ui::px(54.0f), textY + ui::px(1.0f)),
                            ui::toU32(stateColour), active ? aiStateLabel(aiState) : "DOWN", 0.1f);

            if (active)
            {
                const std::string range = formatRange(glm::distance(target->getPosition(), m_missile->getPosition()));
                char speed[24];
                std::snprintf(speed, sizeof(speed), "%.0f m/s", glm::length(target->getVelocity()));
                const float monoSize = ui::px(13.0f);
                drawList->AddText(font.mono, monoSize, ImVec2(min.x + ui::px(214.0f) - textWidth(font.mono, monoSize, range.c_str()), textY + ui::px(1.0f)),
                                  ui::toU32(ui::color::text), range.c_str());
                drawList->AddText(font.mono, monoSize, ImVec2(max.x - ui::px(16.0f) - textWidth(font.mono, monoSize, speed), textY + ui::px(1.0f)),
                                  ui::toU32(ui::color::textMuted), speed);
            }
        }
    }

    // ---- Flight data strip (bottom centre) ------------------------------------------
    float stripTop = origin.y + height;
    {
        const glm::vec3 position = m_missile->getPosition();
        const glm::vec3 velocity = m_missile->getVelocity();
        const float speed = glm::length(velocity);
        const float altitude = std::max(position.y, 0.0f);
        float mach = 0.0f;
        if (m_physicsEngine)
        {
            const Atmosphere::State air = m_physicsEngine->getAtmosphereState(altitude);
            mach = air.speedOfSoundMetersPerSecond > 0.0f ? speed / air.speedOfSoundMetersPerSecond : 0.0f;
        }
        const glm::vec3 felt = m_missile->getAcceleration() + glm::vec3(0.0f, kGravity, 0.0f);
        const float loadFactor = m_missileInFlight ? glm::length(felt) / kGravity : 1.0f;
        const float fuelFraction = m_missileFuel > 0.0f ? std::clamp(m_missile->getFuel() / m_missileFuel, 0.0f, 1.0f) : 0.0f;

        struct Cell
        {
            const char *label;
            char value[24];
            const char *unit;
        };
        Cell cells[6] = {{"SPEED", "", "m/s"}, {"MACH", "", ""}, {"ALTITUDE", "", "m"}, {"LOAD", "", "g"}, {"FUEL", "", "%"}, {"FLIGHT", "", "s"}};
        std::snprintf(cells[0].value, sizeof(cells[0].value), "%.0f", speed);
        std::snprintf(cells[1].value, sizeof(cells[1].value), "%.2f", mach);
        std::snprintf(cells[2].value, sizeof(cells[2].value), "%.0f", altitude);
        std::snprintf(cells[3].value, sizeof(cells[3].value), "%.1f", loadFactor);
        std::snprintf(cells[4].value, sizeof(cells[4].value), "%.0f", fuelFraction * 100.0f);
        std::snprintf(cells[5].value, sizeof(cells[5].value), "%.1f", m_missileFlightTime);

        const float cellWidth = ui::px(98.0f);
        const float stripHeight = ui::px(68.0f);
        const float stripWidth = cellWidth * 6.0f + ui::px(12.0f);
        const float usableWidth = width - hudRightInset();
        const ImVec2 min(std::round(origin.x + (usableWidth - stripWidth) * 0.5f), origin.y + height - margin - stripHeight);
        const ImVec2 max(min.x + stripWidth, min.y + stripHeight);
        stripTop = min.y;
        drawGlass(drawList, min, max, 1.0f, ui::px(10.0f));

        const float valueAlpha = (m_missileInFlight || m_launchSequence.active) ? 1.0f : 0.55f;
        for (int i = 0; i < 6; ++i)
        {
            const float cellX = min.x + ui::px(6.0f) + cellWidth * static_cast<float>(i);
            if (i > 0)
            {
                drawList->AddLine(ImVec2(cellX, min.y + ui::px(16.0f)), ImVec2(cellX, max.y - ui::px(16.0f)), ui::toU32(ui::color::hairline));
            }
            const float innerX = cellX + ui::px(14.0f);
            ui::drawTracked(drawList, font.display, ui::px(11.5f), ImVec2(innerX, min.y + ui::px(12.0f)),
                            ui::toU32(ui::color::textMuted), cells[i].label, 0.18f);
            const float valueSize = ui::px(23.0f);
            const bool fuelLow = i == 4 && fuelFraction < 0.15f && m_missileInFlight;
            drawList->AddText(font.monoMedium, valueSize, ImVec2(innerX, min.y + ui::px(29.0f)),
                              ui::toU32(fuelLow ? ui::color::danger : ui::color::text, valueAlpha), cells[i].value);
            if (cells[i].unit[0] != '\0')
            {
                const float valueWidth = textWidth(font.monoMedium, valueSize, cells[i].value);
                drawList->AddText(font.body, ui::px(13.0f), ImVec2(innerX + valueWidth + ui::px(4.0f), min.y + ui::px(38.0f)),
                                  ui::toU32(ui::color::textMuted, valueAlpha), cells[i].unit);
            }
            if (i == 4)
            {
                const float barWidth = cellWidth - ui::px(28.0f);
                const float barY = max.y - ui::px(10.0f);
                drawList->AddRectFilled(ImVec2(innerX, barY), ImVec2(innerX + barWidth, barY + ui::px(2.5f)),
                                        ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.10f)), ui::px(1.0f));
                drawList->AddRectFilled(ImVec2(innerX, barY), ImVec2(innerX + barWidth * fuelFraction, barY + ui::px(2.5f)),
                                        ui::toU32(fuelLow ? ui::color::danger : ui::color::accent, valueAlpha), ui::px(1.0f));
            }
        }

        // Launch prompt while the round is on the rail.
        if (!m_missileInFlight && !m_launchSequence.active && !m_detonationHoldActive)
        {
            const char *launch = "Launch";
            const char *cue = "Seeker cue";
            const char *cueState = m_seekerCueEnabled ? "ON" : "OFF";
            const float bodySize = ui::px(14.5f);
            const float launchWidth = textWidth(font.body, bodySize, launch);
            const float cueWidth = textWidth(font.body, bodySize, cue);
            const float stateWidth = ui::measureTracked(font.display, ui::px(13.0f), cueState, 0.12f).x;
            const float promptWidth = ui::px(22.0f + 8.0f) + launchWidth + ui::px(28.0f) + ui::px(22.0f + 8.0f) + cueWidth + ui::px(8.0f) + stateWidth;
            float x = std::round(origin.x + (usableWidth - promptWidth) * 0.5f);
            const float y = min.y - ui::px(38.0f);
            x += ui::drawKeycap(drawList, ImVec2(x, y), "F") + ui::px(8.0f);
            drawList->AddText(font.body, bodySize, ImVec2(x, y + ui::px(2.5f)), ui::toU32(ui::color::text), launch);
            x += launchWidth + ui::px(28.0f);
            x += ui::drawKeycap(drawList, ImVec2(x, y), "R") + ui::px(8.0f);
            drawList->AddText(font.body, bodySize, ImVec2(x, y + ui::px(2.5f)), ui::toU32(ui::color::text), cue);
            x += cueWidth + ui::px(8.0f);
            ui::drawTracked(drawList, font.display, ui::px(13.0f), ImVec2(x, y + ui::px(4.0f)),
                            ui::toU32(m_seekerCueEnabled ? ui::color::accent : ui::color::textFaint), cueState, 0.12f);
        }
    }

    // ---- Key hints (bottom left, fade after the first seconds) --------------------------
    {
        const float alpha = 1.0f - std::clamp((m_hud.engagementTime - kHintsVisibleSeconds) / 1.5f, 0.0f, 1.0f);
        if (alpha > 0.001f)
        {
            const char *const hints[][2] = {{"Tab", "Controls"}, {"V", "Camera"}, {"H", "Hide HUD"}, {"Esc", "Menu"}};
            float y = origin.y + height - margin - ui::px(24.0f) * 4.0f;
            for (const auto &hint : hints)
            {
                const float capWidth = ui::drawKeycap(drawList, ImVec2(origin.x + margin, y), hint[0], alpha);
                (void)capWidth;
                drawList->AddText(font.body, ui::px(14.0f), ImVec2(origin.x + margin + ui::px(46.0f), y + ui::px(3.0f)),
                                  ui::toU32(ui::color::textMuted, alpha), hint[1]);
                y += ui::px(24.0f);
            }
        }
    }

    // ---- RWR scope (fighter camera) ----------------------------------------------------
    if (m_cameraMode == CameraMode::FIGHTER_JET)
    {
        const Target *jet = trackedTarget != nullptr ? trackedTarget : findBestTarget();
        const float radius = ui::px(74.0f);
        const ImVec2 centre(origin.x + width - margin - hudRightInset() - radius - ui::px(8.0f),
                            std::min(origin.y + height * 0.5f + ui::px(40.0f), stripTop - radius - ui::px(70.0f)));
        drawList->AddCircleFilled(centre, radius + ui::px(10.0f), ui::toU32(ImVec4(0.035f, 0.043f, 0.059f, 0.66f)), 64);
        drawList->AddCircle(centre, radius + ui::px(10.0f), ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.07f)), 64, 1.0f);
        drawList->AddCircle(centre, radius, ui::toU32(ui::color::textFaint), 64, 1.2f);
        drawList->AddCircle(centre, radius * 0.55f, ui::toU32(ui::color::hairline), 48, 1.0f);
        drawList->AddLine(ImVec2(centre.x - radius, centre.y), ImVec2(centre.x + radius, centre.y), ui::toU32(ui::color::hairline));
        drawList->AddLine(ImVec2(centre.x, centre.y - radius), ImVec2(centre.x, centre.y + radius), ui::toU32(ui::color::hairline));
        drawList->AddTriangleFilled(ImVec2(centre.x, centre.y - ui::px(6.0f)), ImVec2(centre.x - ui::px(4.5f), centre.y + ui::px(4.0f)),
                                    ImVec2(centre.x + ui::px(4.5f), centre.y + ui::px(4.0f)), ui::toU32(ui::color::text));
        ui::drawTracked(drawList, font.display, ui::px(12.0f), ImVec2(centre.x - ui::px(12.0f), centre.y - radius - ui::px(26.0f)),
                        ui::toU32(ui::color::textMuted), "RWR", 0.16f);

        const bool threat = jet != nullptr && jet->hasThreatAssessment();
        if (threat)
        {
            const glm::vec3 forward = safeNormalize(jet->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
            const glm::vec3 flatForward = safeNormalize(glm::vec3(forward.x, 0.0f, forward.z), glm::vec3(0.0f, 0.0f, 1.0f));
            const glm::vec3 right = safeNormalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), flatForward), glm::vec3(1.0f, 0.0f, 0.0f));
            const glm::vec3 incoming = jet->getThreatMissilePosition() - jet->getPosition();
            glm::vec2 cue(-glm::dot(incoming, right), glm::dot(incoming, flatForward));
            const float cueLength = glm::length(cue);
            cue = cueLength > 0.001f ? cue / cueLength : glm::vec2(0.0f, 1.0f);
            const ImVec2 cuePoint(centre.x + cue.x * radius * 0.78f, centre.y - cue.y * radius * 0.78f);
            const float pulse = 0.6f + 0.4f * (0.5f + 0.5f * std::sin(time * 9.0f));
            drawList->AddLine(centre, cuePoint, ui::toU32(ui::color::danger, 0.6f), ui::px(1.4f));
            drawList->AddCircleFilled(cuePoint, ui::px(5.5f), ui::toU32(ui::color::danger, pulse), 16);
            drawList->AddCircle(cuePoint, ui::px(11.0f), ui::toU32(ui::color::danger, 0.5f * pulse), 20, 1.4f);
        }

        char line[64];
        const float textY = centre.y + radius + ui::px(18.0f);
        const char *status = threat ? "MAWS  INBOUND" : "MAWS  CLEAR";
        const float statusWidth = ui::measureTracked(font.display, ui::px(14.0f), status, 0.12f).x;
        ui::drawTracked(drawList, font.display, ui::px(14.0f), ImVec2(centre.x - statusWidth * 0.5f, textY),
                        ui::toU32(threat ? ui::color::danger : ui::color::positive), status, 0.12f);
        if (threat)
        {
            std::snprintf(line, sizeof(line), "%s  \xC2\xB7  TCA %.1f s", formatRange(jet->getThreatDistance()).c_str(),
                          jet->getThreatTimeToClosestApproach());
            const float lineWidth = textWidth(font.mono, ui::px(12.5f), line);
            drawList->AddText(font.mono, ui::px(12.5f), ImVec2(centre.x - lineWidth * 0.5f, textY + ui::px(20.0f)),
                              ui::toU32(ui::color::textMuted), line);
        }
    }

    // ---- Engagement result card -------------------------------------------------------
    if (m_hud.resultVisible)
    {
        const float alpha = fadeWindow(m_hud.resultAge, kResultDuration, 0.25f, 0.8f);
        const float rise = (1.0f - std::clamp(m_hud.resultAge / 0.35f, 0.0f, 1.0f)) * ui::px(10.0f);
        const char *title = m_hud.resultHit ? "TARGET DESTROYED" : "MISSED";
        const ImVec4 tone = m_hud.resultHit ? ui::color::positive : ui::color::danger;
        const float titleSize = ui::px(40.0f);
        const float titleWidth = ui::measureTracked(font.display, titleSize, title, 0.1f).x;

        char flight[32], pass[32], mach[32];
        std::snprintf(flight, sizeof(flight), "%.1f s", m_hud.resultFlightTime);
        if (m_hud.resultClosestPass < 999999.0f)
        {
            std::snprintf(pass, sizeof(pass), "%s", formatRange(m_hud.resultClosestPass).c_str());
        }
        else
        {
            std::snprintf(pass, sizeof(pass), "\xE2\x80\x94");
        }
        std::snprintf(mach, sizeof(mach), "%.2f", m_hud.resultPeakMach);
        const char *const labels[] = {"FLIGHT TIME", "CLOSEST PASS", "PEAK MACH"};
        const char *const values[] = {flight, pass, mach};
        const float columnWidth = ui::px(150.0f);
        const float statsWidth = columnWidth * 3.0f;
        const float cardWidth = std::max(titleWidth, statsWidth) + ui::px(80.0f);
        const float cardHeight = ui::px(156.0f);
        const float usableWidth = width - hudRightInset();
        const ImVec2 min(std::round(origin.x + (usableWidth - cardWidth) * 0.5f), std::round(origin.y + height * 0.2f + rise));
        const ImVec2 max(min.x + cardWidth, min.y + cardHeight);
        drawList->AddRectFilled(min, max, ui::toU32(ImVec4(0.035f, 0.043f, 0.059f, 0.80f), alpha), ui::px(10.0f));
        drawList->AddRect(min, max, ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.08f), alpha), ui::px(10.0f));
        drawList->AddRectFilled(ImVec2(min.x + cardWidth * 0.5f - ui::px(18.0f), min.y), ImVec2(min.x + cardWidth * 0.5f + ui::px(18.0f), min.y + ui::px(3.0f)),
                                ui::toU32(tone, alpha));
        ui::drawTracked(drawList, font.display, titleSize, ImVec2(min.x + (cardWidth - titleWidth) * 0.5f, min.y + ui::px(24.0f)),
                        ui::toU32(mix(tone, ui::color::text, 0.25f), alpha), title, 0.1f);

        const float statsX = min.x + (cardWidth - statsWidth) * 0.5f;
        for (int i = 0; i < 3; ++i)
        {
            const float columnX = statsX + columnWidth * static_cast<float>(i);
            const float labelWidth = ui::measureTracked(font.display, ui::px(11.5f), labels[i], 0.16f).x;
            ui::drawTracked(drawList, font.display, ui::px(11.5f), ImVec2(columnX + (columnWidth - labelWidth) * 0.5f, min.y + ui::px(88.0f)),
                            ui::toU32(ui::color::textMuted, alpha), labels[i], 0.16f);
            const float valueWidth = textWidth(font.monoMedium, ui::px(19.0f), values[i]);
            drawList->AddText(font.monoMedium, ui::px(19.0f), ImVec2(columnX + (columnWidth - valueWidth) * 0.5f, min.y + ui::px(108.0f)),
                              ui::toU32(ui::color::text, alpha), values[i]);
        }
    }
}
