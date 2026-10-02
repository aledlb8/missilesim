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

#include "objects/Fighter.h"
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
    constexpr float kHintsVisibleSeconds = 12.0f;
    constexpr float kGravity = 9.80665f;

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
    const missilesim::sim::Shot *shot = followedShot();
    if (shot == nullptr)
    {
        if (m_world && m_world->readyRound() == nullptr)
        {
            return m_playerRole == PlayerRole::Fighter ? "EMPTY" : "RELOADING";
        }
        return "READY";
    }
    if (shot->coldLaunch.active && !shot->coldLaunch.motorIgnited)
    {
        return "EJECT";
    }
    if (shot->coldLaunch.active && !shot->coldLaunch.guidanceArmed)
    {
        return "IGNITION";
    }
    const Missile &missile = *shot->missile;
    if (missile.isFox2())
    {
        if (missile.fox2SeekerLocked() || missile.fox2OnTrackMemory())
        {
            return missile.isThrustEnabled() ? "INTERCEPT" : "GLIDE";
        }
        return missile.isThrustEnabled() ? "BOOST" : "BALLISTIC";
    }
    const bool locked = missile.isGuidanceEnabled() && getTrackedMissileTarget() != nullptr;
    const bool thrust = missile.isThrustEnabled();
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

void Application::updateHudTracker(float deltaTime)
{
    if (!m_isPaused)
    {
        m_hud.engagementTime += std::max(deltaTime, 0.0f);
    }
}

void Application::renderHud()
{
    if (!m_renderer || !m_world)
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

    Target *trackedTarget = getTrackedMissileTarget();
    const std::vector<std::unique_ptr<Target>> &aircraft = targets();
    auto targetIndex = [&aircraft](const Target *target) -> int
    {
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            if (aircraft[i].get() == target)
            {
                return static_cast<int>(i) + 1;
            }
        }
        return 0;
    };
    const Missile *focus = focusMissile();
    const missilesim::sim::Shot *shot = followedShot();
    const bool inFlight = shot != nullptr;

    // ---- Projection (keeps off-screen and behind-camera directions) --------
    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    const glm::vec3 cameraForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 cameraRight = safeNormalize(m_renderer->getCameraRight(), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 cameraUp = safeNormalize(m_renderer->getCameraUp(), glm::vec3(0.0f, 1.0f, 0.0f));
    // Ranges are read from the round the player is watching or about to fire.
    const glm::vec3 rangeOrigin = focus != nullptr ? focus->getPosition() : cameraPosition;
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
        const Target *chaseSubject = (m_cameraMode == CameraMode::FIGHTER_JET && m_playerRole != PlayerRole::Fighter)
                                         ? (trackedTarget != nullptr ? trackedTarget : findBestTarget())
                                         : nullptr;
        const float edgeInset = ui::px(44.0f);
        const ImVec2 centre(origin.x + width * 0.5f, origin.y + height * 0.5f);

        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            const Target *target = aircraft[i].get();
            if (!target->isActive() || target == chaseSubject)
            {
                continue;
            }
            const bool isLocked = target == trackedTarget;
            const bool warning = target->isMissileWarningActive();
            const ImVec4 colour = isLocked ? ui::color::accent : ui::withAlpha(ui::color::text, 0.88f);
            const float range = glm::distance(target->getPosition(), rangeOrigin);
            const std::string rangeText = formatRange(range);
            char idText[8];
            std::snprintf(idText, sizeof(idText), "T%zu", i + 1);

            const Projected p = project(target->getRenderPosition());
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

        // A marker on every round in the air, except the one being ridden.
        for (const auto &flying : m_world->shots())
        {
            if (flying->id == m_followedShot && m_cameraMode == CameraMode::MISSILE)
            {
                continue;
            }
            const Projected p = project(flying->missile->getRenderPosition());
            if (p.onScreen)
            {
                const float s = ui::px(6.0f);
                const bool followed = flying->id == m_followedShot;
                const ImU32 colour = ui::toU32(ui::color::info, followed ? 1.0f : 0.7f);
                drawList->AddQuad(ImVec2(p.screen.x, p.screen.y - s), ImVec2(p.screen.x + s, p.screen.y),
                                  ImVec2(p.screen.x, p.screen.y + s), ImVec2(p.screen.x - s, p.screen.y), colour, ui::px(1.6f));
                drawList->AddText(font.monoMedium, ui::px(11.5f), ImVec2(p.screen.x + s + ui::px(5.0f), p.screen.y - ui::px(6.5f)),
                                  colour, "MSL");
            }
        }
    }

    // ---- Mouse aim (fighter camera) ----------------------------------------------
    // War Thunder's two marks: the circle is where the mouse asks the nose to
    // go, the cross is where the nose points now. The instructor flies the
    // cross onto the circle.
    const Fighter *jet = fighter();
    if (m_playerRole == PlayerRole::Fighter && jet && m_cameraMode == CameraMode::FIGHTER_JET)
    {
        constexpr float kMarkerRange = 1500.0f;
        const glm::vec3 origin3 = jet->getRenderPosition();
        const ImU32 shade = ui::toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.35f));
        const ImU32 ink = ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.92f));

        const Projected nose = project(origin3 + jet->getRenderNose() * kMarkerRange);
        if (nose.onScreen)
        {
            const float arm = ui::px(9.0f);
            const float gap = ui::px(3.0f);
            const ImVec2 c = nose.screen;
            for (int pass = 0; pass < 2; ++pass)
            {
                const ImU32 colour = pass == 0 ? shade : ink;
                const float width = pass == 0 ? ui::px(3.4f) : ui::px(1.6f);
                drawList->AddLine(ImVec2(c.x - arm, c.y), ImVec2(c.x - gap, c.y), colour, width);
                drawList->AddLine(ImVec2(c.x + gap, c.y), ImVec2(c.x + arm, c.y), colour, width);
                drawList->AddLine(ImVec2(c.x, c.y - arm), ImVec2(c.x, c.y - gap), colour, width);
                drawList->AddLine(ImVec2(c.x, c.y + gap), ImVec2(c.x, c.y + arm), colour, width);
            }
        }

        const Projected aim = project(origin3 + m_aimCamera.aimDirection() * kMarkerRange);
        if (aim.onScreen)
        {
            const float radius = ui::px(12.0f);
            const bool freeLook = m_aimCamera.isFreeLook();
            drawList->AddCircle(aim.screen, radius, shade, 40, ui::px(3.6f));
            drawList->AddCircle(aim.screen, radius, freeLook ? ui::toU32(ui::color::accent) : ink, 40, ui::px(1.7f));
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
        else if (std::strcmp(state, "BALLISTIC") == 0 || std::strcmp(state, "EMPTY") == 0 || std::strcmp(state, "RELOADING") == 0)
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
        if (trackedTarget != nullptr && (inFlight || seekerUncaged()))
        {
            detail += "  \xC2\xB7  T" + std::to_string(targetIndex(trackedTarget));
        }
        if (inFlight)
        {
            char flight[32];
            std::snprintf(flight, sizeof(flight), "  \xC2\xB7  %.1f s", shot->flightTime);
            detail += flight;
        }
        const std::size_t airborne = m_world->shots().size();
        if (airborne > 1)
        {
            detail += "  \xC2\xB7  " + std::to_string(airborne) + " in the air";
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
        }
    }

    // ---- Target roster (top right) -----------------------------------------------
    if (!aircraft.empty())
    {
        const size_t rows = std::min<size_t>(aircraft.size(), 8);
        const float panelWidth = ui::px(296.0f);
        const float rowHeight = ui::px(26.0f);
        const float right = origin.x + width - margin - hudRightInset();
        const ImVec2 min(right - panelWidth, origin.y + margin - ui::px(4.0f));
        const ImVec2 max(right, min.y + ui::px(42.0f) + rowHeight * static_cast<float>(rows) + ui::px(8.0f));
        drawGlass(drawList, min, max, 1.0f, ui::px(8.0f));

        int activeCount = 0;
        for (const auto &target : aircraft)
        {
            activeCount += target->isActive() ? 1 : 0;
        }
        ui::drawTracked(drawList, font.display, ui::px(13.0f), ImVec2(min.x + ui::px(16.0f), min.y + ui::px(14.0f)),
                        ui::toU32(ui::color::textMuted), "TARGETS", 0.16f);
        char count[32];
        std::snprintf(count, sizeof(count), "%d / %zu", activeCount, aircraft.size());
        drawList->AddText(font.monoMedium, ui::px(13.0f), ImVec2(max.x - ui::px(16.0f) - textWidth(font.monoMedium, ui::px(13.0f), count), min.y + ui::px(14.0f)),
                          ui::toU32(ui::color::text), count);

        for (size_t i = 0; i < rows; ++i)
        {
            const Target *target = aircraft[i].get();
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
                const std::string range = formatRange(glm::distance(target->getPosition(), rangeOrigin));
                char speed[24];
                std::snprintf(speed, sizeof(speed), "%.0f km/h", glm::length(target->getVelocity()) * 3.6f);
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
        const glm::vec3 position = focus != nullptr ? focus->getPosition() : glm::vec3(0.0f);
        const glm::vec3 velocity = focus != nullptr ? focus->getVelocity() : glm::vec3(0.0f);
        const float speed = glm::length(velocity);
        const float altitude = std::max(position.y, 0.0f);
        float mach = 0.0f;
        if (physics())
        {
            const Atmosphere::State air = physics()->getAtmosphereState(altitude);
            mach = air.speedOfSoundMetersPerSecond > 0.0f ? speed / air.speedOfSoundMetersPerSecond : 0.0f;
        }
        // The jet's own numbers unless the camera rides a round in the air.
        const bool fighterStrip = m_playerRole == PlayerRole::Fighter && jet != nullptr &&
                                  !(inFlight && m_cameraMode == CameraMode::MISSILE);
        const glm::vec3 felt = (focus != nullptr ? focus->getAcceleration() : glm::vec3(0.0f)) + glm::vec3(0.0f, kGravity, 0.0f);
        const float loadFactor = fighterStrip ? jet->getLoadFactor()
                                               : (inFlight ? glm::length(felt) / kGravity : 1.0f);
        const float fuelCapacity = focus != nullptr ? focus->getFuelCapacity() : 0.0f;
        const float fuelFraction = fuelCapacity > 0.0f ? std::clamp(focus->getFuel() / fuelCapacity, 0.0f, 1.0f) : 0.0f;
        const float stripSpeed = fighterStrip ? glm::length(jet->getVelocity()) : speed;
        float stripMach = mach;
        float stripAltitude = altitude;
        if (fighterStrip && physics())
        {
            stripAltitude = std::max(jet->getPosition().y, 0.0f);
            const Atmosphere::State fighterAir = physics()->getAtmosphereState(stripAltitude);
            stripMach = fighterAir.speedOfSoundMetersPerSecond > 0.0f ? stripSpeed / fighterAir.speedOfSoundMetersPerSecond : 0.0f;
        }
        const float throttleFraction = fighterStrip ? glm::clamp(jet->getThrottleLever() / Fighter::kMaxLever, 0.0f, 1.0f) : fuelFraction;

        struct Cell
        {
            const char *label;
            char value[24];
            const char *unit;
        };
        Cell cells[6] = {{"SPEED", "", "km/h"}, {"MACH", "", ""}, {"ALTITUDE", "", "m"}, {"LOAD", "", "g"}, {"FUEL", "", "%"}, {"FLIGHT", "", "s"}};
        if (fighterStrip)
        {
            cells[4].label = "THR";
            cells[4].unit = jet->isAfterburner() ? "AB" : "%";
            cells[5].label = "RND";
            cells[5].unit = "";
        }
        std::snprintf(cells[0].value, sizeof(cells[0].value), "%.0f", stripSpeed * 3.6f);
        std::snprintf(cells[1].value, sizeof(cells[1].value), "%.2f", stripMach);
        std::snprintf(cells[2].value, sizeof(cells[2].value), "%.0f", stripAltitude);
        std::snprintf(cells[3].value, sizeof(cells[3].value), "%.1f", loadFactor);
        std::snprintf(cells[4].value, sizeof(cells[4].value), "%.0f",
                      fighterStrip ? jet->getThrottleLever() * 100.0f : fuelFraction * 100.0f);
        if (fighterStrip)
        {
            std::snprintf(cells[5].value, sizeof(cells[5].value), "%d", std::max(m_world->roundsRemaining(), 0));
        }
        else
        {
            std::snprintf(cells[5].value, sizeof(cells[5].value), "%.1f", inFlight ? shot->flightTime : 0.0f);
        }

        const float cellWidth = ui::px(116.0f);
        const float stripHeight = ui::px(68.0f);
        const float stripWidth = cellWidth * 6.0f + ui::px(12.0f);
        const float usableWidth = width - hudRightInset();
        const ImVec2 min(std::round(origin.x + (usableWidth - stripWidth) * 0.5f), origin.y + height - margin - stripHeight);
        const ImVec2 max(min.x + stripWidth, min.y + stripHeight);
        stripTop = min.y;
        drawGlass(drawList, min, max, 1.0f, ui::px(10.0f));

        const float valueAlpha = (fighterStrip || inFlight) ? 1.0f : 0.55f;
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
            const bool fuelLow = i == 4 && !fighterStrip && fuelFraction < 0.15f && inFlight;
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
                drawList->AddRectFilled(ImVec2(innerX, barY), ImVec2(innerX + barWidth * (fighterStrip ? throttleFraction : fuelFraction), barY + ui::px(2.5f)),
                                        ui::toU32(fuelLow ? ui::color::danger : ui::color::accent, valueAlpha), ui::px(1.0f));
            }
        }

        // Launch prompt while a round is loaded, or what is holding it up.
        const Missile *ready = m_world->readyRound();
        if (ready != nullptr)
        {
            const char *launch = "Launch";
            const char *cue = "Seeker cue";
            if (ready->fox2Spec() != nullptr)
            {
                launch = ready->fox2Spec()->displayName;
            }
            const char *cueState = seekerUncaged() ? "ON" : "OFF";
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
                            ui::toU32(seekerUncaged() ? ui::color::accent : ui::color::textFaint), cueState, 0.12f);
        }
        else
        {
            const bool rearmable = m_playerRole == PlayerRole::Fighter;
            const char *status = rearmable ? "Rails empty" : "Reloading";
            const float bodySize = ui::px(14.5f);
            const float statusWidth = textWidth(font.body, bodySize, status);
            const float capWidth = rearmable ? ui::px(22.0f + 8.0f) + textWidth(font.body, bodySize, "Rearm") + ui::px(28.0f) : 0.0f;
            float x = std::round(origin.x + (usableWidth - statusWidth - capWidth) * 0.5f);
            const float y = min.y - ui::px(38.0f);
            drawList->AddText(font.body, bodySize, ImVec2(x, y + ui::px(2.5f)), ui::toU32(ui::color::textMuted), status);
            if (rearmable)
            {
                x += statusWidth + ui::px(28.0f);
                x += ui::drawKeycap(drawList, ImVec2(x, y), "G") + ui::px(8.0f);
                drawList->AddText(font.body, bodySize, ImVec2(x, y + ui::px(2.5f)), ui::toU32(ui::color::text), "Rearm");
            }
        }

        // Why the last launch was refused, above the prompt.
        if (m_launchNoticeTimer > 0.0f && m_launchNotice != missilesim::sim::LaunchBlock::None)
        {
            std::string notice = missilesim::sim::launchBlockMessage(m_launchNotice);
            std::transform(notice.begin(), notice.end(), notice.begin(),
                           [](unsigned char c)
                           { return static_cast<char>(std::toupper(c)); });
            const float alpha = std::clamp(m_launchNoticeTimer / 0.4f, 0.0f, 1.0f);
            const float noticeWidth = ui::measureTracked(font.display, ui::px(14.0f), notice.c_str(), 0.14f).x;
            ui::drawTracked(drawList, font.display, ui::px(14.0f),
                            ImVec2(std::round(origin.x + (usableWidth - noticeWidth) * 0.5f), min.y - ui::px(66.0f)),
                            ui::toU32(ui::color::danger, alpha), notice.c_str(), 0.14f);
        }
    }

    // ---- Key hints (bottom left, fade after the first seconds) --------------------------
    {
        const float alpha = 1.0f - std::clamp((m_hud.engagementTime - kHintsVisibleSeconds) / 1.5f, 0.0f, 1.0f);
        if (alpha > 0.001f)
        {
            static const char *const samHints[][2] = {{"Tab", "Controls"}, {"V", "Camera"}, {"H", "Hide HUD"}, {"Esc", "Menu"}};
            static const char *const fighterHints[][2] = {{"Mouse", "Aim"},       {"RMB", "Free look"},
                                                          {"Shift", "Throttle up"}, {"Ctrl", "Throttle down"}, {"X", "Afterburner"},
                                                          {"F", "Fire"},          {"V", "Camera"},         {"Esc", "Menu"}};
            const bool fighterRole = m_playerRole == PlayerRole::Fighter;
            const auto *hints = fighterRole ? fighterHints : samHints;
            const size_t hintCount = fighterRole ? std::size(fighterHints) : std::size(samHints);
            float y = origin.y + height - margin - ui::px(24.0f) * static_cast<float>(hintCount);
            const float textX = origin.x + margin + ui::px(fighterRole ? 64.0f : 46.0f);
            for (size_t i = 0; i < hintCount; ++i)
            {
                ui::drawKeycap(drawList, ImVec2(origin.x + margin, y), hints[i][0], alpha);
                drawList->AddText(font.body, ui::px(14.0f), ImVec2(textX, y + ui::px(3.0f)),
                                  ui::toU32(ui::color::textMuted, alpha), hints[i][1]);
                y += ui::px(24.0f);
            }
        }
    }

    // ---- RWR scope (fighter camera) ----------------------------------------------------
    if (m_cameraMode == CameraMode::FIGHTER_JET)
    {
        const Target *subject = trackedTarget != nullptr ? trackedTarget : findBestTarget();
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

        const bool threat = m_playerRole != PlayerRole::Fighter && subject != nullptr && subject->hasThreatAssessment();
        if (threat)
        {
            const glm::vec3 forward = safeNormalize(subject->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
            const glm::vec3 flatForward = safeNormalize(glm::vec3(forward.x, 0.0f, forward.z), glm::vec3(0.0f, 0.0f, 1.0f));
            const glm::vec3 right = safeNormalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), flatForward), glm::vec3(1.0f, 0.0f, 0.0f));
            const glm::vec3 incoming = subject->getThreatMissilePosition() - subject->getPosition();
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
            std::snprintf(line, sizeof(line), "%s  \xC2\xB7  TCA %.1f s", formatRange(subject->getThreatDistance()).c_str(),
                          subject->getThreatTimeToClosestApproach());
            const float lineWidth = textWidth(font.mono, ui::px(12.5f), line);
            drawList->AddText(font.mono, ui::px(12.5f), ImVec2(centre.x - lineWidth * 0.5f, textY + ui::px(20.0f)),
                              ui::toU32(ui::color::textMuted), line);
        }
    }
}
