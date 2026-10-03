// In-engagement HUD: mission status, camera selector, target roster, flight
// data strip, aircraft markers with off-screen arrows, the fighter's fire
// control (radar lock, seeker circle, radar contacts), the radar scope and the
// RWR. Everything is drawn on the background draw list, so the HUD never
// captures the mouse and always sits beneath panels and menus.
//
// The fighter's targeting symbols all come from one picture,
// sim::World::fireControl(): what the radar and the seeker know, never a live
// aircraft. Only the pilot's own eyes mark aircraft directly, and only inside
// visual range. The SAM sandbox keeps its omniscient brackets.
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
#include "sim/Fox2Catalog.h"
#include "sim/Fox3Catalog.h"
#include "ui/RadarScope.h"
#include "ui/RwrScope.h"
#include "ui/TargetSymbols.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

using missilesim::application::detail::safeNormalize;
namespace sim = missilesim::sim;
namespace ui = missilesim::ui;

namespace
{
    constexpr float kHintsVisibleSeconds = 12.0f;
    constexpr float kGravity = 9.80665f;
    // Sim choice: past about 8 km a fighter is a speck, so the pilot's own
    // eyes mark nothing further out. The radar and seeker see further.
    constexpr float kVisualRangeM = 8000.0f;
    // Directions are drawn as points this far down them.
    constexpr float kMarkerRange = 1500.0f;
    // Display choice: the seeker circle's angular radius. Not its field of view.
    constexpr float kSeekerCircleRad = 0.0436f; // 2.5 degrees
    // A threat stays on the RWR this long after it was last heard, fading,
    // so a sweeping radar reads as one steady symbol.
    constexpr double kRwrHoldSeconds = 1.5;
    // Scope range scales the Y key steps through. The fighter radar's
    // detection range on a 1 m² contact is about 18 km.
    constexpr float kRadarRangeScalesM[] = {10000.0f, 20000.0f, 40000.0f};

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

    const char *seekerStateLabel(sim::SeekerState state)
    {
        switch (state)
        {
        case sim::SeekerState::Off:
            return "off";
        case sim::SeekerState::Caged:
            return "caged";
        case sim::SeekerState::Search:
            return "searching";
        case sim::SeekerState::Slaved:
            return "slaved to the radar";
        case sim::SeekerState::Designated:
            return "designated";
        case sim::SeekerState::Locked:
            return "heat lock";
        }
        return "off";
    }

    float pulse(float time, float rate)
    {
        return 0.5f + 0.5f * std::sin(time * rate);
    }

    struct Projected
    {
        ImVec2 screen;
        float ndcX = 0.0f;
        float ndcY = 0.0f;
        bool onScreen = false;
    };

    // The camera's projection. Off-screen and behind-camera points keep a
    // direction, so an edge arrow can point at them.
    struct ScreenProjector
    {
        glm::vec3 position{0.0f};
        glm::vec3 forward{0.0f, 0.0f, 1.0f};
        glm::vec3 right{1.0f, 0.0f, 0.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        float tanHalfFov = 1.0f;
        float aspect = 1.0f;
        ImVec2 origin{0.0f, 0.0f};
        float width = 1.0f;
        float height = 1.0f;

        Projected project(const glm::vec3 &world) const
        {
            Projected p;
            const glm::vec3 toPoint = world - position;
            const float depth = glm::dot(toPoint, forward);
            const float safeDepth = std::max(std::abs(depth), 0.01f);
            p.ndcX = glm::dot(toPoint, right) / (safeDepth * tanHalfFov * aspect);
            p.ndcY = glm::dot(toPoint, up) / (safeDepth * tanHalfFov);
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
        }

        // Screen pixels an angle spans near the middle of the view.
        float pixelsForAngle(float radians) const
        {
            return std::tan(radians) / std::max(tanHalfFov, 1.0e-4f) * height * 0.5f;
        }
    };

    // Arrow at the screen edge pointing at an off-screen point, labelled inside it.
    void drawEdgeArrow(ImDrawList *drawList, const ScreenProjector &screen, const Projected &p, ImU32 colour,
                       const char *label)
    {
        float dx = p.ndcX;
        float dy = -p.ndcY;
        const float length = std::sqrt(dx * dx + dy * dy);
        if (length < 0.0001f)
        {
            return;
        }
        dx /= length;
        dy /= length;
        const float edgeInset = ui::px(44.0f);
        const ImVec2 centre(screen.origin.x + screen.width * 0.5f, screen.origin.y + screen.height * 0.5f);
        const float reachX = screen.width * 0.5f - edgeInset;
        const float reachY = screen.height * 0.5f - edgeInset;
        const float t = std::min(reachX / std::max(std::abs(dx), 0.0001f), reachY / std::max(std::abs(dy), 0.0001f));
        const ImVec2 point(centre.x + dx * t, centre.y + dy * t);
        const ImVec2 perp(-dy, dx);
        const float tip = ui::px(9.0f);
        const float back = ui::px(5.0f);
        const float wing = ui::px(7.0f);
        drawList->AddTriangleFilled(ImVec2(point.x + dx * tip, point.y + dy * tip),
                                    ImVec2(point.x - dx * back + perp.x * wing, point.y - dy * back + perp.y * wing),
                                    ImVec2(point.x - dx * back - perp.x * wing, point.y - dy * back - perp.y * wing), colour);
        if (label == nullptr || label[0] == '\0')
        {
            return;
        }
        const ui::Fonts &font = ui::fonts();
        const float labelWidth = textWidth(font.mono, ui::px(12.5f), label);
        const ImVec2 labelCentre(point.x - dx * ui::px(30.0f), point.y - dy * ui::px(24.0f));
        drawList->AddText(font.mono, ui::px(12.5f), ImVec2(labelCentre.x - labelWidth * 0.5f, labelCentre.y - ui::px(7.0f)),
                          colour, label);
    }

    // Small square marks on the radar's other contacts, in the world.
    void drawRadarContacts(ImDrawList *drawList, const ScreenProjector &screen, const sim::World &world)
    {
        const double now = world.time();
        for (const sim::TrackEstimate &track : world.playerRadar().tracks().tracks())
        {
            if (track.life == sim::TrackLife::Lost || track.id == world.designatedRadarTrack())
            {
                continue;
            }
            const glm::vec3 position = track.position + track.velocity * static_cast<float>(std::max(0.0, now - track.time));
            const Projected p = screen.project(position);
            if (!p.onScreen)
            {
                continue;
            }
            const float alpha = track.life == sim::TrackLife::Coasting ? 0.35f : (track.life == sim::TrackLife::Tentative ? 0.5f : 0.8f);
            ui::drawContactMark(drawList, p.screen, alpha);
        }
    }

    // The radar lock: a box on the track's estimated position, its range and
    // closing speed, and a SHOOT cue when the selected weapon can fire on it.
    void drawLockBox(ImDrawList *drawList, const ScreenProjector &screen, const sim::FireControl &fire, float time)
    {
        const sim::LockPicture &lock = fire.lock;
        if (!lock.valid)
        {
            return;
        }
        // A lock the beam is not holding, or a track flying on memory, draws faint.
        const bool held = fire.radar == sim::RadarMode::Track && lock.life != sim::TrackLife::Coasting;
        const bool ready = fire.clearance == sim::LaunchBlock::None;
        const ui::LockStyle style = ready ? ui::LockStyle::Shoot : (held ? ui::LockStyle::Held : ui::LockStyle::Memory);
        const std::string range = formatRange(lock.rangeM);
        const Projected p = screen.project(lock.position);
        if (!p.onScreen)
        {
            const std::string label = "LOCK  " + range;
            const ImVec4 &tone = ready ? ui::color::positive : ui::color::accent;
            drawEdgeArrow(drawList, screen, p, ui::toU32(tone, held ? 1.0f : 0.5f), label.c_str());
            return;
        }
        char closing[24];
        std::snprintf(closing, sizeof(closing), "%+.0f m/s", static_cast<double>(lock.closingMps));
        ui::drawLockBox(drawList, p.screen, style, range.c_str(), closing, time);
    }

    // The heat seeker's head: a circle where it looks. White while it
    // searches or follows the radar, amber with a heat lock.
    void drawSeekerCircle(ImDrawList *drawList, const ScreenProjector &screen, const sim::FireControl &fire,
                          const glm::vec3 &origin, float time)
    {
        const sim::SeekerPicture &seeker = fire.seeker;
        if (fire.weapon != sim::FighterWeapon::Fox2 || seeker.family != sim::SensorFamily::Infrared ||
            seeker.state == sim::SeekerState::Off || seeker.state == sim::SeekerState::Caged)
        {
            return;
        }
        const Projected p = screen.project(origin + seeker.lookDirection * kMarkerRange);
        if (!p.onScreen)
        {
            return;
        }
        const float radius = std::clamp(screen.pixelsForAngle(kSeekerCircleRad), ui::px(12.0f), ui::px(42.0f));
        ui::SeekerMark mark = ui::SeekerMark::Search;
        switch (seeker.state)
        {
        case sim::SeekerState::Slaved:
            mark = ui::SeekerMark::Slaved;
            break;
        case sim::SeekerState::Designated:
            mark = ui::SeekerMark::Designated;
            break;
        case sim::SeekerState::Locked:
            mark = ui::SeekerMark::Locked;
            break;
        default:
            break;
        }
        ui::drawSeekerCircle(drawList, p.screen, radius, mark, time);
    }

    // The weapon line above the flight strip: what is selected, what its
    // sensors have, and the one key that moves it forward.
    struct WeaponLine
    {
        const char *status = "";
        ImVec4 tone = ui::color::textMuted;
        const char *hintKey = nullptr;
        const char *hintText = nullptr;
    };

    WeaponLine weaponLine(const sim::FireControl &fire)
    {
        using sim::LaunchBlock;
        WeaponLine line;
        if (fire.clearance == LaunchBlock::NoRound || fire.clearance == LaunchBlock::RadarMagazineEmpty)
        {
            line.status = "EMPTY";
            line.tone = ui::color::danger;
            line.hintKey = "G";
            line.hintText = "Rearm";
            return line;
        }
        if (fire.clearance == LaunchBlock::Reloading)
        {
            line.status = "RELOADING";
            return line;
        }
        if (fire.clearance == LaunchBlock::NoLauncher)
        {
            line.status = "NO LAUNCHER";
            line.tone = ui::color::danger;
            return line;
        }

        if (fire.weapon == sim::FighterWeapon::RadarRound)
        {
            if (fire.clearance == LaunchBlock::None)
            {
                line.status = "SHOOT";
                line.tone = ui::color::positive;
                line.hintKey = "F";
                line.hintText = "Fire";
            }
            else if (!fire.lock.valid)
            {
                line.status = "NO LOCK";
                line.hintKey = "T";
                line.hintText = "Radar lock";
            }
            else if (fire.clearance == LaunchBlock::NoLaunchQuality)
            {
                line.status = "LOCKING";
                line.tone = ui::color::accent;
            }
            else
            {
                // The card cannot be flown in this build; the scope says why.
                line.status = "UNAVAILABLE";
                line.tone = ui::color::danger;
            }
            return line;
        }

        switch (fire.seeker.state)
        {
        case sim::SeekerState::Off:
            line.status = "NO SEEKER";
            break;
        case sim::SeekerState::Caged:
            line.status = "CAGED";
            line.hintKey = "R";
            line.hintText = "Uncage";
            break;
        case sim::SeekerState::Search:
            line.status = "SEARCH";
            line.tone = ui::color::text;
            line.hintKey = "T";
            line.hintText = "Radar lock";
            break;
        case sim::SeekerState::Slaved:
            line.status = "SLAVED";
            line.tone = ui::color::info;
            break;
        case sim::SeekerState::Designated:
            line.status = "DESIGNATED";
            line.tone = ui::color::accent;
            break;
        case sim::SeekerState::Locked:
            line.status = "IR LOCK";
            line.tone = ui::color::accent;
            break;
        }
        if (fire.clearance == LaunchBlock::None)
        {
            line.tone = ui::color::positive;
            line.hintKey = "F";
            line.hintText = "Fire";
        }
        return line;
    }

    ui::RwrKind rwrKind(sim::WarningKind kind)
    {
        switch (kind)
        {
        case sim::WarningKind::RadarSearch:
            return ui::RwrKind::Search;
        case sim::WarningKind::RadarTrack:
            return ui::RwrKind::Track;
        case sim::WarningKind::RadarLaunch:
            return ui::RwrKind::Launch;
        case sim::WarningKind::MissileSeeker:
            return ui::RwrKind::Seeker;
        case sim::WarningKind::Approach:
            return ui::RwrKind::Approach;
        }
        return ui::RwrKind::Search;
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
    const sim::Shot *shot = followedShot();
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
    if (shot->radarGuided)
    {
        using Phase = sim::RadarHomingState::Phase;
        switch (shot->homing.state().phase)
        {
        case Phase::Midcourse:
            return shot->supportFresh ? "DATALINK" : "COAST";
        case Phase::SeekerSearch:
            return "SEARCH";
        case Phase::Terminal:
            return "TERMINAL";
        case Phase::Memory:
            return "MEMORY";
        case Phase::Ended:
            return "ENDED";
        }
        return "ENDED";
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

void Application::cycleRadarRangeScale()
{
    const std::size_t count = std::size(kRadarRangeScalesM);
    std::size_t next = 0;
    for (std::size_t index = 0; index < count; ++index)
    {
        if (std::abs(kRadarRangeScalesM[index] - m_radarRangeScaleM) < 1.0f)
        {
            next = (index + 1) % count;
            break;
        }
    }
    m_radarRangeScaleM = kRadarRangeScalesM[next];
}

void Application::updateRwrMemory()
{
    if (!m_world || m_playerRole != PlayerRole::Fighter)
    {
        m_rwrMemory.clear();
        return;
    }
    const double now = m_world->time();
    // A restart rewinds the clock; nothing heard before it is still there.
    m_rwrMemory.erase(std::remove_if(m_rwrMemory.begin(), m_rwrMemory.end(),
                                     [now](const RwrMemory &memory) { return memory.heardTime > now + 1.0e-9; }),
                      m_rwrMemory.end());

    const sim::WarningPicture &picture = m_world->warnings();
    const auto remember = [this, now](const sim::Warning &warning) {
        const bool approach = warning.kind == sim::WarningKind::Approach;
        for (RwrMemory &memory : m_rwrMemory)
        {
            if (memory.source == warning.source.value && memory.approach == approach)
            {
                memory.kind = warning.kind;
                memory.azimuthRad = warning.azimuthRad;
                memory.heardTime = now;
                return;
            }
        }
        m_rwrMemory.push_back(RwrMemory{warning.source.value, approach, warning.kind, warning.azimuthRad, now});
    };
    for (const sim::Warning &warning : picture.radar)
    {
        remember(warning);
    }
    for (const sim::Warning &warning : picture.approach)
    {
        remember(warning);
    }
    m_rwrMemory.erase(std::remove_if(m_rwrMemory.begin(), m_rwrMemory.end(),
                                     [now](const RwrMemory &memory) { return now - memory.heardTime > kRwrHoldSeconds; }),
                      m_rwrMemory.end());
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
    const sim::Shot *shot = followedShot();
    const bool inFlight = shot != nullptr;
    const Fighter *jet = fighter();
    const bool fighterRole = m_playerRole == PlayerRole::Fighter;
    const bool fighterHud = fighterRole && jet != nullptr;
    const bool jetCamera = m_cameraMode == CameraMode::FIGHTER_JET;
    const sim::FireControl fire = fighterHud ? m_world->fireControl() : sim::FireControl{};
    if (fighterRole)
    {
        updateRwrMemory();
    }

    // ---- Projection --------------------------------------------------------
    ScreenProjector screen;
    screen.position = m_renderer->getCameraPosition();
    screen.forward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    screen.right = safeNormalize(m_renderer->getCameraRight(), glm::vec3(1.0f, 0.0f, 0.0f));
    screen.up = safeNormalize(m_renderer->getCameraUp(), glm::vec3(0.0f, 1.0f, 0.0f));
    screen.tanHalfFov = std::tan(glm::radians(m_renderer->getCameraFOV() * 0.5f));
    screen.aspect = width / std::max(height, 1.0f);
    screen.origin = origin;
    screen.width = width;
    screen.height = height;
    // Ranges are read from the round the player is watching or about to fire.
    const glm::vec3 rangeOrigin = focus != nullptr ? focus->getPosition() : screen.position;

    // ---- Aircraft markers ----------------------------------------------------
    if (m_showTargetInfo && !fighterRole)
    {
        // SAM sandbox: omniscient brackets on every aircraft.
        const Target *chaseSubject = m_cameraMode == CameraMode::FIGHTER_JET
                                         ? (trackedTarget != nullptr ? trackedTarget : findBestTarget())
                                         : nullptr;
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

            const Projected p = screen.project(target->getRenderPosition());
            if (p.onScreen)
            {
                const float half = ui::px(13.0f) + ui::px(12.0f) * std::clamp(900.0f / std::max(range, 1.0f), 0.0f, 1.0f);
                ui::drawCornerBrackets(drawList, p.screen, half, ui::toU32(colour), isLocked ? ui::px(2.0f) : ui::px(1.4f), 0.42f);
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
                const std::string label = std::string(idText) + "  " + rangeText;
                drawEdgeArrow(drawList, screen, p, ui::toU32(colour), label.c_str());
            }
        }
    }
    else if (m_showTargetInfo && fighterHud)
    {
        // The pilot's eyes: a small chevron over each aircraft inside visual
        // range and in the clear, with its range. Nothing off-screen.
        const glm::vec3 eye = jet->getPosition();
        for (const auto &target : aircraft)
        {
            if (!target->isActive())
            {
                continue;
            }
            const float range = glm::distance(target->getPosition(), eye);
            if (range > kVisualRangeM || !m_world->terrain().lineOfSight(eye, target->getPosition()))
            {
                continue;
            }
            const Projected p = screen.project(target->getRenderPosition());
            if (!p.onScreen)
            {
                continue;
            }
            ui::drawSpottingMark(drawList, p.screen, formatRange(range).c_str());
        }
    }

    // A marker on every round in the air, except the one being ridden.
    if (m_showTargetInfo)
    {
        for (const auto &flying : m_world->shots())
        {
            if (flying->id == m_followedShot && m_cameraMode == CameraMode::MISSILE)
            {
                continue;
            }
            // The fighter knows its own rounds (it launched them), not the opponent's.
            if (fighterRole && flying->team != sim::Team::Blue)
            {
                continue;
            }
            const Projected p = screen.project(flying->missile->getRenderPosition());
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

    // ---- Fighter camera: flight marks and fire control ------------------------
    if (fighterHud && jetCamera)
    {
        const glm::vec3 origin3 = jet->getRenderPosition();

        // Radar picture first, so the flight marks sit on top of it.
        drawRadarContacts(drawList, screen, *m_world);
        drawLockBox(drawList, screen, fire, time);
        drawSeekerCircle(drawList, screen, fire, origin3, time);

        // War Thunder's two marks: the circle is where the mouse asks the nose
        // to go, the cross is where the nose points now. The instructor flies
        // the cross onto the circle.
        const ImU32 shade = ui::toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.35f));
        const ImU32 ink = ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.92f));
        const Projected nose = screen.project(origin3 + jet->getRenderNose() * kMarkerRange);
        if (nose.onScreen)
        {
            const float arm = ui::px(9.0f);
            const float gap = ui::px(3.0f);
            const ImVec2 c = nose.screen;
            for (int pass = 0; pass < 2; ++pass)
            {
                const ImU32 colour = pass == 0 ? shade : ink;
                const float thickness = pass == 0 ? ui::px(3.4f) : ui::px(1.6f);
                drawList->AddLine(ImVec2(c.x - arm, c.y), ImVec2(c.x - gap, c.y), colour, thickness);
                drawList->AddLine(ImVec2(c.x + gap, c.y), ImVec2(c.x + arm, c.y), colour, thickness);
                drawList->AddLine(ImVec2(c.x, c.y - arm), ImVec2(c.x, c.y - gap), colour, thickness);
                drawList->AddLine(ImVec2(c.x, c.y + gap), ImVec2(c.x, c.y + arm), colour, thickness);
            }
        }

        const Projected aim = screen.project(origin3 + m_aimCamera.aimDirection() * kMarkerRange);
        if (aim.onScreen)
        {
            const float radius = ui::px(12.0f);
            const bool freeLook = m_aimCamera.isFreeLook();
            drawList->AddCircle(aim.screen, radius, shade, 40, ui::px(3.6f));
            drawList->AddCircle(aim.screen, radius, freeLook ? ui::toU32(ui::color::accent) : ink, 40, ui::px(1.7f));
        }
    }

    // ---- Mission status (top left) ---------------------------------------------
    float statusBottom = origin.y + margin;
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
        const float dotAlpha = pulsing ? 0.55f + 0.45f * pulse(time, 6.0f) : 1.0f;
        drawList->AddCircleFilled(ImVec2(x + ui::px(5.0f), y + ui::px(13.0f)), ui::px(4.5f), ui::toU32(stateColour, dotAlpha), 16);
        ui::drawTracked(drawList, font.display, ui::px(25.0f), ImVec2(x + ui::px(18.0f), y), ui::toU32(stateColour), state, 0.08f);

        std::string detail;
        if (fighterHud && !inFlight)
        {
            // On the rail the story is the shared picture: radar, then seeker.
            detail = fire.radar == sim::RadarMode::Track ? "Radar lock" : "Radar search";
            if (fire.weapon == sim::FighterWeapon::Fox2)
            {
                detail += std::string("  \xC2\xB7  Seeker ") + seekerStateLabel(fire.seeker.state);
            }
        }
        else
        {
            detail = std::string("Seeker ") + getMissileSeekerStateLabel();
            std::transform(detail.begin() + 7, detail.end(), detail.begin() + 7,
                           [](unsigned char c)
                           { return static_cast<char>(std::tolower(c)); });
            if (trackedTarget != nullptr && (inFlight || seekerUncaged()) && !fighterRole)
            {
                detail += "  \xC2\xB7  T" + std::to_string(targetIndex(trackedTarget));
            }
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
        statusBottom = y + ui::px(52.0f);
    }

    // ---- Camera selector (top centre) --------------------------------------------
    float belowSelectorY = origin.y + margin;
    const float usableWidth = width - hudRightInset();
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

    // ---- Threat banner (fighter, under the selector) --------------------------------
    if (fighterHud)
    {
        ui::RwrKind top = ui::RwrKind::Search;
        bool heard = false;
        for (const RwrMemory &memory : m_rwrMemory)
        {
            // Only what is being heard right now raises the banner.
            if (memory.heardTime < m_world->time() - 1.0e-9)
            {
                continue;
            }
            const ui::RwrKind kind = rwrKind(memory.kind);
            if (!heard || static_cast<int>(kind) > static_cast<int>(top))
            {
                top = kind;
            }
            heard = true;
        }
        const char *banner = nullptr;
        if (heard && (top == ui::RwrKind::Seeker || top == ui::RwrKind::Approach))
        {
            banner = "MISSILE";
        }
        else if (heard && top == ui::RwrKind::Launch)
        {
            banner = "LAUNCH";
        }
        if (banner != nullptr)
        {
            const float size = ui::px(18.0f);
            const float tracking = 0.22f;
            const float textW = ui::measureTracked(font.display, size, banner, tracking).x;
            const float chipW = textW + ui::px(36.0f);
            const float chipH = ui::px(34.0f);
            const ImVec2 chipMin(std::round(origin.x + (usableWidth - chipW) * 0.5f), belowSelectorY);
            const ImVec2 chipMax(chipMin.x + chipW, chipMin.y + chipH);
            const float flash = 0.55f + 0.45f * pulse(time, 2.0f * 3.14159265f * 3.0f);
            drawList->AddRectFilled(chipMin, chipMax, ui::toU32(ImVec4(0.30f, 0.04f, 0.04f, 0.72f)), chipH * 0.5f);
            drawList->AddRect(chipMin, chipMax, ui::toU32(ui::color::danger, 0.8f * flash), chipH * 0.5f, 0, ui::px(1.4f));
            ui::drawTracked(drawList, font.display, size, ImVec2(chipMin.x + ui::px(18.0f), chipMin.y + (chipH - size) * 0.5f - ui::px(1.0f)),
                            ui::toU32(ui::color::danger, flash), banner, tracking);
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
        const bool fighterStrip = fighterHud && !(inFlight && m_cameraMode == CameraMode::MISSILE);
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
            // Rounds of the selected weapon.
            std::snprintf(cells[5].value, sizeof(cells[5].value), "%d", std::max(fire.rounds, 0));
        }
        else
        {
            std::snprintf(cells[5].value, sizeof(cells[5].value), "%.1f", inFlight ? shot->flightTime : 0.0f);
        }

        const float cellWidth = ui::px(116.0f);
        const float stripHeight = ui::px(68.0f);
        const float stripWidth = cellWidth * 6.0f + ui::px(12.0f);
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

        // Weapon line above the strip, or the SAM's launch prompt.
        const Missile *ready = m_world->readyRound();
        const float bodySize = ui::px(14.5f);
        const float lineY = min.y - ui::px(38.0f);
        if (fighterHud)
        {
            // [B] weapon  rounds   status   [key] next step
            const char *weaponName = "Radar round";
            if (fire.weapon == sim::FighterWeapon::RadarRound)
            {
                if (const missilesim::fox3::Spec *fox3 = missilesim::fox3::find(m_world->fox3Id().c_str()))
                {
                    weaponName = fox3->displayName;
                }
            }
            else
            {
                weaponName = "Heat seeker";
                if (const missilesim::fox2::Spec *fox2 = missilesim::fox2::find(m_world->fox2Id().c_str()))
                {
                    weaponName = fox2->displayName;
                }
            }
            const WeaponLine status = weaponLine(fire);
            char count[16];
            std::snprintf(count, sizeof(count), "\xC3\x97%d", std::max(fire.rounds, 0));
            const float statusSize = ui::px(14.0f);
            const float statusTracking = 0.14f;
            const float capWidth = ui::px(22.0f) + ui::px(8.0f);
            const float nameWidth = textWidth(font.body, bodySize, weaponName);
            const float countWidth = textWidth(font.monoMedium, ui::px(13.5f), count);
            const float statusWidth = ui::measureTracked(font.display, statusSize, status.status, statusTracking).x;
            float hintWidth = 0.0f;
            if (status.hintKey != nullptr)
            {
                hintWidth = ui::px(28.0f) + std::max(ui::px(22.0f), textWidth(font.monoMedium, ui::px(12.5f), status.hintKey) + ui::px(14.0f)) +
                            ui::px(8.0f) + textWidth(font.body, bodySize, status.hintText);
            }
            const float dot = ui::px(14.0f);
            const float lineWidth = capWidth + nameWidth + ui::px(8.0f) + countWidth + ui::px(28.0f) + dot + statusWidth + hintWidth;
            float x = std::round(origin.x + (usableWidth - lineWidth) * 0.5f);
            x += ui::drawKeycap(drawList, ImVec2(x, lineY), "B") + ui::px(8.0f);
            drawList->AddText(font.body, bodySize, ImVec2(x, lineY + ui::px(2.5f)), ui::toU32(ui::color::text), weaponName);
            x += nameWidth + ui::px(8.0f);
            drawList->AddText(font.monoMedium, ui::px(13.5f), ImVec2(x, lineY + ui::px(3.5f)), ui::toU32(ui::color::textMuted), count);
            x += countWidth + ui::px(28.0f);
            drawList->AddCircleFilled(ImVec2(x + ui::px(4.0f), lineY + ui::px(11.0f)), ui::px(4.0f), ui::toU32(status.tone), 12);
            x += dot;
            ui::drawTracked(drawList, font.display, statusSize, ImVec2(x, lineY + ui::px(4.0f)), ui::toU32(status.tone), status.status,
                            statusTracking);
            x += statusWidth;
            if (status.hintKey != nullptr)
            {
                x += ui::px(28.0f);
                x += ui::drawKeycap(drawList, ImVec2(x, lineY), status.hintKey) + ui::px(8.0f);
                drawList->AddText(font.body, bodySize, ImVec2(x, lineY + ui::px(2.5f)), ui::toU32(ui::color::text), status.hintText);
            }
        }
        else if (ready != nullptr)
        {
            const char *launch = "Launch";
            const char *cue = "Seeker cue";
            const char *cueState = seekerUncaged() ? "ON" : "OFF";
            const float launchWidth = textWidth(font.body, bodySize, launch);
            const float cueWidth = textWidth(font.body, bodySize, cue);
            const float stateWidth = ui::measureTracked(font.display, ui::px(13.0f), cueState, 0.12f).x;
            const float promptWidth = ui::px(22.0f + 8.0f) + launchWidth + ui::px(28.0f) + ui::px(22.0f + 8.0f) + cueWidth + ui::px(8.0f) + stateWidth;
            float x = std::round(origin.x + (usableWidth - promptWidth) * 0.5f);
            x += ui::drawKeycap(drawList, ImVec2(x, lineY), "F") + ui::px(8.0f);
            drawList->AddText(font.body, bodySize, ImVec2(x, lineY + ui::px(2.5f)), ui::toU32(ui::color::text), launch);
            x += launchWidth + ui::px(28.0f);
            x += ui::drawKeycap(drawList, ImVec2(x, lineY), "R") + ui::px(8.0f);
            drawList->AddText(font.body, bodySize, ImVec2(x, lineY + ui::px(2.5f)), ui::toU32(ui::color::text), cue);
            x += cueWidth + ui::px(8.0f);
            ui::drawTracked(drawList, font.display, ui::px(13.0f), ImVec2(x, lineY + ui::px(4.0f)),
                            ui::toU32(seekerUncaged() ? ui::color::accent : ui::color::textFaint), cueState, 0.12f);
        }
        else
        {
            const char *status = "Reloading";
            const float statusWidth = textWidth(font.body, bodySize, status);
            const float x = std::round(origin.x + (usableWidth - statusWidth) * 0.5f);
            drawList->AddText(font.body, bodySize, ImVec2(x, lineY + ui::px(2.5f)), ui::toU32(ui::color::textMuted), status);
        }

        // Why the last launch was refused, above the prompt.
        if (m_launchNoticeTimer > 0.0f && m_launchNotice != sim::LaunchBlock::None)
        {
            std::string notice = sim::launchBlockMessage(m_launchNotice);
            std::transform(notice.begin(), notice.end(), notice.begin(),
                           [](unsigned char c)
                           { return static_cast<char>(std::toupper(c)); });
            const float alpha = std::clamp(m_launchNoticeTimer / 0.4f, 0.0f, 1.0f);
            const float noticeWidth = ui::measureTracked(font.display, ui::px(14.0f), notice.c_str(), 0.14f).x;
            ui::drawTracked(drawList, font.display, ui::px(14.0f),
                            ImVec2(std::round(origin.x + (usableWidth - noticeWidth) * 0.5f), min.y - ui::px(66.0f)),
                            ui::toU32(ui::color::danger, alpha), notice.c_str(), 0.14f);
        }
        else if (m_shotEndNoticeTimer > 0.0f && !m_shotEndNotice.empty())
        {
            const std::string notice = "Shot ended: " + m_shotEndNotice;
            const float alpha = std::clamp(m_shotEndNoticeTimer / 0.4f, 0.0f, 1.0f);
            const float noticeWidth = ui::measureTracked(font.display, ui::px(14.0f), notice.c_str(), 0.14f).x;
            ui::drawTracked(drawList, font.display, ui::px(14.0f),
                            ImVec2(std::round(origin.x + (usableWidth - noticeWidth) * 0.5f), min.y - ui::px(66.0f)),
                            ui::toU32(ui::color::text, alpha), notice.c_str(), 0.14f);
        }
    }

    // ---- Key hints (bottom left, fade after the first seconds) --------------------------
    {
        const float alpha = 1.0f - std::clamp((m_hud.engagementTime - kHintsVisibleSeconds) / 1.5f, 0.0f, 1.0f);
        if (alpha > 0.001f)
        {
            static const char *const samHints[][2] = {{"Tab", "Controls"}, {"V", "Camera"}, {"H", "Hide HUD"}, {"Esc", "Menu"}};
            static const char *const fighterHints[][2] = {{"Mouse", "Aim"},          {"RMB", "Free look"},     {"Shift", "Throttle up"},
                                                          {"Ctrl", "Throttle down"}, {"X", "Afterburner"},     {"B", "Weapon"},
                                                          {"T", "Radar lock"},       {"R", "Uncage seeker"},   {"Y", "Radar range"},
                                                          {"Z", "Chaff"},            {"Space", "Flares"},      {"N", "Hostile radar"},
                                                          {"F", "Fire"},             {"V", "Camera"},          {"Esc", "Menu"}};
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

    // ---- RWR (right side) ------------------------------------------------------------
    // The fighter always carries its receivers. The SAM sandbox shows the
    // chased aircraft's missile-approach warner from its own camera.
    const Target *subject = trackedTarget != nullptr ? trackedTarget : findBestTarget();
    const bool samThreat = !fighterRole && jetCamera && subject != nullptr && subject->hasThreatAssessment();
    if (fighterHud || (!fighterRole && jetCamera))
    {
        const float radius = ui::px(74.0f);
        const ImVec2 centre(origin.x + width - margin - hudRightInset() - radius - ui::px(10.0f),
                            std::min(origin.y + height * 0.5f + ui::px(40.0f), stripTop - radius - ui::px(90.0f)));
        ui::RwrView view;
        view.time = time;
        char detail[64];
        detail[0] = '\0';
        if (fighterHud)
        {
            const double now = m_world->time();
            for (const RwrMemory &memory : m_rwrMemory)
            {
                ui::RwrThreat threat;
                threat.kind = rwrKind(memory.kind);
                threat.azimuthRad = memory.azimuthRad;
                const double silence = std::max(0.0, now - memory.heardTime);
                threat.fade = silence <= 1.0e-9 ? 1.0f : static_cast<float>(std::clamp(1.0 - silence / kRwrHoldSeconds, 0.0, 1.0)) * 0.7f;
                view.threats.push_back(threat);
            }
            view.chaff = m_world->chaffRemaining();
            view.flares = m_world->flaresRemaining();
        }
        else
        {
            // The chased aircraft carries an approach warner, not a radar receiver.
            view.caption = "MAWS";
        }
        if (samThreat)
        {
            // The chased aircraft's frame: heading level, right wing to the right.
            const glm::vec3 forward = safeNormalize(subject->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
            const glm::vec3 flatForward = safeNormalize(glm::vec3(forward.x, 0.0f, forward.z), glm::vec3(0.0f, 0.0f, 1.0f));
            const glm::vec3 rightWing = safeNormalize(glm::cross(flatForward, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(-1.0f, 0.0f, 0.0f));
            const glm::vec3 incoming = subject->getThreatMissilePosition() - subject->getPosition();
            ui::RwrThreat threat;
            threat.kind = ui::RwrKind::Approach;
            threat.azimuthRad = std::atan2(glm::dot(incoming, rightWing), glm::dot(incoming, flatForward));
            view.threats.push_back(threat);
            std::snprintf(detail, sizeof(detail), "%s  \xC2\xB7  TCA %.1f s", formatRange(subject->getThreatDistance()).c_str(),
                          subject->getThreatTimeToClosestApproach());
            view.detail = detail;
        }
        ui::drawRwrScope(drawList, centre, radius, view);
    }

    // ---- Radar scope (fighter, top left under the status) ------------------------------
    if (fighterHud)
    {
        ui::RadarScopeView view;
        view.rangeScaleM = m_radarRangeScaleM;
        view.azimuthHalfRad = m_world->radarVolume().azimuthHalfRad;
        const sim::BeamPoint &beam = m_world->playerRadar().beam();
        // Scan and scope share one convention: positive azimuth is the right wing.
        view.beamAzimuthRad = beam.azimuthRad;
        view.beamElevationRad = beam.elevationRad;
        view.singleTargetTrack = fire.radar == sim::RadarMode::Track;
        view.bar = beam.bar;
        view.barCount = std::max(m_world->radarVolume().bars, 1);
        view.modeLabel = view.singleTargetTrack ? "TRACK" : "SEARCH";
        // The scope speaks for the radar round; the heat seeker has its own line.
        if (fire.weapon == sim::FighterWeapon::RadarRound && fire.clearance != sim::LaunchBlock::None &&
            fire.clearance != sim::LaunchBlock::NoLaunchQuality)
        {
            view.launchBlock = sim::launchBlockMessage(fire.clearance);
        }
        view.hasLock = fire.lock.valid;
        view.lockRangeM = fire.lock.rangeM;
        view.lockClosingMps = fire.lock.closingMps;

        // Every contact is placed in the jet's own sensor frame.
        sim::SensorBody ownship;
        ownship.position = jet->getPosition();
        ownship.forward = jet->getNose();
        ownship.up = jet->getUp();
        const sim::SensorAxes axes = sim::sensorAxes(ownship);
        constexpr float kTrendSeconds = 4.0f;
        const glm::vec3 ownMotion = jet->getVelocity() * kTrendSeconds;
        const double now = m_world->time();
        for (const sim::TrackEstimate &track : m_world->playerRadar().tracks().tracks())
        {
            if (track.life == sim::TrackLife::Lost)
            {
                continue;
            }
            const glm::vec3 position = track.position + track.velocity * static_cast<float>(std::max(0.0, now - track.time));
            const sim::SensorBearing bearing = sim::bearingFrom(axes, ownship.position, position);
            // Relative motion, so a contact closing on the nose heads down the scope.
            const sim::SensorBearing trend =
                sim::bearingFrom(axes, ownship.position + ownMotion, position + track.velocity * kTrendSeconds);
            ui::ScopeContact contact;
            contact.trackId = track.id.value;
            contact.rangeM = bearing.rangeM;
            contact.azimuthRad = bearing.azimuthRad;
            contact.hasTrend = true;
            contact.trendRangeM = trend.rangeM;
            contact.trendAzimuthRad = trend.azimuthRad;
            contact.ageSeconds = std::max(0.0, now - track.lastMeasurementTime);
            contact.designated = track.id == m_world->designatedRadarTrack();
            contact.launchQuality = sim::isLaunchQuality(track, now, m_world->radarLaunchAgeLimit());
            switch (track.life)
            {
            case sim::TrackLife::Confirmed:
                contact.life = ui::ScopeLife::Confirmed;
                break;
            case sim::TrackLife::Coasting:
                contact.life = ui::ScopeLife::Coasting;
                break;
            case sim::TrackLife::Lost:
                contact.life = ui::ScopeLife::Lost;
                break;
            case sim::TrackLife::Tentative:
                contact.life = ui::ScopeLife::Tentative;
                break;
            }
            view.contacts.push_back(contact);
        }

        const float scopeSize = std::min(ui::px(260.0f), height * 0.36f);
        ui::drawRadarScope(drawList, ImVec2(origin.x + margin, statusBottom + ui::px(10.0f)), scopeSize, view);
    }
}
