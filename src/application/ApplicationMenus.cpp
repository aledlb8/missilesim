// Front-end screens: title screen (over the live scene), pause menu, settings,
// and the screen-to-screen fades. All drawn with the ui:: widgets and tokens.
#include "Application.h"
#include "ApplicationDetail.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

#include <glm/gtx/norm.hpp>

#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#ifndef MISSILESIM_VERSION
#define MISSILESIM_VERSION "dev"
#endif

using missilesim::application::detail::safeNormalize;
namespace ui = missilesim::ui;

namespace
{
    struct KeyBinding
    {
        const char *keys;
        const char *action;
    };

    constexpr KeyBinding kFlightBindings[] = {
        {"Mouse", "Fighter: point where to fly; the instructor flies there"},
        {"W|S", "Fighter: pitch down / up assist (overrides the instructor)"},
        {"A|D", "Fighter: roll assist"},
        {"Q|E", "Fighter: rudder assist"},
        {"Shift|Ctrl", "Fighter: throttle up / down (past 100% is afterburner)"},
        {"X", "Fighter: afterburner on / off"},
        {"F", "Launch the missile"},
        {"B", "Fighter: heat seeker or radar round"},
        {"T", "Fighter: radar lock, nearest the nose first; again for the next, then search"},
        {"R", "Uncage the heat seeker (a radar lock slaves it on its own)"},
        {"Y", "Fighter: radar scope range, 10, 20 or 40 km"},
        {"Z", "Fighter: release chaff (hold for a stream)"},
        {"Space", "Fighter: release flares, a pair at a time (hold for a stream)"},
        {"N", "Fighter: hostile radar on / off"},
        {"G", "Rearm the wingtip rails"},
        {"Enter", "Pause or resume the simulation"},
    };
    constexpr KeyBinding kCameraBindings[] = {
        {"V", "Cycle camera: free, missile, fighter"},
        {"C", "Frame the engagement; hold for free look while flying"},
        {"RMB", "Hold and drag to look around (the fighter keeps its course)"},
        {"Wheel", "Fighter: camera distance"},
        {"W|A|S|D", "Move the free camera"},
        {"Space|Ctrl", "Free camera up / down"},
        {"Shift", "Move the camera faster"},
    };
    constexpr KeyBinding kInterfaceBindings[] = {
        {"Tab", "Show or hide the control panel"},
        {"H", "Show or hide the HUD"},
        {"F11", "Toggle fullscreen"},
        {"F12", "Save a screenshot to the screenshots folder"},
        {"Esc", "Open the menu"},
    };

    constexpr ImGuiWindowFlags kHostFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings |
                                            ImGuiWindowFlags_NoBackground |
                                            ImGuiWindowFlags_NoScrollWithMouse |
                                            ImGuiWindowFlags_NoNav;

    float easeOutCubic(float t)
    {
        t = std::clamp(t, 0.0f, 1.0f);
        const float inverse = 1.0f - t;
        return 1.0f - inverse * inverse * inverse;
    }

    // Begins an invisible window covering the whole viewport that screens draw into.
    bool beginHost(const char *name, bool bringToFront)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->Pos);
        ImGui::SetNextWindowSize(viewport->Size);
        if (bringToFront)
        {
            ImGui::SetNextWindowFocus();
        }
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        const ImGuiWindowFlags flags = kHostFlags | (bringToFront ? 0 : ImGuiWindowFlags_NoBringToFrontOnFocus);
        const bool open = ImGui::Begin(name, nullptr, flags);
        ImGui::PopStyleVar(2);
        return open;
    }

    // Up/Down (or W/S) move the selection; Enter/Space activate it.
    // Returns the activated index or -1.
    int handleMenuKeyboard(int &selection, int count)
    {
        if (ImGui::GetIO().WantTextInput)
        {
            return -1;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow) || ImGui::IsKeyPressed(ImGuiKey_S))
        {
            selection = (selection + 1) % count;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow) || ImGui::IsKeyPressed(ImGuiKey_W))
        {
            selection = (selection + count - 1) % count;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_Space, false))
        {
            return selection;
        }
        return -1;
    }

    // Draws a vertical list of menu items; returns the clicked/activated index or -1.
    int drawMenuList(const char *const *labels, int count, int &selection)
    {
        const bool mouseMoved = ImGui::GetIO().MouseDelta.x != 0.0f || ImGui::GetIO().MouseDelta.y != 0.0f;
        int activated = handleMenuKeyboard(selection, count);
        for (int i = 0; i < count; ++i)
        {
            bool hovered = false;
            if (ui::menuItem(labels[i], selection == i, &hovered))
            {
                activated = i;
            }
            if (hovered && mouseMoved)
            {
                selection = i;
            }
        }
        return activated;
    }

    void drawCard(ImDrawList *drawList, ImVec2 min, ImVec2 max, float alpha)
    {
        const float rounding = ui::px(12.0f);
        for (int i = 1; i <= 5; ++i)
        {
            const float spread = ui::px(4.0f) * static_cast<float>(i);
            drawList->AddRectFilled(ImVec2(min.x - spread, min.y - spread * 0.5f),
                                    ImVec2(max.x + spread, max.y + spread * 1.5f),
                                    ui::toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.045f), alpha), rounding + spread);
        }
        drawList->AddRectFilled(min, max, ui::toU32(ImVec4(0.051f, 0.063f, 0.082f, 0.985f), alpha), rounding);
        drawList->AddRect(min, max, ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.09f), alpha), rounding);
    }

    void drawBindings(const char *title, const KeyBinding *bindings, size_t count)
    {
        ui::sectionLabel(title);
        for (size_t i = 0; i < count; ++i)
        {
            ui::keyHintRow(bindings[i].keys, bindings[i].action);
        }
    }

    const char *settingsPageName(int page)
    {
        static const char *const names[] = {"Display", "Graphics", "Audio", "Controls"};
        return names[std::clamp(page, 0, 3)];
    }
}

bool Application::gameplayInputEnabled() const
{
    return m_screen == Screen::Playing && m_overlay == Overlay::None;
}

void Application::beginScreenFade(float duration)
{
    m_fadeAlpha = 1.0f;
    m_fadeDuration = std::max(duration, 0.01f);
}

void Application::startEngagement()
{
    m_screen = Screen::Playing;
    m_overlay = Overlay::None;
    m_isPaused = false;
    m_screenTime = 0.0f;
    m_hud = HudTracker{};

    // The targets have been flying around behind the title screen; start the
    // engagement from a fresh, framed setup with a new seed.
    restartWorld();
    if (m_playerRole == PlayerRole::Fighter)
    {
        // Straight into the cockpit view: mouse aim from the first frame.
        setCameraMode(CameraMode::FIGHTER_JET);
        resetAimCamera();
    }
    else
    {
        setCameraMode(CameraMode::FREE, true);
    }
    if (m_renderer)
    {
        m_renderer->setCameraFOV(m_savedCameraFOV);
    }
    beginScreenFade(0.55f);
}

void Application::returnToTitle()
{
    releaseMouseCameraCapture();
    m_screen = Screen::Title;
    m_overlay = Overlay::None;
    m_isPaused = false;
    m_menuSelection = 0;
    m_screenTime = 0.0f;
    m_cameraMode = CameraMode::FREE;
    m_hud = HudTracker{};
    resetChaseCameraState();
    restartWorld();
    beginScreenFade(0.7f);
}

void Application::openPauseMenu()
{
    releaseMouseCameraCapture();
    m_pausedBeforeMenu = m_isPaused;
    m_isPaused = true;
    m_overlay = Overlay::Pause;
    m_overlayTime = 0.0f;
    m_menuSelection = 0;
}

void Application::closePauseMenu()
{
    m_overlay = Overlay::None;
    m_isPaused = m_pausedBeforeMenu;
}

void Application::openSettings(SettingsPage page)
{
    m_overlayReturn = m_overlay;
    m_overlay = Overlay::Settings;
    m_settingsPage = page;
    m_overlayTime = 0.0f;
    m_uiScaleSliderHeld = false;
}

void Application::closeSettings()
{
    m_overlay = m_overlayReturn;
    m_overlayReturn = Overlay::None;
    m_overlayTime = 0.0f;
}

void Application::updateTitleCamera(float deltaTime)
{
    if (!m_renderer)
    {
        return;
    }

    Target *subject = nullptr;
    for (const auto &target : targets())
    {
        if (target && target->isActive())
        {
            subject = target.get();
            break;
        }
    }
    if (subject == nullptr)
    {
        return;
    }

    // Fly alongside the lead fighter on a slow orbit. Its heading is smoothed so
    // AI turns sweep the shot gently instead of whipping it around.
    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 heading = safeNormalize(subject->getVelocity(), m_titleCameraForward);
    const float headingBlend = 1.0f - std::exp(-1.2f * deltaTime);
    m_titleCameraForward = safeNormalize(glm::mix(m_titleCameraForward, heading, headingBlend), heading);
    m_titleOrbitAngle += deltaTime * 0.07f;

    const glm::vec3 forward = m_titleCameraForward;
    const glm::vec3 right = safeNormalize(glm::cross(forward, worldUp), glm::vec3(1.0f, 0.0f, 0.0f));
    const float distance = std::clamp(subject->getRadius() * 6.5f, 24.0f, 60.0f);
    const glm::vec3 subjectPosition = subject->getRenderPosition();

    glm::vec3 cameraPosition = subjectPosition +
                               (-std::cos(m_titleOrbitAngle) * forward + std::sin(m_titleOrbitAngle) * right) * distance +
                               worldUp * (distance * 0.18f);
    if (physics())
    {
        cameraPosition.y = std::max(cameraPosition.y, physics()->getTerrain().heightAt(cameraPosition) + 6.0f);
    }

    // Compose the aircraft on the right third, clear of the menu on the left.
    const glm::vec3 viewDirection = safeNormalize(subjectPosition - cameraPosition, forward);
    const glm::vec3 viewRight = safeNormalize(glm::cross(viewDirection, worldUp), right);
    const glm::vec3 lookTarget = subjectPosition - viewRight * (distance * 0.32f);

    m_renderer->setCameraPosition(cameraPosition);
    m_renderer->setCameraTarget(lookTarget);
}

void Application::renderTitleScreen()
{
    if (!beginHost("##title", false))
    {
        ImGui::End();
        return;
    }

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    const float width = viewport->Size.x;
    const float height = viewport->Size.y;
    const ImVec2 origin = viewport->Pos;
    ImDrawList *drawList = ImGui::GetWindowDrawList();

    // Scrims: a left-hand wash for legibility and a floor fade.
    const ImU32 clear = ui::toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    const ImU32 shade = ui::toU32(ImVec4(0.012f, 0.016f, 0.024f, 0.80f));
    const ImU32 floorShade = ui::toU32(ImVec4(0.012f, 0.016f, 0.024f, 0.55f));
    drawList->AddRectFilledMultiColor(origin, ImVec2(origin.x + width * 0.62f, origin.y + height), shade, clear, clear, shade);
    drawList->AddRectFilledMultiColor(ImVec2(origin.x, origin.y + height * 0.68f), ImVec2(origin.x + width, origin.y + height),
                                      clear, clear, floorShade, floorShade);

    const bool menuVisible = m_overlay == Overlay::None;
    const float intro = easeOutCubic((m_screenTime - 0.35f) / 1.1f);
    const float alpha = intro * (menuVisible ? 1.0f : 0.0f);
    if (alpha <= 0.001f)
    {
        ImGui::End();
        return;
    }
    const float slide = (1.0f - intro) * ui::px(28.0f);

    const float left = std::round(origin.x + std::max(ui::px(72.0f), width * 0.075f));
    const char *const items[] = {"START ENGAGEMENT", "SETTINGS", "CONTROLS", "QUIT"};
    const int itemCount = static_cast<int>(std::size(items));
    const float itemHeight = ui::px(46.0f) + ImGui::GetStyle().ItemSpacing.y;
    const float blockHeight = ui::px(34.0f + 116.0f + 30.0f + 60.0f) + itemHeight * itemCount;
    float y = std::round(origin.y + (height - blockHeight) * 0.5f);

    // Eyebrow: accent rule + tracked caps.
    drawList->AddRectFilled(ImVec2(left + slide, y + ui::px(7.0f)), ImVec2(left + slide + ui::px(28.0f), y + ui::px(9.0f)),
                            ui::toU32(ui::color::accent, alpha));
    ui::drawTracked(drawList, ui::fonts().displayMedium, ui::px(15.0f), ImVec2(left + slide + ui::px(40.0f), y),
                    ui::toU32(ui::color::textMuted, alpha), "SURFACE-TO-AIR ENGAGEMENT SIMULATOR", 0.22f);
    y += ui::px(34.0f);

    // Wordmark.
    ui::drawTracked(drawList, ui::fonts().display, ui::px(104.0f), ImVec2(left - ui::px(4.0f) + slide * 1.4f, y),
                    ui::toU32(ui::color::text, alpha), "MISSILESIM", 0.012f);
    y += ui::px(116.0f);

    drawList->AddText(ui::fonts().body, ui::px(19.0f), ImVec2(left + slide * 1.8f, y),
                      ui::toU32(ui::color::textMuted, alpha),
                      "Guided missiles against evasive fighters, with real flight physics.");
    y += ui::px(30.0f + 60.0f);

    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
    ImGui::SetCursorScreenPos(ImVec2(left - ui::px(18.0f) + slide * 2.2f, y));
    ImGui::BeginChild("##title-menu", ImVec2(ui::px(460.0f), itemHeight * itemCount), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    const int activated = drawMenuList(items, itemCount, m_menuSelection);
    ImGui::EndChild();
    ImGui::PopStyleVar();

    // Footer: version and the two keys worth knowing up front.
    const float footerY = origin.y + height - ui::px(52.0f);
    char version[48];
    std::snprintf(version, sizeof(version), "v%s", MISSILESIM_VERSION);
    drawList->AddText(ui::fonts().mono, ui::px(13.0f), ImVec2(left, footerY + ui::px(4.0f)),
                      ui::toU32(ui::color::textFaint, alpha), version);

    const char *hints[][2] = {{"Enter", "Select"}, {"F11", "Fullscreen"}};
    float hintX = origin.x + width - std::max(ui::px(72.0f), width * 0.075f);
    for (int i = static_cast<int>(std::size(hints)) - 1; i >= 0; --i)
    {
        const float labelWidth = ui::fonts().body->CalcTextSizeA(ui::px(14.5f), FLT_MAX, 0.0f, hints[i][1]).x;
        hintX -= labelWidth;
        drawList->AddText(ui::fonts().body, ui::px(14.5f), ImVec2(hintX, footerY + ui::px(2.0f)),
                          ui::toU32(ui::color::textMuted, alpha), hints[i][1]);
        const float capWidth = ui::fonts().monoMedium->CalcTextSizeA(ui::px(12.5f), FLT_MAX, 0.0f, hints[i][0]).x + ui::px(14.0f);
        hintX -= std::max(capWidth, ui::px(22.0f)) + ui::px(8.0f);
        ui::drawKeycap(drawList, ImVec2(hintX, footerY), hints[i][0], alpha);
        hintX -= ui::px(28.0f);
    }

    ImGui::End();

    switch (activated)
    {
    case 0:
        startEngagement();
        break;
    case 1:
        openSettings(SettingsPage::Display);
        break;
    case 2:
        openSettings(SettingsPage::Controls);
        break;
    case 3:
        glfwSetWindowShouldClose(m_window, GLFW_TRUE);
        break;
    default:
        break;
    }
}

void Application::renderMenuOverlays()
{
    const bool escape = ImGui::IsKeyPressed(ImGuiKey_Escape, false) && !ImGui::GetIO().WantTextInput;
    if (m_overlay == Overlay::Settings)
    {
        if (escape)
        {
            closeSettings();
        }
    }
    else if (m_overlay == Overlay::Pause)
    {
        if (escape)
        {
            closePauseMenu();
        }
    }
    else if (escape && m_screen == Screen::Playing)
    {
        openPauseMenu();
    }

    switch (m_overlay)
    {
    case Overlay::Pause:
        renderPauseMenu();
        break;
    case Overlay::Settings:
        renderSettingsScreen();
        break;
    case Overlay::None:
        break;
    }
}

void Application::renderPauseMenu()
{
    const bool firstFrame = m_overlayTime <= 0.0f;
    if (!beginHost("##pause", firstFrame))
    {
        ImGui::End();
        return;
    }

    const float appear = easeOutCubic(m_overlayTime / 0.25f);
    ui::dimBackground(appear);

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    const char *const items[] = {"RESUME", "RESTART ENGAGEMENT", "SETTINGS", "CONTROLS", "MAIN MENU", "QUIT TO DESKTOP"};
    const int itemCount = static_cast<int>(std::size(items));
    const float itemHeight = ui::px(46.0f) + ImGui::GetStyle().ItemSpacing.y;
    const float columnWidth = ui::px(420.0f);
    const float blockHeight = ui::px(64.0f + 36.0f) + itemHeight * itemCount;
    const float left = std::round(viewport->Pos.x + (viewport->Size.x - columnWidth) * 0.5f);
    float y = std::round(viewport->Pos.y + (viewport->Size.y - blockHeight) * 0.5f + (1.0f - appear) * ui::px(14.0f));

    ui::drawTracked(drawList, ui::fonts().display, ui::px(56.0f), ImVec2(left + ui::px(18.0f), y),
                    ui::toU32(ui::color::text, appear), "PAUSED", 0.08f);
    y += ui::px(64.0f);
    drawList->AddRectFilled(ImVec2(left + ui::px(18.0f), y), ImVec2(left + ui::px(46.0f), y + ui::px(2.0f)),
                            ui::toU32(ui::color::accent, appear));
    y += ui::px(36.0f);

    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, appear);
    ImGui::SetCursorScreenPos(ImVec2(left, y));
    ImGui::BeginChild("##pause-menu", ImVec2(columnWidth, itemHeight * itemCount), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    const int activated = drawMenuList(items, itemCount, m_menuSelection);
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::End();

    switch (activated)
    {
    case 0:
        closePauseMenu();
        break;
    case 1:
        restartWorld();
        if (m_playerRole == PlayerRole::Fighter)
        {
            setCameraMode(CameraMode::FIGHTER_JET);
            resetAimCamera();
        }
        else
        {
            setCameraMode(CameraMode::FREE, true);
        }
        m_pausedBeforeMenu = false;
        closePauseMenu();
        beginScreenFade(0.4f);
        break;
    case 2:
        openSettings(SettingsPage::Display);
        break;
    case 3:
        openSettings(SettingsPage::Controls);
        break;
    case 4:
        returnToTitle();
        break;
    case 5:
        glfwSetWindowShouldClose(m_window, GLFW_TRUE);
        break;
    default:
        break;
    }
}

void Application::renderSettingsScreen()
{
    const bool firstFrame = m_overlayTime <= 0.0f;
    if (!beginHost("##settings", firstFrame))
    {
        ImGui::End();
        return;
    }

    const float appear = easeOutCubic(m_overlayTime / 0.22f);
    ui::dimBackground(appear);

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    const float cardWidth = std::min(ui::px(920.0f), viewport->Size.x - ui::px(48.0f));
    const float cardHeight = std::min(ui::px(640.0f), viewport->Size.y - ui::px(48.0f));
    const ImVec2 cardMin(std::round(viewport->Pos.x + (viewport->Size.x - cardWidth) * 0.5f),
                         std::round(viewport->Pos.y + (viewport->Size.y - cardHeight) * 0.5f + (1.0f - appear) * ui::px(18.0f)));
    const ImVec2 cardMax(cardMin.x + cardWidth, cardMin.y + cardHeight);
    drawCard(drawList, cardMin, cardMax, appear);

    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, appear);

    // Header.
    const float padding = ui::px(32.0f);
    const float headerHeight = ui::px(84.0f);
    ui::drawTracked(drawList, ui::fonts().display, ui::px(30.0f), ImVec2(cardMin.x + padding, cardMin.y + ui::px(28.0f)),
                    ui::toU32(ui::color::text), "SETTINGS", 0.1f);
    {
        const char *backLabel = "Back";
        const float labelWidth = ui::fonts().body->CalcTextSizeA(ui::px(14.5f), FLT_MAX, 0.0f, backLabel).x;
        const float labelX = cardMax.x - padding - labelWidth;
        const float rowY = cardMin.y + ui::px(34.0f);
        drawList->AddText(ui::fonts().body, ui::px(14.5f), ImVec2(labelX, rowY + ui::px(2.0f)), ui::toU32(ui::color::textMuted), backLabel);
        const float capWidth = ui::fonts().monoMedium->CalcTextSizeA(ui::px(12.5f), FLT_MAX, 0.0f, "Esc").x + ui::px(14.0f);
        ui::drawKeycap(drawList, ImVec2(labelX - capWidth - ui::px(8.0f), rowY), "Esc");
    }
    drawList->AddLine(ImVec2(cardMin.x, cardMin.y + headerHeight), ImVec2(cardMax.x, cardMin.y + headerHeight),
                      ui::toU32(ui::color::hairline));

    // Navigation column.
    const float navWidth = ui::px(200.0f);
    const float footerHeight = ui::px(76.0f);
    const float bodyTop = cardMin.y + headerHeight + ui::px(20.0f);
    const float bodyBottom = cardMax.y - footerHeight - ui::px(8.0f);
    int page = static_cast<int>(m_settingsPage);
    for (int i = 0; i < 4; ++i)
    {
        const float itemHeight = ui::px(40.0f);
        const ImVec2 itemMin(cardMin.x + ui::px(16.0f), bodyTop + itemHeight * static_cast<float>(i));
        ImGui::SetCursorScreenPos(itemMin);
        ImGui::PushID(i);
        if (ImGui::InvisibleButton("##nav", ImVec2(navWidth - ui::px(16.0f), itemHeight)))
        {
            page = i;
        }
        const bool hovered = ImGui::IsItemHovered();
        if (hovered)
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }
        const float selected = ui::animate(ImGui::GetItemID(), page == i ? 1.0f : (hovered ? 0.35f : 0.0f), 16.0f);
        ImGui::PopID();

        const ImVec2 itemMax(itemMin.x + navWidth - ui::px(16.0f), itemMin.y + itemHeight - ui::px(4.0f));
        if (selected > 0.001f)
        {
            drawList->AddRectFilled(itemMin, itemMax, ui::toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.06f), selected), ui::px(6.0f));
        }
        if (page == i)
        {
            drawList->AddRectFilled(ImVec2(itemMin.x, itemMin.y + ui::px(9.0f)), ImVec2(itemMin.x + ui::px(3.0f), itemMax.y - ui::px(9.0f)),
                                    ui::toU32(ui::color::accent), ui::px(1.5f));
        }
        const float size = ui::px(ui::type::body + 0.5f);
        const ImVec4 textColour = page == i ? ui::color::text : (hovered ? ui::withAlpha(ui::color::text, 0.8f) : ui::color::textMuted);
        drawList->AddText(ui::fonts().medium, size,
                          ImVec2(itemMin.x + ui::px(18.0f), itemMin.y + (itemHeight - ui::px(4.0f) - size) * 0.5f),
                          ui::toU32(textColour), settingsPageName(i));
    }
    m_settingsPage = static_cast<SettingsPage>(page);
    drawList->AddLine(ImVec2(cardMin.x + navWidth + ui::px(12.0f), bodyTop),
                      ImVec2(cardMin.x + navWidth + ui::px(12.0f), bodyBottom), ui::toU32(ui::color::hairline));

    // Page content.
    const float contentX = cardMin.x + navWidth + ui::px(40.0f);
    ImGui::SetCursorScreenPos(ImVec2(contentX, bodyTop - ui::px(6.0f)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::BeginChild("##settings-page", ImVec2(cardMax.x - padding - contentX, bodyBottom - bodyTop + ui::px(6.0f)),
                      ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleVar();
    renderSettingsPage(m_settingsPage);
    ImGui::EndChild();

    // Footer.
    const float footerTop = cardMax.y - footerHeight;
    drawList->AddLine(ImVec2(cardMin.x, footerTop), ImVec2(cardMax.x, footerTop), ui::toU32(ui::color::hairline));
    drawList->AddText(ui::fonts().body, ui::px(14.5f), ImVec2(cardMin.x + padding, footerTop + (footerHeight - ui::px(14.5f)) * 0.5f),
                      ui::toU32(ui::color::textFaint), "Changes apply immediately and are saved automatically.");
    const float doneWidth = ui::px(120.0f);
    ImGui::SetCursorScreenPos(ImVec2(cardMax.x - padding - doneWidth, footerTop + (footerHeight - ui::px(36.0f)) * 0.5f));
    const bool done = ui::button("DONE", ui::ButtonStyle::Primary, doneWidth);

    ImGui::PopStyleVar();
    ImGui::End();

    if (done)
    {
        closeSettings();
    }
}

void Application::renderSettingsPage(SettingsPage page)
{
    switch (page)
    {
    case SettingsPage::Display:
    {
        ui::sectionLabel("WINDOW");
        const char *const modes[] = {"Windowed", "Borderless", "Fullscreen"};
        int mode = static_cast<int>(m_displayMode);
        if (ui::segmentedRow("Display mode", &mode, modes, 3,
                             "Borderless covers the screen without changing its resolution\n"
                             "and switches apps instantly. F11 toggles it."))
        {
            setDisplayMode(static_cast<DisplayMode>(mode));
        }
        bool vsync = m_vsyncEnabled;
        if (ui::toggleRow("Vertical sync", &vsync, "Caps the frame rate to the display to avoid tearing."))
        {
            setVsyncEnabled(vsync);
        }

        ui::sectionLabel("INTERFACE");
        // Applied on release: rescaling while dragging would move the slider
        // under the cursor. Snaps to 5% steps.
        if (!m_uiScaleSliderHeld)
        {
            m_pendingUiScalePercent = std::round(m_uiScale * 100.0f);
        }
        ui::sliderRow("Interface size", &m_pendingUiScalePercent, 80.0f, 150.0f, "%.0f%%",
                      "Scales all menus, panels and the HUD on top of the Windows display scale.");
        m_uiScaleSliderHeld = ImGui::IsItemActive();
        m_pendingUiScalePercent = std::round(m_pendingUiScalePercent / 5.0f) * 5.0f;
        if (!m_uiScaleSliderHeld && std::abs(m_pendingUiScalePercent - m_uiScale * 100.0f) > 0.5f)
        {
            m_uiScale = m_pendingUiScalePercent / 100.0f;
        }
        ui::toggleRow("Show HUD", &m_hudVisible, "Flight data, target markers and messages. H toggles it.");
        ui::toggleRow("Target labels", &m_showTargetInfo, "Range and state beside each aircraft.");

        ui::sectionLabel("CAMERA");
        if (m_renderer)
        {
            float fov = m_savedCameraFOV;
            if (ui::sliderRow("Field of view", &fov, 30.0f, 100.0f, "%.0f\xC2\xB0"))
            {
                m_renderer->setCameraFOV(fov);
                m_savedCameraFOV = fov;
            }
        }
        break;
    }
    case SettingsPage::Graphics:
    {
        if (!m_renderer || !m_renderer->hasPBR())
        {
            ImGui::TextDisabled("Advanced lighting is unavailable on this graphics driver.");
            break;
        }
        ui::sectionLabel("IMAGE");
        float exposure = m_renderer->getPBRExposure();
        if (ui::sliderRow("Exposure", &exposure, 0.1f, 5.0f, "%.2f"))
        {
            m_renderer->setPBRExposure(exposure);
        }
        float bloomStrength = m_renderer->getPBRBloomStrength();
        if (ui::sliderRow("Bloom strength", &bloomStrength, 0.0f, 0.2f, "%.3f"))
        {
            m_renderer->setPBRBloomStrength(bloomStrength);
        }
        int bloomPasses = m_renderer->getPBRBloomPasses();
        if (ui::sliderRowInt("Bloom reach", &bloomPasses, 0, 10, "%d", "How far glow spreads. 0 turns bloom off."))
        {
            m_renderer->setPBRBloomPasses(bloomPasses);
        }
        float fog = m_renderer->getPBRFogDensityScale();
        if (ui::sliderRow("Haze", &fog, 0.0f, 3.0f, "%.2f"))
        {
            m_renderer->setPBRFogDensityScale(fog);
        }

        ui::sectionLabel("LIGHTING");
        bool shadows = m_renderer->getPBRShadowsEnabled();
        if (ui::toggleRow("Shadows", &shadows))
        {
            m_renderer->setPBRShadowsEnabled(shadows);
        }
        bool effectLights = m_renderer->getEffectLightsEnabled();
        if (ui::toggleRow("Effect lights", &effectLights, "Explosions, launches and engine plumes light the scene."))
        {
            m_renderer->setEffectLightsEnabled(effectLights);
        }
        float azimuth = 0.0f, elevation = 0.0f, intensity = 0.0f;
        m_renderer->getSunOrientation(azimuth, elevation, intensity);
        bool sunChanged = false;
        sunChanged |= ui::sliderRow("Sun direction", &azimuth, -180.0f, 180.0f, "%.0f\xC2\xB0");
        sunChanged |= ui::sliderRow("Sun height", &elevation, 5.0f, 89.0f, "%.0f\xC2\xB0");
        sunChanged |= ui::sliderRow("Sun strength", &intensity, 0.5f, 8.0f, "%.2f");
        if (sunChanged)
        {
            m_renderer->setSunOrientation(azimuth, elevation, intensity);
        }

        break;
    }
    case SettingsPage::Audio:
    {
        ui::sectionLabel("VOLUME");
        float volumePercent = m_audioVolume * 100.0f;
        if (ui::sliderRow("Master volume", &volumePercent, 0.0f, 100.0f, "%.0f%%"))
        {
            m_audioVolume = volumePercent / 100.0f;
        }
        ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));
        ImGui::PushTextWrapPos(0.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, ui::color::textMuted);
        ImGui::TextUnformatted("Every sound is synthesised from the simulation: engines, boosters and "
                               "explosions travel at the speed of sound, with Doppler shift and echoes "
                               "off the terrain.");
        ImGui::PopStyleColor();
        ImGui::PopTextWrapPos();
        break;
    }
    case SettingsPage::Controls:
    {
        ui::sectionLabel("MOUSE AIM");
        float sensitivityPercent = m_mouseAimSensitivity / 0.06f * 100.0f;
        if (ui::sliderRow("Aim sensitivity", &sensitivityPercent, 25.0f, 300.0f, "%.0f%%",
                          "How far the aim moves per mouse movement. 100% is 0.06 degrees per count."))
        {
            m_mouseAimSensitivity = sensitivityPercent / 100.0f * 0.06f;
            scheduleSettingsSave();
        }
        if (ui::toggleRow("Invert vertical aim", &m_invertMouseY))
        {
            scheduleSettingsSave();
        }
        float lagMs = 1000.0f / m_cameraSmoothing;
        if (ui::sliderRow("Camera lag", &lagMs, 25.0f, 300.0f, "%.0f ms",
                          "How long the view takes to catch up with the aim. Lower is tighter."))
        {
            m_cameraSmoothing = 1000.0f / std::max(lagMs, 1.0f);
            scheduleSettingsSave();
        }
        drawBindings("FLIGHT", kFlightBindings, std::size(kFlightBindings));
        drawBindings("CAMERA", kCameraBindings, std::size(kCameraBindings));
        drawBindings("INTERFACE", kInterfaceBindings, std::size(kInterfaceBindings));
        break;
    }
    }
}

void Application::renderScreenFade()
{
    if (m_fadeAlpha <= 0.0f)
    {
        return;
    }
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::GetForegroundDrawList()->AddRectFilled(
        viewport->Pos, ImVec2(viewport->Pos.x + viewport->Size.x, viewport->Pos.y + viewport->Size.y),
        ui::toU32(ImVec4(0.012f, 0.016f, 0.024f, 1.0f), easeOutCubic(m_fadeAlpha)));
    m_fadeAlpha = std::max(0.0f, m_fadeAlpha - ImGui::GetIO().DeltaTime / m_fadeDuration);
}

void Application::advanceMenuClocks(float deltaTime)
{
    m_screenTime += deltaTime;
    m_overlayTime += deltaTime;
}
