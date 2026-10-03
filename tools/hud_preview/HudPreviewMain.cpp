// HUD preview: the radar scope, the RWR and the fire-control symbols drawn
// with scripted pictures in a hidden window, read back and written as PNG
// files. It exercises the same drawing code the game runs, with no simulation
// behind it.
//
// Usage: hud_preview [out-dir]   (default: screenshots/hud_preview)
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include "ui/RadarScope.h"
#include "ui/RwrScope.h"
#include "ui/TargetSymbols.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace ui = missilesim::ui;

namespace
{
    constexpr int kWidth = 760;
    constexpr int kHeight = 420;

    float deg(float degrees)
    {
        return degrees * 3.14159265f / 180.0f;
    }

    struct Picture
    {
        const char *name;
        ui::RadarScopeView scope;
        ui::RwrView rwr;
        bool symbols = false; // the fire-control symbol sheet instead of the scopes
    };

    // Every world symbol in each of its states, side by side.
    void drawSymbolSheet(ImDrawList *drawList)
    {
        const float time = 0.1f;
        ui::drawSpottingMark(drawList, ImVec2(90.0f, 120.0f), "5.21 km");
        ui::drawContactMark(drawList, ImVec2(90.0f, 120.0f), 0.8f);
        ui::drawContactMark(drawList, ImVec2(160.0f, 120.0f), 0.35f);

        ui::drawLockBox(drawList, ImVec2(270.0f, 120.0f), ui::LockStyle::Held, "9.80 km", "+412 m/s", time);
        ui::drawLockBox(drawList, ImVec2(420.0f, 120.0f), ui::LockStyle::Memory, "9.62 km", "+398 m/s", time);
        ui::drawLockBox(drawList, ImVec2(570.0f, 120.0f), ui::LockStyle::Shoot, "8.71 km", "+420 m/s", time);

        ui::drawSeekerCircle(drawList, ImVec2(120.0f, 300.0f), 22.0f, ui::SeekerMark::Search, time);
        ui::drawSeekerCircle(drawList, ImVec2(270.0f, 300.0f), 22.0f, ui::SeekerMark::Slaved, time);
        ui::drawLockBox(drawList, ImVec2(420.0f, 300.0f), ui::LockStyle::Held, "4.10 km", "+380 m/s", time);
        ui::drawSeekerCircle(drawList, ImVec2(420.0f, 300.0f), 22.0f, ui::SeekerMark::Designated, time);
        ui::drawLockBox(drawList, ImVec2(570.0f, 300.0f), ui::LockStyle::Shoot, "3.95 km", "+385 m/s", time);
        ui::drawSeekerCircle(drawList, ImVec2(570.0f, 300.0f), 22.0f, ui::SeekerMark::Locked, time);
    }

    ui::ScopeContact contact(std::uint32_t id, float rangeM, float azimuthDeg, ui::ScopeLife life, float trendRangeM,
                             float trendAzimuthDeg)
    {
        ui::ScopeContact c;
        c.trackId = id;
        c.rangeM = rangeM;
        c.azimuthRad = deg(azimuthDeg);
        c.life = life;
        c.hasTrend = true;
        c.trendRangeM = trendRangeM;
        c.trendAzimuthRad = deg(trendAzimuthDeg);
        c.ageSeconds = life == ui::ScopeLife::Coasting ? 2.4 : 0.0;
        return c;
    }

    std::vector<Picture> pictures()
    {
        std::vector<Picture> list;

        Picture search{"search", {}, {}};
        search.scope.rangeScaleM = 20000.0f;
        search.scope.azimuthHalfRad = deg(30.0f);
        search.scope.beamAzimuthRad = deg(-12.0f);
        search.scope.modeLabel = "SEARCH";
        search.scope.contacts = {contact(1, 14200.0f, -8.0f, ui::ScopeLife::Confirmed, 13000.0f, -6.0f),
                                 contact(2, 9800.0f, 17.0f, ui::ScopeLife::Confirmed, 9900.0f, 24.0f),
                                 contact(3, 17600.0f, 3.0f, ui::ScopeLife::Tentative, 17000.0f, 3.0f),
                                 contact(4, 6300.0f, -21.0f, ui::ScopeLife::Coasting, 6100.0f, -26.0f)};
        search.rwr.threats = {{ui::RwrKind::Search, deg(20.0f), 1.0f}, {ui::RwrKind::Search, deg(-95.0f), 0.5f}};
        search.rwr.chaff = 120;
        search.rwr.flares = 120;
        list.push_back(search);

        Picture lock{"lock", {}, {}};
        lock.scope = search.scope;
        lock.scope.modeLabel = "TRACK";
        lock.scope.singleTargetTrack = true;
        lock.scope.bar = -1;
        lock.scope.beamAzimuthRad = deg(17.0f);
        lock.scope.contacts[1].designated = true;
        lock.scope.contacts[1].launchQuality = true;
        lock.scope.hasLock = true;
        lock.scope.lockRangeM = 9800.0f;
        lock.scope.lockClosingMps = 412.0f;
        lock.rwr.threats = {{ui::RwrKind::Track, deg(20.0f), 1.0f}, {ui::RwrKind::Search, deg(-95.0f), 1.0f}};
        lock.rwr.chaff = 87;
        lock.rwr.flares = 104;
        list.push_back(lock);

        Picture missile{"missile", {}, {}};
        missile.scope = lock.scope;
        missile.scope.launchBlock = "Radar magazine is empty";
        missile.rwr.threats = {{ui::RwrKind::Launch, deg(24.0f), 1.0f},
                               {ui::RwrKind::Seeker, deg(150.0f), 1.0f},
                               {ui::RwrKind::Approach, deg(-140.0f), 1.0f},
                               {ui::RwrKind::Search, deg(-60.0f), 0.6f}};
        missile.rwr.chaff = 0;
        missile.rwr.flares = 36;
        missile.rwr.time = 0.05f;
        list.push_back(missile);

        Picture symbols{"symbols", {}, {}};
        symbols.symbols = true;
        list.push_back(symbols);
        return list;
    }

    bool writeFrame(const std::filesystem::path &path)
    {
        std::vector<unsigned char> pixels(static_cast<std::size_t>(kWidth) * kHeight * 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        stbi_flip_vertically_on_write(1);
        return stbi_write_png(path.string().c_str(), kWidth, kHeight, 4, pixels.data(), kWidth * 4) != 0;
    }
}

int main(int argc, char **argv)
{
    const std::filesystem::path outDir = argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path("screenshots/hud_preview");
    std::error_code error;
    std::filesystem::create_directories(outDir, error);

    if (!glfwInit())
    {
        std::fprintf(stderr, "hud_preview: glfwInit failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(kWidth, kHeight, "hud_preview", nullptr, nullptr);
    if (window == nullptr)
    {
        std::fprintf(stderr, "hud_preview: no OpenGL 4.5 window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::fprintf(stderr, "hud_preview: glad failed\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 450");
    ui::initializeTheme(1.0f);

    int written = 0;
    for (const Picture &picture : pictures())
    {
        // Two frames: the first builds the font atlas.
        for (int frame = 0; frame < 2; ++frame)
        {
            glViewport(0, 0, kWidth, kHeight);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::GetIO().DisplaySize = ImVec2(static_cast<float>(kWidth), static_cast<float>(kHeight));
            ImGui::NewFrame();

            // A daylight sky behind the glass, so contrast reads as in flight.
            ImDrawList *drawList = ImGui::GetBackgroundDrawList();
            drawList->AddRectFilledMultiColor(ImVec2(0.0f, 0.0f), ImVec2(static_cast<float>(kWidth), static_cast<float>(kHeight)),
                                              IM_COL32(88, 128, 176, 255), IM_COL32(88, 128, 176, 255),
                                              IM_COL32(172, 196, 214, 255), IM_COL32(172, 196, 214, 255));
            if (picture.symbols)
            {
                drawSymbolSheet(drawList);
            }
            else
            {
                ui::drawRadarScope(drawList, ImVec2(24.0f, 24.0f), 300.0f, picture.scope);
                ui::drawRwrScope(drawList, ImVec2(560.0f, 170.0f), 100.0f, picture.rwr);
            }

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        const std::filesystem::path file = outDir / (std::string(picture.name) + ".png");
        if (writeFrame(file))
        {
            std::printf("wrote %s\n", file.string().c_str());
            ++written;
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return written > 0 ? 0 : 1;
}
