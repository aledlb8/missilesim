// Main window management: creation and placement, display modes (windowed /
// borderless / fullscreen), DPI tracking, the window icon and native styling.
#include "Application.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ui/Theme.h"

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <dwmapi.h>
#pragma comment(lib, "dwmapi.lib")
#endif

namespace
{
    constexpr int kMinWindowWidth = 960;
    constexpr int kMinWindowHeight = 540;
    constexpr float kWindowedWorkAreaFraction = 0.8f;

    struct Rect
    {
        int x = 0;
        int y = 0;
        int width = 0;
        int height = 0;
    };

    Rect monitorWorkArea(GLFWmonitor *monitor)
    {
        Rect area;
        glfwGetMonitorWorkarea(monitor, &area.x, &area.y, &area.width, &area.height);
        return area;
    }

    // The largest 16:9 client area that fits the given fraction of the work area.
    Rect fitWindowedClientArea(const Rect &workArea)
    {
        float width = static_cast<float>(workArea.width) * kWindowedWorkAreaFraction;
        float height = width * 9.0f / 16.0f;
        const float maxHeight = static_cast<float>(workArea.height) * kWindowedWorkAreaFraction;
        if (height > maxHeight)
        {
            height = maxHeight;
            width = height * 16.0f / 9.0f;
        }

        Rect client;
        client.width = std::max(static_cast<int>(std::lround(width)), std::min(kMinWindowWidth, workArea.width));
        client.height = std::max(static_cast<int>(std::lround(height)), std::min(kMinWindowHeight, workArea.height));
        return client;
    }

    // Positions a client area so the whole window, including its frame, is
    // centred in the work area.
    void centreInWorkArea(GLFWwindow *window, const Rect &workArea, Rect &client)
    {
        int left = 0, top = 0, right = 0, bottom = 0;
        glfwGetWindowFrameSize(window, &left, &top, &right, &bottom);
        const int outerWidth = client.width + left + right;
        const int outerHeight = client.height + top + bottom;
        client.x = workArea.x + std::max(0, (workArea.width - outerWidth) / 2) + left;
        client.y = workArea.y + std::max(0, (workArea.height - outerHeight) / 2) + top;
    }

    GLFWmonitor *monitorContainingWindow(GLFWwindow *window)
    {
        int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
        glfwGetWindowPos(window, &windowX, &windowY);
        glfwGetWindowSize(window, &windowWidth, &windowHeight);
        const int centreX = windowX + windowWidth / 2;
        const int centreY = windowY + windowHeight / 2;

        int count = 0;
        GLFWmonitor **monitors = glfwGetMonitors(&count);
        for (int i = 0; i < count; ++i)
        {
            int monitorX = 0, monitorY = 0;
            glfwGetMonitorPos(monitors[i], &monitorX, &monitorY);
            const GLFWvidmode *mode = glfwGetVideoMode(monitors[i]);
            if (mode != nullptr &&
                centreX >= monitorX && centreX < monitorX + mode->width &&
                centreY >= monitorY && centreY < monitorY + mode->height)
            {
                return monitors[i];
            }
        }
        return glfwGetPrimaryMonitor();
    }

    // The app icon: an amber heading arrow on a dark rounded tile, rasterised
    // at each size with 4x4 supersampling so it stays sharp in the taskbar.
    std::vector<unsigned char> rasteriseIcon(int size)
    {
        struct Point
        {
            float x, y;
        };
        const Point tip{0.50f, 0.17f};
        const Point leftBase{0.23f, 0.81f};
        const Point notch{0.50f, 0.63f};
        const Point rightBase{0.77f, 0.81f};

        auto edge = [](const Point &a, const Point &b, const Point &p)
        { return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x); };
        auto inTriangle = [&](const Point &a, const Point &b, const Point &c, const Point &p)
        {
            const float e0 = edge(a, b, p), e1 = edge(b, c, p), e2 = edge(c, a, p);
            return (e0 >= 0.0f && e1 >= 0.0f && e2 >= 0.0f) || (e0 <= 0.0f && e1 <= 0.0f && e2 <= 0.0f);
        };
        auto inTile = [](const Point &p)
        {
            constexpr float radius = 0.22f;
            const float dx = std::max({radius - p.x, 0.0f, p.x - (1.0f - radius)});
            const float dy = std::max({radius - p.y, 0.0f, p.y - (1.0f - radius)});
            return dx * dx + dy * dy <= radius * radius;
        };

        constexpr std::array<float, 4> tile{0.063f, 0.082f, 0.110f, 1.0f};
        const ImVec4 &accent = missilesim::ui::color::accent;
        constexpr int samples = 4;

        std::vector<unsigned char> pixels(static_cast<size_t>(size) * size * 4);
        for (int y = 0; y < size; ++y)
        {
            for (int x = 0; x < size; ++x)
            {
                float tileCoverage = 0.0f;
                float arrowCoverage = 0.0f;
                for (int sy = 0; sy < samples; ++sy)
                {
                    for (int sx = 0; sx < samples; ++sx)
                    {
                        const Point p{(x + (sx + 0.5f) / samples) / size, (y + (sy + 0.5f) / samples) / size};
                        if (inTile(p))
                        {
                            tileCoverage += 1.0f;
                            if (inTriangle(tip, leftBase, notch, p) || inTriangle(tip, notch, rightBase, p))
                            {
                                arrowCoverage += 1.0f;
                            }
                        }
                    }
                }
                tileCoverage /= samples * samples;
                arrowCoverage /= samples * samples;

                const float arrowMix = tileCoverage > 0.0f ? arrowCoverage / tileCoverage : 0.0f;
                unsigned char *out = &pixels[(static_cast<size_t>(y) * size + x) * 4];
                out[0] = static_cast<unsigned char>(std::lround(255.0f * (tile[0] + (accent.x - tile[0]) * arrowMix)));
                out[1] = static_cast<unsigned char>(std::lround(255.0f * (tile[1] + (accent.y - tile[1]) * arrowMix)));
                out[2] = static_cast<unsigned char>(std::lround(255.0f * (tile[2] + (accent.z - tile[2]) * arrowMix)));
                out[3] = static_cast<unsigned char>(std::lround(255.0f * tileCoverage));
            }
        }
        return pixels;
    }

    void applyWindowIcon(GLFWwindow *window)
    {
        constexpr std::array<int, 6> sizes{16, 24, 32, 48, 64, 128};
        std::array<std::vector<unsigned char>, sizes.size()> pixels;
        std::array<GLFWimage, sizes.size()> images{};
        for (size_t i = 0; i < sizes.size(); ++i)
        {
            pixels[i] = rasteriseIcon(sizes[i]);
            images[i].width = sizes[i];
            images[i].height = sizes[i];
            images[i].pixels = pixels[i].data();
        }
        glfwSetWindowIcon(window, static_cast<int>(images.size()), images.data());
    }

    // Dark title bar and frame on Windows 10/11 so the window chrome matches
    // the UI. Attributes the running Windows version lacks are ignored.
    void applyNativeWindowStyling(GLFWwindow *window)
    {
#ifdef _WIN32
        HWND hwnd = glfwGetWin32Window(window);
        if (hwnd == nullptr)
        {
            return;
        }
        constexpr DWORD kUseImmersiveDarkMode = 20; // DWMWA_USE_IMMERSIVE_DARK_MODE
        constexpr DWORD kBorderColour = 34;         // DWMWA_BORDER_COLOR (Windows 11)
        constexpr DWORD kCaptionColour = 35;        // DWMWA_CAPTION_COLOR (Windows 11)
        constexpr DWORD kTextColour = 36;           // DWMWA_TEXT_COLOR (Windows 11)

        const BOOL dark = TRUE;
        DwmSetWindowAttribute(hwnd, kUseImmersiveDarkMode, &dark, sizeof(dark));
        const COLORREF caption = RGB(12, 15, 20);
        const COLORREF border = RGB(34, 40, 49);
        const COLORREF text = RGB(200, 206, 214);
        DwmSetWindowAttribute(hwnd, kCaptionColour, &caption, sizeof(caption));
        DwmSetWindowAttribute(hwnd, kBorderColour, &border, sizeof(border));
        DwmSetWindowAttribute(hwnd, kTextColour, &text, sizeof(text));
#else
        (void)window;
#endif
    }
}

void Application::createMainWindow()
{
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Stay hidden through the slow start-up work (shader compiles, IBL bakes,
    // model loads) and appear once the first frame is ready.
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    const Rect workArea = monitorWorkArea(glfwGetPrimaryMonitor());
    Rect client = fitWindowedClientArea(workArea);

    m_window = glfwCreateWindow(client.width, client.height, m_title.c_str(), nullptr, nullptr);
    if (!m_window)
    {
        const char *reason = nullptr;
        glfwGetError(&reason);
        std::string message =
            "Could not create an OpenGL 4.5 window.\n\n"
            "MissileSim needs a graphics card and driver that support OpenGL 4.5. "
            "Updating your graphics driver usually fixes this.";
        if (reason)
        {
            message += "\n\nDriver said: ";
            message += reason;
        }
        glfwTerminate();
        throw std::runtime_error(message);
    }

    centreInWorkArea(m_window, workArea, client);
    glfwSetWindowPos(m_window, client.x, client.y);
    glfwSetWindowSizeLimits(m_window,
                            std::min(kMinWindowWidth, workArea.width),
                            std::min(kMinWindowHeight, workArea.height),
                            GLFW_DONT_CARE, GLFW_DONT_CARE);
    m_windowedPlacement = {client.x, client.y, client.width, client.height, true};
    m_width = client.width;
    m_height = client.height;
    m_displayMode = DisplayMode::Windowed;

    applyWindowIcon(m_window);
    applyNativeWindowStyling(m_window);

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(m_vsyncEnabled ? 1 : 0);
}

float Application::windowContentScale() const
{
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    if (m_window)
    {
        glfwGetWindowContentScale(m_window, &scaleX, &scaleY);
    }
    return scaleX;
}

void Application::setDisplayMode(DisplayMode mode)
{
    if (!m_window || mode == m_displayMode)
    {
        return;
    }

    if (m_displayMode == DisplayMode::Windowed)
    {
        glfwGetWindowPos(m_window, &m_windowedPlacement.x, &m_windowedPlacement.y);
        glfwGetWindowSize(m_window, &m_windowedPlacement.width, &m_windowedPlacement.height);
        m_windowedPlacement.valid = true;
    }

    GLFWmonitor *monitor = monitorContainingWindow(m_window);
    const GLFWvidmode *video = glfwGetVideoMode(monitor);
    if (video == nullptr)
    {
        return;
    }
    int monitorX = 0, monitorY = 0;
    glfwGetMonitorPos(monitor, &monitorX, &monitorY);

    switch (mode)
    {
    case DisplayMode::Windowed:
    {
        Rect client{m_windowedPlacement.x, m_windowedPlacement.y, m_windowedPlacement.width, m_windowedPlacement.height};
        if (!m_windowedPlacement.valid)
        {
            const Rect workArea = monitorWorkArea(monitor);
            client = fitWindowedClientArea(workArea);
            centreInWorkArea(m_window, workArea, client);
        }
        glfwSetWindowAttrib(m_window, GLFW_DECORATED, GLFW_TRUE);
        glfwSetWindowMonitor(m_window, nullptr, client.x, client.y, client.width, client.height, GLFW_DONT_CARE);
        break;
    }
    case DisplayMode::Borderless:
        if (m_displayMode == DisplayMode::Fullscreen)
        {
            glfwSetWindowMonitor(m_window, nullptr, monitorX, monitorY, video->width, video->height, GLFW_DONT_CARE);
        }
        glfwSetWindowAttrib(m_window, GLFW_DECORATED, GLFW_FALSE);
        glfwSetWindowMonitor(m_window, nullptr, monitorX, monitorY, video->width, video->height, GLFW_DONT_CARE);
        break;
    case DisplayMode::Fullscreen:
        glfwSetWindowMonitor(m_window, monitor, 0, 0, video->width, video->height, video->refreshRate);
        break;
    }

    m_displayMode = mode;
    // Some drivers reset the swap interval when the window changes monitor mode.
    glfwSwapInterval(m_vsyncEnabled ? 1 : 0);
}

void Application::toggleFullscreen()
{
    setDisplayMode(m_displayMode == DisplayMode::Windowed ? DisplayMode::Borderless : DisplayMode::Windowed);
}

bool Application::isWindowMinimized() const
{
    if (!m_window)
    {
        return false;
    }
    int width = 0, height = 0;
    glfwGetFramebufferSize(m_window, &width, &height);
    return glfwGetWindowAttrib(m_window, GLFW_ICONIFIED) == GLFW_TRUE || width == 0 || height == 0;
}

void Application::updateWindowFrame()
{
    if (!m_window)
    {
        return;
    }

    const bool fullscreenKeyDown = glfwGetKey(m_window, GLFW_KEY_F11) == GLFW_PRESS;
    if (fullscreenKeyDown && !m_fullscreenKeyHeld)
    {
        toggleFullscreen();
    }
    m_fullscreenKeyHeld = fullscreenKeyDown;

    missilesim::ui::setContentScale(windowContentScale() * m_uiScale);
}

void Application::setVsyncEnabled(bool enabled)
{
    m_vsyncEnabled = enabled;
    if (m_window)
    {
        glfwSwapInterval(enabled ? 1 : 0);
    }
}

void Application::revealWindowAfterFirstFrame()
{
    if (m_window && !m_windowRevealed)
    {
        glfwShowWindow(m_window);
        glfwFocusWindow(m_window);
        m_windowRevealed = true;
    }
}
