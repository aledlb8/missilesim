#include "Theme.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

namespace missilesim::ui
{
    namespace
    {
        Fonts g_fonts;
        float g_contentScale = 0.0f;

        std::filesystem::path resolveFontPath(const char *fileName)
        {
            const std::filesystem::path relative = std::filesystem::path("fonts") / fileName;
            const std::filesystem::path sourceAssetRoot(MISSILESIM_SOURCE_ASSET_DIR);
            const std::filesystem::path candidates[] = {
                std::filesystem::current_path() / "assets" / relative,
                std::filesystem::current_path() / relative,
                sourceAssetRoot.empty() ? std::filesystem::path() : sourceAssetRoot / relative};

            for (const std::filesystem::path &candidate : candidates)
            {
                std::error_code error;
                if (!candidate.empty() && std::filesystem::exists(candidate, error))
                {
                    return candidate;
                }
            }
            return {};
        }

        ImFont *loadFont(const char *fileName, ImFont *fallback)
        {
            const std::filesystem::path path = resolveFontPath(fileName);
            if (path.empty())
            {
                std::cerr << "WARNING: UI font not found: " << fileName << std::endl;
                return fallback;
            }

            ImFontConfig config;
            // Glyphs are rasterised on demand at whatever size is pushed (ImGui
            // 1.92 dynamic fonts); a little horizontal oversampling keeps small
            // condensed text crisp.
            config.OversampleH = 2;
            config.OversampleV = 1;
            ImFont *font = ImGui::GetIO().Fonts->AddFontFromFileTTF(path.string().c_str(), type::body, &config);
            if (font == nullptr)
            {
                std::cerr << "WARNING: Failed to load UI font: " << path << std::endl;
                return fallback;
            }
            return font;
        }

        ImGuiStyle buildStyle()
        {
            ImGuiStyle style;
            ImGui::StyleColorsDark(&style);

            style.WindowPadding = ImVec2(16.0f, 14.0f);
            style.FramePadding = ImVec2(10.0f, 6.0f);
            style.CellPadding = ImVec2(8.0f, 5.0f);
            style.ItemSpacing = ImVec2(10.0f, 8.0f);
            style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);
            style.IndentSpacing = 18.0f;
            style.ScrollbarSize = 10.0f;
            style.GrabMinSize = 12.0f;

            style.WindowBorderSize = 1.0f;
            style.ChildBorderSize = 1.0f;
            style.PopupBorderSize = 1.0f;
            style.FrameBorderSize = 0.0f;
            style.TabBorderSize = 0.0f;
            style.TabBarBorderSize = 1.0f;
            style.TabBarOverlineSize = 0.0f;
            style.SeparatorTextBorderSize = 1.0f;

            style.WindowRounding = 8.0f;
            style.ChildRounding = 6.0f;
            style.FrameRounding = 5.0f;
            style.PopupRounding = 6.0f;
            style.ScrollbarRounding = 6.0f;
            style.GrabRounding = 4.0f;
            style.TabRounding = 5.0f;

            style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
            style.WindowMenuButtonPosition = ImGuiDir_None;
            style.SeparatorTextAlign = ImVec2(0.0f, 0.5f);
            style.SeparatorTextPadding = ImVec2(0.0f, 6.0f);

            const ImVec4 clear(0.0f, 0.0f, 0.0f, 0.0f);
            const ImVec4 white(1.0f, 1.0f, 1.0f, 1.0f);
            auto whiteAlpha = [&](float alpha)
            { return withAlpha(white, alpha); };
            auto accentAlpha = [&](float alpha)
            { return withAlpha(color::accent, alpha); };

            ImVec4 *c = style.Colors;
            c[ImGuiCol_Text] = color::text;
            c[ImGuiCol_TextDisabled] = color::textMuted;
            c[ImGuiCol_WindowBg] = color::surface;
            c[ImGuiCol_ChildBg] = clear;
            c[ImGuiCol_PopupBg] = color::surfaceRaised;
            c[ImGuiCol_Border] = color::hairline;
            c[ImGuiCol_BorderShadow] = clear;

            c[ImGuiCol_FrameBg] = color::fill;
            c[ImGuiCol_FrameBgHovered] = whiteAlpha(0.085f);
            c[ImGuiCol_FrameBgActive] = whiteAlpha(0.11f);

            c[ImGuiCol_TitleBg] = color::surface;
            c[ImGuiCol_TitleBgActive] = color::surfaceRaised;
            c[ImGuiCol_TitleBgCollapsed] = color::surface;
            c[ImGuiCol_MenuBarBg] = clear;

            c[ImGuiCol_ScrollbarBg] = clear;
            c[ImGuiCol_ScrollbarGrab] = whiteAlpha(0.12f);
            c[ImGuiCol_ScrollbarGrabHovered] = whiteAlpha(0.20f);
            c[ImGuiCol_ScrollbarGrabActive] = whiteAlpha(0.28f);

            c[ImGuiCol_CheckMark] = color::accent;
            c[ImGuiCol_SliderGrab] = color::accent;
            c[ImGuiCol_SliderGrabActive] = color::accentBright;

            c[ImGuiCol_Button] = whiteAlpha(0.065f);
            c[ImGuiCol_ButtonHovered] = whiteAlpha(0.11f);
            c[ImGuiCol_ButtonActive] = whiteAlpha(0.15f);

            c[ImGuiCol_Header] = accentAlpha(0.14f);
            c[ImGuiCol_HeaderHovered] = accentAlpha(0.20f);
            c[ImGuiCol_HeaderActive] = accentAlpha(0.26f);

            c[ImGuiCol_Separator] = color::hairline;
            c[ImGuiCol_SeparatorHovered] = accentAlpha(0.50f);
            c[ImGuiCol_SeparatorActive] = color::accent;

            c[ImGuiCol_ResizeGrip] = clear;
            c[ImGuiCol_ResizeGripHovered] = accentAlpha(0.35f);
            c[ImGuiCol_ResizeGripActive] = accentAlpha(0.60f);

            c[ImGuiCol_InputTextCursor] = color::accent;

            c[ImGuiCol_Tab] = clear;
            c[ImGuiCol_TabHovered] = whiteAlpha(0.08f);
            c[ImGuiCol_TabSelected] = whiteAlpha(0.10f);
            c[ImGuiCol_TabSelectedOverline] = color::accent;
            c[ImGuiCol_TabDimmed] = clear;
            c[ImGuiCol_TabDimmedSelected] = whiteAlpha(0.06f);
            c[ImGuiCol_TabDimmedSelectedOverline] = clear;

            c[ImGuiCol_PlotLines] = color::textMuted;
            c[ImGuiCol_PlotLinesHovered] = color::accent;
            c[ImGuiCol_PlotHistogram] = color::accent;
            c[ImGuiCol_PlotHistogramHovered] = color::accentBright;

            c[ImGuiCol_TableHeaderBg] = whiteAlpha(0.04f);
            c[ImGuiCol_TableBorderStrong] = color::hairline;
            c[ImGuiCol_TableBorderLight] = whiteAlpha(0.045f);
            c[ImGuiCol_TableRowBg] = clear;
            c[ImGuiCol_TableRowBgAlt] = whiteAlpha(0.02f);

            c[ImGuiCol_TextLink] = color::accent;
            c[ImGuiCol_TextSelectedBg] = accentAlpha(0.30f);
            c[ImGuiCol_DragDropTarget] = color::accent;
            c[ImGuiCol_NavCursor] = color::accent;
            c[ImGuiCol_NavWindowingHighlight] = whiteAlpha(0.70f);
            c[ImGuiCol_NavWindowingDimBg] = ImVec4(0.02f, 0.03f, 0.04f, 0.55f);
            c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.02f, 0.03f, 0.04f, 0.60f);

            style.FontSizeBase = type::body;
            return style;
        }

        void applyStyle(float scale)
        {
            ImGuiStyle style = buildStyle();
            style.ScaleAllSizes(scale);
            style.FontScaleDpi = scale;
            ImGui::GetStyle() = style;
        }
    }

    ImVec4 withAlpha(const ImVec4 &colour, float alphaScale)
    {
        return ImVec4(colour.x, colour.y, colour.z, colour.w * alphaScale);
    }

    ImU32 toU32(const ImVec4 &colour, float alphaScale)
    {
        // Honour PushStyleVar(ImGuiStyleVar_Alpha) like ImGui's own GetColorU32,
        // so custom-drawn widgets fade together with the screen around them.
        const float styleAlpha = ImGui::GetCurrentContext() != nullptr ? ImGui::GetStyle().Alpha : 1.0f;
        return ImGui::ColorConvertFloat4ToU32(withAlpha(colour, alphaScale * styleAlpha));
    }

    void initializeTheme(float scale)
    {
        ImGuiIO &io = ImGui::GetIO();
        io.Fonts->Clear();

        // The default font is only a safety net for missing files.
        ImFont *fallback = io.Fonts->AddFontDefault();
        g_fonts.body = loadFont("Barlow-Regular.ttf", fallback);
        g_fonts.medium = loadFont("Barlow-Medium.ttf", g_fonts.body);
        g_fonts.strong = loadFont("Barlow-SemiBold.ttf", g_fonts.medium);
        g_fonts.display = loadFont("BarlowCondensed-SemiBold.ttf", g_fonts.strong);
        g_fonts.displayMedium = loadFont("BarlowCondensed-Medium.ttf", g_fonts.display);
        g_fonts.mono = loadFont("IBMPlexMono-Regular.ttf", g_fonts.body);
        g_fonts.monoMedium = loadFont("IBMPlexMono-Medium.ttf", g_fonts.mono);
        io.FontDefault = g_fonts.medium;

        g_contentScale = 0.0f;
        setContentScale(scale);
    }

    void setContentScale(float scale)
    {
        scale = std::clamp(std::isfinite(scale) ? scale : 1.0f, 0.5f, 4.0f);
        if (std::abs(scale - g_contentScale) < 0.001f)
        {
            return;
        }
        g_contentScale = scale;
        applyStyle(scale);
    }

    float contentScale()
    {
        return g_contentScale > 0.0f ? g_contentScale : 1.0f;
    }

    const Fonts &fonts()
    {
        return g_fonts;
    }
}
