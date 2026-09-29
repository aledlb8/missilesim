#pragma once

#include <imgui.h>

// The visual language of every screen: colour tokens, type scale, fonts and the
// ImGui style built from them. UI code should take colours and sizes from here
// rather than hardcoding IM_COL32 values, so screens stay consistent.
namespace missilesim::ui
{
    namespace color
    {
        // Neutrals: a cool near-black glass with three text strengths.
        inline constexpr ImVec4 text{0.906f, 0.918f, 0.937f, 1.0f};      // #E7EAEF
        inline constexpr ImVec4 textMuted{0.576f, 0.612f, 0.667f, 1.0f}; // #939CAA
        inline constexpr ImVec4 textFaint{0.384f, 0.420f, 0.471f, 1.0f}; // #626B78
        inline constexpr ImVec4 surface{0.043f, 0.055f, 0.071f, 0.88f};  // panels over the scene
        inline constexpr ImVec4 surfaceRaised{0.071f, 0.086f, 0.110f, 0.97f};
        inline constexpr ImVec4 hairline{1.0f, 1.0f, 1.0f, 0.075f};
        inline constexpr ImVec4 fill{1.0f, 1.0f, 1.0f, 0.055f}; // controls at rest

        // One interactive accent, plus semantic states.
        inline constexpr ImVec4 accent{1.0f, 0.710f, 0.278f, 1.0f};       // #FFB547
        inline constexpr ImVec4 accentBright{1.0f, 0.800f, 0.471f, 1.0f}; // #FFCC78
        inline constexpr ImVec4 positive{0.357f, 0.839f, 0.604f, 1.0f};   // #5BD69A
        inline constexpr ImVec4 danger{1.0f, 0.353f, 0.322f, 1.0f};       // #FF5A52
        inline constexpr ImVec4 info{0.400f, 0.780f, 0.957f, 1.0f};       // #66C7F4
    }

    // Returns the colour with its alpha multiplied by alphaScale (for fades).
    ImVec4 withAlpha(const ImVec4 &colour, float alphaScale);
    ImU32 toU32(const ImVec4 &colour, float alphaScale = 1.0f);

    // Font sizes in pixels at 100% display scale; ImGui applies the DPI factor.
    namespace type
    {
        inline constexpr float caption = 12.5f;
        inline constexpr float body = 15.5f;
        inline constexpr float heading = 19.0f;
        inline constexpr float title = 30.0f;
        inline constexpr float display = 64.0f;
    }

    struct Fonts
    {
        ImFont *body = nullptr;          // Barlow Regular: running text
        ImFont *medium = nullptr;        // Barlow Medium: labels, controls
        ImFont *strong = nullptr;        // Barlow SemiBold: buttons, emphasis
        ImFont *display = nullptr;       // Barlow Condensed SemiBold: headings, HUD caps
        ImFont *displayMedium = nullptr; // Barlow Condensed Medium: secondary headings
        ImFont *mono = nullptr;          // IBM Plex Mono Regular: tabular numbers
        ImFont *monoMedium = nullptr;    // IBM Plex Mono Medium: emphasised numbers
    };

    // Loads the fonts and applies the style. Call once after ImGui::CreateContext()
    // and before the first NewFrame(). Missing font files fall back to ImGui's
    // built-in font (and are logged) so the app still starts.
    void initializeTheme(float contentScale);

    // Rebuilds sizes for a new monitor content scale (DPI). No-op when unchanged.
    void setContentScale(float contentScale);
    float contentScale();

    const Fonts &fonts();

    // Pushes a font at a base size for the lifetime of the object.
    class ScopedFont
    {
    public:
        ScopedFont(ImFont *font, float baseSize) { ImGui::PushFont(font, baseSize); }
        ~ScopedFont() { ImGui::PopFont(); }
        ScopedFont(const ScopedFont &) = delete;
        ScopedFont &operator=(const ScopedFont &) = delete;
    };
}
