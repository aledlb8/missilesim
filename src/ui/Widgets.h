#pragma once

#include <imgui.h>

// Reusable controls drawn in the MissileSim visual language (see Theme.h).
// Row widgets lay out as "label ........ control" across the available width
// so every settings list and panel lines up the same way.
namespace missilesim::ui
{
    // Converts a size at 100% scale into pixels at the current DPI/UI scale.
    float px(float value);

    // Moves a remembered 0..1 value toward `target` at `ratePerSecond`; keyed by
    // ImGui ID so any widget can ease hover/selection states without storage.
    float animate(ImGuiID id, float target, float ratePerSecond = 12.0f);

    // Letter-spaced single-line text. `tracking` is in ems (0.1 = 10% of size).
    // `size` is the final pixel size (already scaled).
    ImVec2 measureTracked(ImFont *font, float size, const char *text, float tracking);
    void drawTracked(ImDrawList *drawList, ImFont *font, float size, ImVec2 position,
                     ImU32 colour, const char *text, float tracking);

    // Small uppercase label used above groups ("DISPLAY", "GUIDANCE", ...).
    void sectionLabel(const char *text);

    // Large uppercase menu entry (title and pause menus). `highlighted` is the
    // keyboard selection; hovering reports through `hovered`. Returns true on click.
    bool menuItem(const char *label, bool highlighted, bool *hovered = nullptr);

    enum class ButtonStyle
    {
        Primary,   // amber, one per surface
        Secondary, // neutral fill
        Ghost      // text only
    };
    // Width 0 = fit label, negative = fill available width.
    bool button(const char *label, ButtonStyle style = ButtonStyle::Secondary, float width = 0.0f);

    // Rows: label on the left, control on the right. Return true when changed.
    // Double-click a slider to type an exact value.
    bool sliderRow(const char *label, float *value, float minValue, float maxValue,
                   const char *format, const char *hint = nullptr);
    bool sliderRowInt(const char *label, int *value, int minValue, int maxValue,
                      const char *format = "%d", const char *hint = nullptr);
    bool toggleRow(const char *label, bool *value, const char *hint = nullptr);
    bool segmentedRow(const char *label, int *selected, const char *const *items, int itemCount,
                      const char *hint = nullptr);
    void readoutRow(const char *label, const char *value, const ImVec4 *valueColour = nullptr);

    // Keyboard key drawn as a keycap, followed on the same line by its action.
    void keyHintRow(const char *keys, const char *action);
    // A keycap at an absolute position; returns its width.
    float drawKeycap(ImDrawList *drawList, ImVec2 position, const char *key, float alpha = 1.0f);

    // Full-screen translucent wash behind modal screens.
    void dimBackground(float alpha);

    // Four corner marks around a point: the one target bracket every screen
    // draws. Each mark runs legFraction of `half` along both edges.
    void drawCornerBrackets(ImDrawList *drawList, ImVec2 centre, float half, ImU32 colour, float thickness,
                            float legFraction = 0.44f);
}
