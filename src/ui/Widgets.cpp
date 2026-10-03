#include "Widgets.h"
#include "Theme.h"

#include <imgui_internal.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>

namespace missilesim::ui
{
    namespace
    {
        std::unordered_map<ImGuiID, float> g_animations;

        // Slider text-entry state: only one slider can be in typing mode.
        ImGuiID g_editingSlider = 0;
        bool g_editFocusPending = false;
        bool g_pressStartedOnValue = false;

        constexpr float kRowHeight = 32.0f;
        constexpr float kControlFraction = 0.54f;

        ImVec4 mix(const ImVec4 &a, const ImVec4 &b, float t)
        {
            return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                          a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
        }

        struct Row
        {
            ImVec2 origin;
            float width = 0.0f;
            float height = 0.0f;
            float controlX = 0.0f;
            float controlWidth = 0.0f;
        };

        // Draws the row label and returns the geometry of the control area.
        Row layoutRow(const char *label, const char *hint)
        {
            Row row;
            row.origin = ImGui::GetCursorScreenPos();
            row.width = ImGui::GetContentRegionAvail().x;
            row.height = px(kRowHeight);
            row.controlWidth = std::floor(row.width * kControlFraction);
            row.controlX = row.origin.x + row.width - row.controlWidth;

            ImFont *font = fonts().medium;
            const float size = px(type::body);
            const ImVec2 labelPos(row.origin.x, row.origin.y + (row.height - size) * 0.5f);
            const ImVec2 labelSize = font->CalcTextSizeA(size, FLT_MAX, 0.0f, label);
            ImDrawList *drawList = ImGui::GetWindowDrawList();
            drawList->AddText(font, size, labelPos, toU32(color::text, 0.9f), label);

            if (hint != nullptr &&
                ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
                ImGui::IsMouseHoveringRect(labelPos, ImVec2(labelPos.x + labelSize.x, labelPos.y + labelSize.y)))
            {
                ImGui::SetTooltip("%s", hint);
            }
            return row;
        }

        void drawRowHoverWash(const Row &row, float amount)
        {
            if (amount <= 0.001f)
            {
                return;
            }
            const float bleed = px(6.0f);
            ImGui::GetWindowDrawList()->AddRectFilled(
                ImVec2(row.origin.x - bleed, row.origin.y),
                ImVec2(row.origin.x + row.width + bleed, row.origin.y + row.height),
                toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.035f), amount), px(5.0f));
        }

        bool sliderImpl(const char *label, float *value, float minValue, float maxValue,
                        const char *format, const char *hint, bool integer)
        {
            ImGui::PushID(label);
            const Row row = layoutRow(label, hint);
            ImDrawList *drawList = ImGui::GetWindowDrawList();
            const ImGuiID editId = ImGui::GetID("##edit");
            bool changed = false;

            if (g_editingSlider == editId)
            {
                const float frameHeight = ImGui::GetFrameHeight();
                ImGui::SetCursorScreenPos(ImVec2(row.controlX, row.origin.y + (row.height - frameHeight) * 0.5f));
                ImGui::SetNextItemWidth(row.controlWidth);
                if (g_editFocusPending)
                {
                    ImGui::SetKeyboardFocusHere();
                    g_editFocusPending = false;
                }
                float typed = *value;
                ImGui::PushFont(fonts().monoMedium, type::body - 1.0f);
                const bool entered = ImGui::InputFloat("##edit", &typed, 0.0f, 0.0f, integer ? "%.0f" : format,
                                                       ImGuiInputTextFlags_EnterReturnsTrue |
                                                           ImGuiInputTextFlags_AutoSelectAll);
                ImGui::PopFont();
                if (entered || ImGui::IsItemDeactivatedAfterEdit())
                {
                    typed = std::clamp(typed, minValue, maxValue);
                    if (integer)
                    {
                        typed = std::round(typed);
                    }
                    changed = typed != *value;
                    *value = typed;
                    g_editingSlider = 0;
                }
                else if (ImGui::IsItemDeactivated())
                {
                    g_editingSlider = 0;
                }
                // Keep the row height identical to the non-editing layout.
                ImGui::SetCursorScreenPos(ImVec2(row.origin.x, row.origin.y + row.height));
                ImGui::Dummy(ImVec2(0.0f, 0.0f));
                ImGui::PopID();
                return changed;
            }

            const float valueWidth = px(76.0f);
            const float gap = px(14.0f);
            const float knobRadius = px(7.0f);
            const float trackX0 = row.controlX + knobRadius;
            const float trackX1 = row.controlX + row.controlWidth - valueWidth - gap - knobRadius;
            const float trackWidth = std::max(trackX1 - trackX0, 1.0f);

            ImGui::SetCursorScreenPos(ImVec2(row.controlX, row.origin.y));
            ImGui::InvisibleButton("##slider", ImVec2(row.controlWidth, row.height));
            const ImGuiID id = ImGui::GetItemID();
            const bool hovered = ImGui::IsItemHovered();
            const bool active = ImGui::IsItemActive();
            const float mouseX = ImGui::GetIO().MousePos.x;

            if (ImGui::IsItemActivated())
            {
                g_pressStartedOnValue = mouseX > trackX1 + knobRadius;
            }

            if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                g_editingSlider = editId;
                g_editFocusPending = true;
            }
            else if (active && !g_pressStartedOnValue)
            {
                const float t = std::clamp((mouseX - trackX0) / trackWidth, 0.0f, 1.0f);
                float next = minValue + t * (maxValue - minValue);
                if (integer)
                {
                    next = std::round(next);
                }
                if (next != *value)
                {
                    *value = next;
                    changed = true;
                    ImGui::MarkItemEdited(id);
                }
            }
            else if (ImGui::IsItemDeactivated() && g_pressStartedOnValue && hovered)
            {
                g_editingSlider = editId;
                g_editFocusPending = true;
            }

            const float emphasis = animate(id, (hovered || active) ? 1.0f : 0.0f, 14.0f);
            const float centreY = row.origin.y + row.height * 0.5f;
            const float fraction = maxValue > minValue ? std::clamp((*value - minValue) / (maxValue - minValue), 0.0f, 1.0f) : 0.0f;
            const float knobX = trackX0 + fraction * trackWidth;
            const float trackThickness = px(4.0f);

            drawList->AddRectFilled(ImVec2(trackX0 - knobRadius * 0.5f, centreY - trackThickness * 0.5f),
                                    ImVec2(trackX1 + knobRadius * 0.5f, centreY + trackThickness * 0.5f),
                                    toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.11f + 0.04f * emphasis)), trackThickness);
            drawList->AddRectFilled(ImVec2(trackX0 - knobRadius * 0.5f, centreY - trackThickness * 0.5f),
                                    ImVec2(knobX, centreY + trackThickness * 0.5f),
                                    toU32(mix(color::accent, color::accentBright, emphasis)), trackThickness);
            if (active)
            {
                drawList->AddCircleFilled(ImVec2(knobX, centreY), knobRadius * 2.0f, toU32(color::accent, 0.16f), 24);
            }
            drawList->AddCircleFilled(ImVec2(knobX, centreY), knobRadius * (1.0f + 0.12f * emphasis), toU32(color::text), 24);

            // Integer rows use "%d"-style formats; float rows use "%f"-style ones.
            char text[64];
            if (integer)
            {
                std::snprintf(text, sizeof(text), format, static_cast<int>(std::lround(*value)));
            }
            else
            {
                std::snprintf(text, sizeof(text), format, static_cast<double>(*value));
            }
            ImFont *mono = fonts().mono;
            const float monoSize = px(type::body - 1.5f);
            const ImVec2 textSize = mono->CalcTextSizeA(monoSize, FLT_MAX, 0.0f, text);
            const float valueRight = row.controlX + row.controlWidth;
            drawList->AddText(mono, monoSize, ImVec2(valueRight - textSize.x, centreY - monoSize * 0.5f),
                              toU32(mix(color::textMuted, color::text, 0.55f + 0.45f * emphasis)), text);

            if (hovered)
            {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
            ImGui::PopID();
            return changed;
        }
    }

    float px(float value)
    {
        return value * contentScale();
    }

    float animate(ImGuiID id, float target, float ratePerSecond)
    {
        auto [entry, inserted] = g_animations.try_emplace(id, target);
        if (!inserted)
        {
            const float step = 1.0f - std::exp(-ratePerSecond * ImGui::GetIO().DeltaTime);
            entry->second += (target - entry->second) * step;
            if (std::abs(target - entry->second) < 0.001f)
            {
                entry->second = target;
            }
        }
        return entry->second;
    }

    ImVec2 measureTracked(ImFont *font, float size, const char *text, float tracking)
    {
        float width = 0.0f;
        int glyphs = 0;
        for (const char *c = text; *c != '\0'; ++c, ++glyphs)
        {
            width += font->CalcTextSizeA(size, FLT_MAX, 0.0f, c, c + 1).x;
        }
        if (glyphs > 1)
        {
            width += tracking * size * static_cast<float>(glyphs - 1);
        }
        return ImVec2(width, size);
    }

    void drawTracked(ImDrawList *drawList, ImFont *font, float size, ImVec2 position,
                     ImU32 colour, const char *text, float tracking)
    {
        float x = position.x;
        for (const char *c = text; *c != '\0'; ++c)
        {
            drawList->AddText(font, size, ImVec2(std::round(x), position.y), colour, c, c + 1);
            x += font->CalcTextSizeA(size, FLT_MAX, 0.0f, c, c + 1).x + tracking * size;
        }
    }

    void sectionLabel(const char *text)
    {
        ImGui::Dummy(ImVec2(0.0f, px(6.0f)));
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float size = px(13.0f);
        ImFont *font = fonts().display;
        ImDrawList *drawList = ImGui::GetWindowDrawList();
        const ImVec2 textSize = measureTracked(font, size, text, 0.14f);
        drawTracked(drawList, font, size, origin, toU32(color::textMuted), text, 0.14f);
        const float lineX = origin.x + textSize.x + px(10.0f);
        const float lineY = std::round(origin.y + size * 0.55f);
        if (lineX < origin.x + width)
        {
            drawList->AddLine(ImVec2(lineX, lineY), ImVec2(origin.x + width, lineY), toU32(color::hairline), 1.0f);
        }
        ImGui::Dummy(ImVec2(width, size + px(4.0f)));
    }

    bool menuItem(const char *label, bool highlighted, bool *hovered)
    {
        const float height = px(46.0f);
        const float width = ImGui::GetContentRegionAvail().x;
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const bool clicked = ImGui::InvisibleButton(label, ImVec2(width, height));
        const bool isHovered = ImGui::IsItemHovered();
        if (hovered != nullptr)
        {
            *hovered = isHovered;
        }
        if (isHovered)
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }

        const float t = animate(ImGui::GetItemID(), (highlighted || isHovered) ? 1.0f : 0.0f, 16.0f);
        ImDrawList *drawList = ImGui::GetWindowDrawList();
        ImFont *font = fonts().display;
        const float size = px(28.0f);
        const float textY = origin.y + (height - size) * 0.5f;

        const float barHeight = size * 0.62f * t;
        if (barHeight > 0.5f)
        {
            const float barY = origin.y + height * 0.5f;
            drawList->AddRectFilled(ImVec2(origin.x, barY - barHeight * 0.5f),
                                    ImVec2(origin.x + px(3.0f), barY + barHeight * 0.5f),
                                    toU32(color::accent, t), px(1.5f));
        }
        const ImVec4 textColour = mix(withAlpha(color::text, 0.62f), color::text, t);
        drawTracked(drawList, font, size, ImVec2(origin.x + px(18.0f) + px(8.0f) * t, textY),
                    toU32(textColour), label, 0.07f);
        return clicked;
    }

    bool button(const char *label, ButtonStyle style, float width)
    {
        ImFont *font = fonts().display;
        const float size = px(16.0f);
        const float tracking = 0.09f;
        const ImVec2 textSize = measureTracked(font, size, label, tracking);
        const float height = px(36.0f);
        if (width == 0.0f)
        {
            width = textSize.x + px(32.0f);
        }
        else if (width < 0.0f)
        {
            width = ImGui::GetContentRegionAvail().x;
        }

        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const bool clicked = ImGui::InvisibleButton(label, ImVec2(width, height));
        const bool hovered = ImGui::IsItemHovered();
        const bool held = ImGui::IsItemActive();
        const float t = animate(ImGui::GetItemID(), hovered ? 1.0f : 0.0f, 16.0f);
        if (hovered)
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }

        ImDrawList *drawList = ImGui::GetWindowDrawList();
        const ImVec2 max(origin.x + width, origin.y + height);
        const float rounding = px(5.0f);
        ImVec4 textColour = color::text;
        switch (style)
        {
        case ButtonStyle::Primary:
        {
            ImVec4 fill = mix(color::accent, color::accentBright, t);
            if (held)
            {
                fill = mix(fill, ImVec4(0.0f, 0.0f, 0.0f, 1.0f), 0.12f);
            }
            drawList->AddRectFilled(origin, max, toU32(fill), rounding);
            textColour = ImVec4(0.086f, 0.067f, 0.035f, 1.0f);
            break;
        }
        case ButtonStyle::Secondary:
            drawList->AddRectFilled(origin, max, toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.07f + 0.05f * t + (held ? 0.04f : 0.0f))), rounding);
            drawList->AddRect(origin, max, toU32(color::hairline), rounding);
            break;
        case ButtonStyle::Ghost:
            if (t > 0.001f)
            {
                drawList->AddRectFilled(origin, max, toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.05f * t)), rounding);
            }
            textColour = mix(color::textMuted, color::text, t);
            break;
        }

        drawTracked(drawList, font, size,
                    ImVec2(origin.x + (width - textSize.x) * 0.5f, origin.y + (height - size) * 0.5f),
                    toU32(textColour), label, tracking);
        return clicked;
    }

    bool sliderRow(const char *label, float *value, float minValue, float maxValue,
                   const char *format, const char *hint)
    {
        return sliderImpl(label, value, minValue, maxValue, format, hint, false);
    }

    bool sliderRowInt(const char *label, int *value, int minValue, int maxValue,
                      const char *format, const char *hint)
    {
        float asFloat = static_cast<float>(*value);
        const bool changed = sliderImpl(label, &asFloat, static_cast<float>(minValue),
                                        static_cast<float>(maxValue), format, hint, true);
        if (changed)
        {
            *value = static_cast<int>(std::lround(asFloat));
        }
        return changed;
    }

    bool toggleRow(const char *label, bool *value, const char *hint)
    {
        ImGui::PushID(label);
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = px(kRowHeight);

        // The whole row is the hit target; the label is drawn over it.
        const bool clicked = ImGui::InvisibleButton("##toggle", ImVec2(width, height));
        const bool hovered = ImGui::IsItemHovered();
        if (clicked)
        {
            *value = !*value;
            ImGui::MarkItemEdited(ImGui::GetItemID());
        }
        if (hovered)
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }

        const float on = animate(ImGui::GetItemID(), *value ? 1.0f : 0.0f, 18.0f);
        const float hover = animate(ImGui::GetID("##hover"), hovered ? 1.0f : 0.0f, 16.0f);

        Row wash;
        wash.origin = origin;
        wash.width = width;
        wash.height = height;
        drawRowHoverWash(wash, hover);
        ImGui::SetCursorScreenPos(origin);
        layoutRow(label, hint);

        ImDrawList *drawList = ImGui::GetWindowDrawList();
        const float switchWidth = px(38.0f);
        const float switchHeight = px(20.0f);
        const ImVec2 switchMin(origin.x + width - switchWidth, origin.y + (height - switchHeight) * 0.5f);
        const ImVec2 switchMax(switchMin.x + switchWidth, switchMin.y + switchHeight);
        drawList->AddRectFilled(switchMin, switchMax,
                                toU32(mix(ImVec4(1.0f, 1.0f, 1.0f, 0.14f + 0.04f * hover), color::accent, on)),
                                switchHeight * 0.5f);
        const float knobRadius = switchHeight * 0.5f - px(3.0f);
        const float knobTravel = switchWidth - switchHeight;
        const ImVec2 knobCentre(switchMin.x + switchHeight * 0.5f + knobTravel * on, switchMin.y + switchHeight * 0.5f);
        drawList->AddCircleFilled(knobCentre, knobRadius,
                                  toU32(mix(color::text, ImVec4(0.086f, 0.067f, 0.035f, 1.0f), on * 0.85f)), 20);

        ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + height));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::PopID();
        return clicked;
    }

    bool segmentedRow(const char *label, int *selected, const char *const *items, int itemCount,
                      const char *hint)
    {
        ImGui::PushID(label);
        const Row row = layoutRow(label, hint);
        ImDrawList *drawList = ImGui::GetWindowDrawList();

        const float height = px(30.0f);
        const float top = row.origin.y + (row.height - height) * 0.5f;
        const float inset = px(3.0f);
        const float rounding = px(6.0f);
        drawList->AddRectFilled(ImVec2(row.controlX, top), ImVec2(row.controlX + row.controlWidth, top + height),
                                toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.05f)), rounding);

        const float segmentWidth = (row.controlWidth - inset * 2.0f) / static_cast<float>(std::max(itemCount, 1));
        const ImGuiID indicatorId = ImGui::GetID("##indicator");
        const float indicator = animate(indicatorId, static_cast<float>(*selected), 18.0f);
        const float indicatorX = row.controlX + inset + indicator * segmentWidth;
        drawList->AddRectFilled(ImVec2(indicatorX, top + inset), ImVec2(indicatorX + segmentWidth, top + height - inset),
                                toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.13f)), rounding - inset * 0.5f);

        bool changed = false;
        ImFont *font = fonts().medium;
        const float size = px(type::body - 1.0f);
        for (int i = 0; i < itemCount; ++i)
        {
            ImGui::PushID(i);
            const float x = row.controlX + inset + segmentWidth * static_cast<float>(i);
            ImGui::SetCursorScreenPos(ImVec2(x, row.origin.y));
            if (ImGui::InvisibleButton("##segment", ImVec2(segmentWidth, row.height)) && *selected != i)
            {
                *selected = i;
                changed = true;
                ImGui::MarkItemEdited(ImGui::GetItemID());
            }
            const bool hovered = ImGui::IsItemHovered();
            if (hovered)
            {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
            const ImVec2 textSize = font->CalcTextSizeA(size, FLT_MAX, 0.0f, items[i]);
            const ImVec4 textColour = (*selected == i) ? color::text : (hovered ? mix(color::textMuted, color::text, 0.6f) : color::textMuted);
            drawList->AddText(font, size, ImVec2(std::round(x + (segmentWidth - textSize.x) * 0.5f), top + (height - size) * 0.5f),
                              toU32(textColour), items[i]);
            ImGui::PopID();
        }

        ImGui::SetCursorScreenPos(ImVec2(row.origin.x, row.origin.y + row.height));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::PopID();
        return changed;
    }

    void readoutRow(const char *label, const char *value, const ImVec4 *valueColour)
    {
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = px(24.0f);
        ImDrawList *drawList = ImGui::GetWindowDrawList();

        ImFont *labelFont = fonts().body;
        const float labelSize = px(type::body - 1.0f);
        drawList->AddText(labelFont, labelSize, ImVec2(origin.x, origin.y + (height - labelSize) * 0.5f),
                          toU32(color::textMuted), label);

        ImFont *valueFont = fonts().mono;
        const float valueSize = px(type::body - 2.0f);
        const ImVec2 valueExtent = valueFont->CalcTextSizeA(valueSize, FLT_MAX, 0.0f, value);
        drawList->AddText(valueFont, valueSize,
                          ImVec2(origin.x + width - valueExtent.x, origin.y + (height - valueSize) * 0.5f),
                          toU32(valueColour != nullptr ? *valueColour : color::text), value);
        ImGui::Dummy(ImVec2(width, height));
    }

    float drawKeycap(ImDrawList *drawList, ImVec2 position, const char *key, float alpha)
    {
        ImFont *font = fonts().monoMedium;
        const float size = px(12.5f);
        const ImVec2 textSize = font->CalcTextSizeA(size, FLT_MAX, 0.0f, key);
        const float height = px(22.0f);
        const float width = std::max(height, textSize.x + px(14.0f));
        const float rounding = px(4.0f);
        const ImVec2 max(position.x + width, position.y + height);

        drawList->AddRectFilled(ImVec2(position.x, position.y + px(1.5f)), ImVec2(max.x, max.y + px(1.5f)),
                                toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.35f), alpha), rounding);
        drawList->AddRectFilled(position, max, toU32(ImVec4(0.16f, 0.18f, 0.21f, 0.95f), alpha), rounding);
        drawList->AddRect(position, max, toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.12f), alpha), rounding);
        drawList->AddText(font, size,
                          ImVec2(std::round(position.x + (width - textSize.x) * 0.5f), position.y + (height - size) * 0.5f - px(0.5f)),
                          toU32(color::text, alpha), key);
        return width;
    }

    void keyHintRow(const char *keys, const char *action)
    {
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = px(30.0f);
        ImDrawList *drawList = ImGui::GetWindowDrawList();

        // Keys are separated by '|'; each becomes its own keycap.
        float x = origin.x;
        const float capY = origin.y + (height - px(22.0f)) * 0.5f;
        std::string key;
        for (const char *c = keys;; ++c)
        {
            if (*c == '|' || *c == '\0')
            {
                if (!key.empty())
                {
                    x += drawKeycap(drawList, ImVec2(x, capY), key.c_str()) + px(5.0f);
                    key.clear();
                }
                if (*c == '\0')
                {
                    break;
                }
                continue;
            }
            key.push_back(*c);
        }

        ImFont *font = fonts().body;
        const float size = px(type::body - 0.5f);
        const float actionX = std::max(x + px(8.0f), origin.x + px(150.0f));
        drawList->AddText(font, size, ImVec2(actionX, origin.y + (height - size) * 0.5f), toU32(color::textMuted), action);
        ImGui::Dummy(ImVec2(width, height));
    }

    void dimBackground(float alpha)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImDrawList *drawList = ImGui::GetWindowDrawList();
        const ImVec2 min = viewport->Pos;
        const ImVec2 max(viewport->Pos.x + viewport->Size.x, viewport->Pos.y + viewport->Size.y);
        drawList->AddRectFilled(min, max, toU32(ImVec4(0.016f, 0.024f, 0.035f, 0.62f), alpha));
        // Soft vignette so the centre card reads as the focus.
        const ImU32 clear = toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
        const ImU32 edge = toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.35f), alpha);
        const float band = viewport->Size.y * 0.35f;
        drawList->AddRectFilledMultiColor(min, ImVec2(max.x, min.y + band), edge, edge, clear, clear);
        drawList->AddRectFilledMultiColor(ImVec2(min.x, max.y - band), max, clear, clear, edge, edge);
    }

    void drawCornerBrackets(ImDrawList *drawList, ImVec2 centre, float half, ImU32 colour, float thickness,
                            float legFraction)
    {
        if (drawList == nullptr || !(half > 0.0f))
        {
            return;
        }
        const float leg = half * std::clamp(legFraction, 0.0f, 1.0f);
        const float left = centre.x - half;
        const float right = centre.x + half;
        const float top = centre.y - half;
        const float bottom = centre.y + half;
        // Each corner is one polyline, so the joint is mitred, not overlapped.
        const ImVec2 corners[4][3] = {
            {ImVec2(left + leg, top), ImVec2(left, top), ImVec2(left, top + leg)},
            {ImVec2(right - leg, top), ImVec2(right, top), ImVec2(right, top + leg)},
            {ImVec2(left + leg, bottom), ImVec2(left, bottom), ImVec2(left, bottom - leg)},
            {ImVec2(right - leg, bottom), ImVec2(right, bottom), ImVec2(right, bottom - leg)},
        };
        for (const auto &corner : corners)
        {
            drawList->AddPolyline(corner, 3, colour, ImDrawFlags_None, thickness);
        }
    }
}
