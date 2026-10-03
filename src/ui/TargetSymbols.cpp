#include "TargetSymbols.h"

#include "Theme.h"
#include "Widgets.h"

#include <algorithm>
#include <cmath>

namespace missilesim::ui
{
    namespace
    {
        constexpr float kTwoPi = 6.28318531f;

        float wave(float time, float hertz)
        {
            return 0.5f + 0.5f * std::sin(time * kTwoPi * hertz);
        }

        ImU32 shadow(float alpha)
        {
            return toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.35f * alpha));
        }

        // World labels sit on bright sky as often as on dark ground, so every
        // one carries a soft drop shadow.
        void shadowedText(ImDrawList *drawList, ImFont *font, float size, ImVec2 at, const ImVec4 &tone, float alpha,
                          const char *text)
        {
            const float offset = std::max(px(1.0f), 1.0f);
            drawList->AddText(font, size, ImVec2(at.x + offset, at.y + offset), toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.6f), alpha), text);
            drawList->AddText(font, size, at, toU32(tone, alpha), text);
        }

        void shadowedTracked(ImDrawList *drawList, ImFont *font, float size, ImVec2 at, const ImVec4 &tone, float alpha,
                             const char *text, float tracking)
        {
            const float offset = std::max(px(1.0f), 1.0f);
            drawTracked(drawList, font, size, ImVec2(at.x + offset, at.y + offset), toU32(ImVec4(0.0f, 0.0f, 0.0f, 0.6f), alpha),
                        text, tracking);
            drawTracked(drawList, font, size, at, toU32(tone, alpha), text, tracking);
        }
    }

    void drawLockBox(ImDrawList *drawList, ImVec2 centre, LockStyle style, const char *range, const char *closing, float time)
    {
        if (drawList == nullptr)
        {
            return;
        }
        const Fonts &font = fonts();
        const bool shoot = style == LockStyle::Shoot;
        const float alpha = style == LockStyle::Memory ? 0.5f : 1.0f;
        const ImVec4 &tone = shoot ? color::positive : color::accent;
        const ImU32 colour = toU32(tone, alpha);

        const float half = px(15.0f);
        const ImVec2 min(std::round(centre.x - half), std::round(centre.y - half));
        const ImVec2 max(std::round(centre.x + half), std::round(centre.y + half));
        drawList->AddRect(min, max, shadow(alpha), 0.0f, 0, px(3.6f));
        drawList->AddRect(min, max, colour, 0.0f, 0, px(1.8f));
        if (shoot)
        {
            // Shoot cue: an outer box breathing slowly around the lock.
            const float grow = px(4.0f) + px(2.0f) * wave(time, 0.8f);
            drawList->AddRect(ImVec2(min.x - grow, min.y - grow), ImVec2(max.x + grow, max.y + grow), toU32(tone, 0.55f), 0.0f, 0,
                              px(1.2f));
        }

        const char *tag = shoot ? "SHOOT" : (style == LockStyle::Memory ? "MEMORY" : "LOCK");
        if (font.display != nullptr)
        {
            shadowedTracked(drawList, font.display, px(12.5f), ImVec2(min.x, min.y - px(19.0f)), tone, alpha, tag, 0.16f);
        }
        if (font.monoMedium != nullptr && range != nullptr)
        {
            shadowedText(drawList, font.monoMedium, px(12.5f), ImVec2(min.x, max.y + px(5.0f)), tone, alpha, range);
        }
        if (font.mono != nullptr && closing != nullptr)
        {
            shadowedText(drawList, font.mono, px(12.0f), ImVec2(min.x, max.y + px(21.0f)), color::text, 0.85f * alpha, closing);
        }
    }

    void drawSeekerCircle(ImDrawList *drawList, ImVec2 centre, float radius, SeekerMark mark, float time)
    {
        if (drawList == nullptr || !(radius > 1.0f))
        {
            return;
        }
        const bool locked = mark == SeekerMark::Locked;
        const bool held = locked || mark == SeekerMark::Designated;
        const ImVec4 &tone = held ? color::accent : color::text;
        const float alpha = mark == SeekerMark::Search ? 0.8f : 1.0f;
        const float thickness = locked ? px(2.4f) : px(1.6f);

        drawList->AddCircle(centre, radius, shadow(1.0f), 48, thickness + px(1.8f));
        drawList->AddCircle(centre, radius, toU32(tone, alpha), 48, thickness);
        if (mark == SeekerMark::Slaved)
        {
            // Four short ticks: the head is pointed by the radar, not searching.
            const float inner = radius - px(4.0f);
            const float outer = radius + px(4.0f);
            const ImU32 colour = toU32(tone, alpha);
            drawList->AddLine(ImVec2(centre.x, centre.y - outer), ImVec2(centre.x, centre.y - inner), colour, px(1.4f));
            drawList->AddLine(ImVec2(centre.x, centre.y + inner), ImVec2(centre.x, centre.y + outer), colour, px(1.4f));
            drawList->AddLine(ImVec2(centre.x - outer, centre.y), ImVec2(centre.x - inner, centre.y), colour, px(1.4f));
            drawList->AddLine(ImVec2(centre.x + inner, centre.y), ImVec2(centre.x + outer, centre.y), colour, px(1.4f));
        }
        if (locked)
        {
            drawList->AddCircle(centre, radius + px(4.0f) + px(1.5f) * wave(time, 1.5f), toU32(tone, 0.35f), 48, px(1.2f));
            drawList->AddCircleFilled(centre, px(2.2f), toU32(tone), 12);
        }
    }

    void drawContactMark(ImDrawList *drawList, ImVec2 centre, float alpha)
    {
        if (drawList == nullptr)
        {
            return;
        }
        const float half = px(4.5f);
        const ImVec2 min(std::round(centre.x - half), std::round(centre.y - half));
        const ImVec2 max(std::round(centre.x + half), std::round(centre.y + half));
        drawList->AddRect(min, max, shadow(alpha), 0.0f, 0, px(3.0f));
        drawList->AddRect(min, max, toU32(color::info, alpha), 0.0f, 0, std::max(px(1.4f), 1.0f));
    }

    void drawSpottingMark(ImDrawList *drawList, ImVec2 aircraft, const char *range)
    {
        if (drawList == nullptr)
        {
            return;
        }
        const Fonts &font = fonts();
        const float lift = px(14.0f);
        const float half = px(5.0f);
        const ImVec2 tip(aircraft.x, aircraft.y - lift);
        const ImVec2 left(tip.x - half, tip.y - half * 1.3f);
        const ImVec2 right(tip.x + half, tip.y - half * 1.3f);
        drawList->AddTriangle(left, right, tip, shadow(1.0f), px(3.0f));
        drawList->AddTriangle(left, right, tip, toU32(color::text, 0.85f), px(1.4f));
        if (font.mono != nullptr && range != nullptr && range[0] != '\0')
        {
            const float size = px(11.5f);
            const float width = font.mono->CalcTextSizeA(size, FLT_MAX, 0.0f, range).x;
            shadowedText(drawList, font.mono, size, ImVec2(std::round(tip.x - width * 0.5f), std::round(left.y - px(16.0f))),
                         color::text, 0.85f, range);
        }
    }
}
