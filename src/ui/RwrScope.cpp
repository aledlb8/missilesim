#include "RwrScope.h"

#include "Theme.h"
#include "Widgets.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>

namespace missilesim::ui
{
    namespace
    {
        constexpr float kPi = 3.14159265f;
        // Urgency rings, as a fraction of the scope radius.
        constexpr float kSearchRing = 0.80f;
        constexpr float kTrackRing = 0.58f;
        constexpr float kMissileRing = 0.36f;

        // The enum is declared least to most urgent.
        int urgency(RwrKind kind)
        {
            return static_cast<int>(kind);
        }

        float ringFraction(RwrKind kind)
        {
            switch (kind)
            {
            case RwrKind::Search:
                return kSearchRing;
            case RwrKind::Track:
            case RwrKind::Launch:
                return kTrackRing;
            case RwrKind::Seeker:
            case RwrKind::Approach:
                return kMissileRing;
            }
            return kSearchRing;
        }

        ImVec4 tone(RwrKind kind)
        {
            switch (kind)
            {
            case RwrKind::Search:
                return color::info;
            case RwrKind::Track:
                return color::accent;
            case RwrKind::Launch:
            case RwrKind::Seeker:
            case RwrKind::Approach:
                return color::danger;
            }
            return color::info;
        }

        // Nose up, right wing to the right of the screen.
        ImVec2 onRing(ImVec2 centre, float radius, float azimuth)
        {
            return ImVec2(centre.x + std::sin(azimuth) * radius, centre.y - std::cos(azimuth) * radius);
        }

        void drawOwnship(ImDrawList *drawList, ImVec2 centre, float size, ImU32 colour, float thickness)
        {
            const ImVec2 wings[3] = {ImVec2(centre.x - size, centre.y + size * 0.30f), ImVec2(centre.x, centre.y - size * 0.15f),
                                     ImVec2(centre.x + size, centre.y + size * 0.30f)};
            drawList->AddLine(ImVec2(centre.x, centre.y - size), ImVec2(centre.x, centre.y + size * 0.85f), colour, thickness);
            drawList->AddPolyline(wings, 3, colour, ImDrawFlags_None, thickness);
            drawList->AddLine(ImVec2(centre.x - size * 0.42f, centre.y + size * 0.85f),
                              ImVec2(centre.x + size * 0.42f, centre.y + size * 0.85f), colour, thickness);
        }

        void drawDiamond(ImDrawList *drawList, ImVec2 centre, float half, ImU32 colour, float thickness)
        {
            drawList->AddQuad(ImVec2(centre.x, centre.y - half), ImVec2(centre.x + half, centre.y),
                              ImVec2(centre.x, centre.y + half), ImVec2(centre.x - half, centre.y), colour, thickness);
        }

        void drawGlyph(ImDrawList *drawList, ImVec2 centre, const char *glyph, ImU32 colour)
        {
            ImFont *font = fonts().display;
            if (font == nullptr)
            {
                return;
            }
            const float size = px(14.0f);
            const ImVec2 extent = font->CalcTextSizeA(size, FLT_MAX, 0.0f, glyph);
            drawList->AddText(font, size, ImVec2(std::round(centre.x - extent.x * 0.5f), std::round(centre.y - size * 0.52f)),
                              colour, glyph);
        }

        void drawThreat(ImDrawList *drawList, ImVec2 centre, float radius, const RwrThreat &threat, float time)
        {
            const float fade = std::clamp(threat.fade, 0.0f, 1.0f);
            if (fade <= 0.01f || !std::isfinite(threat.azimuthRad))
            {
                return;
            }

            // A launch or a missile flashes while it is being heard; a fading
            // symbol holds steady so it reads as a memory, not a live threat.
            float alpha = fade;
            const bool urgent = urgency(threat.kind) >= urgency(RwrKind::Launch);
            if (urgent && fade > 0.999f)
            {
                alpha *= 0.55f + 0.45f * (0.5f + 0.5f * std::sin(time * 2.0f * kPi * 3.0f));
            }
            const ImU32 colour = toU32(tone(threat.kind), alpha);
            const float thickness = std::max(px(1.5f), 1.0f);
            const ImVec2 at = onRing(centre, radius * ringFraction(threat.kind), threat.azimuthRad);

            if (urgent)
            {
                // A line from the aircraft says which way to look.
                const ImVec2 from = onRing(centre, px(13.0f), threat.azimuthRad);
                const ImVec2 to = onRing(centre, radius * ringFraction(threat.kind) - px(10.0f), threat.azimuthRad);
                drawList->AddLine(from, to, toU32(tone(threat.kind), alpha * 0.55f), std::max(px(1.2f), 1.0f));
            }

            switch (threat.kind)
            {
            case RwrKind::Search:
                drawGlyph(drawList, at, "F", colour);
                break;
            case RwrKind::Track:
                drawDiamond(drawList, at, px(9.5f), colour, thickness);
                drawGlyph(drawList, at, "F", colour);
                break;
            case RwrKind::Launch:
                drawDiamond(drawList, at, px(9.5f), colour, thickness);
                drawDiamond(drawList, at, px(13.0f), colour, thickness);
                drawGlyph(drawList, at, "F", colour);
                break;
            case RwrKind::Seeker:
                drawDiamond(drawList, at, px(9.5f), colour, thickness);
                drawGlyph(drawList, at, "M", colour);
                break;
            case RwrKind::Approach:
            {
                // Arrowhead pointing in at the aircraft.
                const float tip = px(8.0f);
                const float back = px(6.0f);
                const float wing = px(6.5f);
                const ImVec2 inward(-std::sin(threat.azimuthRad), std::cos(threat.azimuthRad)); // screen, toward the centre
                const ImVec2 side(-inward.y, inward.x);
                drawList->AddTriangleFilled(ImVec2(at.x + inward.x * tip, at.y + inward.y * tip),
                                            ImVec2(at.x - inward.x * back + side.x * wing, at.y - inward.y * back + side.y * wing),
                                            ImVec2(at.x - inward.x * back - side.x * wing, at.y - inward.y * back - side.y * wing),
                                            colour);
                break;
            }
            }
        }

        void drawCentred(ImDrawList *drawList, ImFont *font, float size, float centreX, float y, ImU32 colour, const char *text)
        {
            if (font == nullptr || text == nullptr || text[0] == '\0')
            {
                return;
            }
            const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0f, text).x;
            drawList->AddText(font, size, ImVec2(std::round(centreX - width * 0.5f), std::round(y)), colour, text);
        }
    }

    RwrKind mostUrgent(const RwrView &view, bool &any)
    {
        any = false;
        RwrKind top = RwrKind::Search;
        for (const RwrThreat &threat : view.threats)
        {
            if (threat.fade <= 0.01f)
            {
                continue;
            }
            if (!any || urgency(threat.kind) > urgency(top))
            {
                top = threat.kind;
            }
            any = true;
        }
        return top;
    }

    float drawRwrScope(ImDrawList *drawList, ImVec2 centre, float radius, const RwrView &view)
    {
        if (drawList == nullptr || !(radius > 4.0f))
        {
            return centre.y;
        }
        const Fonts &font = fonts();
        const float hair = std::max(px(1.0f), 1.0f);

        // Glass disc, the scale ring, and the two inner urgency rings.
        const float rim = radius + px(10.0f);
        drawList->AddCircleFilled(centre, rim, toU32(ImVec4(0.035f, 0.043f, 0.059f, 0.66f)), 64);
        drawList->AddCircle(centre, rim, toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.07f)), 64, hair);
        drawList->AddCircle(centre, radius, toU32(color::textFaint), 64, std::max(px(1.2f), 1.0f));
        drawList->AddCircle(centre, radius * kTrackRing, toU32(color::hairline), 48, hair);
        drawList->AddCircle(centre, radius * kMissileRing, toU32(color::hairline), 40, hair);

        // A tick every 30 degrees; the quarters are longer and the nose is bright.
        for (int index = 0; index < 12; ++index)
        {
            const float azimuth = static_cast<float>(index) * kPi / 6.0f;
            const float length = index % 3 == 0 ? px(7.0f) : px(4.0f);
            const ImVec4 &tickTone = index == 0 ? color::text : color::textFaint;
            drawList->AddLine(onRing(centre, radius - length, azimuth), onRing(centre, radius, azimuth), toU32(tickTone), hair);
        }
        drawOwnship(drawList, centre, px(7.0f), toU32(color::text), std::max(px(1.4f), 1.0f));

        // Least urgent first, so the most urgent symbol lands on top.
        std::vector<std::size_t> order(view.threats.size());
        std::iota(order.begin(), order.end(), std::size_t{0});
        std::stable_sort(order.begin(), order.end(), [&view](std::size_t a, std::size_t b) {
            return urgency(view.threats[a].kind) < urgency(view.threats[b].kind);
        });
        for (const std::size_t index : order)
        {
            drawThreat(drawList, centre, radius, view.threats[index], view.time);
        }

        bool any = false;
        const RwrKind top = mostUrgent(view, any);
        const char *status = "CLEAR";
        ImVec4 statusTone = color::positive;
        if (any)
        {
            switch (top)
            {
            case RwrKind::Search:
                status = "SEARCH";
                statusTone = color::info;
                break;
            case RwrKind::Track:
                status = "TRACKED";
                statusTone = color::accent;
                break;
            case RwrKind::Launch:
                status = "LAUNCH";
                statusTone = color::danger;
                break;
            case RwrKind::Seeker:
            case RwrKind::Approach:
                status = "MISSILE";
                statusTone = color::danger;
                break;
            }
        }

        // "<caption>  <status>", chaff and detail lines on one glass plate under the disc.
        const char *caption = view.caption != nullptr ? view.caption : "";
        char chaffLine[32];
        chaffLine[0] = '\0';
        if (view.chaff >= 0)
        {
            std::snprintf(chaffLine, sizeof(chaffLine), "CHAFF  %d", view.chaff);
        }
        const bool hasDetail = view.detail != nullptr && view.detail[0] != '\0';
        const float statusSize = px(14.0f);
        const float lineSize = px(12.5f);
        const float captionGap = px(8.0f);
        const float captionWidth = font.display != nullptr && caption[0] != '\0' ? measureTracked(font.display, statusSize, caption, 0.14f).x : 0.0f;
        const float statusWidth = font.display != nullptr ? measureTracked(font.display, statusSize, status, 0.14f).x : 0.0f;
        const float headWidth = captionWidth + (captionWidth > 0.0f ? captionGap : 0.0f) + statusWidth;
        float plateWidth = headWidth;
        if (font.mono != nullptr)
        {
            if (chaffLine[0] != '\0')
            {
                plateWidth = std::max(plateWidth, font.mono->CalcTextSizeA(lineSize, FLT_MAX, 0.0f, chaffLine).x);
            }
            if (hasDetail)
            {
                plateWidth = std::max(plateWidth, font.mono->CalcTextSizeA(lineSize, FLT_MAX, 0.0f, view.detail).x);
            }
        }
        plateWidth += px(28.0f);
        const float plateTop = centre.y + rim + px(8.0f);
        const float plateHeight = px(10.0f) + px(19.0f) + (chaffLine[0] != '\0' ? px(18.0f) : 0.0f) + (hasDetail ? px(18.0f) : 0.0f) +
                                  px(4.0f);
        const ImVec2 plateMin(std::round(centre.x - plateWidth * 0.5f), std::round(plateTop));
        const ImVec2 plateMax(std::round(centre.x + plateWidth * 0.5f), std::round(plateTop + plateHeight));
        drawList->AddRectFilled(plateMin, plateMax, toU32(ImVec4(0.035f, 0.043f, 0.059f, 0.66f)), px(8.0f));
        drawList->AddRect(plateMin, plateMax, toU32(ImVec4(1.0f, 1.0f, 1.0f, 0.07f)), px(8.0f), 0, hair);

        float y = plateTop + px(8.0f);
        if (font.display != nullptr)
        {
            const float x = std::round(centre.x - headWidth * 0.5f);
            if (captionWidth > 0.0f)
            {
                drawTracked(drawList, font.display, statusSize, ImVec2(x, y), toU32(color::textMuted), caption, 0.14f);
            }
            drawTracked(drawList, font.display, statusSize, ImVec2(x + headWidth - statusWidth, y), toU32(statusTone), status, 0.14f);
        }
        y += px(19.0f);
        if (chaffLine[0] != '\0')
        {
            drawCentred(drawList, font.mono, lineSize, centre.x, y, toU32(view.chaff > 0 ? color::textMuted : color::danger), chaffLine);
            y += px(18.0f);
        }
        if (hasDetail)
        {
            drawCentred(drawList, font.mono, lineSize, centre.x, y, toU32(color::textMuted), view.detail);
            y += px(18.0f);
        }
        return plateMax.y;
    }
}
