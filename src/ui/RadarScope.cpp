#include "RadarScope.h"

#include "Theme.h"
#include "Widgets.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace missilesim::ui
{
    namespace
    {
        constexpr float kRadToDeg = 57.2957795f;

        bool hasText(const char *text)
        {
            return text != nullptr && text[0] != '\0';
        }

        float monoWidth(ImFont *font, float size, const char *text)
        {
            if (font == nullptr || !hasText(text))
            {
                return 0.0f;
            }
            return font->CalcTextSizeA(size, FLT_MAX, 0.0f, text).x;
        }

        void formatKilometres(char *buffer, std::size_t size, float metres, bool withUnit)
        {
            const float kilometres = metres / 1000.0f;
            // Whole kilometres print bare, so a 20 km axis reads 5, 10, 15, 20.
            const bool whole = std::fabs(kilometres - std::round(kilometres)) < 0.005f;
            const char *pattern = "%.0f";
            if (kilometres < 0.995f)
            {
                pattern = "%.2f";
            }
            else if (kilometres < 9.95f && !whole)
            {
                pattern = "%.1f";
            }
            if (withUnit)
            {
                char number[16];
                std::snprintf(number, sizeof(number), pattern, static_cast<double>(kilometres));
                std::snprintf(buffer, size, "%s km", number);
            }
            else
            {
                std::snprintf(buffer, size, pattern, static_cast<double>(kilometres));
            }
        }

        void formatAzimuth(char *buffer, std::size_t size, float radians)
        {
            const float degrees = radians * kRadToDeg;
            if (!std::isfinite(degrees) || std::fabs(degrees) < 0.5f)
            {
                std::snprintf(buffer, size, "0");
                return;
            }
            std::snprintf(buffer, size, "%+.0f\xC2\xB0", static_cast<double>(degrees));
        }

        void formatElevation(char *buffer, std::size_t size, float radians)
        {
            if (!std::isfinite(radians))
            {
                buffer[0] = '\0';
                return;
            }
            const float degrees = radians * kRadToDeg;
            if (std::fabs(degrees) >= 10.0f)
            {
                std::snprintf(buffer, size, "EL %+.0f\xC2\xB0", static_cast<double>(degrees));
            }
            else
            {
                std::snprintf(buffer, size, "EL %+.1f\xC2\xB0", static_cast<double>(degrees));
            }
        }

        void formatAge(char *buffer, std::size_t size, double seconds)
        {
            if (!std::isfinite(seconds))
            {
                buffer[0] = '\0';
                return;
            }
            if (std::fabs(seconds) >= 9.95)
            {
                std::snprintf(buffer, size, "%.0fs", seconds);
            }
            else
            {
                std::snprintf(buffer, size, "%.1fs", seconds);
            }
        }

        // Positive azimuth is to the right. Range increases upward (smaller screen y).
        float axisX(ImVec2 plotMin, ImVec2 plotMax, float azimuth, float azimuthHalf)
        {
            const float span = plotMax.x - plotMin.x;
            return plotMin.x + (azimuth / azimuthHalf + 1.0f) * 0.5f * span;
        }

        float axisY(ImVec2 plotMin, ImVec2 plotMax, float range, float rangeScale)
        {
            const float span = plotMax.y - plotMin.y;
            return plotMax.y - (range / rangeScale) * span;
        }

        bool contactVisible(const ScopeContact &contact, float azimuthHalf, float rangeScale)
        {
            switch (contact.life)
            {
            case ScopeLife::Tentative:
            case ScopeLife::Confirmed:
            case ScopeLife::Coasting:
                break;
            default:
                return false;
            }
            if (!std::isfinite(contact.rangeM) || !std::isfinite(contact.azimuthRad))
            {
                return false;
            }
            if (contact.rangeM < 0.0f || contact.rangeM > rangeScale)
            {
                return false;
            }
            return contact.azimuthRad >= -azimuthHalf && contact.azimuthRad <= azimuthHalf;
        }

        // Underside bracket. This is launch quality, not the designation corners.
        void drawLaunchBracket(ImDrawList *drawList, float centreX, float top, float halfWidth, float depth, ImU32 colour, float thickness)
        {
            const float bottom = top + depth;
            const float left = centreX - halfWidth;
            const float right = centreX + halfWidth;
            drawList->AddLine(ImVec2(left, top), ImVec2(left, bottom), colour, thickness);
            drawList->AddLine(ImVec2(right, top), ImVec2(right, bottom), colour, thickness);
            drawList->AddLine(ImVec2(left, bottom), ImVec2(right, bottom), colour, thickness);
        }

        void drawContact(ImDrawList *drawList, ImVec2 centre, const ScopeContact &contact, ImFont *mono, float textSize)
        {
            const bool tentative = contact.life == ScopeLife::Tentative;
            const bool coasting = contact.life == ScopeLife::Coasting;
            const float half = tentative ? px(3.0f) : px(3.6f);
            const float bracketHalf = px(7.5f);
            const float thickness = std::max(px(1.25f), 1.0f);
            const float alpha = coasting ? 0.40f : 1.0f;
            const ImVec2 markMin(centre.x - half, centre.y - half);
            const ImVec2 markMax(centre.x + half, centre.y + half);

            if (tentative)
            {
                drawList->AddRect(markMin, markMax, toU32(color::textMuted), 0.0f, 0, thickness);
            }
            else
            {
                drawList->AddRectFilled(markMin, markMax, toU32(color::info, alpha * 0.9f));
                drawList->AddRect(markMin, markMax, toU32(color::info, alpha), 0.0f, 0, thickness);
            }

            if (contact.launchQuality)
            {
                const float launchHalf = contact.designated ? bracketHalf : half + px(2.0f);
                const float top = centre.y + (contact.designated ? bracketHalf : half) + px(1.5f);
                drawLaunchBracket(drawList, centre.x, top, launchHalf, px(4.0f), toU32(color::positive), thickness);
            }
            if (contact.designated)
            {
                drawCornerBrackets(drawList, centre, bracketHalf, toU32(color::accent), thickness, 0.46f);
            }
            if (coasting && mono != nullptr)
            {
                char age[24];
                formatAge(age, sizeof(age), contact.ageSeconds);
                if (age[0] != '\0')
                {
                    const float x = centre.x + (contact.designated ? bracketHalf : half) + px(4.0f);
                    const float y = std::round(centre.y - textSize * 0.5f);
                    drawList->AddText(mono, textSize, ImVec2(x, y), toU32(color::textMuted), age);
                }
            }
        }
    }

    void drawRadarScope(ImDrawList *drawList, ImVec2 topLeft, float sizePx, const RadarScopeView &view)
    {
        if (drawList == nullptr || !std::isfinite(sizePx) || sizePx <= 1.0f)
        {
            return;
        }

        const Fonts &font = fonts();
        const ImVec2 scopeMax(topLeft.x + sizePx, topLeft.y + sizePx);
        const float pad = std::min(px(8.0f), sizePx * 0.06f);
        const float rounding = std::min(px(8.0f), sizePx * 0.08f);
        const float hair = std::max(px(1.0f), 1.0f);
        drawList->AddRectFilled(topLeft, scopeMax, toU32(color::surface), rounding);
        drawList->AddRect(topLeft, scopeMax, toU32(color::hairline), rounding, 0, hair);

        const bool blocked = hasText(view.launchBlock);
        const bool lockLine = view.hasLock && std::isfinite(view.lockRangeM) && font.mono != nullptr;
        const float axisSize = px(11.5f);
        const float headerSize = px(12.0f);
        const float sentenceSize = px(13.5f);
        const float lockSize = px(12.5f);
        const float headerBand = headerSize + px(8.0f);
        const float azimuthBand = axisSize + px(6.0f);
        const float lockBand = lockLine ? lockSize + px(6.0f) : 0.0f;
        const float warnBand = blocked ? sentenceSize + px(6.0f) : 0.0f;

        const float rangeScale = view.rangeScaleM;
        const float azimuthHalf = std::fabs(view.azimuthHalfRad);
        char scaleLabel[24];
        scaleLabel[0] = '\0';
        if (std::isfinite(rangeScale) && rangeScale > 0.0f)
        {
            formatKilometres(scaleLabel, sizeof(scaleLabel), rangeScale, true);
        }

        float leftGutter = px(28.0f);
        if (scaleLabel[0] != '\0')
        {
            leftGutter = std::max(leftGutter, monoWidth(font.mono, axisSize, scaleLabel) + px(6.0f));
        }
        leftGutter = std::min(leftGutter, std::max(px(18.0f), sizePx * 0.36f));

        const bool showBars = view.barCount > 1 && view.barCount <= 12;
        const float rightGutter = showBars ? px(14.0f) : px(4.0f);
        const ImVec2 plotMin(topLeft.x + pad + leftGutter, topLeft.y + pad + headerBand);
        const ImVec2 plotMax(scopeMax.x - pad - rightGutter, scopeMax.y - pad - azimuthBand - lockBand - warnBand);
        const bool plotUsable = plotMax.x > plotMin.x + 8.0f && plotMax.y > plotMin.y + 8.0f && azimuthHalf > 1.0e-4f &&
                                std::isfinite(azimuthHalf) && rangeScale > 1.0e-3f && std::isfinite(rangeScale);

        char elevation[24];
        formatElevation(elevation, sizeof(elevation), view.beamElevationRad);
        char barLabel[16];
        barLabel[0] = '\0';
        if (view.barCount > 1 && view.bar >= 0)
        {
            // The readout is 1-based. Bar 0 is the bottom of the side column.
            // A held beam is not on a bar, so single-target track shows none.
            std::snprintf(barLabel, sizeof(barLabel), "%d/%d", view.bar + 1, view.barCount);
        }
        char readout[48];
        readout[0] = '\0';
        if (elevation[0] != '\0' && barLabel[0] != '\0')
        {
            std::snprintf(readout, sizeof(readout), "%s  %s", elevation, barLabel);
        }
        else if (elevation[0] != '\0')
        {
            std::snprintf(readout, sizeof(readout), "%s", elevation);
        }
        else if (barLabel[0] != '\0')
        {
            std::snprintf(readout, sizeof(readout), "%s", barLabel);
        }

        drawList->PushClipRect(topLeft, scopeMax, true);

        float modeWidth = 0.0f;
        if (hasText(view.modeLabel) && font.display != nullptr)
        {
            const float tracking = 0.14f;
            modeWidth = measureTracked(font.display, headerSize, view.modeLabel, tracking).x;
            const ImU32 modeColour = toU32(view.singleTargetTrack ? color::accent : color::textMuted);
            drawTracked(drawList, font.display, headerSize,
                        ImVec2(std::round(topLeft.x + pad), std::round(topLeft.y + pad)), modeColour, view.modeLabel, tracking);
        }
        if (font.mono != nullptr && readout[0] != '\0')
        {
            const float width = monoWidth(font.mono, axisSize, readout);
            const float x = scopeMax.x - pad - width;
            if (x >= topLeft.x + pad + modeWidth + px(8.0f))
            {
                const float y = std::round(topLeft.y + pad + (headerSize - axisSize) * 0.5f);
                ScopedFont mono(font.mono, 11.5f);
                drawList->AddText(font.mono, axisSize, ImVec2(std::round(x), y), toU32(color::textMuted), readout);
            }
        }

        if (plotUsable)
        {
            drawList->AddRect(plotMin, plotMax, toU32(color::hairline), 0.0f, 0, hair);
            const float boresight = std::round(axisX(plotMin, plotMax, 0.0f, azimuthHalf));
            drawList->AddLine(ImVec2(boresight, plotMin.y), ImVec2(boresight, plotMax.y), toU32(color::hairline), hair);

            if (font.mono != nullptr)
            {
                ScopedFont mono(font.mono, 11.5f);
                for (int i = 1; i <= 4; ++i)
                {
                    const float fraction = static_cast<float>(i) / 4.0f;
                    const float y = std::round(axisY(plotMin, plotMax, fraction * rangeScale, rangeScale));
                    if (i < 4)
                    {
                        drawList->AddLine(ImVec2(plotMin.x, y), ImVec2(plotMax.x, y), toU32(color::hairline), hair);
                    }
                    char label[24];
                    formatKilometres(label, sizeof(label), fraction * rangeScale, i == 4);
                    const float width = monoWidth(font.mono, axisSize, label);
                    const float x = std::max(topLeft.x + px(2.0f), plotMin.x - px(4.0f) - width);
                    drawList->AddText(font.mono, axisSize, ImVec2(std::round(x), std::round(y - axisSize * 0.5f)),
                                      toU32(color::textMuted), label);
                }

                for (int i = 0; i <= 4; ++i)
                {
                    const float fraction = static_cast<float>(i) / 4.0f;
                    const float azimuth = -azimuthHalf + fraction * (2.0f * azimuthHalf);
                    const float x = std::round(axisX(plotMin, plotMax, azimuth, azimuthHalf));
                    const float tick = i == 2 ? px(7.0f) : px(4.0f);
                    drawList->AddLine(ImVec2(x, plotMax.y), ImVec2(x, plotMax.y - tick), toU32(color::textFaint), hair);
                    if (i % 2 != 0)
                    {
                        continue;
                    }
                    char label[16];
                    formatAzimuth(label, sizeof(label), azimuth);
                    const float width = monoWidth(font.mono, axisSize, label);
                    drawList->AddText(font.mono, axisSize,
                                      ImVec2(std::round(x - width * 0.5f), std::round(plotMax.y + px(3.0f))),
                                      toU32(color::textMuted), label);
                }
            }
            else
            {
                for (int i = 1; i < 4; ++i)
                {
                    const float fraction = static_cast<float>(i) / 4.0f;
                    const float y = std::round(axisY(plotMin, plotMax, fraction * rangeScale, rangeScale));
                    drawList->AddLine(ImVec2(plotMin.x, y), ImVec2(plotMax.x, y), toU32(color::hairline), hair);
                }
            }

            if (showBars)
            {
                const int count = view.barCount;
                const float slot = (plotMax.y - plotMin.y) / static_cast<float>(count);
                const float x0 = plotMax.x + px(4.0f);
                const float x1 = std::min(scopeMax.x - px(3.0f), x0 + px(4.0f));
                if (slot >= px(4.0f) && x1 > x0 + 1.0f)
                {
                    const float gap = std::min(px(1.5f), slot * 0.22f);
                    for (int bar = 0; bar < count; ++bar)
                    {
                        const float yBottom = plotMax.y - slot * static_cast<float>(bar);
                        const float yTop = yBottom - slot;
                        const bool active = bar == view.bar;
                        const ImVec4 pip = active ? (view.singleTargetTrack ? color::accent : color::info) : color::textFaint;
                        drawList->AddRectFilled(ImVec2(x0, yTop + gap), ImVec2(x1, yBottom - gap),
                                                toU32(pip, active ? 1.0f : 0.45f), px(1.0f));
                    }
                }
            }

            drawList->PushClipRect(plotMin, plotMax, true);
            for (int pass = 0; pass < 2; ++pass)
            {
                const bool designatedPass = pass == 1;
                for (const ScopeContact &contact : view.contacts)
                {
                    if (contact.designated != designatedPass || !contactVisible(contact, azimuthHalf, rangeScale))
                    {
                        continue;
                    }
                    const ImVec2 centre(std::round(axisX(plotMin, plotMax, contact.azimuthRad, azimuthHalf)),
                                        std::round(axisY(plotMin, plotMax, contact.rangeM, rangeScale)));
                    if (contact.hasTrend && contact.life != ScopeLife::Tentative && std::isfinite(contact.trendRangeM) &&
                        std::isfinite(contact.trendAzimuthRad))
                    {
                        // Heading line toward where the contact is going, capped
                        // so a fast crosser does not streak across the plot.
                        const ImVec2 ahead(axisX(plotMin, plotMax, contact.trendAzimuthRad, azimuthHalf),
                                           axisY(plotMin, plotMax, contact.trendRangeM, rangeScale));
                        const float dx = ahead.x - centre.x;
                        const float dy = ahead.y - centre.y;
                        const float length = std::sqrt(dx * dx + dy * dy);
                        const float shown = std::min(length, px(16.0f));
                        if (length > px(2.0f))
                        {
                            const float alpha = contact.life == ScopeLife::Coasting ? 0.40f : 0.85f;
                            const ImVec4 &lineTone = contact.designated ? color::accent : color::info;
                            drawList->AddLine(centre, ImVec2(centre.x + dx / length * shown, centre.y + dy / length * shown),
                                              toU32(lineTone, alpha), std::max(px(1.25f), 1.0f));
                        }
                    }
                    drawContact(drawList, centre, contact, font.mono, axisSize);
                }
            }
            drawList->PopClipRect();

            if (std::isfinite(view.beamAzimuthRad))
            {
                const float beamAzimuth = std::clamp(view.beamAzimuthRad, -azimuthHalf, azimuthHalf);
                const float x = std::round(axisX(plotMin, plotMax, beamAzimuth, azimuthHalf));
                const ImU32 colour = toU32(view.singleTargetTrack ? color::accent : color::text);
                const float thickness = view.singleTargetTrack ? px(1.8f) : px(1.3f);
                const float halfW = px(4.5f);
                const float y0 = plotMax.y;
                const float y1 = plotMax.y - px(9.0f);
                drawList->AddLine(ImVec2(x - halfW, y0), ImVec2(x - halfW, y1), colour, thickness);
                drawList->AddLine(ImVec2(x + halfW, y0), ImVec2(x + halfW, y1), colour, thickness);
                drawList->AddLine(ImVec2(x - halfW, y1), ImVec2(x + halfW, y1), colour, thickness);
            }
        }

        if (lockLine)
        {
            // Lock readout: range on the left, closing speed on the right.
            const float y = std::round(scopeMax.y - pad - warnBand - lockSize);
            char range[32];
            formatKilometres(range, sizeof(range), view.lockRangeM, true);
            char left[48];
            std::snprintf(left, sizeof(left), "LOCK  %s", range);
            char closing[32];
            std::snprintf(closing, sizeof(closing), "%+.0f m/s", std::isfinite(view.lockClosingMps) ? static_cast<double>(view.lockClosingMps) : 0.0);
            ScopedFont mono(font.mono, 12.5f);
            drawList->AddText(font.mono, lockSize, ImVec2(std::round(topLeft.x + pad), y), toU32(color::accent), left);
            const float closingWidth = monoWidth(font.mono, lockSize, closing);
            drawList->AddText(font.mono, lockSize, ImVec2(std::round(scopeMax.x - pad - closingWidth), y),
                              toU32(view.lockClosingMps >= 0.0f ? color::text : color::textMuted), closing);
        }

        if (blocked)
        {
            ImFont *sentenceFont = font.body != nullptr ? font.body : font.mono;
            if (sentenceFont != nullptr)
            {
                const float y = std::round(scopeMax.y - pad - sentenceSize);
                ScopedFont sentence(sentenceFont, 13.5f);
                drawList->PushClipRect(ImVec2(topLeft.x + pad, y), ImVec2(scopeMax.x - pad, scopeMax.y), true);
                drawList->AddText(sentenceFont, sentenceSize, ImVec2(std::round(topLeft.x + pad), y), toU32(color::danger),
                                  view.launchBlock);
                drawList->PopClipRect();
            }
        }

        drawList->PopClipRect();
    }
}
