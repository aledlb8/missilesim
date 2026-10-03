#pragma once

#include <imgui.h>

#include <cstdint>
#include <vector>

// Rectangular radar scope. Callers pass only tracks the player is allowed to know.
namespace missilesim::ui
{
    enum class ScopeLife : uint8_t
    {
        Tentative,
        Confirmed,
        Coasting,
        Lost
    };

    struct ScopeContact
    {
        uint32_t trackId = 0;
        float rangeM = 0.0f;
        float azimuthRad = 0.0f; // right of the nose
        ScopeLife life = ScopeLife::Tentative;
        double ageSeconds = 0.0;
        bool designated = false;
        bool launchQuality = false;
        // Where the contact will be a few seconds on: its heading line.
        bool hasTrend = false;
        float trendRangeM = 0.0f;
        float trendAzimuthRad = 0.0f;
    };

    struct RadarScopeView
    {
        float rangeScaleM = 20000.0f;
        float azimuthHalfRad = 1.0472f; // display limit, about 60 deg
        float beamAzimuthRad = 0.0f;
        float beamElevationRad = 0.0f;
        bool singleTargetTrack = false;
        int bar = 0;
        int barCount = 1;
        const char *launchBlock = nullptr; // nullptr or empty means the weapon can launch
        const char *modeLabel = "SEARCH";
        std::vector<ScopeContact> contacts;
        // The lock's range and closing speed under the plot. Hidden without a lock.
        bool hasLock = false;
        float lockRangeM = 0.0f;
        float lockClosingMps = 0.0f; // positive while the range is shrinking
    };

    // Square B-scope. Range increases upward. Positive azimuth is to the right.
    void drawRadarScope(ImDrawList *drawList, ImVec2 topLeft, float sizePx, const RadarScopeView &view);
}
