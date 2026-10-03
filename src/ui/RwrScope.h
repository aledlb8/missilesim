#pragma once

#include <imgui.h>

#include <cstdint>
#include <vector>

// Round radar-warning display. Callers pass only what the receivers heard.
namespace missilesim::ui
{
    // What a symbol means, least to most urgent.
    enum class RwrKind : std::uint8_t
    {
        Search,   // a radar sweeping past
        Track,    // a radar holding its beam on this aircraft
        Launch,   // that held beam is guiding a round
        Seeker,   // a missile's own radar is on this aircraft
        Approach, // a missile closing, seen by the approach warner
    };

    struct RwrThreat
    {
        RwrKind kind = RwrKind::Search;
        float azimuthRad = 0.0f; // toward the right wing
        float fade = 1.0f;       // 1 while heard, easing to 0 once it stops
    };

    struct RwrView
    {
        std::vector<RwrThreat> threats;
        const char *caption = "RWR";  // the receiver's name on the status plate
        int chaff = -1;               // < 0 hides the counter
        const char *detail = nullptr; // optional line under the status (SAM: range and closest approach)
        float time = 0.0f;            // seconds, drives the flashing
    };

    // Most urgent threat still showing. Search when nothing is.
    RwrKind mostUrgent(const RwrView &view, bool &any);

    // Scope centred on `centre`. The aircraft sits in the middle, nose up.
    // Search sits on the outer ring, a held beam inside it, a missile on the
    // inner ring. The status line and counters go below. Returns the bottom y
    // of everything drawn.
    float drawRwrScope(ImDrawList *drawList, ImVec2 centre, float radius, const RwrView &view);
}
