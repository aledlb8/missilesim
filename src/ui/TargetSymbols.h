#pragma once

#include <imgui.h>

#include <cstdint>

// Symbols the fighter HUD draws over the world: the radar lock box, the heat
// seeker's circle, the radar's other contacts and the pilot's own spotting
// marks. The HUD projects; these only draw at the screen points they are given.
namespace missilesim::ui
{
    enum class LockStyle : std::uint8_t
    {
        Held,   // the beam is holding the lock
        Memory, // the beam lost it and the track is flying on its estimate
        Shoot,  // the selected weapon can fire on it
    };

    // Square box on the lock, its tag above and range and closing speed under it.
    void drawLockBox(ImDrawList *drawList, ImVec2 centre, LockStyle style, const char *range, const char *closing, float time);

    enum class SeekerMark : std::uint8_t
    {
        Search,     // looking along the nose
        Slaved,     // looking where the radar lock is
        Designated, // has its aircraft, no heat lock yet
        Locked,     // heat lock
    };

    // Circle where the heat seeker's head looks.
    void drawSeekerCircle(ImDrawList *drawList, ImVec2 centre, float radius, SeekerMark mark, float time);

    // Small square on one of the radar's other contacts. alpha dims weak tracks.
    void drawContactMark(ImDrawList *drawList, ImVec2 centre, float alpha);

    // Chevron over an aircraft the pilot can see, with its range above.
    void drawSpottingMark(ImDrawList *drawList, ImVec2 aircraft, const char *range);
}
