#pragma once

#include <cstdint>
#include <functional>

namespace missilesim::sim
{
    // Stable identity of a simulation entity (platform, shot, countermeasure).
    // Ids are handed out by World in creation order and are never reused
    // within a world's lifetime, so an id held by a HUD row, a track or an
    // event can never silently start meaning a different object. 0 is "none".
    struct EntityId
    {
        std::uint32_t value = 0;

        constexpr bool valid() const { return value != 0; }
        constexpr explicit operator bool() const { return valid(); }

        friend constexpr bool operator==(EntityId a, EntityId b) { return a.value == b.value; }
        friend constexpr bool operator!=(EntityId a, EntityId b) { return a.value != b.value; }
        friend constexpr bool operator<(EntityId a, EntityId b) { return a.value < b.value; }
    };

    constexpr EntityId kNoEntity{};
}

template <>
struct std::hash<missilesim::sim::EntityId>
{
    std::size_t operator()(missilesim::sim::EntityId id) const noexcept
    {
        return std::hash<std::uint32_t>{}(id.value);
    }
};
