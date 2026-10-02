#pragma once

#include "sim/EntityId.h"

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    // What happened. The simulation reports outcomes as events instead of
    // leaving presentation code to infer them from object state (the old
    // "target inactive but not at the origin" hit test). Every event is stamped
    // with the step it happened in and the simulation time it refers to.
    enum class EventType : std::uint8_t
    {
        PlatformSpawned,        // subject: platform
        PlatformRemoved,        // subject: platform, taken out of play without being destroyed
        WeaponLaunched,         // subject: shot, other: designated target, value: launch speed (m/s), detail: LaunchKind
        MotorIgnition,          // subject: shot
        MotorBurnout,           // subject: shot
        FuzeArmed,              // subject: shot, value: distance flown (m)
        SeekerTargetChanged,    // subject: shot, other: new source platform (none: no source), detail: 1 when a decoy
        CountermeasureReleased, // subject: countermeasure, other: dispensing platform
        CountermeasureExpired,  // subject: countermeasure
        ClosestApproach,        // subject: shot, other: platform, value: miss distance (m); closing speed changed sign
        Impact,                 // subject: shot, other: platform struck, value: 1 armed / 0 unarmed
        Detonation,             // subject: shot, other: platform that triggered the fuze (or none), detail: DetonationTrigger
        Damage,                 // subject: platform, other: shot, value: damage applied (0..1 of structure)
        PlatformDestroyed,      // subject: platform, other: shot responsible (none: ground collision)
        ShotEnded,              // subject: shot, other: platform involved (or none), detail: ShotEndReason
        GroundCollision,        // subject: platform that flew into the ground
        PlatformRespawned,      // subject: platform, placed back in play by the scenario
    };

    enum class LaunchKind : std::uint8_t
    {
        Rail,       // leaves the launcher at the aircraft's velocity
        ColdLaunch, // ejected from a ground cell, motor lit in the air
        AirLaunch,  // custom round released clear of the ground, motor lit at once
    };

    enum class DetonationTrigger : std::uint8_t
    {
        Proximity,    // fuze saw a platform inside its radius
        Contact,      // armed round struck a platform
        Ground,       // armed round struck the ground
        SelfDestruct, // the round destroyed itself
    };

    // Why a shot stopped flying. Every shot ends exactly once with one reason.
    enum class ShotEndReason : std::uint8_t
    {
        None,
        ProximityDetonation, // warhead fired by the proximity fuze
        DirectHit,           // armed round struck a platform
        DudImpact,           // unarmed round struck a platform: kinetic damage only
        GroundImpact,        // flew into the ground
        SelfDestruct,        // seeker lost its target or the round timed out
        Overshot,            // passed the target with the range opening and no way back
        EnergyExhausted,     // motor out and too slow to fly
        LeftArena,           // left the simulated volume
        Removed,             // taken out by a scenario reset
    };

    struct SimEvent
    {
        std::uint64_t sequence = 0; // order of emission within the world, starting at 1
        std::uint64_t tick = 0;     // steps completed when it was emitted (the step that produced it, or the last one)
        double time = 0.0;          // simulation time the event refers to (s)
        EventType type = EventType::PlatformSpawned;
        EntityId subject;
        EntityId other;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        float value = 0.0f;
        std::uint8_t detail = 0;
    };

    const char *eventTypeName(EventType type);
    const char *shotEndReasonName(ShotEndReason reason);
    const char *detonationTriggerName(DetonationTrigger trigger);
    const char *launchKindName(LaunchKind kind);

    // One readable line, for logs and the harness report.
    std::string describeEvent(const SimEvent &event);

    // Exact, platform-independent text of an event (floats as hexadecimal), so
    // two runs compare bit for bit rather than to a printed precision.
    void appendCanonicalEvent(std::string &out, const SimEvent &event);

    // 64-bit FNV-1a over the canonical text of every event, in order.
    std::uint64_t traceHash(const std::vector<SimEvent> &events);
}
