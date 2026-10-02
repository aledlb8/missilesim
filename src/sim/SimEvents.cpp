#include "SimEvents.h"

#include "sim/Random.h"

#include <cstdio>

namespace missilesim::sim
{
    const char *eventTypeName(EventType type)
    {
        switch (type)
        {
        case EventType::PlatformSpawned:
            return "platform-spawned";
        case EventType::PlatformRemoved:
            return "platform-removed";
        case EventType::WeaponLaunched:
            return "weapon-launched";
        case EventType::MotorIgnition:
            return "motor-ignition";
        case EventType::MotorBurnout:
            return "motor-burnout";
        case EventType::FuzeArmed:
            return "fuze-armed";
        case EventType::SeekerTargetChanged:
            return "seeker-target-changed";
        case EventType::CountermeasureReleased:
            return "countermeasure-released";
        case EventType::CountermeasureExpired:
            return "countermeasure-expired";
        case EventType::ClosestApproach:
            return "closest-approach";
        case EventType::Impact:
            return "impact";
        case EventType::Detonation:
            return "detonation";
        case EventType::Damage:
            return "damage";
        case EventType::PlatformDestroyed:
            return "platform-destroyed";
        case EventType::ShotEnded:
            return "shot-ended";
        case EventType::GroundCollision:
            return "ground-collision";
        case EventType::PlatformRespawned:
            return "platform-respawned";
        }
        return "unknown";
    }

    const char *shotEndReasonName(ShotEndReason reason)
    {
        switch (reason)
        {
        case ShotEndReason::None:
            return "none";
        case ShotEndReason::ProximityDetonation:
            return "proximity-detonation";
        case ShotEndReason::DirectHit:
            return "direct-hit";
        case ShotEndReason::DudImpact:
            return "dud-impact";
        case ShotEndReason::GroundImpact:
            return "ground-impact";
        case ShotEndReason::SelfDestruct:
            return "self-destruct";
        case ShotEndReason::Overshot:
            return "overshot";
        case ShotEndReason::EnergyExhausted:
            return "energy-exhausted";
        case ShotEndReason::LeftArena:
            return "left-arena";
        case ShotEndReason::Removed:
            return "removed";
        }
        return "unknown";
    }

    const char *detonationTriggerName(DetonationTrigger trigger)
    {
        switch (trigger)
        {
        case DetonationTrigger::Proximity:
            return "proximity";
        case DetonationTrigger::Contact:
            return "contact";
        case DetonationTrigger::Ground:
            return "ground";
        case DetonationTrigger::SelfDestruct:
            return "self-destruct";
        }
        return "unknown";
    }

    const char *launchKindName(LaunchKind kind)
    {
        switch (kind)
        {
        case LaunchKind::Rail:
            return "rail";
        case LaunchKind::ColdLaunch:
            return "cold-launch";
        case LaunchKind::AirLaunch:
            return "air-launch";
        }
        return "unknown";
    }

    namespace
    {
        const char *detailName(const SimEvent &event)
        {
            switch (event.type)
            {
            case EventType::ShotEnded:
                return shotEndReasonName(static_cast<ShotEndReason>(event.detail));
            case EventType::Detonation:
                return detonationTriggerName(static_cast<DetonationTrigger>(event.detail));
            case EventType::WeaponLaunched:
                return launchKindName(static_cast<LaunchKind>(event.detail));
            case EventType::SeekerTargetChanged:
                return event.detail != 0 ? "decoy" : "airframe";
            default:
                return nullptr;
            }
        }
    }

    std::string describeEvent(const SimEvent &event)
    {
        char line[256];
        const char *detail = detailName(event);
        std::snprintf(line, sizeof(line), "t=%9.3f  #%-6llu %-24s subj=%-4u other=%-4u value=%-10.3f pos=(%.1f, %.1f, %.1f)%s%s",
                      event.time, static_cast<unsigned long long>(event.tick), eventTypeName(event.type),
                      event.subject.value, event.other.value, static_cast<double>(event.value),
                      static_cast<double>(event.position.x), static_cast<double>(event.position.y),
                      static_cast<double>(event.position.z), detail != nullptr ? "  " : "", detail != nullptr ? detail : "");
        return line;
    }

    void appendCanonicalEvent(std::string &out, const SimEvent &event)
    {
        char line[384];
        std::snprintf(line, sizeof(line), "%llu|%llu|%a|%u|%u|%u|%a|%a|%a|%a|%a|%a|%a|%u\n",
                      static_cast<unsigned long long>(event.sequence), static_cast<unsigned long long>(event.tick), event.time,
                      static_cast<unsigned>(event.type), event.subject.value, event.other.value,
                      static_cast<double>(event.position.x), static_cast<double>(event.position.y),
                      static_cast<double>(event.position.z), static_cast<double>(event.velocity.x),
                      static_cast<double>(event.velocity.y), static_cast<double>(event.velocity.z),
                      static_cast<double>(event.value), static_cast<unsigned>(event.detail));
        out += line;
    }

    std::uint64_t traceHash(const std::vector<SimEvent> &events)
    {
        std::uint64_t hash = 0xcbf29ce484222325ull;
        std::string line;
        for (const SimEvent &event : events)
        {
            line.clear();
            appendCanonicalEvent(line, event);
            hash = fnv1a64(line, hash);
        }
        return hash;
    }
}
