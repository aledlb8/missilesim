#pragma once

// The simulated engagement, independent of windows, rendering and audio.
//
// World owns every simulated entity: the physics engine and atmosphere, the
// player's fighter, the target aircraft, their flares, the player's weapon
// stations and every shot in the air. It advances on a fixed step and reports
// what happened as time-stamped events (sim/SimEvents.h). The game presents
// those events; the headless harness (tools/engagement_harness) records them.
// Both drive this same class, so what the harness checks is what the game runs.
//
// Ownership and lifetime rules:
//  - Every entity has a stable EntityId. Ids are never reused until restart().
//  - Removing a target, flare or shot clears every reference other entities
//    hold to it in the same call, so no missile or station keeps a dangling
//    pointer.
//  - A shot ends exactly once, with one ShotEndReason. Damage, destruction and
//    the end of a shot are separate events; a platform is destroyed once.
//  - New flares join the world at the end of the step that released them
//    (a documented step boundary), not after a rendered frame.

#include "sim/EntityId.h"
#include "sim/Random.h"
#include "sim/SimEvents.h"
#include "sim/SimulationConfig.h"
#include "sim/Terrain.h"
#include "objects/Target.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class Fighter;
class Flare;
class Missile;
class PhysicsEngine;

namespace missilesim::sim
{
    enum class Team : std::uint8_t
    {
        Blue, // the player's side
        Red,  // the target aircraft
    };

    enum class PlatformKind : std::uint8_t
    {
        Fighter,
        Target,
    };

    enum class PlayerRole : std::uint8_t
    {
        Sam,     // ground launcher firing the custom cold-launch round
        Fighter, // F-16 carrying catalog Fox 2 rounds on the wingtip rails
    };

    // Read-only state of a platform at one simulation time, built from the
    // live flight model so consumers never reach into it.
    struct PlatformSnapshot
    {
        EntityId id;
        PlatformKind kind = PlatformKind::Target;
        Team team = Team::Red;
        double time = 0.0;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 forward{0.0f, 0.0f, 1.0f}; // body nose axis
        glm::vec3 up{0.0f, 1.0f, 0.0f};      // body up axis
        float radius = 0.0f;                 // collision radius (m)
        float throttle = 0.0f;               // 0..1 of military power
        bool afterburner = false;
        float health = 1.0f; // remaining structure, 0..1
        bool alive = false;
    };

    // The custom (SAM) round: control-panel values plus the airframe curves
    // from the simulation config. Validated when a round is built.
    struct CustomRoundSpec
    {
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f, 0.0f, 50.0f};
        float dryMass = 100.0f;
        float dragCoefficient = 0.1f;
        float crossSectionalArea = 0.1f;
        float liftCoefficient = 0.1f;
        float aspectRatio = 18.0f;
        float oswaldEfficiency = 0.8f;
        float maxLiftCoefficient = 20.0f;
        float maxLoadFactorG = 40.0f;
        std::vector<glm::vec2> machDragMultiplier;

        float thrust = 10000.0f;
        float fuelMass = 100.0f;
        float fuelConsumptionRate = 0.5f;
        float nozzleExitArea = 0.01f;
        float nozzleExitPressure = 101325.0f;

        bool guidanceEnabled = true;
        float navigationGain = 4.0f;
        float maxSteeringForce = 20000.0f;
        float trackingAngleDegrees = 85.0f;
        float proximityFuseRadius = 18.0f;
        float countermeasureResistance = 0.65f;
        bool terrainAvoidanceEnabled = true;
        float terrainClearance = 90.0f;
        float terrainLookAheadTime = 6.0f;

        // Height of the round's origin above the pad while it stands in its
        // cell. Comes from the missile mesh (Renderer::getMissileGroundRestOffset).
        float padRestHeight = 1.6f;
    };

    CustomRoundSpec customRoundSpecFromConfig(const SimulationConfig &config);

    // Why a launch request was refused, in words a player can act on.
    enum class LaunchBlock : std::uint8_t
    {
        None,
        NoLauncher,        // the role has no launcher (fighter not spawned)
        NoRound,           // stations empty
        Reloading,         // next round not ready yet
        SeekerCaged,       // seeker not uncaged, so nothing can be designated
        NoDesignation,     // no target in the seeker's designation cone
        NeedsInfraredLock, // lock-before-launch round without an infrared lock
    };

    const char *launchBlockMessage(LaunchBlock block);

    struct LaunchResult
    {
        LaunchBlock block = LaunchBlock::None;
        EntityId shot;

        bool launched() const { return block == LaunchBlock::None && shot.valid(); }
    };

    // A round put into the air by the scenario rather than by the player's
    // stores: a scripted threat, or a test body. It belongs to the scenario
    // site of the given team.
    struct ScriptedLaunch
    {
        Team team = Team::Red;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        // Catalog Fox 2 id; empty fires the custom round (current spec) with
        // its motor cold and guidance off, an unguided body.
        std::string fox2Id;
        // Designated target (a target aircraft), none for an unguided round.
        EntityId target;
    };

    // One hardpoint or launch cell.
    struct Station
    {
        int index = 0;
        float side = 0.0f;              // fighter rails: +1 right wingtip, -1 left; 0 for the SAM cell
        std::unique_ptr<Missile> round; // the loaded round (null when empty)
        int reserve = 0;                // rounds left to reload here; kUnlimitedReserve = sandbox magazine
        double readyTime = 0.0;         // simulation time the next round is loaded
    };

    // Cold-launch program of the custom SAM round: a soft ejection charge
    // lobs the round clear of the cell, the motor lights in the air at boost
    // thrust, the round pitches over toward the target at a capped rate, and
    // guidance takes over once the flight path is inside the seeker cone.
    struct ColdLaunchProgram
    {
        bool active = false;
        bool motorIgnited = false;
        bool guidanceArmed = false;
        bool boostComplete = false;
        bool restoreGuidanceEnabled = true;
        float elapsed = 0.0f;
        float ignitionDelay = 0.85f;     // eject coast before the motor lights (s)
        float thrustRampDuration = 0.30f; // ignition -> full throttle (s)
        float guidanceArmDelay = 1.30f;   // backstop: guidance is armed by here (s)
        float boostDuration = 1.5f;       // high-thrust booster after ignition (s)
        float sustainThrust = 10000.0f;   // configured motor thrust restored after the boost (N)
        glm::vec3 ejectDirection{0.0f, 1.0f, 0.0f};
        glm::vec3 launchDirection{0.0f, 0.0f, 1.0f};
        glm::vec3 aimDirection{0.0f, 1.0f, 0.0f};
    };

    struct Shot
    {
        EntityId id;
        EntityId owner; // launching platform or launcher site
        Team team = Team::Blue;
        LaunchKind launchKind = LaunchKind::Rail;
        EntityId designatedTarget; // target at launch
        EntityId seekerSource;     // platform the seeker currently follows (none: decoy or nothing)
        bool seekerOnDecoy = false;
        std::unique_ptr<Missile> missile;
        ColdLaunchProgram coldLaunch;

        double launchTime = 0.0;
        float flightTime = 0.0f;
        glm::vec3 launchPosition{0.0f};
        glm::vec3 stepStartPosition{0.0f};
        bool motorBurning = false;
        bool fuzeArmed = false;

        // Closest approach to the designated target over the whole flight, and
        // the closing speed last step (its sign change marks a pass).
        float closestApproach = -1.0f; // < 0: not measured yet
        float lastClosingSpeed = 0.0f;
        bool closingSampled = false;

        ShotEndReason endReason = ShotEndReason::None;
        EntityId endOther;
        glm::vec3 endPosition{0.0f};

        bool ended() const { return endReason != ShotEndReason::None; }
    };

    class World
    {
    public:
        static constexpr int kUnlimitedReserve = -1;

        World(const SimulationConfig &config, std::uint64_t seed);
        ~World();

        World(const World &) = delete;
        World &operator=(const World &) = delete;

        // Clears every entity, event and the clock, and re-seeds the random
        // streams. Entity ids start again from 1. The player role and round
        // specs are kept; stations are reloaded.
        void restart(std::uint64_t seed);

        // ---- Clock --------------------------------------------------------------
        void step();
        std::uint64_t tick() const { return m_tick; }
        double time() const { return static_cast<double>(m_tick) * static_cast<double>(m_fixedStep); }
        float fixedStep() const { return m_fixedStep; }
        std::uint64_t seed() const { return m_random.seed(); }

        // ---- Events -------------------------------------------------------------
        // Events since the previous call, for presentation.
        std::vector<SimEvent> drainEvents();
        // Full log since restart when recording is on (the harness turns it on).
        void setTraceRecording(bool enabled) { m_recordTrace = enabled; }
        const std::vector<SimEvent> &trace() const { return m_trace; }

        // ---- Environment -------------------------------------------------------
        PhysicsEngine &physics() { return *m_physics; }
        const PhysicsEngine &physics() const { return *m_physics; }
        // The terrain's base height (the launch site and the altitude datum).
        float groundLevel() const;
        // The one ground surface: physics contacts, the fighter's crash test,
        // the AI's height floor and line of sight all read it, and the
        // renderer meshes it.
        const Terrain &terrain() const { return *m_terrain; }
        std::shared_ptr<const Terrain> sharedTerrain() const { return m_terrain; }
        // Replaces the terrain everywhere at once. Aircraft already placed are
        // not moved; follow with respawnTargets (or restart) for a fresh field.
        void setTerrain(const TerrainConfig &config);

        // ---- Platforms -----------------------------------------------------------
        Fighter *fighter() { return m_fighter.get(); }
        const Fighter *fighter() const { return m_fighter.get(); }
        EntityId fighterId() const { return m_fighterId; }
        // Puts the fighter at the origin at the first target's altitude, pointed
        // at it, at a medium-altitude cruise speed (a scenario choice).
        void placeFighterAtEngagement();

        const std::vector<std::unique_ptr<Target>> &targets() const { return m_targets; }
        Target *findTarget(EntityId id) const;
        EntityId targetIdOf(const Target *target) const;
        bool targetAlive(EntityId id) const;
        bool anyTargetAlive() const;

        void setTargetAIConfig(const TargetAIConfig &config);
        const TargetAIConfig &targetAIConfig() const { return m_targetAIConfig; }
        void applyTargetAIConfigToAll();

        EntityId spawnTarget(const glm::vec3 &position, float radius);
        // Clears the target field (and its flares) and spawns `count` aircraft
        // at seeded random stations around the launcher.
        void respawnTargets(int count);
        void removeTarget(EntityId id);
        void clearTargets();

        float health(EntityId platform) const;
        bool platformAlive(EntityId platform) const;
        bool snapshot(EntityId platform, PlatformSnapshot &out) const;
        std::vector<PlatformSnapshot> platformSnapshots() const;

        // ---- Countermeasures ---------------------------------------------------
        const std::vector<std::unique_ptr<Flare>> &flares() const { return m_flares; }
        EntityId flareIdOf(const Flare *flare) const;

        // ---- Player weapons --------------------------------------------------
        PlayerRole role() const { return m_role; }
        void setRoleSam(const CustomRoundSpec &spec);
        void setRoleFighter(const std::string &fox2Id);
        // Replaces the round type on the loaded rails (rounds in the air keep theirs).
        void selectFox2(const std::string &fox2Id);
        const std::string &fox2Id() const { return m_fox2Id; }
        // Stores a card id and, if the fighter already exists, swaps its flight
        // model in place. The mesh stays models/jet.obj. Returns false when the
        // id is not in the catalog.
        bool selectAircraft(const std::string &aircraftId);
        const std::string &aircraftId() const { return m_aircraftId; }
        // Updates the custom round; with reloadNow the loaded round is rebuilt.
        void setCustomRoundSpec(const CustomRoundSpec &spec, bool reloadNow);
        const CustomRoundSpec &customRoundSpec() const { return m_customSpec; }
        void rearm();

        const std::vector<Station> &stations() const { return m_stations; }
        int selectedStation() const { return m_selectedStation; }
        // The round that fires next (null while reloading or empty).
        Missile *readyRound();
        const Missile *readyRound() const;
        int roundsRemaining() const;

        void setSeekerUncaged(bool uncaged);
        bool seekerUncaged() const { return m_seekerUncaged; }
        // SAM: the launcher's seeker cue designation (a Fox 2 designates itself).
        void designate(EntityId target);
        // SAM: points the ready round like the launcher would fire it.
        void aimReadyRound(const glm::vec3 &fallbackAim);
        // Why launch() would refuse right now, without launching.
        LaunchBlock launchClearance() const;
        LaunchResult launch(const glm::vec3 &fallbackAim);

        // ---- Shots -------------------------------------------------------------
        const std::vector<std::unique_ptr<Shot>> &shots() const { return m_shots; }
        const Shot *findShot(EntityId id) const;
        EntityId shotIdOf(const Missile *missile) const;
        // Ends every shot in the air (reason Removed).
        void removeAllShots();
        EntityId launchScripted(const ScriptedLaunch &launch);

        // ---- Rendering support -------------------------------------------------
        // Blends every owned object's last two step states (render interpolation).
        void setRenderBlend(float alpha);

    private:
        struct PlatformRecord
        {
            EntityId id;
            PlatformKind kind = PlatformKind::Target;
            Team team = Team::Red;
            float health = 1.0f;
            bool alive = true;
        };

        struct PlatformMotion
        {
            EntityId id;
            glm::vec3 start{0.0f};
            glm::vec3 end{0.0f};
            float radius = 0.0f;
        };

        EntityId allocateId();
        SimEvent makeEvent(EventType type, EntityId subject, EntityId other = kNoEntity) const;
        void publish(SimEvent event);
        PlatformRecord *record(EntityId id);
        const PlatformRecord *record(EntityId id) const;

        // Step stages, in order.
        void beginStep();
        void stepFighter();
        void stepStations();
        void stepColdLaunches();
        void stepShotBookkeeping(Shot &shot);
        void resolveShotOutcomes(const std::vector<PlatformMotion> &platforms);
        void detonate(Shot &shot, const glm::vec3 &point, float stepFraction, DetonationTrigger trigger,
                      EntityId triggeringPlatform, const std::vector<PlatformMotion> &platforms);
        void applyDamage(EntityId platform, float amount, EntityId cause, const glm::vec3 &point);
        void destroyPlatform(EntityId platform, EntityId cause, const glm::vec3 &point);
        void endShot(Shot &shot, ShotEndReason reason, EntityId other, const glm::vec3 &position, double eventTime);
        void retireEndedShots();
        void collectFlareLaunches();
        void retireExpiredFlares();

        // Stations and rounds.
        std::unique_ptr<Missile> buildCustomRound() const;
        std::unique_ptr<Missile> buildFox2Round() const;
        void reloadStations();
        void positionOnRail(Station &station) const;
        void refreshPrelaunch(Station &station);
        LaunchBlock clearanceFor(const Station &station) const;
        LaunchResult launchSam(Station &station, const glm::vec3 &fallbackAim);
        LaunchResult launchFox2(Station &station);
        void advanceSelectedStation();
        // The station launch() fires from: the selected one, or the next loaded one.
        int stationToFire() const;
        glm::vec3 samLaunchDirection(const Missile &round, const Target *locked, const glm::vec3 &fallbackAim) const;
        void updateColdLaunch(Shot &shot, float dt);

        void spawnFighter();
        void removeFighter();
        void forgetTargetEverywhere(const Target *target);
        void forgetFlareEverywhere(const Flare *flare);
        std::vector<Target *> aliveTargets() const;
        float missileBodyRadius(const Missile &missile) const;
        float lethalRadius(const Missile &missile) const;

        SimulationConfig m_config;
        float m_fixedStep = 0.01f;
        std::uint64_t m_tick = 0;
        RandomStreams m_random;
        RandomStream m_targetSpawnRandom;
        std::uint32_t m_nextId = 1;
        std::uint64_t m_nextEventSequence = 1;
        std::vector<SimEvent> m_pending;
        std::vector<SimEvent> m_trace;
        bool m_recordTrace = false;

        std::unique_ptr<PhysicsEngine> m_physics;
        std::shared_ptr<const Terrain> m_terrain;

        std::unique_ptr<Fighter> m_fighter;
        EntityId m_fighterId;
        bool m_fighterRespawnPending = false; // destroyed this step, back in play at its end
        std::vector<std::unique_ptr<Target>> m_targets;
        std::vector<PlatformRecord> m_platforms;
        TargetAIConfig m_targetAIConfig;

        std::vector<std::unique_ptr<Flare>> m_flares;

        PlayerRole m_role = PlayerRole::Sam;
        EntityId m_launcherSiteId; // SAM owner (not a damageable platform)
        EntityId m_scenarioSiteId; // owner of scripted launches
        CustomRoundSpec m_customSpec;
        std::string m_fox2Id = "aim-9x-blk2";
        std::string m_aircraftId = "f-16c-block-50";
        std::vector<Station> m_stations;
        int m_selectedStation = 0;
        bool m_seekerUncaged = false;

        std::vector<std::unique_ptr<Shot>> m_shots;
    };
}
