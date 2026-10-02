#include "World.h"

#include "flight/AircraftCatalog.h"
#include "objects/Fighter.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "physics/PhysicsEngine.h"
#include "sim/EngagementRules.h"
#include "sim/Fox2Flight.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

namespace missilesim::sim
{
    namespace
    {
        bool finite(const glm::vec3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        glm::vec3 unitOr(const glm::vec3 &v, const glm::vec3 &fallback)
        {
            return glm::length2(v) > 1.0e-8f ? glm::normalize(v) : fallback;
        }

        MAWSConfig toRuntime(const TargetMawsConfig &config)
        {
            MAWSConfig result;
            result.enabled = config.enabled;
            result.detectionRange = config.detectionRange;
            result.reactionTimeWindow = config.reactionTimeWindow;
            result.closestApproachThreshold = config.closestApproachThreshold;
            return result;
        }

        FlareDispenserConfig toRuntime(const TargetFlareConfig &config)
        {
            FlareDispenserConfig result;
            result.enabled = config.enabled;
            result.inventory = config.inventory;
            result.burstSize = config.burstSize;
            result.burstInterval = config.burstInterval;
            result.cooldown = config.cooldown;
            result.ejectSpeed = config.ejectSpeed;
            result.aftLaunchOffset = config.aftLaunchOffset;
            result.lateralLaunchOffset = config.lateralLaunchOffset;
            result.lateralEjectFraction = config.lateralEjectFraction;
            result.downwardEjectFraction = config.downwardEjectFraction;
            result.lifetime = config.lifetime;
            result.heatSignature = config.heatSignature;
            result.heatDecayRate = config.heatDecayRate;
            result.mass = config.mass;
            result.dragCoefficient = config.dragCoefficient;
            result.crossSectionalArea = config.crossSectionalArea;
            return result;
        }

        EvasiveManeuverConfig toRuntime(const TargetEvasiveConfig &config)
        {
            EvasiveManeuverConfig result;
            result.cruisePitchRateDegrees = config.cruisePitchRateDegrees;
            result.defensivePitchRateDegrees = config.defensivePitchRateDegrees;
            result.defensiveBreakWeight = config.defensiveBreakWeight;
            result.defensiveAwayWeight = config.defensiveAwayWeight;
            result.defensiveThreatAwayWeight = config.defensiveThreatAwayWeight;
            result.defensiveIncomingWeight = config.defensiveIncomingWeight;
            result.altitudeCorrectionRange = config.altitudeCorrectionRange;
            result.maxAltitudePitchBias = config.maxAltitudePitchBias;
            result.innerDistanceAltitudeOffset = config.innerDistanceAltitudeOffset;
            result.outerDistanceAltitudeOffset = config.outerDistanceAltitudeOffset;
            result.defensiveLowEnergyAltitudeOffset = config.defensiveLowEnergyAltitudeOffset;
            result.defensiveThreatBelowAltitudeOffset = config.defensiveThreatBelowAltitudeOffset;
            result.defensiveThreatAboveAltitudeOffset = config.defensiveThreatAboveAltitudeOffset;
            result.defensiveSpeedBlend = config.defensiveSpeedBlend;
            result.defensiveSpeedUrgencyBlend = config.defensiveSpeedUrgencyBlend;
            result.nearDistanceSpeedupThreshold = config.nearDistanceSpeedupThreshold;
            result.farDistanceSlowdownThreshold = config.farDistanceSlowdownThreshold;
            result.farDistanceSpeedFloor = config.farDistanceSpeedFloor;
            result.recoveryEnergyThreshold = config.recoveryEnergyThreshold;
            result.repositionDistanceThreshold = config.repositionDistanceThreshold;
            return result;
        }

        TargetAIConfig targetAIFromConfig(const TargetGroupConfig &config)
        {
            TargetAIConfig result;
            result.minSpeed = config.minSpeed;
            result.maxSpeed = config.maxSpeed;
            result.preferredDistance = config.preferredDistance;
            return result;
        }
    }

    CustomRoundSpec customRoundSpecFromConfig(const SimulationConfig &config)
    {
        const MissileAirframeConfig &airframe = config.missile.airframe;
        const MissileMotorConfig &motor = config.missile.motor;
        const MissileGuidanceConfig &guidance = config.missile.guidance;

        CustomRoundSpec spec;
        spec.position = airframe.initialPosition;
        spec.velocity = airframe.initialVelocity;
        spec.dryMass = airframe.dryMass;
        spec.dragCoefficient = airframe.dragCoefficient;
        spec.crossSectionalArea = airframe.crossSectionalArea;
        spec.liftCoefficient = airframe.liftCoefficient;
        spec.aspectRatio = airframe.aspectRatio;
        spec.oswaldEfficiency = airframe.oswaldEfficiency;
        spec.maxLiftCoefficient = airframe.maxLiftCoefficient;
        spec.maxLoadFactorG = airframe.maxLoadFactorG;
        spec.machDragMultiplier = airframe.machDragMultiplier;
        spec.thrust = motor.thrust;
        spec.fuelMass = motor.fuelMass;
        spec.fuelConsumptionRate = motor.fuelConsumptionRate;
        spec.nozzleExitArea = motor.nozzleExitArea;
        spec.nozzleExitPressure = motor.nozzleExitPressure;
        spec.guidanceEnabled = guidance.enabled;
        spec.navigationGain = guidance.navigationGain;
        spec.maxSteeringForce = guidance.maxSteeringForce;
        spec.trackingAngleDegrees = guidance.trackingAngle;
        spec.proximityFuseRadius = guidance.proximityFuseRadius;
        spec.countermeasureResistance = guidance.countermeasureResistance;
        spec.terrainAvoidanceEnabled = guidance.terrainAvoidanceEnabled;
        spec.terrainClearance = guidance.terrainClearance;
        spec.terrainLookAheadTime = guidance.terrainLookAheadTime;
        return spec;
    }

    const char *launchBlockMessage(LaunchBlock block)
    {
        switch (block)
        {
        case LaunchBlock::None:
            return "Ready";
        case LaunchBlock::NoLauncher:
            return "No launcher";
        case LaunchBlock::NoRound:
            return "No rounds left";
        case LaunchBlock::Reloading:
            return "Reloading";
        case LaunchBlock::SeekerCaged:
            return "Seeker caged";
        case LaunchBlock::NoDesignation:
            return "No target in the seeker";
        case LaunchBlock::NeedsInfraredLock:
            return "Needs an infrared lock";
        }
        return "Unavailable";
    }

    World::World(const SimulationConfig &config, std::uint64_t seed)
        : m_config(config),
          m_random(seed),
          m_physics(std::make_unique<PhysicsEngine>())
    {
        m_fixedStep = (std::isfinite(config.environment.fixedTimeStep) && config.environment.fixedTimeStep > 0.0f)
                          ? config.environment.fixedTimeStep
                          : 0.01f;
        m_physics->setGravity(config.environment.gravity);
        m_physics->setAirDensity(config.environment.seaLevelAirDensity);
        m_physics->setGroundEnabled(config.environment.groundCollisionEnabled);
        m_physics->setGroundRestitution(config.environment.groundRestitution);
        m_targetAIConfig = targetAIFromConfig(config.targets);
        m_customSpec = customRoundSpecFromConfig(config);
        m_targetSpawnRandom = m_random.stream("targets.spawn");
        m_launcherSiteId = allocateId();
        m_scenarioSiteId = allocateId();
        setRoleSam(m_customSpec);
    }

    World::~World()
    {
        // Shots and stations hold raw target and flare pointers; release them
        // before the objects they point at.
        m_shots.clear();
        m_stations.clear();
        clearTargets();
        removeFighter();
    }

    void World::restart(std::uint64_t seed)
    {
        for (const auto &shot : m_shots)
        {
            m_physics->removeObject(shot->missile.get());
        }
        m_shots.clear();
        for (const auto &flare : m_flares)
        {
            m_physics->removeFlare(flare.get());
        }
        m_flares.clear();
        for (const auto &target : m_targets)
        {
            m_physics->removeTarget(target.get());
        }
        m_targets.clear();
        m_platforms.clear();
        m_fighter.reset();
        m_fighterId = kNoEntity;
        m_fighterRespawnPending = false;

        m_tick = 0;
        m_nextId = 1;
        m_nextEventSequence = 1;
        m_pending.clear();
        m_trace.clear();
        m_random = RandomStreams(seed);
        m_targetSpawnRandom = m_random.stream("targets.spawn");
        m_launcherSiteId = allocateId();
        m_scenarioSiteId = allocateId();
        m_seekerUncaged = false;

        if (m_role == PlayerRole::Fighter)
        {
            setRoleFighter(m_fox2Id);
        }
        else
        {
            setRoleSam(m_customSpec);
        }
    }

    EntityId World::allocateId()
    {
        return EntityId{m_nextId++};
    }

    SimEvent World::makeEvent(EventType type, EntityId subject, EntityId other) const
    {
        SimEvent event;
        event.tick = m_tick;
        event.time = time();
        event.type = type;
        event.subject = subject;
        event.other = other;
        return event;
    }

    void World::publish(SimEvent event)
    {
        event.sequence = m_nextEventSequence++;
        if (m_recordTrace)
        {
            m_trace.push_back(event);
        }
        m_pending.push_back(event);
    }

    std::vector<SimEvent> World::drainEvents()
    {
        std::vector<SimEvent> drained;
        drained.swap(m_pending);
        return drained;
    }

    float World::groundLevel() const
    {
        return m_physics->getGroundLevel();
    }

    World::PlatformRecord *World::record(EntityId id)
    {
        for (PlatformRecord &entry : m_platforms)
        {
            if (entry.id == id)
            {
                return &entry;
            }
        }
        return nullptr;
    }

    const World::PlatformRecord *World::record(EntityId id) const
    {
        for (const PlatformRecord &entry : m_platforms)
        {
            if (entry.id == id)
            {
                return &entry;
            }
        }
        return nullptr;
    }

    // ---- Fighter ---------------------------------------------------------------

    bool World::selectAircraft(const std::string &aircraftId)
    {
        const missilesim::flight::AircraftCard *card = missilesim::flight::findAircraft(aircraftId.c_str());
        if (card == nullptr || !card->flyable)
        {
            return false;
        }
        if (m_aircraftId == card->id)
        {
            return true;
        }
        m_aircraftId = card->id;
        if (m_fighter)
        {
            m_fighter->setAircraft(card->id);
            for (Station &station : m_stations)
            {
                positionOnRail(station);
            }
        }
        return true;
    }

    void World::spawnFighter()
    {
        if (m_fighter)
        {
            return;
        }
        const missilesim::flight::AircraftCard *card = missilesim::flight::findAircraft(m_aircraftId.c_str());
        if (card == nullptr || !card->flyable)
        {
            m_aircraftId = missilesim::flight::defaultAircraftId();
        }
        m_fighter = std::make_unique<Fighter>();
        m_fighter->setAircraft(m_aircraftId.c_str());
        m_fighterId = allocateId();
        m_fighter->setEntityId(m_fighterId);
        m_platforms.push_back(PlatformRecord{m_fighterId, PlatformKind::Fighter, Team::Blue, 1.0f, true});
        placeFighterAtEngagement();
        SimEvent event = makeEvent(EventType::PlatformSpawned, m_fighterId);
        event.position = m_fighter->getPosition();
        event.velocity = m_fighter->getVelocity();
        publish(event);
    }

    void World::removeFighter()
    {
        if (!m_fighter)
        {
            return;
        }
        const EntityId id = m_fighterId;
        m_platforms.erase(std::remove_if(m_platforms.begin(), m_platforms.end(),
                                         [id](const PlatformRecord &entry) { return entry.id == id; }),
                          m_platforms.end());
        SimEvent event = makeEvent(EventType::PlatformRemoved, id);
        event.position = m_fighter->getPosition();
        publish(event);
        m_fighter.reset();
        m_fighterId = kNoEntity;
        m_fighterRespawnPending = false;
    }

    void World::placeFighterAtEngagement()
    {
        if (!m_fighter)
        {
            return;
        }

        float altitude = rules::kFighterDefaultSpawnAltitudeM;
        glm::vec3 heading(0.0f, 0.0f, 1.0f);
        for (const auto &target : m_targets)
        {
            if (target && target->isActive())
            {
                altitude = target->getPosition().y;
                heading = unitOr(glm::vec3(target->getPosition().x, 0.0f, target->getPosition().z), heading);
                break;
            }
        }
        altitude = std::max(altitude, rules::kFighterMinimumSpawnAltitudeM);
        m_fighter->place(glm::vec3(0.0f, altitude, 0.0f), heading * rules::kFighterSpawnSpeedMps, heading);
        for (Station &station : m_stations)
        {
            positionOnRail(station);
        }
    }

    void World::stepFighter()
    {
        if (!m_fighter || m_fighterRespawnPending)
        {
            return;
        }

        const Atmosphere::State air = m_physics->getAtmosphereState(m_fighter->getPosition().y);
        const float gravity = std::max(m_physics->getGravity(), 0.0f);
        m_fighter->updateFlight(m_fixedStep, air.densityKgPerCubicMeter, air.speedOfSoundMetersPerSecond, gravity);

        const glm::vec3 position = m_fighter->getPosition();
        // The fighter has no ground contact model, so the ground is always solid to it.
        const bool grounded = position.y <= groundLevel() + rules::kFighterGroundClearanceM;
        if (!finite(position) || grounded)
        {
            SimEvent event = makeEvent(EventType::GroundCollision, m_fighterId);
            event.position = finite(position) ? position : glm::vec3(0.0f);
            event.velocity = m_fighter->getVelocity();
            publish(event);
            destroyPlatform(m_fighterId, kNoEntity, event.position);
        }
    }

    // ---- Targets -------------------------------------------------------------------

    Target *World::findTarget(EntityId id) const
    {
        if (!id.valid())
        {
            return nullptr;
        }
        for (const auto &target : m_targets)
        {
            if (target->getEntityId() == id)
            {
                return target.get();
            }
        }
        return nullptr;
    }

    EntityId World::targetIdOf(const Target *target) const
    {
        return target != nullptr ? target->getEntityId() : kNoEntity;
    }

    bool World::targetAlive(EntityId id) const
    {
        const Target *target = findTarget(id);
        const PlatformRecord *entry = record(id);
        return target != nullptr && entry != nullptr && entry->alive && target->isActive();
    }

    bool World::anyTargetAlive() const
    {
        for (const auto &target : m_targets)
        {
            if (targetAlive(target->getEntityId()))
            {
                return true;
            }
        }
        return false;
    }

    std::vector<Target *> World::aliveTargets() const
    {
        std::vector<Target *> alive;
        alive.reserve(m_targets.size());
        for (const auto &target : m_targets)
        {
            if (targetAlive(target->getEntityId()))
            {
                alive.push_back(target.get());
            }
        }
        return alive;
    }

    void World::setTargetAIConfig(const TargetAIConfig &config)
    {
        m_targetAIConfig = config;
    }

    void World::applyTargetAIConfigToAll()
    {
        for (const auto &target : m_targets)
        {
            target->setAIConfig(m_targetAIConfig);
        }
    }

    EntityId World::spawnTarget(const glm::vec3 &position, float radius)
    {
        const TargetSpawnConfig &spawn = m_config.targets.spawn;
        glm::vec3 place = position;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(place[axis]))
            {
                place[axis] = spawn.fallbackPosition[axis];
            }
        }
        const float safeRadius = (std::isfinite(radius) && radius > 0.0f) ? radius : spawn.fallbackRadius;

        auto target = std::make_unique<Target>(place, safeRadius);
        target->setAIConfig(m_targetAIConfig);
        target->setMAWSConfig(toRuntime(m_config.targets.maws));
        target->setFlareDispenserConfig(toRuntime(m_config.targets.flares));
        target->setEvasiveManeuverConfig(toRuntime(m_config.targets.evasive));

        const EntityId id = allocateId();
        target->setEntityId(id);
        m_physics->addTarget(target.get());
        m_platforms.push_back(PlatformRecord{id, PlatformKind::Target, Team::Red, 1.0f, true});

        SimEvent event = makeEvent(EventType::PlatformSpawned, id);
        event.position = target->getPosition();
        event.velocity = target->getVelocity();
        event.value = safeRadius;
        publish(event);
        m_targets.push_back(std::move(target));
        return id;
    }

    void World::respawnTargets(int count)
    {
        clearTargets();

        const TargetSpawnConfig &spawn = m_config.targets.spawn;
        const int maximum = std::max(m_config.targets.maxCount, 1);
        const int total = std::clamp(count, 1, maximum);
        const float preferred = (std::isfinite(m_targetAIConfig.preferredDistance) && m_targetAIConfig.preferredDistance > 0.0f)
                                    ? m_targetAIConfig.preferredDistance
                                    : m_config.targets.preferredDistance;

        // Draw order is part of the scenario definition: distance, bearing,
        // altitude, radius, per aircraft.
        for (int index = 0; index < total; ++index)
        {
            const float spawnDistance = preferred * m_targetSpawnRandom.uniform(spawn.distanceScaleMin, spawn.distanceScaleMax);
            const float minimumAltitude = std::max(spawn.minimumAltitudeFloor,
                                                   std::min(spawnDistance * spawn.minimumAltitudeDistanceFraction,
                                                            spawn.minimumAltitudeCeiling));
            const float maximumAltitude = std::max(minimumAltitude + spawn.minimumAltitudeBand,
                                                   std::min(spawnDistance * spawn.maximumAltitudeDistanceFraction,
                                                            spawn.maximumAltitudeCeiling));
            const float bearing = m_targetSpawnRandom.uniform(0.0f, glm::two_pi<float>());
            const float altitude = m_targetSpawnRandom.uniform(minimumAltitude, maximumAltitude);
            const float radius = m_targetSpawnRandom.uniform(spawn.radiusMin, spawn.radiusMax);
            spawnTarget(glm::vec3(spawnDistance * std::cos(bearing), altitude, spawnDistance * std::sin(bearing)), radius);
        }

        // One short update starts each autonomous controller from a live state.
        for (const auto &target : m_targets)
        {
            const Atmosphere::State air = m_physics->getAtmosphereState(target->getPosition().y);
            target->setAmbientConditions(air.densityKgPerCubicMeter, air.speedOfSoundMetersPerSecond);
            target->update(spawn.warmupTimeStep);
            target->consumePendingFlareLaunches();
        }
    }

    void World::forgetTargetEverywhere(const Target *target)
    {
        if (target == nullptr)
        {
            return;
        }
        for (const auto &shot : m_shots)
        {
            if (shot->missile && shot->missile->getTargetObject() == target)
            {
                shot->missile->clearTarget();
            }
        }
        for (Station &station : m_stations)
        {
            if (station.round && station.round->getTargetObject() == target)
            {
                station.round->clearTarget();
                station.round->clearFox2Lock();
            }
        }
    }

    void World::removeTarget(EntityId id)
    {
        const auto found = std::find_if(m_targets.begin(), m_targets.end(),
                                        [id](const std::unique_ptr<Target> &target) { return target->getEntityId() == id; });
        if (found == m_targets.end())
        {
            return;
        }

        Target *target = found->get();
        forgetTargetEverywhere(target);
        m_physics->removeTarget(target);
        SimEvent event = makeEvent(EventType::PlatformRemoved, id);
        event.position = target->getPosition();
        publish(event);
        m_platforms.erase(std::remove_if(m_platforms.begin(), m_platforms.end(),
                                         [id](const PlatformRecord &entry) { return entry.id == id; }),
                          m_platforms.end());
        m_targets.erase(found);
    }

    void World::clearTargets()
    {
        // Their flares go with them: a respawned field starts clean.
        for (const auto &flare : m_flares)
        {
            forgetFlareEverywhere(flare.get());
            m_physics->removeFlare(flare.get());
        }
        m_flares.clear();

        while (!m_targets.empty())
        {
            removeTarget(m_targets.front()->getEntityId());
        }
    }

    // ---- Platform state ------------------------------------------------------------

    float World::health(EntityId platform) const
    {
        const PlatformRecord *entry = record(platform);
        return entry != nullptr ? entry->health : 0.0f;
    }

    bool World::platformAlive(EntityId platform) const
    {
        const PlatformRecord *entry = record(platform);
        if (entry == nullptr || !entry->alive)
        {
            return false;
        }
        if (entry->kind == PlatformKind::Target)
        {
            const Target *target = findTarget(platform);
            return target != nullptr && target->isActive();
        }
        return m_fighter != nullptr && !m_fighterRespawnPending;
    }

    bool World::snapshot(EntityId platform, PlatformSnapshot &out) const
    {
        const PlatformRecord *entry = record(platform);
        if (entry == nullptr)
        {
            return false;
        }

        out = PlatformSnapshot{};
        out.id = platform;
        out.kind = entry->kind;
        out.team = entry->team;
        out.time = time();
        out.health = entry->health;
        out.alive = platformAlive(platform);

        if (entry->kind == PlatformKind::Fighter)
        {
            if (!m_fighter)
            {
                return false;
            }
            out.position = m_fighter->getPosition();
            out.velocity = m_fighter->getVelocity();
            out.forward = m_fighter->getNose();
            out.up = m_fighter->getUp();
            out.radius = m_fighter->getRadius();
            out.throttle = m_fighter->getThrottle();
            out.afterburner = m_fighter->isAfterburner();
            return true;
        }

        const Target *target = findTarget(platform);
        if (target == nullptr)
        {
            return false;
        }
        // The target flies a point-mass model with its body on the velocity.
        out.position = target->getPosition();
        out.velocity = target->getVelocity();
        out.forward = unitOr(target->getVelocity(), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::vec3 right = unitOr(glm::cross(out.forward, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(1.0f, 0.0f, 0.0f));
        out.up = unitOr(glm::cross(right, out.forward), glm::vec3(0.0f, 1.0f, 0.0f));
        out.radius = target->getRadius();
        out.throttle = target->getThrottle();
        out.afterburner = target->getThrottle() >= missilesim::fox2::kAiAfterburnerThrottle;
        return true;
    }

    std::vector<PlatformSnapshot> World::platformSnapshots() const
    {
        std::vector<PlatformSnapshot> snapshots;
        snapshots.reserve(m_platforms.size());
        for (const PlatformRecord &entry : m_platforms)
        {
            PlatformSnapshot snap;
            if (snapshot(entry.id, snap))
            {
                snapshots.push_back(snap);
            }
        }
        return snapshots;
    }

    // ---- Countermeasures -------------------------------------------------------------

    EntityId World::flareIdOf(const Flare *flare) const
    {
        return flare != nullptr ? flare->getEntityId() : kNoEntity;
    }

    void World::forgetFlareEverywhere(const Flare *flare)
    {
        for (const auto &shot : m_shots)
        {
            if (shot->missile)
            {
                shot->missile->forgetFlare(flare);
            }
        }
        for (Station &station : m_stations)
        {
            if (station.round)
            {
                station.round->forgetFlare(flare);
            }
        }
    }

    void World::collectFlareLaunches()
    {
        // Dispensers queue releases while the physics sub-steps run; the flares
        // join the world here, at the end of the step, in target order.
        for (const auto &target : m_targets)
        {
            for (const FlareLaunchRequest &request : target->consumePendingFlareLaunches())
            {
                auto flare = std::make_unique<Flare>(request);
                if (!flare->isActive())
                {
                    continue;
                }
                const EntityId id = allocateId();
                flare->setEntityId(id);
                m_physics->addFlare(flare.get());
                SimEvent event = makeEvent(EventType::CountermeasureReleased, id, target->getEntityId());
                event.position = flare->getPosition();
                event.velocity = flare->getVelocity();
                publish(event);
                m_flares.push_back(std::move(flare));
            }
        }
    }

    void World::retireExpiredFlares()
    {
        for (auto it = m_flares.begin(); it != m_flares.end();)
        {
            Flare *flare = it->get();
            if (flare->isActive())
            {
                ++it;
                continue;
            }
            SimEvent event = makeEvent(EventType::CountermeasureExpired, flare->getEntityId());
            event.position = flare->getPosition();
            publish(event);
            forgetFlareEverywhere(flare);
            m_physics->removeFlare(flare);
            it = m_flares.erase(it);
        }
    }

    // ---- Step --------------------------------------------------------------------------

    void World::beginStep()
    {
        if (m_fighter)
        {
            m_fighter->beginFixedStep();
        }
        for (const auto &target : m_targets)
        {
            target->beginFixedStep();
        }
        for (const auto &flare : m_flares)
        {
            flare->beginFixedStep();
        }
        for (Station &station : m_stations)
        {
            if (station.round)
            {
                station.round->beginFixedStep();
            }
        }
        for (const auto &shot : m_shots)
        {
            shot->missile->beginFixedStep();
            shot->stepStartPosition = shot->missile->getPosition();
        }
    }

    void World::step()
    {
        ++m_tick;

        beginStep();

        // Where every living platform starts this step, for the swept tests.
        std::vector<PlatformMotion> motions;
        motions.reserve(m_platforms.size());
        for (const PlatformRecord &entry : m_platforms)
        {
            PlatformSnapshot start;
            if (entry.alive && snapshot(entry.id, start) && start.alive)
            {
                motions.push_back(PlatformMotion{entry.id, start.position, start.position, start.radius});
            }
        }

        stepFighter();
        stepStations();
        stepColdLaunches();

        m_physics->update(m_fixedStep);

        // Platforms still alive after their own flight step take part in the
        // outcome tests with their straight-line motion across the step.
        motions.erase(std::remove_if(motions.begin(), motions.end(),
                                     [this](const PlatformMotion &motion) { return !platformAlive(motion.id); }),
                      motions.end());
        for (PlatformMotion &motion : motions)
        {
            PlatformSnapshot end;
            if (snapshot(motion.id, end))
            {
                motion.end = end.position;
            }
        }

        for (const auto &shot : m_shots)
        {
            stepShotBookkeeping(*shot);
        }
        resolveShotOutcomes(motions);
        retireEndedShots();

        collectFlareLaunches();
        retireExpiredFlares();

        if (m_fighterRespawnPending && m_fighter)
        {
            m_fighterRespawnPending = false;
            if (PlatformRecord *entry = record(m_fighterId))
            {
                entry->health = 1.0f;
                entry->alive = true;
            }
            placeFighterAtEngagement();
            SimEvent event = makeEvent(EventType::PlatformRespawned, m_fighterId);
            event.position = m_fighter->getPosition();
            event.velocity = m_fighter->getVelocity();
            publish(event);
        }
    }

    void World::setRenderBlend(float alpha)
    {
        if (m_fighter)
        {
            m_fighter->setRenderBlend(alpha);
        }
        for (const auto &target : m_targets)
        {
            target->setRenderBlend(alpha);
        }
        for (const auto &flare : m_flares)
        {
            flare->setRenderBlend(alpha);
        }
        for (Station &station : m_stations)
        {
            if (station.round)
            {
                station.round->setRenderBlend(alpha);
            }
        }
        for (const auto &shot : m_shots)
        {
            shot->missile->setRenderBlend(alpha);
        }
    }
}
