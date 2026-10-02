// Player weapon stations, round building, launch clearance and the custom
// round's cold-launch program.
#include "World.h"

#include "objects/Fighter.h"
#include "objects/Missile.h"
#include "physics/PhysicsEngine.h"
#include "sim/EngagementRules.h"
#include "sim/Fox2Catalog.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/gtx/norm.hpp>

namespace missilesim::sim
{
    namespace
    {
        constexpr const char *kDefaultFox2Id = "aim-9x-blk2";
        // Staged rounds sit this far along the eject vector when released so
        // the round starts clear of the cell floor.
        constexpr float kCellClearanceM = 0.45f;

        glm::vec3 normalizeOr(const glm::vec3 &value, const glm::vec3 &fallback)
        {
            if (glm::length2(value) > 1.0e-4f)
            {
                return glm::normalize(value);
            }
            if (glm::length2(fallback) > 1.0e-4f)
            {
                return glm::normalize(fallback);
            }
            return glm::vec3(0.0f, 0.0f, 1.0f);
        }

        float smooth01(float value)
        {
            const float t = std::clamp(value, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        }

        float finiteOr(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }

        float positiveOr(float value, float fallback)
        {
            return (std::isfinite(value) && value > 0.0f) ? value : fallback;
        }

        glm::vec3 finiteOr(const glm::vec3 &value, const glm::vec3 &fallback)
        {
            glm::vec3 result = value;
            for (int axis = 0; axis < 3; ++axis)
            {
                if (!std::isfinite(result[axis]))
                {
                    result[axis] = fallback[axis];
                }
            }
            return result;
        }

        // The spec with every value made safe to build from; the defaults are
        // the struct's own.
        CustomRoundSpec sanitized(const CustomRoundSpec &spec)
        {
            const CustomRoundSpec defaults;
            CustomRoundSpec safe = spec;
            safe.position = finiteOr(spec.position, glm::vec3(0.0f));
            safe.velocity = finiteOr(spec.velocity, glm::vec3(0.0f));
            safe.dryMass = positiveOr(spec.dryMass, defaults.dryMass);
            safe.dragCoefficient = std::isfinite(spec.dragCoefficient) && spec.dragCoefficient >= 0.0f ? spec.dragCoefficient : defaults.dragCoefficient;
            safe.crossSectionalArea = positiveOr(spec.crossSectionalArea, defaults.crossSectionalArea);
            safe.liftCoefficient = std::isfinite(spec.liftCoefficient) && spec.liftCoefficient >= 0.0f ? spec.liftCoefficient : defaults.liftCoefficient;
            safe.navigationGain = std::clamp(finiteOr(spec.navigationGain, defaults.navigationGain), 1.0f, 4.0f);
            safe.maxSteeringForce = positiveOr(spec.maxSteeringForce, defaults.maxSteeringForce);
            safe.trackingAngleDegrees = std::clamp(finiteOr(spec.trackingAngleDegrees, defaults.trackingAngleDegrees), 5.0f, 180.0f);
            safe.proximityFuseRadius = std::isfinite(spec.proximityFuseRadius) && spec.proximityFuseRadius >= 0.0f ? spec.proximityFuseRadius : defaults.proximityFuseRadius;
            safe.countermeasureResistance = std::clamp(finiteOr(spec.countermeasureResistance, defaults.countermeasureResistance), 0.0f, 1.0f);
            safe.terrainClearance = std::isfinite(spec.terrainClearance) && spec.terrainClearance >= 0.0f ? spec.terrainClearance : defaults.terrainClearance;
            safe.terrainLookAheadTime = std::isfinite(spec.terrainLookAheadTime) && spec.terrainLookAheadTime >= 0.5f ? spec.terrainLookAheadTime : defaults.terrainLookAheadTime;
            safe.thrust = std::isfinite(spec.thrust) && spec.thrust >= 0.0f ? spec.thrust : defaults.thrust;
            safe.fuelMass = std::isfinite(spec.fuelMass) && spec.fuelMass >= 0.0f ? spec.fuelMass : defaults.fuelMass;
            safe.fuelConsumptionRate = std::isfinite(spec.fuelConsumptionRate) && spec.fuelConsumptionRate >= 0.0f ? spec.fuelConsumptionRate : defaults.fuelConsumptionRate;
            safe.padRestHeight = positiveOr(spec.padRestHeight, rules::kGroundLaunchClearanceM);
            return safe;
        }
    }

    // ---- Roles and rounds ------------------------------------------------------

    void World::setRoleSam(const CustomRoundSpec &spec)
    {
        m_role = PlayerRole::Sam;
        m_customSpec = spec;
        removeFighter();

        m_stations.clear();
        Station cell;
        cell.index = 0;
        cell.reserve = kUnlimitedReserve;
        cell.round = buildCustomRound();
        cell.readyTime = time();
        m_stations.push_back(std::move(cell));
        m_selectedStation = 0;
    }

    void World::setRoleFighter(const std::string &fox2Id)
    {
        m_role = PlayerRole::Fighter;
        m_fox2Id = missilesim::fox2::find(fox2Id.c_str()) != nullptr ? fox2Id : std::string(kDefaultFox2Id);
        spawnFighter();

        m_stations.clear();
        const float sides[] = {1.0f, -1.0f}; // right wingtip fires first
        for (int index = 0; index < 2; ++index)
        {
            Station rail;
            rail.index = index;
            rail.side = sides[index];
            rail.round = buildFox2Round();
            rail.readyTime = time();
            m_stations.push_back(std::move(rail));
        }
        m_selectedStation = 0;
        for (Station &station : m_stations)
        {
            positionOnRail(station);
        }
    }

    void World::selectFox2(const std::string &fox2Id)
    {
        if (m_role != PlayerRole::Fighter || missilesim::fox2::find(fox2Id.c_str()) == nullptr)
        {
            return;
        }
        m_fox2Id = fox2Id;
        for (Station &station : m_stations)
        {
            if (station.round)
            {
                station.round = buildFox2Round();
                positionOnRail(station);
            }
        }
    }

    void World::setCustomRoundSpec(const CustomRoundSpec &spec, bool reloadNow)
    {
        m_customSpec = spec;
        if (reloadNow && m_role == PlayerRole::Sam)
        {
            for (Station &station : m_stations)
            {
                if (station.round)
                {
                    station.round = buildCustomRound();
                }
            }
        }
    }

    void World::rearm()
    {
        for (Station &station : m_stations)
        {
            station.round = (m_role == PlayerRole::Fighter) ? buildFox2Round() : buildCustomRound();
            station.readyTime = time();
            positionOnRail(station);
        }
        m_selectedStation = 0;
    }

    std::unique_ptr<Missile> World::buildFox2Round() const
    {
        const missilesim::fox2::Spec *spec = missilesim::fox2::find(m_fox2Id.c_str());
        if (spec == nullptr)
        {
            spec = missilesim::fox2::find(kDefaultFox2Id);
        }
        auto round = std::make_unique<Missile>();
        round->setGroundReferenceAltitude(groundLevel());
        if (spec != nullptr)
        {
            round->configureFox2(*spec);
        }
        return round;
    }

    std::unique_ptr<Missile> World::buildCustomRound() const
    {
        const CustomRoundSpec spec = sanitized(m_customSpec);
        auto round = std::make_unique<Missile>(spec.position, spec.velocity, spec.dryMass, spec.dragCoefficient,
                                               spec.crossSectionalArea, spec.liftCoefficient);

        round->setGuidanceEnabled(spec.guidanceEnabled);
        round->setNavigationGain(spec.navigationGain);
        round->setMaxSteeringForce(spec.maxSteeringForce);
        round->setTrackingAngle(spec.trackingAngleDegrees);
        round->setProximityFuseRadius(spec.proximityFuseRadius);
        round->setCountermeasureResistance(spec.countermeasureResistance);
        round->setTerrainAvoidanceEnabled(spec.terrainAvoidanceEnabled);
        round->setTerrainClearance(spec.terrainClearance);
        round->setTerrainLookAheadTime(spec.terrainLookAheadTime);
        round->setGroundReferenceAltitude(groundLevel());

        missilesim::physics::AeroProfile profile;
        profile.referenceArea = spec.crossSectionalArea;
        profile.baseDragCoefficient = spec.dragCoefficient;
        profile.aspectRatio = spec.aspectRatio;
        profile.oswaldEfficiency = spec.oswaldEfficiency;
        profile.maxLiftCoefficient = spec.maxLiftCoefficient;
        profile.machDragMultiplier = spec.machDragMultiplier.empty()
                                         ? missilesim::physics::defaultSupersonicDragRiseCurve()
                                         : spec.machDragMultiplier;
        round->setAeroProfile(profile);
        round->setMaxLoadFactorG(spec.maxLoadFactorG);

        // Inert until launch: the launch program lights the motor.
        round->setThrust(spec.thrust);
        round->setThrottle(1.0f);
        round->setThrustEnabled(false);
        round->setFuel(spec.fuelMass);
        round->setFuelConsumptionRate(spec.fuelConsumptionRate);
        round->setNozzleExitArea(spec.nozzleExitArea);
        round->setNozzleExitPressure(spec.nozzleExitPressure);

        const float ground = groundLevel();
        glm::vec3 staged = round->getPosition();
        const bool groundProfile =
            (staged.y - ground) <= (rules::kGroundLaunchProfileHeightM + rules::kGroundLaunchClearanceM);
        if (groundProfile)
        {
            // Upright in its cell with its base on the ground.
            round->setThrustDirection(glm::vec3(0.0f, 1.0f, 0.0f));
            staged.y = ground + spec.padRestHeight;
            round->setPosition(staged);
        }
        else
        {
            const glm::vec3 standbyAim = normalizeOr(spec.velocity, glm::vec3(0.0f, 0.0f, 1.0f));
            round->setThrustDirection(samLaunchDirection(*round, nullptr, standbyAim));
        }
        return round;
    }

    void World::positionOnRail(Station &station) const
    {
        if (m_role != PlayerRole::Fighter || !m_fighter || !station.round)
        {
            return;
        }
        const glm::vec3 nose = m_fighter->getNose();
        const glm::vec3 right = m_fighter->getRight();
        const glm::vec3 up = m_fighter->getUp();
        const float scale = std::max(m_fighter->getRadius(), 1.0f);
        const float side = station.side >= 0.0f ? 1.0f : -1.0f;
        Missile &round = *station.round;
        round.setPosition(m_fighter->getPosition() +
                          scale * (right * (side * rules::kRailOutboard) - up * rules::kRailBelow - nose * rules::kRailAft));
        round.setVelocity(m_fighter->getVelocity());
        round.setBodyForward(nose);
        round.setThrustDirection(nose);
        round.setThrustEnabled(false);
        round.setThrottle(1.0f);
    }

    void World::refreshPrelaunch(Station &station)
    {
        if (m_role != PlayerRole::Fighter || !m_fighter || !station.round || !station.round->isFox2())
        {
            return;
        }
        if (!m_seekerUncaged)
        {
            station.round->clearTarget();
            station.round->clearFox2Lock();
            return;
        }
        station.round->updateFox2Prelaunch(aliveTargets(), m_fighter->getNose(), m_fighter->getPosition());
    }

    void World::stepStations()
    {
        for (Station &station : m_stations)
        {
            if (!station.round && station.reserve != 0 && time() >= station.readyTime)
            {
                station.round = (m_role == PlayerRole::Fighter) ? buildFox2Round() : buildCustomRound();
                if (station.reserve > 0)
                {
                    --station.reserve;
                }
            }
            positionOnRail(station);
        }

        const int next = stationToFire();
        if (next >= 0)
        {
            refreshPrelaunch(m_stations[static_cast<std::size_t>(next)]);
        }
    }

    void World::advanceSelectedStation()
    {
        const int count = static_cast<int>(m_stations.size());
        for (int offset = 1; offset <= count; ++offset)
        {
            const int candidate = (m_selectedStation + offset) % count;
            if (m_stations[static_cast<std::size_t>(candidate)].round)
            {
                m_selectedStation = candidate;
                return;
            }
        }
    }

    int World::stationToFire() const
    {
        const int count = static_cast<int>(m_stations.size());
        if (count == 0)
        {
            return -1;
        }
        const int selected = std::clamp(m_selectedStation, 0, count - 1);
        for (int offset = 0; offset < count; ++offset)
        {
            const int candidate = (selected + offset) % count;
            if (m_stations[static_cast<std::size_t>(candidate)].round)
            {
                return candidate;
            }
        }
        return selected;
    }

    Missile *World::readyRound()
    {
        const int index = stationToFire();
        return index >= 0 ? m_stations[static_cast<std::size_t>(index)].round.get() : nullptr;
    }

    const Missile *World::readyRound() const
    {
        const int index = stationToFire();
        return index >= 0 ? m_stations[static_cast<std::size_t>(index)].round.get() : nullptr;
    }

    int World::roundsRemaining() const
    {
        int loaded = 0;
        for (const Station &station : m_stations)
        {
            if (station.reserve == kUnlimitedReserve)
            {
                return kUnlimitedReserve;
            }
            loaded += (station.round ? 1 : 0) + station.reserve;
        }
        return loaded;
    }

    void World::setSeekerUncaged(bool uncaged)
    {
        m_seekerUncaged = uncaged;
        if (!uncaged)
        {
            for (Station &station : m_stations)
            {
                if (station.round)
                {
                    station.round->clearTarget();
                    station.round->clearFox2Lock();
                }
            }
        }
        else if (const int next = stationToFire(); next >= 0)
        {
            refreshPrelaunch(m_stations[static_cast<std::size_t>(next)]);
        }
    }

    void World::designate(EntityId target)
    {
        Missile *round = readyRound();
        if (m_role != PlayerRole::Sam || round == nullptr)
        {
            return;
        }
        Target *body = targetAlive(target) && m_seekerUncaged ? findTarget(target) : nullptr;
        if (body != nullptr)
        {
            round->setTargetObject(body);
        }
        else
        {
            round->clearTarget();
        }
    }

    // ---- Launch ------------------------------------------------------------------

    LaunchBlock World::clearanceFor(const Station &station) const
    {
        if (!station.round)
        {
            return station.reserve != 0 ? LaunchBlock::Reloading : LaunchBlock::NoRound;
        }
        if (!m_seekerUncaged)
        {
            return LaunchBlock::SeekerCaged;
        }

        const Missile &round = *station.round;
        const Target *designated = round.getTargetObject();
        if (designated == nullptr || !targetAlive(designated->getEntityId()))
        {
            return LaunchBlock::NoDesignation;
        }
        if (m_role == PlayerRole::Fighter)
        {
            const missilesim::fox2::Spec *spec = round.fox2Spec();
            const bool lockAfterLaunch = spec != nullptr && spec->homing == missilesim::fox2::LaunchHoming::LockAfterLaunch;
            if (!lockAfterLaunch && !round.hasFox2InfraredLock())
            {
                return LaunchBlock::NeedsInfraredLock;
            }
        }
        return LaunchBlock::None;
    }

    LaunchBlock World::launchClearance() const
    {
        if (m_role == PlayerRole::Fighter && !m_fighter)
        {
            return LaunchBlock::NoLauncher;
        }
        const int index = stationToFire();
        if (index < 0)
        {
            return LaunchBlock::NoLauncher;
        }
        return clearanceFor(m_stations[static_cast<std::size_t>(index)]);
    }

    LaunchResult World::launch(const glm::vec3 &fallbackAim)
    {
        if (m_stations.empty() || (m_role == PlayerRole::Fighter && !m_fighter))
        {
            return LaunchResult{LaunchBlock::NoLauncher, kNoEntity};
        }

        m_selectedStation = stationToFire();
        Station &station = m_stations[static_cast<std::size_t>(m_selectedStation)];
        if (m_role == PlayerRole::Fighter)
        {
            positionOnRail(station);
            refreshPrelaunch(station);
        }

        const LaunchBlock block = clearanceFor(station);
        if (block != LaunchBlock::None)
        {
            return LaunchResult{block, kNoEntity};
        }
        return m_role == PlayerRole::Fighter ? launchFox2(station) : launchSam(station, fallbackAim);
    }

    LaunchResult World::launchFox2(Station &station)
    {
        std::unique_ptr<Missile> missile = std::move(station.round);
        const glm::vec3 nose = m_fighter->getNose();
        const glm::vec3 velocity = m_fighter->getVelocity();
        const bool infrared = missile->hasFox2InfraredLock();
        missile->setVelocity(velocity);
        missile->beginFox2Flight(nose, infrared);

        auto shot = std::make_unique<Shot>();
        shot->id = allocateId();
        shot->owner = m_fighterId;
        shot->team = Team::Blue;
        shot->launchKind = LaunchKind::Rail;
        shot->designatedTarget = targetIdOf(missile->getTargetObject());
        shot->seekerSource = shot->designatedTarget;
        shot->launchTime = time();
        shot->launchPosition = missile->getPosition();
        shot->stepStartPosition = shot->launchPosition;
        shot->motorBurning = true;
        shot->fuzeArmed = missile->isFuzeArmed();
        missile->setEntityId(shot->id);
        m_physics->addObject(missile.get());
        shot->missile = std::move(missile);

        SimEvent launched = makeEvent(EventType::WeaponLaunched, shot->id, shot->designatedTarget);
        launched.position = shot->launchPosition;
        launched.velocity = velocity;
        launched.value = glm::length(velocity);
        launched.detail = static_cast<std::uint8_t>(LaunchKind::Rail);
        publish(launched);
        SimEvent ignition = makeEvent(EventType::MotorIgnition, shot->id);
        ignition.position = shot->launchPosition;
        ignition.velocity = nose; // motor axis
        publish(ignition);

        const EntityId id = shot->id;
        m_shots.push_back(std::move(shot));
        m_seekerUncaged = false;
        advanceSelectedStation();
        return LaunchResult{LaunchBlock::None, id};
    }

    glm::vec3 World::samLaunchDirection(const Missile &round, const Target *locked, const glm::vec3 &fallbackAim) const
    {
        const glm::vec3 stagedVelocity = sanitized(m_customSpec).velocity;
        const glm::vec3 fallback = normalizeOr(stagedVelocity, glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::vec3 aim = normalizeOr(fallbackAim, fallback);
        glm::vec3 direction = aim;
        glm::vec3 targetDirection = aim;
        if (locked != nullptr)
        {
            targetDirection = normalizeOr(locked->getPosition() - round.getPosition(), aim);
            direction = targetDirection;
        }

        // From the ground the round climbs out between the minimum and maximum
        // launch pitch whatever the target elevation.
        const float clearance = round.getPosition().y - groundLevel();
        if (clearance <= rules::kGroundLaunchProfileHeightM)
        {
            glm::vec3 flat(direction.x, 0.0f, direction.z);
            if (glm::length2(flat) <= 1.0e-4f)
            {
                flat = glm::vec3(targetDirection.x, 0.0f, targetDirection.z);
            }
            flat = normalizeOr(flat, glm::vec3(0.0f, 0.0f, 1.0f));
            const float minimumPitch = glm::radians(rules::kGroundLaunchMinimumPitchDeg);
            const float maximumPitch = glm::radians(rules::kGroundLaunchMaximumPitchDeg);
            const float currentPitch = std::asin(std::clamp(direction.y, -1.0f, 1.0f));
            const float pitch = std::clamp(std::max(currentPitch, minimumPitch), minimumPitch, maximumPitch);
            direction = normalizeOr(flat * std::cos(pitch) + glm::vec3(0.0f, 1.0f, 0.0f) * std::sin(pitch),
                                    glm::vec3(0.0f, 1.0f, 0.0f));
        }
        return direction;
    }

    void World::aimReadyRound(const glm::vec3 &fallbackAim)
    {
        Missile *round = readyRound();
        if (m_role != PlayerRole::Sam || round == nullptr)
        {
            return;
        }
        // A round standing in its cell stays upright until it is ejected.
        const bool groundProfile = (round->getPosition().y - groundLevel()) <=
                                   (rules::kGroundLaunchProfileHeightM + rules::kGroundLaunchClearanceM);
        if (groundProfile)
        {
            round->setThrustDirection(glm::vec3(0.0f, 1.0f, 0.0f));
            return;
        }
        const Target *locked = round->getTargetObject();
        round->setThrustDirection(samLaunchDirection(*round, (locked != nullptr && locked->isActive()) ? locked : nullptr,
                                                     fallbackAim));
    }

    LaunchResult World::launchSam(Station &station, const glm::vec3 &fallbackAim)
    {
        const CustomRoundSpec spec = sanitized(m_customSpec);
        std::unique_ptr<Missile> missile = std::move(station.round);
        const Target *locked = missile->getTargetObject();
        const glm::vec3 launchDirection = samLaunchDirection(*missile, locked, fallbackAim);

        const float ground = groundLevel();
        glm::vec3 launchPosition = missile->getPosition();
        launchPosition.y = std::max(launchPosition.y, ground + rules::kGroundLaunchClearanceM);
        const bool groundProfile =
            (launchPosition.y - ground) <= (rules::kGroundLaunchProfileHeightM + rules::kGroundLaunchClearanceM);

        // From the ground the ejection charge throws the round near-vertically,
        // leaning toward the target azimuth; only clear of the cell does the
        // motor light and pitch over. Clear of the ground it lights at once.
        glm::vec3 ejectDirection = launchDirection;
        if (groundProfile)
        {
            const glm::vec3 flat = normalizeOr(glm::vec3(launchDirection.x, 0.0f, launchDirection.z), glm::vec3(0.0f, 0.0f, 1.0f));
            const float pitch = glm::radians(rules::kColdLaunchEjectPitchDeg);
            ejectDirection = normalizeOr(flat * std::cos(pitch) + glm::vec3(0.0f, 1.0f, 0.0f) * std::sin(pitch),
                                         glm::vec3(0.0f, 1.0f, 0.0f));
        }
        missile->setPosition(launchPosition + ejectDirection * kCellClearanceM);

        // Inert on the eject: the program arms thrust and guidance.
        missile->setGuidanceEnabled(false);
        missile->setThrust(spec.thrust);
        missile->setThrustDirection(ejectDirection);
        missile->setFuel(spec.fuelMass);
        missile->setFuelConsumptionRate(spec.fuelConsumptionRate);
        missile->setThrottle(0.0f);
        missile->setThrustEnabled(false);
        const float ejectSpeed = groundProfile ? rules::kColdLaunchEjectSpeedMps
                                               : std::max(glm::length(spec.velocity), rules::kColdLaunchEjectSpeedMps);
        missile->setVelocity(ejectDirection * ejectSpeed);

        auto shot = std::make_unique<Shot>();
        shot->id = allocateId();
        shot->owner = m_launcherSiteId;
        shot->team = Team::Blue;
        shot->launchKind = groundProfile ? LaunchKind::ColdLaunch : LaunchKind::AirLaunch;
        shot->designatedTarget = targetIdOf(locked);
        shot->seekerSource = shot->designatedTarget;
        shot->launchTime = time();
        shot->launchPosition = missile->getPosition();
        shot->stepStartPosition = shot->launchPosition;
        shot->fuzeArmed = missile->isFuzeArmed();

        ColdLaunchProgram &program = shot->coldLaunch;
        program.active = true;
        program.restoreGuidanceEnabled = spec.guidanceEnabled;
        program.ejectDirection = ejectDirection;
        program.launchDirection = launchDirection;
        program.aimDirection = ejectDirection;
        program.sustainThrust = spec.thrust;
        if (!groundProfile)
        {
            program.ignitionDelay = rules::kAirLaunchIgnitionDelayS;
            program.guidanceArmDelay = rules::kAirLaunchGuidanceArmDelayS;
        }

        missile->setEntityId(shot->id);
        m_physics->addObject(missile.get());
        shot->missile = std::move(missile);

        SimEvent launched = makeEvent(EventType::WeaponLaunched, shot->id, shot->designatedTarget);
        launched.position = launchPosition;
        launched.velocity = shot->missile->getVelocity();
        launched.value = ejectSpeed;
        launched.detail = static_cast<std::uint8_t>(shot->launchKind);
        publish(launched);

        const EntityId id = shot->id;
        m_shots.push_back(std::move(shot));
        station.readyTime = time() + rules::kSamReloadSeconds;
        m_seekerUncaged = false;
        return LaunchResult{LaunchBlock::None, id};
    }

    EntityId World::launchScripted(const ScriptedLaunch &launch)
    {
        std::unique_ptr<Missile> missile;
        const missilesim::fox2::Spec *spec = launch.fox2Id.empty() ? nullptr : missilesim::fox2::find(launch.fox2Id.c_str());
        if (spec != nullptr)
        {
            missile = std::make_unique<Missile>();
            missile->setGroundReferenceAltitude(groundLevel());
            missile->configureFox2(*spec);
        }
        else
        {
            missile = buildCustomRound();
            missile->setGuidanceEnabled(false);
            missile->setTerrainAvoidanceEnabled(false);
        }

        Target *designated = targetAlive(launch.target) ? findTarget(launch.target) : nullptr;
        missile->setPosition(launch.position);
        missile->setVelocity(launch.velocity);
        const glm::vec3 axis = normalizeOr(launch.velocity, glm::vec3(0.0f, 0.0f, 1.0f));
        missile->setThrustDirection(axis);
        missile->setTargetObject(designated);
        if (spec != nullptr)
        {
            missile->beginFox2Flight(axis, designated != nullptr);
        }
        else
        {
            missile->setThrustEnabled(false);
        }

        auto shot = std::make_unique<Shot>();
        shot->id = allocateId();
        shot->owner = m_scenarioSiteId;
        shot->team = launch.team;
        shot->launchKind = LaunchKind::AirLaunch;
        shot->designatedTarget = targetIdOf(designated);
        shot->seekerSource = shot->designatedTarget;
        shot->launchTime = time();
        shot->launchPosition = launch.position;
        shot->stepStartPosition = launch.position;
        shot->motorBurning = missile->isThrustEnabled();
        shot->fuzeArmed = missile->isFuzeArmed();
        missile->setEntityId(shot->id);
        m_physics->addObject(missile.get());
        shot->missile = std::move(missile);

        SimEvent launched = makeEvent(EventType::WeaponLaunched, shot->id, shot->designatedTarget);
        launched.position = launch.position;
        launched.velocity = launch.velocity;
        launched.value = glm::length(launch.velocity);
        launched.detail = static_cast<std::uint8_t>(LaunchKind::AirLaunch);
        publish(launched);

        const EntityId id = shot->id;
        m_shots.push_back(std::move(shot));
        return id;
    }

    // ---- Cold-launch program ---------------------------------------------------------

    void World::stepColdLaunches()
    {
        for (const auto &shot : m_shots)
        {
            if (shot->coldLaunch.active)
            {
                updateColdLaunch(*shot, m_fixedStep);
            }
        }
    }

    void World::updateColdLaunch(Shot &shot, float dt)
    {
        ColdLaunchProgram &program = shot.coldLaunch;
        Missile &missile = *shot.missile;
        program.elapsed += dt;

        const glm::vec3 ejectDirection = normalizeOr(program.ejectDirection, missile.getThrustDirection());
        const glm::vec3 launchDirection = normalizeOr(program.launchDirection, ejectDirection);

        // Eject coast: the round rises on the charge alone, motor cold.
        if (!program.motorIgnited)
        {
            missile.setThrustDirection(ejectDirection);
            missile.setThrottle(0.0f);
            if (program.elapsed >= program.ignitionDelay)
            {
                // Ignition: the booster lights at elevated thrust.
                program.motorIgnited = true;
                missile.setThrust(program.sustainThrust * rules::kColdLaunchBoostThrustMultiplier);
                missile.setThrustEnabled(true);
                missile.setThrottle(rules::kColdLaunchIgnitionThrottle);
                shot.motorBurning = true;
                SimEvent ignition = makeEvent(EventType::MotorIgnition, shot.id);
                ignition.position = missile.getPosition();
                ignition.velocity = ejectDirection; // motor axis
                publish(ignition);
            }
        }

        if (program.motorIgnited)
        {
            const float sinceIgnition = program.elapsed - program.ignitionDelay;
            const float ramp = program.thrustRampDuration > 1.0e-4f ? sinceIgnition / program.thrustRampDuration : 1.0f;
            missile.setThrottle(glm::mix(rules::kColdLaunchIgnitionThrottle, 1.0f, smooth01(ramp)));

            // Pitch-over toward the target at a capped rate; hand off to
            // guidance once the flight path sits inside the handoff cone.
            if (!program.guidanceArmed)
            {
                const Target *target = missile.getTargetObject();
                const bool targetLive = target != nullptr && target->isActive();
                const glm::vec3 targetAim = targetLive ? normalizeOr(target->getPosition() - missile.getPosition(), launchDirection)
                                                       : launchDirection;
                const float maxStep = glm::radians(rules::kColdLaunchPitchRateDegPerS) * dt;
                const glm::vec3 currentAim = normalizeOr(program.aimDirection, ejectDirection);
                const float angle = std::acos(std::clamp(glm::dot(currentAim, targetAim), -1.0f, 1.0f));
                const float fraction = angle > 1.0e-4f ? std::clamp(maxStep / angle, 0.0f, 1.0f) : 1.0f;
                const glm::vec3 nextAim = normalizeOr(glm::mix(currentAim, targetAim, fraction), targetAim);
                program.aimDirection = nextAim;
                missile.setThrustDirection(nextAim);

                const glm::vec3 velocity = missile.getVelocity();
                const float speed = glm::length(velocity);
                if (speed > 1.0f && targetLive)
                {
                    const glm::vec3 lineOfSight = normalizeOr(target->getPosition() - missile.getPosition(), nextAim);
                    if (glm::dot(velocity / speed, lineOfSight) >= std::cos(glm::radians(rules::kColdLaunchHandoffConeDeg)))
                    {
                        program.guidanceArmed = true;
                        missile.setGuidanceEnabled(program.restoreGuidanceEnabled);
                    }
                }
            }

            // Booster cutoff: back to the configured sustainer thrust.
            if (!program.boostComplete && sinceIgnition >= program.boostDuration)
            {
                program.boostComplete = true;
                missile.setThrust(program.sustainThrust);
            }
        }

        // Backstop: guidance is never left disarmed past this point.
        if (!program.guidanceArmed && program.elapsed >= program.guidanceArmDelay)
        {
            program.guidanceArmed = true;
            missile.setGuidanceEnabled(program.restoreGuidanceEnabled);
        }

        const float end = std::max(program.guidanceArmDelay, program.ignitionDelay + program.boostDuration);
        if (program.elapsed >= end)
        {
            if (!program.guidanceArmed)
            {
                program.guidanceArmed = true;
                missile.setGuidanceEnabled(program.restoreGuidanceEnabled);
            }
            if (!program.boostComplete)
            {
                program.boostComplete = true;
                missile.setThrust(program.sustainThrust);
            }
            missile.setThrottle(1.0f);
            program.active = false;
        }
    }
}
