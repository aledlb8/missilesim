// Player and opponent scanning radars, the fictional radar round, the warning
// picture, the fire-control picture both of the player's weapons share, and the
// fighter's chaff and flare dispensers.
// Fox 2 launch and Fox 2 sensing do not come through here; the radar lock only
// cues the rail seeker (WorldWeapons.cpp). Nothing in this file reads the
// camera or a live Target* for guidance.
#include "World.h"

#include "sim/Fox2Catalog.h"
#include "sim/Fox3Catalog.h"
#include "objects/Fighter.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/PhysicsEngine.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    namespace
    {
        constexpr int kPlayerRadarRounds = 4;
        constexpr int kRedRadarRounds = 2;
        // Sim choices: the F-16's dispensers hold 120 cartridges in all; the
        // player gets that many of each kind. A held key releases a chaff
        // bundle every 0.2 s and a flare pair every 0.3 s.
        constexpr int kPlayerChaff = 120;
        constexpr int kPlayerFlares = 120;
        constexpr double kChaffIntervalS = 0.2;
        constexpr double kFlareIntervalS = 0.3;
        // Cartridges leave the dispensers under the aft fuselage, this far
        // behind the centre of mass, below it and out to each side.
        constexpr float kDispenserAftM = 4.0f;
        constexpr float kDispenserBelowM = 0.8f;
        constexpr float kDispenserSideM = 0.7f;
        constexpr double kRedReactionS = 1.5;
        constexpr double kSupportPeriodS = 1.0;
        constexpr float kSeparationSpeedMps = 20.0f;
        // Sim choice: the seeker accepts returns this close to where it expects
        // the target, the same radius the track store associates within.
        constexpr float kSeekerGateM = kDefaultAssociationGateM;
        // Sim choices for the pulse-Doppler model every radar here shares (not
        // a measured radar): an echo whose ground speed along the beam is under
        // 30 m/s is lost in the clutter when the beam looks down onto ground
        // inside 20 km, and a tracking radar takes echoes within 40 m/s of the
        // closing speed it expects. A 250 m/s fighter has to hold its beam
        // aspect within about 7 degrees to sit in the notch.
        constexpr float kClutterNotchMps = 30.0f;
        constexpr float kClutterReachM = 20000.0f;
        constexpr float kVelocityGateMps = 40.0f;
        // Sim choice: the cloud loses its release speed with a 0.4 s time
        // constant (still, in under 2 s) and settles to a 1.5 m/s fall.
        constexpr double kChaffDragTimeS = 0.4;
        constexpr float kChaffFallSpeedMps = 1.5f;

        glm::vec3 unitOr(const glm::vec3 &value, const glm::vec3 &fallback)
        {
            const float length = glm::length(value);
            return length > 1.0e-4f ? value / length : fallback;
        }

        // Declared fictional set. The same single-pulse equation as sensor model 1.
        // Not a measured radar, and not a lock range.
        DopplerFilter pulseDoppler()
        {
            DopplerFilter filter;
            filter.clutterNotchMps = kClutterNotchMps;
            filter.clutterReachM = kClutterReachM;
            filter.velocityGateMps = kVelocityGateMps;
            return filter;
        }

        RadarSet fighterRadar()
        {
            RadarSet radar;
            radar.peakPowerW = 10000.0f;
            radar.gainTransmit = 1000.0f;
            radar.gainReceive = 1000.0f;
            radar.wavelengthM = 0.03f;
            radar.pulseWidthS = 1.0e-6f;
            radar.systemTemperatureK = 290.0f;
            radar.systemLoss = 1.0f;
            radar.snrThreshold = 10.0f;
            return radar;
        }

        // Smaller aperture and power than the fighter set, so the same equation
        // only clears the gate at a shorter range. The activation range on the
        // homing spec is when the seeker is allowed to look, not a detection range.
        RadarSet seekerRadar()
        {
            RadarSet radar = fighterRadar();
            radar.peakPowerW = 1000.0f;
            radar.gainTransmit = 100.0f;
            radar.gainReceive = 100.0f;
            return radar;
        }

        RadarCrossSectionProfile fighterSignature()
        {
            RadarCrossSectionProfile profile;
            profile.noseM2 = 1.0f;
            profile.beamM2 = 1.0f;
            profile.tailM2 = 1.0f;
            return profile;
        }

        ScanVolume fighterVolume()
        {
            ScanVolume volume;
            volume.azimuthHalfRad = 0.523599f;   // ±30 degrees
            volume.elevationHalfRad = 0.261799f; // ±15 degrees
            volume.beamwidthRad = 0.209440f;     // 12 degrees, so the nose bar covers co-altitude
            volume.bars = 1;
            volume.dwellS = 0.05;
            // Sim choice: a held beam follows its contact out to ±60 degrees,
            // twice the search volume, so a lock survives a turn the raster
            // would lose it in.
            volume.gimbalHalfRad = 1.047198f;
            return volume;
        }

        RadarHomingSpec radarRoundSpec()
        {
            RadarHomingSpec spec;
            spec.navigationGain = 4.0f;
            spec.maxAcceleration = 300.0f;
            spec.datalinkPeriodS = static_cast<float>(kSupportPeriodS);
            spec.seekerActivationRangeM = 12000.0f;
            spec.seekerConeHalfRad = 0.5f;
            spec.seekerSnrThreshold = 10.0f;
            spec.seekerRadar = seekerRadar();
            spec.targetRcs = fighterSignature();
            spec.seekerGateM = kSeekerGateM;
            spec.doppler = pulseDoppler();
            spec.searchTimeoutS = 6.0;
            spec.memoryTimeoutS = 3.0;
            return spec;
        }

        SensorBody toSensorBody(const PlatformSnapshot &snap)
        {
            SensorBody body;
            body.id = snap.id;
            body.position = snap.position;
            body.velocity = snap.velocity;
            body.forward = snap.forward;
            body.up = snap.up;
            body.throttle = snap.throttle;
            body.afterburner = snap.afterburner;
            body.alive = snap.alive;
            return body;
        }

        const TrackEstimate *findTrack(const ScanRadar &radar, EntityId id)
        {
            if (!id.valid())
            {
                return nullptr;
            }
            for (const TrackEstimate &track : radar.tracks().tracks())
            {
                if (track.id == id)
                {
                    return &track;
                }
            }
            return nullptr;
        }

        bool shotInFlight(const Shot &shot)
        {
            return shot.radarGuided && !shot.ended();
        }
    }

    void World::configureRadars()
    {
        m_radarVolume = fighterVolume();
        const RadarSet radar = fighterRadar();
        const RadarCrossSectionProfile signature = fighterSignature();
        m_playerRadar.setVolume(m_radarVolume);
        m_playerRadar.setRadar(radar, signature);
        m_playerRadar.setDoppler(pulseDoppler());
        m_redRadar.setVolume(m_radarVolume);
        m_redRadar.setRadar(radar, signature);
        m_redRadar.setDoppler(pulseDoppler());
    }

    void World::resetRadarStores()
    {
        m_radarMagazine = kPlayerRadarRounds;
        m_redRadarMagazine = kRedRadarRounds;
        m_chaff.remaining = kPlayerChaff;
        m_chaff.rcsM2 = 20.0f;
        m_chaff.lifetimeS = 8.0;
        m_chaff.ejectSpeed = 30.0f;
        m_chaff.dragTimeS = kChaffDragTimeS;
        m_chaff.fallSpeedMps = kChaffFallSpeedMps;
        m_chaffRounds.clear();
        m_playerFlares = kPlayerFlares;
        m_lastChaffTime = -1.0e9;
        m_lastFlareTime = -1.0e9;
        m_chaffSide = 1.0f;
        m_designatedTrack = kNoEntity;
        m_playerWarnings = {};
        m_redQualitySince = -1.0;
        configureRadars();
    }

    void World::rearmRadarStores()
    {
        m_radarMagazine = kPlayerRadarRounds;
        m_chaff.remaining = kPlayerChaff;
        m_playerFlares = kPlayerFlares;
    }

    void World::rearmHostileRadar()
    {
        m_redRadarMagazine = kRedRadarRounds;
        m_redQualitySince = -1.0;
        for (const auto &shot : m_shots)
        {
            if (shot->team == Team::Red && shotInFlight(*shot))
            {
                return;
            }
        }
        m_redRadar.setVolume(m_radarVolume);
        m_redRadar.setRadar(fighterRadar(), fighterSignature());
        m_redRadar.setDoppler(pulseDoppler());
    }

    void World::setFox3(const std::string &fox3Id)
    {
        if (fox3::find(fox3Id.c_str()) != nullptr)
        {
            m_fox3Id = fox3Id;
        }
    }

    LaunchBlock World::fox3Refusal() const
    {
        const fox3::Spec *spec = fox3::find(m_fox3Id.c_str());
        if (spec == nullptr)
        {
            return LaunchBlock::None;
        }
        switch (spec->refusal)
        {
        case fox3::LaunchRefusal::None:
            return LaunchBlock::None;
        case fox3::LaunchRefusal::SemiActiveMidcourse:
            return LaunchBlock::SemiActiveNotInBuild;
        case fox3::LaunchRefusal::DuctedRocket:
            return LaunchBlock::ThrustModelUnpublished;
        case fox3::LaunchRefusal::SeekerUnresolved:
            return LaunchBlock::SeekerUnresolved;
        case fox3::LaunchRefusal::SecondarySource:
            return LaunchBlock::SecondarySource;
        case fox3::LaunchRefusal::NoPerformanceCard:
            return LaunchBlock::NoPerformanceCard;
        }
        return LaunchBlock::None;
    }

    void World::cycleFighterWeapon()
    {
        m_fighterWeapon = m_fighterWeapon == FighterWeapon::Fox2 ? FighterWeapon::RadarRound : FighterWeapon::Fox2;
    }

    void World::cycleRadarDesignation()
    {
        // Living tracks, nearest the nose first. Ties keep id order.
        struct Candidate
        {
            EntityId id;
            float offNoseRad = 0.0f;
        };
        std::vector<Candidate> live;
        const bool haveJet = m_fighter && !m_fighterRespawnPending;
        const glm::vec3 nose = haveJet ? unitOr(m_fighter->getNose(), glm::vec3(0.0f, 0.0f, 1.0f)) : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 origin = haveJet ? m_fighter->getPosition() : glm::vec3(0.0f);
        for (const TrackEstimate &track : m_playerRadar.tracks().tracks())
        {
            if (track.life == TrackLife::Lost)
            {
                continue;
            }
            const glm::vec3 lineOfSight = unitOr(track.position - origin, nose);
            const float offNose = haveJet ? std::acos(std::clamp(glm::dot(lineOfSight, nose), -1.0f, 1.0f)) : 0.0f;
            live.push_back(Candidate{track.id, offNose});
        }
        std::stable_sort(live.begin(), live.end(), [](const Candidate &a, const Candidate &b) {
            return a.offNoseRad < b.offNoseRad;
        });

        // Search -> nearest the nose -> ... -> furthest -> search. A
        // designation that is no longer listed starts again from the nearest.
        EntityId next = live.empty() ? kNoEntity : live.front().id;
        for (std::size_t index = 0; index < live.size(); ++index)
        {
            if (live[index].id == m_designatedTrack)
            {
                next = index + 1 < live.size() ? live[index + 1].id : kNoEntity;
                break;
            }
        }
        m_designatedTrack = next;
        m_playerRadar.setDesignatedTrack(m_designatedTrack);
    }

    bool World::radarLockPoint(glm::vec3 &point) const
    {
        if (m_role != PlayerRole::Fighter || !m_fighter || m_fighterRespawnPending || !m_playerRadar.singleTargetTrack() ||
            m_playerRadar.designatedTrack() != m_designatedTrack)
        {
            return false;
        }
        const TrackEstimate *track = findTrack(m_playerRadar, m_designatedTrack);
        if (track == nullptr || track->life == TrackLife::Lost)
        {
            return false;
        }
        point = track->position + track->velocity * static_cast<float>(std::max(0.0, time() - track->time));
        return true;
    }

    FireControl World::fireControl() const
    {
        FireControl picture;
        picture.weapon = m_fighterWeapon;
        picture.clearance = launchClearance();
        picture.radarRound = radarFlightStatus();
        const bool haveJet = m_role == PlayerRole::Fighter && m_fighter && !m_fighterRespawnPending;
        if (!haveJet)
        {
            return picture;
        }

        picture.radar = m_playerRadar.singleTargetTrack() ? RadarMode::Track : RadarMode::Search;
        picture.rounds = m_fighterWeapon == FighterWeapon::RadarRound ? m_radarMagazine : std::max(roundsRemaining(), 0);

        // The lock, from the track estimate alone.
        if (const TrackEstimate *track = findTrack(m_playerRadar, m_designatedTrack))
        {
            if (track->life != TrackLife::Lost)
            {
                SensorBody ownship;
                ownship.position = m_fighter->getPosition();
                ownship.forward = m_fighter->getNose();
                ownship.up = m_fighter->getUp();
                const double age = std::max(0.0, time() - track->time);
                LockPicture &lock = picture.lock;
                lock.valid = true;
                lock.track = track->id;
                lock.life = track->life;
                lock.position = track->position + track->velocity * static_cast<float>(age);
                lock.velocity = track->velocity;
                const SensorBearing bearing = bearingFrom(sensorAxes(ownship), ownship.position, lock.position);
                lock.rangeM = bearing.rangeM;
                lock.azimuthRad = bearing.azimuthRad;
                lock.elevationRad = bearing.elevationRad;
                lock.closingMps = glm::dot(m_fighter->getVelocity() - track->velocity, bearing.lineOfSight);
                lock.ageS = std::max(0.0, time() - track->lastMeasurementTime);
                lock.launchQuality = isLaunchQuality(*track, time(), radarLaunchAgeLimit());
            }
        }

        SeekerPicture &seeker = picture.seeker;
        seeker.lookDirection = unitOr(m_fighter->getNose(), glm::vec3(0.0f, 0.0f, 1.0f));
        if (m_fighterWeapon == FighterWeapon::RadarRound)
        {
            // The radar round's own seeker stays cold on the rail; the radar is its eye.
            seeker.family = SensorFamily::Radar;
            seeker.state = SeekerState::Off;
            return picture;
        }

        seeker.family = SensorFamily::Infrared;
        const Missile *round = readyRound();
        const fox2::Spec *spec = round != nullptr ? round->fox2Spec() : nullptr;
        if (spec == nullptr)
        {
            seeker.state = SeekerState::Off;
            return picture;
        }
        seeker.gimbalHalfRad = glm::radians(std::max(spec->gimbalDeg, 0.0f));
        if (!seekerPowered())
        {
            seeker.state = SeekerState::Caged;
            return picture;
        }

        glm::vec3 cuePoint{0.0f};
        const bool slaved = radarLockPoint(cuePoint);
        seeker.lookDirection = unitOr(round->getSeekerBoresight(), seeker.lookDirection);
        const Target *held = round->getTargetObject();
        if (round->hasFox2InfraredLock())
        {
            seeker.state = SeekerState::Locked;
            seeker.tone = 1.0f;
        }
        else if (held != nullptr && held->isActive())
        {
            seeker.state = SeekerState::Designated;
            seeker.tone = 0.6f;
        }
        else
        {
            seeker.state = slaved ? SeekerState::Slaved : SeekerState::Search;
            // Growl: rises as heat comes near the head's line of sight. It
            // reads aircraft positions the way the rail seeker itself does.
            const glm::vec3 head = round->getPosition();
            float nearest = seeker.gimbalHalfRad;
            for (const auto &target : m_targets)
            {
                if (!target || !target->isActive())
                {
                    continue;
                }
                const glm::vec3 lineOfSight = unitOr(target->getPosition() - head, seeker.lookDirection);
                nearest = std::min(nearest, std::acos(std::clamp(glm::dot(lineOfSight, seeker.lookDirection), -1.0f, 1.0f)));
            }
            if (seeker.gimbalHalfRad > 1.0e-4f)
            {
                const float closeness = 1.0f - std::clamp(nearest / seeker.gimbalHalfRad, 0.0f, 1.0f);
                seeker.tone = 0.5f * closeness * closeness;
            }
        }
        return picture;
    }

    RadarFlightStatus World::radarFlightStatus() const
    {
        RadarFlightStatus status;
        for (const auto &shot : m_shots)
        {
            if (shot->team != Team::Blue || !shotInFlight(*shot))
            {
                continue;
            }
            status.flying = true;
            status.phase = shot->homing.state().phase;
            status.supportFresh = shot->supportFresh;
            status.seekerTracking = shot->homing.state().seekerTracking;
        }
        return status;
    }

    std::vector<SensorBody> World::sensorBodies(EntityId ownship, const Team *friendly) const
    {
        std::vector<SensorBody> bodies;
        for (const PlatformSnapshot &snap : platformSnapshots())
        {
            if (!snap.alive || (ownship.valid() && snap.id == ownship) || (friendly != nullptr && snap.team == *friendly))
            {
                continue;
            }
            bodies.push_back(toSensorBody(snap));
        }
        for (const ChaffRound &round : m_chaffRounds)
        {
            if (!round.alive || round.rcsM2 <= 0.0f)
            {
                continue;
            }
            SensorBody body;
            body.id = round.id;
            body.position = round.position;
            body.velocity = round.velocity;
            body.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            body.up = glm::vec3(0.0f, 1.0f, 0.0f);
            body.alive = true;
            body.radarCrossSectionM2 = round.rcsM2;
            bodies.push_back(body);
        }
        return bodies;
    }

    LaunchBlock World::radarClearance(const ScanRadar &radar, EntityId track, int magazine) const
    {
        if (magazine <= 0)
        {
            return LaunchBlock::RadarMagazineEmpty;
        }
        const TrackEstimate *estimate = findTrack(radar, track);
        if (estimate == nullptr || !isLaunchQuality(*estimate, time(), radarLaunchAgeLimit()))
        {
            return LaunchBlock::NoLaunchQuality;
        }
        return LaunchBlock::None;
    }

    LaunchResult World::launchRadarRound(Team team, EntityId owner, ScanRadar &radar, EntityId track, int &magazine)
    {
        // The opponent always flies the reference round. A named card the player
        // selected must not change that shot, and a refused card must not spend a round.
        if (team == Team::Blue)
        {
            const LaunchBlock refused = fox3Refusal();
            if (refused != LaunchBlock::None)
            {
                return LaunchResult{refused, kNoEntity};
            }
        }

        const LaunchBlock block = radarClearance(radar, track, magazine);
        if (block != LaunchBlock::None)
        {
            return LaunchResult{block, kNoEntity};
        }

        glm::vec3 origin{0.0f};
        glm::vec3 velocity{0.0f};
        glm::vec3 nose{0.0f, 0.0f, 1.0f};
        if (team == Team::Blue)
        {
            if (!m_fighter || m_fighterRespawnPending)
            {
                return LaunchResult{LaunchBlock::NoLauncher, kNoEntity};
            }
            origin = m_fighter->getPosition();
            velocity = m_fighter->getVelocity();
            nose = unitOr(m_fighter->getNose(), nose);
            origin += nose * (std::max(m_fighter->getRadius(), 1.0f) + 3.0f);
        }
        else
        {
            Target *shooter = findTarget(owner);
            if (shooter == nullptr || !shooter->isActive())
            {
                return LaunchResult{LaunchBlock::NoLauncher, kNoEntity};
            }
            origin = shooter->getPosition();
            velocity = shooter->getVelocity();
            const TrackEstimate *estimate = findTrack(radar, track);
            const glm::vec3 toward = estimate != nullptr ? estimate->position - origin : glm::vec3(0.0f, 0.0f, 1.0f);
            nose = glm::length(velocity) > 5.0f ? unitOr(velocity, toward) : unitOr(toward, nose);
            origin += nose * 12.0f;
        }

        // Red, and a blue "reference" card, both take referenceFlyout(). Named
        // solids may substitute a labeled shell. The 18 m radius below is this
        // build's software hit test. It is not a warhead and it is not on the card.
        fox3::Flyout body = fox3::referenceFlyout();
        if (team == Team::Blue)
        {
            const fox3::Spec *spec = fox3::find(m_fox3Id.c_str());
            if (spec != nullptr && spec->refusal == fox3::LaunchRefusal::None && spec->shell != fox3::Shell::Refused)
            {
                body = fox3::flyout(*spec);
            }
        }

        auto missile = std::make_unique<Missile>(origin, velocity + nose * kSeparationSpeedMps, body.massKg,
                                                 body.dragCoefficient, body.areaM2, 0.0f);
        missile->setGuidanceEnabled(false);
        missile->setTerrainAvoidanceEnabled(false);
        missile->setProximityFuseRadius(18.0f);
        missile->setGroundReferenceAltitude(groundLevel());
        missile->setThrust(body.thrustN);
        missile->setThrottle(1.0f);
        missile->setThrustEnabled(true);
        missile->setThrustDirection(nose);
        missile->setBodyForward(nose);
        missile->setFuel(body.fuelKg);
        missile->setFuelConsumptionRate(body.fuelPerS);
        missile->clearTarget();
        missile->setFuzeHeld(true);
        missile->setExternalAcceleration(glm::vec3(0.0f));

        auto shot = std::make_unique<Shot>();
        shot->id = allocateId();
        shot->owner = owner;
        shot->team = team;
        shot->launchKind = LaunchKind::Rail;
        shot->designatedTarget = kNoEntity;
        shot->seekerSource = kNoEntity;
        shot->launchTime = time();
        shot->launchPosition = missile->getPosition();
        shot->stepStartPosition = shot->launchPosition;
        shot->motorBurning = true;
        shot->fuzeArmed = false;
        shot->radarGuided = true;
        shot->supportTrack = track;
        shot->nextSupportTime = time();
        shot->homing.reset(radarRoundSpec());
        missile->setEntityId(shot->id);
        m_physics->addObject(missile.get());
        shot->missile = std::move(missile);

        SimEvent launched = makeEvent(EventType::WeaponLaunched, shot->id, kNoEntity);
        launched.position = shot->launchPosition;
        launched.velocity = shot->missile->getVelocity();
        launched.value = glm::length(shot->missile->getVelocity());
        launched.detail = static_cast<std::uint8_t>(LaunchKind::Rail);
        publish(launched);
        SimEvent ignition = makeEvent(EventType::MotorIgnition, shot->id);
        ignition.position = shot->launchPosition;
        ignition.velocity = nose;
        publish(ignition);

        const EntityId id = shot->id;
        m_shots.push_back(std::move(shot));
        --magazine;
        return LaunchResult{LaunchBlock::None, id};
    }

    bool World::dispenseChaff()
    {
        if (!m_fighter || m_fighterRespawnPending || m_chaff.remaining <= 0 || time() < m_lastChaffTime + kChaffIntervalS)
        {
            return false;
        }

        // Down and out of the belly, left and right dispensers in turn.
        const glm::vec3 nose = m_fighter->getNose();
        const glm::vec3 up = m_fighter->getUp();
        const glm::vec3 right = m_fighter->getRight();
        const glm::vec3 origin = m_fighter->getPosition() - nose * kDispenserAftM - up * kDispenserBelowM +
                                 right * (kDispenserSideM * m_chaffSide);
        const glm::vec3 eject = -up + right * (0.35f * m_chaffSide);
        const EntityId id = allocateId();
        // The free function shares this method's name. Qualify it.
        const bool released = ::missilesim::sim::releaseChaff(m_chaff, m_chaffRounds, id, time(), origin,
                                                              m_fighter->getVelocity(), eject);
        if (!released)
        {
            return false;
        }
        m_lastChaffTime = time();
        m_chaffSide = -m_chaffSide;

        SimEvent event = makeEvent(EventType::CountermeasureReleased, id, m_fighterId);
        event.position = origin;
        event.velocity = m_chaffRounds.back().velocity;
        publish(event);
        return true;
    }

    bool World::dispenseFlares()
    {
        if (!m_fighter || m_fighterRespawnPending || m_playerFlares <= 0 || time() < m_lastFlareTime + kFlareIntervalS)
        {
            return false;
        }

        // The same cartridge the opponents carry (heat, burn time, drag), shot
        // down and outboard so the pair fans apart under the tail.
        const TargetFlareConfig &cartridge = m_config.targets.flares;
        const glm::vec3 nose = m_fighter->getNose();
        const glm::vec3 up = m_fighter->getUp();
        const glm::vec3 right = m_fighter->getRight();
        const glm::vec3 base = m_fighter->getPosition() - nose * kDispenserAftM - up * kDispenserBelowM;
        bool any = false;
        for (const float side : {1.0f, -1.0f})
        {
            if (m_playerFlares <= 0)
            {
                break;
            }
            FlareLaunchRequest request;
            request.position = base + right * (kDispenserSideM * side);
            request.velocity = m_fighter->getVelocity() +
                               glm::normalize(-up * 0.8f + right * (0.45f * side) - nose * 0.25f) * cartridge.ejectSpeed;
            request.mass = cartridge.mass;
            request.dragCoefficient = cartridge.dragCoefficient;
            request.crossSectionalArea = cartridge.crossSectionalArea;
            request.lifetime = cartridge.lifetime;
            request.heatSignature = cartridge.heatSignature;
            request.heatDecayRate = cartridge.heatDecayRate;
            --m_playerFlares;

            auto flare = std::make_unique<Flare>(request);
            if (!flare->isActive())
            {
                continue;
            }
            const EntityId id = allocateId();
            flare->setEntityId(id);
            m_physics->addFlare(flare.get());
            SimEvent event = makeEvent(EventType::CountermeasureReleased, id, m_fighterId);
            event.position = flare->getPosition();
            event.velocity = flare->getVelocity();
            publish(event);
            m_flares.push_back(std::move(flare));
            any = true;
        }
        m_lastFlareTime = time();
        return any;
    }

    void World::stepRadarBeforePhysics()
    {
        stepChaff(m_chaffRounds, time(), m_fixedStep);

        if (m_fighter && !m_fighterRespawnPending)
        {
            PlatformSnapshot ownship;
            if (snapshot(m_fighterId, ownship) && ownship.alive)
            {
                m_playerRadar.update(time(), terrain(), toSensorBody(ownship), sensorBodies(m_fighterId, &ownship.team));
            }
        }

        const TrackEstimate *designated = findTrack(m_playerRadar, m_designatedTrack);
        if (m_designatedTrack.valid() && (designated == nullptr || designated->life == TrackLife::Lost))
        {
            m_designatedTrack = kNoEntity;
            m_playerRadar.setDesignatedTrack(kNoEntity);
        }

        Target *red = nullptr;
        for (const auto &target : m_targets)
        {
            if (target && target->isActive())
            {
                red = target.get();
                break;
            }
        }

        PlatformSnapshot redPose;
        const bool redRadarOn = m_redRadarEnabled && red != nullptr && snapshot(red->getEntityId(), redPose) && redPose.alive;
        if (redRadarOn)
        {
            m_redRadar.update(time(), terrain(), toSensorBody(redPose), sensorBodies(red->getEntityId(), &redPose.team));

            // When the held beam drops back to the raster, steer the first
            // living track. The designation lives inside the red radar.
            if (!m_redRadar.singleTargetTrack())
            {
                for (const TrackEstimate &track : m_redRadar.tracks().tracks())
                {
                    if (track.life != TrackLife::Lost)
                    {
                        m_redRadar.setDesignatedTrack(track.id);
                        break;
                    }
                }
            }

            // Fire only on the track the beam is holding, never on another
            // contact that merely happens to be confirmed.
            EntityId quality = kNoEntity;
            if (const TrackEstimate *held = findTrack(m_redRadar, m_redRadar.designatedTrack()))
            {
                if (m_redRadar.singleTargetTrack() && isLaunchQuality(*held, time(), radarLaunchAgeLimit()))
                {
                    quality = held->id;
                }
            }
            bool redRoundFlying = false;
            for (const auto &shot : m_shots)
            {
                if (shot->team == Team::Red && shotInFlight(*shot))
                {
                    redRoundFlying = true;
                    break;
                }
            }
            if (!quality.valid())
            {
                m_redQualitySince = -1.0;
            }
            else if (!redRoundFlying && m_redRadarMagazine > 0)
            {
                if (!(m_redQualitySince >= 0.0))
                {
                    m_redQualitySince = time();
                }
                if (time() >= m_redQualitySince + kRedReactionS)
                {
                    const LaunchResult fired =
                        launchRadarRound(Team::Red, red->getEntityId(), m_redRadar, quality, m_redRadarMagazine);
                    if (fired.launched())
                    {
                        m_redQualitySince = -1.0;
                    }
                }
            }
        }

        const std::vector<SensorBody> bodies = sensorBodies(kNoEntity);
        for (const auto &shotPointer : m_shots)
        {
            Shot &shot = *shotPointer;
            if (!shotInFlight(shot) || !shot.missile)
            {
                continue;
            }

            Missile &missile = *shot.missile;
            const glm::vec3 velocity = missile.getVelocity();
            if (glm::length(velocity) > 30.0f)
            {
                // The airframe weathervanes onto its velocity. The seeker then
                // reads that body axis; guidance does not aim along velocity itself.
                const glm::vec3 nose = glm::normalize(velocity);
                missile.setBodyForward(nose);
                missile.setThrustDirection(nose);
            }

            SupportPacket packet;
            const SupportPacket *packetPtr = nullptr;
            if (time() + 1.0e-9 >= shot.nextSupportTime)
            {
                const ScanRadar &radar = shot.team == Team::Blue ? m_playerRadar : m_redRadar;
                if (const TrackEstimate *track = findTrack(radar, shot.supportTrack))
                {
                    if (track->life != TrackLife::Lost)
                    {
                        packet.time = track->time;
                        packet.position = track->position;
                        packet.velocity = track->velocity;
                        packet.fresh = true;
                        packetPtr = &packet;
                        shot.nextSupportTime = time() + kSupportPeriodS;
                    }
                }
            }

            SensorBody nose;
            nose.id = shot.id;
            nose.position = missile.getPosition();
            nose.velocity = velocity;
            nose.forward = missile.getBodyForward();
            nose.up = glm::vec3(0.0f, 1.0f, 0.0f);
            nose.alive = true;

            const HomingCommand command = shot.homing.step(time(), m_fixedStep, missile.getPosition(), velocity, packetPtr,
                                                           terrain(), nose, bodies);
            missile.setExternalAcceleration(command.acceleration);
            shot.supportFresh = command.supportFresh;
            shot.seekerEmitting = command.seekerOn;
        }

        m_playerWarnings = {};
        if (m_fighter && !m_fighterRespawnPending)
        {
            PlatformSnapshot ownship;
            if (snapshot(m_fighterId, ownship) && ownship.alive)
            {
                std::vector<Emission> emissions;
                // The receiver hears the radar only while its energy is on this
                // aircraft: in its scan volume, or in the beam it is holding.
                if (redRadarOn && m_redRadar.illuminates(toSensorBody(redPose), ownship.position))
                {
                    Emission emission;
                    emission.source = red->getEntityId();
                    emission.position = redPose.position;
                    emission.family = SensorFamily::Radar;
                    emission.missileSeeker = false;
                    emission.singleTargetTrack = m_redRadar.singleTargetTrack();
                    // The held beam also carries a round's datalink once one
                    // is flying on the track it holds.
                    for (const auto &flying : m_shots)
                    {
                        if (emission.singleTargetTrack && flying->team == Team::Red && shotInFlight(*flying) &&
                            flying->supportTrack == m_redRadar.designatedTrack())
                        {
                            emission.guidingMissile = true;
                        }
                    }
                    emissions.push_back(emission);
                }
                std::vector<ClosingBody> closing;
                for (const auto &shotPointer : m_shots)
                {
                    const Shot &shot = *shotPointer;
                    if (shot.ended() || !shot.missile || shot.owner == m_fighterId)
                    {
                        continue;
                    }
                    if (shot.radarGuided && shot.seekerEmitting)
                    {
                        Emission emission;
                        emission.source = shot.id;
                        emission.position = shot.missile->getPosition();
                        emission.family = SensorFamily::Radar;
                        emission.missileSeeker = true;
                        emission.singleTargetTrack = false;
                        emissions.push_back(emission);
                    }
                    ClosingBody body;
                    body.id = shot.id;
                    body.position = shot.missile->getPosition();
                    body.velocity = shot.missile->getVelocity();
                    body.infraredOnly = shot.missile->isFox2();
                    closing.push_back(body);
                }

                WarningSet set;
                set.radarWarning = true;
                set.approachWarning = true;
                m_playerWarnings = hear(time(), set, toSensorBody(ownship), terrain(), emissions, closing);
            }
        }
    }

    void World::stepRadarAfterOutcomes()
    {
        for (const auto &shotPointer : m_shots)
        {
            Shot &shot = *shotPointer;
            if (!shotInFlight(shot) || !shot.missile)
            {
                continue;
            }
            if (shot.homing.state().phase == RadarHomingState::Phase::Ended)
            {
                endShot(shot, ShotEndReason::SelfDestruct, kNoEntity, shot.missile->getPosition(), time());
            }
        }
    }
}
