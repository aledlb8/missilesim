// The bridge between sim::World and the presentation: accessors, the
// engagement lifecycle, player role and stores actions, and the handling of
// simulation events (effects, sound, camera holds, launch notices).
#include "Application.h"
#include "ApplicationDetail.h"

#include <algorithm>
#include <iostream>

#include "audio/AudioSystem.h"
#include "objects/Fighter.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "sim/Fox2Catalog.h"
#include "sim/Fox3Catalog.h"

using missilesim::application::detail::safeNormalize;
namespace sim = missilesim::sim;

namespace
{
    const std::vector<std::unique_ptr<Target>> kNoTargets;
    const std::vector<std::unique_ptr<Flare>> kNoFlares;
}

// ---- Accessors ---------------------------------------------------------------------

PhysicsEngine *Application::physics() const
{
    return m_world ? &m_world->physics() : nullptr;
}

Fighter *Application::fighter() const
{
    return m_world ? m_world->fighter() : nullptr;
}

const std::vector<std::unique_ptr<Target>> &Application::targets() const
{
    return m_world ? m_world->targets() : kNoTargets;
}

const std::vector<std::unique_ptr<Flare>> &Application::flares() const
{
    return m_world ? m_world->flares() : kNoFlares;
}

const sim::Shot *Application::followedShot() const
{
    return m_world ? m_world->findShot(m_followedShot) : nullptr;
}

Missile *Application::focusMissile() const
{
    if (const sim::Shot *shot = followedShot())
    {
        return shot->missile.get();
    }
    return m_world ? m_world->readyRound() : nullptr;
}

bool Application::seekerUncaged() const
{
    return m_world && m_world->seekerUncaged();
}

sim::CustomRoundSpec Application::customRoundSpec() const
{
    sim::CustomRoundSpec spec = sim::customRoundSpecFromConfig(m_simulationConfig);
    spec.position = glm::vec3(m_initialPosition[0], m_initialPosition[1], m_initialPosition[2]);
    spec.velocity = glm::vec3(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]);
    spec.dryMass = m_mass;
    spec.dragCoefficient = m_dragCoefficient;
    spec.crossSectionalArea = m_crossSectionalArea;
    spec.liftCoefficient = m_liftCoefficient;
    spec.thrust = m_missileThrust;
    spec.fuelMass = m_missileFuel;
    spec.fuelConsumptionRate = m_missileFuelConsumptionRate;
    spec.guidanceEnabled = m_guidanceEnabled;
    spec.navigationGain = m_navigationGain;
    spec.maxSteeringForce = m_maxSteeringForce;
    spec.trackingAngleDegrees = m_trackingAngle;
    spec.proximityFuseRadius = m_proximityFuseRadius;
    spec.countermeasureResistance = m_countermeasureResistance;
    spec.terrainAvoidanceEnabled = m_terrainAvoidanceEnabled;
    spec.terrainClearance = m_terrainClearance;
    spec.terrainLookAheadTime = m_terrainLookAheadTime;
    if (m_renderer)
    {
        spec.padRestHeight = m_renderer->getMissileGroundRestOffset();
    }
    return spec;
}

// ---- Engagement lifecycle ------------------------------------------------------

void Application::selectTerrain(missilesim::sim::TerrainKind kind)
{
    m_terrainKind = kind;
    if (!m_world || m_world->terrain().kind() == kind)
    {
        return;
    }
    missilesim::sim::TerrainConfig terrain = m_simulationConfig.terrain;
    terrain.kind = kind;
    m_world->setTerrain(terrain);
    if (m_renderer)
    {
        m_renderer->setTerrain(m_world->sharedTerrain());
    }
    // Aircraft placed over the old ground would be inside the new one.
    restartWorld();
}

void Application::restartWorld()
{
    if (!m_world)
    {
        return;
    }

    // A fresh seed per engagement; the telemetry panel shows it so a run can
    // be replayed in the engagement harness.
    // The world keeps its role across a restart; it only differs from the
    // player's choice on the very first restart after startup.
    m_world->restart(m_seedSource());
    m_world->setTargetAIConfig(m_targetAIConfig);
    m_world->setCustomRoundSpec(customRoundSpec(), true);
    m_world->respawnTargets(m_targetCount);
    if (m_playerRole == PlayerRole::Fighter)
    {
        m_world->selectAircraft(m_aircraftId);
        m_aircraftId = m_world->aircraftId();
        if (m_world->role() != sim::PlayerRole::Fighter || m_world->fox2Id() != m_fox2Id)
        {
            m_world->setRoleFighter(m_fox2Id);
        }
        // Targets first: the fighter starts pointed at the lead aircraft.
        m_world->placeFighterAtEngagement();
        m_world->setRedRadar(m_hostileRadar);
    }
    else if (m_world->role() != sim::PlayerRole::Sam)
    {
        m_world->setRoleSam(customRoundSpec());
    }
    m_world->setFox3(m_fox3Id);
    m_world->drainEvents();

    m_clock.setStep(m_world->fixedStep());
    m_clock.reset();
    m_followedShot = sim::kNoEntity;
    m_detonationHoldActive = false;
    m_detonationHoldTimer = 0.0f;
    m_launchNotice = sim::LaunchBlock::None;
    m_launchNoticeTimer = 0.0f;
    m_shotEndNotice.clear();
    m_shotEndNoticeTimer = 0.0f;
    m_chaffDrawOrigin.clear();
    if (m_renderer)
    {
        m_renderer->clearEffects();
    }
    if (m_audioSystem)
    {
        m_audioSystem->stopAllEmitters();
    }
    invalidateTrajectoryPreviewCache();
}

void Application::resetTargets()
{
    if (!m_world)
    {
        return;
    }

    m_world->setTargetAIConfig(m_targetAIConfig);
    m_world->respawnTargets(m_targetCount);
    if (m_playerRole == PlayerRole::Fighter && m_hostileRadar)
    {
        m_world->rearmHostileRadar();
    }
    if (m_renderer)
    {
        m_renderer->clearEffects();
    }
    invalidateTrajectoryPreviewCache();
}

void Application::setPlayerRole(PlayerRole role)
{
    if (role == m_playerRole || !m_world)
    {
        return;
    }

    m_playerRole = role;
    m_followedShot = sim::kNoEntity;
    m_detonationHoldActive = false;
    if (role == PlayerRole::Sam)
    {
        m_fox2Id = "custom";
        m_world->setRoleSam(customRoundSpec());
        m_world->setRedRadar(false);
        return;
    }

    if (missilesim::fox2::find(m_fox2Id.c_str()) == nullptr)
    {
        m_fox2Id = "aim-9x-blk2";
    }
    m_world->selectAircraft(m_aircraftId);
    m_aircraftId = m_world->aircraftId();
    m_world->setRoleFighter(m_fox2Id);
    m_world->setRedRadar(m_hostileRadar);
    m_world->setFox3(m_fox3Id);
    setCameraMode(CameraMode::FIGHTER_JET);
    resetAimCamera();
}

void Application::selectFox2(const char *id)
{
    if (m_playerRole != PlayerRole::Fighter || id == nullptr || missilesim::fox2::find(id) == nullptr || !m_world)
    {
        return;
    }
    m_fox2Id = id;
    m_world->selectFox2(m_fox2Id);
}

void Application::selectFox3(const char *id)
{
    if (id == nullptr || missilesim::fox3::find(id) == nullptr || !m_world)
    {
        return;
    }
    m_fox3Id = id;
    m_world->setFox3(m_fox3Id);
    if (m_playerRole == PlayerRole::Fighter)
    {
        m_world->setFighterWeapon(sim::FighterWeapon::RadarRound);
    }
}

void Application::selectAircraft(const char *id)
{
    if (id == nullptr || !m_world)
    {
        return;
    }
    if (!m_world->selectAircraft(id))
    {
        return;
    }
    m_aircraftId = m_world->aircraftId();
}

void Application::rearm()
{
    if (!m_world)
    {
        return;
    }
    if (m_playerRole == PlayerRole::Sam)
    {
        m_world->setCustomRoundSpec(customRoundSpec(), false);
    }
    m_world->rearm();
}

// ---- Simulation events ---------------------------------------------------------

void Application::processSimEvents()
{
    if (!m_world)
    {
        return;
    }

    // Detonation events precede their ShotEnded event in the same drain, so
    // the shots listed here already have their explosion.
    std::vector<sim::EntityId> detonatedShots;
    for (const sim::SimEvent &event : m_world->drainEvents())
    {
        handleSimEvent(event, detonatedShots);
    }
}

void Application::handleSimEvent(const sim::SimEvent &event, std::vector<sim::EntityId> &detonatedShots)
{
    switch (event.type)
    {
    case sim::EventType::WeaponLaunched:
    {
        const auto kind = static_cast<sim::LaunchKind>(event.detail);
        if (kind == sim::LaunchKind::ColdLaunch && m_renderer)
        {
            // A big, lingering cloud of ejection gas across the pad; the
            // ignition plume follows once the motor lights in the air.
            const float ground = physics() ? physics()->getGroundLevel() : 0.0f;
            m_renderer->spawnLaunchGroundCloudEffect(glm::vec3(event.position.x, ground + 0.2f, event.position.z),
                                                     glm::vec3(0.0f, 1.0f, 0.0f), 1.4f);
        }
        if (m_audioSystem && kind != sim::LaunchKind::AirLaunch)
        {
            m_audioSystem->playLaunch(event.position);
        }
        break;
    }
    case sim::EventType::MotorIgnition:
    {
        if (!m_renderer)
        {
            break;
        }
        const sim::Shot *shot = m_world->findShot(event.subject);
        const bool rail = shot != nullptr && shot->launchKind == sim::LaunchKind::Rail;
        const glm::vec3 axis = shot != nullptr ? shot->missile->getThrustDirection() : safeNormalize(event.velocity, glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::vec3 velocity = shot != nullptr ? shot->missile->getVelocity() : event.velocity;
        float flash = 1.35f; // the cold-launch booster lights at four times sustainer thrust
        if (rail)
        {
            flash = (shot->missile->isReducedSmoke()) ? 0.35f : 1.0f;
        }
        m_renderer->spawnMissileLaunchEffect(event.position, axis, velocity, flash);
        break;
    }
    case sim::EventType::Detonation:
        createExplosion(event.position, event.velocity);
        detonatedShots.push_back(event.subject);
        break;
    case sim::EventType::ShotEnded:
    {
        const auto reason = static_cast<sim::ShotEndReason>(event.detail);
        const bool detonated = std::find(detonatedShots.begin(), detonatedShots.end(), event.subject) != detonatedShots.end();
        // A round that ends its flight without the warhead going off (spent,
        // overshot, a dud strike, an unarmed ground strike or destruct) still
        // breaks up; rounds that leave the arena or are cleared just vanish.
        const bool silent = reason == sim::ShotEndReason::LeftArena || reason == sim::ShotEndReason::Removed;
        if (!detonated && !silent)
        {
            createExplosion(event.position, event.velocity);
        }
        if (event.subject == m_followedShot)
        {
            if (silent)
            {
                finishDetonationHold();
            }
            else
            {
                beginDetonationHold(event.position);
            }
        }
        m_shotEndNotice = sim::shotEndReasonName(reason);
        m_shotEndNoticeTimer = 4.0f;
        std::cout << "Shot " << event.subject.value << " ended: " << sim::shotEndReasonName(reason);
        if (event.value >= 0.0f)
        {
            std::cout << ", closest approach " << event.value << " m";
        }
        std::cout << std::endl;
        break;
    }
    case sim::EventType::GroundCollision:
        createExplosion(event.position, event.velocity);
        break;
    case sim::EventType::PlatformRespawned:
        if (event.subject == m_world->fighterId())
        {
            resetAimCamera();
        }
        break;
    default:
        break;
    }
}
