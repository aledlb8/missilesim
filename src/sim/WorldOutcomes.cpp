// Shot bookkeeping and outcome resolution: fuze and contact tests against
// every platform, detonation, damage, destruction and the end of each shot.
//
// Guidance, fuzing, physical contact and damage are separate here. A round
// that has lost its seeker track still strikes whatever its path crosses, a
// proximity fuze fires on the first platform inside its volume (it cannot
// know which aircraft the seeker wanted), and a detonation damages every
// platform its blast reaches, including the one that fired it.
#include "World.h"

#include "objects/Fighter.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "physics/PhysicsEngine.h"
#include "sim/EngagementRules.h"
#include "sim/Sweep.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

namespace missilesim::sim
{
    namespace
    {
        enum class Trigger
        {
            Dud,
            Contact,
            Proximity,
        };

        // Blast damage at a surface distance from the burst: full inside the
        // lethal radius, then a linear fall to zero (see EngagementRules.h).
        float blastDamage(float surfaceDistance, float lethalRadius)
        {
            if (!(lethalRadius > 0.0f))
            {
                return 0.0f;
            }
            if (surfaceDistance <= lethalRadius)
            {
                return 1.0f;
            }
            const float outer = lethalRadius * rules::kBlastFalloffOuterFactor;
            if (surfaceDistance >= outer)
            {
                return 0.0f;
            }
            return 1.0f - (surfaceDistance - lethalRadius) / (outer - lethalRadius);
        }
    }

    float World::missileBodyRadius(const Missile &missile) const
    {
        if (const missilesim::fox2::Spec *spec = missile.fox2Spec())
        {
            return std::max(spec->diameterM * 0.5f, 0.0f);
        }
        return std::sqrt(std::max(missile.getCrossSectionalArea(), 0.0f) / glm::pi<float>());
    }

    float World::lethalRadius(const Missile &missile) const
    {
        // Sandbox rule: the warhead is lethal out to its fuze radius. The
        // research gives no generic lethal radius, and a catalog round with
        // no published fuze radius is contact-only (radius 0).
        return std::max(missile.getProximityFuseRadius(), 0.0f);
    }

    const Shot *World::findShot(EntityId id) const
    {
        for (const auto &shot : m_shots)
        {
            if (shot->id == id)
            {
                return shot.get();
            }
        }
        return nullptr;
    }

    EntityId World::shotIdOf(const Missile *missile) const
    {
        return missile != nullptr ? missile->getEntityId() : kNoEntity;
    }

    void World::stepShotBookkeeping(Shot &shot)
    {
        if (shot.ended())
        {
            return;
        }

        Missile &missile = *shot.missile;
        shot.flightTime += m_fixedStep;

        // The radar round leaves the rail inside the shooter's fuze radius.
        // Hold the fuze until it has flown clear, or for one second.
        if (shot.radarGuided)
        {
            const float flown = glm::length(missile.getPosition() - shot.launchPosition);
            if (shot.flightTime >= 1.0f || flown >= 300.0f)
            {
                missile.setFuzeHeld(false);
            }
        }

        const bool burning = missile.isThrustEnabled() && missile.getFuel() > 0.0f;
        if (shot.motorBurning && !burning && !shot.coldLaunch.active)
        {
            shot.motorBurning = false;
            SimEvent event = makeEvent(EventType::MotorBurnout, shot.id);
            event.position = missile.getPosition();
            event.velocity = missile.getVelocity();
            publish(event);
        }

        if (!shot.fuzeArmed && missile.isFuzeArmed())
        {
            shot.fuzeArmed = true;
            SimEvent event = makeEvent(EventType::FuzeArmed, shot.id);
            event.position = missile.getPosition();
            event.value = glm::length(missile.getPosition() - shot.launchPosition);
            publish(event);
        }

        const EntityId source = targetIdOf(missile.getTargetObject());
        const bool decoy = missile.isTrackingDecoy();
        if (source != shot.seekerSource || decoy != shot.seekerOnDecoy)
        {
            shot.seekerSource = source;
            shot.seekerOnDecoy = decoy;
            shot.closingSampled = false;
            const Flare *flare = decoy ? missile.getTrackedFlare() : nullptr;
            SimEvent event = makeEvent(EventType::SeekerTargetChanged, shot.id,
                                       flare != nullptr ? flare->getEntityId() : source);
            event.position = missile.getPosition();
            event.detail = decoy ? 1 : 0;
            publish(event);
        }
    }

    void World::resolveShotOutcomes(const std::vector<PlatformMotion> &platforms)
    {
        const double stepStartTime = time() - static_cast<double>(m_fixedStep);
        const auto timeAt = [&](float fraction) {
            return stepStartTime + static_cast<double>(fraction) * static_cast<double>(m_fixedStep);
        };

        for (const auto &shotPointer : m_shots)
        {
            Shot &shot = *shotPointer;
            if (shot.ended())
            {
                continue;
            }

            Missile &missile = *shot.missile;
            const glm::vec3 start = shot.stepStartPosition;
            const glm::vec3 end = missile.getPosition();
            const bool armed = missile.isFuzeArmed();
            const float bodyRadius = missileBodyRadius(missile);
            const float fuzeRadius = armed ? std::max(missile.getProximityFuseRadius(), 0.0f) : 0.0f;

            // ---- Pass diagnostics against the seeker's current target -------
            if (shot.seekerSource.valid())
            {
                for (const PlatformMotion &platform : platforms)
                {
                    if (platform.id != shot.seekerSource)
                    {
                        continue;
                    }
                    const SweepResult pass = sweepClosestApproach(start, end, platform.start, platform.end);
                    shot.closestApproach = shot.closestApproach < 0.0f ? pass.distance : std::min(shot.closestApproach, pass.distance);

                    const Target *target = findTarget(platform.id);
                    const glm::vec3 offset = platform.end - end;
                    const float range = glm::length(offset);
                    if (target != nullptr && range > 1.0e-3f)
                    {
                        const float closing = -glm::dot(target->getVelocity() - missile.getVelocity(), offset / range);
                        if (shot.closingSampled && shot.lastClosingSpeed > 0.0f && closing <= 0.0f)
                        {
                            SimEvent event = makeEvent(EventType::ClosestApproach, shot.id, platform.id);
                            event.time = timeAt(pass.fraction);
                            event.position = lerpPosition(start, end, pass.fraction);
                            event.velocity = missile.getVelocity();
                            event.value = pass.distance;
                            publish(event);
                        }
                        shot.lastClosingSpeed = closing;
                        shot.closingSampled = true;
                    }
                    break;
                }
            }

            // ---- Fuze and contact: the earliest trigger across all platforms --
            float bestFraction = 2.0f;
            EntityId bestPlatform;
            Trigger bestTrigger = Trigger::Dud;
            for (const PlatformMotion &platform : platforms)
            {
                // The launcher is not a fuze or contact candidate until the round
                // arms: the round leaves its rail inside the fighter's radius.
                if (platform.id == shot.owner && !armed)
                {
                    continue;
                }
                const float contact = sweepEntryFraction(start, end, platform.start, platform.end, platform.radius + bodyRadius);
                const float proximity = fuzeRadius > 0.0f
                                            ? sweepEntryFraction(start, end, platform.start, platform.end, platform.radius + fuzeRadius)
                                            : -1.0f;

                float fraction = -1.0f;
                Trigger trigger = Trigger::Dud;
                if (armed && proximity >= 0.0f && (contact < 0.0f || proximity < contact))
                {
                    fraction = proximity;
                    trigger = Trigger::Proximity;
                }
                else if (contact >= 0.0f)
                {
                    fraction = contact;
                    trigger = armed ? Trigger::Contact : Trigger::Dud;
                }
                if (fraction >= 0.0f && fraction < bestFraction)
                {
                    bestFraction = fraction;
                    bestPlatform = platform.id;
                    bestTrigger = trigger;
                }
            }

            if (bestPlatform.valid())
            {
                const glm::vec3 point = lerpPosition(start, end, bestFraction);
                if (bestTrigger != Trigger::Proximity)
                {
                    SimEvent impact = makeEvent(EventType::Impact, shot.id, bestPlatform);
                    impact.time = timeAt(bestFraction);
                    impact.position = point;
                    impact.velocity = missile.getVelocity();
                    impact.value = bestTrigger == Trigger::Contact ? 1.0f : 0.0f;
                    publish(impact);
                }

                if (bestTrigger == Trigger::Dud)
                {
                    applyDamage(bestPlatform, rules::kDudImpactDamage, shot.id, point);
                    endShot(shot, ShotEndReason::DudImpact, bestPlatform, point, timeAt(bestFraction));
                }
                else
                {
                    const DetonationTrigger cause = bestTrigger == Trigger::Contact ? DetonationTrigger::Contact
                                                                                    : DetonationTrigger::Proximity;
                    detonate(shot, point, bestFraction, cause, bestPlatform, platforms);
                    endShot(shot,
                            bestTrigger == Trigger::Contact ? ShotEndReason::DirectHit : ShotEndReason::ProximityDetonation,
                            bestPlatform, point, timeAt(bestFraction));
                }
                continue;
            }

            // ---- Ground: first contact during the physics sub-steps ----------
            const auto &contacts = m_physics->getGroundContacts();
            const auto contact = std::find_if(contacts.begin(), contacts.end(),
                                              [&missile](const GroundContact &entry) { return entry.object == &missile; });
            if (contact != contacts.end())
            {
                const float fraction = std::clamp(contact->timeIntoUpdate / m_fixedStep, 0.0f, 1.0f);
                if (armed)
                {
                    detonate(shot, contact->position, fraction, DetonationTrigger::Ground, kNoEntity, platforms);
                }
                endShot(shot, ShotEndReason::GroundImpact, kNoEntity, contact->position, timeAt(fraction));
                continue;
            }

            // ---- The round ended itself (lost seeker, flight timer) -----------
            if (missile.consumeSelfDestructRequest())
            {
                if (armed)
                {
                    detonate(shot, end, 1.0f, DetonationTrigger::SelfDestruct, kNoEntity, platforms);
                }
                endShot(shot, ShotEndReason::SelfDestruct, kNoEntity, end, time());
                continue;
            }

            // ---- Left the simulated volume -----------------------------------
            if (glm::length(glm::vec2(end.x, end.z)) > rules::kArenaHorizontalRadiusM || end.y > rules::kArenaCeilingM ||
                !std::isfinite(end.x) || !std::isfinite(end.y) || !std::isfinite(end.z))
            {
                endShot(shot, ShotEndReason::LeftArena, kNoEntity, end, time());
                continue;
            }

            // ---- Custom round: the pass is over ------------------------------
            const float speed = glm::length(missile.getVelocity());
            if (!missile.isFox2() && shot.flightTime > rules::kOvershootMinFlightS && speed > 1.0f)
            {
                const Target *tracked = missile.getTargetObject();
                if (tracked != nullptr && tracked->isActive() && shot.closestApproach >= 0.0f)
                {
                    const glm::vec3 toTarget = tracked->getPosition() - end;
                    const float distance = glm::length(toTarget);
                    const float margin = std::max(tracked->getRadius() * rules::kOvershootRadiusMultiple, rules::kOvershootOpeningMarginM);
                    if (distance > shot.closestApproach + margin && distance > 1.0e-2f)
                    {
                        const glm::vec3 lineOfSight = toTarget / distance;
                        const float rangeRate = glm::dot(tracked->getVelocity() - missile.getVelocity(), lineOfSight);
                        const float lookCosine = glm::dot(missile.getVelocity() / speed, lineOfSight);
                        if (rangeRate > rules::kOvershootOpeningMps && lookCosine < rules::kOvershootBehindCosine)
                        {
                            endShot(shot, ShotEndReason::Overshot, targetIdOf(tracked), end, time());
                            continue;
                        }
                    }
                }
            }

            // ---- Burned out and too slow to fly ------------------------------
            if (!missile.isThrustEnabled() && !shot.coldLaunch.active && shot.flightTime > rules::kEnergyExhaustedMinFlightS &&
                speed < rules::kEnergyExhaustedSpeedMps)
            {
                endShot(shot, ShotEndReason::EnergyExhausted, kNoEntity, end, time());
                continue;
            }
        }
    }

    void World::detonate(Shot &shot, const glm::vec3 &point, float stepFraction, DetonationTrigger trigger,
                         EntityId triggeringPlatform, const std::vector<PlatformMotion> &platforms)
    {
        const double stepStartTime = time() - static_cast<double>(m_fixedStep);
        SimEvent event = makeEvent(EventType::Detonation, shot.id, triggeringPlatform);
        event.time = stepStartTime + static_cast<double>(stepFraction) * static_cast<double>(m_fixedStep);
        event.position = point;
        event.velocity = shot.missile->getVelocity();
        event.value = lethalRadius(*shot.missile);
        event.detail = static_cast<std::uint8_t>(trigger);
        publish(event);

        const float lethal = lethalRadius(*shot.missile);
        for (const PlatformMotion &platform : platforms)
        {
            float damage = 0.0f;
            if (trigger == DetonationTrigger::Contact && platform.id == triggeringPlatform)
            {
                damage = 1.0f; // the warhead went off against its skin
            }
            else
            {
                const glm::vec3 position = lerpPosition(platform.start, platform.end, stepFraction);
                const float surfaceDistance = std::max(glm::length(point - position) - platform.radius, 0.0f);
                damage = blastDamage(surfaceDistance, lethal);
            }
            if (damage > 0.0f)
            {
                applyDamage(platform.id, damage, shot.id, point);
            }
        }
    }

    void World::applyDamage(EntityId platform, float amount, EntityId cause, const glm::vec3 &point)
    {
        PlatformRecord *entry = record(platform);
        if (entry == nullptr || !entry->alive || !(amount > 0.0f))
        {
            return;
        }

        const float applied = std::min(amount, entry->health);
        entry->health -= applied;
        SimEvent event = makeEvent(EventType::Damage, platform, cause);
        event.position = point;
        event.value = applied;
        publish(event);

        if (entry->health <= rules::kDestroyedHealth)
        {
            destroyPlatform(platform, cause, point);
        }
    }

    void World::destroyPlatform(EntityId platform, EntityId cause, const glm::vec3 &point)
    {
        PlatformRecord *entry = record(platform);
        if (entry == nullptr || !entry->alive)
        {
            return; // destroyed once
        }
        entry->alive = false;
        entry->health = 0.0f;

        SimEvent event = makeEvent(EventType::PlatformDestroyed, platform, cause);
        event.position = point;
        if (entry->kind == PlatformKind::Target)
        {
            if (Target *target = findTarget(platform))
            {
                event.velocity = target->getVelocity();
                target->setActive(false);
            }
        }
        else if (m_fighter)
        {
            event.velocity = m_fighter->getVelocity();
            // Sandbox: the player is back in play at the end of the step.
            m_fighterRespawnPending = true;
        }
        publish(event);
    }

    void World::endShot(Shot &shot, ShotEndReason reason, EntityId other, const glm::vec3 &position, double eventTime)
    {
        if (shot.ended())
        {
            return;
        }
        shot.endReason = reason;
        shot.endOther = other;
        shot.endPosition = position;
        shot.coldLaunch.active = false;

        SimEvent event = makeEvent(EventType::ShotEnded, shot.id, other);
        event.time = eventTime;
        event.position = position;
        event.velocity = shot.missile->getVelocity();
        event.value = shot.closestApproach;
        event.detail = static_cast<std::uint8_t>(reason);
        publish(event);
    }

    void World::retireEndedShots()
    {
        for (auto it = m_shots.begin(); it != m_shots.end();)
        {
            if ((*it)->ended())
            {
                m_physics->removeObject((*it)->missile.get());
                it = m_shots.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void World::removeAllShots()
    {
        for (const auto &shot : m_shots)
        {
            endShot(*shot, ShotEndReason::Removed, kNoEntity, shot->missile->getPosition(), time());
        }
        retireEndedShots();
    }
}
