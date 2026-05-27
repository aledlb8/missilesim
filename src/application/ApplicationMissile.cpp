#include "Application.h"
#include "ApplicationDetail.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <unordered_map>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

#include "audio/AudioSystem.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/Atmosphere.h"
#include "physics/PhysicsEngine.h"
#include "physics/forces/Drag.h"
#include "physics/forces/Lift.h"
#include "rendering/Renderer.h"

using missilesim::application::detail::formatBoolValue;
using missilesim::application::detail::formatVec3Value;
using missilesim::application::detail::parseBoolValue;
using missilesim::application::detail::parseFloatValue;
using missilesim::application::detail::parseIntValue;
using missilesim::application::detail::parseVec3Value;
using missilesim::application::detail::safeNormalize;
using missilesim::application::detail::trimWhitespace;

namespace
{
    constexpr float kGroundLaunchClearanceMeters = 1.6f;
    constexpr float kGroundLaunchProfileHeightMeters = 12.0f;
    constexpr float kGroundLaunchMinimumPitchDegrees = 8.0f;
    constexpr float kGroundLaunchMaximumPitchDegrees = 82.0f;

    // Cold-launch tuning. The ejection charge lobs the round vertically at a
    // gentle speed so the lift is readable; the booster then multiplies the
    // configured motor thrust for a short, violent climb before the sustainer.
    constexpr float kColdLaunchEjectSpeed = 26.0f;       // ejection muzzle velocity (m/s)
    constexpr float kColdLaunchEjectPitchDegrees = 82.0f; // near-vertical ejection
    constexpr float kBoostThrustMultiplier = 4.0f;        // booster vs. sustainer thrust
    constexpr float kColdLaunchIgnitionThrottle = 0.35f;  // throttle at motor light-up

    // Pitch-over: the powered round turns its aim toward the live target at a
    // capped rate, then hands off to proportional navigation the moment its
    // velocity sits comfortably inside the seeker cone (so it never commits to a
    // blind vertical climb that the seeker would later reject).
    constexpr float kColdLaunchPitchRateDegPerSec = 110.0f;
    constexpr float kColdLaunchHandoffConeDegrees = 55.0f;

    glm::vec3 normalizeOrFallback(const glm::vec3 &value, const glm::vec3 &fallback)
    {
        if (glm::length2(value) > 0.0001f)
        {
            return glm::normalize(value);
        }

        if (glm::length2(fallback) > 0.0001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 0.0f, 1.0f);
    }

    float smooth01(float value)
    {
        const float t = glm::clamp(value, 0.0f, 1.0f);
        return t * t * (3.0f - (2.0f * t));
    }
}

void Application::launchMissile()
{
    try
    {
        if (m_missileInFlight)
        {
            return;
        }

        if (!m_missile)
        {
            resetMissile();
        }

        if (!m_missile)
        {
            std::cerr << "ERROR: Missile is null in launchMissile()" << std::endl;
            return;
        }

        Target *lockedTarget = getTrackedMissileTarget();
        if (!lockedTarget)
        {
            std::cout << "Launch blocked: seeker has no target lock" << std::endl;
            return;
        }

        const glm::vec3 stagedVelocity(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]);
        const glm::vec3 cameraForward = m_renderer
                                            ? safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f))
                                            : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 launchDirection = computeMissileLaunchDirection(lockedTarget, cameraForward, stagedVelocity);
        const float groundLevel = m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f;
        glm::vec3 launchPosition = m_missile->getPosition();
        if (launchPosition.y < groundLevel + kGroundLaunchClearanceMeters)
        {
            launchPosition.y = groundLevel + kGroundLaunchClearanceMeters;
        }

        const bool groundLaunchProfile =
            (launchPosition.y - groundLevel) <= (kGroundLaunchProfileHeightMeters + kGroundLaunchClearanceMeters);

        // The ejection charge throws the round near-vertically; only once it is
        // clear of the cell does the motor light and pitch over toward the
        // target. A launch already clear of the ground skips the eject and
        // lights almost immediately.
        const glm::vec3 ejectDirection = groundLaunchProfile
                                             ? computeColdLaunchEjectDirection(launchDirection)
                                             : launchDirection;
        m_missile->setPosition(launchPosition + (ejectDirection * 0.45f));

        // Stage the round inert: no thrust, no guidance. Both are armed by the
        // launch sequence so the missile never snap-steers at t=0.
        m_missile->setGuidanceEnabled(false);
        m_missile->setThrust(m_missileThrust);
        m_missile->setThrustDirection(ejectDirection);
        m_missile->setFuel(m_missileFuel);
        m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);
        m_missile->setThrottle(0.0f);
        m_missile->setThrustEnabled(false);

        const float ejectSpeed = groundLaunchProfile
                                     ? kColdLaunchEjectSpeed
                                     : std::max(glm::length(stagedVelocity), kColdLaunchEjectSpeed);
        m_missile->setVelocity(ejectDirection * ejectSpeed);
        if (m_physicsEngine)
        {
            m_physicsEngine->addObject(m_missile.get());
        }

        m_launchSequence = MissileLaunchSequence{};
        m_launchSequence.active = true;
        m_launchSequence.restoreGuidanceEnabled = m_guidanceEnabled;
        m_launchSequence.ejectDirection = ejectDirection;
        m_launchSequence.launchDirection = launchDirection;
        m_launchSequence.aimDirection = ejectDirection;
        m_launchSequence.sustainThrust = m_missileThrust;
        if (!groundLaunchProfile)
        {
            // No ejection coast when we are already airborne: light at once.
            m_launchSequence.ignitionDelay = 0.05f;
            m_launchSequence.guidanceArmDelay = 0.55f;
        }

        // Roll a big, lingering ground cloud of ejection gas across the pad. The
        // violent ignition plume is spawned in mid-air by the launch sequence
        // once the motor lights.
        if (m_renderer && groundLaunchProfile)
        {
            const glm::vec3 padPosition(launchPosition.x, groundLevel + 0.2f, launchPosition.z);
            m_renderer->spawnLaunchGroundCloudEffect(padPosition, glm::vec3(0.0f, 1.0f, 0.0f), 1.4f);
        }

        // Log launch
        std::cout << "Cold-launch sequence initialized: eject " << ejectSpeed
                  << " m/s, ignition delay " << m_launchSequence.ignitionDelay
                  << " s, booster " << (m_missileThrust * kBoostThrustMultiplier)
                  << " N for " << m_launchSequence.boostDuration
                  << " s, fuel " << m_missileFuel << " kg, target lock retained" << std::endl;

        m_seekerCueEnabled = false;
        m_missileInFlight = true;
        m_missileFlightTime = 0.0f;
        m_closestTargetDistance = 1000000.0f;
        invalidateTrajectoryPreviewCache();
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in launchMissile: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in launchMissile" << std::endl;
    }
}

glm::vec3 Application::computeMissileLaunchDirection(Target *lockedTarget,
                                                     const glm::vec3 &cameraForward,
                                                     const glm::vec3 &stagedVelocity) const
{
    const glm::vec3 fallbackDirection = normalizeOrFallback(stagedVelocity, glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 aimDirection = normalizeOrFallback(cameraForward, fallbackDirection);
    glm::vec3 targetDirection = aimDirection;

    if (m_missile && lockedTarget)
    {
        targetDirection = normalizeOrFallback(lockedTarget->getPosition() - m_missile->getPosition(), aimDirection);
    }

    glm::vec3 launchDirection = (lockedTarget != nullptr) ? targetDirection : aimDirection;

    const float groundLevel = m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f;
    const float terrainClearance = m_missile ? (m_missile->getPosition().y - groundLevel) : kGroundLaunchProfileHeightMeters;
    if (terrainClearance <= kGroundLaunchProfileHeightMeters)
    {
        glm::vec3 flatDirection(launchDirection.x, 0.0f, launchDirection.z);
        if (glm::length2(flatDirection) <= 0.0001f)
        {
            flatDirection = glm::vec3(targetDirection.x, 0.0f, targetDirection.z);
        }
        flatDirection = normalizeOrFallback(flatDirection, glm::vec3(0.0f, 0.0f, 1.0f));

        const float currentPitch = std::asin(glm::clamp(launchDirection.y, -1.0f, 1.0f));
        const float minimumPitch = glm::radians(kGroundLaunchMinimumPitchDegrees);
        const float maximumPitch = glm::radians(kGroundLaunchMaximumPitchDegrees);
        const float launchPitch = glm::clamp(std::max(currentPitch, minimumPitch), minimumPitch, maximumPitch);

        launchDirection = normalizeOrFallback((flatDirection * std::cos(launchPitch)) +
                                                  (glm::vec3(0.0f, 1.0f, 0.0f) * std::sin(launchPitch)),
                                              glm::vec3(0.0f, 1.0f, 0.0f));
    }

    return launchDirection;
}

glm::vec3 Application::computeColdLaunchEjectDirection(const glm::vec3 &launchDirection) const
{
    // Lob the round near-vertically, leaning slightly toward the target azimuth
    // so the subsequent pitch-over reads as a deliberate turn rather than a
    // sharp kink.
    glm::vec3 flatDirection(launchDirection.x, 0.0f, launchDirection.z);
    flatDirection = normalizeOrFallback(flatDirection, glm::vec3(0.0f, 0.0f, 1.0f));

    const float ejectPitch = glm::radians(kColdLaunchEjectPitchDegrees);
    return normalizeOrFallback((flatDirection * std::cos(ejectPitch)) +
                                   (glm::vec3(0.0f, 1.0f, 0.0f) * std::sin(ejectPitch)),
                               glm::vec3(0.0f, 1.0f, 0.0f));
}

void Application::resetMissileLaunchSequence()
{
    m_launchSequence = MissileLaunchSequence{};
    if (m_missile)
    {
        // Clear any leftover booster thrust so a re-staged round uses the
        // configured sustainer value.
        m_missile->setThrust(m_missileThrust);
        m_missile->setThrottle(1.0f);
    }
}

void Application::updateMissileLaunchSequence(float deltaTime)
{
    if (!m_launchSequence.active || !m_missile || !m_missileInFlight)
    {
        return;
    }

    const float dt = (deltaTime > 0.0f && std::isfinite(deltaTime)) ? deltaTime : 0.0f;
    m_launchSequence.elapsed += dt;

    const glm::vec3 ejectDirection = normalizeOrFallback(m_launchSequence.ejectDirection, m_missile->getThrustDirection());
    const glm::vec3 launchDirection = normalizeOrFallback(m_launchSequence.launchDirection, ejectDirection);

    // Phase 1 - eject coast: the round rises on the ejection charge alone,
    // pointed along the launch (vertical) vector while the motor stays cold.
    if (!m_launchSequence.motorIgnited)
    {
        m_missile->setThrustDirection(ejectDirection);
        m_missile->setThrottle(0.0f);

        if (m_launchSequence.elapsed >= m_launchSequence.ignitionDelay)
        {
            // Phase 2 - ignition: light the booster at elevated thrust and
            // announce it with the airborne plume + roar.
            m_launchSequence.motorIgnited = true;
            m_missile->setThrust(m_launchSequence.sustainThrust * kBoostThrustMultiplier);
            m_missile->setThrustEnabled(true);
            m_missile->setThrottle(kColdLaunchIgnitionThrottle);

            if (!m_launchSequence.ignitionEffectEmitted)
            {
                m_launchSequence.ignitionEffectEmitted = true;
                if (m_renderer)
                {
                    m_renderer->spawnMissileLaunchEffect(m_missile->getPosition(),
                                                         ejectDirection,
                                                         m_missile->getVelocity(),
                                                         1.35f);
                }
                if (m_audioSystem)
                {
                    m_audioSystem->playLaunch(m_missile->getPosition(), m_missile->getVelocity());
                }
            }
        }
    }

    if (m_launchSequence.motorIgnited)
    {
        const float timeSinceIgnition = m_launchSequence.elapsed - m_launchSequence.ignitionDelay;

        // Throttle ramps quickly from the ignition kick to full boost.
        const float rampProgress = (m_launchSequence.thrustRampDuration > 0.0001f)
                                       ? (timeSinceIgnition / m_launchSequence.thrustRampDuration)
                                       : 1.0f;
        m_missile->setThrottle(glm::mix(kColdLaunchIgnitionThrottle, 1.0f, smooth01(rampProgress)));

        // Phase 3 - pitch-over: turn the aim from the vertical eject vector
        // toward the *live* target at a capped rate, then hand off to guidance
        // as soon as the velocity sits inside the seeker cone. This keeps the
        // dramatic vertical pop without committing to a blind climb the seeker
        // would later reject.
        if (!m_launchSequence.guidanceArmed)
        {
            // Aim at where the target is now, not a stale launch-time bearing.
            glm::vec3 targetAim = launchDirection;
            const Target *target = m_missile->getTargetObject();
            if (target && target->isActive())
            {
                targetAim = normalizeOrFallback(target->getPosition() - m_missile->getPosition(), launchDirection);
            }

            // Rotate the current aim toward the target by at most this step.
            const float maxStep = glm::radians(kColdLaunchPitchRateDegPerSec) * dt;
            const glm::vec3 currentAim = normalizeOrFallback(m_launchSequence.aimDirection, ejectDirection);
            const float angleToTarget = std::acos(glm::clamp(glm::dot(currentAim, targetAim), -1.0f, 1.0f));
            const float stepFraction = (angleToTarget > 1e-4f) ? glm::clamp(maxStep / angleToTarget, 0.0f, 1.0f) : 1.0f;
            const glm::vec3 nextAim = normalizeOrFallback(glm::mix(currentAim, targetAim, stepFraction), targetAim);
            m_launchSequence.aimDirection = nextAim;
            m_missile->setThrustDirection(nextAim);

            // Hand off once the flight path is comfortably within the seeker
            // cone of the line of sight (PN finishes the turn cleanly from here).
            const glm::vec3 velocity = m_missile->getVelocity();
            const float speed = glm::length(velocity);
            if (speed > 1.0f && target && target->isActive())
            {
                const glm::vec3 lineOfSight =
                    normalizeOrFallback(target->getPosition() - m_missile->getPosition(), nextAim);
                const float alignment = glm::dot(velocity / speed, lineOfSight);
                if (alignment >= std::cos(glm::radians(kColdLaunchHandoffConeDegrees)))
                {
                    m_launchSequence.guidanceArmed = true;
                    m_missile->setGuidanceEnabled(m_launchSequence.restoreGuidanceEnabled);
                }
            }
        }

        // Phase 4 - booster cutoff: drop back to the configured sustainer.
        if (!m_launchSequence.boostComplete && timeSinceIgnition >= m_launchSequence.boostDuration)
        {
            m_launchSequence.boostComplete = true;
            m_missile->setThrust(m_launchSequence.sustainThrust);
        }
    }

    // Time backstop: never leave guidance disarmed past this point.
    if (!m_launchSequence.guidanceArmed && m_launchSequence.elapsed >= m_launchSequence.guidanceArmDelay)
    {
        m_launchSequence.guidanceArmed = true;
        m_missile->setGuidanceEnabled(m_launchSequence.restoreGuidanceEnabled);
    }

    const float sequenceEnd = std::max(m_launchSequence.guidanceArmDelay,
                                       m_launchSequence.ignitionDelay + m_launchSequence.boostDuration);
    if (m_launchSequence.elapsed >= sequenceEnd)
    {
        if (!m_launchSequence.guidanceArmed)
        {
            m_launchSequence.guidanceArmed = true;
            m_missile->setGuidanceEnabled(m_launchSequence.restoreGuidanceEnabled);
        }
        if (!m_launchSequence.boostComplete)
        {
            m_launchSequence.boostComplete = true;
            m_missile->setThrust(m_launchSequence.sustainThrust);
        }
        m_missile->setThrottle(1.0f);
        m_launchSequence.active = false;
    }
}

void Application::updatePreLaunchMissileAim(Target *trackedTarget)
{
    if (!m_missile || m_missileInFlight)
    {
        return;
    }

    const glm::vec3 stagedVelocity(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]);
    const glm::vec3 fallbackAim = normalizeOrFallback(stagedVelocity, m_missile->getThrustDirection());
    const glm::vec3 cameraForward = m_renderer
                                        ? safeNormalize(m_renderer->getCameraFront(), fallbackAim)
                                        : fallbackAim;
    m_missile->setThrustDirection(computeMissileLaunchDirection(trackedTarget, cameraForward, stagedVelocity));
}

void Application::resetMissile()
{
    try
    {
        m_missileInFlight = false;
        m_missileFlightTime = 0.0f;
        m_closestTargetDistance = 1000000.0f;
        resetMissileLaunchSequence();
        m_explosions.clear();
        if (m_renderer)
        {
            m_renderer->clearEffects();
        }
        if (m_audioSystem)
        {
            m_audioSystem->stopAllEmitters();
        }
        invalidateTrajectoryPreviewCache();

        // First, if there's an existing missile, remove it from physics engine
        if (m_missile)
        {
            m_physicsEngine->removeObject(m_missile.get());
        }

        // Validate initial parameters to prevent crashes
        for (int i = 0; i < 3; i++)
        {
            // Check for NaN or inf in initial position and velocity
            if (std::isnan(m_initialPosition[i]) || std::isinf(m_initialPosition[i]))
            {
                m_initialPosition[i] = 0.0f;
            }
            if (std::isnan(m_initialVelocity[i]) || std::isinf(m_initialVelocity[i]))
            {
                m_initialVelocity[i] = 0.0f;
            }
        }

        // Ensure mass is valid
        if (m_mass <= 0.0f || std::isnan(m_mass) || std::isinf(m_mass))
        {
            m_mass = 100.0f; // Default safe value
        }

        // Ensure drag coefficient is valid
        if (m_dragCoefficient < 0.0f || std::isnan(m_dragCoefficient) || std::isinf(m_dragCoefficient))
        {
            m_dragCoefficient = 0.1f; // Default safe value
        }

        // Ensure cross-sectional area is valid
        if (m_crossSectionalArea <= 0.0f || std::isnan(m_crossSectionalArea) || std::isinf(m_crossSectionalArea))
        {
            m_crossSectionalArea = 0.1f; // Default safe value
        }

        // Ensure lift coefficient is valid
        if (m_liftCoefficient < 0.0f || std::isnan(m_liftCoefficient) || std::isinf(m_liftCoefficient))
        {
            m_liftCoefficient = 0.1f; // Default safe value
        }

        // Create missile with validated parameters
        m_missile = std::make_unique<Missile>(
            glm::vec3(m_initialPosition[0], m_initialPosition[1], m_initialPosition[2]),
            glm::vec3(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]),
            m_mass, m_dragCoefficient, m_crossSectionalArea, m_liftCoefficient);

        // Validate guidance parameters
        if (std::isnan(m_navigationGain) || std::isinf(m_navigationGain))
        {
            m_navigationGain = 4.0f;
        }
        m_navigationGain = std::clamp(m_navigationGain, 1.0f, 4.0f);

        if (m_maxSteeringForce <= 0.0f || std::isnan(m_maxSteeringForce) || std::isinf(m_maxSteeringForce))
        {
            m_maxSteeringForce = 20000.0f;
        }

        if (std::isnan(m_trackingAngle) || std::isinf(m_trackingAngle))
        {
            m_trackingAngle = 85.0f;
        }
        m_trackingAngle = std::clamp(m_trackingAngle, 5.0f, 180.0f);

        if (std::isnan(m_proximityFuseRadius) || std::isinf(m_proximityFuseRadius) || m_proximityFuseRadius < 0.0f)
        {
            m_proximityFuseRadius = 18.0f;
        }

        if (std::isnan(m_countermeasureResistance) || std::isinf(m_countermeasureResistance))
        {
            m_countermeasureResistance = 0.65f;
        }
        m_countermeasureResistance = glm::clamp(m_countermeasureResistance, 0.0f, 1.0f);

        if (std::isnan(m_terrainClearance) || std::isinf(m_terrainClearance) || m_terrainClearance < 0.0f)
        {
            m_terrainClearance = 90.0f;
        }

        if (std::isnan(m_terrainLookAheadTime) || std::isinf(m_terrainLookAheadTime) || m_terrainLookAheadTime < 0.5f)
        {
            m_terrainLookAheadTime = 6.0f;
        }

        // Set guidance parameters
        m_missile->setGuidanceEnabled(m_guidanceEnabled);
        m_missile->setNavigationGain(m_navigationGain);
        m_missile->setMaxSteeringForce(m_maxSteeringForce);
        m_missile->setTrackingAngle(m_trackingAngle);
        m_missile->setProximityFuseRadius(m_proximityFuseRadius);
        m_missile->setCountermeasureResistance(m_countermeasureResistance);
        m_missile->setTerrainAvoidanceEnabled(m_terrainAvoidanceEnabled);
        m_missile->setTerrainClearance(m_terrainClearance);
        m_missile->setTerrainLookAheadTime(m_terrainLookAheadTime);
        m_missile->setGroundReferenceAltitude(m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f);

        // Build the Mach-dependent aerodynamic profile from configuration.
        {
            const missilesim::sim::MissileAirframeConfig &airframe = m_simulationConfig.missile.airframe;
            missilesim::physics::AeroProfile profile;
            profile.referenceArea = m_crossSectionalArea;
            profile.baseDragCoefficient = m_dragCoefficient;
            profile.aspectRatio = airframe.aspectRatio;
            profile.oswaldEfficiency = airframe.oswaldEfficiency;
            profile.maxLiftCoefficient = airframe.maxLiftCoefficient;
            profile.machDragMultiplier = airframe.machDragMultiplier.empty()
                                             ? missilesim::physics::defaultSupersonicDragRiseCurve()
                                             : airframe.machDragMultiplier;
            m_missile->setAeroProfile(profile);
            m_missile->setMaxLoadFactorG(airframe.maxLoadFactorG);
        }

        // Set thrust parameters but disable thrust until launch
        m_missile->setThrust(m_missileThrust);
        m_missile->setThrottle(1.0f);
        m_missile->setThrustEnabled(false);
        m_missile->setFuel(m_missileFuel);
        m_missile->setFuelConsumptionRate(m_missileFuelConsumptionRate);
        m_missile->setNozzleExitArea(m_simulationConfig.missile.motor.nozzleExitArea);
        m_missile->setNozzleExitPressure(m_simulationConfig.missile.motor.nozzleExitPressure);

        const glm::vec3 stagedVelocity(m_initialVelocity[0], m_initialVelocity[1], m_initialVelocity[2]);
        const glm::vec3 standbyAim = normalizeOrFallback(stagedVelocity, glm::vec3(0.0f, 0.0f, 1.0f));
        m_missile->setThrustDirection(computeMissileLaunchDirection(nullptr, standbyAim, stagedVelocity));

        const float groundLevel = m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f;
        glm::vec3 stagedPosition = m_missile->getPosition();
        if (stagedPosition.y < groundLevel + kGroundLaunchClearanceMeters)
        {
            stagedPosition.y = groundLevel + kGroundLaunchClearanceMeters;
            m_missile->setPosition(stagedPosition);
        }

        // Keep the staged missile inert until launch; it is added to physics
        // by launchMissile() after the rail kick and arming sequence are set.
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in resetMissile: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in resetMissile" << std::endl;
    }
}

Target *Application::findBestTarget()
{
    // Find the first active target
    for (const auto &target : m_targets)
    {
        if (target->isActive())
        {
            return target.get();
        }
    }

    // No active targets found
    return nullptr;
}

bool Application::projectTargetToSeekerScreen(const Target *target, ImVec2 &screenPosition, float *pixelDistanceFromCenter) const
{
    if (!m_renderer || !target || !target->isActive())
    {
        return false;
    }

    const int safeHeight = std::max(m_height, 1);
    const float aspectRatio = static_cast<float>(m_width) / static_cast<float>(safeHeight);
    const float tanHalfFovY = std::tan(glm::radians(m_renderer->getCameraFOV() * 0.5f));
    if (tanHalfFovY <= 0.0f)
    {
        return false;
    }

    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    const glm::vec3 cameraForward = safeNormalize(m_renderer->getCameraFront(), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 cameraRight = safeNormalize(m_renderer->getCameraRight(), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 cameraUp = safeNormalize(m_renderer->getCameraUp(), glm::vec3(0.0f, 1.0f, 0.0f));

    const glm::vec3 aimPoint = target->getPosition() + glm::vec3(0.0f, target->getRadius() * 0.3f, 0.0f);
    const glm::vec3 toTarget = aimPoint - cameraPosition;
    const float forwardDepth = glm::dot(toTarget, cameraForward);
    if (forwardDepth <= 0.1f)
    {
        return false;
    }

    const float ndcX = glm::dot(toTarget, cameraRight) / (forwardDepth * tanHalfFovY * aspectRatio);
    const float ndcY = glm::dot(toTarget, cameraUp) / (forwardDepth * tanHalfFovY);
    if (std::abs(ndcX) > 1.0f || std::abs(ndcY) > 1.0f)
    {
        return false;
    }

    screenPosition.x = (ndcX + 1.0f) * 0.5f * m_width;
    screenPosition.y = (1.0f - ((ndcY + 1.0f) * 0.5f)) * safeHeight;

    if (pixelDistanceFromCenter != nullptr)
    {
        const float offsetX = screenPosition.x - (m_width * 0.5f);
        const float offsetY = screenPosition.y - (safeHeight * 0.5f);
        *pixelDistanceFromCenter = std::sqrt((offsetX * offsetX) + (offsetY * offsetY));
    }

    return true;
}

Target *Application::findSeekerCueTarget() const
{
    if (!m_renderer || !m_missile || m_missileInFlight || !m_seekerCueEnabled || !m_guidanceEnabled)
    {
        return nullptr;
    }

    Target *lockedTarget = getTrackedMissileTarget();
    ImVec2 lockedScreenPosition(0.0f, 0.0f);
    if (lockedTarget != nullptr && projectTargetToSeekerScreen(lockedTarget, lockedScreenPosition, nullptr))
    {
        return lockedTarget;
    }

    const glm::vec3 cameraPosition = m_renderer->getCameraPosition();
    Target *bestTarget = nullptr;
    float bestPixelDistance = std::numeric_limits<float>::max();
    float bestRange = std::numeric_limits<float>::max();

    for (const auto &target : m_targets)
    {
        if (!target || !target->isActive())
        {
            continue;
        }

        ImVec2 screenPosition(0.0f, 0.0f);
        float pixelDistance = 0.0f;
        if (!projectTargetToSeekerScreen(target.get(), screenPosition, &pixelDistance))
        {
            continue;
        }

        if (pixelDistance > m_seekerCueRadiusPixels)
        {
            continue;
        }

        const float targetRange = glm::length(target->getPosition() - cameraPosition);
        if (pixelDistance < bestPixelDistance || (std::abs(pixelDistance - bestPixelDistance) < 0.5f && targetRange < bestRange))
        {
            bestTarget = target.get();
            bestPixelDistance = pixelDistance;
            bestRange = targetRange;
        }
    }

    return bestTarget;
}

Target *Application::getTrackedMissileTarget() const
{
    if (!m_missile || !m_missile->hasTarget())
    {
        return nullptr;
    }

    Target *trackedTarget = m_missile->getTargetObject();
    if (trackedTarget == nullptr)
    {
        return nullptr;
    }

    for (const auto &target : m_targets)
    {
        if (target.get() == trackedTarget)
        {
            return target->isActive() ? target.get() : nullptr;
        }
    }

    return nullptr;
}

const char *Application::getMissileSeekerStateLabel() const
{
    if (!m_missile || !m_missile->isGuidanceEnabled())
    {
        return "Disabled";
    }

    if (!m_missileInFlight && !m_seekerCueEnabled)
    {
        return "Standby";
    }

    if (!m_missile->hasTarget())
    {
        return "Searching";
    }

    return m_missile->isTrackingDecoy() ? "Tracking flare" : "Tracking airframe";
}

const char *Application::getMissileSeekerTrackLabel() const
{
    if (!m_missile || !m_missile->isGuidanceEnabled())
    {
        return "DISABLED";
    }

    if (!m_missileInFlight && !m_seekerCueEnabled)
    {
        return "STBY";
    }

    if (!m_missile->hasTarget())
    {
        return "SEARCH";
    }

    return m_missile->isTrackingDecoy() ? "FLARE" : "AIRFRAME";
}

void Application::updatePreLaunchSeekerLock()
{
    if (!m_missile || m_missileInFlight)
    {
        return;
    }

    if (!m_seekerCueEnabled || !m_guidanceEnabled)
    {
        m_missile->clearTarget();
        updatePreLaunchMissileAim(nullptr);
        return;
    }

    Target *cueTarget = findSeekerCueTarget();
    if (cueTarget)
    {
        m_missile->setTargetObject(cueTarget);
    }
    else
    {
        m_missile->clearTarget();
    }

    updatePreLaunchMissileAim(cueTarget);
}

void Application::renderPreLaunchSeekerCue() const
{
    if (!m_seekerCueEnabled || !m_renderer || !m_missile || m_missileInFlight || ImGui::GetCurrentContext() == nullptr)
    {
        return;
    }

    ImVec2 cueCenter(m_width * 0.5f, m_height * 0.5f);
    bool hasLock = false;
    if (Target *trackedTarget = getTrackedMissileTarget())
    {
        hasLock = projectTargetToSeekerScreen(trackedTarget, cueCenter, nullptr);
    }

    const ImU32 ringColor = hasLock ? IM_COL32(255, 76, 76, 255) : IM_COL32(255, 255, 255, 240);
    ImDrawList *drawList = ImGui::GetForegroundDrawList();
    drawList->AddCircle(cueCenter, m_seekerCueRadiusPixels, ringColor, 64, 2.2f);
    drawList->AddLine(ImVec2(cueCenter.x - 7.0f, cueCenter.y), ImVec2(cueCenter.x + 7.0f, cueCenter.y), ringColor, 1.2f);
    drawList->AddLine(ImVec2(cueCenter.x, cueCenter.y - 7.0f), ImVec2(cueCenter.x, cueCenter.y + 7.0f), ringColor, 1.2f);
}

void Application::beginDetonationHold(const glm::vec3 &position)
{
    // Ignore if a hold is already running so the timer/explosion isn't restarted.
    if (m_detonationHoldActive)
    {
        return;
    }

    invalidateTrajectoryPreviewCache();

    // Spawn the explosion VFX/SFX before zeroing the missile velocity so the
    // blast inherits a sensible debris direction.
    createExplosion(position);

    m_detonationHoldActive = true;
    m_detonationHoldTimer = 0.0f;
    m_detonationHoldPosition = position;
    resetMissileLaunchSequence();

    // The missile is destroyed in the blast: stop flight management and pull it
    // out of the physics engine so it neither falls nor renders during the
    // hold (resetMissile re-adds a fresh missile afterwards).
    m_missileInFlight = false;
    if (m_missile)
    {
        m_missile->setThrustEnabled(false);
        m_missile->setGuidanceEnabled(false);
        m_missile->clearTarget();
        m_missile->setVelocity(glm::vec3(0.0f));
        if (m_physicsEngine)
        {
            m_physicsEngine->removeObject(m_missile.get());
        }
    }
}

void Application::finishDetonationHold()
{
    m_detonationHoldActive = false;
    m_detonationHoldTimer = 0.0f;

    // Respawn the target field if the engagement cleared it, then rearm.
    bool allTargetsInactive = true;
    for (const auto &target : m_targets)
    {
        if (target && target->isActive())
        {
            allTargetsInactive = false;
            break;
        }
    }
    if (allTargetsInactive && !m_targets.empty())
    {
        resetTargets();
    }

    resetMissile();
}
