#include "Missile.h"
#include "Flare.h"
#include "Target.h" // Include Target class
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace
{
    constexpr float kDecoyAngularSeparationMinDegrees = 0.15f;
    constexpr float kDecoyAngularSeparationMaxDegrees = 1.6f;
    constexpr float kMinimumVelocityCoherence = 0.05f;

    glm::vec3 normalizeOrFallback(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        if (glm::length2(vector) > 0.0001f)
        {
            return glm::normalize(vector);
        }

        if (glm::length2(fallback) > 0.0001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 0.0f, 1.0f);
    }

    float computeAngleDegreesFromDot(float dotValue)
    {
        return glm::degrees(std::acos(glm::clamp(dotValue, -1.0f, 1.0f)));
    }
} // namespace

Missile::Missile(const glm::vec3 &position, const glm::vec3 &velocity,
                 float mass, float dragCoefficient, float crossSectionalArea, float liftCoefficient)
    : PhysicsObject(position, velocity, std::max(mass, 0.01f)),
      m_dragCoefficient(dragCoefficient),
      m_crossSectionalArea(crossSectionalArea),
      m_liftCoefficient(liftCoefficient),
      m_dryMass(std::max(mass, 0.01f))
{
    // Seed the Mach-dependent aerodynamic profile from the scalar airframe
    // inputs plus a default transonic drag-rise signature. Config (A3) may
    // later replace the curve and lifting-surface parameters wholesale.
    m_aeroProfile.referenceArea = m_crossSectionalArea;
    m_aeroProfile.baseDragCoefficient = m_dragCoefficient;
    // Coefficients are referenced to the small body cross-section, so a slender
    // missile's max normal-force coefficient is large; the induced-drag term
    // depends on span (S*AR), keeping turn energy bleed realistic.
    m_aeroProfile.aspectRatio = 18.0f;
    m_aeroProfile.oswaldEfficiency = 0.8f;   // span efficiency factor
    m_aeroProfile.maxLiftCoefficient = 20.0f; // normal-force ceiling (control authority)
    m_aeroProfile.machDragMultiplier = missilesim::physics::defaultSupersonicDragRiseCurve();

    setMaxLoadFactorG(40.0f); // structural limit; config may override

    synchronizeMass();
}

void Missile::setMass(float mass)
{
    m_dryMass = std::max(mass, 0.01f);
    synchronizeMass();
}

void Missile::setTrackingAngle(float angleDegrees)
{
    if (std::isnan(angleDegrees) || std::isinf(angleDegrees))
    {
        return;
    }

    m_trackingAngleDegrees = glm::clamp(angleDegrees, 5.0f, 180.0f);
}

void Missile::setProximityFuseRadius(float radiusMeters)
{
    if (std::isnan(radiusMeters) || std::isinf(radiusMeters))
    {
        return;
    }

    m_proximityFuseRadius = std::max(radiusMeters, 0.0f);
}

void Missile::setCountermeasureResistance(float resistance)
{
    if (std::isnan(resistance) || std::isinf(resistance))
    {
        return;
    }

    m_countermeasureResistance = glm::clamp(resistance, 0.0f, 1.0f);
}

void Missile::setFuel(float kg)
{
    m_fuel = std::max(kg, 0.0f);
    if (m_fuel <= 0.0f)
    {
        m_thrustEnabled = false;
    }
    synchronizeMass();
}

void Missile::synchronizeMass()
{
    m_mass = std::max(m_dryMass + m_fuel, 0.01f);
}

void Missile::setTarget(const glm::vec3 &targetPosition)
{
    m_hasTarget = true;
    m_targetPosition = targetPosition;
    m_trackedSourceVelocity = glm::vec3(0.0f);
    m_targetObject = nullptr;
    m_trackedFlare = nullptr;
    m_trackingDecoy = false;
    m_selfDestructRequested = false;
}

void Missile::setTargetObject(Target *target)
{
    m_targetObject = target;
    m_hasTarget = (target != nullptr);

    if (m_hasTarget)
    {
        m_targetPosition = target->getPosition();
        m_trackedSourceVelocity = target->getVelocity();
        m_trackedFlare = nullptr;
        m_trackingDecoy = false;
        m_selfDestructRequested = false;
    }
    else
    {
        m_trackedSourceVelocity = glm::vec3(0.0f);
        m_trackedFlare = nullptr;
        m_trackingDecoy = false;
        m_selfDestructRequested = false;
    }
}

void Missile::clearTarget()
{
    m_hasTarget = false;
    m_targetObject = nullptr;
    m_trackedFlare = nullptr;
    m_trackingDecoy = false;
    m_trackedSourceVelocity = glm::vec3(0.0f);
    m_selfDestructRequested = false;
}

void Missile::updateHeatSeeker(const std::vector<Target *> &targets, const std::vector<Flare *> &flares, float deltaTime)
{
    if (!m_guidanceEnabled || !m_thrustEnabled || deltaTime <= 0.0f)
    {
        return;
    }

    const glm::vec3 seekerForward = (!m_thrustEnabled || glm::length2(m_velocity) < 0.0001f)
                                        ? normalizeOrFallback(m_thrustDirection, m_velocity)
                                        : normalizeOrFallback(m_velocity, m_thrustDirection);
    const float seekerCosLimit = std::cos(glm::radians(m_trackingAngleDegrees));
    const float resistanceBlend = glm::clamp(m_countermeasureResistance, 0.0f, 1.0f);

    Target *bestTarget = nullptr;
    float bestTargetScore = -1.0f;
    for (Target *target : targets)
    {
        if (target == nullptr || !target->isActive())
        {
            continue;
        }

        const glm::vec3 targetPosition = target->getPosition();
        const glm::vec3 targetVelocity = target->getVelocity();
        const glm::vec3 relativePosition = targetPosition - m_position;
        const float distanceSq = std::max(glm::dot(relativePosition, relativePosition), 1.0f);
        const glm::vec3 directionToTarget = normalizeOrFallback(relativePosition, seekerForward);
        const float alignment = glm::clamp(glm::dot(seekerForward, directionToTarget), -1.0f, 1.0f);
        if (alignment < seekerCosLimit)
        {
            continue;
        }

        float targetHeat = std::max(target->getHeatSignature(), 0.0f);
        const float targetSpeed = glm::length(targetVelocity);
        if (targetSpeed > 0.5f)
        {
            const glm::vec3 exhaustDirection = -glm::normalize(targetVelocity);
            const glm::vec3 missileFromTarget = normalizeOrFallback(m_position - targetPosition, exhaustDirection);
            const float rearAspect = glm::clamp(glm::dot(missileFromTarget, exhaustDirection), 0.0f, 1.0f);
            targetHeat *= glm::mix(0.55f, 1.0f, rearAspect);
        }

        const float angleWeight = glm::smoothstep(seekerCosLimit, 1.0f, alignment);
        const bool retainingPrimaryTrack = !m_trackingDecoy && target == m_targetObject;
        const float primaryTrackBias = retainingPrimaryTrack
                                           ? (1.0f + m_lockRetentionBias + (resistanceBlend * 1.35f))
                                           : 1.0f;
        const float score = (targetHeat / distanceSq) * angleWeight * primaryTrackBias;
        if (score > bestTargetScore)
        {
            bestTargetScore = score;
            bestTarget = target;
        }
    }

    if (bestTarget == nullptr)
    {
        clearTarget();
        return;
    }

    const glm::vec3 primaryTargetPosition = bestTarget->getPosition();
    const glm::vec3 primaryTargetVelocity = bestTarget->getVelocity();
    const glm::vec3 primaryRelativePosition = primaryTargetPosition - m_position;
    const glm::vec3 primaryDirection = normalizeOrFallback(primaryRelativePosition, seekerForward);
    float bestScore = bestTargetScore;
    float primaryScore = bestTargetScore;
    glm::vec3 bestPosition = primaryTargetPosition;
    glm::vec3 bestVelocity = primaryTargetVelocity;
    const Flare *bestFlare = nullptr;
    bool bestIsDecoy = false;

    m_targetObject = bestTarget;
    m_hasTarget = true;
    m_selfDestructRequested = false;

    for (Flare *flare : flares)
    {
        if (flare == nullptr || !flare->isActive())
        {
            continue;
        }

        const glm::vec3 relativePosition = flare->getPosition() - m_position;
        const float distanceSq = std::max(glm::dot(relativePosition, relativePosition), 1.0f);
        const glm::vec3 directionToFlare = normalizeOrFallback(relativePosition, seekerForward);
        const float alignment = glm::clamp(glm::dot(seekerForward, directionToFlare), -1.0f, 1.0f);
        if (alignment < seekerCosLimit)
        {
            continue;
        }

        float score = flare->getHeatSignature() / distanceSq;
        score *= glm::smoothstep(seekerCosLimit, 1.0f, alignment);

        const float primaryFlareAlignment = glm::clamp(glm::dot(primaryDirection, directionToFlare), 0.0f, 1.0f);
        const float directionalCoherence = glm::mix(0.1f, 1.0f, primaryFlareAlignment);
        const float angularSeparationDegrees = computeAngleDegreesFromDot(primaryFlareAlignment);
        const float separationCoherence = glm::smoothstep(
            kDecoyAngularSeparationMinDegrees,
            kDecoyAngularSeparationMaxDegrees,
            angularSeparationDegrees);
        const float targetSpeed = std::max(glm::length(primaryTargetVelocity), 10.0f);
        const float velocityMismatch = glm::length(flare->getVelocity() - primaryTargetVelocity) / targetSpeed;
        const float velocityCoherence = glm::clamp(
            1.0f - glm::smoothstep(0.08f, 0.45f, velocityMismatch),
            kMinimumVelocityCoherence,
            1.0f);
        const float decoyCoherence = directionalCoherence * separationCoherence * velocityCoherence;
        const float irccmBlend = 0.45f + (0.55f * resistanceBlend);
        score *= glm::mix(1.0f, decoyCoherence, irccmBlend);

        if (!m_trackingDecoy && primaryScore > 0.0f)
        {
            const float switchMargin = glm::mix(1.35f, 2.5f, resistanceBlend);
            if (score <= (primaryScore * switchMargin))
            {
                continue;
            }
        }

        if (m_trackingDecoy && m_trackedFlare == flare)
        {
            score *= 1.0f + m_lockRetentionBias;
        }

        if (score > bestScore)
        {
            bestScore = score;
            bestPosition = flare->getPosition();
            bestVelocity = flare->getVelocity();
            bestFlare = flare;
            bestIsDecoy = true;
        }
    }

    m_targetPosition = bestPosition;
    m_trackedSourceVelocity = bestVelocity;
    m_trackedFlare = bestFlare;
    m_trackingDecoy = bestIsDecoy;
}

bool Missile::consumeSelfDestructRequest()
{
    const bool requested = m_selfDestructRequested;
    m_selfDestructRequested = false;
    return requested;
}

void Missile::applyGuidance(float deltaTime, float airDensity)
{
    // Only apply guidance if enabled and target exists
    if (!m_guidanceEnabled || !m_hasTarget)
    {
        m_commandedLiftCoefficient = 0.0f;
        return;
    }

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        return;
    }

    try
    {
        glm::vec3 targetVelocity = m_trackedSourceVelocity;

        if (m_targetObject != nullptr && m_targetObject->isActive())
        {
            if (!m_trackingDecoy)
            {
                m_targetPosition = m_targetObject->getPosition();
                targetVelocity = m_targetObject->getVelocity();
            }
        }
        else if (m_targetObject != nullptr)
        {
            clearTarget();
            return;
        }

        // Safety check for invalid target position (NaN or infinity)
        if (std::isnan(m_targetPosition.x) || std::isnan(m_targetPosition.y) || std::isnan(m_targetPosition.z) ||
            std::isinf(m_targetPosition.x) || std::isinf(m_targetPosition.y) || std::isinf(m_targetPosition.z))
        {
            std::cerr << "Error: Invalid target position detected (NaN or infinity)" << std::endl;
            m_hasTarget = false;
            m_guidanceEnabled = false;
            return;
        }

        // Check if our own position is valid
        if (std::isnan(m_position.x) || std::isnan(m_position.y) || std::isnan(m_position.z) ||
            std::isinf(m_position.x) || std::isinf(m_position.y) || std::isinf(m_position.z))
        {
            std::cerr << "Error: Invalid missile position detected (NaN or infinity)" << std::endl;
            m_hasTarget = false;
            m_guidanceEnabled = false;
            return;
        }

        const glm::vec3 currentDirection = normalizeOrFallback(m_velocity, m_thrustDirection);
        const float speed = glm::length(m_velocity);
        if (speed < 0.1f)
        {
            if (m_thrustEnabled)
            {
                const glm::vec3 boostDirection = normalizeOrFallback(m_targetPosition - m_position, currentDirection);
                m_thrustDirection = boostDirection;
            }
            return;
        }

        const glm::vec3 relativePosition = m_targetPosition - m_position;
        const float distanceToTarget = glm::length(relativePosition);
        if (distanceToTarget < 0.5f)
        {
            return;
        }

        const glm::vec3 lineOfSight = relativePosition / distanceToTarget;

        // Self-destruct if the target leaves the seeker field of view.
        const float seekerAlignment = glm::clamp(glm::dot(currentDirection, lineOfSight), -1.0f, 1.0f);
        const float seekerAngleDegrees = glm::degrees(std::acos(seekerAlignment));
        if (seekerAngleDegrees > m_trackingAngleDegrees)
        {
            m_selfDestructRequested = true;
            m_commandedLiftCoefficient = 0.0f;
            return;
        }

        // --- True proportional navigation --------------------------------
        //   LOS angular velocity:  omega = (R x Vr) / (R . R)
        //   Closing velocity:      Vc    = -(Vr . uLOS)
        //   Command:               a_n   = N * Vc * (omega x uLOS)
        // The command is normal to the line of sight; on a collision course
        // (zero LOS rate) it is zero, which is the PN ideal.
        const glm::vec3 relativeVelocity = targetVelocity - m_velocity;
        const float rangeSquared = std::max(glm::dot(relativePosition, relativePosition), 1e-4f);
        const glm::vec3 losRate = glm::cross(relativePosition, relativeVelocity) / rangeSquared;
        const float closingSpeed = -glm::dot(relativeVelocity, lineOfSight);
        const float navigationConstant = glm::clamp(m_navigationGain, 1.0f, 6.0f);
        // A floor on the effective closing speed keeps authority just after
        // launch when the geometry can momentarily be opening.
        const float effectiveClosingSpeed = std::max(std::abs(closingSpeed), speed * 0.1f);
        glm::vec3 commandedAcceleration =
            navigationConstant * effectiveClosingSpeed * glm::cross(losRate, lineOfSight);

        // Lift does no work along the flight path: keep the command normal to
        // the current velocity.
        commandedAcceleration -= currentDirection * glm::dot(commandedAcceleration, currentDirection);

        // --- Terrain avoidance, as an added upward acceleration demand ----
        if (m_terrainAvoidanceEnabled)
        {
            const float currentClearance = m_position.y - m_groundReferenceAltitude;
            const float predictedClearance = currentClearance + (m_velocity.y * m_terrainLookAheadTime);
            if (predictedClearance < m_terrainClearance)
            {
                const float deficit = m_terrainClearance - predictedClearance;
                const float lookAhead = std::max(m_terrainLookAheadTime, 0.5f);
                // Acceleration needed to erase the clearance deficit within the
                // look-ahead window (s = 0.5*a*t^2 -> a = 2s/t^2).
                commandedAcceleration.y += (2.0f * deficit) / (lookAhead * lookAhead);
            }
        }

        // --- Control authority --------------------------------------------
        // The lateral acceleration the airframe can actually produce is bounded
        // by the lift available at the local dynamic pressure and by the
        // structural load-factor limit. At low q (high altitude / low speed)
        // the missile becomes sluggish - real interceptor behaviour.
        const float dynamicPressure = 0.5f * std::max(airDensity, 0.0f) * speed * speed;
        const float referenceArea =
            (m_aeroProfile.referenceArea > 0.0f) ? m_aeroProfile.referenceArea : m_crossSectionalArea;
        const float maxLiftCoefficient = std::max(m_aeroProfile.maxLiftCoefficient, 0.0f);
        const float aerodynamicLimit =
            (dynamicPressure * maxLiftCoefficient * referenceArea) / std::max(m_mass, 0.01f);
        const float structuralLimit =
            (m_maxLoadFactorG > 0.0f) ? (m_maxLoadFactorG * 9.80665f) : aerodynamicLimit;
        const float availableLateralAcceleration = std::min(aerodynamicLimit, structuralLimit);

        float commandedMagnitude = glm::length(commandedAcceleration);
        if (commandedMagnitude > availableLateralAcceleration && commandedMagnitude > 1e-4f)
        {
            commandedAcceleration *= (availableLateralAcceleration / commandedMagnitude);
            commandedMagnitude = availableLateralAcceleration;
        }

        applyForce(commandedAcceleration * m_mass);

        // Record the operating lift coefficient (Cl = L / (q*S), L = m*a_n) so
        // the drag model charges the correct induced drag for this maneuver.
        const float liftDenominator = std::max(dynamicPressure * referenceArea, 1e-4f);
        m_commandedLiftCoefficient =
            glm::clamp((m_mass * commandedMagnitude) / liftDenominator, 0.0f, maxLiftCoefficient);

        // Point the airframe slightly into the maneuver (a small angle of
        // attack) so thrust acts along the body axis.
        if (m_thrustEnabled)
        {
            m_thrustDirection =
                normalizeOrFallback((currentDirection * speed) + (commandedAcceleration * deltaTime), currentDirection);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in missile guidance: " << e.what() << std::endl;
        m_guidanceEnabled = false;
    }
    catch (...)
    {
        std::cerr << "Unknown error in missile guidance" << std::endl;
        m_guidanceEnabled = false;
    }
}

bool Missile::applyThrust(float deltaTime)
{
    // Check if thrust is enabled and we have fuel
    if (!m_thrustEnabled || m_fuel <= 0.0f || m_thrust <= 0.0f)
    {
        if (m_fuel <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
            synchronizeMass();
        }
        return false;
    }

    try
    {
        if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime) || m_fuelConsumptionRate <= 0.0f)
        {
            return false;
        }

        // Safety checks for valid thrust direction
        if (glm::length2(m_thrustDirection) < 0.001f ||
            std::isnan(m_thrustDirection.x) || std::isnan(m_thrustDirection.y) || std::isnan(m_thrustDirection.z) ||
            std::isinf(m_thrustDirection.x) || std::isinf(m_thrustDirection.y) || std::isinf(m_thrustDirection.z))
        {
            // Default to current velocity direction, or forward if velocity is too small
            if (glm::length2(m_velocity) > 0.001f)
            {
                m_thrustDirection = glm::normalize(m_velocity);
            }
            else
            {
                m_thrustDirection = glm::vec3(0.0f, 0.0f, 1.0f); // Default forward direction
            }
        }

        // Calculate fuel consumption for this step
        const float requestedFuel = m_fuelConsumptionRate * deltaTime;
        if (requestedFuel <= 0.0f)
        {
            return false;
        }

        // Limit consumption to available fuel
        const float fuelConsumed = std::min(requestedFuel, m_fuel);
        const float throttleFraction = glm::clamp(fuelConsumed / requestedFuel, 0.0f, 1.0f);
        if (throttleFraction <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
            synchronizeMass();
            return false;
        }

        // Physical thrust = momentum thrust (mdot * Ve, with Ve = thrust/mdot so
        // the configured value is the sea-level full-burn thrust) plus a nozzle
        // back-pressure term. As ambient pressure falls with altitude the
        // back-pressure term grows, so the motor produces more thrust up high -
        // approaching its vacuum thrust. Both terms require propellant flow, so
        // they scale with the throttle (fuel-availability) fraction.
        const float backPressureThrust = (m_nozzleExitPressure - m_ambientPressure) * m_nozzleExitArea;
        const float thrustMagnitude = std::max(throttleFraction * (m_thrust + backPressureThrust), 0.0f);

        // Apply thrust force in the thrust direction
        glm::vec3 thrustForce = m_thrustDirection * thrustMagnitude;
        applyForce(thrustForce);

        // Update remaining fuel and wet mass after the burn.
        m_fuel -= fuelConsumed;
        if (m_fuel <= 0.0f)
        {
            m_fuel = 0.0f;
            m_thrustEnabled = false;
        }
        synchronizeMass();

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error applying thrust: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown error applying thrust" << std::endl;
        return false;
    }
}

void Missile::update(float deltaTime)
{
    // applyGuidance(deltaTime);

    // Apply thrust if enabled
    applyThrust(deltaTime);

    // Continue with normal physics update
    PhysicsObject::update(deltaTime);

    // When guidance is inactive, keep thrust aligned with the current flight path.
    if ((!m_guidanceEnabled || !m_hasTarget || !m_thrustEnabled) && glm::length2(m_velocity) > 0.001f)
    {
        m_thrustDirection = glm::normalize(m_velocity);
    }
}
