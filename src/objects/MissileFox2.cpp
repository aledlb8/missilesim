#include "Missile.h"
#include "Flare.h"
#include "sim/Fox2Flight.h"

#include <algorithm>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

namespace
{
    constexpr float kInertialLeadSeconds = 1.5f;
    constexpr float kMinimumAspectSpeed = 1.0f;
    // How long the round keeps flying its last seeker track after the lock
    // drops. A sim choice, not a published memory time: long enough to ride
    // out the seeker falling behind a hard turn, short enough not to chase a
    // ghost for the rest of the flight.
    constexpr float kTrackMemorySeconds = 3.0f;
    // Heading error to the collision point over which guidance blends a
    // full-authority turn in on top of proportional navigation. Navigation
    // alone asks little while the line of sight is still barely moving, so
    // an off-boresight shot would otherwise start its turn late and gently.
    constexpr float kTurnBlendStartRad = 15.0f * missilesim::fox2::kPi / 180.0f;
    constexpr float kTurnBlendFullRad = 40.0f * missilesim::fox2::kPi / 180.0f;

    glm::vec3 normalizeOrFallback(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        if (glm::length2(vector) > 1.0e-8f)
        {
            return glm::normalize(vector);
        }
        if (glm::length2(fallback) > 1.0e-8f)
        {
            return glm::normalize(fallback);
        }
        return glm::vec3(0.0f, 0.0f, 1.0f);
    }

    float angleBetween(const glm::vec3 &a, const glm::vec3 &b)
    {
        const float cosine = glm::clamp(glm::dot(normalizeOrFallback(a, b), normalizeOrFallback(b, a)), -1.0f, 1.0f);
        return std::acos(cosine);
    }

    glm::vec3 rotateToward(const glm::vec3 &from, const glm::vec3 &to, float maxStepRadians)
    {
        const glm::vec3 source = normalizeOrFallback(from, to);
        const glm::vec3 destination = normalizeOrFallback(to, source);
        const float cosine = glm::clamp(glm::dot(source, destination), -1.0f, 1.0f);
        const float angle = std::acos(cosine);
        if (angle < 1.0e-5f || maxStepRadians >= angle)
        {
            return destination;
        }

        const glm::vec3 axis = glm::cross(source, destination);
        if (glm::length2(axis) < 1.0e-10f)
        {
            return source;
        }

        const float step = std::max(maxStepRadians, 0.0f);
        const float s = std::sin(step);
        const float c = std::cos(step);
        const glm::vec3 unitAxis = glm::normalize(axis);
        return glm::normalize((source * c) + (glm::cross(unitAxis, source) * s) + (unitAxis * glm::dot(unitAxis, source) * (1.0f - c)));
    }

    glm::vec3 clampToCone(const glm::vec3 &axis, const glm::vec3 &direction, float halfAngleRadians)
    {
        const glm::vec3 unitAxis = normalizeOrFallback(axis, direction);
        const float half = glm::clamp(halfAngleRadians, 0.0f, missilesim::fox2::kPi);
        const glm::vec3 turned = rotateToward(unitAxis, direction, half);
        return turned;
    }

    bool targetAfterburner(const Target &target)
    {
        return target.getThrottle() >= missilesim::fox2::kAiAfterburnerThrottle;
    }
}

void Missile::clearFox2()
{
    m_fox2Active = false;
    m_countPropellantInMass = false;
    m_fuelCapacity = 0.0f;
    m_axialThrustScale = 1.0f;
    m_fox2MotorStarted = false;
    m_fox2Locked = false;
    m_fox2HadLock = false;
    m_fox2GuidanceExpired = false;
}

void Missile::clearFox2Lock()
{
    m_fox2Locked = false;
}

void Missile::setBodyForward(const glm::vec3 &forward)
{
    m_bodyForward = normalizeOrFallback(forward, glm::vec3(0.0f, 0.0f, 1.0f));
    m_thrustDirection = m_bodyForward;
}

void Missile::configureFox2(const missilesim::fox2::Spec &spec)
{
    m_fox2 = spec;
    m_fox2Active = true;
    m_countPropellantInMass = spec.propellantKg > 0.0f && spec.motor == missilesim::fox2::MotorConfidence::DerivedAverage;

    const float launchMass = std::max(spec.massKg, 0.05f);
    const float propellant = m_countPropellantInMass ? std::min(spec.propellantKg, launchMass - 0.05f) : 0.0f;
    const float burn = std::max(spec.resolvedBurnS, 0.05f);
    m_dryMass = m_countPropellantInMass ? std::max(launchMass - propellant, 0.05f) : launchMass;
    m_fuelCapacity = m_countPropellantInMass ? propellant : 1.0f;
    m_fuel = m_fuelCapacity;
    m_fuelConsumptionRate = m_fuelCapacity / burn;
    synchronizeMass();

    const float area = missilesim::fox2::bodyArea(std::max(spec.diameterM, 0.01f));
    setCrossSectionalArea(area);
    setLiftCoefficient(0.0f);

    missilesim::physics::AeroProfile profile;
    profile.referenceArea = area;
    profile.baseDragCoefficient = 0.0f;
    profile.aspectRatio = missilesim::fox2::kAssumedFinAspectRatio;
    profile.oswaldEfficiency = missilesim::fox2::kAssumedOswaldEfficiency;
    profile.maxLiftCoefficient = std::max(spec.resolvedCnMax, 0.0f);
    profile.machDragMultiplier.clear();
    setAeroProfile(profile);

    // The integrator's total-acceleration clamp would clip the motor (an AIM-9B
    // is about 24:1 thrust to weight) down to the structural lateral limit.
    // Lateral g is capped inside Fox 2 guidance instead. 0 leaves thrust intact.
    setMaxLoadFactorG(0.0f);
    setThrust(std::max(spec.resolvedThrustN, 0.0f));
    setThrottle(1.0f);
    setThrustEnabled(false);
    setNozzleExitArea(missilesim::fox2::kAssumedExitAreaRatio * area);
    setNozzleExitPressure(missilesim::fox2::kSeaLevelPressure);
    setNavigationGain(4.0f);
    setGuidanceEnabled(true);
    setTerrainAvoidanceEnabled(false);
    setProximityFuseRadius(spec.proximityPublished ? std::max(spec.proximityM, 0.0f) : 0.0f);

    m_axialThrustScale = 1.0f;
    m_commandedLiftCoefficient = 0.0f;
    m_fox2FlightTime = 0.0f;
    m_fox2BurnoutTime = -1.0f;
    m_fox2InhibitLeft = 0.0f;
    m_fox2Locked = false;
    m_fox2HadLock = false;
    m_fox2GuidanceExpired = false;
    m_fox2MotorStarted = false;
    m_fox2RiseFlare = nullptr;
    m_fox2RiseIrradiance = 0.0f;
    m_fox2RiseTime = -1.0f;
    m_trackingDecoy = false;
    m_trackedFlare = nullptr;
    m_fox2MemoryAge = -1.0f;
    if (glm::length2(m_thrustDirection) > 1.0e-6f)
    {
        m_bodyForward = glm::normalize(m_thrustDirection);
    }
    m_boresight = m_bodyForward;
}

bool Missile::fox2OnTrackMemory() const
{
    return m_fox2Active && !m_fox2Locked && !m_fox2GuidanceExpired && m_fox2MemoryAge >= 0.0f &&
           m_fox2MemoryAge <= kTrackMemorySeconds;
}

void Missile::rememberFox2Track(const glm::vec3 &position, const glm::vec3 &velocity)
{
    m_fox2MemoryPosition = position;
    m_fox2MemoryVelocity = velocity;
    m_fox2MemoryAge = 0.0f;
}

void Missile::beginFox2Flight(const glm::vec3 &nose, bool irLock)
{
    setBodyForward(nose);
    // A locked seeker leaves the rail already looking at the target (it was
    // uncaged and tracking); only a caged one starts on the body axis.
    const float gimbal = glm::radians(std::max(m_fox2.gimbalDeg, 0.0f));
    m_boresight = irLock ? clampToCone(m_bodyForward, m_boresight, gimbal) : m_bodyForward;
    m_fox2MemoryAge = -1.0f;
    if (irLock && m_targetObject != nullptr && m_targetObject->isActive())
    {
        rememberFox2Track(m_targetObject->getPosition(), m_targetObject->getVelocity());
    }
    m_fox2LaunchPosition = m_position;
    m_fox2FlightTime = 0.0f;
    m_fox2BurnoutTime = -1.0f;
    m_fox2InhibitLeft = std::max(m_fox2.inhibitS, 0.0f);
    m_fox2GuidanceExpired = false;
    m_fox2MotorStarted = true;
    m_fox2Locked = irLock;
    m_fox2HadLock = irLock;
    m_axialThrustScale = 1.0f;
    m_commandedLiftCoefficient = 0.0f;
    m_trackingDecoy = false;
    m_trackedFlare = nullptr;
    m_selfDestructRequested = false;
    m_fox2RiseFlare = nullptr;
    m_fox2RiseIrradiance = 0.0f;
    m_fox2RiseTime = -1.0f;
    setThrottle(1.0f);
    setThrustEnabled(true);
    setGuidanceEnabled(true);
}

bool Missile::sampleZeroLiftDrag(float mach, float dynamicPressurePa, float &cd0) const
{
    if (!m_fox2Active || m_fox2.lengthM <= 0.0f || m_fox2.diameterM <= 0.0f)
    {
        return false;
    }

    const bool powered = m_thrustEnabled && m_fuel > 0.0f;
    cd0 = missilesim::fox2::scaledCd0(mach, dynamicPressurePa, m_fox2.lengthM, m_fox2.diameterM, powered,
                                      missilesim::fox2::kAim9bDragScale);
    return std::isfinite(cd0);
}

bool Missile::isFuzeArmed() const
{
    if (!m_fox2Active)
    {
        return true;
    }

    if (m_fox2.armDistanceM > 0.0f)
    {
        const float flown = glm::length(m_position - m_fox2LaunchPosition);
        if (flown < m_fox2.armDistanceM)
        {
            return false;
        }
    }

    if (m_fox2.armTimeS > 0.0f && m_fox2FlightTime < m_fox2.armTimeS)
    {
        return false;
    }

    if (m_fox2.armAfterBurnoutS > 0.0f)
    {
        if (m_fox2BurnoutTime < 0.0f)
        {
            return false;
        }
        if (m_fox2FlightTime < m_fox2BurnoutTime + m_fox2.armAfterBurnoutS)
        {
            return false;
        }
    }

    return true;
}

namespace
{
    struct SourceSignal
    {
        float intensity = 0.0f;
        float rangeM = 0.0f;
        float irradiance = 0.0f;
        bool visible = false;
    };

    SourceSignal measureTarget(const missilesim::fox2::Spec &spec, const glm::vec3 &missilePosition, const Target &target)
    {
        SourceSignal signal;
        const glm::vec3 offset = missilePosition - target.getPosition();
        signal.rangeM = std::max(glm::length(offset), 1.0f);
        const glm::vec3 fromTarget = offset / signal.rangeM;
        float cosPsi = -1.0f;
        if (glm::length(target.getVelocity()) >= kMinimumAspectSpeed)
        {
            cosPsi = glm::clamp(glm::dot(glm::normalize(target.getVelocity()), fromTarget), -1.0f, 1.0f);
        }

        const float aspect = missilesim::fox2::seekerIntensity(
            spec.aspect, cosPsi, spec.forwardFraction, spec.rearConeHalfAngleDeg, spec.noseGateHalfAngleDeg,
            spec.noseGateFraction, spec.afterburnerScalesTail, targetAfterburner(target));
        const float heat = std::max(target.getHeatSignature(), 0.0f);
        signal.intensity = heat * std::max(aspect, 0.0f);
        signal.irradiance = signal.intensity / (signal.rangeM * signal.rangeM);
        const bool rangeOpen = spec.tailAcquisitionM <= 0.0f ||
                               signal.rangeM <= spec.tailAcquisitionM * std::sqrt(std::max(signal.intensity, 0.0f));
        signal.visible = signal.intensity > 0.0f && rangeOpen;
        return signal;
    }
}

void Missile::updateFox2Prelaunch(const std::vector<Target *> &targets, const glm::vec3 &fighterNose, const glm::vec3 &fighterPosition)
{
    if (!m_fox2Active)
    {
        return;
    }

    m_bodyForward = normalizeOrFallback(fighterNose, m_bodyForward);
    m_boresight = m_bodyForward;
    m_thrustDirection = m_bodyForward;
    m_fox2HadLock = false;

    const bool fullSphere = m_fox2.rearHemisphereDesignation;
    const bool lockAfterLaunch = m_fox2.homing == missilesim::fox2::LaunchHoming::LockAfterLaunch;
    float coneDegrees = m_fox2.gimbalDeg;
    if (fullSphere)
    {
        coneDegrees = 180.0f;
    }
    else if (lockAfterLaunch && m_fox2.cueDeg > 0.0f)
    {
        coneDegrees = m_fox2.cueDeg;
    }

    Target *best = nullptr;
    float bestAngle = 1.0e9f;
    float bestRange = 1.0e9f;
    const glm::vec3 nose = m_bodyForward;
    for (Target *target : targets)
    {
        if (target == nullptr || !target->isActive())
        {
            continue;
        }

        const glm::vec3 offset = target->getPosition() - fighterPosition;
        const float range = glm::length(offset);
        if (range < 1.0f)
        {
            continue;
        }

        const float angle = glm::degrees(angleBetween(nose, offset));
        if (angle > coneDegrees + 0.05f)
        {
            continue;
        }

        if (angle + 0.05f < bestAngle || (std::abs(angle - bestAngle) <= 0.05f && range < bestRange))
        {
            best = target;
            bestAngle = angle;
            bestRange = range;
        }
    }

    if (best == nullptr)
    {
        clearTarget();
        m_fox2Locked = false;
        return;
    }

    const SourceSignal signal = measureTarget(m_fox2, m_position, *best);
    const float gimbalDegrees = std::max(m_fox2.gimbalDeg, 0.0f);
    const bool inGimbal = bestAngle <= gimbalDegrees + 0.05f;
    const bool infrared = signal.visible && inGimbal;
    if (lockAfterLaunch || infrared)
    {
        setTargetObject(best);
        m_fox2Locked = infrared;
        m_fox2HadLock = false;
        if (infrared)
        {
            // The uncaged head sits on the target, not on the fighter's nose.
            m_boresight = clampToCone(m_bodyForward, best->getPosition() - m_position, glm::radians(gimbalDegrees));
        }
        return;
    }

    clearTarget();
    m_fox2Locked = false;
}

void Missile::updateFox2InFlight(const std::vector<Target *> &targets, const std::vector<Flare *> &flares, float deltaTime)
{
    (void)targets;
    if (!m_fox2Active || deltaTime <= 0.0f)
    {
        return;
    }

    m_fox2FlightTime += deltaTime;
    m_fox2InhibitLeft = std::max(0.0f, m_fox2InhibitLeft - deltaTime);
    if (m_fox2MemoryAge >= 0.0f)
    {
        m_fox2MemoryAge += deltaTime;
    }
    if (m_fox2.guidanceLimitS > 0.0f && m_fox2FlightTime >= m_fox2.guidanceLimitS)
    {
        m_fox2GuidanceExpired = true;
    }
    if (m_fox2.selfDestructS > 0.0f && m_fox2FlightTime >= m_fox2.selfDestructS)
    {
        m_selfDestructRequested = true;
    }

    const float speed = glm::length(m_velocity);
    if (speed > 5.0f)
    {
        const glm::vec3 velocityDirection = m_velocity / speed;
        const float alignStep = missilesim::fox2::kBodyAlignRateRadPerS * deltaTime;
        m_bodyForward = rotateToward(m_bodyForward, velocityDirection, alignStep);
    }
    m_thrustDirection = m_bodyForward;

    if (m_fox2GuidanceExpired)
    {
        m_fox2Locked = false;
        m_trackingDecoy = false;
        m_trackedFlare = nullptr;
        m_boresight = clampToCone(m_bodyForward, m_boresight, glm::radians(std::max(m_fox2.gimbalDeg, 0.0f)));
        return;
    }

    const bool designatedAlive = m_targetObject != nullptr && m_targetObject->isActive();
    bool flareAlive = false;
    if (m_trackedFlare != nullptr)
    {
        for (const Flare *flare : flares)
        {
            if (flare == m_trackedFlare && flare != nullptr && flare->isActive())
            {
                flareAlive = true;
                break;
            }
        }
    }
    if (!flareAlive)
    {
        if (m_trackingDecoy)
        {
            m_trackingDecoy = false;
            m_trackedFlare = nullptr;
        }
    }

    // Where the head is driven: the source it tracks; after a lost lock, where
    // the remembered track says the target is now; before any lock (lock
    // after launch), the launch aircraft's designation.
    glm::vec3 aimPoint = m_position + m_bodyForward;
    if (m_trackingDecoy && flareAlive)
    {
        aimPoint = m_trackedFlare->getPosition();
    }
    else if (m_fox2Locked && designatedAlive)
    {
        aimPoint = m_targetObject->getPosition();
    }
    else if (fox2OnTrackMemory())
    {
        aimPoint = m_fox2MemoryPosition + m_fox2MemoryVelocity * m_fox2MemoryAge;
    }
    else if (designatedAlive)
    {
        aimPoint = m_targetObject->getPosition();
    }
    const float trackStep = glm::radians(std::max(m_fox2.trackRateDegPerS, 0.0f)) * deltaTime;
    m_boresight = rotateToward(m_boresight, aimPoint - m_position, trackStep);
    m_boresight = clampToCone(m_bodyForward, m_boresight, glm::radians(std::max(m_fox2.gimbalDeg, 0.0f)));

    if (!designatedAlive && !flareAlive)
    {
        m_fox2Locked = false;
        m_fox2MemoryAge = -1.0f;
        m_targetPosition = m_position + m_bodyForward;
        m_trackedSourceVelocity = glm::vec3(0.0f);
        return;
    }

    SourceSignal primary;
    if (designatedAlive)
    {
        primary = measureTarget(m_fox2, m_position, *m_targetObject);
        m_targetPosition = m_targetObject->getPosition();
        m_trackedSourceVelocity = m_targetObject->getVelocity();
    }

    const auto offBoresightDegrees = [&](const glm::vec3 &point) {
        return glm::degrees(angleBetween(m_boresight, point - m_position));
    };
    const auto bodyAngleDegrees = [&](const glm::vec3 &point) {
        return glm::degrees(angleBetween(m_bodyForward, point - m_position));
    };
    const auto geometryHolds = [&](const glm::vec3 &point) {
        if (m_fox2.ifovDeg > 0.0f)
        {
            return offBoresightDegrees(point) <= (m_fox2.ifovDeg * 0.5f);
        }
        return bodyAngleDegrees(point) <= m_fox2.gimbalDeg + 0.05f;
    };

    const Flare *keptFlare = nullptr;
    if ((primary.visible || m_trackingDecoy) && designatedAlive)
    {
        const glm::vec3 primaryDirection = normalizeOrFallback(m_targetObject->getPosition() - m_position, m_boresight);
        const float targetSpeed = glm::length(m_targetObject->getVelocity());
        float bestIrradiance = 0.0f;
        for (Flare *flare : flares)
        {
            if (flare == nullptr || !flare->isActive())
            {
                continue;
            }
            if (!primary.visible && flare != m_trackedFlare)
            {
                continue;
            }

            const glm::vec3 toFlare = flare->getPosition() - m_position;
            const float flareRange = std::max(glm::length(toFlare), 1.0f);
            const float flareIrradiance = std::max(flare->getHeatSignature(), 0.0f) / (flareRange * flareRange);
            const float separation = glm::degrees(angleBetween(primaryDirection, toFlare));
            bool rejected = false;
            if (m_fox2.irccm == missilesim::fox2::IrccmKind::Imaging)
            {
                rejected = separation > missilesim::fox2::kImagingSeparationDeg;
            }
            else if (m_fox2.irccm == missilesim::fox2::IrccmKind::Kinematic)
            {
                const float gate = std::max(m_fox2.ifovDeg, missilesim::fox2::kKinematicSeparationDeg);
                const bool slow = targetSpeed > 30.0f &&
                                  glm::length(flare->getVelocity()) < missilesim::fox2::kKinematicFlareSpeedFraction * targetSpeed;
                rejected = slow && separation > gate;
            }
            else if (m_fox2.irccm == missilesim::fox2::IrccmKind::Rise && m_fox2RiseFlare == flare && m_fox2RiseTime >= 0.0f)
            {
                // One stored sample. A different flare has no history yet, so the
                // first look cannot trip the rise test, and it must not erase the
                // sample of the flare already being watched.
                const float age = m_fox2FlightTime - m_fox2RiseTime;
                if (age <= missilesim::fox2::kRiseWindowIllustrationS && m_fox2RiseIrradiance > 0.0f &&
                    flareIrradiance >= missilesim::fox2::kRiseRatioIllustration * m_fox2RiseIrradiance)
                {
                    rejected = true;
                }
            }

            if (rejected || !geometryHolds(flare->getPosition()))
            {
                continue;
            }

            const bool seduced = primary.irradiance <= 0.0f ||
                                 flareIrradiance >= missilesim::fox2::kUnhardenedSeductionRatio * primary.irradiance;
            if (!seduced)
            {
                continue;
            }
            if (flareIrradiance > bestIrradiance)
            {
                bestIrradiance = flareIrradiance;
                keptFlare = flare;
            }
        }
    }

    if (keptFlare != nullptr)
    {
        m_trackedFlare = keptFlare;
        m_trackingDecoy = true;
        m_targetPosition = keptFlare->getPosition();
        m_trackedSourceVelocity = keptFlare->getVelocity();
        m_fox2Locked = true;
        m_fox2HadLock = true;
        m_hasTarget = true;
        rememberFox2Track(m_targetPosition, m_trackedSourceVelocity);
        if (m_fox2.irccm == missilesim::fox2::IrccmKind::Rise && keptFlare != m_fox2RiseFlare)
        {
            m_fox2RiseFlare = keptFlare;
            m_fox2RiseIrradiance = std::max(keptFlare->getHeatSignature(), 0.0f) /
                                   std::max(glm::length2(keptFlare->getPosition() - m_position), 1.0f);
            m_fox2RiseTime = m_fox2FlightTime;
        }
        else if (m_fox2.irccm == missilesim::fox2::IrccmKind::Rise && m_fox2RiseFlare == keptFlare &&
                 m_fox2FlightTime - m_fox2RiseTime >= missilesim::fox2::kRiseWindowIllustrationS)
        {
            m_fox2RiseIrradiance = std::max(keptFlare->getHeatSignature(), 0.0f) /
                                   std::max(glm::length2(keptFlare->getPosition() - m_position), 1.0f);
            m_fox2RiseTime = m_fox2FlightTime;
        }
        return;
    }

    m_trackedFlare = nullptr;
    m_trackingDecoy = false;
    if (designatedAlive && primary.visible && geometryHolds(m_targetObject->getPosition()))
    {
        m_fox2Locked = true;
        m_fox2HadLock = true;
        m_hasTarget = true;
        m_targetPosition = m_targetObject->getPosition();
        m_trackedSourceVelocity = m_targetObject->getVelocity();
        rememberFox2Track(m_targetPosition, m_trackedSourceVelocity);
        return;
    }

    m_fox2Locked = false;
    if (designatedAlive)
    {
        m_hasTarget = true;
        m_targetPosition = m_targetObject->getPosition();
        m_trackedSourceVelocity = m_targetObject->getVelocity();
    }
}

void Missile::applyFox2Guidance(float deltaTime, float airDensity)
{
    m_commandedLiftCoefficient = 0.0f;
    m_axialThrustScale = 1.0f;
    m_thrustDirection = m_bodyForward;
    if (!m_fox2Active || deltaTime <= 0.0f)
    {
        return;
    }
    if (m_fox2GuidanceExpired || !m_guidanceEnabled || m_fox2InhibitLeft > 0.0f)
    {
        return;
    }

    const bool tracking = m_fox2Locked && (m_trackingDecoy || (m_targetObject != nullptr && m_targetObject->isActive()));
    const bool memory = !tracking && fox2OnTrackMemory();
    const bool pursuit = !tracking && !memory && !m_fox2HadLock && m_targetObject != nullptr && m_targetObject->isActive();
    if (!tracking && !memory && !pursuit)
    {
        return;
    }

    glm::vec3 aimPosition = m_targetPosition;
    glm::vec3 aimVelocity = m_trackedSourceVelocity;
    if (memory)
    {
        aimPosition = m_fox2MemoryPosition + m_fox2MemoryVelocity * m_fox2MemoryAge;
        aimVelocity = m_fox2MemoryVelocity;
    }
    else if (pursuit)
    {
        aimPosition = m_targetObject->getPosition() + (m_targetObject->getVelocity() * kInertialLeadSeconds);
        aimVelocity = m_targetObject->getVelocity();
    }
    else if (!m_trackingDecoy && m_targetObject != nullptr && m_targetObject->isActive())
    {
        aimPosition = m_targetObject->getPosition();
        aimVelocity = m_targetObject->getVelocity();
    }

    const float speed = glm::length(m_velocity);
    if (speed < 0.5f)
    {
        return;
    }

    const glm::vec3 velocityDirection = m_velocity / speed;
    const glm::vec3 offset = aimPosition - m_position;
    const float range = glm::length(offset);
    if (range < 0.5f)
    {
        return;
    }
    const glm::vec3 lineOfSight = offset / range;

    const float dynamicPressure = 0.5f * std::max(airDensity, 0.0f) * speed * speed;
    const float area = std::max(m_crossSectionalArea, 1.0e-6f);
    const float mass = std::max(m_mass, 0.05f);
    const float aeroAcceleration = (dynamicPressure * std::max(m_fox2.resolvedCnMax, 0.0f) * area) / mass;
    const bool burning = m_thrustEnabled && m_fuel > 0.0f && m_thrust > 0.0f;
    const float structuralG = burning ? m_fox2.resolvedBurnG : m_fox2.resolvedCoastG;
    const float structuralAcceleration = std::max(structuralG, 0.0f) * missilesim::fox2::kGravity;

    // Thrust vectoring: the vanes deflect the jet only a few degrees, but that
    // moment swings the whole body (and motor) off the flight path, which at
    // launch dynamic pressure nothing aerodynamic resists. The lateral share of
    // thrust is T sin(body angle), up to all of it, paid for in axial thrust
    // below. (The old T sin(vane angle) took the vane's own side force for the
    // whole effect and left TVC rounds with a few g at the rail.)
    float vaneAcceleration = 0.0f;
    if (burning && m_fox2.hasTvc)
    {
        vaneAcceleration = m_thrust / mass;
    }

    float available = aeroAcceleration;
    if (burning && m_fox2.hasTvc)
    {
        available = aeroAcceleration + vaneAcceleration;
    }
    if (structuralAcceleration > 0.0f)
    {
        available = std::min(available, structuralAcceleration);
    }

    glm::vec3 commanded = glm::vec3(0.0f);
    if (tracking || memory)
    {
        // Pure proportional navigation, a = N Vm (omega x v_m). Scaling by the
        // missile's own speed instead of the closing speed keeps full gain in
        // beam and lag shots, where closing speed is small or negative and
        // the closing-speed form's command lies almost along the velocity
        // (and was projected away, so the round barely turned).
        const glm::vec3 relativeVelocity = aimVelocity - m_velocity;
        const float rangeSquared = std::max(glm::dot(offset, offset), 1.0e-4f);
        const glm::vec3 losRate = glm::cross(offset, relativeVelocity) / rangeSquared;
        const float navigation = glm::clamp(m_navigationGain, 3.0f, 5.0f);
        commanded = navigation * speed * glm::cross(losRate, velocityDirection);

        // Off-boresight: while the flight path is far off the collision
        // course, turn at full authority toward it, fading to pure navigation
        // as the heading error closes.
        glm::vec3 collision = lineOfSight;
        const float a = glm::dot(aimVelocity, aimVelocity) - speed * speed;
        const float b = 2.0f * glm::dot(offset, aimVelocity);
        const float c = glm::dot(offset, offset);
        float interceptTime = -1.0f;
        if (std::abs(a) < 1.0e-3f)
        {
            interceptTime = b < 0.0f ? -c / b : -1.0f;
        }
        else
        {
            const float discriminant = b * b - 4.0f * a * c;
            if (discriminant >= 0.0f)
            {
                const float root = std::sqrt(discriminant);
                const float t1 = (-b - root) / (2.0f * a);
                const float t2 = (-b + root) / (2.0f * a);
                const float low = std::min(t1, t2);
                const float high = std::max(t1, t2);
                interceptTime = low > 0.0f ? low : high;
            }
        }
        if (interceptTime > 0.0f)
        {
            collision = normalizeOrFallback(offset + aimVelocity * interceptTime, lineOfSight);
        }
        const float headingError = angleBetween(velocityDirection, collision);
        const float turnBlend = glm::smoothstep(kTurnBlendStartRad, kTurnBlendFullRad, headingError);
        if (turnBlend > 0.0f)
        {
            const glm::vec3 turnDirection = collision - velocityDirection * glm::dot(collision, velocityDirection);
            if (glm::length2(turnDirection) > 1.0e-8f)
            {
                commanded += glm::normalize(turnDirection) * (available * turnBlend);
            }
        }
    }
    else
    {
        const glm::vec3 desired = lineOfSight;
        const glm::vec3 lateralDirection = desired - velocityDirection * glm::dot(desired, velocityDirection);
        commanded = lateralDirection * (speed / kInertialLeadSeconds);
    }

    commanded -= velocityDirection * glm::dot(commanded, velocityDirection);

    float magnitude = glm::length(commanded);
    if (magnitude > available && magnitude > 1.0e-4f)
    {
        commanded *= available / magnitude;
        magnitude = available;
    }

    const float aeroUsed = std::min(magnitude, aeroAcceleration);
    const float tvcUsed = std::max(0.0f, magnitude - aeroUsed);
    if (burning && m_fox2.hasTvc && m_thrust > 1.0f && tvcUsed > 0.0f)
    {
        const float sine = glm::clamp((tvcUsed * mass) / m_thrust, 0.0f, 1.0f);
        m_axialThrustScale = std::sqrt(std::max(0.0f, 1.0f - sine * sine));
    }

    if (magnitude > 1.0e-4f)
    {
        applyForce(commanded * mass);
    }

    const float liftDenominator = std::max(dynamicPressure * area, 1.0e-4f);
    m_commandedLiftCoefficient = glm::clamp((mass * aeroUsed) / liftDenominator, 0.0f, std::max(m_fox2.resolvedCnMax, 0.0f));
}
