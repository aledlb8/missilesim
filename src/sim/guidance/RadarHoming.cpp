#include "sim/guidance/RadarHoming.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace missilesim::sim
{
    using Phase = RadarHomingState::Phase;

    namespace
    {
        constexpr float kPi = 3.14159265f;
        constexpr float kCoincidentM = 1.0e-3f;
        constexpr float kRangeSquaredFloor = 1.0e-4f;
        constexpr float kVelocityFloor = 1.0e-3f;

        glm::vec3 unitOr(const glm::vec3 &vector, const glm::vec3 &fallback)
        {
            const float length = glm::length(vector);
            return length > 1.0e-6f ? vector / length : fallback;
        }

        // position + velocity * (now - packet.time). A negative age still uses
        // that expression; the caller owns the packet clock.
        glm::vec3 coastedPosition(const SupportPacket &packet, double time)
        {
            const float age = static_cast<float>(time - packet.time);
            return packet.position + packet.velocity * age;
        }

        bool supportIsFresh(const RadarHomingState &state, const RadarHomingSpec &spec, double time)
        {
            if (!state.haveSupport)
            {
                return false;
            }
            const double period = static_cast<double>(spec.datalinkPeriodS > 0.0f ? spec.datalinkPeriodS : 0.0f);
            return time - state.lastSupport.time <= period;
        }

        // Textbook proportional navigation: a = N * Vc * (omega × uLOS),
        // omega = (R × Vr) / R², Vc = -(Vr · uLOS). N is clamped to 3..5.
        // The part along the missile velocity is removed when the velocity
        // has a direction, then the remainder is clamped to maxAcceleration.
        glm::vec3 proportionalCommand(const glm::vec3 &missilePosition, const glm::vec3 &missileVelocity,
                                      const glm::vec3 &aimPosition, const glm::vec3 &aimVelocity, float navigationGain,
                                      float maxAcceleration)
        {
            const glm::vec3 relativePosition = aimPosition - missilePosition;
            const float rangeSquared = glm::dot(relativePosition, relativePosition);
            if (rangeSquared < kRangeSquaredFloor)
            {
                return glm::vec3{0.0f};
            }

            const glm::vec3 lineOfSight = relativePosition / std::sqrt(rangeSquared);
            const glm::vec3 relativeVelocity = aimVelocity - missileVelocity;
            const glm::vec3 lineOfSightRate = glm::cross(relativePosition, relativeVelocity) / rangeSquared;
            const float closingSpeed = -glm::dot(relativeVelocity, lineOfSight);
            const float gain = std::clamp(navigationGain, 3.0f, 5.0f);
            glm::vec3 acceleration = gain * closingSpeed * glm::cross(lineOfSightRate, lineOfSight);

            const float speed = glm::length(missileVelocity);
            if (speed > kVelocityFloor)
            {
                const glm::vec3 direction = missileVelocity / speed;
                acceleration -= direction * glm::dot(acceleration, direction);
            }

            const float cap = std::max(maxAcceleration, 0.0f);
            const float magnitude = glm::length(acceleration);
            if (magnitude > cap && magnitude > 1.0e-6f)
            {
                acceleration *= cap / magnitude;
            }
            return acceleration;
        }

        // Strongest return that clears the nose cone, the terrain, the single
        // SNR gate, and, when one is given, the gate sphere about the point the
        // seeker expects. No platform id is read. The cone axis is the nose.
        // Where the seeker expects its target, and, when it is tracking in
        // speed as well, how fast.
        struct Expectation
        {
            const glm::vec3 *position = nullptr;
            const glm::vec3 *velocity = nullptr;
        };

        bool acquire(const RadarHomingSpec &spec, double time, const glm::vec3 &missilePosition, const Terrain &terrain,
                     const SensorBody &missile, const std::vector<SensorBody> &bodies, const Expectation &expect,
                     Observation &measurement)
        {
            const glm::vec3 *gateCenter = expect.position;
            const bool gated = gateCenter != nullptr && spec.seekerGateM > 0.0f;
            const bool speedGated = gateCenter != nullptr && expect.velocity != nullptr;
            const float expectedClosing =
                speedGated ? closingSpeedMps(missilePosition, missile.velocity, *gateCenter, *expect.velocity) : 0.0f;
            const glm::vec3 nose = unitOr(missile.forward, glm::vec3{0.0f, 0.0f, 1.0f});
            const float halfAngle = std::clamp(spec.seekerConeHalfRad, 0.0f, kPi);
            const float minimumCosine = std::cos(halfAngle);

            bool found = false;
            double bestSnr = -1.0;
            for (const SensorBody &body : bodies)
            {
                if (!body.alive)
                {
                    continue;
                }

                const glm::vec3 offset = body.position - missilePosition;
                const float trueRange = glm::length(offset);
                if (trueRange < kCoincidentM)
                {
                    continue;
                }

                const glm::vec3 lineOfSight = offset / trueRange;
                if (glm::dot(lineOfSight, nose) < minimumCosine)
                {
                    continue;
                }
                if (gated && glm::length(body.position - *gateCenter) > spec.seekerGateM)
                {
                    continue;
                }
                if (!terrain.lineOfSight(missilePosition, body.position))
                {
                    continue;
                }
                if (inClutterNotch(spec.doppler, terrain, missilePosition, body.position, body.velocity))
                {
                    continue;
                }
                const float closing = closingSpeedMps(missilePosition, missile.velocity, body.position, body.velocity);
                if (speedGated && !inVelocityGate(spec.doppler, closing, expectedClosing))
                {
                    continue;
                }

                const float aspect = aspectFromNoseRad(body.position, body.forward, missilePosition);
                // A positive body cross section (chaff) replaces the installed profile.
                const float rcs = body.radarCrossSectionM2 > 0.0f ? body.radarCrossSectionM2
                                                                  : meanRadarCrossSectionM2(spec.targetRcs, aspect);
                const float range = std::max(trueRange, kSensorRangeFloorM);
                const double snr = monostaticSnr(spec.seekerRadar, rcs, range);
                if (snr < static_cast<double>(spec.seekerSnrThreshold) || snr <= bestSnr)
                {
                    continue;
                }

                Observation observation;
                observation.family = SensorFamily::Radar;
                observation.time = time;
                observation.origin = missilePosition;
                observation.rangeM = trueRange;
                observation.position = missilePosition + lineOfSight * trueRange;
                observation.quality = snr;
                measurement = observation;
                bestSnr = snr;
                found = true;
            }
            return found;
        }

        bool seekerPowered(Phase phase)
        {
            return phase == Phase::SeekerSearch || phase == Phase::Terminal || phase == Phase::Memory;
        }
    }

    void RadarHoming::remember(double time, float dt, const glm::vec3 &measuredPosition)
    {
        // Rate from the change in measured position only. Not a platform velocity.
        if (m_haveMeasurement)
        {
            const double span = time - m_measurementTime;
            const double divisor = span > 1.0e-4 ? span : static_cast<double>(dt);
            if (divisor > 1.0e-4)
            {
                m_measuredVelocity = (measuredPosition - m_state.seekerAimPoint) / static_cast<float>(divisor);
            }
        }
        m_state.seekerAimPoint = measuredPosition;
        m_measurementTime = time;
        m_haveMeasurement = true;
        m_state.seekerTracking = true;
    }

    void RadarHoming::endAsSelfDestruct()
    {
        // Search timeout and memory timeout use SelfDestruct.
        m_state.phase = Phase::Ended;
        m_state.endReason = ShotEndReason::SelfDestruct;
        m_state.seekerTracking = false;
    }

    void RadarHoming::reset(const RadarHomingSpec &spec)
    {
        m_spec = spec;
        m_state = {};
        m_measuredVelocity = glm::vec3{0.0f};
        m_measurementTime = 0.0;
        m_haveMeasurement = false;
    }

    HomingCommand RadarHoming::step(double time, float dt, const glm::vec3 &missilePosition, const glm::vec3 &missileVelocity,
                                    const SupportPacket *packet, const Terrain &terrain, const SensorBody &missile,
                                    const std::vector<SensorBody> &bodies)
    {
        if (packet != nullptr)
        {
            m_state.lastSupport = *packet;
            m_state.haveSupport = true;
        }

        const bool fresh = supportIsFresh(m_state, m_spec, time);
        if (m_state.haveSupport)
        {
            m_state.lastSupport.fresh = fresh;
        }

        glm::vec3 aimPosition{0.0f};
        glm::vec3 aimVelocity{0.0f};
        bool haveAim = false;

        if (m_state.phase != Phase::Ended)
        {
            const bool haveCoast = m_state.haveSupport;
            glm::vec3 coastPosition{0.0f};
            glm::vec3 coastVelocity{0.0f};
            if (haveCoast)
            {
                coastPosition = coastedPosition(m_state.lastSupport, time);
                coastVelocity = m_state.lastSupport.velocity;
            }

            if (m_state.phase == Phase::Midcourse && haveCoast &&
                glm::length(coastPosition - missilePosition) <= std::max(m_spec.seekerActivationRangeM, 0.0f))
            {
                m_state.phase = Phase::SeekerSearch;
                m_state.phaseStart = time;
            }

            // Where the seeker expects the target: the coasted support point
            // while searching, the extrapolated last measurement afterwards.
            // Its speed gates the search and the track, not the memory.
            glm::vec3 expected{0.0f};
            glm::vec3 expectedVelocity{0.0f};
            Expectation expectation;
            if (m_state.phase == Phase::SeekerSearch && haveCoast)
            {
                expected = coastPosition;
                expectedVelocity = coastVelocity;
                expectation.position = &expected;
                expectation.velocity = &expectedVelocity;
            }
            else if ((m_state.phase == Phase::Terminal || m_state.phase == Phase::Memory) && m_haveMeasurement)
            {
                expected = m_state.seekerAimPoint + m_measuredVelocity * static_cast<float>(time - m_measurementTime);
                expectedVelocity = m_measuredVelocity;
                expectation.position = &expected;
                expectation.velocity = m_state.phase == Phase::Terminal ? &expectedVelocity : nullptr;
            }

            Observation measurement;
            const bool looking = seekerPowered(m_state.phase);
            const bool acquired = looking && acquire(m_spec, time, missilePosition, terrain, missile, bodies, expectation,
                                                     measurement);

            if (m_state.phase == Phase::SeekerSearch)
            {
                if (acquired)
                {
                    m_state.phase = Phase::Terminal;
                    m_state.phaseStart = time;
                    // One look gives a position, not a velocity: the cue's
                    // velocity stands in until the next measurement.
                    if (!m_haveMeasurement && haveCoast)
                    {
                        m_measuredVelocity = coastVelocity;
                    }
                    remember(time, dt, measurement.position);
                }
                else if (time - m_state.phaseStart >= m_spec.searchTimeoutS)
                {
                    endAsSelfDestruct();
                }
            }
            else if (m_state.phase == Phase::Terminal)
            {
                if (acquired)
                {
                    remember(time, dt, measurement.position);
                }
                else
                {
                    m_state.phase = Phase::Memory;
                    m_state.phaseStart = time;
                    m_state.seekerTracking = false;
                }
            }
            else if (m_state.phase == Phase::Memory && acquired)
            {
                m_state.phase = Phase::Terminal;
                m_state.phaseStart = time;
                remember(time, dt, measurement.position);
            }

            if (m_state.phase == Phase::Memory && time - m_state.phaseStart >= m_spec.memoryTimeoutS)
            {
                endAsSelfDestruct();
            }

            if (m_state.phase == Phase::Midcourse || m_state.phase == Phase::SeekerSearch)
            {
                if (haveCoast)
                {
                    aimPosition = coastPosition;
                    aimVelocity = coastVelocity;
                    haveAim = true;
                }
            }
            else if ((m_state.phase == Phase::Terminal || m_state.phase == Phase::Memory) && m_haveMeasurement)
            {
                const float age = static_cast<float>(time - m_measurementTime);
                aimPosition = m_state.seekerAimPoint + m_measuredVelocity * age;
                aimVelocity = m_measuredVelocity;
                haveAim = true;
            }
        }

        HomingCommand command;
        command.phase = m_state.phase;
        command.supportFresh = fresh;
        command.seekerOn = seekerPowered(m_state.phase);
        command.seekerTracking = m_state.seekerTracking;
        if (haveAim)
        {
            command.acceleration = proportionalCommand(missilePosition, missileVelocity, aimPosition, aimVelocity,
                                                       m_spec.navigationGain, m_spec.maxAcceleration);
        }
        return command;
    }

    int runRadarHomingChecks()
    {
        int failures = 0;
        const auto expect = [&](bool ok, const char *name, const std::string &detail = {}) {
            if (ok)
            {
                std::printf("PASS %s\n", name);
                return;
            }
            if (detail.empty())
            {
                std::printf("FAIL %s\n", name);
            }
            else
            {
                std::printf("FAIL %s (%s)\n", name, detail.c_str());
            }
            ++failures;
        };
        const auto text = [](const glm::vec3 &value) {
            char buffer[96];
            std::snprintf(buffer, sizeof(buffer), "%.3f %.3f %.3f", static_cast<double>(value.x),
                          static_cast<double>(value.y), static_cast<double>(value.z));
            return std::string(buffer);
        };
        const auto gap = [](const glm::vec3 &a, const glm::vec3 &b) { return glm::length(a - b); };

        TerrainConfig flatConfig;
        flatConfig.kind = TerrainKind::Flat;
        const Terrain flat(flatConfig);

        const auto radar = [] {
            RadarSet set;
            set.peakPowerW = 10000.0f;
            set.gainTransmit = 1000.0f;
            set.gainReceive = 1000.0f;
            set.wavelengthM = 0.03f;
            set.pulseWidthS = 1.0e-6f;
            set.systemTemperatureK = 290.0f;
            set.systemLoss = 1.0f;
            set.snrThreshold = 10.0f;
            // Longer than the check steps. A scheduler grid would skip them.
            set.revisitS = 100.0;
            return set;
        };

        RadarCrossSectionProfile signature;
        signature.noseM2 = signature.beamM2 = signature.tailM2 = 1.0f;

        const glm::vec3 altitude{0.0f, 2500.0f, 0.0f};
        SensorBody missile;
        missile.position = altitude;
        missile.forward = glm::vec3{0.0f, 0.0f, 1.0f}; // nose, the cone axis
        missile.up = glm::vec3{0.0f, 1.0f, 0.0f};
        const glm::vec3 missileVelocity{300.0f, 0.0f, 0.0f};

        // No support yet: nothing to steer toward, and no truth to borrow.
        {
            RadarHoming homing;
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            homing.reset(spec);
            const HomingCommand quiet =
                homing.step(0.0, 0.1f, missile.position, missileVelocity, nullptr, flat, missile, {});
            expect(homing.state().phase == Phase::Midcourse && quiet.phase == Phase::Midcourse &&
                       !quiet.supportFresh && !quiet.seekerOn && gap(quiet.acceleration, glm::vec3{0.0f}) < 1.0e-4f,
                   "midcourse: with no packet the acceleration is zero");
        }

        // Stationary offset. Pursuit would steer; a collision course would not.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.navigationGain = 4.0f;
            spec.maxAcceleration = 300.0f;

            const glm::vec3 velocity{300.0f, 0.0f, 0.0f};
            SupportPacket collision;
            collision.time = 0.0;
            collision.position = glm::vec3{36000.0f, 2500.0f, 12000.0f};
            collision.velocity = glm::vec3{0.0f, 0.0f, -100.0f};
            collision.fresh = true;
            RadarHoming onCourse;
            onCourse.reset(spec);
            const HomingCommand zero =
                onCourse.step(0.0, 0.1f, missile.position, velocity, &collision, flat, missile, {});
            expect(onCourse.state().phase == Phase::Midcourse && glm::length(zero.acceleration) < 1.0e-3f,
                   "midcourse: a collision course commands no acceleration", text(zero.acceleration));

            SupportPacket offset;
            offset.time = 0.0;
            offset.position = glm::vec3{30000.0f, 2500.0f, -8000.0f};
            offset.velocity = glm::vec3{0.0f};
            offset.fresh = true;
            RadarHoming turning;
            turning.reset(spec);
            const HomingCommand toward =
                turning.step(0.0, 0.1f, missile.position, velocity, &offset, flat, missile, {});
            const glm::vec3 direction = velocity / glm::length(velocity);
            const glm::vec3 line = offset.position - missile.position;
            const glm::vec3 lateral = line - direction * glm::dot(line, direction);
            const float along = glm::dot(toward.acceleration, lateral);
            expect(turning.state().phase == Phase::Midcourse && along > 0.0f && glm::length(toward.acceleration) > 0.2f &&
                       glm::length(toward.acceleration) <= spec.maxAcceleration + 1.0e-3f,
                   "midcourse: the command turns toward the packet position", text(toward.acceleration));
        }

        // Coast the packet. A body on the nose must not replace it.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.navigationGain = 4.0f;
            spec.maxAcceleration = 300.0f;
            spec.datalinkPeriodS = 2.0f;

            SupportPacket packet;
            packet.time = 0.0;
            packet.position = glm::vec3{18000.0f, 2500.0f, -6000.0f};
            packet.velocity = glm::vec3{0.0f, 0.0f, 1500.0f};
            packet.fresh = true;

            SensorBody decoy;
            decoy.position = glm::vec3{0.0f, 2500.0f, 3000.0f};
            decoy.forward = glm::vec3{0.0f, 0.0f, -1.0f};
            decoy.alive = true;
            const std::vector<SensorBody> bodies{decoy};

            RadarHoming homing;
            homing.reset(spec);
            const HomingCommand first =
                homing.step(0.0, 0.1f, missile.position, missileVelocity, &packet, flat, missile, bodies);

            const double elapsed = 8.0;
            const glm::vec3 predicted = packet.position + packet.velocity * static_cast<float>(elapsed - packet.time);
            const HomingCommand coasted =
                homing.step(elapsed, 0.1f, missile.position, missileVelocity, nullptr, flat, missile, bodies);

            SupportPacket predictedPacket;
            predictedPacket.time = elapsed;
            predictedPacket.position = predicted;
            predictedPacket.velocity = packet.velocity;
            predictedPacket.fresh = true;
            RadarHoming reference;
            reference.reset(spec);
            const HomingCommand expected = reference.step(elapsed, 0.1f, missile.position, missileVelocity,
                                                          &predictedPacket, flat, missile, {});

            SupportPacket frozen = packet;
            frozen.time = elapsed;
            RadarHoming stuck;
            stuck.reset(spec);
            const HomingCommand uncoasted =
                stuck.step(elapsed, 0.1f, missile.position, missileVelocity, &frozen, flat, missile, {});

            SupportPacket decoyPacket;
            decoyPacket.time = elapsed;
            decoyPacket.position = decoy.position;
            decoyPacket.velocity = glm::vec3{0.0f};
            decoyPacket.fresh = true;
            RadarHoming stolen;
            stolen.reset(spec);
            const HomingCommand atDecoy =
                stolen.step(elapsed, 0.1f, missile.position, missileVelocity, &decoyPacket, flat, missile, {});

            expect(first.supportFresh && first.phase == Phase::Midcourse && !first.seekerOn &&
                       glm::length(first.acceleration) > 0.2f,
                   "midcourse: a fresh packet is guided and the seeker stays off");
            expect(gap(coasted.acceleration, expected.acceleration) < 1.0e-3f &&
                       gap(coasted.acceleration, uncoasted.acceleration) > 0.25f &&
                       gap(coasted.acceleration, atDecoy.acceleration) > 0.25f,
                   "midcourse: a missed packet still aims at the coasted point",
                   "coast " + text(coasted.acceleration) + " expected " + text(expected.acceleration) + " frozen " +
                       text(uncoasted.acceleration));
            expect(!coasted.supportFresh && coasted.phase == Phase::Midcourse &&
                       homing.state().endReason == ShotEndReason::None && !homing.state().lastSupport.fresh,
                   "midcourse: stopping packets does not end the shot");

            const double later = 40.0;
            const HomingCommand still =
                homing.step(later, 0.1f, missile.position, missileVelocity, nullptr, flat, missile, bodies);
            SupportPacket laterPacket;
            laterPacket.time = later;
            laterPacket.position = packet.position + packet.velocity * static_cast<float>(later - packet.time);
            laterPacket.velocity = packet.velocity;
            RadarHoming laterReference;
            laterReference.reset(spec);
            const HomingCommand laterExpected = laterReference.step(later, 0.1f, missile.position, missileVelocity,
                                                                    &laterPacket, flat, missile, {});
            expect(still.phase == Phase::Midcourse && homing.state().endReason == ShotEndReason::None &&
                       !still.supportFresh && gap(still.acceleration, laterExpected.acceleration) < 1.0e-3f,
                   "midcourse: a long datalink gap keeps coasting", text(still.acceleration));
        }

        // Nose cone, not the velocity axis. Velocity is +X; the nose is +Z.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.seekerSnrThreshold = 10.0f;
            spec.seekerConeHalfRad = 0.5f;
            spec.searchTimeoutS = 4.0;

            SupportPacket cue;
            cue.time = 0.0;
            cue.position = glm::vec3{0.0f, 2500.0f, 4500.0f};
            cue.velocity = glm::vec3{0.0f};
            cue.fresh = true;

            SensorBody onNose;
            onNose.position = glm::vec3{0.0f, 2500.0f, 4000.0f};
            onNose.forward = glm::vec3{0.0f, 0.0f, -1.0f};
            onNose.alive = true;
            const float noseRange = glm::length(onNose.position - missile.position);
            const double noseSnr = monostaticSnr(spec.seekerRadar, 1.0f, noseRange);

            RadarHoming homing;
            homing.reset(spec);
            const HomingCommand terminal =
                homing.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, {onNose});
            expect(noseSnr > static_cast<double>(spec.seekerSnrThreshold) && terminal.phase == Phase::Terminal &&
                       terminal.seekerOn && terminal.seekerTracking &&
                       gap(homing.state().seekerAimPoint, onNose.position) < 0.05f &&
                       gap(homing.state().seekerAimPoint, cue.position) > 100.0f,
                   "seeker: a body in the nose cone, above SNR, and in line of sight is terminal",
                   text(homing.state().seekerAimPoint));

            SensorBody onVelocity;
            onVelocity.position = glm::vec3{4000.0f, 2500.0f, 0.0f};
            onVelocity.forward = glm::vec3{-1.0f, 0.0f, 0.0f};
            onVelocity.alive = true;
            const float velocityRange = glm::length(onVelocity.position - missile.position);
            const glm::vec3 velocityLos = (onVelocity.position - missile.position) / velocityRange;
            const float offNose = std::acos(glm::clamp(glm::dot(velocityLos, missile.forward), -1.0f, 1.0f));
            const double velocitySnr = monostaticSnr(spec.seekerRadar, 1.0f, velocityRange);
            RadarHoming rejected;
            rejected.reset(spec);
            const HomingCommand searching =
                rejected.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, {onVelocity});
            expect(offNose > spec.seekerConeHalfRad && velocitySnr > static_cast<double>(spec.seekerSnrThreshold) &&
                       searching.phase == Phase::SeekerSearch && searching.seekerOn && !searching.seekerTracking &&
                       rejected.state().endReason == ShotEndReason::None &&
                       gap(rejected.state().seekerAimPoint, onVelocity.position) > 100.0f,
                   "seeker: outside the nose cone is not acquired, even above SNR",
                   "angle " + std::to_string(offNose));

            SensorBody buried = onNose;
            buried.position = glm::vec3{0.0f, -50.0f, 8000.0f};
            const bool masked = !flat.lineOfSight(missile.position, buried.position);
            const float buriedRange = glm::length(buried.position - missile.position);
            const glm::vec3 buriedLos = (buried.position - missile.position) / buriedRange;
            const float buriedAngle = std::acos(glm::clamp(glm::dot(buriedLos, missile.forward), -1.0f, 1.0f));
            RadarHoming hidden;
            hidden.reset(spec);
            const HomingCommand blocked =
                hidden.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, {buried});
            expect(masked && buriedAngle < spec.seekerConeHalfRad && blocked.phase == Phase::SeekerSearch &&
                       !blocked.seekerTracking,
                   "seeker: terrain masking is not acquisition");

            RadarHomingSpec deaf = spec;
            deaf.seekerSnrThreshold = 1.0e9f;
            RadarHoming quiet;
            quiet.reset(deaf);
            const HomingCommand below =
                quiet.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, {onNose});
            expect(noseSnr < static_cast<double>(deaf.seekerSnrThreshold) && below.phase == Phase::SeekerSearch &&
                       !below.seekerTracking,
                   "seeker: under the SNR gate is not acquisition");
        }

        // Two measurements, then the body leaves the nose cone.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.seekerConeHalfRad = 0.5f;
            spec.memoryTimeoutS = 3.0;
            spec.searchTimeoutS = 4.0;
            spec.navigationGain = 4.0f;
            spec.maxAcceleration = 300.0f;

            const glm::vec3 velocity{800.0f, 0.0f, 0.0f};
            const glm::vec3 firstPosition{300.0f, 4000.0f, 8000.0f};
            const glm::vec3 secondPosition{500.0f, 4000.0f, 7900.0f};
            const glm::vec3 jumpedPosition{9000.0f, 4000.0f, -500.0f};
            missile.position = glm::vec3{0.0f, 4000.0f, 0.0f};

            SupportPacket cue;
            cue.time = 0.0;
            cue.position = glm::vec3{0.0f, 4000.0f, 9000.0f};
            cue.velocity = glm::vec3{0.0f};
            cue.fresh = true;

            SensorBody body;
            body.forward = glm::vec3{0.0f, 0.0f, -1.0f};
            body.alive = true;
            body.position = firstPosition;

            RadarHoming homing;
            homing.reset(spec);
            const HomingCommand acquired =
                homing.step(0.0, 1.0f, missile.position, velocity, &cue, flat, missile, {body});
            const glm::vec3 firstAim = homing.state().seekerAimPoint;
            body.position = secondPosition;
            const HomingCommand tracking =
                homing.step(1.0, 1.0f, missile.position, velocity, nullptr, flat, missile, {body});
            const glm::vec3 lastMeasurement = homing.state().seekerAimPoint;
            body.position = jumpedPosition;
            const HomingCommand memory =
                homing.step(2.0, 1.0f, missile.position, velocity, nullptr, flat, missile, {body});

            const glm::vec3 measuredVelocity = lastMeasurement - firstAim;
            const glm::vec3 coast = lastMeasurement + measuredVelocity;
            RadarHomingSpec coastSpec = spec;
            coastSpec.seekerActivationRangeM = 1.0f;
            SupportPacket coastPacket;
            coastPacket.time = 2.0;
            coastPacket.position = coast;
            coastPacket.velocity = measuredVelocity;
            RadarHoming coastHoming;
            coastHoming.reset(coastSpec);
            const HomingCommand coastCommand =
                coastHoming.step(2.0, 1.0f, missile.position, velocity, &coastPacket, flat, missile, {});

            SupportPacket truthPacket;
            truthPacket.time = 2.0;
            truthPacket.position = jumpedPosition;
            truthPacket.velocity = measuredVelocity;
            RadarHoming truthHoming;
            truthHoming.reset(coastSpec);
            const HomingCommand truthCommand =
                truthHoming.step(2.0, 1.0f, missile.position, velocity, &truthPacket, flat, missile, {});

            expect(acquired.phase == Phase::Terminal && tracking.phase == Phase::Terminal && tracking.seekerTracking &&
                       gap(lastMeasurement, secondPosition) < 0.05f,
                   "terminal: successive looks stay on the measured position");
            expect(memory.phase == Phase::Memory && memory.seekerOn && !memory.seekerTracking &&
                       homing.state().endReason == ShotEndReason::None && gap(homing.state().seekerAimPoint, secondPosition) < 0.05f &&
                       gap(homing.state().seekerAimPoint, jumpedPosition) > 1000.0f &&
                       gap(memory.acceleration, coastCommand.acceleration) < 1.0e-2f &&
                       gap(memory.acceleration, truthCommand.acceleration) > 0.25f,
                   "memory: loss coasts the last measurement, not the body's new position",
                   "memory " + text(memory.acceleration) + " coast " + text(coastCommand.acceleration) + " truth " +
                       text(truthCommand.acceleration));

            const HomingCommand stillMemory =
                homing.step(4.5, 0.5f, missile.position, velocity, nullptr, flat, missile, {body});
            const HomingCommand ended =
                homing.step(5.0, 0.5f, missile.position, velocity, nullptr, flat, missile, {body});
            expect(stillMemory.phase == Phase::Memory && ended.phase == Phase::Ended &&
                       ended.phase == homing.state().phase && homing.state().endReason == ShotEndReason::SelfDestruct &&
                       gap(ended.acceleration, glm::vec3{0.0f}) < 1.0e-4f && !ended.seekerOn,
                   "memory: timeout is self-destruct and the command goes to zero");
        }

        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.searchTimeoutS = 4.0;
            spec.seekerActivationRangeM = 12000.0f;

            missile.position = glm::vec3{0.0f, 2500.0f, 0.0f};
            SupportPacket cue;
            cue.time = 0.0;
            cue.position = glm::vec3{6000.0f, 2500.0f, 4000.0f};
            cue.velocity = glm::vec3{0.0f};
            cue.fresh = true;

            RadarHoming homing;
            homing.reset(spec);
            const HomingCommand opened =
                homing.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, {});
            const HomingCommand waiting =
                homing.step(2.0, 0.1f, missile.position, missileVelocity, nullptr, flat, missile, {});
            const Phase phaseWhileSearching = homing.state().phase;
            const HomingCommand timedOut =
                homing.step(spec.searchTimeoutS, 0.1f, missile.position, missileVelocity, nullptr, flat, missile, {});
            const ShotEndReason reasonAtTimeout = homing.state().endReason;

            SensorBody late;
            late.position = glm::vec3{0.0f, 2500.0f, 4000.0f};
            late.forward = glm::vec3{0.0f, 0.0f, -1.0f};
            late.alive = true;
            SupportPacket latePacket = cue;
            latePacket.time = spec.searchTimeoutS + 0.5;
            const HomingCommand after = homing.step(spec.searchTimeoutS + 0.5, 0.1f, missile.position, missileVelocity,
                                                    &latePacket, flat, missile, {late});
            expect(opened.phase == Phase::SeekerSearch && opened.seekerOn && !opened.seekerTracking &&
                       waiting.phase == Phase::SeekerSearch && phaseWhileSearching == Phase::SeekerSearch &&
                       glm::length(waiting.acceleration) > 1.0f,
                   "search: support inside range is not acquisition", text(waiting.acceleration));
            expect(timedOut.phase == Phase::Ended && reasonAtTimeout == ShotEndReason::SelfDestruct &&
                       gap(timedOut.acceleration, glm::vec3{0.0f}) < 1.0e-4f && after.phase == Phase::Ended &&
                       !after.seekerOn && gap(after.acceleration, glm::vec3{0.0f}) < 1.0e-4f &&
                       homing.state().endReason == ShotEndReason::SelfDestruct,
                   "search: no acquisition self-destructs at the timeout and stays ended");
        }

        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.seekerActivationRangeM = 1.0f;
            spec.maxAcceleration = 100000.0f;
            const glm::vec3 velocity{500.0f, 0.0f, 0.0f};
            SupportPacket packet;
            packet.time = 0.0;
            packet.position = glm::vec3{8000.0f, 2500.0f, 2000.0f};
            packet.velocity = glm::vec3{0.0f};
            missile.position = glm::vec3{0.0f, 2500.0f, 0.0f};

            spec.navigationGain = 5.0f;
            RadarHoming atFive;
            atFive.reset(spec);
            const HomingCommand five =
                atFive.step(0.0, 0.1f, missile.position, velocity, &packet, flat, missile, {});
            spec.navigationGain = 9.0f;
            RadarHoming atNine;
            atNine.reset(spec);
            const HomingCommand nine =
                atNine.step(0.0, 0.1f, missile.position, velocity, &packet, flat, missile, {});
            spec.navigationGain = 3.0f;
            RadarHoming atThree;
            atThree.reset(spec);
            const HomingCommand three =
                atThree.step(0.0, 0.1f, missile.position, velocity, &packet, flat, missile, {});
            spec.navigationGain = 1.0f;
            RadarHoming atOne;
            atOne.reset(spec);
            const HomingCommand one =
                atOne.step(0.0, 0.1f, missile.position, velocity, &packet, flat, missile, {});

            const float fiveLength = glm::length(five.acceleration);
            const float threeLength = glm::length(three.acceleration);
            expect(gap(nine.acceleration, five.acceleration) < 1.0e-3f && gap(one.acceleration, three.acceleration) < 1.0e-3f &&
                       fiveLength > 1.0f && std::abs(threeLength / fiveLength - 0.6f) < 1.0e-3f,
                   "command: navigation gain is clamped to 3..5",
                   "3/5 " + std::to_string(threeLength / fiveLength));

            spec.navigationGain = 5.0f;
            spec.maxAcceleration = 10.0f;
            RadarHoming capped;
            capped.reset(spec);
            const HomingCommand limited =
                capped.step(0.0, 0.1f, missile.position, velocity, &packet, flat, missile, {});
            expect(fiveLength > 10.0f && std::abs(glm::length(limited.acceleration) - 10.0f) < 1.0e-3f,
                   "command: acceleration is clamped to the declared cap",
                   text(limited.acceleration));
        }

        // Memory keeps the seeker looking. A body back in the cone is terminal again.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.seekerConeHalfRad = 0.5f;
            spec.memoryTimeoutS = 3.0;

            missile.position = glm::vec3{0.0f, 4000.0f, 0.0f};
            const glm::vec3 velocity{800.0f, 0.0f, 0.0f};
            SupportPacket cue;
            cue.position = glm::vec3{0.0f, 4000.0f, 9000.0f};
            cue.fresh = true;

            SensorBody body;
            body.forward = glm::vec3{0.0f, 0.0f, -1.0f};
            body.alive = true;
            body.position = glm::vec3{300.0f, 4000.0f, 8000.0f};

            RadarHoming homing;
            homing.reset(spec);
            const HomingCommand first = homing.step(0.0, 0.5f, missile.position, velocity, &cue, flat, missile, {body});
            body.position = glm::vec3{9000.0f, 4000.0f, -500.0f};
            const HomingCommand lost = homing.step(0.5, 0.5f, missile.position, velocity, nullptr, flat, missile, {body});
            body.position = glm::vec3{400.0f, 4000.0f, 7900.0f};
            const HomingCommand back = homing.step(1.0, 0.5f, missile.position, velocity, nullptr, flat, missile, {body});
            expect(first.phase == Phase::Terminal && lost.phase == Phase::Memory && lost.seekerOn &&
                       back.phase == Phase::Terminal && back.seekerTracking &&
                       gap(homing.state().seekerAimPoint, body.position) < 0.05f &&
                       homing.state().endReason == ShotEndReason::None,
                   "memory: a return in the cone is terminal again");
        }

        // The gate keeps a stronger return away from the expected point out.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.seekerConeHalfRad = 0.5f;

            missile.position = glm::vec3{0.0f, 2500.0f, 0.0f};
            SupportPacket cue;
            cue.position = glm::vec3{0.0f, 2500.0f, 4500.0f};
            cue.fresh = true;

            SensorBody strongBody;
            strongBody.forward = glm::vec3{0.0f, 0.0f, -1.0f};
            strongBody.alive = true;
            strongBody.position = glm::vec3{0.0f, 2500.0f, 1500.0f};
            SensorBody expectedBody = strongBody;
            expectedBody.position = glm::vec3{0.0f, 2500.0f, 4400.0f};
            const std::vector<SensorBody> pair{strongBody, expectedBody};

            RadarHoming open;
            open.reset(spec);
            const HomingCommand ungated = open.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, pair);

            spec.seekerGateM = 2000.0f;
            RadarHoming gatedHoming;
            gatedHoming.reset(spec);
            const HomingCommand gated =
                gatedHoming.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, pair);

            RadarHoming nothing;
            nothing.reset(spec);
            const HomingCommand empty =
                nothing.step(0.0, 0.1f, missile.position, missileVelocity, &cue, flat, missile, {strongBody});
            expect(ungated.phase == Phase::Terminal && gap(open.state().seekerAimPoint, strongBody.position) < 0.05f &&
                       gated.phase == Phase::Terminal && gap(gatedHoming.state().seekerAimPoint, expectedBody.position) < 0.05f &&
                       empty.phase == Phase::SeekerSearch && !empty.seekerTracking,
                   "seeker: the gate rejects a stronger return away from the expected point",
                   text(gatedHoming.state().seekerAimPoint));
        }

        // Pulse-Doppler. Chaff stopped in the air is twenty times the target's
        // echo; only its closing speed gives it away.
        {
            RadarHomingSpec spec;
            spec.seekerRadar = radar();
            spec.targetRcs = signature;
            spec.seekerConeHalfRad = 0.5f;
            spec.seekerGateM = 2000.0f;
            spec.doppler.clutterNotchMps = 30.0f;
            spec.doppler.clutterReachM = 20000.0f;
            spec.doppler.velocityGateMps = 40.0f;

            SensorBody seeker;
            seeker.position = glm::vec3{0.0f, 2500.0f, 0.0f};
            seeker.forward = glm::vec3{0.0f, 0.0f, 1.0f};
            seeker.up = glm::vec3{0.0f, 1.0f, 0.0f};
            seeker.velocity = glm::vec3{0.0f, 0.0f, 600.0f};

            SensorBody chaff;
            chaff.alive = true;
            chaff.position = glm::vec3{0.0f, 2500.0f, 3950.0f};
            chaff.velocity = glm::vec3{0.0f, -1.5f, 0.0f};
            chaff.radarCrossSectionM2 = 20.0f;

            // Level, so the beam goes on into sky: no clutter. Returns the aim point.
            const auto look = [&](const glm::vec3 &targetVelocity, const RadarHomingSpec &with) {
                SensorBody target;
                target.alive = true;
                target.position = glm::vec3{0.0f, 2500.0f, 4000.0f};
                target.velocity = targetVelocity;
                target.forward = glm::vec3{0.0f, 0.0f, 1.0f};
                SupportPacket cue;
                cue.position = target.position;
                cue.velocity = targetVelocity;
                cue.fresh = true;
                RadarHoming homing;
                homing.reset(with);
                homing.step(0.0, 0.01f, seeker.position, seeker.velocity, &cue, flat, seeker, {target, chaff});
                return homing.state().seekerAimPoint;
            };

            // Flying away: 250 m/s less closing than the stopped cloud.
            const glm::vec3 coldAim = look(glm::vec3{0.0f, 0.0f, 250.0f}, spec);
            RadarHomingSpec blind = spec;
            blind.doppler = DopplerFilter{};
            const glm::vec3 blindAim = look(glm::vec3{0.0f, 0.0f, 250.0f}, blind);
            expect(gap(coldAim, glm::vec3{0.0f, 2500.0f, 4000.0f}) < 0.05f && gap(blindAim, chaff.position) < 0.05f,
                   "doppler: chaff does not decoy a seeker off a target that is not beaming", text(coldAim));

            // Beaming: the target's closing speed is the cloud's, and the cloud is louder.
            const glm::vec3 beamAim = look(glm::vec3{250.0f, 0.0f, 0.0f}, spec);
            expect(gap(beamAim, chaff.position) < 0.05f, "doppler: a beaming target's chaff takes the seeker",
                   text(beamAim));

            // Looking down onto the ground: a beaming target is in the clutter notch.
            const auto lookDown = [&](const glm::vec3 &targetVelocity) {
                SensorBody high = seeker;
                high.position = glm::vec3{0.0f, 3000.0f, 0.0f};
                SensorBody target;
                target.alive = true;
                target.position = glm::vec3{0.0f, 500.0f, 4000.0f};
                target.velocity = targetVelocity;
                target.forward = glm::vec3{1.0f, 0.0f, 0.0f};
                high.forward = glm::normalize(target.position - high.position);
                high.velocity = high.forward * 600.0f;
                SupportPacket cue;
                cue.position = target.position;
                cue.velocity = targetVelocity;
                cue.fresh = true;
                RadarHoming homing;
                homing.reset(spec);
                const HomingCommand command =
                    homing.step(0.0, 0.01f, high.position, high.velocity, &cue, flat, high, {target});
                return command.phase;
            };
            const Phase notched = lookDown(glm::vec3{250.0f, 0.0f, 0.0f});
            const Phase hot = lookDown(glm::vec3{0.0f, 0.0f, -250.0f});
            expect(notched == Phase::SeekerSearch && hot == Phase::Terminal,
                   "doppler: a beaming target below the seeker is lost in the ground clutter");
        }

        return failures;
    }
}
