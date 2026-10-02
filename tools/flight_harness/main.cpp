// Flies the game's Jet through scripted scenarios and prints handling numbers.
// Usage: flight_harness [scenario-name-filter | --check]
#include "HandlingChecks.h"
#include "flight/Jet.h"
#include "physics/Atmosphere.h"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>
#include <vector>

using namespace missilesim::flight;

namespace
{
    constexpr float kGravity = 9.80665f;
    constexpr float kFrame = 0.01f; // the game's fixed physics step

    Atmosphere g_atmosphere;

    AirData airAt(float altitude)
    {
        const Atmosphere::State s = g_atmosphere.sample(altitude);
        return {s.densityKgPerCubicMeter, s.speedOfSoundMetersPerSecond};
    }

    struct Stats
    {
        float maxNz = -100.0f;
        float minNz = 100.0f;
        float maxAlphaDeg = -100.0f;
        float maxBetaDeg = 0.0f;
        float maxRollRateDeg = 0.0f;
        float settleTime = -1.0f;   // first time angle-off < 3 deg and stays for 1 s
        float overshootDeg = 0.0f;  // worst angle-off after first reaching 3 deg
        float finalAngleDeg = 0.0f;
        float minAltitude = 1.0e9f;
        bool finite = true;
    };

    Jet makeJet(float altitude, float speed, const glm::vec3 &forward = {0.0f, 0.0f, 1.0f})
    {
        Jet jet;
        jet.reset(glm::vec3(0.0f, altitude, 0.0f), forward * speed, forward, glm::vec3(0.0f, 1.0f, 0.0f));
        return jet;
    }

    // Runs `seconds` with `setup` choosing controls each frame.
    Stats fly(Jet &jet, float seconds, const std::function<void(float, JetControls &)> &setup)
    {
        Stats s;
        float firstInside = -1.0f;
        float insideSince = -1.0f;
        for (float t = 0.0f; t < seconds; t += kFrame)
        {
            JetControls c = jet.controls();
            setup(t, c);
            jet.setControls(c, kFrame);
            jet.step(kFrame, airAt(jet.airframe().position().y), kGravity);
            const AirframeTelemetry &tm = jet.airframe().telemetry();
            s.maxNz = std::max(s.maxNz, tm.normalLoad);
            s.minNz = std::min(s.minNz, tm.normalLoad);
            s.maxAlphaDeg = std::max(s.maxAlphaDeg, glm::degrees(tm.alpha));
            s.maxBetaDeg = std::max(s.maxBetaDeg, std::abs(glm::degrees(tm.beta)));
            s.maxRollRateDeg = std::max(s.maxRollRateDeg, std::abs(glm::degrees(jet.airframe().bodyRates().x)));
            s.minAltitude = std::min(s.minAltitude, jet.airframe().position().y);
            const float angle = glm::degrees(jet.instructor().status().angleOff);
            s.finalAngleDeg = angle;
            if (angle < 3.0f)
            {
                if (firstInside < 0.0f)
                {
                    firstInside = t;
                }
                if (insideSince < 0.0f)
                {
                    insideSince = t;
                }
                if (s.settleTime < 0.0f && t - insideSince >= 1.0f)
                {
                    s.settleTime = insideSince;
                }
            }
            else
            {
                insideSince = -1.0f;
                if (s.settleTime >= 0.0f)
                {
                    s.settleTime = -1.0f; // left the cone again
                }
            }
            if (firstInside >= 0.0f)
            {
                s.overshootDeg = std::max(s.overshootDeg, angle);
            }
            const glm::vec3 p = jet.airframe().position();
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(tm.alpha))
            {
                s.finite = false;
                break;
            }
        }
        return s;
    }

    void report(const char *name, const Jet &jet, const Stats &s)
    {
        const AirframeTelemetry &tm = jet.airframe().telemetry();
        std::printf("%-34s settle %5.2fs  over %4.1f  final %5.1f  Nz %5.2f/%5.2f  a %5.1f  b %4.1f  p %5.0f  V %5.0f  M %4.2f  alt %6.0f min %6.0f%s\n",
                    name, s.settleTime, s.overshootDeg, s.finalAngleDeg, s.minNz, s.maxNz, s.maxAlphaDeg, s.maxBetaDeg, s.maxRollRateDeg,
                    tm.airspeed, tm.mach, jet.airframe().position().y, s.minAltitude, s.finite ? "" : "  NON-FINITE");
    }

    glm::vec3 direction(float azimuthDeg, float elevationDeg)
    {
        // Azimuth measured from +z toward the pilot's right (-x); elevation up.
        const float az = glm::radians(azimuthDeg);
        const float el = glm::radians(elevationDeg);
        return glm::vec3(-std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el));
    }

    bool matches(const std::string &filter, const char *name)
    {
        return filter.empty() || std::string(name).find(filter) != std::string::npos;
    }
}

int main(int argc, char **argv)
{
    const std::string filter = argc > 1 ? argv[1] : "";
    if (filter == "--check")
    {
        return runHandlingChecks();
    }

    // ---- Mouse-aim step responses: aim jumps once, instructor flies there.
    struct AimCase
    {
        const char *name;
        float altitude, speed, azimuth, elevation, seconds;
    };
    const AimCase aims[] = {
        {"aim hold level 250m/s 3km", 3000.0f, 250.0f, 0.0f, 0.0f, 30.0f},
        {"aim 5 right 250m/s", 3000.0f, 250.0f, 5.0f, 0.0f, 8.0f},
        {"aim 5 up 250m/s", 3000.0f, 250.0f, 0.0f, 5.0f, 8.0f},
        {"aim 5 down 250m/s", 3000.0f, 250.0f, 0.0f, -5.0f, 8.0f},
        {"aim 30 up 250m/s", 3000.0f, 250.0f, 0.0f, 30.0f, 10.0f},
        {"aim 30 right 250m/s", 3000.0f, 250.0f, 30.0f, 0.0f, 10.0f},
        {"aim 45 right-up 250m/s", 3000.0f, 250.0f, 45.0f, 20.0f, 10.0f},
        {"aim 90 left 250m/s", 3000.0f, 250.0f, -90.0f, 0.0f, 12.0f},
        {"aim 150 right 250m/s", 3000.0f, 250.0f, 150.0f, 0.0f, 16.0f},
        {"aim 40 down 250m/s", 3000.0f, 250.0f, 0.0f, -40.0f, 10.0f},
        {"aim 80 up 250m/s", 3000.0f, 250.0f, 0.0f, 80.0f, 12.0f},
        {"aim 30 right 150m/s 1km", 1000.0f, 150.0f, 30.0f, 0.0f, 12.0f},
        {"aim 90 right 400m/s 8km", 8000.0f, 400.0f, 90.0f, 0.0f, 16.0f},
        {"aim 30 up 110m/s 6km", 6000.0f, 110.0f, 0.0f, 30.0f, 14.0f},
    };
    for (const AimCase &c : aims)
    {
        if (!matches(filter, c.name))
        {
            continue;
        }
        Jet jet = makeJet(c.altitude, c.speed);
        const glm::vec3 aim = direction(c.azimuth, c.elevation);
        const Stats s = fly(jet, c.seconds, [&](float, JetControls &controls) {
            controls.instructor.aimDirection = aim;
            controls.throttle = 1.0f;
        });
        report(c.name, jet, s);
    }

    // ---- Small-input response, any card: how hard the jet reacts to a nudge
    // of the aim. Roll acceleration and g onset are what the player feels;
    // a real fighter needs a few tenths of a second to build either.
    if (matches(filter, "response"))
    {
        std::vector<const char *> cards;
        for (int i = 0; i < aircraftCatalogCount(); ++i)
        {
            if (aircraftCatalog()[i].flyable)
            {
                cards.push_back(aircraftCatalog()[i].id);
            }
        }
        const struct
        {
            const char *name;
            float azimuth, elevation;
        } nudges[] = {{"1 right", 1.0f, 0.0f}, {"3 right", 3.0f, 0.0f}, {"3 up", 0.0f, 3.0f},   {"10 right", 10.0f, 0.0f},
                      {"30 right", 30.0f, 0.0f}, {"30 up", 0.0f, 30.0f}, {"120 right", 120.0f, 0.0f}};
        for (const char *card : cards)
        {
            for (const auto &nudge : nudges)
            {
                Jet jet(card);
                jet.reset(glm::vec3(0.0f, 3000.0f, 0.0f), glm::vec3(0.0f, 0.0f, 250.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                          glm::vec3(0.0f, 1.0f, 0.0f));
                // Settle trimmed on a level aim first, so each jet starts from
                // its own steady nose, then nudge the aim from that nose.
                const glm::vec3 level = direction(0.0f, 0.0f);
                for (float t = 0.0f; t < 4.0f; t += kFrame)
                {
                    JetControls controls = jet.controls();
                    controls.instructor.aimDirection = level;
                    controls.throttle = 0.9f;
                    jet.setControls(controls, kFrame);
                    jet.step(kFrame, airAt(jet.position().y), kGravity);
                }
                const glm::vec3 nose = jet.forward();
                const float az = glm::radians(nudge.azimuth);
                const float el = glm::radians(nudge.elevation);
                const glm::vec3 aim = glm::normalize(nose * (std::cos(az) * std::cos(el)) + jet.right() * (std::sin(az) * std::cos(el)) +
                                                     jet.up() * std::sin(el));
                float previousRoll = glm::degrees(jet.bodyRates().x);
                float previousNz = jet.telemetry().normalLoad;
                float peakRoll = 0.0f;
                float peakRollAccel = 0.0f;
                float peakOnset = 0.0f;
                float peakNz = 0.0f;
                float peakBank = 0.0f;
                float settle = -1.0f;
                float insideSince = -1.0f;
                float worstAfter = 0.0f;
                bool reached = false;
                float finalOff = 0.0f;
                for (float t = 0.0f; t < 12.0f; t += kFrame)
                {
                    JetControls controls = jet.controls();
                    controls.instructor.aimDirection = aim;
                    controls.throttle = 0.9f;
                    jet.setControls(controls, kFrame);
                    jet.step(kFrame, airAt(jet.position().y), kGravity);
                    const float roll = glm::degrees(jet.bodyRates().x);
                    const float nz = jet.telemetry().normalLoad;
                    peakRoll = std::max(peakRoll, std::abs(roll));
                    peakRollAccel = std::max(peakRollAccel, std::abs(roll - previousRoll) / kFrame);
                    peakOnset = std::max(peakOnset, std::abs(nz - previousNz) / kFrame);
                    peakNz = std::max(peakNz, nz);
                    peakBank = std::max(peakBank, std::abs(glm::degrees(std::atan2(jet.right().y, jet.up().y))));
                    previousRoll = roll;
                    previousNz = nz;
                    const float off = glm::degrees(jet.instructor().status().angleOff);
                    finalOff = off;
                    if (off < 0.5f)
                    {
                        reached = true;
                        insideSince = insideSince < 0.0f ? t : insideSince;
                        if (settle < 0.0f && t - insideSince >= 0.5f)
                        {
                            settle = insideSince;
                        }
                    }
                    else
                    {
                        insideSince = -1.0f;
                    }
                    if (reached)
                    {
                        worstAfter = std::max(worstAfter, off);
                    }
                }
                std::printf("response %-15s %-9s roll %6.1f deg/s  roll accel %7.0f deg/s2  onset %6.1f g/s  Nz %4.2f  bank %5.1f  settle %5.2fs  over %4.2f  final %5.2f\n",
                            card, nudge.name, peakRoll, peakRollAccel, peakOnset, peakNz, peakBank, settle, worstAfter, finalOff);
            }
        }
    }

    // ---- Tracking a steadily moving aim point (a turning target).
    if (matches(filter, "track"))
    {
        Jet jet = makeJet(3000.0f, 250.0f);
        const Stats s = fly(jet, 20.0f, [&](float t, JetControls &controls) {
            controls.instructor.aimDirection = direction(12.0f * t, 0.0f); // 12 deg/s sweep
            controls.throttle = 1.0f;
        });
        report("track 12deg/s sweep 250m/s", jet, s);
    }

    // ---- Keyboard stick: full roll, full pull, low-speed pull.
    if (matches(filter, "stick"))
    {
        Jet roll = makeJet(3000.0f, 250.0f);
        report("stick full roll 250m/s", roll, fly(roll, 2.0f, [&](float, JetControls &controls) {
                   controls.instructor.aimDirection = roll.airframe().forward();
                   controls.instructor.rollKey = 1.0f;
                   controls.throttle = 1.0f;
               }));
        Jet slowRoll = makeJet(6000.0f, 120.0f);
        report("stick full roll 120m/s 6km", slowRoll, fly(slowRoll, 2.0f, [&](float, JetControls &controls) {
                   controls.instructor.aimDirection = slowRoll.airframe().forward();
                   controls.instructor.rollKey = 1.0f;
                   controls.throttle = 1.0f;
               }));
        Jet pull = makeJet(3000.0f, 250.0f);
        report("stick full pull 250m/s 6s", pull, fly(pull, 6.0f, [&](float, JetControls &controls) {
                   controls.instructor.aimDirection = pull.airframe().forward();
                   controls.instructor.pitchKey = 1.0f;
                   controls.afterburner = true;
               }));
        Jet slow = makeJet(6000.0f, 110.0f);
        report("stick full pull 110m/s 6km", slow, fly(slow, 6.0f, [&](float, JetControls &controls) {
                   controls.instructor.aimDirection = slow.airframe().forward();
                   controls.instructor.pitchKey = 1.0f;
                   controls.throttle = 1.0f;
               }));
        Jet push = makeJet(3000.0f, 250.0f);
        report("stick full push 250m/s 3s", push, fly(push, 3.0f, [&](float, JetControls &controls) {
                   controls.instructor.aimDirection = push.airframe().forward();
                   controls.instructor.pitchKey = -1.0f;
                   controls.throttle = 1.0f;
               }));
        Jet rollPull = makeJet(3000.0f, 200.0f);
        report("stick roll while pulling 200m/s", rollPull, fly(rollPull, 4.0f, [&](float, JetControls &controls) {
                   controls.instructor.aimDirection = rollPull.airframe().forward();
                   controls.instructor.pitchKey = 1.0f;
                   controls.instructor.rollKey = 1.0f;
                   controls.throttle = 1.0f;
               }));
    }

    // ---- Energy: level acceleration and top speed, sustained turn.
    if (matches(filter, "energy"))
    {
        const struct
        {
            const char *name;
            float altitude, speed, seconds;
            bool afterburner;
        } legs[] = {
            {"energy mil SL from 150m/s 120s", 0.0f + 300.0f, 150.0f, 120.0f, false},
            {"energy AB SL from 150m/s 120s", 300.0f, 150.0f, 120.0f, true},
            {"energy AB 9km from 200m/s 240s", 9000.0f, 200.0f, 240.0f, true},
            {"energy idle 250m/s 30s", 3000.0f, 250.0f, 30.0f, false},
        };
        for (const auto &leg : legs)
        {
            Jet jet = makeJet(leg.altitude, leg.speed);
            const bool idle = std::string(leg.name).find("idle") != std::string::npos;
            report(leg.name, jet, fly(jet, leg.seconds, [&](float, JetControls &controls) {
                       controls.instructor.aimDirection = glm::vec3(0.0f, 0.0f, 1.0f);
                       controls.throttle = idle ? 0.0f : 1.0f;
                       controls.afterburner = leg.afterburner;
                   }));
        }

        // Sustained level turn: aim sweeps as fast as the jet can hold altitude.
        Jet turn = makeJet(3000.0f, 230.0f);
        float heading = 0.0f;
        const Stats s = fly(turn, 60.0f, [&](float, JetControls &controls) {
            heading += 25.0f * kFrame; // ask more than it can do
            controls.instructor.aimDirection = direction(heading, 0.0f);
            controls.afterburner = true;
        });
        const glm::vec3 v = turn.airframe().velocity();
        std::printf("sustained turn AB 3km: final V %.0f m/s, Nz %.2f, turn rate %.1f deg/s (V*omega = Nz_h*g)\n",
                    glm::length(v), turn.airframe().telemetry().normalLoad,
                    glm::degrees(kGravity * std::sqrt(std::max(0.0f, turn.airframe().telemetry().normalLoad * turn.airframe().telemetry().normalLoad - 1.0f)) /
                                 std::max(glm::length(v), 1.0f)));
        report("sustained turn AB 3km", turn, s);
    }

    // High-speed turn: heading change, load, and alpha at the HUD's 400 m/s case.
    if (matches(filter, "diag"))
    {
        const auto headingOf = [](const Jet &jet) {
            const glm::vec3 v = jet.airframe().velocity();
            return std::atan2(v.x, v.z);
        };
        const auto bankDeg = [](const Jet &jet) {
            const auto &air = jet.airframe();
            return glm::degrees(std::atan2(air.right().y, air.up().y));
        };
        const struct
        {
            const char *name;
            float altitude;
            float speed;
        } points[] = {
            {"250m/s 3km", 3000.0f, 250.0f},
            {"400m/s SL", 300.0f, 400.0f},
            {"400m/s 3km", 3000.0f, 400.0f},
            {"400m/s 8km", 8000.0f, 400.0f},
            {"500m/s SL", 300.0f, 500.0f},
        };
        const auto run = [&](Jet &jet, float seconds, const std::function<void(float, JetControls &)> &setup) {
            float maxNz = -100.0f;
            float maxAlpha = -100.0f;
            for (float t = 0.0f; t < seconds; t += kFrame)
            {
                JetControls controls = jet.controls();
                setup(t, controls);
                jet.setControls(controls, kFrame);
                jet.step(kFrame, airAt(jet.airframe().position().y), kGravity);
                const AirframeTelemetry &tm = jet.airframe().telemetry();
                maxNz = std::max(maxNz, tm.normalLoad);
                maxAlpha = std::max(maxAlpha, glm::degrees(tm.alpha));
            }
            return std::pair<float, float>{maxNz, maxAlpha};
        };
        for (const auto &point : points)
        {
            Jet aimed = makeJet(point.altitude, point.speed);
            const float aimStart = headingOf(aimed);
            const auto aimPeak = run(aimed, 8.0f, [&](float, JetControls &controls) {
                controls.instructor.aimDirection = direction(90.0f, 0.0f);
                controls.afterburner = true;
            });
            const AirframeTelemetry &aimTm = aimed.airframe().telemetry();
            std::printf("diag aim90 %-12s  dHdg %6.1f  Nz %5.2f/%5.2f  a %5.1f  bank %6.1f  V %5.0f  M %4.2f  lim %d/%d\n",
                        point.name, glm::degrees(headingOf(aimed) - aimStart), aimTm.normalLoad, aimPeak.first,
                        aimPeak.second, bankDeg(aimed), aimTm.airspeed, aimTm.mach,
                        aimed.flightControl().status().loadLimited ? 1 : 0,
                        aimed.flightControl().status().alphaLimited ? 1 : 0);

            Jet banked = makeJet(point.altitude, point.speed);
            const float bankStart = headingOf(banked);
            const auto bankPeak = run(banked, 6.0f, [&](float t, JetControls &controls) {
                controls.instructor.aimDirection = banked.airframe().forward();
                controls.instructor.rollKey = t < 0.45f ? 1.0f : 0.0f;
                controls.instructor.pitchKey = t >= 0.45f ? 1.0f : 0.0f;
                controls.afterburner = true;
            });
            const AirframeTelemetry &bankTm = banked.airframe().telemetry();
            std::printf("diag bank+pull %-12s  dHdg %6.1f  Nz %5.2f/%5.2f  a %5.1f  bank %6.1f  V %5.0f  M %4.2f\n",
                        point.name, glm::degrees(headingOf(banked) - bankStart), bankTm.normalLoad, bankPeak.first,
                        bankPeak.second, bankDeg(banked), bankTm.airspeed, bankTm.mach);
        }
    }
    return 0;
}
