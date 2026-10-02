#include "EngagementChecks.h"

#include "Scenarios.h"

#include "objects/Fighter.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "sim/EngagementRules.h"
#include "sim/FixedStepClock.h"
#include "sim/Random.h"
#include "sim/Sweep.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>

namespace harness
{
    namespace sim = missilesim::sim;
    using sim::EventType;
    using sim::SimEvent;

    namespace
    {
        struct Checks
        {
            int failures = 0;
            int count = 0;

            void expect(bool passed, const std::string &name, const std::string &detail = std::string())
            {
                ++count;
                if (!passed)
                {
                    ++failures;
                }
                std::printf("%s  %-44s %s\n", passed ? "PASS" : "FAIL", name.c_str(), detail.c_str());
            }
        };

        std::string format(const char *pattern, double a = 0.0, double b = 0.0, double c = 0.0)
        {
            char buffer[256];
            std::snprintf(buffer, sizeof(buffer), pattern, a, b, c);
            return buffer;
        }

        std::size_t countEvents(const std::vector<SimEvent> &trace, EventType type)
        {
            return static_cast<std::size_t>(std::count_if(trace.begin(), trace.end(),
                                                          [type](const SimEvent &event) { return event.type == type; }));
        }

        const Scenario &scenarioNamed(const std::vector<Scenario> &scenarios, const char *name)
        {
            const Scenario *scenario = findScenario(scenarios, name);
            if (scenario == nullptr)
            {
                std::fprintf(stderr, "missing scenario %s\n", name);
                std::abort();
            }
            return *scenario;
        }

        // ---- Reproducibility ---------------------------------------------------------

        void checkReproducibleTraces(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            for (const Scenario &scenario : scenarios)
            {
                const RunResult first = runScenario(config, scenario);
                const RunResult second = runScenario(config, scenario);
                char detail[160];
                std::snprintf(detail, sizeof(detail), "%zu events, hash %016llx", first.trace.size(),
                              static_cast<unsigned long long>(first.hash));
                checks.expect(!first.trace.empty() && first.hash == second.hash && first.trace.size() == second.trace.size(),
                              "same seed, same trace: " + scenario.name, detail);
            }
        }

        void checkSeedChangesScenario(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            Scenario a = scenarioNamed(scenarios, "fox2-single");
            Scenario b = a;
            a.durationSeconds = 0.05;
            b.durationSeconds = 0.05;
            b.seed = a.seed + 1;
            const RunResult first = runScenario(config, a);
            const RunResult second = runScenario(config, b);
            const auto spawnOf = [](const RunResult &run) {
                for (const SimEvent &event : run.trace)
                {
                    if (event.type == EventType::PlatformSpawned && event.value > 0.0f)
                    {
                        return event.position; // targets carry their radius in value
                    }
                }
                return glm::vec3(0.0f);
            };
            checks.expect(glm::length(spawnOf(first) - spawnOf(second)) > 1.0f, "a different seed moves the targets");
        }

        void checkFramePacing(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            const Scenario &scenario = scenarioNamed(scenarios, "sam-single");
            const RunResult direct = runScenario(config, scenario);
            const RunResult at30 = runScenario(config, scenario, nullptr, [](std::uint64_t) { return 1.0f / 30.0f; });
            const RunResult at144 = runScenario(config, scenario, nullptr, [](std::uint64_t) { return 1.0f / 144.0f; });
            const RunResult jittery = runScenario(config, scenario, nullptr, [](std::uint64_t frame) {
                std::uint64_t state = frame;
                const std::uint64_t bits = sim::splitMix64(state);
                return 0.004f + 0.046f * static_cast<float>(bits >> 40) / static_cast<float>(1u << 24);
            });
            checks.expect(direct.hash == at30.hash && direct.hash == at144.hash && direct.hash == jittery.hash,
                          "frame pacing does not change the simulation",
                          "direct, 30 Hz, 144 Hz and jittered frames");
        }

        void checkClockPolicy(Checks &checks)
        {
            sim::FixedStepClock clock(0.01f);
            int steps = 0;
            for (int frame = 0; frame < 60; ++frame)
            {
                steps += clock.advance(1.0f / 60.0f, 1.0f);
            }
            checks.expect(steps >= 99 && steps <= 100, "clock: one second at 60 Hz is 100 steps", format("%.0f steps", steps));

            clock.reset();
            steps = 0;
            for (int frame = 0; frame < 60; ++frame)
            {
                steps += clock.advance(1.0f / 60.0f, 10.0f);
            }
            checks.expect(steps >= 999 && steps <= 1000 && clock.droppedSeconds() == 0.0,
                          "clock: 10x time scale at 60 Hz keeps up", format("%.0f steps, %.3f s dropped", steps, clock.droppedSeconds()));

            clock.reset();
            steps = clock.advance(0.5f, 1.0f);
            checks.expect(steps == 10, "clock: a 0.5 s hitch is replayed as 0.1 s", format("%.0f steps", steps));

            clock.reset();
            steps = 0;
            for (int frame = 0; frame < 10; ++frame)
            {
                steps += clock.advance(0.2f, 10.0f);
            }
            checks.expect(steps == 10 * sim::FixedStepClock::kMaxStepsPerFrame && clock.droppedSeconds() > 0.0 && clock.alpha() <= 1.0f,
                          "clock: overload drops time instead of spiralling",
                          format("%.0f steps, %.2f s dropped", steps, clock.droppedSeconds()));
        }

        // ---- Weapons and outcomes -------------------------------------------------------

        void checkSalvoIndependence(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            const Scenario &scenario = scenarioNamed(scenarios, "fox2-salvo");
            std::size_t concurrent = 0;
            bool otherKeptFlying = false;
            bool firstEndSeen = false;
            const RunResult run = runScenario(config, scenario, [&](World &world, const std::vector<SimEvent> &events) {
                concurrent = std::max(concurrent, world.shots().size());
                for (const SimEvent &event : events)
                {
                    if (event.type == EventType::ShotEnded && !firstEndSeen)
                    {
                        firstEndSeen = true;
                        // The other round is untouched by this one ending.
                        for (const auto &shot : world.shots())
                        {
                            if (shot->id != event.subject && !shot->ended())
                            {
                                otherKeptFlying = true;
                            }
                        }
                    }
                }
            });

            std::map<std::uint32_t, int> ends;
            for (const SimEvent &event : run.trace)
            {
                if (event.type == EventType::ShotEnded)
                {
                    ++ends[event.subject.value];
                }
            }
            checks.expect(countEvents(run.trace, EventType::WeaponLaunched) == 2 && concurrent == 2,
                          "salvo: two rounds in the air at once", format("peak %.0f in flight", static_cast<double>(concurrent)));
            checks.expect(otherKeptFlying, "salvo: one round ending leaves the other flying");
            bool eachOnce = ends.size() == 2;
            for (const auto &entry : ends)
            {
                eachOnce = eachOnce && entry.second == 1;
            }
            checks.expect(eachOnce, "salvo: each round ends exactly once");
        }

        void checkOutcomesOnceOnly(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            bool ok = true;
            std::string problem;
            std::size_t destroyed = 0;
            std::size_t detonations = 0;
            for (const Scenario &scenario : scenarios)
            {
                const RunResult run = runScenario(config, scenario);
                std::set<std::uint32_t> launched;
                std::map<std::uint32_t, int> ended;
                std::map<std::uint32_t, std::uint64_t> endTick;
                std::set<std::uint32_t> down; // destroyed and not yet back in play
                std::set<std::uint32_t> created;
                const double step = static_cast<double>(config.environment.fixedTimeStep);
                for (const SimEvent &event : run.trace)
                {
                    const auto fail = [&](const char *what) {
                        if (ok)
                        {
                            problem = scenario.name + ": " + what + " (subject " + std::to_string(event.subject.value) + ")";
                        }
                        ok = false;
                    };
                    // Events inside a step refer to a time inside that step.
                    if (event.tick > 0 && (event.time < static_cast<double>(event.tick - 1) * step - 1.0e-9 ||
                                           event.time > static_cast<double>(event.tick) * step + 1.0e-9))
                    {
                        fail("event time outside its step");
                    }
                    switch (event.type)
                    {
                    case EventType::PlatformSpawned:
                    case EventType::WeaponLaunched:
                    case EventType::CountermeasureReleased:
                        if (!created.insert(event.subject.value).second)
                        {
                            fail("entity id reused");
                        }
                        if (event.type == EventType::WeaponLaunched)
                        {
                            launched.insert(event.subject.value);
                        }
                        break;
                    case EventType::ShotEnded:
                        if (launched.count(event.subject.value) == 0)
                        {
                            fail("a shot ended that never launched");
                        }
                        if (++ended[event.subject.value] > 1)
                        {
                            fail("a shot ended twice");
                        }
                        endTick[event.subject.value] = event.tick;
                        break;
                    case EventType::Detonation:
                        ++detonations;
                        break;
                    case EventType::Damage:
                        if (down.count(event.subject.value) != 0)
                        {
                            fail("damage to a destroyed platform");
                        }
                        break;
                    case EventType::PlatformDestroyed:
                        ++destroyed;
                        if (!down.insert(event.subject.value).second)
                        {
                            fail("a platform destroyed twice");
                        }
                        break;
                    case EventType::PlatformRespawned:
                        down.erase(event.subject.value);
                        break;
                    default:
                        break;
                    }
                }
                // Every detonation ends its shot in the same step.
                for (const SimEvent &event : run.trace)
                {
                    if (event.type == EventType::Detonation)
                    {
                        const auto found = endTick.find(event.subject.value);
                        if (found == endTick.end() || found->second != event.tick)
                        {
                            ok = false;
                            problem = scenario.name + ": detonation without its shot ending";
                        }
                    }
                }
            }
            char detail[200];
            std::snprintf(detail, sizeof(detail), "%zu detonations, %zu platforms destroyed%s%s", detonations, destroyed,
                          ok ? "" : "; ", problem.c_str());
            checks.expect(ok, "outcomes are once-only and inside their step", detail);
        }

        void checkPlayerDamage(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            const Scenario &scenario = scenarioNamed(scenarios, "threat-on-player");
            sim::EntityId fighterId;
            bool aliveAtEnd = false;
            float healthAtEnd = 0.0f;
            const RunResult run = runScenario(config, scenario, [&](World &world, const std::vector<SimEvent> &) {
                fighterId = world.fighterId();
                aliveAtEnd = world.platformAlive(fighterId);
                healthAtEnd = world.health(fighterId);
            });

            std::size_t damage = 0;
            std::size_t destroyed = 0;
            std::size_t respawned = 0;
            for (const SimEvent &event : run.trace)
            {
                if (event.subject != fighterId)
                {
                    continue;
                }
                damage += event.type == EventType::Damage ? 1u : 0u;
                destroyed += event.type == EventType::PlatformDestroyed ? 1u : 0u;
                respawned += event.type == EventType::PlatformRespawned ? 1u : 0u;
            }
            char detail[128];
            std::snprintf(detail, sizeof(detail), "%zu damage, %zu destroyed, %zu respawned", damage, destroyed, respawned);
            checks.expect(damage >= 1 && destroyed == 1 && respawned == 1 && aliveAtEnd && healthAtEnd == 1.0f,
                          "the player can be damaged, destroyed and respawned", detail);
        }

        void checkRemovalMidFlight(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            Scenario scenario = scenarioNamed(scenarios, "fox2-single");
            scenario.name = "fox2-remove-target";
            scenario.durationSeconds = 8.0;
            sim::EntityId removed;
            const auto launch = scenario.beforeStep;
            scenario.beforeStep = [&removed, launch](World &world, std::uint64_t tick) {
                launch(world, tick);
                if (tick == tickAt(world, 1.5) && !world.shots().empty())
                {
                    removed = world.shots().front()->designatedTarget;
                    world.removeTarget(removed);
                }
            };

            bool referencesValid = true;
            bool removedGone = true;
            runScenario(config, scenario, [&](World &world, const std::vector<SimEvent> &) {
                std::set<const Target *> live;
                for (const auto &target : world.targets())
                {
                    live.insert(target.get());
                }
                const auto valid = [&live](const Missile *missile) {
                    return missile == nullptr || missile->getTargetObject() == nullptr || live.count(missile->getTargetObject()) != 0;
                };
                for (const auto &shot : world.shots())
                {
                    referencesValid = referencesValid && valid(shot->missile.get());
                }
                for (const sim::Station &station : world.stations())
                {
                    referencesValid = referencesValid && valid(station.round.get());
                }
                if (removed.valid())
                {
                    removedGone = removedGone && world.findTarget(removed) == nullptr;
                }
            });
            checks.expect(removed.valid() && removedGone && referencesValid, "removing a target mid-flight leaves no references");
        }

        void checkFlareBirths(Checks &checks, const sim::SimulationConfig &config, const std::vector<Scenario> &scenarios)
        {
            std::size_t released = 0;
            bool present = true;
            bool stamped = true;
            bool expiredGone = true;
            for (const char *name : {"sam-single", "sam-ripple", "fox2-salvo"})
            {
                runScenario(config, scenarioNamed(scenarios, name), [&](World &world, const std::vector<SimEvent> &events) {
                    for (const SimEvent &event : events)
                    {
                        if (event.type != EventType::CountermeasureReleased && event.type != EventType::CountermeasureExpired)
                        {
                            continue;
                        }
                        const bool inWorld = std::any_of(world.flares().begin(), world.flares().end(), [&](const std::unique_ptr<Flare> &flare) {
                            return flare->getEntityId() == event.subject;
                        });
                        stamped = stamped && event.tick == world.tick();
                        if (event.type == EventType::CountermeasureReleased)
                        {
                            ++released;
                            present = present && inWorld;
                        }
                        else
                        {
                            expiredGone = expiredGone && !inWorld;
                        }
                    }
                });
            }
            char detail[96];
            std::snprintf(detail, sizeof(detail), "%zu flares released", released);
            checks.expect(released > 0 && present && stamped && expiredGone,
                          "flares join and leave at the step boundary", detail);
        }

        void checkLaunchRefusals(Checks &checks, const sim::SimulationConfig &config)
        {
            {
                World world(config, 7);
                world.respawnTargets(1);
                world.setRoleSam(world.customRoundSpec());
                const sim::LaunchBlock caged = world.launchClearance();
                world.setSeekerUncaged(true);
                const sim::LaunchBlock undesignated = world.launchClearance();
                world.designate(world.targets().front()->getEntityId());
                const sim::LaunchBlock ready = world.launchClearance();
                const bool launched = world.launch(glm::vec3(0.0f, 0.0f, 1.0f)).launched();
                const sim::LaunchBlock reloading = world.launchClearance();
                while (world.time() < static_cast<double>(sim::rules::kSamReloadSeconds) + 0.05)
                {
                    world.step();
                }
                const sim::LaunchBlock reloaded = world.launchClearance();
                checks.expect(caged == sim::LaunchBlock::SeekerCaged && undesignated == sim::LaunchBlock::NoDesignation &&
                                  ready == sim::LaunchBlock::None && launched && reloading == sim::LaunchBlock::Reloading &&
                                  reloaded == sim::LaunchBlock::SeekerCaged,
                              "SAM refusals: caged, no target, reloading");
            }
            {
                World world(config, 8);
                world.respawnTargets(1);
                world.setRoleFighter("aim-9x-blk2");
                world.step();
                const bool first = fireFox2(world).valid();
                const bool second = fireFox2(world).valid();
                const sim::LaunchBlock empty = world.launchClearance();
                const int remaining = world.roundsRemaining();
                world.rearm();
                checks.expect(first && second && empty == sim::LaunchBlock::NoRound && remaining == 0 && world.roundsRemaining() == 2,
                              "fighter stores: two rails, empty, rearm");
            }
        }

        // ---- Swept collision ----------------------------------------------------------

        void checkSweep(Checks &checks)
        {
            // Perpendicular crossing at 1,500 m/s of closure over one 0.01 s
            // step: the paths pass 3 m apart at mid-step while every end
            // point is more than 7 m from the other body.
            const glm::vec3 aStart(-7.5f, 0.0f, 0.0f), aEnd(7.5f, 0.0f, 0.0f);
            const glm::vec3 bStart(0.0f, 3.0f, -7.5f), bEnd(0.0f, 3.0f, 7.5f);
            const sim::SweepResult pass = sim::sweepClosestApproach(aStart, aEnd, bStart, bEnd);
            const float endPointMiss = std::min(glm::length(aEnd - bEnd), glm::length(aStart - bStart));
            checks.expect(std::abs(pass.distance - 3.0f) < 1.0e-4f && std::abs(pass.fraction - 0.5f) < 1.0e-4f && endPointMiss > 5.0f,
                          "sweep: crossing pair found mid-step",
                          format("miss %.4f m at s=%.3f, end points %.1f m apart", pass.distance, pass.fraction, endPointMiss));

            const float entry = sim::sweepEntryFraction(aStart, aEnd, bStart, bEnd, 5.0f);
            const float expected = (7.5f - std::sqrt(8.0f)) / 15.0f;
            const float entryDistance = glm::length(sim::lerpPosition(aStart, aEnd, entry) - sim::lerpPosition(bStart, bEnd, entry));
            checks.expect(std::abs(entry - expected) < 1.0e-4f && std::abs(entryDistance - 5.0f) < 1.0e-3f && entry < pass.fraction,
                          "sweep: fuze fires on entry, before the closest point", format("entry s=%.4f (expected %.4f)", entry, expected));

            // Splitting the step into sub-steps must not change the answer.
            bool stable = true;
            for (int parts : {2, 4, 8, 16})
            {
                float best = 1.0e9f;
                float firstEntry = -1.0f;
                for (int part = 0; part < parts; ++part)
                {
                    const float s0 = static_cast<float>(part) / static_cast<float>(parts);
                    const float s1 = static_cast<float>(part + 1) / static_cast<float>(parts);
                    const glm::vec3 a0 = sim::lerpPosition(aStart, aEnd, s0), a1 = sim::lerpPosition(aStart, aEnd, s1);
                    const glm::vec3 b0 = sim::lerpPosition(bStart, bEnd, s0), b1 = sim::lerpPosition(bStart, bEnd, s1);
                    best = std::min(best, sim::sweepClosestApproach(a0, a1, b0, b1).distance);
                    const float local = sim::sweepEntryFraction(a0, a1, b0, b1, 5.0f);
                    if (firstEntry < 0.0f && local >= 0.0f)
                    {
                        firstEntry = s0 + local * (s1 - s0);
                    }
                }
                stable = stable && std::abs(best - pass.distance) < 1.0e-3f && std::abs(firstEntry - entry) < 1.0e-3f;
            }
            checks.expect(stable, "sweep: sub-stepping gives the same miss and entry");

            // Formation flight: constant separation, no entry unless already inside.
            const glm::vec3 offset(0.0f, 0.0f, 20.0f);
            const float outside = sim::sweepEntryFraction(aStart, aEnd, aStart + offset, aEnd + offset, 10.0f);
            const float inside = sim::sweepEntryFraction(aStart, aEnd, aStart + offset, aEnd + offset, 30.0f);
            const float opening = sim::sweepEntryFraction(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -50.0f),
                                                          glm::vec3(0.0f, 0.0f, 40.0f), glm::vec3(0.0f, 0.0f, 90.0f), 30.0f);
            checks.expect(outside < 0.0f && inside == 0.0f && opening < 0.0f, "sweep: formation and opening pairs");
        }

        // ---- Config validation -----------------------------------------------------------

        void checkConfigValidation(Checks &checks)
        {
            namespace fs = std::filesystem;
            const fs::path directory = fs::temp_directory_path() /
                                       ("missilesim_harness_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
            fs::create_directories(directory);
            const auto write = [&directory](const char *name, const char *text) {
                const fs::path path = directory / name;
                std::ofstream(path) << text;
                return path;
            };

            const sim::SimulationConfigLoadResult future = sim::loadSimulationConfig(write("future.json", R"({"schema_version": 2})"));
            const sim::SimulationConfigLoadResult array = sim::loadSimulationConfig(write("array.json", "[1, 2, 3]"));
            const sim::SimulationConfigLoadResult broken = sim::loadSimulationConfig(write("broken.json", R"({"schema_version": 1,)"));
            const sim::SimulationConfigLoadResult unversioned = sim::loadSimulationConfig(write("unversioned.json", "{}"));
            checks.expect(!future.loaded && future.error.find("schema_version 2") != std::string::npos && !array.loaded &&
                              !broken.loaded && broken.error.find("broken.json") != std::string::npos && !unversioned.loaded,
                          "config: wrong schema, non-object and malformed files are refused", future.error);

            const sim::SimulationConfigLoadResult typo = sim::loadSimulationConfig(write(
                "typo.json", R"({"schema_version": 1, "environment": {"gravity_m_s2": "high", "fixed_time_step_s": 5.0}})"));
            bool namesGravity = false;
            bool namesStep = false;
            for (const std::string &warning : typo.warnings)
            {
                namesGravity = namesGravity || warning.find("gravity_m_s2") != std::string::npos;
                namesStep = namesStep || warning.find("fixed_time_step_s") != std::string::npos;
            }
            checks.expect(typo.loaded && namesGravity && namesStep && typo.config.environment.fixedTimeStep == 0.1f &&
                              typo.config.environment.gravity == sim::EnvironmentConfig{}.gravity,
                          "config: bad fields are reported with the value used",
                          typo.warnings.empty() ? std::string() : typo.warnings.front());

            const sim::SimulationConfigLoadResult shipped = sim::loadDefaultSimulationConfig();
            checks.expect(shipped.loaded && shipped.warnings.empty(), "config: the shipped config loads cleanly",
                          shipped.loaded ? shipped.path.string() : shipped.error);

            std::error_code ignored;
            fs::remove_all(directory, ignored);
        }
    }

    int runEngagementChecks(const sim::SimulationConfig &config)
    {
        Checks checks;
        const std::vector<Scenario> scenarios = standardScenarios();

        checkSweep(checks);
        checkClockPolicy(checks);
        checkConfigValidation(checks);
        checkReproducibleTraces(checks, config, scenarios);
        checkSeedChangesScenario(checks, config, scenarios);
        checkFramePacing(checks, config, scenarios);
        checkSalvoIndependence(checks, config, scenarios);
        checkOutcomesOnceOnly(checks, config, scenarios);
        checkPlayerDamage(checks, config, scenarios);
        checkRemovalMidFlight(checks, config, scenarios);
        checkFlareBirths(checks, config, scenarios);
        checkLaunchRefusals(checks, config);

        std::printf("%d/%d engagement checks passed\n", checks.count - checks.failures, checks.count);
        return checks.failures;
    }
}
