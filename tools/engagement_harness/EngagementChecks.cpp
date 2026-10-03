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
#include "sim/Terrain.h"
#include "sim/Fox3Catalog.h"
#include "sim/defense/Warnings.h"
#include "sim/guidance/RadarHoming.h"
#include "sim/sensors/ScanRadar.h"
#include "sim/sensors/SensorScheduler.h"
#include "sim/sensors/SensorTypes.h"
#include "sim/tracking/TrackStore.h"

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

        // ---- Terrain -------------------------------------------------------------------

        sim::TerrainConfig ridgeTerrain()
        {
            sim::TerrainConfig terrain;
            terrain.kind = sim::TerrainKind::Ridge;
            return terrain;
        }

        sim::TerrainConfig mountainTerrain()
        {
            sim::TerrainConfig terrain;
            terrain.kind = sim::TerrainKind::Mountains;
            return terrain;
        }

        sim::SimulationConfig withTerrain(const sim::SimulationConfig &config, const sim::TerrainConfig &terrain)
        {
            sim::SimulationConfig out = config;
            out.terrain = terrain;
            return out;
        }

        // Height of the mesh triangle containing (x, z), built from the
        // samples and Terrain::kCellTriangles exactly as the renderer builds it.
        bool meshHeight(const sim::Terrain &terrain, float x, float z, float &out)
        {
            const float u = (x - terrain.gridOrigin()) / terrain.cellSize();
            const float w = (z - terrain.gridOrigin()) / terrain.cellSize();
            const int i = std::clamp(static_cast<int>(std::floor(u)), 0, terrain.gridCells() - 1);
            const int j = std::clamp(static_cast<int>(std::floor(w)), 0, terrain.gridCells() - 1);
            const glm::vec2 p(u - static_cast<float>(i), w - static_cast<float>(j));
            for (int triangle = 0; triangle < 2; ++triangle)
            {
                glm::vec2 corner[3];
                float height[3];
                for (int k = 0; k < 3; ++k)
                {
                    const int *offset = sim::Terrain::kCellTriangles[triangle * 3 + k];
                    corner[k] = glm::vec2(static_cast<float>(offset[0]), static_cast<float>(offset[1]));
                    height[k] = terrain.sample(i + offset[0], j + offset[1]);
                }
                const glm::vec2 e1 = corner[1] - corner[0];
                const glm::vec2 e2 = corner[2] - corner[0];
                const glm::vec2 d = p - corner[0];
                const float det = e1.x * e2.y - e1.y * e2.x;
                const float b1 = (d.x * e2.y - d.y * e2.x) / det;
                const float b2 = (e1.x * d.y - e1.y * d.x) / det;
                const float b0 = 1.0f - b1 - b2;
                if (b0 >= -1.0e-5f && b1 >= -1.0e-5f && b2 >= -1.0e-5f)
                {
                    out = b0 * height[0] + b1 * height[1] + b2 * height[2];
                    return true;
                }
            }
            return false;
        }

        void checkMountainsAndWater(Checks &checks)
        {
            const std::shared_ptr<const sim::Terrain> shared = sim::Terrain::shared(mountainTerrain());
            const sim::Terrain &terrain = *shared;
            const sim::TerrainConfig &shape = terrain.config();

            sim::RandomStream points = sim::RandomStreams(78).stream("terrain.mountains");
            const float half = -terrain.gridOrigin();
            float worstMesh = 0.0f;
            bool allInside = true;
            for (int index = 0; index < 20000; ++index)
            {
                const float x = points.uniform(-half, half);
                const float z = points.uniform(-half, half);
                float mesh = 0.0f;
                allInside = meshHeight(terrain, x, z, mesh) && allInside;
                worstMesh = std::max(worstMesh, std::abs(mesh - terrain.bedHeightAt(x, z)));
            }
            checks.expect(allInside && worstMesh < 1.0e-2f && sim::Terrain::shared(mountainTerrain()) == shared,
                          "terrain: mountain land is the drawn land, built once",
                          format("worst %.4f m, peaks %.0f m", worstMesh, terrain.maxHeight() - shape.baseHeightM));

            // The launch site is flat; the lake and the sea are solid water
            // over a bed below it; a body dropping onto the water stops at it.
            const float lakeX = 2300.0f;
            const float lakeZ = -2100.0f;
            const float sea = half + 4000.0f;
            const bool siteFlat = terrain.heightAt(0.0f, 0.0f) == shape.baseHeightM &&
                                  terrain.heightAt(shape.apronRadiusM * 0.9f, 0.0f) == shape.baseHeightM;
            const bool lake = terrain.isWater(lakeX, lakeZ) && terrain.heightAt(lakeX, lakeZ) == terrain.waterLevel() &&
                              terrain.bedHeightAt(lakeX, lakeZ) < terrain.waterLevel() - 5.0f;
            const bool open = terrain.isWater(sea, 0.0f) && terrain.heightAt(sea, 0.0f) == terrain.waterLevel();
            float fraction = 0.0f;
            const glm::vec3 above(lakeX, terrain.waterLevel() + 50.0f, lakeZ);
            const glm::vec3 below(lakeX + 3.0f, terrain.bedHeightAt(lakeX, lakeZ) - 1.0f, lakeZ);
            const bool hit = terrain.segmentHit(above, below, &fraction);
            const float stop = (above + (below - above) * fraction).y - terrain.waterLevel();
            checks.expect(siteFlat && lake && open && hit && std::abs(stop) < 1.0e-3f,
                          "terrain: flat launch site, solid lake and sea",
                          format("water %.1f m, lake bed %.1f m, stop %.4f m", terrain.waterLevel(),
                                 terrain.bedHeightAt(lakeX, lakeZ), stop));
        }

        void checkTerrainSurface(Checks &checks)
        {
            const sim::Terrain terrain(ridgeTerrain());
            const sim::TerrainConfig &shape = terrain.config();

            // Seeded points over the whole grid, plus every sample point of a band.
            sim::RandomStream points = sim::RandomStreams(77).stream("terrain.points");
            const float half = -terrain.gridOrigin();
            float worstMesh = 0.0f;
            bool allInside = true;
            for (int index = 0; index < 20000; ++index)
            {
                const float x = points.uniform(-half, half);
                const float z = points.uniform(-half, half);
                float mesh = 0.0f;
                allInside = meshHeight(terrain, x, z, mesh) && allInside;
                worstMesh = std::max(worstMesh, std::abs(mesh - terrain.heightAt(x, z)));
            }
            float worstSample = 0.0f;
            for (int i = 0; i <= terrain.gridCells(); i += 7)
            {
                const float x = terrain.gridOrigin() + static_cast<float>(i) * terrain.cellSize();
                const float z = terrain.gridOrigin() + static_cast<float>(terrain.gridCells() / 2) * terrain.cellSize();
                worstSample = std::max(worstSample, std::abs(terrain.heightAt(x, z) - terrain.sample(i, terrain.gridCells() / 2)));
            }
            checks.expect(allInside && worstMesh < 1.0e-3f && worstSample < 1.0e-3f,
                          "terrain: the drawn triangles are the collision ones",
                          format("worst %.6f m over 20000 points, %.6f m at samples", worstMesh, worstSample));

            const float apron = terrain.heightAt(0.0f, 0.0f);
            const float apronEdge = terrain.heightAt(shape.apronRadiusM * 0.95f, 0.0f);
            const float beyond = terrain.heightAt(half + 50.0f, 300.0f);
            const float justInside = terrain.heightAt(half - 1.0f, 300.0f);
            const float crest = terrain.heightAt(0.0f, shape.ridgeDistanceM);
            checks.expect(apron == shape.baseHeightM && std::abs(apronEdge - shape.baseHeightM) < 0.5f &&
                              beyond == shape.baseHeightM && std::abs(justInside - shape.baseHeightM) < 0.5f &&
                              crest > shape.baseHeightM + 0.5f * shape.ridgeHeightM && terrain.maxHeight() >= crest,
                          "terrain: flat apron and edge, ridge beyond",
                          format("crest %.1f m, max %.1f m, edge %.3f m", crest, terrain.maxHeight(), justInside));

            // A diving path across the slope: the contact is on the surface,
            // and splitting the path into steps finds the same point.
            const glm::vec3 a(-120.0f, terrain.maxHeight() + 40.0f, shape.ridgeDistanceM - 900.0f);
            const glm::vec3 b(260.0f, shape.baseHeightM - 5.0f, shape.ridgeDistanceM + 200.0f);
            float whole = 0.0f;
            const bool hit = terrain.segmentHit(a, b, &whole);
            const glm::vec3 point = a + (b - a) * whole;
            float split = -1.0f;
            const int pieces = 37;
            for (int piece = 0; piece < pieces && split < 0.0f; ++piece)
            {
                const float t0 = static_cast<float>(piece) / pieces;
                const float t1 = static_cast<float>(piece + 1) / pieces;
                float local = 0.0f;
                if (terrain.segmentHit(a + (b - a) * t0, a + (b - a) * t1, &local))
                {
                    split = t0 + (t1 - t0) * local;
                }
            }
            const float onSurface = std::abs(terrain.heightAbove(point));
            const float stepGap = glm::length((b - a) * (split - whole));
            checks.expect(hit && onSurface < 0.01f && split >= 0.0f && stepGap < 0.05f,
                          "terrain: swept contact is on the surface, any steps",
                          format("off surface %.4f m, step gap %.4f m", onSurface, stepGap));

            const glm::vec3 site(0.0f, shape.baseHeightM + 100.0f, 0.0f);
            const glm::vec3 farSide(0.0f, shape.baseHeightM + 100.0f, shape.ridgeDistanceM + 2000.0f);
            const glm::vec3 siteHigh(0.0f, terrain.maxHeight() + 20.0f, 0.0f);
            const glm::vec3 farHigh(0.0f, terrain.maxHeight() + 20.0f, shape.ridgeDistanceM + 2000.0f);
            const sim::Terrain flat;
            checks.expect(!terrain.lineOfSight(site, farSide) && terrain.lineOfSight(siteHigh, farHigh) &&
                              flat.lineOfSight(site, farSide),
                          "terrain: the ridge blocks line of sight below its crest");
        }

        void checkTerrainContacts(Checks &checks, const sim::SimulationConfig &config)
        {
            const sim::TerrainConfig shape = ridgeTerrain();

            // An unguided body fired level at the ridge face ends on that face,
            // at the same point whatever the fixed step.
            auto fireAtRidge = [&](float fixedStep, glm::vec3 &impact) {
                sim::SimulationConfig ridge = withTerrain(config, shape);
                ridge.environment.fixedTimeStep = fixedStep;
                Scenario scenario;
                scenario.name = "terrain-impact";
                scenario.seed = 707;
                scenario.durationSeconds = 6.0;
                scenario.setup = [](World &world) { world.respawnTargets(1); };
                scenario.beforeStep = [&shape](World &world, std::uint64_t tick) {
                    if (tick != 1)
                    {
                        return;
                    }
                    sim::ScriptedLaunch launch;
                    launch.team = sim::Team::Red;
                    launch.position = glm::vec3(0.0f, shape.baseHeightM + 200.0f, 1200.0f);
                    launch.velocity = glm::vec3(0.0f, 0.0f, 400.0f);
                    world.launchScripted(launch);
                };
                bool ended = false;
                runScenario(ridge, scenario, [&](World &, const std::vector<SimEvent> &events) {
                    for (const SimEvent &event : events)
                    {
                        if (!ended && event.type == EventType::ShotEnded &&
                            event.detail == static_cast<std::uint8_t>(sim::ShotEndReason::GroundImpact))
                        {
                            impact = event.position;
                            ended = true;
                        }
                    }
                });
                return ended;
            };
            glm::vec3 coarse(0.0f);
            glm::vec3 fine(0.0f);
            const bool coarseHit = fireAtRidge(0.01f, coarse);
            const bool fineHit = fireAtRidge(0.004f, fine);
            const sim::Terrain terrain(shape);
            const float offSurface = std::abs(terrain.heightAbove(coarse));
            checks.expect(coarseHit && fineHit && offSurface < 0.05f && coarse.y > shape.baseHeightM + 100.0f &&
                              coarse.z > shape.ridgeDistanceM - shape.ridgeHalfWidthM && coarse.z < shape.ridgeDistanceM &&
                              glm::length(coarse - fine) < 2.0f,
                          "terrain: a round meets the ridge face, any step",
                          format("hit at %.1f m up, z %.1f m, step gap %.2f m", coarse.y - shape.baseHeightM, coarse.z,
                                 glm::length(coarse - fine)));

            // The fighter flies level into the slope: it crashes on the face,
            // not at the datum underneath.
            Scenario crash;
            crash.name = "terrain-crash";
            crash.seed = 708;
            crash.durationSeconds = 12.0;
            crash.setup = [&shape](World &world) {
                world.respawnTargets(1);
                world.setRoleFighter("aim-9x-blk2");
                Fighter *fighter = world.fighter();
                if (fighter == nullptr)
                {
                    return;
                }
                const glm::vec3 nose(0.0f, 0.0f, 1.0f);
                fighter->place(glm::vec3(0.0f, shape.baseHeightM + 150.0f, 900.0f), nose * 200.0f, nose);
                missilesim::flight::InstructorInput input;
                input.aimDirection = nose;
                fighter->setInstructorInput(input, 0.0f);
            };
            sim::EntityId fighterId;
            glm::vec3 crashPoint(0.0f);
            bool crashed = false;
            runScenario(withTerrain(config, shape), crash, [&](World &world, const std::vector<SimEvent> &events) {
                fighterId = world.fighterId();
                for (const SimEvent &event : events)
                {
                    if (!crashed && event.type == EventType::GroundCollision && event.subject == fighterId)
                    {
                        crashPoint = event.position;
                        crashed = true;
                    }
                }
            });
            checks.expect(crashed && crashPoint.y > shape.baseHeightM + 60.0f &&
                              std::abs(terrain.heightAbove(crashPoint)) <= sim::rules::kFighterGroundClearanceM + 0.5f,
                          "terrain: the fighter crashes on the ridge face",
                          format("%.1f m up at z %.1f m, %.2f m off the surface", crashPoint.y - shape.baseHeightM, crashPoint.z,
                                 terrain.heightAbove(crashPoint)));

            // Aircraft keep their height band over the ground under them.
            sim::TerrainConfig near = shape;
            near.ridgeDistanceM = 1800.0f;
            Scenario patrol;
            patrol.name = "terrain-patrol";
            patrol.seed = 709;
            patrol.durationSeconds = 60.0;
            patrol.setup = [](World &world) { world.respawnTargets(8); };
            float lowest = 1.0e9f;
            float highestGround = near.baseHeightM;
            const sim::Terrain nearTerrain(near);
            runScenario(withTerrain(config, near), patrol, [&](World &world, const std::vector<SimEvent> &) {
                for (const auto &target : world.targets())
                {
                    if (target->isActive())
                    {
                        lowest = std::min(lowest, nearTerrain.heightAbove(target->getPosition()));
                        highestGround = std::max(highestGround, nearTerrain.heightAt(target->getPosition()));
                    }
                }
            });
            checks.expect(lowest >= 30.0f && highestGround > near.baseHeightM + 100.0f,
                          "terrain: aircraft keep their band over the ridge",
                          format("lowest %.1f m above ground, highest ground flown over %.1f m", lowest,
                                 highestGround - near.baseHeightM));
        }

        void checkTerrainConfig(Checks &checks)
        {
            namespace fs = std::filesystem;
            const fs::path directory = fs::temp_directory_path() /
                                       ("missilesim_terrain_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
            fs::create_directories(directory);
            const fs::path ridgePath = directory / "ridge.json";
            const fs::path typoPath = directory / "typo.json";
            std::ofstream(ridgePath) << R"({"schema_version": 1, "terrain": {"kind": "ridge", "ridge_height_m": 400.0}})";
            std::ofstream(typoPath) << R"({"schema_version": 1, "terrain": {"kind": "glaciers", "cell_size_m": 1.0}})";
            const sim::SimulationConfigLoadResult ridge = sim::loadSimulationConfig(ridgePath);
            const sim::SimulationConfigLoadResult typo = sim::loadSimulationConfig(typoPath);
            bool namesKind = false;
            bool namesCell = false;
            for (const std::string &warning : typo.warnings)
            {
                namesKind = namesKind || warning.find("glaciers") != std::string::npos;
                namesCell = namesCell || warning.find("cell_size_m") != std::string::npos;
            }
            checks.expect(ridge.loaded && ridge.warnings.empty() && ridge.config.terrain.kind == sim::TerrainKind::Ridge &&
                              ridge.config.terrain.ridgeHeightM == 400.0f && typo.loaded && namesKind && namesCell &&
                              typo.config.terrain.kind == sim::TerrainConfig{}.kind && typo.config.terrain.cellSizeM == 16.0f,
                          "config: terrain kind and ranges are checked",
                          typo.warnings.empty() ? std::string() : typo.warnings.front());
            std::error_code ignored;
            fs::remove_all(directory, ignored);
        }

        // ---- Sensors --------------------------------------------------------------------

        sim::RadarSet referenceRadar()
        {
            sim::RadarSet radar;
            radar.peakPowerW = 10000.0f;
            radar.gainTransmit = 1000.0f;
            radar.gainReceive = 1000.0f;
            radar.wavelengthM = 0.03f;
            radar.pulseWidthS = 1.0e-6f;
            radar.systemTemperatureK = 290.0f;
            radar.systemLoss = 1.0f;
            radar.snrThreshold = 10.0f;
            radar.revisitS = 0.5;
            return radar;
        }

        sim::SensorBody ownshipAtOrigin()
        {
            sim::SensorBody ownship;
            ownship.id = sim::EntityId{1};
            ownship.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            ownship.up = glm::vec3(0.0f, 1.0f, 0.0f);
            return ownship;
        }

        void checkSensorModel(Checks &checks)
        {
            const glm::vec3 origin(0.0f);
            const glm::vec3 nose(0.0f, 0.0f, 1.0f);
            const float ahead = sim::aspectFromNoseRad(origin, nose, glm::vec3(0.0f, 0.0f, 1000.0f));
            const float abeam = sim::aspectFromNoseRad(origin, nose, glm::vec3(1000.0f, 0.0f, 0.0f));
            const float astern = sim::aspectFromNoseRad(origin, nose, glm::vec3(0.0f, 0.0f, -1000.0f));
            sim::RadarCrossSectionProfile shape;
            shape.noseM2 = 1.0f;
            shape.beamM2 = 10.0f;
            shape.tailM2 = 4.0f;
            checks.expect(ahead < 1.0e-4f && std::abs(abeam - 1.5707963f) < 1.0e-4f && std::abs(astern - 3.14159265f) < 1.0e-3f &&
                              sim::meanRadarCrossSectionM2(shape, ahead) == 1.0f &&
                              std::abs(sim::meanRadarCrossSectionM2(shape, abeam) - 10.0f) < 1.0e-4f &&
                              std::abs(sim::meanRadarCrossSectionM2(shape, astern) - 4.0f) < 1.0e-4f,
                          "aspect: the nose sets the radar cross section",
                          "ahead 1 m2, abeam 10 m2, astern 4 m2");

            sim::RadarSet radar = referenceRadar();
            sim::RadarCrossSectionProfile unit;
            unit.noseM2 = unit.beamM2 = unit.tailM2 = 1.0f;
            const double snr10km = sim::monostaticSnr(radar, 1.0f, 10000.0f);
            const double snr20km = sim::monostaticSnr(radar, 1.0f, 20000.0f);
            const double snrTwiceRcs = sim::monostaticSnr(radar, 2.0f, 10000.0f);
            radar.systemLoss = 2.0f;
            const double snrTwiceLoss = sim::monostaticSnr(radar, 1.0f, 10000.0f);
            radar.systemLoss = 1.0f;
            // Wavelength is a float, so the absolute SNR sits a few parts in 1e8
            // under the same expression evaluated in pure double. The ratio
            // checks below are exact.
            checks.expect(std::abs(snr10km - 113.274365138491) / 113.274365138491 < 1.0e-6, "radar: reference SNR at 10 km is 113.27",
                          format("SNR %.6f", snr10km));
            checks.expect(std::abs(snr10km / snr20km - 16.0) < 1.0e-9, "radar: doubling range divides SNR by 16",
                          format("ratio %.6f", snr10km / snr20km));
            checks.expect(std::abs(snrTwiceRcs / snr10km - 2.0) < 1.0e-9 && std::abs(snr10km / snrTwiceLoss - 2.0) < 1.0e-9,
                          "radar: RCS and system loss scale SNR once each");

            sim::InfraredSignatureProfile infraredShape;
            infraredShape.skinWPerSr = 20.0f;
            infraredShape.tailpipeWPerSr = 80.0f;
            infraredShape.plumeBeamWPerSr = 40.0f;
            infraredShape.afterburnerScale = 3.0f;
            const float noseIdle = sim::infraredIntensityWPerSr(infraredShape, ahead, 0.0f, false);
            const float noseReheat = sim::infraredIntensityWPerSr(infraredShape, ahead, 1.0f, true);
            const float tailMilitary = sim::infraredIntensityWPerSr(infraredShape, astern, 1.0f, false);
            const float tailReheat = sim::infraredIntensityWPerSr(infraredShape, astern, 0.2f, true);
            const float beamMilitary = sim::infraredIntensityWPerSr(infraredShape, abeam, 1.0f, false);
            const double nearIrradiance = sim::infraredIrradianceWPerM2(tailMilitary, 1000.0f);
            const double farIrradiance = sim::infraredIrradianceWPerM2(tailMilitary, 2000.0f);
            checks.expect(std::abs(noseIdle - 20.0f) < 1.0e-3f && std::abs(noseReheat - 20.0f) < 1.0e-3f,
                          "infrared: the nose stays skin in afterburner");
            checks.expect(std::abs(tailMilitary - 100.0f) < 1.0e-3f && std::abs(tailReheat - 260.0f) < 1.0e-2f &&
                              std::abs(beamMilitary - 60.0f) < 1.0e-3f,
                          "infrared: throttle and reheat move only the hot lobes",
                          format("tail %.0f, reheat %.0f, beam %.0f", tailMilitary, tailReheat, beamMilitary));
            checks.expect(std::abs(nearIrradiance - 1.0e-4) < 1.0e-12 && std::abs(nearIrradiance / farIrradiance - 4.0) < 1.0e-9,
                          "infrared: irradiance falls with range squared");

            // The ridge masks the low path and leaves the high path clear. The
            // sensor uses that same line of sight; it has no second terrain.
            sim::TerrainConfig ridgeConfig;
            ridgeConfig.kind = sim::TerrainKind::Ridge;
            ridgeConfig.baseHeightM = 0.0f;
            ridgeConfig.ridgeHeightM = 400.0f;
            ridgeConfig.ridgeDistanceM = 2000.0f;
            ridgeConfig.ridgeHalfWidthM = 600.0f;
            ridgeConfig.ridgeBearingDeg = 0.0f;
            ridgeConfig.hillAmplitudeM = 0.0f;
            const sim::Terrain ridge(ridgeConfig);
            sim::TerrainConfig flatConfig;
            flatConfig.kind = sim::TerrainKind::Flat;
            const sim::Terrain flat(flatConfig);

            sim::SensorBody ownship = ownshipAtOrigin();
            ownship.position = glm::vec3(0.0f, 50.0f, 0.0f);
            sim::SensorBody contact;
            contact.id = sim::EntityId{42};
            contact.position = glm::vec3(0.0f, 50.0f, 4000.0f);
            contact.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            const bool lowBlocked = !ridge.lineOfSight(ownship.position, contact.position);

            sim::SensorScheduler masked;
            masked.setRadar(referenceRadar(), unit);
            const sim::SensorProducts lowLook = masked.update(0.0, ridge, ownship, {contact});
            const bool lowMasked = lowLook.observations.empty() && lowLook.debug.size() == 1 &&
                                   lowLook.debug.front().reason == sim::LookFail::Masked &&
                                   lowLook.debug.front().truthPlatform == contact.id;

            ownship.position.y = 900.0f;
            contact.position.y = 900.0f;
            const bool highClear = ridge.lineOfSight(ownship.position, contact.position);
            sim::SensorScheduler clear;
            clear.setRadar(referenceRadar(), unit);
            const sim::SensorProducts highLook = clear.update(0.0, ridge, ownship, {contact});
            const bool highDetected = highLook.observations.size() == 1 && highLook.debug.size() == 1 &&
                                      highLook.debug.front().detected &&
                                      std::abs(highLook.observations.front().azimuthRad) < 1.0e-4f &&
                                      std::abs(highLook.observations.front().elevationRad) < 1.0e-4f;
            checks.expect(lowBlocked && lowMasked && highClear && highDetected,
                          "radar: a ridge masks the path the terrain blocks",
                          format("low blocked %.0f, high clear %.0f", lowBlocked ? 1.0 : 0.0, highClear ? 1.0 : 0.0));

            // Timing is the sample clock. Between grid times nothing is emitted
            // and tracks are not aged. A jump that crosses two grid times still
            // looks once, at the pose it was given.
            ownship = ownshipAtOrigin();
            contact.position = glm::vec3(0.0f, 0.0f, 1000.0f);
            contact.forward = glm::vec3(0.0f, 0.0f, -1.0f); // nose toward the shooter; the cross section is uniform
            sim::SensorScheduler grid;
            grid.setRadar(referenceRadar(), unit);
            int looks = 0;
            bool gapEmitted = false;
            const double samples[] = {0.0, 0.1, 0.49, 0.5};
            for (double sample : samples)
            {
                const sim::SensorProducts products = grid.update(sample, flat, ownship, {contact});
                if (sample == 0.1 || sample == 0.49)
                {
                    gapEmitted = gapEmitted || !products.observations.empty();
                }
                else
                {
                    looks += products.observations.empty() ? 0 : 1;
                }
            }
            sim::SensorScheduler jumped;
            jumped.setRadar(referenceRadar(), unit);
            jumped.update(0.0, flat, ownship, {contact});
            const sim::SensorProducts late = jumped.update(1.2, flat, ownship, {contact});
            checks.expect(!gapEmitted && looks == 2 && late.observations.size() == 1 && late.droppedLooks == 1 &&
                              late.observations.front().time == 1.2 && late.observations.front().sensor == ownship.id &&
                              jumped.radarTracks().tracks().size() == 1 &&
                              jumped.radarTracks().tracks().front().hitCount == 2,
                          "scheduler: looks land on the grid, a late sample once",
                          format("%.0f on-grid looks, %.0f dropped", static_cast<double>(looks),
                                 static_cast<double>(late.droppedLooks)));

            sim::SensorScheduler quiet;
            quiet.setRadar(referenceRadar(), unit);
            quiet.radarTracks().setCoastLimit(2.0);
            quiet.update(0.0, flat, ownship, {contact});
            const sim::SensorProducts between = quiet.update(0.25, flat, ownship, {contact});
            const bool held = between.observations.empty() && quiet.radarTracks().tracks().size() == 1 &&
                              quiet.radarTracks().tracks().front().life == sim::TrackLife::Tentative &&
                              quiet.radarTracks().tracks().front().lastMeasurementTime == 0.0;
            const sim::SensorProducts missed = quiet.update(0.5, flat, ownship, {});
            const bool coasted = missed.observations.empty() && missed.debug.empty() &&
                                 quiet.radarTracks().tracks().size() == 1 &&
                                 quiet.radarTracks().tracks().front().life == sim::TrackLife::Coasting &&
                                 quiet.radarTracks().tracks().front().hitCount == 1;
            checks.expect(held && coasted, "scheduler: a quiet look coasts, a gap does not");

            // Infrared power does not follow the radar set, and a second run of
            // the same samples repeats. Nothing in the call is a camera.
            sim::InfraredSet eye;
            eye.neiWPerM2 = 1.0e-8f;
            eye.snrMin = 5.0f;
            eye.revisitS = 0.5;
            contact.throttle = 1.0f;
            contact.afterburner = false;
            contact.forward = glm::vec3(0.0f, 0.0f, 1.0f); // nose downrange, so the shooter sees the tail
            sim::SensorScheduler firstEye;
            sim::SensorScheduler secondEye;
            sim::RadarSet louder = referenceRadar();
            louder.peakPowerW = 80000.0f;
            firstEye.setRadar(referenceRadar(), unit);
            secondEye.setRadar(louder, unit);
            firstEye.setInfrared(eye, infraredShape);
            secondEye.setInfrared(eye, infraredShape);
            const sim::SensorProducts firstProducts = firstEye.update(0.0, flat, ownship, {contact});
            const sim::SensorProducts secondProducts = secondEye.update(0.0, flat, ownship, {contact});
            const sim::Observation *firstInfrared = nullptr;
            const sim::Observation *secondInfrared = nullptr;
            for (const sim::Observation &observation : firstProducts.observations)
            {
                if (observation.family == sim::SensorFamily::Infrared)
                {
                    firstInfrared = &observation;
                }
            }
            for (const sim::Observation &observation : secondProducts.observations)
            {
                if (observation.family == sim::SensorFamily::Infrared)
                {
                    secondInfrared = &observation;
                }
            }
            checks.expect(firstInfrared != nullptr && secondInfrared != nullptr &&
                              firstInfrared->irradianceWPerM2 == secondInfrared->irradianceWPerM2 &&
                              std::abs(firstInfrared->irradianceWPerM2 - 1.0e-4) < 1.0e-8,
                          "infrared: radar power does not change irradiance");

            // The track is built from the measurements. Platform 42 is on the
            // debug look only; the coasted point follows the measured step,
            // not a later true position the store was never given.
            sim::TrackStore tracks;
            tracks.setConfirmHits(2);
            tracks.setCoastLimit(1.5);
            tracks.setGateRadius(1000.0f);
            sim::Observation firstMeasurement;
            firstMeasurement.position = glm::vec3(0.0f, 1000.0f, 0.0f);
            firstMeasurement.time = 0.0;
            sim::Observation secondMeasurement = firstMeasurement;
            secondMeasurement.position = glm::vec3(200.0f, 1000.0f, 0.0f);
            secondMeasurement.time = 1.0;
            tracks.onLook(0.0, {firstMeasurement});
            const bool tentative = tracks.tracks().size() == 1 && tracks.tracks().front().life == sim::TrackLife::Tentative &&
                                   tracks.tracks().front().id == sim::EntityId{1} && tracks.tracks().front().id != contact.id;
            tracks.onLook(1.0, {secondMeasurement});
            const sim::TrackEstimate confirmed = tracks.tracks().front();
            tracks.onLook(2.0, {});
            const sim::TrackEstimate coasting = tracks.tracks().front();
            const glm::vec3 unreadTruth(400.0f, 1000.0f, 500.0f);
            const bool predicted = coasting.life == sim::TrackLife::Coasting &&
                                   glm::length(coasting.position - glm::vec3(400.0f, 1000.0f, 0.0f)) < 1.0e-3f &&
                                   glm::length(coasting.position - unreadTruth) > 100.0f;
            sim::Observation refresh = secondMeasurement;
            refresh.position = glm::vec3(450.0f, 1000.0f, 20.0f);
            refresh.time = 2.5;
            tracks.onLook(2.5, {refresh});
            const sim::TrackEstimate refreshed = tracks.tracks().front();
            tracks.onLook(4.5, {});
            const bool lost = tracks.tracks().front().life == sim::TrackLife::Lost;
            checks.expect(tentative && confirmed.life == sim::TrackLife::Confirmed && confirmed.hitCount == 2 &&
                              glm::length(confirmed.velocity - glm::vec3(200.0f, 0.0f, 0.0f)) < 1.0e-3f && predicted &&
                              refreshed.life == sim::TrackLife::Confirmed &&
                              glm::length(refreshed.position - refresh.position) < 1.0e-3f && lost,
                          "tracks: measurements confirm, coast, refresh, and end",
                          "platform 42 is not the track id");
        }

        void checkRadarEngagement(Checks &checks, const sim::SimulationConfig &config)
        {
            using sim::EntityId;
            using sim::FighterWeapon;
            using sim::LaunchBlock;
            using sim::LaunchResult;
            using sim::PlatformSnapshot;
            using sim::ScriptedLaunch;
            using sim::Shot;
            using sim::Team;
            using sim::TerrainConfig;
            using sim::TerrainKind;
            using sim::TrackEstimate;
            using sim::TrackLife;
            using sim::Warning;
            using sim::WarningKind;

            TerrainConfig flatConfig;
            flatConfig.kind = TerrainKind::Flat;

            World world(config, 101);
            world.setTraceRecording(true);
            world.setTerrain(flatConfig);
            world.setRoleFighter("aim-9x-blk2");
            const EntityId targetId = world.spawnTarget(glm::vec3(0.0f, 3000.0f, 8000.0f), 8.0f);
            world.placeFighterAtEngagement();
            world.setFighterWeapon(FighterWeapon::RadarRound);

            bool designated = false;
            LaunchResult fired;
            for (int step = 0; step < 800 && !fired.launched(); ++step)
            {
                world.step();
                if (!designated)
                {
                    for (const TrackEstimate &track : world.playerRadar().tracks().tracks())
                    {
                        if (track.life != TrackLife::Lost)
                        {
                            world.cycleRadarDesignation();
                            designated = true;
                            break;
                        }
                    }
                }
                if (designated && world.launchClearance() == LaunchBlock::None)
                {
                    fired = world.launch(glm::vec3(0.0f, 0.0f, 1.0f));
                }
            }

            const Shot *shot = world.findShot(fired.shot);
            const bool supported = shot != nullptr && shot->radarGuided && shot->supportTrack.valid() && shot->supportTrack != targetId &&
                                   shot->missile != nullptr && shot->missile->getTargetObject() == nullptr;
            bool announced = false;
            for (const SimEvent &event : world.trace())
            {
                if (event.type == EventType::WeaponLaunched && event.subject == fired.shot && event.other != targetId)
                {
                    announced = true;
                }
            }
            checks.expect(fired.launched() && supported && announced, "radar: track launch does not carry a platform id");

            bool teleported = false;
            bool gainedTarget = false;
            if (shot != nullptr && shot->missile != nullptr)
            {
                world.removeTarget(targetId);
                glm::vec3 previous = shot->missile->getPosition();
                for (int step = 0; step < 2000; ++step)
                {
                    world.step();
                    const Shot *flying = world.findShot(fired.shot);
                    if (flying == nullptr || flying->missile == nullptr)
                    {
                        shot = nullptr;
                        break;
                    }
                    if (flying->missile->getTargetObject() != nullptr)
                    {
                        gainedTarget = true;
                    }
                    const float jump = glm::length(flying->missile->getPosition() - previous);
                    if (jump > 150.0f)
                    {
                        teleported = true;
                    }
                    previous = flying->missile->getPosition();
                    shot = flying;
                }
            }
            checks.expect(fired.launched() && !teleported && !gainedTarget && shot == nullptr,
                          "radar: a removed target does not teleport the round, and the shot ends");

            // Pulse-Doppler must not cost a clean shot: a target that is not
            // beaming stays in the seeker's velocity gate all the way in.
            {
                World clean(config, 700);
                clean.setTerrain(flatConfig);
                clean.setRoleFighter("aim-9x-blk2");
                const EntityId bandit = clean.spawnTarget(glm::vec3(0.0f, 3000.0f, 8000.0f), 8.0f);
                clean.placeFighterAtEngagement();
                clean.setFighterWeapon(FighterWeapon::RadarRound);
                LaunchResult cleanShot;
                bool locked = false;
                for (int step = 0; step < 800 && !cleanShot.launched(); ++step)
                {
                    clean.step();
                    if (!locked && !clean.playerRadar().tracks().tracks().empty())
                    {
                        clean.cycleRadarDesignation();
                        locked = true;
                    }
                    if (locked && clean.launchClearance() == LaunchBlock::None)
                    {
                        cleanShot = clean.launch(glm::vec3(0.0f, 0.0f, 1.0f));
                    }
                }
                bool droppedTrack = false;
                for (int step = 0; step < 3000; ++step)
                {
                    clean.step();
                    const Shot *flying = clean.findShot(cleanShot.shot);
                    if (flying == nullptr)
                    {
                        break;
                    }
                    droppedTrack = droppedTrack || flying->homing.state().phase == sim::RadarHomingState::Phase::Memory;
                }
                checks.expect(cleanShot.launched() && !droppedTrack && !clean.targetAlive(bandit),
                              "radar: the seeker holds a target that is not beaming to the kill");
            }

            const int spentMagazine = world.radarRoundsRemaining();
            world.rearm();
            checks.expect(fired.launched() && spentMagazine < 4 && world.radarRoundsRemaining() == 4,
                          "radar: rearm refills the radar magazine");

            // Search alone: the dwells that point elsewhere must not coast the
            // contact between its own dwells.
            {
                World search(config, 17);
                search.setTerrain(flatConfig);
                search.setRoleFighter("aim-9x-blk2");
                const EntityId ahead = search.spawnTarget(glm::vec3(0.0f, 3000.0f, 8000.0f), 8.0f);
                search.placeFighterAtEngagement();
                if (Target *aircraft = search.findTarget(ahead))
                {
                    aircraft->setVelocity(glm::vec3(0.0f, 0.0f, -200.0f));
                }
                bool confirmedOnce = false;
                int flickers = 0;
                for (int step = 0; step < 300; ++step)
                {
                    search.step();
                    for (const TrackEstimate &track : search.playerRadar().tracks().tracks())
                    {
                        if (track.life == TrackLife::Confirmed)
                        {
                            confirmedOnce = true;
                        }
                        else if (confirmedOnce)
                        {
                            ++flickers;
                        }
                    }
                }
                checks.expect(confirmedOnce && flickers == 0 && !search.playerRadar().singleTargetTrack(),
                              "radar: a searched contact stays confirmed between its dwells",
                              format("%.0f non-confirmed samples", static_cast<double>(flickers)));
            }

            // Opponent radar pointed away: nothing reaches the receiver.
            {
                World away(config, 23);
                away.setTerrain(flatConfig);
                away.setRoleFighter("aim-9x-blk2");
                const EntityId leader = away.spawnTarget(glm::vec3(0.0f, 3000.0f, 6000.0f), 8.0f);
                away.placeFighterAtEngagement();
                if (Target *aircraft = away.findTarget(leader))
                {
                    aircraft->setVelocity(glm::vec3(0.0f, 0.0f, 220.0f));
                }
                away.setRedRadar(true);
                bool heard = false;
                for (int step = 0; step < 100; ++step)
                {
                    away.step();
                    heard = heard || !away.warnings().radar.empty();
                }
                checks.expect(!heard, "radar: a radar looking away is not heard");
            }

            // Two-ship: the lead's radar does not track its own wingman. The
            // wingman sits about 20 deg off the lead's left, in the raster's
            // first column, so without IFF it would be painted, and held,
            // before the player. Targets are kept inside the patrol airspace.
            {
                World pair(config, 29);
                pair.setTerrain(flatConfig);
                pair.setRoleFighter("aim-9x-blk2");
                const EntityId leader = pair.spawnTarget(glm::vec3(0.0f, 600.0f, 2000.0f), 8.0f);
                const EntityId wingman = pair.spawnTarget(glm::vec3(-250.0f, 600.0f, 1300.0f), 8.0f);
                pair.placeFighterAtEngagement();
                for (const EntityId id : {leader, wingman})
                {
                    if (Target *aircraft = pair.findTarget(id))
                    {
                        aircraft->setVelocity(glm::vec3(0.0f, 0.0f, -200.0f));
                    }
                }
                pair.setRedRadar(true);
                bool trackedPlayer = false;
                bool trackedWingman = false;
                for (int step = 0; step < 150; ++step)
                {
                    pair.step();
                    PlatformSnapshot wing;
                    PlatformSnapshot player;
                    if (!pair.snapshot(wingman, wing) || !pair.snapshot(pair.fighterId(), player))
                    {
                        continue;
                    }
                    // The two close head-on, so a track belongs to whichever body it is nearer.
                    for (const TrackEstimate &track : pair.hostileRadar().tracks().tracks())
                    {
                        const float toWingman = glm::length(track.position - wing.position);
                        const float toPlayer = glm::length(track.position - player.position);
                        trackedWingman = trackedWingman || (toWingman < toPlayer && toWingman < 500.0f);
                        trackedPlayer = trackedPlayer || (toPlayer <= toWingman && toPlayer < 500.0f);
                    }
                }
                checks.expect(trackedPlayer && !trackedWingman, "radar: the opponent's radar leaves its wingman out");
            }

            // One lock for both weapons. T takes the contact nearest the nose
            // first; the heat seeker on the rail, never uncaged, then follows
            // the lock to an aircraft off the nose instead of the one ahead.
            // The patrol airspace is widened, and the two are placed after
            // spawning (a spawn is pulled into the default airspace), so they
            // stay outside one association gate of each other.
            {
                World slave(config, 31);
                slave.setTerrain(flatConfig);
                TargetAIConfig wide = slave.targetAIConfig();
                wide.preferredDistance = 6000.0f;
                slave.setTargetAIConfig(wide);
                slave.setRoleFighter("aim-9x-blk2");
                const float offNose = glm::radians(28.0f);
                const EntityId ahead = slave.spawnTarget(glm::vec3(0.0f, 700.0f, 5000.0f), 8.0f);
                const EntityId aside = slave.spawnTarget(glm::vec3(0.0f, 700.0f, 5000.0f), 8.0f);
                // Positive azimuth is the right wing, -X for a +Z nose.
                const glm::vec3 places[] = {glm::vec3(0.0f, 700.0f, 5000.0f),
                                            glm::vec3(-std::sin(offNose) * 5000.0f, 700.0f, std::cos(offNose) * 5000.0f)};
                const EntityId pairIds[] = {ahead, aside};
                for (int index = 0; index < 2; ++index)
                {
                    if (Target *aircraft = slave.findTarget(pairIds[index]))
                    {
                        aircraft->setPosition(places[index]);
                        aircraft->setVelocity(glm::vec3(0.0f, 0.0f, 200.0f));
                    }
                }
                slave.placeFighterAtEngagement();

                const auto trackNearest = [&slave](EntityId platform) {
                    PlatformSnapshot body;
                    if (!slave.snapshot(platform, body))
                    {
                        return sim::kNoEntity;
                    }
                    EntityId nearest = sim::kNoEntity;
                    float best = 500.0f;
                    for (const TrackEstimate &track : slave.playerRadar().tracks().tracks())
                    {
                        const float gap = glm::length(track.position - body.position);
                        if (track.life != TrackLife::Lost && gap < best)
                        {
                            best = gap;
                            nearest = track.id;
                        }
                    }
                    return nearest;
                };

                bool bothTracked = false;
                for (int step = 0; step < 200 && !bothTracked; ++step)
                {
                    slave.step();
                    bothTracked = trackNearest(ahead).valid() && trackNearest(aside).valid() &&
                                  trackNearest(ahead) != trackNearest(aside);
                }
                slave.cycleRadarDesignation();
                const bool noseFirst = bothTracked && slave.designatedRadarTrack() == trackNearest(ahead);
                slave.cycleRadarDesignation();
                const bool thenAside = bothTracked && slave.designatedRadarTrack() == trackNearest(aside);
                for (int step = 0; step < 30; ++step)
                {
                    slave.step();
                }

                const sim::FireControl fire = slave.fireControl();
                const Missile *round = slave.readyRound();
                PlatformSnapshot owner;
                PlatformSnapshot side;
                bool headOnAside = false;
                if (round != nullptr && slave.snapshot(slave.fighterId(), owner) && slave.snapshot(aside, side))
                {
                    const glm::vec3 lineOfSight = glm::normalize(side.position - round->getPosition());
                    headOnAside = glm::dot(glm::normalize(fire.seeker.lookDirection), lineOfSight) > std::cos(glm::radians(2.0f));
                }
                const bool tookAside = round != nullptr && round->getTargetObject() != nullptr &&
                                       round->getTargetObject()->getEntityId() == aside;
                const bool slavedState = fire.seeker.state == sim::SeekerState::Slaved ||
                                         fire.seeker.state == sim::SeekerState::Designated ||
                                         fire.seeker.state == sim::SeekerState::Locked;
                checks.expect(noseFirst && thenAside, "radar: T locks the contact nearest the nose first");
                checks.expect(!slave.seekerUncaged() && fire.radar == sim::RadarMode::Track && fire.lock.valid &&
                                  slavedState && headOnAside && tookAside &&
                                  slave.launchClearance() != LaunchBlock::SeekerCaged,
                              "fire control: the radar lock slaves the heat seeker");

                // The lock is a radar picture: it reads the track, never the aircraft.
                const bool lockFromTrack = fire.lock.track == slave.designatedRadarTrack() && fire.lock.track != aside &&
                                           std::abs(glm::length(fire.lock.position - owner.position) - fire.lock.rangeM) < 1.0f;
                checks.expect(lockFromTrack, "fire control: the lock is the radar track");

                // Selecting the radar round powers the heat seeker down.
                slave.setFighterWeapon(FighterWeapon::RadarRound);
                slave.step();
                const sim::FireControl radarFire = slave.fireControl();
                const Missile *rail = slave.readyRound();
                checks.expect(radarFire.seeker.state == sim::SeekerState::Off && rail != nullptr &&
                                  rail->getTargetObject() == nullptr && radarFire.lock.valid,
                              "fire control: the radar round keeps the lock and cages the rail");
            }

            // Countermeasures release on demand, with nothing heard yet. The
            // dispenser then cycles, so a held key cannot empty it in one step.
            World dispenser(config, 3);
            dispenser.setRoleFighter("aim-9x-blk2");
            const int chaffBefore = dispenser.chaffRemaining();
            const int flaresBefore = dispenser.flaresRemaining();
            const bool chaffOut = dispenser.dispenseChaff();
            const bool chaffCycling = !dispenser.dispenseChaff();
            const bool flaresOut = dispenser.dispenseFlares();
            const bool flaresCycling = !dispenser.dispenseFlares();
            checks.expect(dispenser.warnings().radar.empty() && dispenser.warnings().approach.empty() && chaffOut &&
                              chaffCycling && dispenser.chaffRemaining() == chaffBefore - 1 &&
                              dispenser.chaffRounds().size() == 1,
                          "countermeasures: chaff releases without a warning, one bundle per cycle");
            checks.expect(flaresOut && flaresCycling && dispenser.flaresRemaining() == flaresBefore - 2 &&
                              dispenser.flares().size() == 2,
                          "countermeasures: flares release in pairs, one pair per cycle");
            for (int step = 0; step < 40; ++step)
            {
                dispenser.step();
            }
            const bool again = dispenser.dispenseChaff() && dispenser.dispenseFlares();
            dispenser.rearm();
            checks.expect(again && dispenser.chaffRemaining() == chaffBefore && dispenser.flaresRemaining() == flaresBefore,
                          "countermeasures: the dispenser cycles again and G refills both stores");
            PlatformSnapshot ownship;
            bool belowAndBehind = false;
            if (dispenser.snapshot(dispenser.fighterId(), ownship) && !dispenser.flares().empty())
            {
                const Flare &flare = *dispenser.flares().back();
                const glm::vec3 relative = flare.getVelocity() - ownship.velocity;
                belowAndBehind = relative.y < 0.0f;
            }
            checks.expect(belowAndBehind, "countermeasures: flares leave downward");

            World infrared(config, 5);
            infrared.setTerrain(flatConfig);
            infrared.setRoleFighter("aim-9x-blk2");
            const EntityId bandit = infrared.spawnTarget(glm::vec3(0.0f, 3000.0f, 12000.0f), 8.0f);
            infrared.placeFighterAtEngagement();
            PlatformSnapshot fighterSnap;
            infrared.snapshot(infrared.fighterId(), fighterSnap);
            ScriptedLaunch fox2;
            fox2.team = Team::Red;
            fox2.position = fighterSnap.position + glm::vec3(0.0f, 0.0f, -2000.0f);
            fox2.velocity = glm::vec3(0.0f, 0.0f, 400.0f);
            fox2.fox2Id = "aim-9x-blk2";
            fox2.target = bandit;
            infrared.launchScripted(fox2);
            bool radarHit = false;
            bool approach = false;
            for (int step = 0; step < 20; ++step)
            {
                infrared.step();
                if (!infrared.warnings().radar.empty())
                {
                    radarHit = true;
                }
                if (!infrared.warnings().approach.empty())
                {
                    approach = true;
                }
            }
            checks.expect(!radarHit && approach, "radar: an infrared shot warns by approach only");

            World opponent(config, 9);
            opponent.setTraceRecording(true);
            opponent.setTerrain(flatConfig);
            opponent.setRoleFighter("aim-9x-blk2");
            const EntityId opponentId = opponent.spawnTarget(glm::vec3(0.0f, 3000.0f, 6000.0f), 8.0f);
            opponent.placeFighterAtEngagement();
            if (Target *banditAircraft = opponent.findTarget(opponentId))
            {
                banditAircraft->setVelocity(glm::vec3(0.0f, 0.0f, -220.0f));
            }
            opponent.setRedRadar(true);
            bool heardRadar = false;
            bool heardLaunch = false;
            bool heardSeeker = false;
            for (int step = 0; step < 4000 && !heardSeeker; ++step)
            {
                opponent.step();
                for (const Warning &warning : opponent.warnings().radar)
                {
                    if (warning.kind == WarningKind::MissileSeeker)
                    {
                        heardSeeker = true;
                    }
                    else
                    {
                        heardRadar = true;
                        heardLaunch = heardLaunch || warning.kind == WarningKind::RadarLaunch;
                    }
                }
            }
            bool opponentLaunch = false;
            for (const SimEvent &event : opponent.trace())
            {
                if (event.type == EventType::WeaponLaunched && event.other != opponentId)
                {
                    opponentLaunch = true;
                }
            }
            checks.expect(heardRadar && heardSeeker && opponentLaunch,
                          "radar: the opponent fires from its own track and the seeker is heard");
            checks.expect(heardLaunch, "radar: the opponent's guided round is heard as a launch");
            checks.expect(opponent.hostileRadarRoundsRemaining() < 2, "radar: the opponent spends a round");
            opponent.rearmHostileRadar();
            checks.expect(opponent.hostileRadarRoundsRemaining() == 2, "radar: a new formation reloads the opponent");
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
        checkTerrainSurface(checks);
        checkMountainsAndWater(checks);
        checkTerrainConfig(checks);
        checkTerrainContacts(checks, config);
        checkSensorModel(checks);
        checks.expect(sim::runScanRadarChecks() == 0, "scan radar suite");
        checks.expect(sim::runRadarHomingChecks() == 0, "radar homing suite");
        checks.expect(sim::runDefenseChecks() == 0, "defense suite");
        checks.expect(missilesim::fox3::runFox3CatalogChecks() == 0, "fox3 catalog");
        checkRadarEngagement(checks, config);

        std::printf("%d/%d engagement checks passed\n", checks.count - checks.failures, checks.count);
        return checks.failures;
    }
}
