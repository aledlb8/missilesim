#include "Scenarios.h"

#include "objects/Fighter.h"
#include "objects/Target.h"
#include "sim/FixedStepClock.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace harness
{
    namespace sim = missilesim::sim;

    namespace
    {
        constexpr float kGravity = 9.81f;

        // The n-th target still alive, in spawn order (none if fewer).
        sim::EntityId aliveTarget(const World &world, std::size_t n)
        {
            std::size_t seen = 0;
            for (const auto &target : world.targets())
            {
                if (world.targetAlive(target->getEntityId()))
                {
                    if (seen == n)
                    {
                        return target->getEntityId();
                    }
                    ++seen;
                }
            }
            return sim::kNoEntity;
        }

        Scenario fox2Scenario(const std::string &fox2Id, const char *name, const char *description, std::uint64_t seed,
                              std::vector<double> launchTimes)
        {
            Scenario scenario;
            scenario.name = name;
            scenario.description = description;
            scenario.seed = seed;
            scenario.durationSeconds = 30.0;
            scenario.setup = [fox2Id](World &world) {
                world.respawnTargets(1);
                world.setRoleFighter(fox2Id);
            };
            scenario.beforeStep = [launchTimes](World &world, std::uint64_t tick) {
                for (const double seconds : launchTimes)
                {
                    if (tick == tickAt(world, seconds))
                    {
                        fireFox2(world);
                    }
                }
            };
            return scenario;
        }

        Scenario samScenario(const char *name, const char *description, std::uint64_t seed, int targets,
                             std::vector<double> launchTimes, double duration)
        {
            Scenario scenario;
            scenario.name = name;
            scenario.description = description;
            scenario.seed = seed;
            scenario.durationSeconds = duration;
            scenario.setup = [targets](World &world) {
                world.respawnTargets(targets);
                world.setRoleSam(world.customRoundSpec());
            };
            scenario.beforeStep = [launchTimes](World &world, std::uint64_t tick) {
                for (std::size_t index = 0; index < launchTimes.size(); ++index)
                {
                    if (tick == tickAt(world, launchTimes[index]))
                    {
                        // Each ripple round goes after a different aircraft
                        // while there are enough left.
                        sim::EntityId target = aliveTarget(world, index);
                        if (!target.valid())
                        {
                            target = aliveTarget(world, 0);
                        }
                        fireSam(world, target);
                    }
                }
            };
            return scenario;
        }
    }

    std::uint64_t tickAt(const World &world, double seconds)
    {
        const double step = static_cast<double>(world.fixedStep());
        return static_cast<std::uint64_t>(std::max(1.0, std::ceil(seconds / step - 1.0e-9)));
    }

    sim::EntityId fireFox2(World &world)
    {
        world.setSeekerUncaged(true);
        const Fighter *jet = world.fighter();
        const glm::vec3 aim = jet != nullptr ? jet->getNose() : glm::vec3(0.0f, 0.0f, 1.0f);
        return world.launch(aim).shot;
    }

    sim::EntityId fireSam(World &world, sim::EntityId target)
    {
        world.setSeekerUncaged(true);
        world.designate(target);
        return world.launch(glm::vec3(0.0f, 0.0f, 1.0f)).shot;
    }

    RunResult runScenario(const sim::SimulationConfig &config, const Scenario &scenario, const StepObserver &observer,
                          const std::function<float(std::uint64_t frame)> &pacing)
    {
        World world(config, scenario.seed);
        world.setTraceRecording(true);
        world.restart(scenario.seed);
        if (scenario.setup)
        {
            scenario.setup(world);
        }
        world.drainEvents();

        const auto totalTicks = static_cast<std::uint64_t>(std::ceil(scenario.durationSeconds / static_cast<double>(world.fixedStep())));
        RunResult result;
        double wallSeconds = 0.0;

        const auto stepOnce = [&]() {
            if (scenario.beforeStep)
            {
                scenario.beforeStep(world, world.tick() + 1);
            }
            const auto start = std::chrono::steady_clock::now();
            world.step();
            wallSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            result.peakShots = std::max(result.peakShots, world.shots().size());
            result.peakFlares = std::max(result.peakFlares, world.flares().size());
            const std::vector<sim::SimEvent> events = world.drainEvents();
            if (observer)
            {
                observer(world, events);
            }
        };

        if (pacing)
        {
            sim::FixedStepClock clock(world.fixedStep());
            for (std::uint64_t frame = 0; world.tick() < totalTicks; ++frame)
            {
                const int steps = clock.advance(pacing(frame), 1.0f);
                for (int step = 0; step < steps && world.tick() < totalTicks; ++step)
                {
                    stepOnce();
                }
            }
        }
        else
        {
            while (world.tick() < totalTicks)
            {
                stepOnce();
            }
        }

        result.trace = world.trace();
        result.hash = sim::traceHash(result.trace);
        result.ticks = world.tick();
        result.wallMicrosecondsPerStep = result.ticks > 0 ? wallSeconds * 1.0e6 / static_cast<double>(result.ticks) : 0.0;
        return result;
    }

    std::vector<Scenario> standardScenarios(const std::string &fox2Id)
    {
        std::vector<Scenario> scenarios;
        scenarios.push_back(fox2Scenario(fox2Id, "fox2-single", "Fighter fires one Fox 2 at the lead aircraft.", 101, {1.0}));
        scenarios.push_back(fox2Scenario(fox2Id, "fox2-salvo", "Fighter ripples both wingtip rounds 0.4 s apart.", 202, {1.0, 1.4}));
        scenarios.push_back(samScenario("sam-single", "SAM cold-launches one custom round at one aircraft.", 303, 1, {0.5}, 30.0));
        scenarios.push_back(samScenario("sam-ripple", "SAM ripples three rounds at three aircraft.", 404, 3, {0.5, 2.6, 4.7}, 40.0));
        scenarios.push_back(samScenario("busy", "Twelve aircraft, four SAM rounds, flares: step cost baseline.", 505, 12,
                                        {0.5, 2.6, 4.7, 6.8}, 30.0));

        Scenario threat;
        threat.name = "threat-on-player";
        threat.description = "An unguided scripted round meets the fighter head-on: player damage and respawn.";
        threat.seed = 606;
        threat.durationSeconds = 6.0;
        threat.setup = [fox2Id](World &world) {
            world.respawnTargets(1);
            world.setRoleFighter(fox2Id);
        };
        threat.beforeStep = [](World &world, std::uint64_t tick) {
            if (tick != tickAt(world, 0.2) || world.fighter() == nullptr)
            {
                return;
            }
            // Head-on at 500 m/s from 1.5 km: meets the fighter in about two
            // seconds; start high by the gravity drop over that time.
            const Fighter &jet = *world.fighter();
            const glm::vec3 nose = jet.getNose();
            const float closure = 500.0f + glm::length(jet.getVelocity());
            const float meet = 1500.0f / closure;
            sim::ScriptedLaunch launch;
            launch.team = sim::Team::Red;
            launch.position = jet.getPosition() + nose * 1500.0f + glm::vec3(0.0f, 0.5f * kGravity * meet * meet, 0.0f);
            launch.velocity = -nose * 500.0f;
            world.launchScripted(launch);
        };
        scenarios.push_back(threat);
        return scenarios;
    }

    const Scenario *findScenario(const std::vector<Scenario> &scenarios, const std::string &name)
    {
        for (const Scenario &scenario : scenarios)
        {
            if (scenario.name == name)
            {
                return &scenario;
            }
        }
        return nullptr;
    }
}
