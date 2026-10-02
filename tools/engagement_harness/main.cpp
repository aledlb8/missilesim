// Headless engagement harness.
//
// Usage:
//   engagement_harness                   summary of every standard scenario
//   engagement_harness <scenario>        full event trace of one scenario
//   engagement_harness --seed N <name>   the same with another seed
//   engagement_harness --no-flares ...   targets carry no flares (isolates guidance)
//   engagement_harness --fox2 ID ...     fighter scenarios carry this catalog round
//   engagement_harness --telemetry <name> every round's state each 0.1 s
//   engagement_harness --check           contract checks (exit code = failures)
//
// Every run steps sim::World, the class the game steps, from the shipped
// simulation config. A run is a pure function of build, config, seed and
// scenario: the printed hash identifies a trace bit for bit.
#include "EngagementChecks.h"
#include "Scenarios.h"

#include "objects/Missile.h"
#include "objects/Target.h"
#include "sim/SimEvents.h"
#include "sim/SimulationConfig.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

namespace
{
    namespace sim = missilesim::sim;

    // Every round in the air, every 0.1 s: where it is, how fast, what its
    // seeker holds and how far the designated aircraft is.
    void printTelemetry(harness::World &world, const std::vector<sim::SimEvent> &)
    {
        if (world.tick() % 10 != 0)
        {
            return;
        }
        for (const auto &shot : world.shots())
        {
            const Missile &missile = *shot->missile;
            const Target *target = world.findTarget(shot->designatedTarget);
            const float range = target != nullptr ? glm::length(target->getPosition() - missile.getPosition()) : -1.0f;
            const char *seeker = missile.isTrackingDecoy()       ? "decoy"
                                 : missile.hasFox2InfraredLock() ? "lock"
                                 : missile.fox2OnTrackMemory()   ? "memory"
                                 : missile.fox2GuidanceExpired() ? "expired"
                                 : missile.hasTarget()           ? "designated"
                                                                 : "none";
            std::printf("t=%7.2f shot %-3u pos (%8.1f %7.1f %8.1f) speed %6.1f  range %8.1f  seeker %-10s CL %6.2f  motor %s\n",
                        world.time(), shot->id.value, static_cast<double>(missile.getPosition().x),
                        static_cast<double>(missile.getPosition().y), static_cast<double>(missile.getPosition().z),
                        static_cast<double>(glm::length(missile.getVelocity())), static_cast<double>(range), seeker,
                        static_cast<double>(missile.getCommandedLiftCoefficient()), missile.isThrustEnabled() ? "on" : "off");
        }
    }

    void printSummary(const harness::Scenario &scenario, const harness::RunResult &run)
    {
        std::map<std::string, int> endings;
        int launches = 0;
        int destroyed = 0;
        int flares = 0;
        for (const sim::SimEvent &event : run.trace)
        {
            switch (event.type)
            {
            case sim::EventType::WeaponLaunched:
                ++launches;
                break;
            case sim::EventType::ShotEnded:
                ++endings[sim::shotEndReasonName(static_cast<sim::ShotEndReason>(event.detail))];
                break;
            case sim::EventType::PlatformDestroyed:
                ++destroyed;
                break;
            case sim::EventType::CountermeasureReleased:
                ++flares;
                break;
            default:
                break;
            }
        }

        std::printf("%-18s seed %-5llu %6zu events  hash %016llx  %6.1f us/step  peak %zu shots, %zu flares\n",
                    scenario.name.c_str(), static_cast<unsigned long long>(scenario.seed), run.trace.size(),
                    static_cast<unsigned long long>(run.hash), run.wallMicrosecondsPerStep, run.peakShots, run.peakFlares);
        std::printf("%18s %s\n", "", scenario.description.c_str());
        std::printf("%18s %d launched, %d destroyed, %d flares;", "", launches, destroyed, flares);
        for (const auto &ending : endings)
        {
            std::printf(" %s x%d", ending.first.c_str(), ending.second);
        }
        std::printf("\n");
    }
}

int main(int argc, char **argv)
{
    const sim::SimulationConfigLoadResult load = sim::loadDefaultSimulationConfig();
    if (!load.loaded)
    {
        std::fprintf(stderr, "Cannot load the simulation config: %s\n", load.error.c_str());
        return 2;
    }
    for (const std::string &warning : load.warnings)
    {
        std::fprintf(stderr, "config warning: %s\n", warning.c_str());
    }

    sim::SimulationConfig config = load.config;
    std::string name;
    std::string fox2Id = "aim-9x-blk2";
    bool check = false;
    bool telemetry = false;
    bool seedGiven = false;
    unsigned long long seed = 0;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--check")
        {
            check = true;
        }
        else if (argument == "--telemetry")
        {
            telemetry = true;
        }
        else if (argument == "--no-flares")
        {
            config.targets.flares.enabled = false;
        }
        else if (argument == "--fox2" && index + 1 < argc)
        {
            fox2Id = argv[++index];
        }
        else if (argument == "--seed" && index + 1 < argc)
        {
            seed = std::strtoull(argv[++index], nullptr, 10);
            seedGiven = true;
        }
        else
        {
            name = argument;
        }
    }

    if (check)
    {
        return harness::runEngagementChecks(config);
    }

    const std::vector<harness::Scenario> scenarios = harness::standardScenarios(fox2Id);
    if (name.empty())
    {
        for (harness::Scenario scenario : scenarios)
        {
            if (seedGiven)
            {
                scenario.seed = seed;
            }
            printSummary(scenario, harness::runScenario(config, scenario));
        }
        return 0;
    }

    const harness::Scenario *found = harness::findScenario(scenarios, name);
    if (found == nullptr)
    {
        std::fprintf(stderr, "Unknown scenario '%s'. Scenarios:", name.c_str());
        for (const harness::Scenario &scenario : scenarios)
        {
            std::fprintf(stderr, " %s", scenario.name.c_str());
        }
        std::fprintf(stderr, "\n");
        return 2;
    }

    harness::Scenario scenario = *found;
    if (seedGiven)
    {
        scenario.seed = seed;
    }
    const harness::RunResult run =
        harness::runScenario(config, scenario, telemetry ? harness::StepObserver(printTelemetry) : harness::StepObserver());
    if (!telemetry)
    {
        for (const sim::SimEvent &event : run.trace)
        {
            std::printf("%s\n", sim::describeEvent(event).c_str());
        }
    }
    printSummary(scenario, run);
    return 0;
}
