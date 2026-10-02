#pragma once

// Seeded engagement scenarios for the headless harness. A scenario is a
// setup plus a script of commands scheduled on simulation ticks, so a run
// is a pure function of (build, config, seed, scenario): the contract the
// trace checks rely on.

#include "sim/SimEvents.h"
#include "sim/SimulationConfig.h"
#include "sim/World.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace harness
{
    using missilesim::sim::World;

    struct Scenario
    {
        std::string name;
        std::string description;
        std::uint64_t seed = 1;
        double durationSeconds = 30.0;
        // Builds the scene on a freshly restarted world.
        std::function<void(World &)> setup;
        // Runs before every step with the index of the step about to run
        // (1 for the first). Commands issued here happen at that tick.
        std::function<void(World &, std::uint64_t)> beforeStep;
    };

    struct RunResult
    {
        std::vector<missilesim::sim::SimEvent> trace;
        std::uint64_t hash = 0;
        std::uint64_t ticks = 0;
        double wallMicrosecondsPerStep = 0.0;
        std::size_t peakShots = 0;
        std::size_t peakFlares = 0;
    };

    // Called after every step, for checks that inspect the live world.
    using StepObserver = std::function<void(World &, const std::vector<missilesim::sim::SimEvent> &)>;

    // Runs a scenario from a fresh world. `pacing` optionally supplies the
    // rendered-frame durations to drive the world through FixedStepClock
    // (nullptr: step directly).
    RunResult runScenario(const missilesim::sim::SimulationConfig &config, const Scenario &scenario,
                          const StepObserver &observer = nullptr,
                          const std::function<float(std::uint64_t frame)> &pacing = nullptr);

    // The standard scenario set. Fighter scenarios carry `fox2Id`.
    std::vector<Scenario> standardScenarios(const std::string &fox2Id = "aim-9x-blk2");
    const Scenario *findScenario(const std::vector<Scenario> &scenarios, const std::string &name);

    // Schedules `action` at the first step at or after `seconds`.
    std::uint64_t tickAt(const World &world, double seconds);

    // Fox 2 salvo helper: uncage and fire the next rail (returns the shot).
    missilesim::sim::EntityId fireFox2(World &world);
    // SAM helper: uncage, designate `target` and fire the cell.
    missilesim::sim::EntityId fireSam(World &world, missilesim::sim::EntityId target);
}
