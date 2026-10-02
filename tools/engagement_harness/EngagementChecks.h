#pragma once

#include "sim/SimulationConfig.h"

namespace harness
{
    // Runs every engagement contract check; returns the number of failures.
    int runEngagementChecks(const missilesim::sim::SimulationConfig &config);
}
