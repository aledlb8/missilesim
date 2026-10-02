#pragma once

// Game rules of the engagement sandbox. None of these is a measured weapon or
// aircraft property; each is a declared simulation choice, kept here so it is
// visible, named and never mistaken for data from the research catalog.

namespace missilesim::sim::rules
{
    // ---- Simulated volume ------------------------------------------------------
    // A shot that leaves this volume is over. It bounds the simulation, it is
    // not a missile performance limit.
    constexpr float kArenaHorizontalRadiusM = 150000.0f;
    constexpr float kArenaCeilingM = 30000.0f;

    // ---- Damage --------------------------------------------------------------------
    // Blast damage is full inside a round's lethal radius and falls linearly
    // to zero at this multiple of it. Distances are to the platform's surface.
    constexpr float kBlastFalloffOuterFactor = 2.0f;
    // A round that strikes an aircraft before its fuze arms delivers only
    // kinetic damage. Sandbox value: enough that two duds bring an aircraft down.
    constexpr float kDudImpactDamage = 0.6f;
    // Remaining structure below which a platform is destroyed.
    constexpr float kDestroyedHealth = 1.0e-4f;

    // ---- Ending a shot ---------------------------------------------------------
    // A burned-out round slower than this cannot fly on and is ended.
    constexpr float kEnergyExhaustedSpeedMps = 15.0f;
    constexpr float kEnergyExhaustedMinFlightS = 2.0f;
    // Custom round only (catalog rounds may lose a target aft and keep
    // flying): the pass is over once the target is behind the round
    // (cosine of the look angle below kOvershootBehindCosine), the range is
    // opening faster than kOvershootOpeningMps, and the range has grown past
    // the closest approach by the larger of the margin and a multiple of the
    // target's radius.
    constexpr float kOvershootMinFlightS = 0.75f;
    constexpr float kOvershootOpeningMarginM = 25.0f;
    constexpr float kOvershootRadiusMultiple = 4.0f;
    constexpr float kOvershootOpeningMps = 15.0f;
    constexpr float kOvershootBehindCosine = -0.15f;

    // ---- Platforms -----------------------------------------------------------------
    // The fighter has no gear and no runway: this close to the ground is a crash.
    constexpr float kFighterGroundClearanceM = 1.5f;
    // Collision radius of the player's fighter (m); the mesh is drawn at this radius.
    constexpr float kFighterRadiusM = 5.0f;
    // Fighter spawn: at the first target's altitude, never below this.
    constexpr float kFighterMinimumSpawnAltitudeM = 600.0f;
    constexpr float kFighterDefaultSpawnAltitudeM = 1500.0f;
    // Medium-altitude cruise at spawn (m/s).
    constexpr float kFighterSpawnSpeedMps = 250.0f;

    // ---- Stores --------------------------------------------------------------------
    // Wingtip rails, in multiples of the fighter's draw radius, measured from
    // assets/models/jet.obj: tips end 0.75 radii out, the wing sits 0.14 below
    // the model centre and the tip trailing edge is 0.57 aft. The round hangs
    // just outboard of and under the tip, tail at the trailing edge.
    constexpr float kRailOutboard = 0.762f;
    constexpr float kRailBelow = 0.16f;
    constexpr float kRailAft = 0.36f;
    // The SAM cell holds one ready round; the next is loaded this long after
    // a launch. The sandbox magazine behind it is unlimited.
    constexpr float kSamReloadSeconds = 2.0f;

    // ---- Custom round launch ---------------------------------------------------------
    // A round standing within this height of the ground is fired with the
    // cold-launch profile; higher it is an air launch and lights at once.
    constexpr float kGroundLaunchClearanceM = 1.6f;
    constexpr float kGroundLaunchProfileHeightM = 12.0f;
    constexpr float kGroundLaunchMinimumPitchDeg = 8.0f;
    constexpr float kGroundLaunchMaximumPitchDeg = 82.0f;
    // Cold launch: the ejection charge lobs the round near-vertically at a
    // gentle speed; the booster then multiplies the configured motor thrust
    // for a short, violent climb before the sustainer.
    constexpr float kColdLaunchEjectSpeedMps = 26.0f;
    constexpr float kColdLaunchEjectPitchDeg = 82.0f;
    constexpr float kColdLaunchBoostThrustMultiplier = 4.0f;
    constexpr float kColdLaunchIgnitionThrottle = 0.35f;
    // Pitch-over: the powered round turns its aim toward the target at a
    // capped rate and hands off to proportional navigation once its velocity
    // is inside this cone of the line of sight. The custom seeker's field is
    // 85 degrees about the velocity, so a blind vertical climb would lose it.
    constexpr float kColdLaunchPitchRateDegPerS = 110.0f;
    constexpr float kColdLaunchHandoffConeDeg = 55.0f;
    // Air launch of the custom round: light almost at once.
    constexpr float kAirLaunchIgnitionDelayS = 0.05f;
    constexpr float kAirLaunchGuidanceArmDelayS = 0.55f;
}
