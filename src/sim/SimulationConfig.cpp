#include "SimulationConfig.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

#include <nlohmann/json.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

namespace missilesim::sim
{
    namespace
    {
        using Json = nlohmann::json;

        constexpr float kLargeFiniteExtent = 1000000.0f;

        const Json *findObject(const Json &object, const char *key)
        {
            const auto found = object.find(key);
            if (found == object.end() || !found->is_object())
            {
                return nullptr;
            }
            return &(*found);
        }

        bool readBool(const Json &object, const char *key, bool fallback)
        {
            const auto found = object.find(key);
            if (found == object.end() || !found->is_boolean())
            {
                return fallback;
            }
            return found->get<bool>();
        }

        int readInt(const Json &object, const char *key, int fallback, int minimum, int maximum)
        {
            const auto found = object.find(key);
            if (found == object.end() || !found->is_number_integer())
            {
                return fallback;
            }
            return std::clamp(found->get<int>(), minimum, maximum);
        }

        float sanitizeFloat(double value, float fallback, float minimum, float maximum)
        {
            if (!std::isfinite(value))
            {
                return fallback;
            }
            return std::clamp(static_cast<float>(value), minimum, maximum);
        }

        float readFloat(const Json &object, const char *key, float fallback, float minimum, float maximum)
        {
            const auto found = object.find(key);
            if (found == object.end() || !found->is_number())
            {
                return fallback;
            }
            return sanitizeFloat(found->get<double>(), fallback, minimum, maximum);
        }

        std::string readString(const Json &object, const char *key, const std::string &fallback)
        {
            const auto found = object.find(key);
            if (found == object.end() || !found->is_string())
            {
                return fallback;
            }
            return found->get<std::string>();
        }

        glm::vec3 readVec3(const Json &object, const char *key, const glm::vec3 &fallback)
        {
            const auto found = object.find(key);
            if (found == object.end())
            {
                return fallback;
            }

            glm::vec3 result = fallback;
            if (found->is_array() && found->size() == 3)
            {
                for (std::size_t index = 0; index < 3; ++index)
                {
                    if (!(*found)[index].is_number())
                    {
                        return fallback;
                    }
                    result[static_cast<glm::length_t>(index)] = sanitizeFloat(
                        (*found)[index].get<double>(),
                        fallback[static_cast<glm::length_t>(index)],
                        -kLargeFiniteExtent,
                        kLargeFiniteExtent);
                }
                return result;
            }

            if (found->is_object())
            {
                result.x = readFloat(*found, "x", fallback.x, -kLargeFiniteExtent, kLargeFiniteExtent);
                result.y = readFloat(*found, "y", fallback.y, -kLargeFiniteExtent, kLargeFiniteExtent);
                result.z = readFloat(*found, "z", fallback.z, -kLargeFiniteExtent, kLargeFiniteExtent);
                return result;
            }

            return fallback;
        }

        // Parse a Mach->multiplier curve given either as an array of [mach, mult]
        // pairs or an array of {"mach": .., "multiplier": ..} objects. Invalid or
        // missing input leaves the fallback untouched. Samples are sorted by mach.
        std::vector<glm::vec2> readMachCurve(const Json &object, const char *key,
                                             const std::vector<glm::vec2> &fallback)
        {
            const auto found = object.find(key);
            if (found == object.end() || !found->is_array())
            {
                return fallback;
            }

            std::vector<glm::vec2> curve;
            for (const Json &entry : *found)
            {
                glm::vec2 sample(0.0f);
                if (entry.is_array() && entry.size() == 2 && entry[0].is_number() && entry[1].is_number())
                {
                    sample.x = sanitizeFloat(entry[0].get<double>(), 0.0f, 0.0f, 100.0f);
                    sample.y = sanitizeFloat(entry[1].get<double>(), 1.0f, 0.0f, 1000.0f);
                }
                else if (entry.is_object())
                {
                    sample.x = readFloat(entry, "mach", 0.0f, 0.0f, 100.0f);
                    sample.y = readFloat(entry, "multiplier", 1.0f, 0.0f, 1000.0f);
                }
                else
                {
                    continue;
                }
                curve.push_back(sample);
            }

            if (curve.size() < 2)
            {
                return fallback;
            }

            std::sort(curve.begin(), curve.end(),
                      [](const glm::vec2 &a, const glm::vec2 &b) { return a.x < b.x; });
            return curve;
        }

        void readEnvironmentConfig(const Json &root, SimulationConfig &config)
        {
            const Json *environment = findObject(root, "environment");
            if (environment == nullptr)
            {
                return;
            }

            config.environment.fixedTimeStep = readFloat(*environment, "fixed_time_step_s", config.environment.fixedTimeStep, 0.0005f, 0.1f);
            config.environment.simulationSpeed = readFloat(*environment, "simulation_speed", config.environment.simulationSpeed, 0.1f, 10.0f);
            config.environment.gravity = readFloat(*environment, "gravity_m_s2", config.environment.gravity, 0.0f, 100.0f);
            config.environment.seaLevelAirDensity = readFloat(*environment, "sea_level_air_density_kg_m3", config.environment.seaLevelAirDensity, 0.0f, 5.0f);
            config.environment.groundCollisionEnabled = readBool(*environment, "ground_collision_enabled", config.environment.groundCollisionEnabled);
            config.environment.groundRestitution = readFloat(*environment, "ground_restitution", config.environment.groundRestitution, 0.0f, 1.0f);
        }

        void readVisualizationConfig(const Json &root, SimulationConfig &config)
        {
            const Json *visualization = findObject(root, "visualization");
            if (visualization == nullptr)
            {
                return;
            }

            config.visualization.showTrajectory = readBool(*visualization, "show_trajectory", config.visualization.showTrajectory);
            config.visualization.showTargetInfo = readBool(*visualization, "show_target_info", config.visualization.showTargetInfo);
            config.visualization.showPredictedTargetPath = readBool(*visualization, "show_predicted_target_path", config.visualization.showPredictedTargetPath);
            config.visualization.showInterceptPoint = readBool(*visualization, "show_intercept_point", config.visualization.showInterceptPoint);
            config.visualization.trajectoryPoints = readInt(*visualization, "trajectory_points", config.visualization.trajectoryPoints, 10, 600);
            config.visualization.trajectoryTime = readFloat(*visualization, "trajectory_time_s", config.visualization.trajectoryTime, 0.5f, 60.0f);
        }

        void readCameraConfig(const Json &root, SimulationConfig &config)
        {
            const Json *camera = findObject(root, "camera");
            if (camera == nullptr)
            {
                return;
            }

            config.camera.fov = readFloat(*camera, "fov_degrees", config.camera.fov, 10.0f, 120.0f);
            config.camera.speed = readFloat(*camera, "speed", config.camera.speed, 0.1f, 2000.0f);
        }

        void readMissileConfig(const Json &root, SimulationConfig &config)
        {
            const Json *missile = findObject(root, "missile");
            if (missile == nullptr)
            {
                return;
            }

            if (const Json *airframe = findObject(*missile, "airframe"))
            {
                config.missile.airframe.initialPosition = readVec3(*airframe, "initial_position_m", config.missile.airframe.initialPosition);
                config.missile.airframe.initialVelocity = readVec3(*airframe, "initial_velocity_m_s", config.missile.airframe.initialVelocity);
                config.missile.airframe.dryMass = readFloat(*airframe, "dry_mass_kg", config.missile.airframe.dryMass, 0.01f, 10000.0f);
                config.missile.airframe.dragCoefficient = readFloat(*airframe, "drag_coefficient", config.missile.airframe.dragCoefficient, 0.0f, 5.0f);
                config.missile.airframe.crossSectionalArea = readFloat(*airframe, "cross_sectional_area_m2", config.missile.airframe.crossSectionalArea, 0.0001f, 25.0f);
                config.missile.airframe.liftCoefficient = readFloat(*airframe, "lift_coefficient", config.missile.airframe.liftCoefficient, 0.0f, 5.0f);
                config.missile.airframe.aspectRatio = readFloat(*airframe, "aspect_ratio", config.missile.airframe.aspectRatio, 0.0f, 50.0f);
                config.missile.airframe.oswaldEfficiency = readFloat(*airframe, "oswald_efficiency", config.missile.airframe.oswaldEfficiency, 0.1f, 1.0f);
                config.missile.airframe.maxLiftCoefficient = readFloat(*airframe, "max_lift_coefficient", config.missile.airframe.maxLiftCoefficient, 0.1f, 50.0f);
                config.missile.airframe.maxLoadFactorG = readFloat(*airframe, "max_load_factor_g", config.missile.airframe.maxLoadFactorG, 0.0f, 1000.0f);
                config.missile.airframe.machDragMultiplier = readMachCurve(*airframe, "mach_drag_multiplier", config.missile.airframe.machDragMultiplier);
            }

            if (const Json *motor = findObject(*missile, "motor"))
            {
                config.missile.motor.thrust = readFloat(*motor, "thrust_n", config.missile.motor.thrust, 0.0f, 10000000.0f);
                config.missile.motor.fuelMass = readFloat(*motor, "fuel_mass_kg", config.missile.motor.fuelMass, 0.0f, 100000.0f);
                config.missile.motor.fuelConsumptionRate = readFloat(*motor, "fuel_consumption_rate_kg_s", config.missile.motor.fuelConsumptionRate, 0.0f, 10000.0f);
                config.missile.motor.nozzleExitArea = readFloat(*motor, "nozzle_exit_area_m2", config.missile.motor.nozzleExitArea, 0.0f, 100.0f);
                config.missile.motor.nozzleExitPressure = readFloat(*motor, "nozzle_exit_pressure_pa", config.missile.motor.nozzleExitPressure, 0.0f, 500000.0f);
            }

            if (const Json *guidance = findObject(*missile, "guidance"))
            {
                config.missile.guidance.enabled = readBool(*guidance, "enabled", config.missile.guidance.enabled);
                config.missile.guidance.navigationGain = readFloat(*guidance, "navigation_gain", config.missile.guidance.navigationGain, 1.0f, 4.0f);
                config.missile.guidance.maxSteeringForce = readFloat(*guidance, "max_steering_force_n", config.missile.guidance.maxSteeringForce, 0.0f, 10000000.0f);
                config.missile.guidance.trackingAngle = readFloat(*guidance, "tracking_angle_degrees", config.missile.guidance.trackingAngle, 5.0f, 180.0f);
                config.missile.guidance.proximityFuseRadius = readFloat(*guidance, "proximity_fuse_radius_m", config.missile.guidance.proximityFuseRadius, 0.0f, 10000.0f);
                config.missile.guidance.countermeasureResistance = readFloat(*guidance, "countermeasure_resistance", config.missile.guidance.countermeasureResistance, 0.0f, 1.0f);
                config.missile.guidance.terrainAvoidanceEnabled = readBool(*guidance, "terrain_avoidance_enabled", config.missile.guidance.terrainAvoidanceEnabled);
                config.missile.guidance.terrainClearance = readFloat(*guidance, "terrain_clearance_m", config.missile.guidance.terrainClearance, 0.0f, 10000.0f);
                config.missile.guidance.terrainLookAheadTime = readFloat(*guidance, "terrain_lookahead_time_s", config.missile.guidance.terrainLookAheadTime, 0.5f, 60.0f);
                config.missile.guidance.seekerCueRadiusPixels = readFloat(*guidance, "seeker_cue_radius_px", config.missile.guidance.seekerCueRadiusPixels, 1.0f, 1000.0f);
            }
        }

        void readTargetMawsConfig(const Json &targets, TargetGroupConfig &config)
        {
            const Json *maws = findObject(targets, "maws");
            if (maws == nullptr)
            {
                return;
            }

            config.maws.enabled = readBool(*maws, "enabled", config.maws.enabled);
            config.maws.detectionRange = readFloat(*maws, "detection_range_m", config.maws.detectionRange, 0.0f, 100000.0f);
            config.maws.reactionTimeWindow = readFloat(*maws, "reaction_time_window_s", config.maws.reactionTimeWindow, 0.1f, 120.0f);
            config.maws.closestApproachThreshold = readFloat(*maws, "closest_approach_threshold_m", config.maws.closestApproachThreshold, 0.0f, 10000.0f);
        }

        void readTargetFlareConfig(const Json &targets, TargetGroupConfig &config)
        {
            const Json *flares = findObject(targets, "flares");
            if (flares == nullptr)
            {
                return;
            }

            config.flares.enabled = readBool(*flares, "enabled", config.flares.enabled);
            config.flares.inventory = readInt(*flares, "inventory", config.flares.inventory, 0, 500);
            config.flares.burstSize = readInt(*flares, "burst_size", config.flares.burstSize, 1, 50);
            config.flares.burstInterval = readFloat(*flares, "burst_interval_s", config.flares.burstInterval, 0.0f, 10.0f);
            config.flares.cooldown = readFloat(*flares, "cooldown_s", config.flares.cooldown, 0.0f, 60.0f);
            config.flares.ejectSpeed = readFloat(*flares, "eject_speed_m_s", config.flares.ejectSpeed, 0.0f, 1000.0f);
            config.flares.aftLaunchOffset = readFloat(*flares, "aft_launch_offset_m", config.flares.aftLaunchOffset, 0.0f, 100.0f);
            config.flares.lateralLaunchOffset = readFloat(*flares, "lateral_launch_offset_m", config.flares.lateralLaunchOffset, 0.0f, 100.0f);
            config.flares.lateralEjectFraction = readFloat(*flares, "lateral_eject_fraction", config.flares.lateralEjectFraction, 0.0f, 10.0f);
            config.flares.downwardEjectFraction = readFloat(*flares, "downward_eject_fraction", config.flares.downwardEjectFraction, 0.0f, 10.0f);
            config.flares.lifetime = readFloat(*flares, "lifetime_s", config.flares.lifetime, 0.0f, 120.0f);
            config.flares.heatSignature = readFloat(*flares, "heat_signature", config.flares.heatSignature, 0.0f, 10000.0f);
            config.flares.heatDecayRate = readFloat(*flares, "heat_decay_rate", config.flares.heatDecayRate, 0.0f, 100.0f);
            config.flares.mass = readFloat(*flares, "mass_kg", config.flares.mass, 0.01f, 1000.0f);
            config.flares.dragCoefficient = readFloat(*flares, "drag_coefficient", config.flares.dragCoefficient, 0.0f, 10.0f);
            config.flares.crossSectionalArea = readFloat(*flares, "cross_sectional_area_m2", config.flares.crossSectionalArea, 0.0001f, 10.0f);
        }

        void readTargetEvasiveConfig(const Json &targets, TargetGroupConfig &config)
        {
            const Json *evasive = findObject(targets, "evasive");
            if (evasive == nullptr)
            {
                return;
            }

            config.evasive.cruisePitchRateDegrees = readFloat(*evasive, "cruise_pitch_rate_degrees_s", config.evasive.cruisePitchRateDegrees, 0.0f, 360.0f);
            config.evasive.defensivePitchRateDegrees = readFloat(*evasive, "defensive_pitch_rate_degrees_s", config.evasive.defensivePitchRateDegrees, 0.0f, 360.0f);
            config.evasive.defensiveBreakWeight = readFloat(*evasive, "defensive_break_weight", config.evasive.defensiveBreakWeight, 0.0f, 20.0f);
            config.evasive.defensiveAwayWeight = readFloat(*evasive, "defensive_away_weight", config.evasive.defensiveAwayWeight, 0.0f, 20.0f);
            config.evasive.defensiveThreatAwayWeight = readFloat(*evasive, "defensive_threat_away_weight", config.evasive.defensiveThreatAwayWeight, 0.0f, 20.0f);
            config.evasive.defensiveIncomingWeight = readFloat(*evasive, "defensive_incoming_weight", config.evasive.defensiveIncomingWeight, 0.0f, 20.0f);
            config.evasive.altitudeCorrectionRange = readFloat(*evasive, "altitude_correction_range_m", config.evasive.altitudeCorrectionRange, 1.0f, 10000.0f);
            config.evasive.maxAltitudePitchBias = readFloat(*evasive, "max_altitude_pitch_bias", config.evasive.maxAltitudePitchBias, 0.0f, 2.0f);
            config.evasive.innerDistanceAltitudeOffset = readFloat(*evasive, "inner_distance_altitude_offset_m", config.evasive.innerDistanceAltitudeOffset, -10000.0f, 10000.0f);
            config.evasive.outerDistanceAltitudeOffset = readFloat(*evasive, "outer_distance_altitude_offset_m", config.evasive.outerDistanceAltitudeOffset, -10000.0f, 10000.0f);
            config.evasive.defensiveLowEnergyAltitudeOffset = readFloat(*evasive, "defensive_low_energy_altitude_offset_m", config.evasive.defensiveLowEnergyAltitudeOffset, -10000.0f, 10000.0f);
            config.evasive.defensiveThreatBelowAltitudeOffset = readFloat(*evasive, "defensive_threat_below_altitude_offset_m", config.evasive.defensiveThreatBelowAltitudeOffset, -10000.0f, 10000.0f);
            config.evasive.defensiveThreatAboveAltitudeOffset = readFloat(*evasive, "defensive_threat_above_altitude_offset_m", config.evasive.defensiveThreatAboveAltitudeOffset, -10000.0f, 10000.0f);
            config.evasive.defensiveSpeedBlend = readFloat(*evasive, "defensive_speed_blend", config.evasive.defensiveSpeedBlend, 0.0f, 1.0f);
            config.evasive.defensiveSpeedUrgencyBlend = readFloat(*evasive, "defensive_speed_urgency_blend", config.evasive.defensiveSpeedUrgencyBlend, 0.0f, 1.0f);
            config.evasive.nearDistanceSpeedupThreshold = readFloat(*evasive, "near_distance_speedup_threshold", config.evasive.nearDistanceSpeedupThreshold, 0.0f, 1.0f);
            config.evasive.farDistanceSlowdownThreshold = readFloat(*evasive, "far_distance_slowdown_threshold", config.evasive.farDistanceSlowdownThreshold, 0.0f, 1.0f);
            config.evasive.farDistanceSpeedFloor = readFloat(*evasive, "far_distance_speed_floor", config.evasive.farDistanceSpeedFloor, 0.0f, 1.0f);
            config.evasive.recoveryEnergyThreshold = readFloat(*evasive, "recovery_energy_threshold", config.evasive.recoveryEnergyThreshold, 0.0f, 1.0f);
            config.evasive.repositionDistanceThreshold = readFloat(*evasive, "reposition_distance_threshold", config.evasive.repositionDistanceThreshold, 0.0f, 1.0f);
        }

        void readTargetConfig(const Json &root, SimulationConfig &config)
        {
            const Json *targets = findObject(root, "targets");
            if (targets == nullptr)
            {
                return;
            }

            config.targets.maxCount = readInt(*targets, "max_count", config.targets.maxCount, 1, 100);
            config.targets.count = readInt(*targets, "count", config.targets.count, 1, config.targets.maxCount);
            if (const Json *ai = findObject(*targets, "ai"))
            {
                config.targets.minSpeed = readFloat(*ai, "min_speed_m_s", config.targets.minSpeed, 40.0f, 2000.0f);
                config.targets.maxSpeed = readFloat(*ai, "max_speed_m_s", config.targets.maxSpeed, config.targets.minSpeed + 10.0f, 2500.0f);
                config.targets.maxSpeed = std::max(config.targets.maxSpeed, config.targets.minSpeed + 10.0f);
                config.targets.preferredDistance = readFloat(*ai, "preferred_distance_m", config.targets.preferredDistance, 250.0f, 100000.0f);
            }
            if (const Json *spawn = findObject(*targets, "spawn"))
            {
                config.targets.spawn.distanceScaleMin = readFloat(*spawn, "distance_scale_min", config.targets.spawn.distanceScaleMin, 0.05f, 10.0f);
                config.targets.spawn.distanceScaleMax = readFloat(*spawn, "distance_scale_max", config.targets.spawn.distanceScaleMax, config.targets.spawn.distanceScaleMin, 10.0f);
                config.targets.spawn.radiusMin = readFloat(*spawn, "radius_min_m", config.targets.spawn.radiusMin, 0.1f, 1000.0f);
                config.targets.spawn.radiusMax = readFloat(*spawn, "radius_max_m", config.targets.spawn.radiusMax, config.targets.spawn.radiusMin, 1000.0f);
                config.targets.spawn.fallbackRadius = readFloat(*spawn, "fallback_radius_m", config.targets.spawn.fallbackRadius, 0.1f, 1000.0f);
                config.targets.spawn.fallbackSpacing = readFloat(*spawn, "fallback_spacing_m", config.targets.spawn.fallbackSpacing, 0.0f, 100000.0f);
                config.targets.spawn.fallbackPosition = readVec3(*spawn, "fallback_position_m", config.targets.spawn.fallbackPosition);
                config.targets.spawn.minimumAltitudeFloor = readFloat(*spawn, "minimum_altitude_floor_m", config.targets.spawn.minimumAltitudeFloor, 0.0f, 100000.0f);
                config.targets.spawn.minimumAltitudeDistanceFraction = readFloat(*spawn, "minimum_altitude_distance_fraction", config.targets.spawn.minimumAltitudeDistanceFraction, 0.0f, 10.0f);
                config.targets.spawn.minimumAltitudeCeiling = readFloat(*spawn, "minimum_altitude_ceiling_m", config.targets.spawn.minimumAltitudeCeiling, config.targets.spawn.minimumAltitudeFloor, 100000.0f);
                config.targets.spawn.maximumAltitudeDistanceFraction = readFloat(*spawn, "maximum_altitude_distance_fraction", config.targets.spawn.maximumAltitudeDistanceFraction, 0.0f, 10.0f);
                config.targets.spawn.maximumAltitudeCeiling = readFloat(*spawn, "maximum_altitude_ceiling_m", config.targets.spawn.maximumAltitudeCeiling, config.targets.spawn.minimumAltitudeFloor, 100000.0f);
                config.targets.spawn.minimumAltitudeBand = readFloat(*spawn, "minimum_altitude_band_m", config.targets.spawn.minimumAltitudeBand, 0.0f, 100000.0f);
                config.targets.spawn.warmupTimeStep = readFloat(*spawn, "warmup_time_step_s", config.targets.spawn.warmupTimeStep, 0.0f, 1.0f);
            }
            readTargetMawsConfig(*targets, config.targets);
            readTargetFlareConfig(*targets, config.targets);
            readTargetEvasiveConfig(*targets, config.targets);
        }

        SimulationConfig parseSimulationConfig(const Json &root)
        {
            SimulationConfig config;
            if (!root.is_object())
            {
                return config;
            }

            config.schemaVersion = readInt(root, "schema_version", config.schemaVersion, 1, 1);
            config.name = readString(root, "name", config.name);
            readEnvironmentConfig(root, config);
            readVisualizationConfig(root, config);
            readCameraConfig(root, config);
            readMissileConfig(root, config);
            readTargetConfig(root, config);
            return config;
        }
    }

    std::filesystem::path resolveDefaultSimulationConfigPath()
    {
        const std::filesystem::path relativePath("config/default_simulation.json");
        const std::filesystem::path sourceAssetRoot(MISSILESIM_SOURCE_ASSET_DIR);

        const std::filesystem::path candidates[] = {
            relativePath,
            std::filesystem::current_path() / relativePath,
            std::filesystem::current_path() / "assets" / relativePath,
            sourceAssetRoot.empty() ? std::filesystem::path() : sourceAssetRoot / relativePath};

        for (const std::filesystem::path &candidate : candidates)
        {
            if (!candidate.empty() && std::filesystem::exists(candidate))
            {
                return candidate;
            }
        }

        return {};
    }

    SimulationConfigLoadResult loadSimulationConfig(const std::filesystem::path &path)
    {
        SimulationConfigLoadResult result;
        result.path = path;

        if (path.empty())
        {
            result.error = "No simulation config path was provided.";
            return result;
        }

        std::ifstream input(path);
        if (!input.is_open())
        {
            result.error = "Unable to open simulation config: " + path.string();
            return result;
        }

        try
        {
            const Json root = Json::parse(input);
            result.config = parseSimulationConfig(root);
            result.loaded = true;
        }
        catch (const std::exception &error)
        {
            result.error = "Unable to parse simulation config '" + path.string() + "': " + error.what();
        }

        return result;
    }

    SimulationConfigLoadResult loadDefaultSimulationConfig()
    {
        const std::filesystem::path path = resolveDefaultSimulationConfigPath();
        if (path.empty())
        {
            SimulationConfigLoadResult result;
            result.error = "Default simulation config not found.";
            return result;
        }

        return loadSimulationConfig(path);
    }
}
