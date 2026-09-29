#include "HandlingChecks.h"

#include "flight/Jet.h"
#include "objects/Fighter.h"
#include "physics/Atmosphere.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    using namespace missilesim::flight;
    constexpr float kGravity = 9.80665f;
    constexpr float kStep = 0.01f;

    struct Checks
    {
        int failures = 0;
        int count = 0;

        void require(bool passed, const std::string &name, float measured, float limit)
        {
            ++count;
            if (!passed)
            {
                ++failures;
                std::printf("FAIL %s: measured %.4f, limit %.4f\n", name.c_str(), measured, limit);
            }
        }

        void below(float measured, float limit, const std::string &name)
        {
            require(std::isfinite(measured) && measured <= limit, name, measured, limit);
        }
    };

    glm::vec3 direction(float azimuth, float elevation)
    {
        const float az = glm::radians(azimuth);
        const float el = glm::radians(elevation);
        return {-std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el)};
    }

    float angleDegrees(const glm::vec3 &a, const glm::vec3 &b)
    {
        return glm::degrees(std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b)));
    }

    void step(Jet &jet, const JetControls &controls, float sampleTime = kStep)
    {
        const auto air = Atmosphere().sample(jet.airframe().position().y);
        jet.setControls(controls, sampleTime);
        jet.step(kStep, {air.densityKgPerCubicMeter, air.speedOfSoundMetersPerSecond}, kGravity);
    }

    Jet trimmedJet(float speed = 250.0f)
    {
        Jet jet;
        jet.reset({0.0f, 3000.0f, 0.0f}, {0.0f, 0.0f, speed}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f});
        JetControls controls;
        controls.throttle = 1.0f;
        for (int i = 0; i < 200; ++i)
        {
            step(jet, controls);
        }
        return jet;
    }

    void checkSmallCorrections(Checks &checks)
    {
        float worstRoll = 0.0f;
        for (float speed : {120.0f, 250.0f, 400.0f})
        {
            for (float elevation : {-20.0f, -5.0f, -2.0f, -1.0f, 1.0f, 5.0f})
            {
                Jet jet = trimmedJet(speed);
                JetControls controls = jet.controls();
                controls.instructor.aimDirection = direction(0.0f, elevation);
                float maxRoll = 0.0f;
                float maxBank = 0.0f;
                for (int i = 0; i < 600; ++i)
                {
                    step(jet, controls);
                    const auto &airframe = jet.airframe();
                    maxRoll = std::max(maxRoll, std::abs(glm::degrees(airframe.bodyRates().x)));
                    maxBank = std::max(maxBank, std::abs(glm::degrees(std::atan2(airframe.right().y, airframe.up().y))));
                }
                const std::string label = "small correction V=" + std::to_string(speed) + " el=" + std::to_string(elevation);
                checks.below(maxRoll, 5.0f, label + " roll rate");
                checks.below(maxBank, 2.0f, label + " bank");
                checks.below(angleDegrees(jet.airframe().forward(), controls.instructor.aimDirection), 0.3f, label + " capture");
                worstRoll = std::max(worstRoll, maxRoll);
            }
        }
        std::printf("Small vertical corrections: worst roll rate %.3f deg/s\n", worstRoll);
    }

    void checkLargeManeuvers(Checks &checks)
    {
        const glm::vec2 aims[] = {{30.0f, 0.0f}, {-90.0f, 0.0f}, {90.0f, 0.0f},
                                 {-150.0f, 0.0f}, {150.0f, 0.0f}, {0.0f, -40.0f}, {0.0f, 80.0f}};
        for (const auto &aim : aims)
        {
            Jet jet = trimmedJet();
            JetControls controls = jet.controls();
            controls.instructor.aimDirection = direction(aim.x, aim.y);
            float maxLoad = 0.0f;
            float maxAlpha = 0.0f;
            // Start with a stationary designation to isolate maneuver capture
            // from the additional feed-forward of a mouse flick.
            jet.setControls(controls, 0.0f);
            for (int i = 0; i < 1200; ++i)
            {
                step(jet, controls);
                maxLoad = std::max(maxLoad, jet.airframe().telemetry().normalLoad);
                maxAlpha = std::max(maxAlpha, glm::degrees(jet.airframe().telemetry().alpha));
            }
            const std::string label = "large maneuver az=" + std::to_string(aim.x) + " el=" + std::to_string(aim.y);
            checks.below(angleDegrees(jet.airframe().forward(), controls.instructor.aimDirection), 1.0f, label + " capture");
            checks.below(maxLoad, 9.5f, label + " g protection");
            checks.below(maxAlpha, 27.0f, label + " AoA protection");
        }
    }

    void checkKeyboardHandoff(Checks &checks)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            for (float sign : {-1.0f, 1.0f})
            {
                Jet jet = trimmedJet();
                JetControls controls = jet.controls();
                // Keep the mouse fixed in world space, as it is in the game.
                // The old harness instead moved it with the aircraft's nose.
                float *key = axis == 0 ? &controls.instructor.pitchKey : axis == 1 ? &controls.instructor.rollKey : &controls.instructor.yawKey;
                *key = sign;
                for (int i = 0; i < 100; ++i)
                {
                    step(jet, controls);
                }
                const auto &airframe = jet.airframe();
                const float measuredPitch = glm::degrees(airframe.bodyRates().y);
                const float measuredRoll = glm::degrees(airframe.bodyRates().x / std::cos(airframe.telemetry().alpha));
                const float measuredSlip = glm::degrees(airframe.telemetry().beta);
                *key = 0.0f;
                step(jet, controls);
                const std::string label = "keyboard axis=" + std::to_string(axis) + " sign=" + std::to_string(sign);
                if (axis == 0)
                {
                    checks.below(std::abs(glm::degrees(jet.lastCommand().pitchRate) - measuredPitch), 5.0f, label + " pitch handoff");
                }
                if (axis <= 1)
                {
                    checks.below(std::abs(glm::degrees(jet.lastCommand().rollRate) - measuredRoll), 5.0f, label + " roll handoff");
                }
                if (axis == 2)
                {
                    checks.below(std::abs(glm::degrees(jet.lastCommand().sideslip) - measuredSlip), 0.2f, label + " yaw handoff");
                }
                for (int i = 0; i < 1000; ++i)
                {
                    step(jet, controls);
                }
                checks.below(angleDegrees(jet.airframe().forward(), controls.instructor.aimDirection), 1.0f, label + " reacquire mouse");

                // A fresh manual command must take effect without waiting
                // for a release blend to finish.
                *key = sign;
                step(jet, controls);
                *key = 0.0f;
                step(jet, controls);
                *key = -sign;
                step(jet, controls);
                const PilotCommand command = jet.lastCommand();
                const float response = axis == 0 ? command.pitchRate : axis == 1 ? command.rollRate : -command.sideslip;
                checks.require(response * sign < 0.0f, label + " immediate manual re-engagement", response, 0.0f);
            }
        }
    }

    void checkAimSampling(Checks &checks)
    {
        const Jet jet = trimmedJet();
        for (int fps : {30, 60, 100, 144, 240})
        {
            MouseAimInstructor moving;
            MouseAimInstructor stationary;
            InstructorInput input;
            const float sampleTime = 1.0f / static_cast<float>(fps);
            float worstError = 0.0f;
            for (int frame = 0; frame <= fps * 2; ++frame)
            {
                input.aimDirection = direction(0.0f, 12.0f * static_cast<float>(frame) * sampleTime);
                moving.observeAim(input.aimDirection, sampleTime);
                stationary.observeAim(input.aimDirection, 0.0f);
                const auto withMotion = moving.update(jet.airframe(), input, kGravity, kStep);
                const auto withoutMotion = stationary.update(jet.airframe(), input, kGravity, kStep);
                if (frame >= fps)
                {
                    const float rate = glm::degrees(withMotion.pitchRate - withoutMotion.pitchRate);
                    worstError = std::max(worstError, std::abs(rate - 12.0f));
                }
            }
            checks.below(worstError, 0.02f, "aim velocity at " + std::to_string(fps) + " FPS");
            std::printf("Aim sampling %3d FPS: worst rate error %.4f deg/s\n", fps, worstError);

            // Paused or recentered input cannot carry an old velocity into
            // the first resumed step, even if the aim changed substantially.
            input.aimDirection = direction(0.0f, -5.0f);
            moving.observeAim(input.aimDirection, 0.0f);
            stationary.observeAim(input.aimDirection, 0.0f);
            const auto resumed = moving.update(jet.airframe(), input, kGravity, kStep);
            const auto fresh = stationary.update(jet.airframe(), input, kGravity, kStep);
            checks.below(std::abs(resumed.pitchRate - fresh.pitchRate), 1.0e-6f, "pause/recenter clears aim motion");
        }
    }

    struct TrackingResult
    {
        glm::vec3 nose;
        glm::vec3 position;
        float maxError = 0.0f;
        bool finite = true;
    };

    TrackingResult trackAtFrameRate(int fps, bool jitter, float timeScale, float physicsStep)
    {
        Fighter fighter;
        fighter.place({0.0f, 3000.0f, 0.0f}, {0.0f, 0.0f, 250.0f}, {0.0f, 0.0f, 1.0f});
        InstructorInput input;
        fighter.setInstructorInput(input, 0.0f);
        TrackingResult result;
        double time = 0.0;
        double accumulator = 0.0;
        int frame = 0;
        while (time < 10.0 - 1.0e-8)
        {
            // Alternating short/long frames exercise multiple physics steps
            // per sample and samples with no physics step at all.
            const double factor = jitter ? (frame % 2 == 0 ? 0.5 : 1.5) : 1.0;
            const double sampleTime = std::min(factor * timeScale / fps, 10.0 - time);
            time += sampleTime;
            input.aimDirection = direction(12.0f * static_cast<float>(time), 0.0f);
            fighter.setInstructorInput(input, static_cast<float>(sampleTime));
            accumulator += sampleTime;
            while (accumulator + 1.0e-8 >= physicsStep)
            {
                const auto air = Atmosphere().sample(fighter.getPosition().y);
                fighter.updateFlight(physicsStep, air.densityKgPerCubicMeter, air.speedOfSoundMetersPerSecond, kGravity);
                accumulator -= physicsStep;
                const float error = angleDegrees(fighter.getNose(), input.aimDirection);
                result.finite = result.finite && std::isfinite(error) && std::isfinite(fighter.getPosition().y);
                if (time >= 2.0)
                {
                    result.maxError = std::max(result.maxError, error);
                }
            }
            ++frame;
        }
        result.nose = fighter.getNose();
        result.position = fighter.getPosition();
        return result;
    }

    void checkFrameRateConsistency(Checks &checks)
    {
        const auto reference = trackAtFrameRate(100, false, 1.0f, kStep);
        for (int fps : {30, 60, 144, 240})
        {
            for (bool jitter : {false, true})
            {
                const auto result = trackAtFrameRate(fps, jitter, 1.0f, kStep);
                const std::string label = std::to_string(fps) + " FPS" + (jitter ? " uneven" : " steady");
                checks.require(result.finite, label + " finite", result.finite ? 0.0f : 1.0f, 0.0f);
                checks.below(angleDegrees(result.nose, reference.nose), 0.5f, label + " nose vs 100 FPS");
                checks.below(glm::distance(result.position, reference.position), 15.0f, label + " position vs 100 FPS");
                checks.below(result.maxError, 2.0f, label + " tracking error after capture");
                std::printf("Tracking %-15s nose difference %.3f deg, position difference %.2f m, max error %.3f deg\n",
                            label.c_str(), angleDegrees(result.nose, reference.nose), glm::distance(result.position, reference.position), result.maxError);
            }
        }
        // Time scale changes the sample interval in simulation seconds.
        // Equivalent simulation-time sampling must give equivalent handling.
        for (float scale : {0.5f, 2.0f})
        {
            const auto scaled = trackAtFrameRate(static_cast<int>(100.0f * scale), false, scale, kStep);
            checks.below(angleDegrees(scaled.nose, reference.nose), 0.01f, "simulation time scale nose");
            checks.below(glm::distance(scaled.position, reference.position), 0.1f, "simulation time scale position");
        }
        for (float physicsStep : {0.005f, 0.02f})
        {
            const auto result = trackAtFrameRate(60, true, 1.0f, physicsStep);
            checks.below(angleDegrees(result.nose, reference.nose), 0.5f, "independent physics timestep nose");
            checks.below(glm::distance(result.position, reference.position), 15.0f, "independent physics timestep position");
        }
    }
}

int runHandlingChecks()
{
    Checks checks;
    checkSmallCorrections(checks);
    checkLargeManeuvers(checks);
    checkKeyboardHandoff(checks);
    checkAimSampling(checks);
    checkFrameRateConsistency(checks);
    std::printf("Handling checks: %d passed, %d failed\n", checks.count - checks.failures, checks.failures);
    return checks.failures == 0 ? 0 : 1;
}
