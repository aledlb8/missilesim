#include "physics/Atmosphere.h"
#include "sim/Fox2Catalog.h"
#include "sim/Fox2Flight.h"

#include <cmath>
#include <iostream>
#include <string>

namespace
{
    missilesim::fox2::TailChaseRequest aim9bRequest(float altitudeM, float dragScale)
    {
        missilesim::fox2::TailChaseRequest request;
        request.altitudeM = altitudeM;
        request.dragScale = dragScale;
        request.launchMach = 1.2f;
        request.targetMach = 0.9f;
        request.maxTimeS = 24.0f;
        return request;
    }

    // Split the relative error of the two OP 2309 acceptance shots.
    float splitError(float seaLevelRange, float highRange)
    {
        constexpr float kSeaLevelPublished = 6000.0f * 0.3048f;
        constexpr float kHighPublished = 28000.0f * 0.3048f;
        const float seaError = std::abs(seaLevelRange - kSeaLevelPublished) / kSeaLevelPublished;
        const float highError = std::abs(highRange - kHighPublished) / kHighPublished;
        return 0.5f * (seaError + highError);
    }
}

int main()
{
    Atmosphere atmosphere;
    const auto sample = [&atmosphere](float altitude) {
        const Atmosphere::State state = atmosphere.sample(altitude);
        struct Air
        {
            float density;
            float pressure;
            float speedOfSound;
        };
        return Air{state.densityKgPerCubicMeter, state.pressurePascals, state.speedOfSoundMetersPerSecond};
    };

    float bestScale = 1.0f;
    float bestError = 1.0e9f;
    missilesim::fox2::TailChaseResult bestSea{};
    missilesim::fox2::TailChaseResult bestHigh{};

    // k is dimensionless on Fleeman's unscaled body Cd0. The accepted band is
    // about 0.2–0.6 after scaling, so k itself can sit well below 1.
    for (int step = 0; step <= 400; ++step)
    {
        const float scale = 0.05f + (1.95f * static_cast<float>(step) / 400.0f);
        const auto sea = missilesim::fox2::integrateTailChase(aim9bRequest(0.0f, scale), sample);
        const auto high = missilesim::fox2::integrateTailChase(aim9bRequest(15240.0f, scale), sample);
        const float error = splitError(sea.rangeM, high.rangeM);
        if (error < bestError)
        {
            bestError = error;
            bestScale = scale;
            bestSea = sea;
            bestHigh = high;
        }
    }

    constexpr float kSeaLevelPublished = 6000.0f * 0.3048f;
    constexpr float kHighPublished = 28000.0f * 0.3048f;
    const auto seaAir = atmosphere.sample(0.0f);
    const auto highAir = atmosphere.sample(15240.0f);
    const float seaSpeed = 1.5f * seaAir.speedOfSoundMetersPerSecond;
    const float seaQ = 0.5f * seaAir.densityKgPerCubicMeter * seaSpeed * seaSpeed;
    const float highSpeed = 2.0f * highAir.speedOfSoundMetersPerSecond;
    const float highQ = 0.5f * highAir.densityKgPerCubicMeter * highSpeed * highSpeed;
    const float seaCd = missilesim::fox2::scaledCd0(1.5f, seaQ, missilesim::fox2::kAim9bLengthM,
                                                    missilesim::fox2::kAim9bDiameterM, false, bestScale);
    const float highCd = missilesim::fox2::scaledCd0(2.0f, highQ, missilesim::fox2::kAim9bLengthM,
                                                     missilesim::fox2::kAim9bDiameterM, false, bestScale);

    std::cout << "scale " << bestScale << "\n";
    std::cout << "sea_m " << bestSea.rangeM << " published " << kSeaLevelPublished
              << " ratio " << (bestSea.rangeM / kSeaLevelPublished) << "\n";
    std::cout << "high_m " << bestHigh.rangeM << " published " << kHighPublished
              << " ratio " << (bestHigh.rangeM / kHighPublished) << "\n";
    std::cout << "sea_burnout_mps " << bestSea.burnoutSpeedMps
              << " high_burnout_mps " << bestHigh.burnoutSpeedMps << "\n";
    std::cout << "cd_sea_m1.5 " << seaCd << " cd_high_m2 " << highCd << "\n";
    std::cout << "split_error " << bestError << "\n";
    std::cout << "thrust_n " << missilesim::fox2::kAim9bThrustN
              << " mass_kg " << missilesim::fox2::kAim9bMassKg << "\n";

    const bool massOk = missilesim::fox2::kAim9bMassKg > 50.0f && missilesim::fox2::kAim9bMassKg < 100.0f;
    const bool burnOk = missilesim::fox2::kAim9bBurnS > 2.0f && missilesim::fox2::kAim9bBurnS < 2.4f;

    // Largest scale whose coast Cd0, outside the transonic peak, stays at or
    // under 0.60 at the Mach numbers a Sidewinder actually flies. flight-model.md:
    // if no scale inside 0.2–0.6 hits both ranges, report the residual.
    const float sampleMach[] = {1.2f, 1.5f, 2.0f, 2.5f, 3.0f};
    const float sampleAlt[] = {0.0f, 15240.0f};
    float peakUnscaled = 0.0f;
    float floorUnscaled = 1.0e9f;
    for (float altitude : sampleAlt)
    {
        const auto air = atmosphere.sample(altitude);
        for (float mach : sampleMach)
        {
            const float speed = mach * air.speedOfSoundMetersPerSecond;
            const float dynamicPressure = 0.5f * air.densityKgPerCubicMeter * speed * speed;
            const float unscaled = missilesim::fox2::fleemanBodyCd0(
                mach, dynamicPressure, missilesim::fox2::kAim9bLengthM, missilesim::fox2::kAim9bDiameterM, false);
            peakUnscaled = std::max(peakUnscaled, unscaled);
            floorUnscaled = std::min(floorUnscaled, unscaled);
            std::cout << "unscaled_cd h=" << altitude << " M=" << mach << " cd=" << unscaled << "\n";
        }
    }
    const float bandScale = (peakUnscaled > 1.0e-4f) ? (0.60f / peakUnscaled) : 1.0f;
    const auto bandSea = missilesim::fox2::integrateTailChase(aim9bRequest(0.0f, bandScale), sample);
    const auto bandHigh = missilesim::fox2::integrateTailChase(aim9bRequest(15240.0f, bandScale), sample);
    const float bandSeaCd = missilesim::fox2::scaledCd0(1.5f, seaQ, missilesim::fox2::kAim9bLengthM,
                                                        missilesim::fox2::kAim9bDiameterM, false, bandScale);
    const float bandHighCd = missilesim::fox2::scaledCd0(2.0f, highQ, missilesim::fox2::kAim9bLengthM,
                                                         missilesim::fox2::kAim9bDiameterM, false, bandScale);
    std::cout << "band_scale " << bandScale << "\n";
    std::cout << "band_sea_m " << bandSea.rangeM << " ratio " << (bandSea.rangeM / kSeaLevelPublished) << "\n";
    std::cout << "band_high_m " << bandHigh.rangeM << " ratio " << (bandHigh.rangeM / kHighPublished) << "\n";
    std::cout << "band_cd_sea_m1.5 " << bandSeaCd << " band_cd_high_m2 " << bandHighCd << "\n";
    std::cout << "band_floor_unscaled " << floorUnscaled << "\n";

    // Unit-error gate. A swapped foot/metre or lbf/newton misses by far more
    // than the aero residual below. The straight Cl=0 integral does not hit
    // OP 2309 inside the 0.2–0.6 coast band; that residual is printed, not forced.
    const bool bandScaleOk = bandSeaCd <= 0.61f && bandHighCd <= 0.61f && bandSeaCd >= 0.15f && bandHighCd >= 0.15f;

    // Every catalog round must have a real airframe. Brochure ranges are not
    // tail-chase targets, so a short stand-in motor is a residual, not a failure.
    // An absurd ratio fails only when publishedMaxRangeM is actually set.
    bool catalogOk = true;
    const int roundCount = missilesim::fox2::catalogCount();
    const missilesim::fox2::Spec *rounds = missilesim::fox2::catalog();
    std::cout << "catalog_count " << roundCount << "\n";
    for (int index = 0; index < roundCount; ++index)
    {
        const missilesim::fox2::Spec &spec = rounds[index];
        const char *id = (spec.id != nullptr && spec.id[0] != '\0') ? spec.id : "?";
        const bool named = spec.id != nullptr && spec.id[0] != '\0' && spec.displayName != nullptr && spec.displayName[0] != '\0';
        const bool geometry = spec.massKg > 0.0f && spec.lengthM > 0.0f && spec.diameterM > 0.0f &&
                              spec.resolvedBurnS > 0.0f && spec.resolvedThrustN > 0.0f && spec.resolvedCnMax > 0.0f;
        const bool finite = std::isfinite(spec.massKg) && std::isfinite(spec.lengthM) && std::isfinite(spec.diameterM) &&
                            std::isfinite(spec.resolvedBurnS) && std::isfinite(spec.resolvedThrustN) &&
                            std::isfinite(spec.resolvedCnMax) && std::isfinite(spec.resolvedBurnG) &&
                            std::isfinite(spec.resolvedCoastG);
        if (!named || !geometry || !finite)
        {
            catalogOk = false;
            std::cerr << "UNIT " << id << "\n";
        }

        missilesim::fox2::TailChaseRequest request;
        request.altitudeM = 5000.0f;
        request.launchMach = 0.9f;
        request.targetMach = 0.8f;
        request.massKg = std::max(spec.massKg, 0.1f);
        request.propellantKg = spec.propellantKg;
        request.diameterM = std::max(spec.diameterM, 0.01f);
        request.lengthM = std::max(spec.lengthM, 0.1f);
        request.seaLevelThrustN = spec.resolvedThrustN;
        request.burnTimeS = spec.resolvedBurnS;
        request.dragScale = missilesim::fox2::kAim9bDragScale;
        request.maxTimeS = 80.0f;
        const auto flyout = missilesim::fox2::integrateTailChase(request, sample);
        std::cout << "flyout " << id << " range_m " << flyout.rangeM;
        if (spec.publishedMaxRangeM > 0.0f)
        {
            const float ratio = flyout.rangeM / spec.publishedMaxRangeM;
            std::cout << " brochure_m " << spec.publishedMaxRangeM << " ratio " << ratio;
            if (!std::isfinite(ratio) || ratio > 50.0f || ratio < 0.02f)
            {
                catalogOk = false;
                std::cerr << "RATIO " << id << " " << ratio << "\n";
            }
        }
        else
        {
            std::cout << " brochure none";
        }
        std::cout << " RESIDUAL\n";
        if (!std::isfinite(flyout.rangeM) || flyout.rangeM < 0.0f)
        {
            catalogOk = false;
            std::cerr << "FLYOUT " << id << "\n";
        }
    }

    if (!massOk || !burnOk || !bandScaleOk || !catalogOk || !std::isfinite(bandSea.rangeM) || bandSea.rangeM <= 0.0f)
    {
        std::cerr << "FAILED mass=" << massOk << " burn=" << burnOk << " band=" << bandScaleOk
                  << " catalog=" << catalogOk << "\n";
        return 1;
    }

    std::cout << "RESIDUAL sea_ratio " << (bandSea.rangeM / kSeaLevelPublished)
              << " high_ratio " << (bandHigh.rangeM / kHighPublished) << "\n";
    std::cout << "OK\n";
    return 0;
}
