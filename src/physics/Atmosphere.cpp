#include "Atmosphere.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace
{
constexpr double kEarthRadiusMeters = 6356766.0;
constexpr double kStandardGravityMetersPerSecondSquared = 9.80665;
constexpr double kSpecificGasConstantAir = 287.05287;
constexpr double kHeatCapacityRatioAir = 1.4;
constexpr double kSeaLevelTemperatureKelvin = 288.15;
constexpr double kSeaLevelPressurePascals = 101325.0;
constexpr double kSutherlandReferenceTemperatureKelvin = 273.15;
constexpr double kSutherlandReferenceViscosityPascalSeconds = 1.716e-5;
constexpr double kSutherlandConstantKelvin = 110.4;

constexpr std::array<double, 7> kLayerTopGeopotentialAltitudesMeters = {
    11000.0,
    20000.0,
    32000.0,
    47000.0,
    51000.0,
    71000.0,
    84852.0
};

constexpr std::array<double, 7> kLayerLapseRatesKelvinPerMeter = {
    -0.0065,
    0.0,
    0.0010,
    0.0028,
    0.0,
    -0.0028,
    -0.0020
};

double clampGeometricAltitude(double altitudeMeters)
{
    return std::clamp(
        altitudeMeters,
        static_cast<double>(Atmosphere::kMinimumSupportedAltitudeMeters),
        static_cast<double>(Atmosphere::kMaximumSupportedAltitudeMeters));
}

double toGeopotentialAltitude(double geometricAltitudeMeters)
{
    return (kEarthRadiusMeters * geometricAltitudeMeters) / (kEarthRadiusMeters + geometricAltitudeMeters);
}

double calculateLayerPressure(
    double basePressurePascals,
    double baseTemperatureKelvin,
    double lapseRateKelvinPerMeter,
    double baseAltitudeMeters,
    double targetAltitudeMeters)
{
    const double deltaAltitudeMeters = targetAltitudeMeters - baseAltitudeMeters;
    if (std::abs(lapseRateKelvinPerMeter) < 1e-12)
    {
        return basePressurePascals *
               std::exp((-kStandardGravityMetersPerSecondSquared * deltaAltitudeMeters) /
                        (kSpecificGasConstantAir * baseTemperatureKelvin));
    }

    const double targetTemperatureKelvin = baseTemperatureKelvin + (lapseRateKelvinPerMeter * deltaAltitudeMeters);
    return basePressurePascals *
           std::pow(baseTemperatureKelvin / targetTemperatureKelvin,
                    kStandardGravityMetersPerSecondSquared / (kSpecificGasConstantAir * lapseRateKelvinPerMeter));
}

double calculateDynamicViscosity(double temperatureKelvin)
{
    return kSutherlandReferenceViscosityPascalSeconds *
           std::pow(temperatureKelvin / kSutherlandReferenceTemperatureKelvin, 1.5) *
           ((kSutherlandReferenceTemperatureKelvin + kSutherlandConstantKelvin) /
            (temperatureKelvin + kSutherlandConstantKelvin));
}
} // namespace

Atmosphere::Atmosphere(float density)
    : m_densityScale(1.0f)
{
    setDensity(density);
}

float Atmosphere::calculateDensityAtAltitude(float altitude) const
{
    return sample(altitude).densityKgPerCubicMeter;
}

float Atmosphere::getDensity() const
{
    return kStandardSeaLevelDensity * m_densityScale;
}

void Atmosphere::setDensity(float density)
{
    if (density <= 0.0f)
    {
        m_densityScale = 0.0f;
        return;
    }

    m_densityScale = density / kStandardSeaLevelDensity;
}

void Atmosphere::setDensityScale(float densityScale)
{
    m_densityScale = std::max(densityScale, 0.0f);
}

Atmosphere::State Atmosphere::sample(float altitude) const
{
    const double finiteAltitudeMeters = std::isfinite(static_cast<double>(altitude)) ? static_cast<double>(altitude) : 0.0;
    const double geometricAltitudeMeters = clampGeometricAltitude(finiteAltitudeMeters);
    const double geopotentialAltitudeMeters = toGeopotentialAltitude(geometricAltitudeMeters);

    double baseAltitudeMeters = 0.0;
    double baseTemperatureKelvin = kSeaLevelTemperatureKelvin;
    double basePressurePascals = kSeaLevelPressurePascals;
    size_t layerIndex = 0;

    for (; layerIndex < kLayerTopGeopotentialAltitudesMeters.size(); ++layerIndex)
    {
        const double layerTopAltitudeMeters = kLayerTopGeopotentialAltitudesMeters[layerIndex];
        if (geopotentialAltitudeMeters <= layerTopAltitudeMeters)
        {
            break;
        }

        const double lapseRateKelvinPerMeter = kLayerLapseRatesKelvinPerMeter[layerIndex];
        const double nextTemperatureKelvin =
            baseTemperatureKelvin + (lapseRateKelvinPerMeter * (layerTopAltitudeMeters - baseAltitudeMeters));
        basePressurePascals = calculateLayerPressure(
            basePressurePascals,
            baseTemperatureKelvin,
            lapseRateKelvinPerMeter,
            baseAltitudeMeters,
            layerTopAltitudeMeters);
        baseAltitudeMeters = layerTopAltitudeMeters;
        baseTemperatureKelvin = nextTemperatureKelvin;
    }

    if (layerIndex >= kLayerLapseRatesKelvinPerMeter.size())
    {
        layerIndex = kLayerLapseRatesKelvinPerMeter.size() - 1;
    }

    const double lapseRateKelvinPerMeter = kLayerLapseRatesKelvinPerMeter[layerIndex];
    const double temperatureKelvin =
        baseTemperatureKelvin + (lapseRateKelvinPerMeter * (geopotentialAltitudeMeters - baseAltitudeMeters));
    const double pressurePascals =
        calculateLayerPressure(
            basePressurePascals,
            baseTemperatureKelvin,
            lapseRateKelvinPerMeter,
            baseAltitudeMeters,
            geopotentialAltitudeMeters) * static_cast<double>(m_densityScale);
    const double densityKgPerCubicMeter =
        (pressurePascals > 0.0 && temperatureKelvin > 0.0)
            ? pressurePascals / (kSpecificGasConstantAir * temperatureKelvin)
            : 0.0;
    const double speedOfSoundMetersPerSecond =
        std::sqrt(std::max(0.0, kHeatCapacityRatioAir * kSpecificGasConstantAir * temperatureKelvin));
    const double dynamicViscosityPascalSeconds = calculateDynamicViscosity(temperatureKelvin);
    const double kinematicViscositySquareMetersPerSecond =
        (densityKgPerCubicMeter > 1e-12) ? (dynamicViscosityPascalSeconds / densityKgPerCubicMeter) : 0.0;

    State state;
    state.geometricAltitudeMeters = static_cast<float>(geometricAltitudeMeters);
    state.geopotentialAltitudeMeters = static_cast<float>(geopotentialAltitudeMeters);
    state.temperatureKelvin = static_cast<float>(temperatureKelvin);
    state.pressurePascals = static_cast<float>(pressurePascals);
    state.densityKgPerCubicMeter = static_cast<float>(densityKgPerCubicMeter);
    state.speedOfSoundMetersPerSecond = static_cast<float>(speedOfSoundMetersPerSecond);
    state.dynamicViscosityPascalSeconds = static_cast<float>(dynamicViscosityPascalSeconds);
    state.kinematicViscositySquareMetersPerSecond = static_cast<float>(kinematicViscositySquareMetersPerSecond);
    return state;
}
