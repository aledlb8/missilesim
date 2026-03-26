#pragma once

class Atmosphere
{
public:
    struct State
    {
        float geometricAltitudeMeters = 0.0f;
        float geopotentialAltitudeMeters = 0.0f;
        float temperatureKelvin = 288.15f;
        float pressurePascals = 101325.0f;
        float densityKgPerCubicMeter = 1.225f;
        float speedOfSoundMetersPerSecond = 340.294f;
        float dynamicViscosityPascalSeconds = 1.7894e-5f;
        float kinematicViscositySquareMetersPerSecond = 1.4607e-5f;
    };

    static constexpr float kStandardSeaLevelDensity = 1.225f;
    static constexpr float kMinimumSupportedAltitudeMeters = -5000.0f;
    static constexpr float kMaximumSupportedAltitudeMeters = 86000.0f;

    Atmosphere(float density = kStandardSeaLevelDensity);
    ~Atmosphere() = default;

    // Sea-level reference density. The sampled atmosphere is scaled from ISA using this value.
    float getDensity() const;
    void setDensity(float density);

    float getDensityScale() const { return m_densityScale; }
    void setDensityScale(float densityScale);

    State sample(float altitude) const;
    float calculateDensityAtAltitude(float altitude) const;

private:
    float m_densityScale;
};