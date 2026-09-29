#pragma once

// Low-speed F-16 aerodynamic and engine data.
//
// Source: NASA TP-1538 (Nguyen, Ogburn, Gilbert, Kibler, Brown, Deal, 1979,
// "Simulator Study of Stall/Post-Stall Characteristics of a Fighter Airplane
// With Relaxed Longitudinal Static Stability"), a US Government work, in the
// reduced form tabulated in Stevens & Lewis, "Aircraft Control and
// Simulation", Appendix A. The tables were cross-checked value for value
// between two independent transcriptions before being generated into
// F16Data.cpp.
//
// Conventions: body axes x forward, y right wing, z down. Angles in degrees,
// body rates in rad/s. Coefficients are referenced to the wing (S, b, cbar).
// The data are low-speed wind-tunnel data: there is no Mach effect on lift or
// moments. The transonic zero-lift drag rise is added separately by the
// airframe from the Brandt F-16 polar (see F16Airframe.cpp).
namespace missilesim::flight::f16
{
    // NASA TP-1538 table I.
    constexpr float kWingAreaM2 = 27.87f;      // 300 ft^2
    constexpr float kSpanM = 9.144f;           // 30 ft
    constexpr float kChordM = 3.45f;           // mean aerodynamic chord, 11.32 ft
    constexpr float kReferenceCg = 0.35f;      // fraction of cbar
    constexpr float kEngineMomentum = 216.9f;  // engine angular momentum, kg m^2/s (TP-1538 appendix B)
    constexpr float kTestWeightKg = 9298.6f;   // 20,500 lb, the weight the inertias below belong to
    constexpr float kIxx = 12875.0f;           // kg m^2
    constexpr float kIyy = 75674.0f;
    constexpr float kIzz = 85552.0f;
    constexpr float kIxz = 1331.0f;

    // Surface limits, TP-1538 table I and appendix A.
    constexpr float kElevatorLimitDeg = 25.0f;
    constexpr float kAileronLimitDeg = 21.5f;
    constexpr float kRudderLimitDeg = 30.0f;

    // Range the tables cover. Lookups clamp to it rather than extrapolate.
    constexpr float kAlphaMinDeg = -10.0f;
    constexpr float kAlphaMaxDeg = 45.0f;
    constexpr float kBetaLimitDeg = 30.0f;

    struct Damping
    {
        float cxq, cyr, cyp, czq, clr, clp, cmq, cnr, cnp;
    };

    float cx(float alphaDeg, float elevatorDeg);
    float cy(float betaDeg, float aileronDeg, float rudderDeg);
    float cz(float alphaDeg, float betaDeg, float elevatorDeg);
    float cm(float alphaDeg, float elevatorDeg);
    float cl(float alphaDeg, float betaDeg);
    float cn(float alphaDeg, float betaDeg);
    // Control derivatives per unit normalised deflection (aileron / 20 deg,
    // rudder / 30 deg), as the source tabulates them.
    float dlda(float alphaDeg, float betaDeg);
    float dldr(float alphaDeg, float betaDeg);
    float dnda(float alphaDeg, float betaDeg);
    float dndr(float alphaDeg, float betaDeg);
    Damping damping(float alphaDeg);

    // Engine (TP-1538 appendix B / table VI, F100-PW-200 installed deck).
    // Power is 0..100 %: 0..50 idle to military, 50..100 afterburner.
    float throttleGearing(float throttle);                         // lever 0..1 -> commanded power
    float powerRate(float power, float commandedPower);            // first-order spool lag, %/s
    float thrustLbf(float power, float altitudeFt, float mach, float milScale, float maxScale);
}
