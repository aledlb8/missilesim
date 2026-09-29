#include "F16Data.h"

#include <algorithm>
#include <cmath>

// Tables generated from NASA TP-1538 data as tabulated by Stevens & Lewis
// (see F16Data.h). Do not hand-edit values; regenerate them instead.
namespace missilesim::flight::f16
{
namespace
{
    // X-force coefficient, rows elevator -24..24 deg step 12, columns alpha -10..45 deg step 5
    constexpr float kCx[5][12] = {
        {-0.099f, -0.081f, -0.081f, -0.063f, -0.025f, 0.044f, 0.097f, 0.113f, 0.145f, 0.167f, 0.174f, 0.166f},
        {-0.048f, -0.038f, -0.04f, -0.021f, 0.016f, 0.083f, 0.127f, 0.137f, 0.162f, 0.177f, 0.179f, 0.167f},
        {-0.022f, -0.02f, -0.021f, -0.004f, 0.032f, 0.094f, 0.128f, 0.13f, 0.154f, 0.161f, 0.155f, 0.138f},
        {-0.04f, -0.038f, -0.039f, -0.025f, 0.006f, 0.062f, 0.087f, 0.085f, 0.1f, 0.11f, 0.104f, 0.091f},
        {-0.083f, -0.073f, -0.076f, -0.072f, -0.046f, 0.012f, 0.024f, 0.025f, 0.043f, 0.053f, 0.047f, 0.04f},
    };

    // Z-force coefficient at zero sideslip and elevator, alpha -10..45 deg step 5
    constexpr float kCz[1][12] = {
        {0.77f, 0.241f, -0.1f, -0.415f, -0.731f, -1.053f, -1.355f, -1.646f, -1.917f, -2.12f, -2.248f, -2.229f},
    };

    // Pitching-moment coefficient, rows elevator -24..24 deg step 12, columns alpha
    constexpr float kCm[5][12] = {
        {0.205f, 0.168f, 0.186f, 0.196f, 0.213f, 0.251f, 0.245f, 0.238f, 0.252f, 0.231f, 0.198f, 0.192f},
        {0.081f, 0.077f, 0.107f, 0.11f, 0.11f, 0.141f, 0.127f, 0.119f, 0.133f, 0.108f, 0.081f, 0.093f},
        {-0.046f, -0.02f, -0.009f, -0.005f, -0.006f, 0.01f, 0.006f, -0.001f, 0.014f, 0.0f, -0.013f, 0.032f},
        {-0.174f, -0.145f, -0.121f, -0.127f, -0.129f, -0.102f, -0.097f, -0.113f, -0.087f, -0.084f, -0.069f, -0.006f},
        {-0.259f, -0.202f, -0.184f, -0.193f, -0.199f, -0.15f, -0.16f, -0.167f, -0.104f, -0.076f, -0.041f, -0.005f},
    };

    // Rolling-moment coefficient, rows |beta| 0..30 deg step 5, columns alpha (odd in beta)
    constexpr float kCl[7][12] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {-0.001f, -0.004f, -0.008f, -0.012f, -0.016f, -0.022f, -0.022f, -0.021f, -0.015f, -0.008f, -0.013f, -0.015f},
        {-0.003f, -0.009f, -0.017f, -0.024f, -0.03f, -0.041f, -0.045f, -0.04f, -0.016f, -0.002f, -0.01f, -0.019f},
        {-0.001f, -0.01f, -0.02f, -0.03f, -0.039f, -0.054f, -0.057f, -0.054f, -0.023f, -0.006f, -0.014f, -0.027f},
        {0.0f, -0.01f, -0.022f, -0.034f, -0.047f, -0.06f, -0.069f, -0.067f, -0.033f, -0.036f, -0.035f, -0.035f},
        {0.007f, -0.01f, -0.023f, -0.034f, -0.049f, -0.063f, -0.081f, -0.079f, -0.06f, -0.058f, -0.062f, -0.059f},
        {0.009f, -0.011f, -0.023f, -0.037f, -0.05f, -0.068f, -0.089f, -0.088f, -0.091f, -0.076f, -0.077f, -0.076f},
    };

    // Yawing-moment coefficient, rows |beta| 0..30 deg step 5, columns alpha (odd in beta)
    constexpr float kCn[7][12] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.018f, 0.019f, 0.018f, 0.019f, 0.019f, 0.018f, 0.013f, 0.007f, 0.004f, -0.014f, -0.017f, -0.033f},
        {0.038f, 0.042f, 0.042f, 0.042f, 0.043f, 0.039f, 0.03f, 0.017f, 0.004f, -0.035f, -0.047f, -0.057f},
        {0.056f, 0.057f, 0.059f, 0.058f, 0.058f, 0.053f, 0.032f, 0.012f, 0.002f, -0.046f, -0.071f, -0.073f},
        {0.064f, 0.077f, 0.076f, 0.074f, 0.073f, 0.057f, 0.029f, 0.007f, 0.012f, -0.034f, -0.065f, -0.041f},
        {0.074f, 0.086f, 0.093f, 0.089f, 0.08f, 0.062f, 0.049f, 0.022f, 0.028f, -0.012f, -0.002f, -0.013f},
        {0.079f, 0.09f, 0.106f, 0.106f, 0.096f, 0.08f, 0.068f, 0.03f, 0.064f, 0.015f, 0.011f, -0.001f},
    };

    // Roll moment per unit aileron (deflection/20 deg), rows beta -30..30 deg step 10, columns alpha
    constexpr float kDlda[7][12] = {
        {-0.041f, -0.052f, -0.053f, -0.056f, -0.05f, -0.056f, -0.082f, -0.059f, -0.042f, -0.038f, -0.027f, -0.017f},
        {-0.041f, -0.053f, -0.053f, -0.053f, -0.05f, -0.051f, -0.066f, -0.043f, -0.038f, -0.027f, -0.023f, -0.016f},
        {-0.042f, -0.053f, -0.052f, -0.051f, -0.049f, -0.049f, -0.043f, -0.035f, -0.026f, -0.016f, -0.018f, -0.014f},
        {-0.04f, -0.052f, -0.051f, -0.052f, -0.048f, -0.048f, -0.042f, -0.037f, -0.031f, -0.026f, -0.017f, -0.012f},
        {-0.043f, -0.049f, -0.048f, -0.049f, -0.043f, -0.042f, -0.042f, -0.036f, -0.025f, -0.021f, -0.016f, -0.011f},
        {-0.044f, -0.048f, -0.048f, -0.047f, -0.042f, -0.041f, -0.02f, -0.028f, -0.013f, -0.014f, -0.011f, -0.01f},
        {-0.043f, -0.049f, -0.047f, -0.045f, -0.042f, -0.037f, -0.003f, -0.013f, -0.01f, -0.003f, -0.007f, -0.008f},
    };

    // Roll moment per unit rudder (deflection/30 deg), rows beta -30..30 deg step 10, columns alpha
    constexpr float kDldr[7][12] = {
        {0.005f, 0.017f, 0.014f, 0.01f, -0.005f, 0.009f, 0.019f, 0.005f, -0.0f, -0.005f, -0.011f, 0.008f},
        {0.007f, 0.016f, 0.014f, 0.014f, 0.013f, 0.009f, 0.012f, 0.005f, 0.0f, 0.004f, 0.009f, 0.007f},
        {0.013f, 0.013f, 0.011f, 0.012f, 0.011f, 0.009f, 0.008f, 0.005f, -0.002f, 0.005f, 0.003f, 0.005f},
        {0.018f, 0.015f, 0.015f, 0.014f, 0.014f, 0.014f, 0.014f, 0.015f, 0.013f, 0.011f, 0.006f, 0.001f},
        {0.015f, 0.014f, 0.013f, 0.013f, 0.012f, 0.011f, 0.011f, 0.01f, 0.008f, 0.008f, 0.007f, 0.003f},
        {0.021f, 0.011f, 0.01f, 0.011f, 0.01f, 0.009f, 0.008f, 0.01f, 0.006f, 0.005f, 0.0f, 0.001f},
        {0.023f, 0.01f, 0.011f, 0.011f, 0.011f, 0.01f, 0.008f, 0.01f, 0.006f, 0.014f, 0.02f, 0.0f},
    };

    // Yaw moment per unit aileron (deflection/20 deg), rows beta -30..30 deg step 10, columns alpha
    constexpr float kDnda[7][12] = {
        {0.001f, -0.027f, -0.017f, -0.013f, -0.012f, -0.016f, 0.001f, 0.017f, 0.011f, 0.017f, 0.008f, 0.016f},
        {0.002f, -0.014f, -0.016f, -0.016f, -0.014f, -0.019f, -0.021f, 0.002f, 0.012f, 0.016f, 0.015f, 0.011f},
        {-0.006f, -0.008f, -0.006f, -0.006f, -0.005f, -0.008f, -0.005f, 0.007f, 0.004f, 0.007f, 0.006f, 0.006f},
        {-0.011f, -0.011f, -0.01f, -0.009f, -0.008f, -0.006f, 0.0f, 0.004f, 0.007f, 0.01f, 0.004f, 0.01f},
        {-0.015f, -0.015f, -0.014f, -0.012f, -0.011f, -0.008f, -0.002f, 0.002f, 0.006f, 0.012f, 0.011f, 0.011f},
        {-0.024f, -0.01f, -0.004f, -0.002f, -0.001f, 0.003f, 0.014f, 0.006f, -0.001f, 0.004f, 0.004f, 0.006f},
        {-0.022f, 0.002f, -0.003f, -0.005f, -0.003f, -0.001f, -0.009f, -0.009f, -0.001f, 0.003f, -0.002f, 0.001f},
    };

    // Yaw moment per unit rudder (deflection/30 deg), rows beta -30..30 deg step 10, columns alpha
    constexpr float kDndr[7][12] = {
        {-0.018f, -0.052f, -0.052f, -0.052f, -0.054f, -0.049f, -0.059f, -0.051f, -0.03f, -0.037f, -0.026f, -0.013f},
        {-0.028f, -0.051f, -0.043f, -0.046f, -0.045f, -0.049f, -0.057f, -0.052f, -0.03f, -0.033f, -0.03f, -0.008f},
        {-0.037f, -0.041f, -0.038f, -0.04f, -0.04f, -0.038f, -0.037f, -0.03f, -0.027f, -0.024f, -0.019f, -0.013f},
        {-0.048f, -0.045f, -0.045f, -0.045f, -0.044f, -0.045f, -0.047f, -0.048f, -0.049f, -0.045f, -0.033f, -0.016f},
        {-0.043f, -0.044f, -0.041f, -0.041f, -0.04f, -0.038f, -0.034f, -0.035f, -0.035f, -0.029f, -0.022f, -0.009f},
        {-0.052f, -0.034f, -0.036f, -0.036f, -0.035f, -0.028f, -0.024f, -0.023f, -0.02f, -0.016f, -0.01f, -0.014f},
        {-0.062f, -0.034f, -0.027f, -0.028f, -0.027f, -0.027f, -0.023f, -0.023f, -0.019f, -0.009f, -0.025f, -0.01f},
    };

    // Rate-damping derivatives CXq CYr CYp CZq Clr Clp Cmq Cnr Cnp, columns alpha
    constexpr float kDamp[9][12] = {
        {-0.267f, -0.11f, 0.308f, 1.34f, 2.08f, 2.91f, 2.76f, 2.05f, 1.5f, 1.49f, 1.83f, 1.21f},
        {0.882f, 0.852f, 0.876f, 0.958f, 0.962f, 0.974f, 0.819f, 0.483f, 0.59f, 1.21f, -0.493f, -1.04f},
        {-0.108f, -0.108f, -0.188f, 0.11f, 0.258f, 0.226f, 0.344f, 0.362f, 0.611f, 0.529f, 0.298f, -2.27f},
        {-8.8f, -25.8f, -28.9f, -31.4f, -31.2f, -30.7f, -27.7f, -28.2f, -29.0f, -29.8f, -38.3f, -35.3f},
        {-0.126f, -0.026f, 0.063f, 0.113f, 0.208f, 0.23f, 0.319f, 0.437f, 0.68f, 0.1f, 0.447f, -0.33f},
        {-0.36f, -0.359f, -0.443f, -0.42f, -0.383f, -0.375f, -0.329f, -0.294f, -0.23f, -0.21f, -0.12f, -0.1f},
        {-7.21f, -0.54f, -5.23f, -5.26f, -6.11f, -6.64f, -5.69f, -6.0f, -6.2f, -6.4f, -6.6f, -6.0f},
        {-0.38f, -0.363f, -0.378f, -0.386f, -0.37f, -0.453f, -0.55f, -0.582f, -0.595f, -0.637f, -1.02f, -0.84f},
        {0.061f, 0.052f, 0.052f, -0.012f, -0.013f, -0.024f, 0.05f, 0.15f, 0.13f, 0.158f, 0.24f, 0.15f},
    };

    // Idle thrust, lbf. Rows Mach 0..1.0 step 0.2, columns altitude 0..50,000 ft step 10,000
    constexpr float kThrustIdle[6][6] = {
        {1060.0f, 670.0f, 880.0f, 1140.0f, 1500.0f, 1860.0f},
        {635.0f, 425.0f, 690.0f, 1010.0f, 1330.0f, 1700.0f},
        {60.0f, 25.0f, 345.0f, 755.0f, 1130.0f, 1525.0f},
        {-1020.0f, -170.0f, -300.0f, 350.0f, 910.0f, 1360.0f},
        {-2700.0f, -1900.0f, -1300.0f, -247.0f, 600.0f, 1100.0f},
        {-3600.0f, -1400.0f, -595.0f, -342.0f, -200.0f, 700.0f},
    };

    // Military thrust, lbf. Rows Mach 0..1.0 step 0.2, columns altitude 0..50,000 ft step 10,000
    constexpr float kThrustMil[6][6] = {
        {12680.0f, 9150.0f, 6200.0f, 3950.0f, 2450.0f, 1400.0f},
        {12680.0f, 9150.0f, 6313.0f, 4040.0f, 2470.0f, 1400.0f},
        {12610.0f, 9312.0f, 6610.0f, 4290.0f, 2600.0f, 1560.0f},
        {12640.0f, 9839.0f, 7090.0f, 4660.0f, 2840.0f, 1660.0f},
        {12390.0f, 10176.0f, 7750.0f, 5320.0f, 3250.0f, 1930.0f},
        {11680.0f, 9848.0f, 8050.0f, 6100.0f, 3800.0f, 2310.0f},
    };

    // Maximum afterburner thrust, lbf. Rows Mach 0..1.0 step 0.2, columns altitude 0..50,000 ft step 10,000
    constexpr float kThrustMax[6][6] = {
        {20000.0f, 15000.0f, 10800.0f, 7000.0f, 4000.0f, 2500.0f},
        {21420.0f, 15700.0f, 11225.0f, 7323.0f, 4435.0f, 2600.0f},
        {22700.0f, 16860.0f, 12250.0f, 8154.0f, 5000.0f, 2835.0f},
        {24240.0f, 18910.0f, 13760.0f, 9285.0f, 5700.0f, 3215.0f},
        {26070.0f, 21075.0f, 15975.0f, 11115.0f, 6860.0f, 3950.0f},
        {28886.0f, 23319.0f, 18300.0f, 13484.0f, 8642.0f, 5057.0f},
    };

    // Fractional table position for a breakpoint grid. The segment index is
    // clamped so an input just past the last breakpoint extrapolates along
    // the end segment, as the source's lookup routine does.
    struct Position
    {
        int index;
        float fraction;
    };

    Position locate(float value, float start, float step, int count)
    {
        const float f = (value - start) / step;
        const int index = std::clamp(static_cast<int>(std::floor(f)), 0, count - 2);
        return {index, f - static_cast<float>(index)};
    }

    template <int Cols>
    float lerpRow(const float (&row)[Cols], Position p)
    {
        return row[p.index] + (row[p.index + 1] - row[p.index]) * p.fraction;
    }

    template <int Rows, int Cols>
    float lerpGrid(const float (&table)[Rows][Cols], Position row, Position col)
    {
        const float a = lerpRow(table[row.index], col);
        const float b = lerpRow(table[row.index + 1], col);
        return a + (b - a) * row.fraction;
    }

    Position alphaAt(float alphaDeg)
    {
        return locate(std::clamp(alphaDeg, kAlphaMinDeg, kAlphaMaxDeg), -10.0f, 5.0f, 12);
    }

    Position elevatorAt(float elevatorDeg)
    {
        return locate(std::clamp(elevatorDeg, -kElevatorLimitDeg, kElevatorLimitDeg), -24.0f, 12.0f, 5);
    }

    Position signedBetaAt(float betaDeg)
    {
        return locate(std::clamp(betaDeg, -kBetaLimitDeg, kBetaLimitDeg), -30.0f, 10.0f, 7);
    }

    float oddInBeta(const float (&table)[7][12], float alphaDeg, float betaDeg)
    {
        const float magnitude = std::min(std::abs(betaDeg), kBetaLimitDeg);
        const float value = lerpGrid(table, locate(magnitude, 0.0f, 5.0f, 7), alphaAt(alphaDeg));
        return betaDeg < 0.0f ? -value : value;
    }

    float rtau(float powerError)
    {
        if (powerError <= 25.0f)
        {
            return 1.0f;
        }
        if (powerError >= 50.0f)
        {
            return 0.1f;
        }
        return 1.9f - 0.036f * powerError;
    }
}

float cx(float alphaDeg, float elevatorDeg)
{
    return lerpGrid(kCx, elevatorAt(elevatorDeg), alphaAt(alphaDeg));
}

float cy(float betaDeg, float aileronDeg, float rudderDeg)
{
    const float beta = std::clamp(betaDeg, -kBetaLimitDeg, kBetaLimitDeg);
    return -0.02f * beta + 0.021f * (aileronDeg / 20.0f) + 0.086f * (rudderDeg / 30.0f);
}

float cz(float alphaDeg, float betaDeg, float elevatorDeg)
{
    const float beta = std::clamp(betaDeg, -kBetaLimitDeg, kBetaLimitDeg) / 57.3f;
    const float base = lerpRow(kCz[0], alphaAt(alphaDeg));
    return base * (1.0f - beta * beta) - 0.19f * (elevatorDeg / 25.0f);
}

float cm(float alphaDeg, float elevatorDeg)
{
    return lerpGrid(kCm, elevatorAt(elevatorDeg), alphaAt(alphaDeg));
}

float cl(float alphaDeg, float betaDeg)
{
    return oddInBeta(kCl, alphaDeg, betaDeg);
}

float cn(float alphaDeg, float betaDeg)
{
    return oddInBeta(kCn, alphaDeg, betaDeg);
}

float dlda(float alphaDeg, float betaDeg)
{
    return lerpGrid(kDlda, signedBetaAt(betaDeg), alphaAt(alphaDeg));
}

float dldr(float alphaDeg, float betaDeg)
{
    return lerpGrid(kDldr, signedBetaAt(betaDeg), alphaAt(alphaDeg));
}

float dnda(float alphaDeg, float betaDeg)
{
    return lerpGrid(kDnda, signedBetaAt(betaDeg), alphaAt(alphaDeg));
}

float dndr(float alphaDeg, float betaDeg)
{
    return lerpGrid(kDndr, signedBetaAt(betaDeg), alphaAt(alphaDeg));
}

Damping damping(float alphaDeg)
{
    const Position p = alphaAt(alphaDeg);
    return {lerpRow(kDamp[0], p), lerpRow(kDamp[1], p), lerpRow(kDamp[2], p),
            lerpRow(kDamp[3], p), lerpRow(kDamp[4], p), lerpRow(kDamp[5], p),
            lerpRow(kDamp[6], p), lerpRow(kDamp[7], p), lerpRow(kDamp[8], p)};
}

float throttleGearing(float throttle)
{
    const float lever = std::clamp(throttle, 0.0f, 1.0f);
    return lever <= 0.77f ? 64.94f * lever : 217.38f * lever - 117.38f;
}

float powerRate(float power, float commandedPower)
{
    // Military and afterburner are separate regimes: crossing between them
    // goes through an intermediate target at the fast (5 /s) rate.
    float target = commandedPower;
    float rate = 0.0f;
    if (commandedPower >= 50.0f)
    {
        if (power >= 50.0f)
        {
            rate = 5.0f;
        }
        else
        {
            target = 60.0f;
            rate = rtau(target - power);
        }
    }
    else if (power >= 50.0f)
    {
        target = 40.0f;
        rate = 5.0f;
    }
    else
    {
        rate = rtau(target - power);
    }
    return rate * (target - power);
}

float thrustLbf(float power, float altitudeFt, float mach, float milScale, float maxScale)
{
    // The deck is tabulated to 50,000 ft and Mach 1.0; beyond that the end
    // segments are extrapolated (as the source does), capped here at
    // 70,000 ft and Mach 2.0 so the extrapolation stays bounded.
    const Position mRow = locate(std::clamp(mach, 0.0f, 2.0f), 0.0f, 0.2f, 6);
    const Position hCol = locate(std::clamp(altitudeFt, 0.0f, 70000.0f), 0.0f, 10000.0f, 6);
    const float mil = lerpGrid(kThrustMil, mRow, hCol) * milScale;
    float thrust = 0.0f;
    if (power < 50.0f)
    {
        const float idle = lerpGrid(kThrustIdle, mRow, hCol);
        thrust = idle + (mil - idle) * power * 0.02f;
    }
    else
    {
        const float max = lerpGrid(kThrustMax, mRow, hCol) * maxScale;
        thrust = mil + (max - mil) * (power - 50.0f) * 0.02f;
    }
    return thrust;
}
}
