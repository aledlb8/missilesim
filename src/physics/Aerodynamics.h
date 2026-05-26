#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

namespace missilesim::physics
{
    // A physically-specified aerodynamic profile for a body. Every field is a
    // real, measurable quantity rather than an output-tuning fudge factor:
    //
    //   referenceArea       Aerodynamic reference area S (m^2).
    //   baseDragCoefficient Zero-lift drag coefficient Cd0 in the incompressible
    //                       (low-Mach) regime. This is the configured "how draggy
    //                       is the body" anchor.
    //   aspectRatio         Lifting-surface aspect ratio AR. 0 disables induced
    //                       drag (e.g. an unfinned coasting body). [used in A2]
    //   oswaldEfficiency    Span efficiency factor e (~0.7-0.9). [used in A2]
    //   machDragMultiplier  Dimensionless compressibility curve Cd0(M)/Cd0(0),
    //                       given as (mach, multiplier) samples sorted by mach.
    //                       This is the standard wind-tunnel/CFD artifact that
    //                       captures the transonic drag-divergence rise. A value
    //                       of 1.0 means "no compressibility effect at this Mach".
    struct AeroProfile
    {
        float referenceArea = 0.1f;
        float baseDragCoefficient = 0.1f;
        float aspectRatio = 0.0f;
        float oswaldEfficiency = 0.85f;
        // Maximum usable lift coefficient (aerodynamic stall/control ceiling).
        // Bounds the lateral acceleration the airframe can generate at a given
        // dynamic pressure: a_max_aero = q * Cl_max * S / m.
        float maxLiftCoefficient = 1.5f;
        std::vector<glm::vec2> machDragMultiplier;
    };

    // Linear interpolation of the compressibility multiplier at a given Mach
    // number. Clamps to the curve endpoints outside the sampled range. Returns
    // 1.0 (no compressibility correction) when no curve is supplied.
    inline float dragRiseMultiplier(const AeroProfile &profile, float mach)
    {
        const std::vector<glm::vec2> &curve = profile.machDragMultiplier;
        if (curve.empty())
        {
            return 1.0f;
        }
        if (mach <= curve.front().x)
        {
            return curve.front().y;
        }
        if (mach >= curve.back().x)
        {
            return curve.back().y;
        }

        for (std::size_t i = 1; i < curve.size(); ++i)
        {
            if (mach <= curve[i].x)
            {
                const glm::vec2 lower = curve[i - 1];
                const glm::vec2 upper = curve[i];
                const float span = std::max(upper.x - lower.x, 1e-6f);
                const float t = (mach - lower.x) / span;
                return lower.y + (t * (upper.y - lower.y));
            }
        }
        return curve.back().y;
    }

    // Zero-lift drag coefficient Cd0 at a given Mach number.
    inline float zeroLiftDragCoefficient(const AeroProfile &profile, float mach)
    {
        return profile.baseDragCoefficient * dragRiseMultiplier(profile, mach);
    }

    // Induced (lift-dependent) drag coefficient: Cd_i = Cl^2 / (pi * AR * e).
    // Returns 0 when no lifting surface is defined (aspectRatio <= 0).
    inline float inducedDragCoefficient(const AeroProfile &profile, float liftCoefficient)
    {
        if (profile.aspectRatio <= 0.0f)
        {
            return 0.0f;
        }
        constexpr float kPi = 3.14159265358979323846f;
        const float denominator = kPi * profile.aspectRatio * std::max(profile.oswaldEfficiency, 1e-3f);
        return (liftCoefficient * liftCoefficient) / denominator;
    }

    // A physically reasonable transonic drag-rise curve for a slender,
    // fin-stabilised supersonic missile body. The multiplier peaks just past
    // Mach 1 (drag divergence) and relaxes toward a supersonic plateau. Bodies
    // that want a different signature supply their own curve via config (A3).
    inline std::vector<glm::vec2> defaultSupersonicDragRiseCurve()
    {
        return {
            {0.00f, 1.00f},
            {0.80f, 1.10f},
            {0.95f, 1.90f},
            {1.05f, 3.80f},
            {1.20f, 3.50f},
            {2.00f, 2.60f},
            {3.00f, 2.00f},
            {5.00f, 1.80f},
        };
    }
}
