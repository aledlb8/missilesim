#pragma once

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    // Closest approach of two bodies that both move in straight lines across
    // one step. Testing a missile's swept segment against a target's *end*
    // position misses a crossing pair whose paths intersect mid-step while
    // neither end position is close (at 1,500 m/s of closure one 0.01 s step
    // is 15 m). The relative position is r(s) = r0 + s (r1 - r0) for s in
    // [0, 1]; its minimum length is found in closed form.
    struct SweepResult
    {
        float fraction = 0.0f;    // s of closest approach, 0 at step start and 1 at step end
        float distance = 0.0f;    // |r(s)| (m)
        glm::vec3 offset{0.0f};   // r(s) = a(s) - b(s)
    };

    inline SweepResult sweepClosestApproach(const glm::vec3 &aStart, const glm::vec3 &aEnd,
                                            const glm::vec3 &bStart, const glm::vec3 &bEnd)
    {
        const glm::vec3 r0 = aStart - bStart;
        const glm::vec3 dr = (aEnd - bEnd) - r0;
        const float drSquared = glm::dot(dr, dr);

        SweepResult result;
        result.fraction = drSquared > 1.0e-12f ? std::clamp(-glm::dot(r0, dr) / drSquared, 0.0f, 1.0f) : 0.0f;
        result.offset = r0 + dr * result.fraction;
        result.distance = glm::length(result.offset);
        return result;
    }

    // First step fraction at which the separation of the same two straight-line
    // motions shrinks to `radius`, or -1 if it stays outside all step. A pair
    // already inside at the start of the step returns 0. A fuze fires when a
    // body enters its volume, not at the closest approach, so this (and not
    // SweepResult::fraction) places a detonation.
    inline float sweepEntryFraction(const glm::vec3 &aStart, const glm::vec3 &aEnd,
                                    const glm::vec3 &bStart, const glm::vec3 &bEnd, float radius)
    {
        if (!(radius > 0.0f))
        {
            return -1.0f;
        }
        const glm::vec3 r0 = aStart - bStart;
        const glm::vec3 dr = (aEnd - bEnd) - r0;
        const float c = glm::dot(r0, r0) - radius * radius;
        if (c <= 0.0f)
        {
            return 0.0f;
        }
        const float a = glm::dot(dr, dr);
        if (a <= 1.0e-12f)
        {
            return -1.0f;
        }
        const float b = 2.0f * glm::dot(r0, dr);
        const float discriminant = b * b - 4.0f * a * c;
        if (b >= 0.0f || discriminant < 0.0f)
        {
            return -1.0f; // separating, or the paths never come that close
        }
        const float entry = (-b - std::sqrt(discriminant)) / (2.0f * a);
        return (entry >= 0.0f && entry <= 1.0f) ? entry : -1.0f;
    }

    // Point on a straight segment at a step fraction.
    inline glm::vec3 lerpPosition(const glm::vec3 &start, const glm::vec3 &end, float fraction)
    {
        return start + (end - start) * fraction;
    }
}
