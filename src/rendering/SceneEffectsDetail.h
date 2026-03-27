#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

namespace missilesim::rendering::detail
{
    inline constexpr std::size_t kMaxParticles = 8192;
    inline constexpr std::size_t kMaxHeatHazeSprites = 1024;

    inline glm::vec3 safeNormalize(const glm::vec3 &value, const glm::vec3 &fallback)
    {
        if (glm::length2(value) > 0.0001f)
        {
            return glm::normalize(value);
        }

        if (glm::length2(fallback) > 0.0001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 1.0f, 0.0f);
    }

    inline glm::vec3 perpendicularTo(const glm::vec3 &direction)
    {
        const glm::vec3 upReference = (std::abs(direction.y) < 0.95f)
                                          ? glm::vec3(0.0f, 1.0f, 0.0f)
                                          : glm::vec3(1.0f, 0.0f, 0.0f);
        return safeNormalize(glm::cross(direction, upReference), glm::vec3(1.0f, 0.0f, 0.0f));
    }

    inline float saturate(float value)
    {
        return glm::clamp(value, 0.0f, 1.0f);
    }
}