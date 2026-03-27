#pragma once

#include <cctype>
#include <sstream>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

namespace missilesim::application::detail
{
    inline std::string trimWhitespace(const std::string &value)
    {
        size_t start = 0;
        while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])))
        {
            ++start;
        }

        size_t end = value.size();
        while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])))
        {
            --end;
        }

        return value.substr(start, end - start);
    }

    inline bool parseBoolValue(const std::string &value, bool fallback)
    {
        const std::string normalized = trimWhitespace(value);
        if (normalized == "true" || normalized == "1")
        {
            return true;
        }
        if (normalized == "false" || normalized == "0")
        {
            return false;
        }
        return fallback;
    }

    inline float parseFloatValue(const std::string &value, float fallback)
    {
        try
        {
            return std::stof(trimWhitespace(value));
        }
        catch (...)
        {
            return fallback;
        }
    }

    inline int parseIntValue(const std::string &value, int fallback)
    {
        try
        {
            return std::stoi(trimWhitespace(value));
        }
        catch (...)
        {
            return fallback;
        }
    }

    inline glm::vec3 parseVec3Value(const std::string &value, const glm::vec3 &fallback)
    {
        std::stringstream stream(value);
        std::string component;
        glm::vec3 parsed = fallback;

        if (std::getline(stream, component, ','))
        {
            parsed.x = parseFloatValue(component, fallback.x);
        }
        if (std::getline(stream, component, ','))
        {
            parsed.y = parseFloatValue(component, fallback.y);
        }
        if (std::getline(stream, component, ','))
        {
            parsed.z = parseFloatValue(component, fallback.z);
        }

        return parsed;
    }

    inline std::string formatBoolValue(bool value)
    {
        return value ? "true" : "false";
    }

    inline std::string formatVec3Value(const glm::vec3 &value)
    {
        std::ostringstream stream;
        stream << value.x << "," << value.y << "," << value.z;
        return stream.str();
    }

    inline glm::vec3 safeNormalize(const glm::vec3 &value, const glm::vec3 &fallback)
    {
        if (glm::length2(value) > 0.0001f)
        {
            return glm::normalize(value);
        }
        return fallback;
    }
}