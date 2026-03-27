#include "Renderer.h"
#include "SceneEffects.h"

#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#ifndef MISSILESIM_SOURCE_ASSET_DIR
#define MISSILESIM_SOURCE_ASSET_DIR ""
#endif

namespace
{
    struct ObjVertexRef
    {
        int position = 0;
        int normal = 0;
        bool hasNormal = false;
    };

    int resolveObjIndex(int rawIndex, std::size_t count)
    {
        if (rawIndex > 0)
        {
            return rawIndex - 1;
        }
        if (rawIndex < 0)
        {
            return static_cast<int>(count) + rawIndex;
        }
        return -1;
    }

    bool parseObjVertexRef(const std::string &token, ObjVertexRef &result)
    {
        std::stringstream tokenStream(token);
        std::string positionToken;
        std::string texcoordToken;
        std::string normalToken;

        if (!std::getline(tokenStream, positionToken, '/') || positionToken.empty())
        {
            return false;
        }

        std::getline(tokenStream, texcoordToken, '/');
        std::getline(tokenStream, normalToken, '/');

        try
        {
            result.position = std::stoi(positionToken);
            if (!normalToken.empty())
            {
                result.normal = std::stoi(normalToken);
                result.hasNormal = true;
            }
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    glm::vec3 normalizeOrFallback(const glm::vec3 &vector, const glm::vec3 &fallback)
    {
        if (glm::length2(vector) > 0.000001f)
        {
            return glm::normalize(vector);
        }

        if (glm::length2(fallback) > 0.000001f)
        {
            return glm::normalize(fallback);
        }

        return glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 rotateAroundAxis(const glm::vec3 &vector, const glm::vec3 &axis, float angleRadians)
    {
        const glm::vec3 normalizedAxis = normalizeOrFallback(axis, glm::vec3(0.0f, 1.0f, 0.0f));
        const float cosine = std::cos(angleRadians);
        const float sine = std::sin(angleRadians);
        return (vector * cosine) +
               (glm::cross(normalizedAxis, vector) * sine) +
               (normalizedAxis * glm::dot(normalizedAxis, vector) * (1.0f - cosine));
    }
}

namespace
{
    float computeFogDensity(float sceneFarPlane)
    {
        return 1.0f / std::max(sceneFarPlane * 0.45f, 6000.0f);
    }

    glm::mat4 buildTargetOrientationMatrix(const glm::vec3 &velocity, const glm::vec3 &acceleration)
    {
        const glm::vec3 forward = glm::normalize(velocity);
        const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        glm::vec3 desiredUp = worldUp;
        glm::vec3 right = glm::cross(worldUp, forward);
        if (glm::length2(right) > 0.0001f)
        {
            right = glm::normalize(right);
            const float lateralAcceleration = glm::dot(acceleration, right);
            const float bankAngle = glm::clamp(std::atan2(lateralAcceleration, 9.81f),
                                               -glm::radians(68.0f),
                                               glm::radians(68.0f));
            desiredUp = rotateAroundAxis(worldUp, forward, -bankAngle);
        }

        if (std::abs(glm::dot(forward, desiredUp)) > 0.98f)
        {
            desiredUp = glm::vec3(0.0f, 0.0f, 1.0f);
        }

        desiredUp = glm::normalize(desiredUp - (forward * glm::dot(desiredUp, forward)));
        right = glm::cross(desiredUp, forward);
        if (glm::length2(right) <= 0.0001f)
        {
            right = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        else
        {
            right = glm::normalize(right);
        }

        glm::mat4 rotation(1.0f);
        rotation[0] = glm::vec4(right, 0.0f);
        rotation[1] = glm::vec4(forward, 0.0f);
        rotation[2] = glm::vec4(-desiredUp, 0.0f);
        return rotation;
    }
}

void Renderer::renderAll(const std::vector<PhysicsObject *> &objects)
{
    renderEnvironment();
    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    // Render all objects
    for (auto *object : objects)
    {
        if (!object)
            continue;

        // Create model matrix for the object
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, object->getPosition());

        // If the object is a missile, orient it along its velocity vector
        if (object->getType() == "Missile")
        {
            // If the object is moving, orient it along the velocity vector
            glm::vec3 velocity = object->getVelocity();
            if (glm::length(velocity) > 0.001f)
            {
                // Rotate code remains the same
                // Use velocity direction for facing the object
                glm::vec3 direction = glm::normalize(velocity);

                // Calculate the rotation axis and angle from default orientation (along y-axis) to velocity direction
                glm::vec3 defaultDir = glm::vec3(0.0f, 1.0f, 0.0f);

                // Calculate the angle between default direction and velocity
                float angle = acos(glm::dot(defaultDir, direction));

                // If angle is not 0 or 180 degrees, we need to rotate
                if (abs(angle) > 0.001f && abs(angle - glm::pi<float>()) > 0.001f)
                {
                    // Calculate rotation axis (perpendicular to both vectors)
                    glm::vec3 rotationAxis = glm::normalize(glm::cross(defaultDir, direction));

                    // Apply rotation
                    model = glm::rotate(model, angle, rotationAxis);
                }
                // Handle special case: if velocity is in opposite direction of default
                else if (abs(angle - glm::pi<float>()) <= 0.001f)
                {
                    // Just rotate 180 degrees around X axis
                    model = glm::rotate(model, glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
                }
            }
        }
        else if (object->getType() == "Target")
        {
            glm::vec3 velocity = object->getVelocity();
            if (glm::length(velocity) > 0.001f)
            {
                model *= buildTargetOrientationMatrix(velocity, object->getAcceleration());
            }
        }

        // Scale based on the type of object
        if (object->getType() == "Missile")
        {
            // Scale to make the missile shape visible
            model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));
            glBindVertexArray(m_vao);

            glUseProgram(m_shaderProgram);
            glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
            if (m_cameraPosLoc != -1)
            {
                glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
            }
            if (m_fogDensityLoc != -1)
            {
                glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));
            }

            // Draw object
            glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
        else if (object->getType() == "Target")
        {
            const Target *targetObject = static_cast<Target *>(object);
            const float targetScale = std::max(targetObject->getRadius(), 1.0f);
            model = glm::scale(model, glm::vec3(targetScale));
            glBindVertexArray(m_targetVAO);

            glUseProgram(m_shaderProgram);
            glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
            if (m_cameraPosLoc != -1)
            {
                glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
            }
            if (m_fogDensityLoc != -1)
            {
                glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));
            }

            // Draw object
            glDrawElements(GL_TRIANGLES, m_targetIndices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }
}

void Renderer::render(PhysicsObject *object)
{
    if (!object)
        return;

    // Create model matrix for the object
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, object->getPosition());

    // If the object is moving, orient it along the velocity vector
    glm::vec3 velocity = object->getVelocity();
    if (object->getType() == "Target" && glm::length(velocity) > 0.001f)
    {
        model *= buildTargetOrientationMatrix(velocity, object->getAcceleration());
    }
    else if (glm::length(velocity) > 0.001f)
    {
        // Use velocity direction for facing the object
        glm::vec3 direction = glm::normalize(velocity);

        // Calculate the rotation axis and angle from default orientation (along y-axis) to velocity direction
        glm::vec3 defaultDir = glm::vec3(0.0f, 1.0f, 0.0f);

        // Calculate the angle between default direction and velocity
        float angle = acos(glm::dot(defaultDir, direction));

        // If angle is not 0 or 180 degrees, we need to rotate
        if (abs(angle) > 0.001f && abs(angle - glm::pi<float>()) > 0.001f)
        {
            // Calculate rotation axis (perpendicular to both vectors)
            glm::vec3 rotationAxis = glm::normalize(glm::cross(defaultDir, direction));

            // Apply rotation
            model = glm::rotate(model, angle, rotationAxis);
        }
        // Handle special case: if velocity is in opposite direction of default
        else if (abs(angle - glm::pi<float>()) <= 0.001f)
        {
            // Just rotate 180 degrees around X axis
            model = glm::rotate(model, glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
        }
    }

    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    // Use shader program
    glUseProgram(m_shaderProgram);

    // Pass transformation matrices to the shader
    glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    if (m_cameraPosLoc != -1)
    {
        glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }
    if (m_fogDensityLoc != -1)
    {
        glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));
    }

    // Draw object based on its type
    if (object->getType() == "Missile")
    {
        // Scale to make the missile shape visible
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));

        // Pass updated model matrix with scale
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        // Draw missile
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    else if (object->getType() == "Target")
    {
        const Target *targetObject = static_cast<Target *>(object);
        const float targetScale = std::max(targetObject->getRadius(), 1.0f);
        model = glm::scale(model, glm::vec3(targetScale));

        // Pass updated model matrix with scale
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        // Draw target
        glBindVertexArray(m_targetVAO);
        glDrawElements(GL_TRIANGLES, m_targetIndices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    else
    {
        // Default rendering for other objects
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Renderer::renderFloor()
{
    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();
    glm::mat4 model = glm::mat4(1.0f);

    glUseProgram(m_shaderProgram);

    glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    if (m_cameraPosLoc != -1)
    {
        glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
    }
    if (m_fogDensityLoc != -1)
    {
        glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));
    }

    glBindVertexArray(m_floorVAO);
    glDrawElements(GL_TRIANGLES, m_floorIndices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Renderer::renderEnvironment()
{
    renderFloor();
    renderWorldGuides();
}

void Renderer::renderWorldGuides()
{
    const float guideY = 0.08f;
    const float runwayHalfWidth = glm::clamp(m_airspaceHalfExtent * 0.035f, 14.0f, 72.0f);
    const float runwayHalfLength = glm::clamp(m_airspaceHalfExtent * 0.55f, 230.0f, m_groundHalfExtent * 0.45f);
    const float airspaceHalfExtent = m_airspaceHalfExtent;
    const float beaconHeight = m_airspaceHeight;
    const float ringStep = glm::clamp(m_airspaceHalfExtent * 0.25f, 160.0f, 2500.0f);
    const int ringCount = std::max(3, std::min(5, static_cast<int>(airspaceHalfExtent / ringStep)));

    const glm::vec3 runwayColor(0.92f, 0.94f, 0.96f);
    const glm::vec3 runwayAccent(0.72f, 0.82f, 0.90f);
    const glm::vec3 axisXColor(0.82f, 0.40f, 0.38f);
    const glm::vec3 axisZColor(0.36f, 0.70f, 0.82f);
    const glm::vec3 guideColor(0.50f, 0.60f, 0.68f);
    const glm::vec3 beaconColor(0.72f, 0.78f, 0.86f);

    renderLine(glm::vec3(-airspaceHalfExtent, guideY, 0.0f), glm::vec3(airspaceHalfExtent, guideY, 0.0f), axisXColor);
    renderLine(glm::vec3(0.0f, guideY, -airspaceHalfExtent), glm::vec3(0.0f, guideY, airspaceHalfExtent), axisZColor);

    for (int ringIndex = 1; ringIndex <= ringCount; ++ringIndex)
    {
        const float radius = std::min(ringStep * ringIndex, airspaceHalfExtent * 0.92f);
        const float ringTint = static_cast<float>(ringIndex) / static_cast<float>(ringCount + 1);
        const glm::vec3 ringColor = glm::mix(glm::vec3(0.40f, 0.48f, 0.54f), glm::vec3(0.56f, 0.64f, 0.72f), ringTint);
        renderGroundCircle(radius, ringColor, 48);
    }

    renderLine(glm::vec3(-runwayHalfWidth, guideY, -runwayHalfLength), glm::vec3(runwayHalfWidth, guideY, -runwayHalfLength), runwayColor);
    renderLine(glm::vec3(runwayHalfWidth, guideY, -runwayHalfLength), glm::vec3(runwayHalfWidth, guideY, runwayHalfLength), runwayColor);
    renderLine(glm::vec3(runwayHalfWidth, guideY, runwayHalfLength), glm::vec3(-runwayHalfWidth, guideY, runwayHalfLength), runwayColor);
    renderLine(glm::vec3(-runwayHalfWidth, guideY, runwayHalfLength), glm::vec3(-runwayHalfWidth, guideY, -runwayHalfLength), runwayColor);

    const float dashLength = glm::clamp(runwayHalfLength * 0.08f, 18.0f, 140.0f);
    const float dashStep = dashLength * 1.9f;
    for (float z = -runwayHalfLength + dashLength; z < runwayHalfLength - dashLength; z += dashStep)
    {
        renderLine(glm::vec3(0.0f, guideY, z), glm::vec3(0.0f, guideY, z + dashLength), runwayAccent);
    }

    const float padHalf = glm::clamp(runwayHalfWidth * 1.3f, 18.0f, 54.0f);
    renderLine(glm::vec3(-padHalf, guideY, -padHalf), glm::vec3(padHalf, guideY, -padHalf), guideColor);
    renderLine(glm::vec3(padHalf, guideY, -padHalf), glm::vec3(padHalf, guideY, padHalf), guideColor);
    renderLine(glm::vec3(padHalf, guideY, padHalf), glm::vec3(-padHalf, guideY, padHalf), guideColor);
    renderLine(glm::vec3(-padHalf, guideY, padHalf), glm::vec3(-padHalf, guideY, -padHalf), guideColor);
    renderPoint(glm::vec3(0.0f, 0.18f, 0.0f), glm::vec3(0.98f, 0.82f, 0.30f), 6.0f);

    const glm::vec3 corners[4] = {
        glm::vec3(-airspaceHalfExtent, 0.0f, -airspaceHalfExtent),
        glm::vec3(airspaceHalfExtent, 0.0f, -airspaceHalfExtent),
        glm::vec3(airspaceHalfExtent, 0.0f, airspaceHalfExtent),
        glm::vec3(-airspaceHalfExtent, 0.0f, airspaceHalfExtent)};

    for (int i = 0; i < 4; ++i)
    {
        const glm::vec3 currentGround = corners[i] + glm::vec3(0.0f, guideY, 0.0f);
        const glm::vec3 nextGround = corners[(i + 1) % 4] + glm::vec3(0.0f, guideY, 0.0f);
        const glm::vec3 currentTop = corners[i] + glm::vec3(0.0f, beaconHeight, 0.0f);
        const glm::vec3 nextTop = corners[(i + 1) % 4] + glm::vec3(0.0f, beaconHeight, 0.0f);

        renderLine(currentGround, nextGround, guideColor);
        renderLine(currentTop, nextTop, glm::vec3(0.56f, 0.64f, 0.72f));
        renderAirspaceBeacon(corners[i], beaconHeight, beaconColor);
    }

    renderLine(glm::vec3(-airspaceHalfExtent, beaconHeight, -airspaceHalfExtent),
               glm::vec3(airspaceHalfExtent, beaconHeight, airspaceHalfExtent),
               glm::vec3(0.42f, 0.50f, 0.58f));
    renderLine(glm::vec3(airspaceHalfExtent, beaconHeight, -airspaceHalfExtent),
               glm::vec3(-airspaceHalfExtent, beaconHeight, airspaceHalfExtent),
               glm::vec3(0.42f, 0.50f, 0.58f));
}

void Renderer::renderGroundCircle(float radius, const glm::vec3 &color, int segments)
{
    const float guideY = 0.08f;
    const float angleStep = glm::two_pi<float>() / static_cast<float>(segments);

    for (int i = 0; i < segments; ++i)
    {
        float startAngle = angleStep * static_cast<float>(i);
        float endAngle = angleStep * static_cast<float>(i + 1);
        glm::vec3 start(radius * cos(startAngle), guideY, radius * sin(startAngle));
        glm::vec3 end(radius * cos(endAngle), guideY, radius * sin(endAngle));
        renderLine(start, end, color);
    }
}

void Renderer::renderAirspaceBeacon(const glm::vec3 &basePosition, float height, const glm::vec3 &color)
{
    const glm::vec3 base = basePosition + glm::vec3(0.0f, 0.08f, 0.0f);
    const glm::vec3 top = basePosition + glm::vec3(0.0f, height, 0.0f);
    renderLine(base, top, color);

    const glm::vec3 tickColor = glm::mix(color, glm::vec3(1.0f, 1.0f, 1.0f), 0.15f);
    const float tickStep = glm::clamp(height * 0.25f, 55.0f, 1200.0f);
    const float tickHalfSpan = glm::clamp(m_airspaceHalfExtent * 0.02f, 8.0f, 65.0f);
    for (float altitude = tickStep; altitude < height; altitude += tickStep)
    {
        const glm::vec3 tickCenter = basePosition + glm::vec3(0.0f, altitude, 0.0f);
        renderLine(tickCenter - glm::vec3(tickHalfSpan, 0.0f, 0.0f), tickCenter + glm::vec3(tickHalfSpan, 0.0f, 0.0f), tickColor);
        renderLine(tickCenter - glm::vec3(0.0f, 0.0f, tickHalfSpan), tickCenter + glm::vec3(0.0f, 0.0f, tickHalfSpan), tickColor);
    }
}

void Renderer::renderExplosion(const glm::vec3 &position, float size)
{
    // Safety check - make sure explosion VAO is valid and parameters are valid
    if (m_explosionVAO == 0 || size <= 0.0f)
    {
        return;
    }

    // Safety check for invalid position
    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
    {
        std::cerr << "Invalid explosion position detected" << std::endl;
        return;
    }

    // Safety check for invalid size
    if (std::isnan(size) || std::isinf(size))
    {
        std::cerr << "Invalid explosion size detected" << std::endl;
        return;
    }

    // Save current OpenGL state
    GLint lastBlendSrc = 0, lastBlendDst = 0;
    GLboolean lastBlendEnabled = GL_FALSE;

    try
    {
        glGetIntegerv(GL_BLEND_SRC, &lastBlendSrc);
        glGetIntegerv(GL_BLEND_DST, &lastBlendDst);
        lastBlendEnabled = glIsEnabled(GL_BLEND);
    }
    catch (...)
    {
        std::cerr << "Error saving OpenGL state" << std::endl;
        // Continue anyway, we'll try to restore default state if this fails
    }

    // Enable additive blending for a glowing effect
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // Create model matrix for the explosion
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, glm::vec3(size));

    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    bool success = true;
    try
    {
        // Use shader program
        glUseProgram(m_shaderProgram);

        // Check if shader program is valid
        if (m_shaderProgram == 0)
        {
            std::cerr << "Invalid shader program" << std::endl;
            success = false;
        }
        else
        {
            // Pass transformation matrices to the shader
            if (m_modelLoc != -1)
                glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            if (m_viewLoc != -1)
                glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            if (m_projLoc != -1)
                glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
            if (m_cameraPosLoc != -1)
                glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
            if (m_fogDensityLoc != -1)
                glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));

            // Draw explosion
            glBindVertexArray(m_explosionVAO);
            glDrawElements(GL_TRIANGLES, m_explosionIndices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }
    catch (...)
    {
        // In case of an OpenGL error, restore state and continue
        std::cerr << "Error rendering explosion" << std::endl;
        success = false;
    }

    // Restore previous OpenGL state
    try
    {
        if (lastBlendEnabled)
        {
            glEnable(GL_BLEND);
            glBlendFunc(lastBlendSrc, lastBlendDst);
        }
        else
        {
            glDisable(GL_BLEND);
        }
    }
    catch (...)
    {
        std::cerr << "Error restoring OpenGL state" << std::endl;
        // If we can't restore the state, set to default safe values
        glDisable(GL_BLEND);
    }
}