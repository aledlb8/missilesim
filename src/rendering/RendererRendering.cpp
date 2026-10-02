#include "Renderer.h"
#include "SceneEffects.h"
#include "pbr/PBRPipeline.h"

#include "../objects/Fighter.h"
#include "../objects/Missile.h"
#include "../objects/PhysicsObject.h"
#include "../objects/Target.h"
#include "../sim/Terrain.h"

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

    glm::mat4 buildAxisOrientationMatrix(const glm::vec3 &direction)
    {
        const glm::vec3 forward = normalizeOrFallback(direction, glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::vec3 defaultDirection(0.0f, 1.0f, 0.0f);
        const float angle = std::acos(glm::clamp(glm::dot(defaultDirection, forward), -1.0f, 1.0f));

        if (std::abs(angle) <= 0.001f)
        {
            return glm::mat4(1.0f);
        }

        if (std::abs(angle - glm::pi<float>()) <= 0.001f)
        {
            return glm::rotate(glm::mat4(1.0f), glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
        }

        const glm::vec3 rotationAxis = normalizeOrFallback(glm::cross(defaultDirection, forward),
                                                           glm::vec3(1.0f, 0.0f, 0.0f));
        return glm::rotate(glm::mat4(1.0f), angle, rotationAxis);
    }

    glm::vec3 getMissileRenderDirection(const Missile *missile)
    {
        if (missile == nullptr)
        {
            return glm::vec3(0.0f, 1.0f, 0.0f);
        }

        if (missile->isFox2())
        {
            return normalizeOrFallback(missile->getBodyForward(), missile->getThrustDirection());
        }

        if (!missile->isThrustEnabled())
        {
            return normalizeOrFallback(missile->getThrustDirection(), missile->getVelocity());
        }

        return normalizeOrFallback(missile->getVelocity(), missile->getThrustDirection());
    }
}

namespace
{
    // Legacy (non-PBR) path only; matches the PBR pipeline's default so the
    // two paths share one fog formula.
    float computeFogDensity(float sceneFarPlane)
    {
        return 1.0f / std::max(sceneFarPlane * 0.6f, 9000.0f);
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

glm::mat4 Renderer::buildObjectModelMatrix(const PhysicsObject &object) const
{
    glm::mat4 model = glm::translate(glm::mat4(1.0f), object.getRenderPosition());
    const glm::vec3 velocity = object.getVelocity();
    if (object.getType() == "Missile")
        model *= buildAxisOrientationMatrix(getMissileRenderDirection(static_cast<const Missile *>(&object)));
    else if (object.getType() == "Target")
    {
        if (glm::length2(velocity) > 0.000001f)
            model *= buildTargetOrientationMatrix(velocity, object.getRenderAcceleration());
        model = glm::scale(model, glm::vec3(std::max(static_cast<const Target &>(object).getRadius(), 1.0f)));
    }
    else if (object.getType() == "Fighter")
    {
        const Fighter &fighter = static_cast<const Fighter &>(object);
        const glm::vec3 forward = normalizeOrFallback(fighter.getRenderNose(), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::vec3 up = normalizeOrFallback(fighter.getRenderUp() - forward * glm::dot(fighter.getRenderUp(), forward),
                                                 glm::vec3(0.0f, 1.0f, 0.0f));
        // The mesh's local x is up x forward (the pilot's left in this
        // right-handed world), which keeps the basis a proper rotation.
        const glm::vec3 right = glm::normalize(glm::cross(up, forward));
        glm::mat4 rotation(1.0f);
        rotation[0] = glm::vec4(right, 0.0f);
        rotation[1] = glm::vec4(forward, 0.0f);
        rotation[2] = glm::vec4(-up, 0.0f);
        model *= rotation;
        model = glm::scale(model, glm::vec3(std::max(fighter.getRadius(), 1.0f)));
    }
    else if (glm::length2(velocity) > 0.000001f)
        model *= buildAxisOrientationMatrix(velocity);
    return model;
}

void Renderer::renderAll(const std::vector<PhysicsObject *> &objects)
{
    renderEnvironment();
    for (auto *object : objects) render(object);
}

void Renderer::render(PhysicsObject *object)
{
    if (!object)
        return;

    const glm::mat4 model = buildObjectModelMatrix(*object);
    const bool isMissile = object->getType() == "Missile";
    submitEnginePlumes(*object);

    // Scale and draw/submit
    if (isMissile)
    {
        if (isPBRActive())
        {
            m_pbrPipeline->submitLegacyMesh(
                m_vao, static_cast<GLsizei>(m_indices.size()), model,
                glm::vec3(0.78f, 0.79f, 0.82f), 0.2f, 0.4f, true, true);
            return;
        }

        glm::mat4 view = buildViewMatrix();
        glm::mat4 projection = buildProjectionMatrix();
        glUseProgram(m_shaderProgram);
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
        if (m_cameraPosLoc != -1)
            glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
        if (m_fogDensityLoc != -1)
            glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));

        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    else if (object->getType() == "Target" || object->getType() == "Fighter")
    {
        if (isPBRActive())
        {
            m_pbrPipeline->submitLegacyMesh(
                m_targetVAO, static_cast<GLsizei>(m_targetIndices.size()), model,
                glm::vec3(0.7f, 0.72f, 0.74f), 0.2f, 0.4f, true, true);
            return;
        }

        glm::mat4 view = buildViewMatrix();
        glm::mat4 projection = buildProjectionMatrix();
        glUseProgram(m_shaderProgram);
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
        if (m_cameraPosLoc != -1)
            glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
        if (m_fogDensityLoc != -1)
            glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));

        glBindVertexArray(m_targetVAO);
        glDrawElements(GL_TRIANGLES, m_targetIndices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    else
    {
        if (isPBRActive())
        {
            m_pbrPipeline->submitLegacyMesh(
                m_vao, static_cast<GLsizei>(m_indices.size()), model,
                glm::vec3(0.5f, 0.5f, 0.5f), 0.0f, 0.5f);
            return;
        }

        glm::mat4 view = buildViewMatrix();
        glm::mat4 projection = buildProjectionMatrix();
        glUseProgram(m_shaderProgram);
        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(m_viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(m_projLoc, 1, GL_FALSE, glm::value_ptr(projection));
        if (m_cameraPosLoc != -1)
            glUniform3fv(m_cameraPosLoc, 1, glm::value_ptr(m_cameraPosition));
        if (m_fogDensityLoc != -1)
            glUniform1f(m_fogDensityLoc, computeFogDensity(m_sceneFarPlane));

        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Renderer::renderFloor()
{
    glm::mat4 model = glm::mat4(1.0f);

    if (isPBRActive())
    {
        m_pbrPipeline->submitLegacyMesh(
            m_floorVAO, static_cast<GLsizei>(m_floorIndices.size()), model,
            glm::vec3(1.0f), 0.0f, 0.86f, true, false, pbr::Surface::Terrain);
        return;
    }

    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

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

void Renderer::renderWater()
{
    if (!isPBRActive() || !m_terrain || !m_terrain->hasWater() || m_waterVAO == 0)
    {
        return;
    }
    // The plane follows the camera in whole steps so it always reaches the
    // horizon; the shader works in world space, so the waves never slide.
    const float step = 1000.0f;
    const glm::vec3 centre(std::floor(m_cameraPosition.x / step) * step, m_terrain->waterLevel(),
                           std::floor(m_cameraPosition.z / step) * step);
    const glm::mat4 model = glm::translate(glm::mat4(1.0f), centre);
    m_pbrPipeline->submitLegacyMesh(m_waterVAO, m_waterIndexCount, model, glm::vec3(1.0f), 0.0f, 0.05f, false, false,
                                    pbr::Surface::Water);
}

void Renderer::renderEnvironment()
{
    renderFloor();
    renderWater();
}

void Renderer::renderExplosion(const glm::vec3 &position, float size)
{
    if (m_explosionVAO == 0 || size <= 0.0f)
        return;

    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
        return;

    if (std::isnan(size) || std::isinf(size))
        return;

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    model = glm::scale(model, glm::vec3(size));

    if (isPBRActive())
    {
        m_pbrPipeline->submitLegacyMesh(
            m_explosionVAO, static_cast<GLsizei>(m_explosionIndices.size()), model,
            glm::vec3(1.0f, 0.6f, 0.1f), 0.0f, 0.5f);
        return;
    }

    GLint lastBlendSrc = 0, lastBlendDst = 0;
    GLboolean lastBlendEnabled = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_BLEND_SRC, &lastBlendSrc);
    glGetIntegerv(GL_BLEND_DST, &lastBlendDst);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glm::mat4 view = buildViewMatrix();
    glm::mat4 projection = buildProjectionMatrix();

    glUseProgram(m_shaderProgram);
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

    glBindVertexArray(m_explosionVAO);
    glDrawElements(GL_TRIANGLES, m_explosionIndices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

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
