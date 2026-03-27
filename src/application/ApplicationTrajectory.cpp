#include "Application.h"
#include "ApplicationDetail.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <unordered_map>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

#include "audio/AudioSystem.h"
#include "objects/Flare.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/Atmosphere.h"
#include "physics/PhysicsEngine.h"
#include "physics/forces/Drag.h"
#include "physics/forces/Lift.h"
#include "rendering/Renderer.h"

using missilesim::application::detail::formatBoolValue;
using missilesim::application::detail::formatVec3Value;
using missilesim::application::detail::parseBoolValue;
using missilesim::application::detail::parseFloatValue;
using missilesim::application::detail::parseIntValue;
using missilesim::application::detail::parseVec3Value;
using missilesim::application::detail::safeNormalize;
using missilesim::application::detail::trimWhitespace;

Application::TrajectoryPreviewConfig Application::captureTrajectoryPreviewConfig() const
{
    TrajectoryPreviewConfig config;
    if (!m_missile)
    {
        return config;
    }

    config.thrustDirection = m_missile->getThrustDirection();
    config.dryMass = m_missile->getDryMass();
    config.dragCoefficient = m_missile->getDragCoefficient();
    config.crossSectionalArea = m_missile->getCrossSectionalArea();
    config.liftCoefficient = m_missile->getLiftCoefficient();
    config.guidanceEnabled = m_missile->isGuidanceEnabled();
    config.navigationGain = m_missile->getNavigationGain();
    config.maxSteeringForce = m_missile->getMaxSteeringForce();
    config.trackingAngle = m_missile->getTrackingAngle();
    config.proximityFuseRadius = m_missile->getProximityFuseRadius();
    config.countermeasureResistance = m_missile->getCountermeasureResistance();
    config.terrainAvoidanceEnabled = m_missile->isTerrainAvoidanceEnabled();
    config.terrainClearance = m_missile->getTerrainClearance();
    config.terrainLookAheadTime = m_missile->getTerrainLookAheadTime();
    config.thrust = m_missile->getThrust();
    config.thrustEnabled = m_missile->isThrustEnabled();
    config.fuel = m_missile->getFuel();
    config.fuelConsumptionRate = m_missile->getFuelConsumptionRate();
    config.trajectoryPoints = m_trajectoryPoints;
    config.trajectoryTime = m_trajectoryTime;
    config.gravityMagnitude = m_physicsEngine ? m_physicsEngine->getGravity() : 9.81f;
    config.airDensity = m_physicsEngine ? m_physicsEngine->getAirDensity() : m_savedAirDensity;
    return config;
}

bool Application::shouldRefreshTrajectoryPreviewCache(Target *target, const TrajectoryPreviewConfig &config) const
{
    if (!m_missile || !target)
    {
        return false;
    }

    if (!m_trajectoryPreviewCache.valid ||
        m_trajectoryPreviewCache.target != target ||
        m_trajectoryPreviewCache.missilePoints.empty())
    {
        return true;
    }

    const TrajectoryPreviewConfig &cachedConfig = m_trajectoryPreviewCache.config;
    const bool configChanged =
        cachedConfig.thrustDirection.x != config.thrustDirection.x ||
        cachedConfig.thrustDirection.y != config.thrustDirection.y ||
        cachedConfig.thrustDirection.z != config.thrustDirection.z ||
        cachedConfig.dryMass != config.dryMass ||
        cachedConfig.dragCoefficient != config.dragCoefficient ||
        cachedConfig.crossSectionalArea != config.crossSectionalArea ||
        cachedConfig.liftCoefficient != config.liftCoefficient ||
        cachedConfig.guidanceEnabled != config.guidanceEnabled ||
        cachedConfig.navigationGain != config.navigationGain ||
        cachedConfig.maxSteeringForce != config.maxSteeringForce ||
        cachedConfig.trackingAngle != config.trackingAngle ||
        cachedConfig.proximityFuseRadius != config.proximityFuseRadius ||
        cachedConfig.countermeasureResistance != config.countermeasureResistance ||
        cachedConfig.terrainAvoidanceEnabled != config.terrainAvoidanceEnabled ||
        cachedConfig.terrainClearance != config.terrainClearance ||
        cachedConfig.terrainLookAheadTime != config.terrainLookAheadTime ||
        cachedConfig.thrust != config.thrust ||
        cachedConfig.thrustEnabled != config.thrustEnabled ||
        cachedConfig.fuel != config.fuel ||
        cachedConfig.fuelConsumptionRate != config.fuelConsumptionRate ||
        cachedConfig.trajectoryPoints != config.trajectoryPoints ||
        cachedConfig.trajectoryTime != config.trajectoryTime ||
        cachedConfig.gravityMagnitude != config.gravityMagnitude ||
        cachedConfig.airDensity != config.airDensity;

    if (configChanged)
    {
        return true;
    }

    const auto cacheAge = std::chrono::steady_clock::now() - m_trajectoryPreviewCache.lastRefresh;
    if (cacheAge < m_trajectoryPreviewRefreshInterval)
    {
        return false;
    }

    const glm::vec3 missilePos = m_missile->getPosition();
    const glm::vec3 missileVel = m_missile->getVelocity();
    const glm::vec3 targetPos = target->getPosition();
    const glm::vec3 targetVel = target->getVelocity();

    const float positionThresholdSq = 4.0f;
    const float velocityThresholdSq = 16.0f;
    const bool missileMoved = glm::distance2(missilePos, m_trajectoryPreviewCache.lastMissilePosition) > positionThresholdSq ||
                              glm::distance2(missileVel, m_trajectoryPreviewCache.lastMissileVelocity) > velocityThresholdSq;
    const bool targetMoved = glm::distance2(targetPos, m_trajectoryPreviewCache.lastTargetPosition) > positionThresholdSq ||
                             glm::distance2(targetVel, m_trajectoryPreviewCache.lastTargetVelocity) > velocityThresholdSq;

    return missileMoved || targetMoved;
}

void Application::updateTrajectoryPreviewCache(Target *target, const TrajectoryPreviewConfig &config)
{
    invalidateTrajectoryPreviewCache();
    if (!m_missile || !target || config.trajectoryPoints < 2 || config.trajectoryTime <= 0.0f)
    {
        return;
    }

    const glm::vec3 missilePos = m_missile->getPosition();
    const glm::vec3 missileVel = m_missile->getVelocity();
    const glm::vec3 targetPos = target->getPosition();
    const glm::vec3 targetVel = target->getVelocity();

    m_trajectoryPreviewCache.missilePoints.reserve(static_cast<std::size_t>(config.trajectoryPoints));
    m_trajectoryPreviewCache.targetPoints.reserve(static_cast<std::size_t>(std::max(1, config.trajectoryPoints / 5)));
    m_trajectoryPreviewCache.missilePoints.push_back(missilePos);
    m_trajectoryPreviewCache.interceptPoint = predictInterceptPoint(missilePos, missileVel, targetPos, targetVel);

    Missile simMissile(missilePos, missileVel, config.dryMass, config.dragCoefficient,
                       config.crossSectionalArea, config.liftCoefficient);
    simMissile.setGuidanceEnabled(config.guidanceEnabled);
    simMissile.setNavigationGain(config.navigationGain);
    simMissile.setMaxSteeringForce(config.maxSteeringForce);
    simMissile.setTrackingAngle(config.trackingAngle);
    simMissile.setProximityFuseRadius(config.proximityFuseRadius);
    simMissile.setCountermeasureResistance(config.countermeasureResistance);
    simMissile.setTerrainAvoidanceEnabled(config.terrainAvoidanceEnabled);
    simMissile.setTerrainClearance(config.terrainClearance);
    simMissile.setTerrainLookAheadTime(config.terrainLookAheadTime);
    simMissile.setGroundReferenceAltitude(m_physicsEngine ? m_physicsEngine->getGroundLevel() : 0.0f);
    simMissile.setThrust(config.thrust);
    simMissile.setThrustEnabled(config.thrustEnabled);
    simMissile.setFuel(config.fuel);
    simMissile.setFuelConsumptionRate(config.fuelConsumptionRate);
    simMissile.setThrustDirection(config.thrustDirection);

    const float dt = config.trajectoryTime / static_cast<float>(config.trajectoryPoints);
    Atmosphere atmosphere(config.airDensity);
    Drag drag(&atmosphere);
    Lift lift(&atmosphere);

    Target simTarget = *target;
    std::vector<Missile *> simulatedMissiles = {&simMissile};

    for (int i = 1; i < config.trajectoryPoints; ++i)
    {
        simMissile.resetForces();
        simMissile.applyForce(glm::vec3(0.0f, -config.gravityMagnitude * simMissile.getMass(), 0.0f));
        drag.applyTo(&simMissile);
        lift.applyTo(&simMissile);
        simTarget.updateThreatAssessment(simulatedMissiles);
        simTarget.update(dt);

        if (i % 5 == 0)
        {
            m_trajectoryPreviewCache.targetPoints.push_back(simTarget.getPosition());
        }

        simMissile.setTargetObject(&simTarget);
        if (simMissile.isGuidanceEnabled() && simMissile.hasTarget())
        {
            simMissile.applyGuidance(dt);
            if (simMissile.consumeSelfDestructRequest())
            {
                m_trajectoryPreviewCache.missilePoints.push_back(simMissile.getPosition());
                break;
            }
        }

        simMissile.update(dt);
        m_trajectoryPreviewCache.missilePoints.push_back(simMissile.getPosition());
    }

    m_trajectoryPreviewCache.lastMissilePosition = missilePos;
    m_trajectoryPreviewCache.lastMissileVelocity = missileVel;
    m_trajectoryPreviewCache.lastTargetPosition = targetPos;
    m_trajectoryPreviewCache.lastTargetVelocity = targetVel;
    m_trajectoryPreviewCache.config = config;
    m_trajectoryPreviewCache.target = target;
    m_trajectoryPreviewCache.lastRefresh = std::chrono::steady_clock::now();
    m_trajectoryPreviewCache.valid = !m_trajectoryPreviewCache.missilePoints.empty();
}

void Application::invalidateTrajectoryPreviewCache()
{
    m_trajectoryPreviewCache.missilePoints.clear();
    m_trajectoryPreviewCache.targetPoints.clear();
    m_trajectoryPreviewCache.valid = false;
    m_trajectoryPreviewCache.target = nullptr;
    m_trajectoryPreviewCache.lastRefresh = std::chrono::steady_clock::time_point{};
}

void Application::renderPredictedTrajectory()
{
    if (!m_missile || !m_renderer)
    {
        return;
    }

    try
    {
        Target *target = getTrackedMissileTarget();
        if (!target)
        {
            invalidateTrajectoryPreviewCache();
            return;
        }

        const TrajectoryPreviewConfig config = captureTrajectoryPreviewConfig();
        if (shouldRefreshTrajectoryPreviewCache(target, config))
        {
            updateTrajectoryPreviewCache(target, config);
        }

        if (!m_trajectoryPreviewCache.valid || m_trajectoryPreviewCache.missilePoints.empty())
        {
            return;
        }

        if (m_showInterceptPoint)
        {
            const glm::vec3 &startPoint = m_trajectoryPreviewCache.missilePoints.front();
            const glm::vec3 &targetPoint = m_trajectoryPreviewCache.lastTargetPosition;
            m_renderer->renderLine(startPoint, m_trajectoryPreviewCache.interceptPoint, glm::vec3(1.0f, 0.0f, 0.0f));
            m_renderer->renderLine(m_trajectoryPreviewCache.interceptPoint, targetPoint, glm::vec3(0.0f, 1.0f, 0.0f));
            m_renderer->renderPoint(m_trajectoryPreviewCache.interceptPoint, glm::vec3(1.0f, 1.0f, 0.0f), 7.0f);
        }

        if (m_showPredictedTargetPath)
        {
            for (const glm::vec3 &predictedTargetPoint : m_trajectoryPreviewCache.targetPoints)
            {
                m_renderer->renderPoint(predictedTargetPoint, glm::vec3(0.0f, 1.0f, 0.0f), 3.0f);
            }
        }

        for (std::size_t i = 1; i < m_trajectoryPreviewCache.missilePoints.size(); ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(m_trajectoryPreviewCache.missilePoints.size());
            const glm::vec3 color(1.0f, 1.0f - t, 0.0f);
            m_renderer->renderLine(m_trajectoryPreviewCache.missilePoints[i - 1], m_trajectoryPreviewCache.missilePoints[i], color);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: Exception in renderPredictedTrajectory: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ERROR: Unknown exception in renderPredictedTrajectory" << std::endl;
    }
}

glm::vec3 Application::predictInterceptPoint(const glm::vec3 &missilePos, const glm::vec3 &missileVel,
                                             const glm::vec3 &targetPos, const glm::vec3 &targetVel)
{
    try
    {
        float missileSpeed = glm::length(missileVel);
        if (missileSpeed < 0.1f)
        {
            return targetPos;
        }

        const glm::vec3 relativePosition = targetPos - missilePos;
        const float missileSpeedSq = missileSpeed * missileSpeed;
        const float a = glm::dot(targetVel, targetVel) - missileSpeedSq;
        const float b = 2.0f * glm::dot(relativePosition, targetVel);
        const float c = glm::dot(relativePosition, relativePosition);
        const float epsilon = 0.0001f;

        float timeToIntercept = 0.0f;
        bool hasInterceptSolution = false;

        if (std::abs(a) < epsilon)
        {
            if (std::abs(b) > epsilon)
            {
                const float candidate = -c / b;
                if (candidate > 0.0f)
                {
                    timeToIntercept = candidate;
                    hasInterceptSolution = true;
                }
            }
        }
        else
        {
            const float discriminant = (b * b) - (4.0f * a * c);
            if (discriminant >= 0.0f)
            {
                const float sqrtDiscriminant = std::sqrt(discriminant);
                const float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
                const float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

                if (t1 > 0.0f && t2 > 0.0f)
                {
                    timeToIntercept = std::min(t1, t2);
                    hasInterceptSolution = true;
                }
                else if (t1 > 0.0f)
                {
                    timeToIntercept = t1;
                    hasInterceptSolution = true;
                }
                else if (t2 > 0.0f)
                {
                    timeToIntercept = t2;
                    hasInterceptSolution = true;
                }
            }
        }

        if (!hasInterceptSolution)
        {
            timeToIntercept = glm::length(relativePosition) / missileSpeed;
        }

        timeToIntercept = glm::clamp(timeToIntercept, 0.0f, 20.0f);
        return targetPos + targetVel * timeToIntercept;
    }
    catch (...)
    {
        return targetPos;
    }
}