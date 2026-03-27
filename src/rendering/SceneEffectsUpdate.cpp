#include "SceneEffects.h"
#include "SceneEffectsDetail.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

using missilesim::rendering::detail::kMaxHeatHazeSprites;
using missilesim::rendering::detail::kMaxParticles;
using missilesim::rendering::detail::perpendicularTo;
using missilesim::rendering::detail::safeNormalize;
using missilesim::rendering::detail::saturate;

void SceneEffects::update(float deltaTime)
{
    if (!m_initialized)
    {
        return;
    }

    const float dt = glm::clamp(deltaTime, 0.0f, 0.05f);
    if (dt <= 0.0f)
    {
        return;
    }

    for (EffectParticle &particle : m_particles)
    {
        particle.age += dt;
        particle.rotation += particle.angularVelocity * dt;
        particle.velocity += glm::vec3(0.0f, particle.upwardAcceleration, 0.0f) * dt;
        particle.velocity *= (1.0f / (1.0f + (particle.drag * dt)));
        particle.position += particle.velocity * dt;
        particle.axis = safeNormalize(glm::mix(particle.axis, particle.velocity, glm::clamp(dt * 4.0f, 0.0f, 1.0f)),
                                      particle.axis);
    }

    m_particles.erase(std::remove_if(m_particles.begin(), m_particles.end(),
                                     [](const EffectParticle &particle)
                                     {
                                         return particle.age >= particle.lifetime;
                                     }),
                      m_particles.end());

    for (HeatHazeSprite &sprite : m_heatHazeSprites)
    {
        sprite.age += dt;
        sprite.rotation += sprite.angularVelocity * dt;
        sprite.velocity *= (1.0f / (1.0f + (sprite.drag * dt)));
        sprite.position += sprite.velocity * dt;
        sprite.axis = safeNormalize(glm::mix(sprite.axis, sprite.velocity, glm::clamp(dt * 3.0f, 0.0f, 1.0f)),
                                    sprite.axis);
    }

    m_heatHazeSprites.erase(std::remove_if(m_heatHazeSprites.begin(), m_heatHazeSprites.end(),
                                           [](const HeatHazeSprite &sprite)
                                           {
                                               return sprite.age >= sprite.lifetime;
                                           }),
                            m_heatHazeSprites.end());
}