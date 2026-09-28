#include "Renderer.h"
#include "SceneEffects.h"
#include "../objects/Missile.h"
#include "../objects/Target.h"

#include <algorithm>
#include <cmath>

std::vector<Renderer::ExhaustSocket> Renderer::getExhaustSockets(const PhysicsObject &object) const
{
    const auto *local = object.getType() == "Missile" ? &m_missileExhaustSockets :
                        object.getType() == "Target" ? &m_targetExhaustSockets : nullptr;
    if (!local) return {};
    const glm::mat4 model = buildObjectModelMatrix(object);
    std::vector<ExhaustSocket> sockets = *local;
    for (auto &socket : sockets)
    {
        socket.position = glm::vec3(model * glm::vec4(socket.position, 1.0f));
        socket.direction = glm::normalize(glm::mat3(model) * socket.direction);
        socket.radius *= glm::length(glm::vec3(model[0]));
    }
    return sockets;
}

void Renderer::submitEnginePlumes(const PhysicsObject &object)
{
    if (!m_sceneEffects) return;
    const bool rocket = object.getType() == "Missile";
    float throttle = 0.0f;
    if (rocket)
    {
        const auto &missile = static_cast<const Missile &>(object);
        if (!missile.isThrustEnabled() || missile.getFuel() <= 0.0f) return;
        throttle = missile.getThrottle();
    }
    else if (object.getType() == "Target")
    {
        const auto &target = static_cast<const Target &>(object);
        if (!target.isActive()) return;
        throttle = target.getThrottle();
    }
    else return;
    if (!std::isfinite(throttle) || throttle <= 0.01f) return;
    throttle = std::clamp(throttle, 0.0f, 1.0f);

    for (const auto &socket : getExhaustSockets(object))
    {
        m_sceneEffects->submitEnginePlume(socket.position, socket.direction, socket.radius, throttle, rocket);
        // Local nozzle illumination; no large light radius or frame stacking.
        EffectLight light;
        light.position = socket.position + socket.direction * socket.radius;
        light.color = rocket ? glm::vec3(1.0f, 0.48f, 0.14f) : glm::vec3(0.65f, 0.55f, 1.0f);
        light.intensity = socket.radius * socket.radius * (rocket ? 14.0f : 8.0f) * throttle;
        light.radius = socket.radius * 10.0f;
        light.seed = socket.radius;
        addEffectLight(light);
    }
}
