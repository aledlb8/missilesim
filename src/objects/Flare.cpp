#include "Flare.h"
#include <algorithm>
#include <cmath>

Flare::Flare(const FlareLaunchRequest &request)
    : PhysicsObject(request.position, request.velocity, std::max(request.mass, 0.01f)),
      m_dragCoefficient(std::max(request.dragCoefficient, 0.0f)),
      m_crossSectionalArea(std::max(request.crossSectionalArea, 0.0001f)),
      m_lifetimeRemaining(std::max(request.lifetime, 0.0f)),
      m_initialHeatSignature(std::max(request.heatSignature, 0.0f)),
      m_currentHeatSignature(std::max(request.heatSignature, 0.0f)),
      m_heatDecayRate(std::max(request.heatDecayRate, 0.0f))
{
    if (m_lifetimeRemaining <= 0.0f || m_initialHeatSignature <= 0.0f)
    {
        m_isActive = false;
        m_currentHeatSignature = 0.0f;
    }
}

void Flare::update(float deltaTime)
{
    if (!m_isActive)
    {
        return;
    }

    if (deltaTime <= 0.0f || std::isnan(deltaTime) || std::isinf(deltaTime))
    {
        return;
    }

    PhysicsObject::update(deltaTime);

    m_lifetimeRemaining = std::max(m_lifetimeRemaining - deltaTime, 0.0f);
    m_currentHeatSignature *= std::exp(-m_heatDecayRate * deltaTime);

    if (m_lifetimeRemaining <= 0.0f || m_currentHeatSignature <= 0.01f)
    {
        m_lifetimeRemaining = 0.0f;
        m_currentHeatSignature = 0.0f;
        m_isActive = false;
    }
}
