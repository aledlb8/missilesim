#include "FixedStepClock.h"

#include <algorithm>
#include <cmath>

namespace missilesim::sim
{
    namespace
    {
        constexpr float kDefaultStepSeconds = 0.01f;
        constexpr float kMinimumStepSeconds = 1.0e-4f;
    }

    FixedStepClock::FixedStepClock(float stepSeconds)
    {
        setStep(stepSeconds);
    }

    void FixedStepClock::setStep(float stepSeconds)
    {
        m_step = (std::isfinite(stepSeconds) && stepSeconds >= kMinimumStepSeconds) ? stepSeconds : kDefaultStepSeconds;
    }

    int FixedStepClock::advance(float frameSeconds, float timeScale)
    {
        const float frame = std::isfinite(frameSeconds) ? std::clamp(frameSeconds, 0.0f, kMaxFrameSeconds) : 0.0f;
        const float scale = (std::isfinite(timeScale) && timeScale > 0.0f) ? timeScale : 0.0f;
        m_accumulator += static_cast<double>(frame) * static_cast<double>(scale);

        const double step = static_cast<double>(m_step);
        const auto owed = static_cast<long long>(std::floor(m_accumulator / step));
        const int steps = static_cast<int>(std::min<long long>(std::max<long long>(owed, 0), kMaxStepsPerFrame));
        m_accumulator -= static_cast<double>(steps) * step;

        const double backlogCap = static_cast<double>(kMaxBacklogSeconds);
        if (m_accumulator > backlogCap)
        {
            m_droppedSeconds += m_accumulator - backlogCap;
            m_accumulator = backlogCap;
        }
        return steps;
    }

    float FixedStepClock::alpha() const
    {
        const double fraction = m_accumulator / static_cast<double>(m_step);
        return static_cast<float>(std::clamp(fraction, 0.0, 1.0));
    }

    void FixedStepClock::reset()
    {
        m_accumulator = 0.0;
        m_droppedSeconds = 0.0;
    }
}
