#include "sim/defense/Chaff.h"

#include <algorithm>
#include <cmath>

namespace missilesim::sim
{
    bool releaseChaff(ChaffDispenser &dispenser, std::vector<ChaffRound> &rounds, EntityId id, double time,
                      const glm::vec3 &origin, const glm::vec3 &aircraftVelocity, const glm::vec3 &ejectDirection)
    {
        if (dispenser.remaining <= 0)
        {
            return false;
        }

        // ejectSpeed is a speed, so a non-unit direction still adds that many m/s.
        glm::vec3 eject = ejectDirection;
        const float ejectLength = glm::length(eject);
        if (ejectLength > 1.0e-6f)
        {
            eject /= ejectLength;
        }
        else
        {
            eject = glm::vec3(0.0f);
        }

        ChaffRound round;
        round.id = id;
        round.position = origin;
        round.velocity = aircraftVelocity + eject * dispenser.ejectSpeed;
        round.rcsM2 = dispenser.rcsM2;
        round.birthRcsM2 = dispenser.rcsM2;
        round.birthTime = time;
        round.lifetimeS = dispenser.lifetimeS;
        round.dragTimeS = std::max(dispenser.dragTimeS, 0.0);
        round.fallSpeedMps = std::max(dispenser.fallSpeedMps, 0.0f);
        round.alive = true;
        rounds.push_back(round);
        --dispenser.remaining;
        return true;
    }

    void stepChaff(std::vector<ChaffRound> &rounds, double time, float dt)
    {
        for (ChaffRound &round : rounds)
        {
            if (!round.alive)
            {
                continue;
            }

            const double life = round.lifetimeS;
            const double age = time - round.birthTime;
            // Age is measured from birth. dt is the step since the RCS stored on the round.
            if (!(life > 0.0) || age >= life)
            {
                round.alive = false;
                round.rcsM2 = 0.0f;
                continue;
            }

            if (round.dragTimeS > 0.0)
            {
                // v(t) = fall + (v0 - fall) e^(-t/tau), and its exact integral.
                const glm::vec3 fall(0.0f, -round.fallSpeedMps, 0.0f);
                const glm::vec3 excess = round.velocity - fall;
                const double tau = round.dragTimeS;
                const float decay = static_cast<float>(std::exp(-static_cast<double>(dt) / tau));
                round.position += fall * dt + excess * static_cast<float>(tau * (1.0 - static_cast<double>(decay)));
                round.velocity = fall + excess * decay;
            }
            else
            {
                round.position += round.velocity * dt;
            }
            if (age <= 0.0)
            {
                continue;
            }

            const double previousAge = std::max(0.0, age - static_cast<double>(dt));
            const double previousSpan = life - previousAge;
            const double span = life - age;
            if (!(previousSpan > 0.0))
            {
                round.rcsM2 = 0.0f;
                continue;
            }
            const double scaled = static_cast<double>(round.rcsM2) * (span / previousSpan);
            round.rcsM2 = scaled > 0.0 ? static_cast<float>(scaled) : 0.0f;
        }
    }
}
