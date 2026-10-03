#pragma once

// One dispensed chaff bundle. The dispenser only spends a finite store and
// appends a round. RCS falls linearly to zero over the round's life. A dead
// round stays in the vector so compaction cannot reuse its slot.
//
// The cloud is light dipoles: the air stops it. Its velocity relaxes from the
// aircraft's toward a slow fall with time constant dragTimeS, so within a
// second it hangs nearly still while the aircraft flies on. That is what a
// Doppler radar tells it apart by. dragTimeS of 0 keeps the release velocity.

#include "sim/EntityId.h"

#include <vector>

#include <glm/glm.hpp>

namespace missilesim::sim
{
    struct ChaffRound
    {
        EntityId id;
        glm::vec3 position{0.0f};
        glm::vec3 velocity{0.0f};
        float rcsM2 = 0.0f;
        // RCS at release. Presentation uses rcs/birth. The decay below does not read it.
        float birthRcsM2 = 0.0f;
        double birthTime = 0.0;
        double lifetimeS = 8.0;
        double dragTimeS = 0.0;    // velocity relaxation time; 0 keeps the velocity
        float fallSpeedMps = 0.0f; // settled sink rate, straight down
        bool alive = false;
    };

    struct ChaffDispenser
    {
        int remaining = 0; // finite. 0 refuses.
        float rcsM2 = 20.0f;
        double lifetimeS = 8.0;
        float ejectSpeed = 30.0f; // m/s, added along the eject direction, plus the aircraft velocity
        double dragTimeS = 0.0;   // copied to each round
        float fallSpeedMps = 0.0f;
    };

    // Returns false when remaining is 0. On success, decrements remaining and appends one alive round.
    // ejectDirection is where the cartridge points (any length; zero ejects at the aircraft velocity).
    bool releaseChaff(ChaffDispenser &dispenser, std::vector<ChaffRound> &rounds, EntityId id, double time,
                      const glm::vec3 &origin, const glm::vec3 &aircraftVelocity, const glm::vec3 &ejectDirection);

    // Alive rounds move with their velocity, which relaxes toward the fall
    // (integrated exactly, so the step size does not change the path). RCS
    // decays linearly to 0 at lifetime.
    // A round whose age exceeds lifetime is not alive and its RCS is 0. Entries are not erased.
    void stepChaff(std::vector<ChaffRound> &rounds, double time, float dt);
}
