#pragma once

// Game-facing audio interface. Every rendered frame it translates the
// simulation's objects into physical sound sources for the real-time acoustics
// engine (src/audio/engine) and the synthesis voices (src/audio/synth). No
// recorded samples are involved: every sound is generated from the state of
// the object that makes it and propagated to the camera through the air.
//
// Per rendered frame, in this order:
//   beginFrame -> setListener -> syncMissile / syncTargets / syncFlares
//   -> syncCockpitCues -> endFrame
// One-shot events (launch, explosion) and retire/stop calls may happen at any
// point of the frame.

#include <glm/glm.hpp>

#include <memory>
#include <vector>

class Flare;
class Missile;
class Target;

struct AudioWorldState
{
    bool paused = false;
    float timeScale = 1.0f; // simulation speed relative to real time
    float seaLevelAirDensity = 1.225f;
    bool groundPresent = true;
    float groundLevel = 0.0f;
    float masterVolume = 1.0f;
};

struct CockpitCueState
{
    bool seekerPowered = false;
    bool seekerLocked = false;
    float seekerSignal = 0.0f; // 0..1
    bool missileWarning = false;
    float missileWarningUrgency = 0.0f; // 0..1
};

class AudioSystem
{
public:
    AudioSystem();
    ~AudioSystem();

    AudioSystem(const AudioSystem &) = delete;
    AudioSystem &operator=(const AudioSystem &) = delete;

    // Opens the output device. Returns false (and stays silent) without one.
    bool initialize();
    void shutdown();
    bool isEnabled() const;

    void beginFrame(const AudioWorldState &world, float wallDeltaSeconds);
    void setListener(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &up);
    void syncMissile(const Missile *missile, bool inFlight);
    void syncTargets(const std::vector<Target *> &activeTargets);
    void syncFlares(const std::vector<Flare *> &activeFlares);
    void syncCockpitCues(const CockpitCueState &cues);
    void endFrame();

    // Cold-launch ejection from the launch cell.
    void playLaunch(const glm::vec3 &position);
    // Warhead detonation.
    void playExplosion(const glm::vec3 &position);
    // The flare burnt out or was removed; its sizzle tails off naturally.
    void retireFlare(const Flare *flare);
    void stopMissileEmitters();
    void stopAllEmitters();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
