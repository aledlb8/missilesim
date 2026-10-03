#pragma once

// Game-facing audio interface. Every rendered frame it translates the
// simulation's objects into physical sound sources for the real-time acoustics
// engine (src/audio/engine) and the synthesis voices (src/audio/synth). No
// recorded samples are involved: every sound is generated from the state of
// the object that makes it and propagated to the camera through the air.
//
// Per rendered frame, in this order:
//   beginFrame -> setListener -> syncMissiles / syncTargets / syncFlares
//   / syncChaff -> syncCockpitCues -> endFrame
// One-shot events (launch, explosion) and stop calls may happen at any point
// of the frame.
//
// Sources are keyed by their simulation entity id, never by address: an id is
// not reused while the world runs, so a new flare allocated where an old one
// was can never inherit the old one's voice. A source missing from a sync
// call is released and its sound tails off naturally.

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <vector>

class Flare;
class Missile;
class Target;

template <typename Object>
struct AudioSource
{
    std::uint32_t id = 0;
    const Object *object = nullptr;
};

using AudioMissileSource = AudioSource<Missile>;
using AudioTargetSource = AudioSource<Target>;
using AudioFlareSource = AudioSource<Flare>;

// A dispensed chaff bundle. Not a physics object: the game fills this from the
// simulation each frame and the voice follows bloom (current RCS / birth RCS).
struct AudioChaffState
{
    glm::vec3 position{0.0f};
    glm::vec3 velocity{0.0f};
    float bloom = 0.0f;
};

using AudioChaffSource = AudioSource<AudioChaffState>;

struct AudioWorldState
{
    bool paused = false;
    float timeScale = 1.0f; // simulation speed relative to real time
    float seaLevelAirDensity = 1.225f;
    bool groundPresent = true;
    float groundLevel = 0.0f;
    float masterVolume = 1.0f;
};

// Headset alerts the pilot can actually hear. None keeps the older missile-warning flag,
// which the SAM camera still uses for its kinematic approach tone.
enum class HeadsetAlert : std::uint8_t
{
    None = 0,
    RadarSearch,
    RadarTrack,
    RadarLaunch,
    MissileSeeker,
    Approach,
};

struct CockpitCueState
{
    bool seekerPowered = false;
    bool seekerLocked = false;
    float seekerSignal = 0.0f; // 0..1
    bool missileWarning = false;
    float missileWarningUrgency = 0.0f; // 0..1
    HeadsetAlert alert = HeadsetAlert::None;
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
    // Rounds in flight (a loaded round makes no sound).
    void syncMissiles(const std::vector<AudioMissileSource> &missiles);
    void syncTargets(const std::vector<AudioTargetSource> &activeTargets);
    void syncFlares(const std::vector<AudioFlareSource> &activeFlares);
    void syncChaff(const std::vector<AudioChaffSource> &activeChaff);
    void syncCockpitCues(const CockpitCueState &cues);
    void endFrame();

    // Cold-launch ejection from the launch cell.
    void playLaunch(const glm::vec3 &position);
    // Warhead detonation.
    void playExplosion(const glm::vec3 &position);
    void stopAllEmitters();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
