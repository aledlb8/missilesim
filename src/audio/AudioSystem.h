#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

class Missile;
class Target;
class Flare;

class AudioSystem
{
public:
    AudioSystem();
    ~AudioSystem();

    bool initialize();
    void shutdown();

    bool isEnabled() const;

    void update(float deltaTime);
    void setListener(const glm::vec3 &position,
                     const glm::vec3 &forward,
                     const glm::vec3 &up,
                     float deltaTime);
    void syncMissile(const Missile *missile, bool missileInFlight, float referenceFuel);
    void syncTargets(const std::vector<Target *> &activeTargets);
    void syncFlares(const std::vector<Flare *> &activeFlares);
    void syncCockpitCues(bool seekerPowered,
                         bool seekerLocked,
                         float seekerSignalStrength,
                         bool mawsThreatActive,
                         float mawsUrgency);

    void playLaunch(const glm::vec3 &position, const glm::vec3 &velocity);
    void playFlareLaunch(const glm::vec3 &position, const glm::vec3 &velocity, float heatSignature);
    void playExplosion(const glm::vec3 &position, const glm::vec3 &velocity, float intensity);
    void retireFlare(const Flare *flare);
    void stopMissileEmitters();
    void stopAllEmitters();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
