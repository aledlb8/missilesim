#include "sim/defense/Warnings.h"

#include "sim/defense/Chaff.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace missilesim::sim
{
    namespace
    {
        struct Sight
        {
            float azimuthRad = 0.0f;
            float rangeM = 0.0f;
            glm::vec3 lineOfSight{0.0f, 0.0f, 1.0f};
            bool separated = false;
        };

        // Same axes as a sensor look: azimuth is positive toward the right wing.
        // A coincident source has no bearing and reads as dead ahead.
        Sight sightFrom(const SensorAxes &axes, const glm::vec3 &from, const glm::vec3 &to)
        {
            const SensorBearing bearing = bearingFrom(axes, from, to);
            Sight sight;
            sight.rangeM = bearing.rangeM;
            sight.lineOfSight = bearing.lineOfSight;
            sight.separated = bearing.rangeM >= 1.0e-3f;
            sight.azimuthRad = sight.separated ? bearing.azimuthRad : 0.0f;
            return sight;
        }

        // Cone about the tail, not the nose. A contact at zero range is inside it.
        bool inRearCone(const SensorAxes &axes, const Sight &sight, float halfRad)
        {
            if (!sight.separated)
            {
                return halfRad >= 0.0f;
            }
            const float cosine = std::clamp(glm::dot(sight.lineOfSight, -axes.forward), -1.0f, 1.0f);
            return std::acos(cosine) <= halfRad;
        }

        SensorBody defended(EntityId id, const glm::vec3 &position)
        {
            SensorBody body;
            body.id = id;
            body.position = position;
            body.forward = glm::vec3(0.0f, 0.0f, 1.0f);
            body.up = glm::vec3(0.0f, 1.0f, 0.0f);
            body.alive = true;
            return body;
        }

        constexpr float kPi = 3.14159265f;
    }

    bool defenseAllowed(double firstWarningTime, double now, double reactionDelayS)
    {
        // No warning has been heard, so a missile that already exists is not enough.
        if (!(firstWarningTime >= 0.0) || now < firstWarningTime)
        {
            return false;
        }
        return !(now < firstWarningTime + reactionDelayS);
    }

    WarningPicture hear(double time, const WarningSet &set, const SensorBody &ownship, const Terrain &terrain,
                        const std::vector<Emission> &emissions, const std::vector<ClosingBody> &closing)
    {
        WarningPicture picture;
        const SensorAxes axes = sensorAxes(ownship);

        if (set.radarWarning)
        {
            for (const Emission &emission : emissions)
            {
                // Infrared energy is not an RWR hit, including an infrared missile seeker.
                if (emission.family != SensorFamily::Radar)
                {
                    continue;
                }
                if (!terrain.lineOfSight(ownship.position, emission.position))
                {
                    continue;
                }

                const Sight sight = sightFrom(axes, ownship.position, emission.position);
                Warning warning;
                if (emission.missileSeeker)
                {
                    warning.kind = WarningKind::MissileSeeker;
                }
                else if (emission.singleTargetTrack)
                {
                    warning.kind = emission.guidingMissile ? WarningKind::RadarLaunch : WarningKind::RadarTrack;
                }
                else
                {
                    warning.kind = WarningKind::RadarSearch;
                }
                warning.source = emission.source;
                warning.time = time;
                warning.azimuthRad = sight.azimuthRad;
                warning.rangeM = sight.rangeM;
                picture.radar.push_back(warning);
            }
        }

        // Approach is MAWS geometry on closing bodies. A radar emission is not copied here.
        if (set.approachWarning)
        {
            for (const ClosingBody &body : closing)
            {
                if (!terrain.lineOfSight(ownship.position, body.position))
                {
                    continue;
                }
                const Sight sight = sightFrom(axes, ownship.position, body.position);
                if (sight.rangeM > set.approachMaxRangeM || !inRearCone(axes, sight, set.approachConeHalfRad))
                {
                    continue;
                }
                // Closure is the body's own velocity toward the ownship; the warner does not use its own motion.
                if (sight.separated && glm::dot(body.velocity, ownship.position - body.position) <= 0.0f)
                {
                    continue;
                }

                Warning warning;
                warning.kind = WarningKind::Approach;
                warning.source = body.id;
                warning.time = time;
                warning.azimuthRad = sight.azimuthRad;
                warning.rangeM = sight.rangeM;
                picture.approach.push_back(warning);
            }
        }

        return picture;
    }

    int runDefenseChecks()
    {
        int failures = 0;
        int count = 0;
        const auto expect = [&](bool passed, const char *name, const char *detail) {
            ++count;
            if (!passed)
            {
                ++failures;
            }
            std::printf("%s  %-44s %s\n", passed ? "PASS" : "FAIL", name, detail);
        };

        const Terrain flat;
        const SensorBody ownship = defended(EntityId{1}, glm::vec3(0.0f, 120.0f, 0.0f));

        Emission search;
        search.source = EntityId{2};
        search.position = glm::vec3(2500.0f, 120.0f, 0.0f); // +X: the left wing of a +Z nose
        search.family = SensorFamily::Radar;
        search.missileSeeker = false;
        search.singleTargetTrack = false;

        Emission tracked = search;
        tracked.source = EntityId{4};
        tracked.position = glm::vec3(0.0f, 120.0f, 1800.0f);
        tracked.singleTargetTrack = true;

        const WarningPicture searchPicture = hear(6.5, WarningSet{}, ownship, flat, {search, tracked}, {});
        const bool searchOk = searchPicture.radar.size() == 2 && searchPicture.approach.empty() &&
                              searchPicture.radar[0].kind == WarningKind::RadarSearch &&
                              searchPicture.radar[0].source == search.source && searchPicture.radar[0].time == 6.5 &&
                              std::abs(searchPicture.radar[0].azimuthRad + kPi * 0.5f) < 1.0e-4f &&
                              std::abs(searchPicture.radar[0].rangeM - 2500.0f) < 1.0e-2f &&
                              searchPicture.radar[1].kind == WarningKind::RadarTrack &&
                              searchPicture.radar[1].source == tracked.source &&
                              std::abs(searchPicture.radar[1].azimuthRad) < 1.0e-4f &&
                              std::abs(searchPicture.radar[1].rangeM - 1800.0f) < 1.0e-2f;
        expect(searchOk, "rwr: search is heard and approach stays empty", searchOk ? "" : "search or track geometry");

        // A held beam that also guides a round is a launch; without the round it stays a track.
        Emission guiding = tracked;
        guiding.source = EntityId{5};
        guiding.guidingMissile = true;
        Emission searchGuiding = search;
        searchGuiding.source = EntityId{6};
        searchGuiding.guidingMissile = true;
        const WarningPicture launchPicture = hear(7.0, WarningSet{}, ownship, flat, {guiding, searchGuiding}, {});
        const bool launchOk = launchPicture.radar.size() == 2 && launchPicture.approach.empty() &&
                              launchPicture.radar[0].kind == WarningKind::RadarLaunch &&
                              launchPicture.radar[0].source == guiding.source &&
                              launchPicture.radar[1].kind == WarningKind::RadarSearch;
        expect(launchOk, "rwr: a held beam guiding a round is a launch", launchOk ? "" : "launch kind");

        WarningSet noRadar;
        noRadar.radarWarning = false;
        const WarningPicture silenced = hear(6.5, noRadar, ownship, flat, {search}, {});
        expect(silenced.radar.empty() && silenced.approach.empty(), "rwr: no receiver hears no radar", "");

        TerrainConfig ridgeConfig;
        ridgeConfig.kind = TerrainKind::Ridge;
        ridgeConfig.baseHeightM = 0.0f;
        ridgeConfig.ridgeHeightM = 400.0f;
        ridgeConfig.ridgeDistanceM = 2000.0f;
        ridgeConfig.ridgeHalfWidthM = 600.0f;
        ridgeConfig.ridgeBearingDeg = 0.0f;
        ridgeConfig.hillAmplitudeM = 0.0f;
        const Terrain ridge(ridgeConfig);

        SensorBody lowOwn = defended(EntityId{1}, glm::vec3(0.0f, 50.0f, 0.0f));
        Emission lowEmission = search;
        lowEmission.source = EntityId{8};
        lowEmission.position = glm::vec3(0.0f, 50.0f, 4000.0f);
        lowEmission.singleTargetTrack = false;
        const bool lowBlocked = !ridge.lineOfSight(lowOwn.position, lowEmission.position);
        const WarningPicture lowPicture = hear(1.0, WarningSet{}, lowOwn, ridge, {lowEmission}, {});

        SensorBody highOwn = lowOwn;
        highOwn.position.y = 900.0f;
        Emission highEmission = lowEmission;
        highEmission.position.y = 900.0f;
        const bool highClear = ridge.lineOfSight(highOwn.position, highEmission.position);
        const WarningPicture highPicture = hear(1.0, WarningSet{}, highOwn, ridge, {highEmission}, {});
        const bool highHeard = highPicture.radar.size() == 1 && highPicture.approach.empty() &&
                               highPicture.radar.front().kind == WarningKind::RadarSearch &&
                               highPicture.radar.front().source == highEmission.source &&
                               std::abs(highPicture.radar.front().azimuthRad) < 1.0e-4f &&
                               std::abs(highPicture.radar.front().rangeM - 4000.0f) < 1.0e-2f;
        char ridgeDetail[96];
        std::snprintf(ridgeDetail, sizeof(ridgeDetail), "blocked %d clear %d low %u high %u", lowBlocked ? 1 : 0,
                      highClear ? 1 : 0, static_cast<unsigned>(lowPicture.radar.size()),
                      static_cast<unsigned>(highPicture.radar.size()));
        expect(lowBlocked && lowPicture.radar.empty() && lowPicture.approach.empty() && highClear && highHeard,
               "rwr: the ridge hides the low path only", ridgeDetail);

        Emission infrared;
        infrared.source = EntityId{9};
        infrared.position = glm::vec3(0.0f, 80.0f, -1500.0f);
        infrared.family = SensorFamily::Infrared;
        infrared.missileSeeker = true;

        ClosingBody heat;
        heat.id = infrared.source;
        heat.position = infrared.position;
        heat.velocity = glm::vec3(0.0f, 0.0f, 400.0f);
        heat.infraredOnly = true;

        ClosingBody heatFar = heat;
        heatFar.id = EntityId{10};
        heatFar.position = glm::vec3(0.0f, 80.0f, -9000.0f);

        ClosingBody heatAhead = heat;
        heatAhead.id = EntityId{11};
        heatAhead.position = glm::vec3(0.0f, 80.0f, 1500.0f);
        heatAhead.velocity = glm::vec3(0.0f, 0.0f, -400.0f);

        ClosingBody opening = heat;
        opening.id = EntityId{12};
        opening.velocity = glm::vec3(0.0f, 0.0f, -50.0f);

        const SensorBody defender = defended(EntityId{3}, glm::vec3(0.0f, 80.0f, 0.0f));
        const std::vector<ClosingBody> heatBodies{heat, heatFar, heatAhead, opening};
        const WarningPicture mawsOff = hear(2.5, WarningSet{}, defender, flat, {infrared}, heatBodies);
        expect(mawsOff.radar.empty() && mawsOff.approach.empty(), "maws: off ignores an infrared missile", "");

        WarningSet maws;
        maws.approachWarning = true;
        const WarningPicture heatPicture = hear(2.5, maws, defender, flat, {infrared}, heatBodies);
        const bool heatOk = heatPicture.radar.empty() && heatPicture.approach.size() == 1 &&
                            heatPicture.approach.front().kind == WarningKind::Approach &&
                            heatPicture.approach.front().source == heat.id && heatPicture.approach.front().time == 2.5 &&
                            std::abs(std::abs(heatPicture.approach.front().azimuthRad) - kPi) < 1.0e-3f &&
                            std::abs(heatPicture.approach.front().rangeM - 1500.0f) < 1.0e-2f;
        expect(heatOk, "maws: infrared missile is approach, not rwr", heatOk ? "" : "approach list");

        const float rearRange = 2000.0f;
        const float inCone = 0.4f;
        Emission seeker;
        seeker.source = EntityId{20};
        seeker.family = SensorFamily::Radar;
        seeker.missileSeeker = true;
        seeker.singleTargetTrack = true;
        seeker.position = defender.position + glm::vec3(std::sin(inCone) * rearRange, 0.0f, -std::cos(inCone) * rearRange);

        ClosingBody rear;
        rear.id = seeker.source;
        rear.position = seeker.position;
        rear.velocity = glm::normalize(defender.position - rear.position) * 350.0f;
        rear.infraredOnly = false;

        const float outside = 1.2f;
        ClosingBody wide = rear;
        wide.id = EntityId{21};
        wide.position = defender.position + glm::vec3(std::sin(outside) * rearRange, 0.0f, -std::cos(outside) * rearRange);
        wide.velocity = glm::normalize(defender.position - wide.position) * 350.0f;

        const WarningPicture seekerPicture = hear(9.0, maws, defender, flat, {seeker, infrared}, {rear, wide, heatAhead});
        // +X is the left wing for a +Z nose, so this source sits aft on the left.
        const float expectedAzimuth = std::atan2(-std::sin(inCone), -std::cos(inCone));
        const bool seekerOk = seekerPicture.radar.size() == 1 && seekerPicture.approach.size() == 1 &&
                              seekerPicture.radar.front().kind == WarningKind::MissileSeeker &&
                              seekerPicture.radar.front().source == seeker.source &&
                              std::abs(seekerPicture.radar.front().azimuthRad - expectedAzimuth) < 1.0e-3f &&
                              seekerPicture.approach.front().kind == WarningKind::Approach &&
                              seekerPicture.approach.front().source == rear.id &&
                              seekerPicture.approach.front().kind != seekerPicture.radar.front().kind;
        expect(seekerOk, "rwr: radar seeker and rear approach stay apart", seekerOk ? "" : "lists mixed");

        // The missile has been in the air since t = 0. Defense waits on the warning, then the delay.
        const double firstWarning = 4.0;
        const double delay = 1.5;
        const bool delayHolds = !defenseAllowed(-1.0, 20.0, delay) && !defenseAllowed(firstWarning, 0.0, delay) &&
                                !defenseAllowed(firstWarning, firstWarning, delay) &&
                                !defenseAllowed(firstWarning, firstWarning + delay - 0.001, delay) &&
                                defenseAllowed(firstWarning, firstWarning + delay, delay) &&
                                defenseAllowed(firstWarning, firstWarning + delay + 2.0, delay);
        expect(delayHolds, "defense: delay holds after the missile exists", "");

        ChaffDispenser empty;
        empty.remaining = 0;
        std::vector<ChaffRound> refusedRounds;
        const bool refused = !releaseChaff(empty, refusedRounds, EntityId{30}, 0.0, glm::vec3(1.0f), glm::vec3(2.0f),
                                            glm::vec3(0.0f, 1.0f, 0.0f));
        expect(refused && empty.remaining == 0 && refusedRounds.empty(), "chaff: an empty dispenser refuses", "");

        ChaffDispenser box;
        box.remaining = 2;
        box.rcsM2 = 20.0f;
        box.lifetimeS = 8.0;
        box.ejectSpeed = 30.0f;
        const glm::vec3 origin(9.0f, 8.0f, 7.0f);
        const glm::vec3 aircraftVelocity(5.0f, 6.0f, 7.0f);
        const glm::vec3 aircraftRight(0.0f, 4.0f, 0.0f);
        std::vector<ChaffRound> rounds;
        const bool spent = releaseChaff(box, rounds, EntityId{31}, 1.25, origin, aircraftVelocity, aircraftRight);
        const bool oneLeft = spent && box.remaining == 1 && rounds.size() == 1 && rounds[0].alive &&
                             rounds[0].id == EntityId{31} && rounds[0].birthTime == 1.25 && rounds[0].lifetimeS == 8.0 &&
                             rounds[0].rcsM2 == 20.0f && rounds[0].position == origin &&
                             glm::length(rounds[0].velocity - glm::vec3(5.0f, 36.0f, 7.0f)) < 1.0e-3f;
        const bool spentAgain = releaseChaff(box, rounds, EntityId{32}, 1.25, origin, aircraftVelocity, aircraftRight);
        const bool lastRefused = !releaseChaff(box, rounds, EntityId{33}, 1.25, origin, aircraftVelocity, aircraftRight);
        expect(oneLeft && spentAgain && box.remaining == 0 && rounds.size() == 2 && lastRefused,
               "chaff: release spends one and leaves the round", oneLeft ? "" : "velocity or count");

        ChaffDispenser lifeBox;
        lifeBox.remaining = 1;
        lifeBox.rcsM2 = 20.0f;
        lifeBox.lifetimeS = 8.0;
        lifeBox.ejectSpeed = 30.0f;
        std::vector<ChaffRound> cloud;
        const glm::vec3 cloudOrigin(5.0f, 0.0f, 1.0f);
        const glm::vec3 cloudVelocity(100.0f, 0.0f, 0.0f);
        const bool launched = releaseChaff(lifeBox, cloud, EntityId{40}, 0.0, cloudOrigin, cloudVelocity,
                                            glm::vec3(0.0f, 0.0f, 1.0f));
        stepChaff(cloud, 4.0, 4.0f);
        const glm::vec3 halfPosition = cloudOrigin + glm::vec3(100.0f, 0.0f, 30.0f) * 4.0f;
        const bool half = launched && cloud.size() == 1 && cloud[0].alive && std::abs(cloud[0].rcsM2 - 10.0f) < 1.0e-3f &&
                          glm::length(cloud[0].position - halfPosition) < 1.0e-2f;
        const std::size_t kept = cloud.size();
        stepChaff(cloud, 8.25, 4.25f);
        stepChaff(cloud, 9.0, 0.75f);
        const bool dead = kept == cloud.size() && !cloud[0].alive && cloud[0].rcsM2 == 0.0f && cloud[0].id == EntityId{40};
        expect(half && dead, "chaff: lifetime ends the round without erasing it", half ? "" : "mid-life rcs");

        // With drag the cloud stops in the air: from 250 m/s it is down to its
        // fall within a couple of seconds, and the path does not depend on the step.
        ChaffDispenser dragBox;
        dragBox.remaining = 2;
        dragBox.rcsM2 = 20.0f;
        dragBox.lifetimeS = 8.0;
        dragBox.ejectSpeed = 0.0f;
        dragBox.dragTimeS = 0.4;
        dragBox.fallSpeedMps = 1.5f;
        std::vector<ChaffRound> fine;
        std::vector<ChaffRound> coarse;
        const glm::vec3 jetVelocity(250.0f, 0.0f, 0.0f);
        releaseChaff(dragBox, fine, EntityId{50}, 0.0, glm::vec3(0.0f), jetVelocity, glm::vec3(0.0f, 0.0f, 1.0f));
        releaseChaff(dragBox, coarse, EntityId{51}, 0.0, glm::vec3(0.0f), jetVelocity, glm::vec3(0.0f, 0.0f, 1.0f));
        for (int step = 1; step <= 200; ++step)
        {
            stepChaff(fine, static_cast<double>(step) * 0.01, 0.01f);
        }
        stepChaff(coarse, 1.0, 1.0f);
        stepChaff(coarse, 2.0, 1.0f);
        const ChaffRound &settled = fine.front();
        // Travel tends to v0 * tau = 100 m along the release direction.
        const bool stopped = settled.alive && std::abs(settled.velocity.x) < 2.0f &&
                             std::abs(settled.velocity.y + 1.5f) < 0.1f && settled.position.x > 95.0f &&
                             settled.position.x < 100.0f &&
                             glm::length(settled.position - coarse.front().position) < 1.0e-2f;
        expect(stopped, "chaff: the air stops the cloud", stopped ? "" : "velocity or path");

        std::printf("%d/%d defense checks passed\n", count - failures, count);
        return failures;
    }
}
