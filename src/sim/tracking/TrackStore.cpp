#include "sim/tracking/TrackStore.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace missilesim::sim
{
    void TrackStore::setConfirmHits(int hits)
    {
        m_confirmHits = std::max(hits, 1);
    }

    void TrackStore::setCoastLimit(double seconds)
    {
        m_coastLimitS = seconds;
    }

    void TrackStore::setGateRadius(float meters)
    {
        m_gateRadiusM = std::max(meters, 0.0f);
    }

    void TrackStore::publish()
    {
        m_published.clear();
        m_published.reserve(m_records.size());
        for (const Record &record : m_records)
        {
            m_published.push_back(record.estimate);
        }
    }

    void TrackStore::onLook(double time, const std::vector<Observation> &observations)
    {
        onLook(time, observations, LookCoverage{});
    }

    void TrackStore::onLook(double time, const std::vector<Observation> &observations, const LookCoverage &coverage)
    {
        // The previous look already published these as Lost.
        m_records.erase(std::remove_if(m_records.begin(), m_records.end(),
                                       [](const Record &record) { return record.estimate.life == TrackLife::Lost; }),
                        m_records.end());

        struct Candidate
        {
            std::size_t observation = 0;
            std::size_t track = 0;
            float distance = 0.0f;
        };

        std::vector<bool> trackUsed(m_records.size(), false);
        for (Record &record : m_records)
        {
            if (record.estimate.life == TrackLife::Lost)
            {
                continue;
            }
            const float predictionTime = static_cast<float>(std::max(0.0, time - record.estimate.lastMeasurementTime));
            record.estimate.position = record.measuredPosition + record.estimate.velocity * predictionTime;
            record.estimate.time = time;
        }

        std::vector<Candidate> candidates;
        for (std::size_t observationIndex = 0; observationIndex < observations.size(); ++observationIndex)
        {
            for (std::size_t trackIndex = 0; trackIndex < m_records.size(); ++trackIndex)
            {
                if (m_records[trackIndex].estimate.life == TrackLife::Lost)
                {
                    continue;
                }
                const float distance =
                    glm::length(observations[observationIndex].position - m_records[trackIndex].estimate.position);
                if (distance <= m_gateRadiusM)
                {
                    candidates.push_back(Candidate{observationIndex, trackIndex, distance});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
            if (a.distance != b.distance)
            {
                return a.distance < b.distance;
            }
            if (a.observation != b.observation)
            {
                return a.observation < b.observation;
            }
            return a.track < b.track;
        });

        std::vector<bool> observationUsed(observations.size(), false);
        for (const Candidate &candidate : candidates)
        {
            if (observationUsed[candidate.observation] || trackUsed[candidate.track])
            {
                continue;
            }
            observationUsed[candidate.observation] = true;
            trackUsed[candidate.track] = true;

            Record &record = m_records[candidate.track];
            const glm::vec3 measured = observations[candidate.observation].position;
            const double interval = time - record.estimate.lastMeasurementTime;
            if (interval > 1.0e-6)
            {
                record.estimate.velocity = (measured - record.measuredPosition) / static_cast<float>(interval);
            }
            record.measuredPosition = measured;
            record.estimate.position = measured;
            record.estimate.lastMeasurementTime = time;
            record.estimate.time = time;
            record.estimate.hitCount += 1;
            record.estimate.missCount = 0;
            record.estimate.life =
                record.estimate.hitCount >= m_confirmHits ? TrackLife::Confirmed : TrackLife::Tentative;
        }

        for (std::size_t trackIndex = 0; trackIndex < m_records.size(); ++trackIndex)
        {
            Record &record = m_records[trackIndex];
            if (record.estimate.life == TrackLife::Lost || trackUsed[trackIndex])
            {
                continue;
            }
            record.estimate.time = time;
            const double sinceMeasurement = time - record.estimate.lastMeasurementTime;
            if (sinceMeasurement > m_coastLimitS)
            {
                record.estimate.life = TrackLife::Lost;
                continue;
            }
            const bool covered = !coverage.covers || coverage.covers(record.estimate.position);
            if (covered)
            {
                record.estimate.missCount += 1;
                record.estimate.life = TrackLife::Coasting;
            }
            else if (sinceMeasurement > coverage.revisitS)
            {
                record.estimate.life = TrackLife::Coasting;
            }
        }

        for (std::size_t observationIndex = 0; observationIndex < observations.size(); ++observationIndex)
        {
            if (observationUsed[observationIndex])
            {
                continue;
            }
            Record record;
            record.estimate.id = EntityId{m_nextId++};
            record.estimate.life = m_confirmHits <= 1 ? TrackLife::Confirmed : TrackLife::Tentative;
            record.estimate.time = time;
            record.estimate.lastMeasurementTime = time;
            record.estimate.position = observations[observationIndex].position;
            record.measuredPosition = observations[observationIndex].position;
            record.estimate.hitCount = 1;
            m_records.push_back(record);
        }

        publish();
    }
}
