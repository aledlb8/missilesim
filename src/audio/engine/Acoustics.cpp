#include "Acoustics.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace missilesim::audio
{
    namespace
    {
        // Doppler compression beyond these read rates is treated as the shock
        // itself (the energy arrives as the N-wave): the history read fades out
        // smoothly between the two rates instead of switching.
        constexpr float kDopplerFadeStartRate = 10.0f;
        constexpr float kDopplerFadeEndRate = 22.0f;
        constexpr float kGroundReflectionGain = 0.72f;
        constexpr float kGroundHighCutHz = 3200.0f;
        constexpr float kHeadShadowMinHz = 1500.0f;
        constexpr float kRearPinnaMinHz = 7000.0f;
        constexpr float kOpenCutoffHz = 23500.0f;
        constexpr int kMaxRootIterations = 320;
        constexpr double kMinRootStep = 4.0;
        constexpr double kBoomMinSpacingSeconds = 0.25;

        // Half-band decimation kernel (23 taps, Blackman window). Only the centre
        // tap and odd offsets are non-zero, so it is stored sparsely.
        constexpr int kHalfBandTaps = 23;
        constexpr int kHalfBandCentre = kHalfBandTaps / 2;
        constexpr int kHalfBandNonZero = 13;

        struct HalfBandKernel
        {
            std::array<int, kHalfBandNonZero> offset{};
            std::array<float, kHalfBandNonZero> coefficient{};

            HalfBandKernel()
            {
                std::array<double, kHalfBandTaps> h{};
                double sum = 0.0;
                for (int n = 0; n < kHalfBandTaps; ++n)
                {
                    const int k = n - kHalfBandCentre;
                    const double ideal = (k == 0) ? 0.5 : std::sin(0.5 * 3.14159265358979 * k) / (3.14159265358979 * k);
                    const double phase = 2.0 * 3.14159265358979 * n / (kHalfBandTaps - 1);
                    const double window = 0.42 - 0.5 * std::cos(phase) + 0.08 * std::cos(2.0 * phase);
                    h[n] = ideal * window;
                    sum += h[n];
                }

                int count = 0;
                for (int n = 0; n < kHalfBandTaps; ++n)
                {
                    const int k = n - kHalfBandCentre;
                    if (k == 0 || (k & 1) != 0)
                    {
                        offset[count] = n;
                        coefficient[count] = static_cast<float>(h[n] / sum);
                        ++count;
                    }
                }
            }
        };

        const HalfBandKernel &halfBand()
        {
            static const HalfBandKernel kernel;
            return kernel;
        }

        // Level-0 time represented by index 0 of level k is shifted by the
        // accumulated group delay of the decimation filters.
        constexpr std::array<double, kMipLevels> kMipDelay = {0.0, 11.0, 33.0, 77.0, 165.0};

        uint64_t nextPowerOfTwo(uint64_t v)
        {
            uint64_t p = 1;
            while (p < v)
            {
                p <<= 1;
            }
            return p;
        }

        // ISO 9613-1 pure-tone atmospheric absorption coefficient in dB/m.
        double isoAbsorptionDbPerMeter(double frequency, double temperatureK, double pressurePa, double humidityPercent)
        {
            constexpr double kT0 = 293.15;
            constexpr double kT01 = 273.16;
            constexpr double kPr = 101.325;
            const double pa = pressurePa / 1000.0;
            const double tRatio = temperatureK / kT0;
            const double c = -6.8346 * std::pow(kT01 / temperatureK, 1.261) + 4.6151;
            const double h = humidityPercent * std::pow(10.0, c) * (kPr / pa);
            const double frO = (pa / kPr) * (24.0 + 4.04e4 * h * (0.02 + h) / (0.391 + h));
            const double frN = (pa / kPr) * std::pow(tRatio, -0.5) *
                               (9.0 + 280.0 * h * std::exp(-4.170 * (std::pow(tRatio, -1.0 / 3.0) - 1.0)));
            const double f2 = frequency * frequency;
            const double classical = 1.84e-11 * (kPr / pa) * std::sqrt(tRatio);
            const double relaxation = std::pow(tRatio, -2.5) *
                                      (0.01275 * std::exp(-2239.1 / temperatureK) / (frO + f2 / frO) +
                                       0.1068 * std::exp(-3352.0 / temperatureK) / (frN + f2 / frN));
            return 8.686 * f2 * (classical + relaxation);
        }

        float boomShape(int n, int length, int rise)
        {
            if (n < rise)
            {
                return 0.5f * (1.0f - std::cos(kPi * static_cast<float>(n) / static_cast<float>(rise)));
            }
            n -= rise;
            if (n < length)
            {
                return 1.0f - 2.0f * static_cast<float>(n) / static_cast<float>(length);
            }
            n -= length;
            return -0.5f * (1.0f + std::cos(kPi * static_cast<float>(n) / static_cast<float>(rise)));
        }
    }

    // ------------------------------------------------------------------
    // Directivity / absorption
    // ------------------------------------------------------------------

    float DirectivityPattern::gainAt(float cosAngle) const
    {
        const float angle = std::acos(clampf(cosAngle, -1.0f, 1.0f)); // 0..pi
        const float position = angle / kPi * static_cast<float>(kPoints - 1);
        const int index = std::min(static_cast<int>(position), kPoints - 2);
        const float t = position - static_cast<float>(index);
        return dbToGain(lerpf(gainDb[index], gainDb[index + 1], t));
    }

    AbsorptionTable AbsorptionTable::compute(float temperatureK, float pressurePa, float relativeHumidityPercent)
    {
        AbsorptionTable table;
        const double logMin = std::log(static_cast<double>(kMinDistance));
        const double logMax = std::log(static_cast<double>(kMaxDistance));
        for (int i = 0; i < kPoints; ++i)
        {
            const double distance = std::exp(logMin + (logMax - logMin) * i / (kPoints - 1));
            auto lossAt = [&](double f)
            {
                return isoAbsorptionDbPerMeter(f, temperatureK, pressurePa, relativeHumidityPercent) * distance;
            };

            double f3;
            if (lossAt(22000.0) < 3.0)
            {
                f3 = 22000.0;
            }
            else
            {
                double lo = std::log(10.0);
                double hi = std::log(22000.0);
                for (int iteration = 0; iteration < 40; ++iteration)
                {
                    const double mid = 0.5 * (lo + hi);
                    (lossAt(std::exp(mid)) < 3.0 ? lo : hi) = mid;
                }
                f3 = std::exp(0.5 * (lo + hi));
            }
            table.cutoffHz[i] = static_cast<float>(f3);
        }
        return table;
    }

    float AbsorptionTable::cutoffAt(float distanceMeters) const
    {
        const float logMin = std::log(kMinDistance);
        const float logMax = std::log(kMaxDistance);
        const float d = clampf(distanceMeters, kMinDistance, kMaxDistance);
        const float position = (std::log(d) - logMin) / (logMax - logMin) * static_cast<float>(kPoints - 1);
        const int index = std::min(static_cast<int>(position), kPoints - 2);
        const float t = position - static_cast<float>(index);
        // Interpolate in log-frequency for a perceptually even sweep.
        return std::exp(lerpf(std::log(cutoffHz[index]), std::log(cutoffHz[index + 1]), t));
    }

    // ------------------------------------------------------------------
    // Emitter
    // ------------------------------------------------------------------

    AcousticEmitter::AcousticEmitter(const EmitterSpec &spec,
                                     std::shared_ptr<SoundSource> source,
                                     std::shared_ptr<EmitterControl> control,
                                     uint64_t seed)
        : m_spec(spec),
          m_source(std::move(source)),
          m_control(std::move(control)),
          m_random(seed)
    {
        m_spec.lobeCount = std::clamp(m_spec.lobeCount, 1, kMaxLobes);
        halfBand(); // make sure the shared kernel is built off the audio thread

        const uint64_t wanted = static_cast<uint64_t>(std::max(m_spec.historySeconds, 0.5f) * kSampleRateF) + 4096;
        m_capacity = nextPowerOfTwo(wanted);
        for (int level = 0; level < kMipLevels; ++level)
        {
            const uint64_t size = m_capacity >> level;
            m_mask[level] = size - 1;
            for (int lobe = 0; lobe < m_spec.lobeCount; ++lobe)
            {
                m_history[level][lobe].assign(static_cast<size_t>(size), 0.0f);
            }
        }

        m_track.resize(static_cast<size_t>(nextPowerOfTwo(2 * m_capacity / kBlockSize + 4)));

        for (auto &ear : m_paths)
        {
            for (auto &path : ear)
            {
                path.scintillation.setTimeConstant(0.55f);
            }
        }

        const Kinematics &initial = m_control->m_kinematics.read();
        m_position = initial.position;
        m_velocity = initial.velocity;
        m_axis = glm::length(initial.axis) > 1.0e-4f ? glm::normalize(initial.axis) : glm::vec3(0.0f, 0.0f, -1.0f);
    }

    AcousticEmitter::~AcousticEmitter() = default;

    void AcousticEmitter::processBlock(uint64_t blockStart,
                                       const ListenerFrame &listener,
                                       const MediumFrame &medium,
                                       const BlockOutput &output)
    {
        if (!m_started)
        {
            m_started = true;
            m_startSample = blockStart;
            for (int level = 0; level < kMipLevels; ++level)
            {
                m_written[level] = blockStart >> level;
            }
            m_track[0] = {m_position, m_axis};
            m_trackCount = 1;
        }

        const uint64_t blockEnd = blockStart + kBlockSize;
        updateKinematics(blockEnd);
        renderEmission(blockStart, medium);
        propagate(blockStart, listener, medium, output);

        if (m_sourceFinished && !m_retireReady)
        {
            const double metersPerSample = medium.speedOfSound / static_cast<double>(kSampleRate);
            const double settle = static_cast<double>(m_finishSample) + 4096.0;
            bool allHeard = true;
            for (int ear = 0; ear < kEarCount && allHeard; ++ear)
            {
                const glm::vec3 finishPosition = sampleAt(settle).position;
                const double travelled = metersPerSample * (static_cast<double>(blockEnd) - settle);
                glm::vec3 image = listener.ears[ear];
                image.y = 2.0f * medium.groundLevel - image.y;
                const double farthest = std::max(glm::length(listener.ears[ear] - finishPosition),
                                                 glm::length(image - finishPosition));
                allHeard = travelled > farthest;
            }
            const bool outOfHistory = (blockEnd - m_finishSample) > m_capacity;
            m_retireReady = allHeard || outOfHistory;
        }
    }

    void AcousticEmitter::updateKinematics(uint64_t blockEnd)
    {
        const Kinematics &target = m_control->m_kinematics.read();
        const float blockSeconds = static_cast<float>(kBlockSize) / kSampleRateF;

        // Extrapolate the last game-thread sample to this block boundary, then
        // steer the smoothed state toward it. Game frames arrive at 60-240 Hz;
        // audio needs a C1-continuous path or every correction becomes a click.
        const double ahead = (static_cast<double>(blockEnd) - static_cast<double>(target.stampSample)) / kSampleRate;
        const float aheadSeconds = static_cast<float>(std::clamp(ahead, 0.0, 0.25));
        const glm::vec3 goal = target.position + target.velocity * aheadSeconds;

        glm::vec3 predicted = m_position + m_velocity * blockSeconds;
        const glm::vec3 error = goal - predicted;
        const float teleportThreshold = 20.0f + glm::length(target.velocity) * 0.15f;
        if (glm::length(error) > teleportThreshold)
        {
            predicted = goal;
        }
        else
        {
            predicted += error * smoothingCoefficient(0.045f, static_cast<float>(kBlockSize));
        }

        m_velocity += (target.velocity - m_velocity) * smoothingCoefficient(0.02f, static_cast<float>(kBlockSize));
        m_position = predicted;
        m_maxSpeed = std::max(m_maxSpeed, glm::length(target.velocity));

        if (glm::length(target.axis) > 1.0e-4f)
        {
            const glm::vec3 blended = glm::mix(m_axis, glm::normalize(target.axis), smoothingCoefficient(0.03f, static_cast<float>(kBlockSize)));
            if (glm::length(blended) > 1.0e-4f)
            {
                m_axis = glm::normalize(blended);
            }
        }

        m_track[m_trackCount & (m_track.size() - 1)] = {m_position, m_axis};
        ++m_trackCount;
    }

    void AcousticEmitter::renderEmission(uint64_t blockStart, const MediumFrame &medium)
    {
        std::array<float *, kMaxLobes> lobes{};
        for (int lobe = 0; lobe < kMaxLobes; ++lobe)
        {
            m_scratch[lobe].fill(0.0f);
            lobes[lobe] = m_scratch[lobe].data();
        }

        if (!m_sourceFinished)
        {
            if (m_control->isReleased() && !m_source->isFinished())
            {
                m_source->release();
            }

            SourceContext context;
            context.position = m_position;
            context.velocity = m_velocity / std::max(medium.timeScale, 1.0e-3f);
            context.axis = m_axis;
            context.speedOfSound = medium.physicalSpeedOfSound;
            context.timeSeconds = static_cast<double>(blockStart) / kSampleRate;
            m_source->render(context, lobes.data(), m_spec.lobeCount, kBlockSize);

            const double fadeSamples = static_cast<double>(m_spec.fadeInSeconds) * kSampleRate;
            const double age = static_cast<double>(blockStart - m_startSample);
            if (age < fadeSamples)
            {
                for (int i = 0; i < kBlockSize; ++i)
                {
                    const float x = static_cast<float>(std::min((age + i) / fadeSamples, 1.0));
                    const float gain = x * x * (3.0f - 2.0f * x);
                    for (int lobe = 0; lobe < m_spec.lobeCount; ++lobe)
                    {
                        lobes[lobe][i] *= gain;
                    }
                }
            }

            if (m_source->isFinished())
            {
                m_sourceFinished = true;
                m_finishSample = blockStart + kBlockSize;
            }
        }

        writeHistory(lobes, blockStart);
    }

    void AcousticEmitter::writeHistory(const std::array<float *, kMaxLobes> &lobes, uint64_t blockStart)
    {
        const HalfBandKernel &kernel = halfBand();
        const int lobeCount = m_spec.lobeCount;

        for (int i = 0; i < kBlockSize; ++i)
        {
            const uint64_t j0 = blockStart + static_cast<uint64_t>(i);
            for (int lobe = 0; lobe < lobeCount; ++lobe)
            {
                m_history[0][lobe][static_cast<size_t>(j0 & m_mask[0])] = lobes[lobe][i];
            }
            m_written[0] = j0 + 1;

            // Cascade: every even sample of level k produces one sample of level k+1.
            uint64_t j = j0;
            for (int level = 0; level + 1 < kMipLevels; ++level)
            {
                if ((j & 1u) != 0)
                {
                    break;
                }
                const uint64_t outIndex = j >> 1;
                const uint64_t mask = m_mask[level];
                for (int lobe = 0; lobe < lobeCount; ++lobe)
                {
                    const float *src = m_history[level][lobe].data();
                    float acc = 0.0f;
                    for (int tap = 0; tap < kHalfBandNonZero; ++tap)
                    {
                        acc += kernel.coefficient[tap] * src[static_cast<size_t>((j - static_cast<uint64_t>(kernel.offset[tap])) & mask)];
                    }
                    m_history[level + 1][lobe][static_cast<size_t>(outIndex & m_mask[level + 1])] = acc;
                }
                m_written[level + 1] = outIndex + 1;
                j = outIndex;
            }
        }
    }

    AcousticEmitter::KinematicSample AcousticEmitter::sampleAt(double tau) const
    {
        const uint64_t trackMask = m_track.size() - 1;
        const uint64_t newest = m_trackCount - 1;
        const uint64_t oldest = (m_trackCount > m_track.size()) ? (m_trackCount - m_track.size()) : 0;

        const double u = (tau - static_cast<double>(m_startSample)) / kBlockSize;
        if (u <= static_cast<double>(oldest))
        {
            return m_track[oldest & trackMask];
        }
        if (u >= static_cast<double>(newest))
        {
            return m_track[newest & trackMask];
        }
        const uint64_t n = static_cast<uint64_t>(u);
        const float t = static_cast<float>(u - static_cast<double>(n));
        const KinematicSample &a = m_track[n & trackMask];
        const KinematicSample &b = m_track[(n + 1) & trackMask];
        return {glm::mix(a.position, b.position, t), glm::mix(a.axis, b.axis, t)};
    }

    glm::vec3 AcousticEmitter::velocityAt(double tau) const
    {
        if (m_trackCount < 2)
        {
            return m_velocity;
        }
        const uint64_t trackMask = m_track.size() - 1;
        const uint64_t newest = m_trackCount - 1;
        const uint64_t oldest = (m_trackCount > m_track.size()) ? (m_trackCount - m_track.size()) : 0;
        const double u = (tau - static_cast<double>(m_startSample)) / kBlockSize;
        uint64_t n = (u <= static_cast<double>(oldest)) ? oldest : static_cast<uint64_t>(u);
        n = std::min(n, newest - 1);
        const float scale = kSampleRateF / static_cast<float>(kBlockSize);
        return (m_track[(n + 1) & trackMask].position - m_track[n & trackMask].position) * scale;
    }

    int AcousticEmitter::solveRoots(const glm::vec3 &receiver, double t, float speedOfSound, double *roots) const
    {
        // Find every emission time tau with c(t - tau) = |receiver - source(tau)|.
        // F is Lipschitz with constant (c + vmax) per second, so stepping back by
        // |F| / L can never skip a root ("sphere tracing" along the time axis).
        const double metersPerSample = static_cast<double>(speedOfSound) / kSampleRate;
        const double lipschitz = (static_cast<double>(speedOfSound) + m_maxSpeed + 1.0) / kSampleRate;
        const uint64_t oldestTrack = (m_trackCount > m_track.size()) ? (m_trackCount - m_track.size()) : 0;
        const double oldestTime = static_cast<double>(m_startSample) + static_cast<double>(oldestTrack) * kBlockSize;
        const double historyStart = t - static_cast<double>(m_capacity) + 512.0;
        const double tauMin = std::max({static_cast<double>(m_startSample), oldestTime, historyStart});
        const bool supersonicHistory = m_maxSpeed > 0.95f * speedOfSound;

        auto evaluate = [&](double tau)
        {
            return metersPerSample * (t - tau) - static_cast<double>(glm::length(receiver - sampleAt(tau).position));
        };

        int count = 0;
        double tau = t;
        double f = evaluate(tau);
        for (int iteration = 0; iteration < kMaxRootIterations && tau > tauMin; ++iteration)
        {
            const double step = std::max(std::abs(f) / lipschitz, kMinRootStep);
            const double tauNext = std::max(tau - step, tauMin);
            const double fNext = evaluate(tauNext);

            if ((f < 0.0) != (fNext < 0.0))
            {
                // Illinois-modified regula falsi inside the bracket.
                double a = tauNext, fa = fNext, b = tau, fb = f;
                double root = 0.5 * (a + b);
                int side = 0;
                for (int refine = 0; refine < 24; ++refine)
                {
                    root = (a * fb - b * fa) / (fb - fa);
                    const double fr = evaluate(root);
                    if ((fr < 0.0) == (fb < 0.0))
                    {
                        b = root;
                        fb = fr;
                        if (side == -1)
                        {
                            fa *= 0.5;
                        }
                        side = -1;
                    }
                    else
                    {
                        a = root;
                        fa = fr;
                        if (side == 1)
                        {
                            fb *= 0.5;
                        }
                        side = 1;
                    }
                    if (b - a < 0.02)
                    {
                        break;
                    }
                }
                roots[count++] = root;
                if (!supersonicHistory || count == kMaxBranches)
                {
                    break;
                }
            }

            tau = tauNext;
            f = fNext;
        }
        return count;
    }

    void AcousticEmitter::configureBranch(Branch &branch,
                                          double tau,
                                          int ear,
                                          int path,
                                          const glm::vec3 &receiver,
                                          const ListenerFrame &listener,
                                          const MediumFrame &medium,
                                          float scintillationGain,
                                          bool born)
    {
        const float c = medium.speedOfSound;
        const KinematicSample state = sampleAt(tau);
        const glm::vec3 sourceVelocity = velocityAt(tau);

        const glm::vec3 toReceiver = receiver - state.position;
        const float distance = std::max(glm::length(toReceiver), 0.05f);
        const glm::vec3 u = toReceiver / distance;

        glm::vec3 receiverVelocity = listener.velocity;
        if (path == 1)
        {
            receiverVelocity.y = -receiverVelocity.y;
        }

        // Analytic Doppler read-rate d(tau)/dt and convective amplification.
        const float closingSource = glm::dot(u, sourceVelocity);
        const float closingListener = glm::dot(u, receiverVelocity);
        // Negative inside the Mach cone of a receding supersonic source: that
        // branch hears earlier emissions in reverse order.
        float sourceTerm = c - closingSource;
        if (std::abs(sourceTerm) < 0.02f * c)
        {
            sourceTerm = std::copysign(0.02f * c, sourceTerm);
        }
        const float analyticRate = (c - closingListener) / sourceTerm;
        const float convective = clampf(c / std::abs(sourceTerm), 0.35f, 3.0f);

        if (born)
        {
            branch.active = true;
            branch.dying = false;
            branch.rate = analyticRate;
            branch.tauStart = tau - static_cast<double>(analyticRate) * kBlockSize;
            branch.tauEnd = tau;
            branch.weightStart.fill(0.0f);
            branch.absorptionA.reset();
            branch.absorptionB.reset();
            branch.shading.reset();
            branch.boomSample = -1;
        }
        else
        {
            branch.tauStart = branch.tauEnd;
            branch.tauEnd = tau;
            branch.rate = static_cast<float>((branch.tauEnd - branch.tauStart) / kBlockSize);
            branch.weightStart = branch.weightEnd;
        }

        const float absRate = std::abs(branch.rate);
        branch.mipLevel = (absRate > 1.1f) ? clampf(std::log2(absRate / 1.1f), 0.0f, static_cast<float>(kMipLevels - 1)) : 0.0f;

        // Sonic boom: a pair of roots appearing out of nothing means the Mach cone
        // just swept over this ear. Whitham far-field N-wave scaling.
        Path &owner = m_paths[ear][path];
        const float sourceSpeed = glm::length(sourceVelocity);
        const float mach = sourceSpeed / c;
        const bool firstArrival = tau < static_cast<double>(m_startSample) + 4.0 * kBlockSize;
        if (born && !firstArrival && mach > 1.02f && m_spec.bodyLengthMeters > 0.0f &&
            std::abs(tau - owner.lastBoomTau) > kBoomMinSpacingSeconds * kSampleRate)
        {
            owner.lastBoomTau = tau;
            const float machTerm = std::max(mach * mach - 1.0f, 0.05f);
            const float length = m_spec.bodyLengthMeters;
            const float diameter = std::max(m_spec.bodyDiameterMeters, 0.01f);
            const float r = std::max(distance, 1.0f);
            const float overpressure = 0.18f * medium.ambientPressurePa * std::pow(machTerm, 0.125f) * diameter *
                                       std::pow(length, -0.25f) * std::pow(r, -0.75f);
            // Evaluated with the physical speed of sound: the crack keeps its
            // natural length whatever the simulation speed.
            const float duration = 1.82f * mach * std::pow(r, 0.25f) * diameter /
                                   (medium.physicalSpeedOfSound * std::pow(machTerm, 0.375f) * std::pow(length, 0.25f));
            const float riseSeconds = 3.0e-5f + r * 1.5e-6f;
            branch.boomSample = 0;
            branch.boomLength = std::max(static_cast<int>(clampf(duration, 1.0e-4f, 0.08f) * kSampleRateF), 4);
            branch.boomRise = std::max(static_cast<int>(clampf(riseSeconds, 2.0e-5f, 0.004f) * kSampleRateF), 2);
            branch.boomPeak = overpressure * (path == 1 ? m_spec.groundReflection * kGroundReflectionGain : 1.0f);
        }

        // Ear-side shading: head shadow for the far ear, pinna darkening behind.
        glm::vec3 arrival = -u;
        if (path == 1)
        {
            arrival.y = -arrival.y;
        }
        const glm::vec3 earNormal = (ear == 0) ? -listener.right : listener.right;
        const float facing = glm::dot(arrival, earNormal);
        const float frontness = glm::dot(arrival, listener.forward);
        float shadeCutoff = kOpenCutoffHz;
        if (facing < 0.0f)
        {
            shadeCutoff = kOpenCutoffHz * std::pow(kHeadShadowMinHz / kOpenCutoffHz, -facing);
        }
        if (frontness < 0.0f)
        {
            shadeCutoff = std::min(shadeCutoff, lerpf(kOpenCutoffHz, kRearPinnaMinHz, -frontness));
        }
        branch.shading.setCutoff(shadeCutoff);
        const float interauralLevel = dbToGain(2.5f * facing);

        // Atmospheric absorption modelled as two matched one-poles whose combined
        // -3 dB point equals the ISO 9613-1 -3 dB frequency for this distance.
        const float absorptionHz = medium.absorption ? medium.absorption->cutoffAt(distance) : 20000.0f;
        const float poleHz = std::min(absorptionHz / 0.6423f, kOpenCutoffHz);
        branch.absorptionA.setCutoff(poleHz);
        branch.absorptionB.setCutoff(path == 1 ? std::min(poleHz, kGroundHighCutHz) : poleHz);

        const float pathGain = (path == 1) ? m_spec.groundReflection * kGroundReflectionGain : 1.0f;
        const float spreading = 1.0f / std::max(distance, 1.0f);
        const float common = spreading * convective * pathGain * scintillationGain * interauralLevel;
        const float cosAngle = glm::dot(state.axis, u);
        const float dopplerFade = 1.0f - clampf(std::log(std::max(absRate, 1.0f) / kDopplerFadeStartRate) /
                                                    std::log(kDopplerFadeEndRate / kDopplerFadeStartRate),
                                                0.0f,
                                                1.0f);
        for (int lobe = 0; lobe < kMaxLobes; ++lobe)
        {
            branch.weightEnd[lobe] = (lobe < m_spec.lobeCount)
                                         ? common * dopplerFade * m_spec.lobes[lobe].gainAt(cosAngle)
                                         : 0.0f;
        }

        // The very first arrival reads history that is silent before the emitter
        // existed, so no fade is needed and the onset transient stays intact.
        if (born && firstArrival)
        {
            branch.weightStart = branch.weightEnd;
        }
    }

    float AcousticEmitter::readLevel(int lobe, int level, double tau) const
    {
        const double scale = 1.0 / static_cast<double>(1u << level);
        double position = (tau + kMipDelay[level]) * scale;
        const double newest = static_cast<double>(m_written[level]) - 3.0;
        if (position > newest)
        {
            position = newest;
        }
        const double floorPosition = std::floor(position);
        const float t = static_cast<float>(position - floorPosition);
        const int64_t index = static_cast<int64_t>(floorPosition);
        const uint64_t mask = m_mask[level];
        const float *data = m_history[level][lobe].data();
        const float xm1 = data[static_cast<size_t>(static_cast<uint64_t>(index - 1) & mask)];
        const float x0 = data[static_cast<size_t>(static_cast<uint64_t>(index) & mask)];
        const float x1 = data[static_cast<size_t>(static_cast<uint64_t>(index + 1) & mask)];
        const float x2 = data[static_cast<size_t>(static_cast<uint64_t>(index + 2) & mask)];
        return hermite4(xm1, x0, x1, x2, t);
    }

    float AcousticEmitter::readHistory(int lobe, float level, double tau) const
    {
        const int lower = static_cast<int>(level);
        const float blend = level - static_cast<float>(lower);
        const float a = readLevel(lobe, lower, tau);
        if (blend < 0.01f || lower + 1 >= kMipLevels)
        {
            return a;
        }
        return lerpf(a, readLevel(lobe, lower + 1, tau), blend);
    }

    void AcousticEmitter::renderBranch(Branch &branch, float *destination, float *send, float sendGain)
    {
        const double dTau = (branch.tauEnd - branch.tauStart) / kBlockSize;
        const float inverseBlock = 1.0f / static_cast<float>(kBlockSize);
        const int lobeCount = m_spec.lobeCount;

        for (int i = 0; i < kBlockSize; ++i)
        {
            const double tau = branch.tauStart + dTau * static_cast<double>(i);
            const float ramp = static_cast<float>(i) * inverseBlock;
            float x = 0.0f;
            for (int lobe = 0; lobe < lobeCount; ++lobe)
            {
                const float weight = lerpf(branch.weightStart[lobe], branch.weightEnd[lobe], ramp);
                if (weight != 0.0f)
                {
                    x += weight * readHistory(lobe, branch.mipLevel, tau);
                }
            }

            if (branch.boomSample >= 0)
            {
                x += branch.boomPeak * boomShape(branch.boomSample, branch.boomLength, branch.boomRise);
                if (++branch.boomSample >= branch.boomLength + 2 * branch.boomRise)
                {
                    branch.boomSample = -1;
                }
            }

            x = branch.absorptionB.lowpass(branch.absorptionA.lowpass(x));
            if (send)
            {
                send[i] += x * sendGain;
            }
            destination[i] += branch.shading.lowpass(x);
        }
    }

    void AcousticEmitter::propagate(uint64_t blockStart,
                                    const ListenerFrame &listener,
                                    const MediumFrame &medium,
                                    const BlockOutput &output)
    {
        const double t = static_cast<double>(blockStart + kBlockSize);
        const float blockSeconds = static_cast<float>(kBlockSize) / kSampleRateF;

        // Turbulence along a path affects both ears (17 cm apart) identically.
        std::array<float, kPathCount> scintillationWalk{};
        for (int pathIndex = 0; pathIndex < kPathCount; ++pathIndex)
        {
            scintillationWalk[pathIndex] = m_paths[0][pathIndex].scintillation.advance(m_random, blockSeconds);
        }

        for (int ear = 0; ear < kEarCount; ++ear)
        {
            float *destination = (ear == 0) ? output.left : output.right;

            for (int pathIndex = 0; pathIndex < kPathCount; ++pathIndex)
            {
                Path &path = m_paths[ear][pathIndex];

                // Branches that finished fading last block are recycled now, unless
                // an N-wave is still sweeping through them.
                for (Branch &branch : path.branches)
                {
                    if (branch.dying && branch.boomSample < 0)
                    {
                        branch.active = false;
                        branch.dying = false;
                    }
                }

                glm::vec3 receiver = listener.ears[ear];
                const bool groundPath = pathIndex == 1;
                const bool pathEnabled = !groundPath ||
                                         (medium.groundPresent && m_spec.groundReflection > 0.0f &&
                                          receiver.y > medium.groundLevel + 0.05f);
                if (groundPath)
                {
                    receiver.y = 2.0f * medium.groundLevel - receiver.y;
                }

                std::array<double, kMaxBranches> roots{};
                int rootCount = 0;
                if (pathEnabled)
                {
                    std::array<double, kMaxBranches> candidates{};
                    const int candidateCount = solveRoots(receiver, t, medium.speedOfSound, candidates.data());
                    for (int i = 0; i < candidateCount; ++i)
                    {
                        if (groundPath && sampleAt(candidates[i]).position.y < medium.groundLevel)
                        {
                            continue; // the reflecting ray would have to pass through the ground
                        }
                        roots[rootCount++] = candidates[i];
                    }
                }

                // Turbulent scintillation grows with path length.
                const float walk = scintillationWalk[pathIndex];
                float referenceDistance = 0.0f;
                if (rootCount > 0)
                {
                    referenceDistance = static_cast<float>(medium.speedOfSound * (t - roots[0]) / kSampleRate);
                }
                const float sigmaDb = clampf(2.2f * std::log10(1.0f + referenceDistance / 250.0f), 0.0f, 6.0f);
                const float scintillationGain = dbToGain(sigmaDb * walk);

                // Continue existing branches onto the nearest predicted root.
                std::array<bool, kMaxBranches> rootTaken{};
                for (Branch &branch : path.branches)
                {
                    if (!branch.active)
                    {
                        continue;
                    }
                    const double predicted = branch.tauEnd + static_cast<double>(branch.rate) * kBlockSize;
                    const double tolerance = 24.0 + 0.5 * std::abs(static_cast<double>(branch.rate)) * kBlockSize;
                    int best = -1;
                    double bestError = tolerance;
                    for (int r = 0; r < rootCount; ++r)
                    {
                        const double error = std::abs(roots[r] - predicted);
                        if (!rootTaken[r] && error < bestError)
                        {
                            best = r;
                            bestError = error;
                        }
                    }

                    if (best >= 0)
                    {
                        rootTaken[best] = true;
                        configureBranch(branch, roots[best], ear, pathIndex, receiver, listener, medium, scintillationGain, false);
                    }
                    else
                    {
                        branch.dying = true;
                        branch.tauStart = branch.tauEnd;
                        branch.tauEnd = predicted;
                        branch.weightStart = branch.weightEnd;
                        branch.weightEnd.fill(0.0f);
                    }
                }

                for (int r = 0; r < rootCount; ++r)
                {
                    if (rootTaken[r])
                    {
                        continue;
                    }
                    for (Branch &branch : path.branches)
                    {
                        if (!branch.active)
                        {
                            configureBranch(branch, roots[r], ear, pathIndex, receiver, listener, medium, scintillationGain, true);
                            break;
                        }
                    }
                }

                // Terrain-echo send. Open ground has no diffuse field, but echoes
                // from terrain at ranges comparable to the source distance decay
                // more slowly than the direct sound, so distant sources get
                // relatively wetter: direct-to-echo ratio ~ +15 dB at 50 m, ~0 dB
                // at 3 km.
                const bool feedsReverb = !groundPath && output.reverbSend != nullptr;
                const float sendGain = feedsReverb
                                           ? 0.25f * m_spec.reverbSend * std::pow(clampf(referenceDistance, 1.0f, 5000.0f), 0.42f)
                                           : 0.0f;

                for (Branch &branch : path.branches)
                {
                    if (branch.active)
                    {
                        renderBranch(branch, destination, feedsReverb ? output.reverbSend : nullptr, sendGain);
                    }
                }
            }
        }
    }
}
