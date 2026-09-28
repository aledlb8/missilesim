#include "Voices.h"

#include <algorithm>
#include <cmath>

namespace missilesim::audio::synth
{
    namespace
    {
        constexpr float kSampleSeconds = 1.0f / kSampleRateF;

        float *lobeBuffer(float *const *lobes, int lobeCount, int index)
        {
            return lobes[index < lobeCount ? index : 0];
        }

        // Linear per-sample ramp across a block, for click-free gain changes.
        struct Ramp
        {
            Ramp(float from, float to, int frames)
                : start(from), step((to - from) / static_cast<float>(std::max(frames, 1)))
            {
            }
            float at(int i) const { return start + step * static_cast<float>(i); }
            float start;
            float step;
        };

        // Symmetric N-wave of total length `duration` with smooth shock fronts.
        float nWave(float t, float duration)
        {
            if (t < 0.0f || t >= duration)
            {
                return 0.0f;
            }
            const float x = t / duration;
            constexpr float kFront = 0.04f;
            if (x < kFront)
            {
                return 0.5f - 0.5f * std::cos(kPi * x / kFront);
            }
            if (x > 1.0f - kFront)
            {
                return -(0.5f - 0.5f * std::cos(kPi * (1.0f - x) / kFront));
            }
            return 1.0f - 2.0f * (x - kFront) / (1.0f - 2.0f * kFront);
        }

        // Zero-net-impulse pressure pulse: a positive half-sine of length
        // `positive` followed by a longer, shallower rarefaction.
        float pressurePulse(float t, float positive)
        {
            constexpr float kNegativeStretch = 1.8f;
            if (t < 0.0f)
            {
                return 0.0f;
            }
            if (t < positive)
            {
                return std::sin(kPi * t / positive);
            }
            const float negative = positive * kNegativeStretch;
            const float u = t - positive;
            if (u < negative)
            {
                return -std::sin(kPi * u / negative) / kNegativeStretch;
            }
            return 0.0f;
        }

        // Ideal-gas speed of sound is supplied per block; guard silly values.
        float safeSpeedOfSound(const SourceContext &context)
        {
            return std::max(context.speedOfSound, 200.0f);
        }
    }

    // ==================================================================
    // Rocket motor
    // ==================================================================

    RocketMotorVoice::RocketMotorVoice(uint64_t seed) : m_random(seed)
    {
        m_mixingFlicker.setDepth(0.32f, 0.18f);
        m_crackleFlicker.setDepth(0.45f, 0.25f);
        m_flowFlicker.setDepth(0.25f, 0.2f);
        m_rumble.set(22.0f, 150.0f);
        m_hiss.set(1800.0f, 8000.0f);
    }

    EmitterSpec RocketMotorVoice::spec(float bodyLength, float bodyDiameter)
    {
        EmitterSpec spec;
        spec.lobeCount = 3;
        // Large-scale turbulence radiates in a cone ~30-40 deg off the exhaust axis.
        spec.lobes[0] = powerNormalisedPattern({-21.0f, -20.0f, -18.0f, -15.0f, -11.0f, -6.0f, -1.0f, 2.0f, -2.0f});
        // Crackle is even more strongly beamed into that cone.
        spec.lobes[1] = powerNormalisedPattern({-28.0f, -27.0f, -25.0f, -21.0f, -15.0f, -8.0f, 0.0f, 1.0f, -4.0f});
        // Fine-scale turbulence, combustion and flow noise: nearly omnidirectional.
        spec.lobes[2] = powerNormalisedPattern({-2.0f, -2.0f, -1.5f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f});
        spec.historySeconds = 21.0f;
        spec.bodyLengthMeters = bodyLength;
        spec.bodyDiameterMeters = bodyDiameter;
        return spec;
    }

    void RocketMotorVoice::render(const SourceContext &context, float *const *lobes, int lobeCount, int frames)
    {
        const RocketMotorParams &params = m_params.read();
        const float dt = static_cast<float>(frames) * kSampleSeconds;
        const float c = safeSpeedOfSound(context);
        const float rho = clampf(params.airDensity, 0.01f, 1.6f);
        const float impedance = rho * c;
        const float speed = glm::length(context.velocity);
        const float nominalUe = clampf(params.exhaustVelocity, 1400.0f, 3000.0f);

        // --- Combustion state machine ---------------------------------
        const float commanded = std::max(params.thrustNewtons, 0.0f);
        const bool burningNow = commanded > 1.0f;
        if (burningNow && !m_burning)
        {
            m_burning = true;
            m_tailSeconds = -1.0f;
            m_ignitionSeconds = 0.0f;
            const float power = jetAcousticEfficiency(nominalUe / c) * 0.5f * commanded * nominalUe;
            m_ignitionAmplitude = 1.6f * pressureAtOneMeter(power, impedance);
        }
        else if (!burningNow && m_burning)
        {
            m_burning = false;
            m_tailSeconds = 0.0f;
            m_tailThrust = m_thrust;
        }

        if (m_burning)
        {
            // Chamber pressurises in ~30 ms with a brief overshoot, then tracks
            // commanded thrust (boost -> sustain transitions included).
            float overshoot = 1.0f;
            if (m_ignitionSeconds >= 0.0f && m_ignitionSeconds < 0.6f)
            {
                overshoot += 0.35f * std::exp(-m_ignitionSeconds / 0.09f);
            }
            const float target = commanded * overshoot;
            const float timeConstant = (target > m_thrust) ? 0.03f : 0.08f;
            m_thrust += (target - m_thrust) * smoothingCoefficient(timeConstant, static_cast<float>(frames));
        }
        else if (m_tailSeconds >= 0.0f)
        {
            // Tail-off: pressure decays while the sliver burns out unevenly.
            m_tailSeconds += dt;
            m_chuffTimer -= dt;
            if (m_chuffTimer <= 0.0f)
            {
                m_chuff = 0.3f + 0.7f * m_random.unit();
                m_chuffTimer = 0.035f + 0.08f * m_random.unit();
            }
            m_chuff *= std::exp(-dt / 0.03f);
            const float onset = std::min(m_tailSeconds / 0.1f, 1.0f);
            m_thrust = m_tailThrust * std::exp(-m_tailSeconds / 0.22f) * (1.0f - 0.7f * m_chuff * onset);
            if (m_tailSeconds > 1.5f)
            {
                m_tailSeconds = -1.0f;
                m_thrust = 0.0f;
            }
        }
        else
        {
            m_thrust = 0.0f;
        }

        // --- Jet noise levels -------------------------------------------
        const float pressureFraction = (m_tailSeconds >= 0.0f && m_tailThrust > 1.0f)
                                           ? clampf(m_thrust / m_tailThrust, 0.0f, 1.0f)
                                           : 1.0f;
        const float ue = nominalUe * (0.55f + 0.45f * pressureFraction);
        // Forward flight lowers the shear between plume and air.
        const float ur = std::max(ue - 0.8f * speed, 0.3f * ue);
        const float jetPower = jetAcousticEfficiency(ur / c) * 0.5f * std::max(m_thrust, 0.0f) * ur;
        const float p0 = pressureAtOneMeter(jetPower, impedance);

        const float nozzle = clampf(params.nozzleDiameter, 0.04f, 1.0f);
        // Rocket plumes peak at very low Strouhal numbers (St ~ 0.03, NASA SP-8072).
        const float peakHz = clampf(0.03f * ur / nozzle, 60.0f, 4000.0f);
        m_large.set(peakHz * 0.35f, peakHz * 2.4f);
        m_fine.set(peakHz * 0.9f, std::min(peakHz * 9.0f, 20000.0f));
        m_crackle.set(300.0f + 1600.0f * clampf(m_thrust / 40000.0f, 0.0f, 1.5f), 30.0e-6f, 0.5e-3f);

        // --- Airframe flow noise (turbulent boundary layer + base wake) ----
        const float bodyDiameter = clampf(params.bodyDiameter, 0.05f, 1.0f);
        const float frontalArea = 0.25f * kPi * bodyDiameter * bodyDiameter;
        const float flightMach = speed / c;
        const float dynamicPressure = 0.5f * rho * speed * speed;
        // Radiation efficiency grows like M^2 up to the sound barrier and then
        // saturates: a coasting round passing close by is a loud tearing
        // whoosh, but at Mach 2 the motor still dominates.
        const float flowPower = 2.5e-5f * dynamicPressure * speed * frontalArea * std::min(flightMach * flightMach, 1.0f);
        const float flowPressure = pressureAtOneMeter(flowPower, impedance);
        const float flowPeak = clampf(0.2f * speed / bodyDiameter, 150.0f, 4000.0f);
        m_flow.set(flowPeak * 0.15f, std::min(flowPeak * 3.0f, 12000.0f));

        // --- Block gains --------------------------------------------------
        if (m_released)
        {
            m_releaseGain = std::max(m_releaseGain - dt / 0.03f, 0.0f);
        }
        const float mixing = m_mixingFlicker.advance(m_random, frames);
        const float crackling = m_crackleFlicker.advance(m_random, frames);
        const float flowing = m_flowFlicker.advance(m_random, frames);
        const float master = m_releaseGain;

        // Power budget of the plume noise: large-scale mixing carries the
        // body of the roar, crackle the tearing edge, combustion rumble the
        // chest-thumping low end; fine-scale mixing and shock hiss add air.
        const Ramp large(m_gainLarge, p0 * std::sqrt(0.52f) * mixing * master, frames);
        const Ramp fine(m_gainFine, p0 * std::sqrt(0.2f) * std::sqrt(mixing) * master, frames);
        const Ramp crackle(m_gainCrackle, p0 * std::sqrt(0.11f) * crackling * master, frames);
        const Ramp rumble(m_gainRumble, p0 * std::sqrt(0.13f) * mixing * master, frames);
        const Ramp hiss(m_gainHiss, p0 * std::sqrt(0.04f) * master, frames);
        const Ramp flow(m_gainFlow, flowPressure * flowing * master, frames);
        m_gainLarge = large.at(frames);
        m_gainFine = fine.at(frames);
        m_gainCrackle = crackle.at(frames);
        m_gainRumble = rumble.at(frames);
        m_gainHiss = hiss.at(frames);
        m_gainFlow = flow.at(frames);

        const bool transientActive = m_ignitionSeconds >= 0.0f && m_ignitionSeconds < 0.25f;
        const float loudest = std::max({large.start, m_gainLarge, flow.start, m_gainFlow, crackle.start, m_gainCrackle});
        if (loudest < 1.0e-6f && !transientActive)
        {
            if (m_ignitionSeconds >= 0.0f)
            {
                m_ignitionSeconds += dt;
            }
            return;
        }

        float *aft = lobeBuffer(lobes, lobeCount, 0);
        float *cone = lobeBuffer(lobes, lobeCount, 1);
        float *broad = lobeBuffer(lobes, lobeCount, 2);
        for (int i = 0; i < frames; ++i)
        {
            aft[i] += m_large.process(m_random) * large.at(i);
            cone[i] += m_crackle.process(m_random) * crackle.at(i);

            float omni = m_fine.process(m_random) * fine.at(i) +
                         m_rumble.process(m_random) * rumble.at(i) +
                         m_hiss.process(m_random) * hiss.at(i) +
                         m_flow.process(m_random) * flow.at(i);

            if (transientActive)
            {
                // Igniter pop, then the ignition-overpressure pulse as the plume
                // shoves the surrounding air out of the way.
                const float t = m_ignitionSeconds + static_cast<float>(i) * kSampleSeconds;
                omni += m_ignitionAmplitude * (0.35f * nWave(t, 0.0012f) + pressurePulse(t - 0.004f, 0.018f)) * master;
            }
            broad[i] += omni;
        }

        if (m_ignitionSeconds >= 0.0f)
        {
            m_ignitionSeconds += dt;
        }
    }

    // ==================================================================
    // Turbofan
    // ==================================================================

    namespace
    {
        constexpr float kIdleSpool = 0.64f;
        constexpr float kFanMaxRpm = 8000.0f;
        constexpr float kFanBlades = 32.0f;
        constexpr float kCompressorMaxRpm = 14500.0f;
        constexpr float kCompressorBlades = 38.0f;
        constexpr float kTurbineBlades = 62.0f;
        constexpr float kMilThrust = 76000.0f;
        constexpr float kAfterburnerExtraThrust = 50000.0f;
        constexpr float kAfterburnerExhaustVelocity = 1150.0f;

        // Dry military-power jet noise power (static), the reference for fan noise.
        float militaryJetPower()
        {
            const float ue = 680.0f;
            return jetAcousticEfficiency(ue / 340.0f) * 0.5f * kMilThrust * ue;
        }
    }

    TurbofanVoice::TurbofanVoice(uint64_t seed, float initialThrottle) : m_random(seed)
    {
        const float demand = clampf(initialThrottle / 0.92f, 0.0f, 1.0f);
        m_n1 = kIdleSpool + (1.0f - kIdleSpool) * std::pow(demand, 0.8f);

        m_jetFlicker.setDepth(0.22f, 0.15f);
        m_fanFlicker.setDepth(0.45f, 0.25f);
        m_augmentorFlicker.setDepth(0.5f, 0.3f);
        m_flowFlicker.setDepth(0.25f, 0.2f);
        m_core.set(180.0f, 900.0f);
        m_augmentorRumble.set(25.0f, 140.0f);
        m_toneWander.setTimeConstant(0.8f);

        // Buzz-saw: every fan blade's leading-edge shock is slightly different,
        // so the pattern repeats once per revolution -> dense shaft harmonics
        // with an irregular (but fixed per engine) amplitude spectrum.
        std::vector<float> amplitudes(48);
        std::vector<float> phases(48);
        for (size_t h = 0; h < amplitudes.size(); ++h)
        {
            const float order = static_cast<float>(h + 1) / 12.0f;
            const float envelope = std::pow(order, 1.5f) / (1.0f + order * order * order);
            amplitudes[h] = envelope * std::exp(0.8f * m_random.gaussian());
            phases[h] = kTwoPi * m_random.unit();
        }
        m_buzzSaw.build(amplitudes, phases);
        m_bladePass2.setPhase(0.25f);
    }

    EmitterSpec TurbofanVoice::spec()
    {
        EmitterSpec spec;
        spec.lobeCount = 3;
        // Exhaust: jet mixing noise peaks ~35-40 deg off the tail.
        spec.lobes[0] = powerNormalisedPattern({-20.0f, -19.0f, -17.0f, -14.0f, -10.0f, -5.0f, -1.0f, 2.0f, -1.5f});
        // Inlet: fan tones radiate forward, blocked by the fuselage behind.
        spec.lobes[1] = powerNormalisedPattern({1.0f, 1.5f, 1.0f, -1.0f, -4.0f, -8.0f, -12.0f, -15.0f, -18.0f});
        spec.lobes[2] = powerNormalisedPattern({-1.5f, -1.0f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, -0.5f, -1.0f});
        spec.historySeconds = 10.5f;
        spec.bodyLengthMeters = 15.0f;
        spec.bodyDiameterMeters = 1.2f;
        spec.fadeInSeconds = 0.6f;
        return spec;
    }

    void TurbofanVoice::render(const SourceContext &context, float *const *lobes, int lobeCount, int frames)
    {
        const TurbofanParams &params = m_params.read();
        const float dt = static_cast<float>(frames) * kSampleSeconds;
        const float c = safeSpeedOfSound(context);
        const float rho = clampf(params.airDensity, 0.01f, 1.6f);
        const float impedance = rho * c;
        const float speed = glm::length(context.velocity);

        // --- Spool and afterburner dynamics -------------------------------
        const float throttle = clampf(params.throttle, 0.0f, 1.0f);
        const float demand = clampf(throttle / 0.92f, 0.0f, 1.0f);
        const float n1Target = kIdleSpool + (1.0f - kIdleSpool) * std::pow(demand, 0.8f);
        const float spoolTime = (n1Target > m_n1) ? 2.0f : 1.4f;
        m_n1 += (n1Target - m_n1) * smoothingCoefficient(spoolTime, static_cast<float>(frames));

        if (!m_afterburnerSelected && throttle >= 0.93f && m_n1 > 0.95f)
        {
            m_afterburnerSelected = true;
            m_lightOffDelay = 0.28f + 0.12f * m_random.unit(); // fuel manifold fill
        }
        else if (m_afterburnerSelected && throttle < 0.88f)
        {
            m_afterburnerSelected = false;
            m_lightOffDelay = -1.0f;
        }

        if (m_afterburnerSelected)
        {
            if (m_lightOffDelay > 0.0f)
            {
                m_lightOffDelay -= dt;
                if (m_lightOffDelay <= 0.0f)
                {
                    m_lightOffDelay = 0.0f;
                    m_lightOffSeconds = 0.0f;
                    const float power = jetAcousticEfficiency(kAfterburnerExhaustVelocity / c) * 0.5f *
                                        (kMilThrust + kAfterburnerExtraThrust) * kAfterburnerExhaustVelocity;
                    m_lightOffAmplitude = 0.9f * pressureAtOneMeter(power, impedance);
                }
            }
            else
            {
                m_afterburner = std::min(1.0f, m_afterburner + dt / 0.35f);
            }
        }
        else
        {
            m_afterburner = std::max(0.0f, m_afterburner - dt / 0.2f);
        }

        // --- Exhaust jet ---------------------------------------------------
        const float spool = clampf((m_n1 - kIdleSpool) / (1.0f - kIdleSpool), 0.0f, 1.0f);
        const float ab = m_afterburner;
        const float dryThrust = 3500.0f + (kMilThrust - 3500.0f) * std::pow(spool, 1.4f);
        const float dryUe = 300.0f + 380.0f * std::pow(spool, 1.1f);
        const float thrust = dryThrust + kAfterburnerExtraThrust * ab;
        const float ue = lerpf(dryUe, kAfterburnerExhaustVelocity, ab);
        const float ur = std::max(ue - 0.7f * speed, 0.35f * ue);
        const float jetPower = jetAcousticEfficiency(ur / c) * 0.5f * thrust * ur;
        const float p0 = pressureAtOneMeter(jetPower, impedance);

        const float nozzle = 0.82f + 0.2f * ab;
        const float strouhal = lerpf(0.18f, 0.1f, ab);
        const float peakHz = clampf(strouhal * ur / nozzle, 40.0f, 2000.0f);
        m_large.set(peakHz * 0.35f, peakHz * 2.6f);
        m_fine.set(peakHz * 0.8f, std::min(peakHz * 16.0f, 20000.0f));
        m_crackle.set(200.0f + 1400.0f * ab, 30.0e-6f, 0.5e-3f);
        const float crackleShare = 0.02f + 0.16f * ab;

        // --- Inlet / turbomachinery -----------------------------------------
        const float wander = 1.0f + 0.003f * m_toneWander.advance(m_random, dt);
        const float shaftHz = m_n1 * kFanMaxRpm / 60.0f * wander;
        const float bladePassHz = kFanBlades * shaftHz;
        const float compressorHz = (0.8f + 0.2f * spool) * kCompressorMaxRpm / 60.0f * kCompressorBlades * wander;
        const float turbineHz = kTurbineBlades * shaftHz;
        const float fanPower = 0.12f * militaryJetPower() * std::pow(m_n1, 6.0f);
        const float fanPressure = pressureAtOneMeter(fanPower, impedance);
        // Relative fan-tip Mach ~1.45 at 100 % N1: supersonic tips -> buzz-saw.
        const float buzz = clampf((m_n1 * 1.45f - 1.02f) / 0.35f, 0.0f, 1.0f);
        m_fanBroadband.set(bladePassHz * 0.25f, std::min(bladePassHz * 2.5f, 20000.0f));

        // --- Airframe flow --------------------------------------------------
        const float dynamicPressure = 0.5f * rho * speed * speed;
        const float flightMach = speed / c;
        const float flowPower = 2.0e-7f * dynamicPressure * speed * 3.0f * std::min(flightMach * flightMach, 4.0f);
        const float flowPressure = pressureAtOneMeter(flowPower, impedance);
        m_flow.set(std::max(0.4f * speed, 40.0f), std::clamp(10.0f * speed, 400.0f, 16000.0f));

        // --- Block gains -----------------------------------------------------
        if (m_released)
        {
            m_releaseGain = std::max(m_releaseGain - dt / 0.06f, 0.0f);
        }
        const float master = m_releaseGain;
        const float jetMod = m_jetFlicker.advance(m_random, frames);
        const float fanMod = m_fanFlicker.advance(m_random, frames);
        const float augmentorMod = m_augmentorFlicker.advance(m_random, frames);
        const float flowMod = m_flowFlicker.advance(m_random, frames);
        const float mixingShare = 1.0f - crackleShare - 0.1f * ab;

        const Ramp large(m_gainLarge, p0 * std::sqrt(0.62f * mixingShare) * jetMod * master, frames);
        const Ramp fine(m_gainFine, p0 * std::sqrt(0.38f * mixingShare) * std::sqrt(jetMod) * master, frames);
        const Ramp crackle(m_gainCrackle, p0 * std::sqrt(crackleShare) * jetMod * master, frames);
        const Ramp rumble(m_gainRumble, p0 * std::sqrt(0.1f * ab) * augmentorMod * master, frames);
        const Ramp core(m_gainCore, pressureAtOneMeter(0.02f * militaryJetPower() * m_n1 * m_n1, impedance) * master, frames);
        const Ramp fanBroadband(m_gainFanBroadband, fanPressure * std::sqrt(0.45f) * master, frames);
        const Ramp bpf(m_gainBpf, fanPressure * std::sqrt(0.25f * (1.0f - 0.5f * buzz)) * fanMod * master, frames);
        const Ramp bpf2(m_gainBpf2, fanPressure * std::sqrt(0.08f) * fanMod * master, frames);
        const Ramp buzzSaw(m_gainBuzz, fanPressure * std::sqrt(0.3f * buzz) * master, frames);
        const Ramp compressor(m_gainCompressor, fanPressure * std::sqrt(0.08f) * std::sqrt(fanMod) * master, frames);
        const Ramp turbine(m_gainTurbine, fanPressure * std::sqrt(0.05f) * master, frames);
        const Ramp flow(m_gainFlow, flowPressure * flowMod * master, frames);
        m_gainLarge = large.at(frames);
        m_gainFine = fine.at(frames);
        m_gainCrackle = crackle.at(frames);
        m_gainRumble = rumble.at(frames);
        m_gainCore = core.at(frames);
        m_gainFanBroadband = fanBroadband.at(frames);
        m_gainBpf = bpf.at(frames);
        m_gainBpf2 = bpf2.at(frames);
        m_gainBuzz = buzzSaw.at(frames);
        m_gainCompressor = compressor.at(frames);
        m_gainTurbine = turbine.at(frames);
        m_gainFlow = flow.at(frames);

        const bool lightOffActive = m_lightOffSeconds >= 0.0f && m_lightOffSeconds < 0.12f;
        const bool augmentorAudible = rumble.start > 0.0f || m_gainRumble > 0.0f;
        const bool crackleAudible = crackle.start > 0.0f || m_gainCrackle > 0.0f;

        float *exhaust = lobeBuffer(lobes, lobeCount, 0);
        float *inlet = lobeBuffer(lobes, lobeCount, 1);
        float *broad = lobeBuffer(lobes, lobeCount, 2);
        for (int i = 0; i < frames; ++i)
        {
            float aft = m_large.process(m_random) * large.at(i) +
                        m_turbine.process(turbineHz) * turbine.at(i);
            if (crackleAudible)
            {
                aft += m_crackle.process(m_random) * crackle.at(i);
            }
            exhaust[i] += aft;

            inlet[i] += m_fanBroadband.process(m_random) * fanBroadband.at(i) +
                        m_bladePass.process(bladePassHz) * bpf.at(i) * 1.41421356f +
                        m_bladePass2.process(2.0f * bladePassHz) * bpf2.at(i) * 1.41421356f +
                        m_buzzSaw.process(shaftHz) * buzzSaw.at(i) +
                        m_compressor.process(compressorHz) * compressor.at(i) * 1.41421356f;

            float omni = m_fine.process(m_random) * fine.at(i) +
                         m_core.process(m_random) * core.at(i) +
                         m_flow.process(m_random) * flow.at(i);
            if (augmentorAudible)
            {
                omni += m_augmentorRumble.process(m_random) * rumble.at(i);
            }
            if (lightOffActive)
            {
                // Augmentor light-off: the whole plume re-pressurises at once.
                const float t = m_lightOffSeconds + static_cast<float>(i) * kSampleSeconds;
                omni += m_lightOffAmplitude * pressurePulse(t, 0.025f) * master;
            }
            broad[i] += omni;
        }

        if (m_lightOffSeconds >= 0.0f)
        {
            m_lightOffSeconds += dt;
        }
    }

    // ==================================================================
    // Explosion
    // ==================================================================

    namespace
    {
        // Far-field reference: beyond ~40 m a small warhead's blast has decayed
        // into a (still strong) acoustic wave that spreads spherically.
        constexpr float kBlastReferenceDistance = 40.0f;
    }

    ExplosionVoice::ExplosionVoice(uint64_t seed, float chargeKg, float ambientPressurePa) : m_random(seed)
    {
        const float charge = clampf(chargeKg, 0.1f, 1000.0f);
        const float cubeRoot = std::cbrt(charge);
        const BlastParameters blast = kinneyGrahamBlast(charge, kBlastReferenceDistance, ambientPressurePa);
        m_blastPeak = blast.peakOverpressurePa * kBlastReferenceDistance;
        m_blastDuration = std::max(blast.positiveDurationSeconds, 0.001f);

        m_fireballRms = 0.05f * m_blastPeak;
        m_fireballDecay = 0.25f * cubeRoot * (0.85f + 0.3f * m_random.unit());
        m_pulseAmplitude = 0.25f * m_blastPeak;
        m_pulseHz = 1.0f / (0.018f * cubeRoot);
        m_fragmentRms = 0.04f * m_blastPeak;

        m_fireball.set(40.0f, 900.0f);
        m_sizzle.set(3000.0f, 12000.0f);
        m_fireballFlicker.setDepth(0.5f, 0.3f);

        const float seconds = std::max(3.0f, 6.0f * m_fireballDecay);
        m_lengthSamples = static_cast<int64_t>(seconds * kSampleRateF);
    }

    EmitterSpec ExplosionVoice::spec()
    {
        EmitterSpec spec;
        spec.lobeCount = 1;
        spec.lobes[0] = DirectivityPattern::omni();
        spec.historySeconds = 21.0f;
        spec.reverbSend = 1.4f;
        return spec;
    }

    void ExplosionVoice::render(const SourceContext &context, float *const *lobes, int lobeCount, int frames)
    {
        (void)context;
        (void)lobeCount;
        float *out = lobes[0];
        const float blockStart = static_cast<float>(m_sample) * kSampleSeconds;

        // Fragment shocklets thin out as the fragments fly out of earshot.
        const float fragmentRate = 4000.0f * std::exp(-blockStart / 0.08f);
        const bool fragmentsActive = fragmentRate > 5.0f;
        if (fragmentsActive)
        {
            m_fragments.set(fragmentRate, 15.0e-6f, 0.08e-3f);
        }
        const float previousFlicker = m_flicker;
        m_flicker = m_fireballFlicker.advance(m_random, frames);

        for (int i = 0; i < frames && m_sample < m_lengthSamples; ++i, ++m_sample)
        {
            const float t = static_cast<float>(m_sample) * kSampleSeconds;
            float p = 0.0f;

            // Friedlander blast wave (b = 1 -> zero net impulse), 50 us shock rise.
            const float x = t / m_blastDuration;
            if (x < 10.0f)
            {
                const float rise = std::min(t / 50.0e-6f, 1.0f);
                p += m_blastPeak * (1.0f - x) * std::exp(-x) * rise;
            }

            // Fireball growth: a slow, deep pressure heave under the crack.
            p += m_pulseAmplitude * pressurePulse(t - 0.002f, 0.5f / m_pulseHz);

            // Afterburning detonation products: a rolling, flickering roar.
            const float attack = 1.0f - std::exp(-t / 0.008f);
            const float envelope = attack * std::exp(-t / m_fireballDecay);
            const float flicker = lerpf(previousFlicker, m_flicker, static_cast<float>(i) / static_cast<float>(frames));
            p += m_fireball.process(m_random) * m_fireballRms * envelope * flicker;

            // High-frequency sizzle of the shattered casing.
            p += m_sizzle.process(m_random) * m_fragmentRms * 0.5f * attack * std::exp(-t / 0.12f);

            if (fragmentsActive)
            {
                p += m_fragments.process(m_random) * m_fragmentRms * std::exp(-t / 0.15f);
            }
            out[i] += p;
        }
    }

    // ==================================================================
    // Flare
    // ==================================================================

    namespace
    {
        constexpr float kFlareBurnRms = 30.0f;      // Pa at 1 m at full heat
        constexpr float kFlarePopPeak = 350.0f;     // impulse cartridge, Pa at 1 m
        constexpr float kFlareThumpPeak = 150.0f;   // ejection piston thump
        constexpr float kFlareIgnitionDelay = 0.03f;
    }

    FlareVoice::FlareVoice(uint64_t seed) : m_random(seed)
    {
        m_fizz.set(900.0f, 9000.0f);
        m_roar.set(150.0f, 900.0f);
        m_sputter.set(500.0f, 20.0e-6f, 0.15e-3f);
        m_burnFlicker.setDepth(0.35f, 0.2f);
    }

    EmitterSpec FlareVoice::spec()
    {
        EmitterSpec spec;
        spec.lobeCount = 1;
        spec.lobes[0] = DirectivityPattern::omni();
        spec.historySeconds = 10.5f;
        spec.reverbSend = 0.8f;
        return spec;
    }

    void FlareVoice::render(const SourceContext &context, float *const *lobes, int lobeCount, int frames)
    {
        (void)context;
        (void)lobeCount;
        const FlareParams &params = m_params.read();
        const float dt = static_cast<float>(frames) * kSampleSeconds;
        m_heat += (clampf(params.heat, 0.0f, 1.0f) - m_heat) * smoothingCoefficient(0.05f, static_cast<float>(frames));
        if (m_released)
        {
            m_releaseGain = std::max(m_releaseGain - dt / 0.15f, 0.0f);
        }

        const float blockStart = static_cast<float>(m_sample) * kSampleSeconds;
        // Pellet ignition flares up with a brief overshoot, then burns with heat.
        const float sinceIgnition = blockStart - kFlareIgnitionDelay;
        float burn = 0.0f;
        if (sinceIgnition > 0.0f)
        {
            burn = (1.0f - std::exp(-sinceIgnition / 0.06f)) * (1.0f + 0.6f * std::exp(-sinceIgnition / 0.15f));
        }
        const float level = kFlareBurnRms * burn * std::pow(m_heat, 0.7f) * m_releaseGain *
                            m_burnFlicker.advance(m_random, frames);

        const Ramp fizz(m_gainFizz, level * std::sqrt(0.55f), frames);
        const Ramp roar(m_gainRoar, level * std::sqrt(0.3f), frames);
        const Ramp sputter(m_gainSputter, level * std::sqrt(0.15f), frames);
        m_gainFizz = fizz.at(frames);
        m_gainRoar = roar.at(frames);
        m_gainSputter = sputter.at(frames);

        float *out = lobes[0];
        for (int i = 0; i < frames; ++i, ++m_sample)
        {
            const float t = static_cast<float>(m_sample) * kSampleSeconds;
            float p = m_fizz.process(m_random) * fizz.at(i) +
                      m_roar.process(m_random) * roar.at(i) +
                      m_sputter.process(m_random) * sputter.at(i);
            if (t < 0.05f)
            {
                p += kFlarePopPeak * nWave(t, 0.0012f) + kFlareThumpPeak * pressurePulse(t - 0.0005f, 0.006f);
            }
            out[i] += p;
        }
    }

    // ==================================================================
    // Cold-launch ejection
    // ==================================================================

    namespace
    {
        constexpr float kEjectSlamPeak = 1800.0f; // Pa at 1 m
        constexpr float kEjectHissRms = 120.0f;
        constexpr std::array<float, 4> kModeBaseHz = {95.0f, 210.0f, 470.0f, 1150.0f};
        constexpr std::array<float, 4> kModeDecaySeconds = {0.35f, 0.25f, 0.15f, 0.08f};
        constexpr std::array<float, 4> kModePeak = {260.0f, 180.0f, 110.0f, 60.0f};
    }

    LaunchEjectVoice::LaunchEjectVoice(uint64_t seed) : m_random(seed)
    {
        m_hiss.set(1500.0f, 9000.0f);
        for (size_t k = 0; k < m_modeHz.size(); ++k)
        {
            m_modeHz[k] = kModeBaseHz[k] * (0.92f + 0.16f * m_random.unit());
        }
        m_lengthSamples = static_cast<int64_t>(2.0f * kSampleRateF);
    }

    EmitterSpec LaunchEjectVoice::spec()
    {
        EmitterSpec spec;
        spec.lobeCount = 1;
        spec.lobes[0] = DirectivityPattern::omni();
        spec.historySeconds = 10.5f;
        spec.reverbSend = 1.2f;
        return spec;
    }

    void LaunchEjectVoice::render(const SourceContext &context, float *const *lobes, int lobeCount, int frames)
    {
        (void)context;
        (void)lobeCount;
        float *out = lobes[0];
        for (int i = 0; i < frames && m_sample < m_lengthSamples; ++i, ++m_sample)
        {
            const float t = static_cast<float>(m_sample) * kSampleSeconds;
            // Gas-generator slam, then a secondary knock as the round clears the rails.
            float p = kEjectSlamPeak * pressurePulse(t, 0.03f) +
                      0.35f * kEjectSlamPeak * pressurePulse(t - 0.045f, 0.012f);
            for (size_t k = 0; k < m_modeHz.size(); ++k)
            {
                const float ring = std::exp(-t / kModeDecaySeconds[k]);
                p += kModePeak[k] * ring * std::sin(kTwoPi * m_modeHz[k] * t);
            }
            const float hissEnvelope = (1.0f - std::exp(-t / 0.005f)) * std::exp(-t / 0.35f);
            p += m_hiss.process(m_random) * kEjectHissRms * hissEnvelope;
            out[i] += p;
        }
    }
}
