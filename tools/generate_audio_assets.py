#!/usr/bin/env python3

from __future__ import annotations

import math
import random
import struct
import wave
from pathlib import Path


TAU = math.tau
SAMPLE_RATE = 48_000
ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "assets" / "audio"


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize(samples: list[float], peak: float = 0.94) -> list[float]:
    if not samples:
        return samples

    current_peak = max(abs(sample) for sample in samples)
    if current_peak <= 1e-6:
        return samples

    scale = peak / current_peak
    return [clamp(sample * scale, -1.0, 1.0) for sample in samples]


def apply_fade(samples: list[float], fade_in: int, fade_out: int) -> list[float]:
    sample_count = len(samples)
    for index in range(min(fade_in, sample_count)):
        samples[index] *= index / max(fade_in - 1, 1)

    for index in range(min(fade_out, sample_count)):
        fade = 1.0 - (index / max(fade_out - 1, 1))
        samples[sample_count - 1 - index] *= fade

    return samples


def make_loop_seamless(samples: list[float], crossfade: int = 2048) -> list[float]:
    if not samples:
        return samples

    blend_count = min(crossfade, len(samples) // 3)
    if blend_count <= 1:
        return samples

    for index in range(blend_count):
        alpha = index / (blend_count - 1)
        tail_index = len(samples) - blend_count + index
        samples[tail_index] = (samples[tail_index] * (1.0 - alpha)) + (samples[index] * alpha)

    samples[-1] = samples[0]
    return samples


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge1 <= edge0:
        return 1.0 if value >= edge1 else 0.0

    t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - (2.0 * t))


def triangle_wave(phase: float) -> float:
    wrapped = (phase / TAU) % 1.0
    return 1.0 - (4.0 * abs(wrapped - 0.5))


def soft_square(phase: float, drive: float = 2.8) -> float:
    return math.tanh(math.sin(phase) * drive)


def pulse_envelope(position: float,
                   start: float,
                   duration: float,
                   attack_fraction: float = 0.12,
                   release_fraction: float = 0.32) -> float:
    if duration <= 0.0 or position < start or position > start + duration:
        return 0.0

    local = (position - start) / duration
    attack = smoothstep(0.0, attack_fraction, local)
    release = 1.0 - smoothstep(1.0 - release_fraction, 1.0, local)
    return attack * release


def write_wav(path: Path, samples: list[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(SAMPLE_RATE)
        frames = bytearray()
        for sample in samples:
            frames.extend(struct.pack("<h", int(clamp(sample, -1.0, 1.0) * 32767.0)))
        output.writeframes(frames)


class LoopNoise:
    def __init__(self, seed: int, anchor_count: int = 64) -> None:
        rng = random.Random(seed)
        self.anchor_count = max(anchor_count, 8)
        self.values = [rng.uniform(-1.0, 1.0) for _ in range(self.anchor_count)]

    def sample(self, phase: float) -> float:
        wrapped = phase % 1.0
        position = wrapped * self.anchor_count
        index = int(position)
        fraction = position - index

        p0 = self.values[(index - 1) % self.anchor_count]
        p1 = self.values[index % self.anchor_count]
        p2 = self.values[(index + 1) % self.anchor_count]
        p3 = self.values[(index + 2) % self.anchor_count]

        fraction_sq = fraction * fraction
        fraction_cu = fraction_sq * fraction
        return 0.5 * (
            (2.0 * p1)
            + (-p0 + p2) * fraction
            + ((2.0 * p0) - (5.0 * p1) + (4.0 * p2) - p3) * fraction_sq
            + (-p0 + (3.0 * p1) - (3.0 * p2) + p3) * fraction_cu
        )


def render_fire_launch(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    pressure_noise = LoopNoise(11, 96)
    roar_noise = LoopNoise(17, 128)
    hiss_noise = LoopNoise(23, 192)
    samples: list[float] = []

    for index in range(sample_count):
        t = index / SAMPLE_RATE
        x = index / sample_count
        ignition_overpressure = math.exp(-96.0 * t)
        motor_attack = 1.0 - math.exp(-34.0 * t)
        motor_body_decay = math.exp(-2.05 * x)
        turbulence_decay = math.exp(-4.1 * x)
        exhaust_pulse = 0.80 + (0.20 * math.sin(TAU * (2.6 * x + 0.08)))

        low_body = (
            math.sin(TAU * (34.0 * t * (1.0 - 0.04 * x))) * 0.34
            + math.sin(TAU * (58.0 * t * (1.0 - 0.07 * x)) + 0.27) * 0.22
        )
        motor_roar = (pressure_noise.sample(x * 0.55) * 0.58) + (roar_noise.sample(x * 1.9) * 0.44)
        bark = soft_square(TAU * (108.0 * t * (1.0 - 0.14 * x)), 2.0) * math.exp(-6.2 * x) * 0.18
        ignition_crack = (
            pressure_noise.sample(x * 15.0) * 0.34
            + roar_noise.sample(x * 21.0) * 0.16
            + math.sin(TAU * 236.0 * t) * 0.18
        ) * ignition_overpressure
        hiss = ((hiss_noise.sample(x * 10.0) * 0.30) + (roar_noise.sample(x * 6.0) * 0.12)) * motor_attack * turbulence_decay

        sample = ((motor_roar * 0.84) + low_body + bark) * motor_attack * motor_body_decay * exhaust_pulse
        sample += ignition_crack
        sample += hiss
        samples.append(math.tanh(sample * 1.62))

    return apply_fade(normalize(samples, peak=0.96), 96, 9_600)


def render_explosion(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    blast_noise = LoopNoise(31, 96)
    debris_noise = LoopNoise(43, 128)
    hiss_noise = LoopNoise(59, 192)
    samples: list[float] = []

    for index in range(sample_count):
        t = index / SAMPLE_RATE
        x = index / sample_count
        shock = math.exp(-58.0 * t)
        body = math.exp(-2.2 * x)
        tail = math.exp(-1.0 * x)
        pressure = 0.88 + 0.12 * math.sin(TAU * (1.7 * x))

        low_boom = math.sin(TAU * (36.0 * t * (1.0 - 0.22 * x)))
        crackle = debris_noise.sample(x * 7.0) * math.exp(-5.5 * x)
        roar = (blast_noise.sample(x * 0.9) * 0.76) + (debris_noise.sample(x * 2.1) * 0.32)
        hiss = hiss_noise.sample(x * 11.0) * math.exp(-8.0 * x)

        sample = ((roar * 0.84) + (low_boom * 0.26)) * body * pressure
        sample += ((shock * 0.46) * blast_noise.sample(x * 12.0))
        sample += crackle * 0.20 * tail
        sample += hiss * 0.18
        samples.append(math.tanh(sample * 1.55))

    return apply_fade(normalize(samples), 64, 18_000)


def render_jet_afterburner_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    low_noise = LoopNoise(71, 64)
    mid_noise = LoopNoise(79, 96)
    high_noise = LoopNoise(83, 160)
    samples: list[float] = []

    for index in range(sample_count):
        x = index / sample_count
        flutter = 0.80 + (0.20 * math.sin(TAU * (3.0 * x + 0.07)))
        rumble = (math.sin(TAU * (8.0 * x + 0.03 * math.sin(TAU * 2.0 * x))) * 0.52)
        harmonics = (math.sin(TAU * 16.0 * x) * 0.24) + (math.sin(TAU * 24.0 * x) * 0.14)
        roar = (low_noise.sample(x) * 0.56) + (mid_noise.sample(x * 2.4) * 0.34)
        hiss = high_noise.sample(x * 8.0) * 0.18

        sample = ((rumble + harmonics) * 0.34) + ((roar * flutter) * 0.64) + hiss
        samples.append(math.tanh(sample * 1.2))

    return normalize(make_loop_seamless(samples))


def render_missile_afterburner_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    low_noise = LoopNoise(97, 72)
    mid_noise = LoopNoise(101, 112)
    high_noise = LoopNoise(103, 176)
    samples: list[float] = []

    for index in range(sample_count):
        x = index / sample_count
        chop = 0.76 + (0.24 * math.sin(TAU * (5.0 * x + 0.13)))
        tone = (math.sin(TAU * (12.0 * x + 0.05 * math.sin(TAU * 3.0 * x))) * 0.44)
        harmonics = (math.sin(TAU * 27.0 * x) * 0.22) + (math.sin(TAU * 41.0 * x) * 0.12)
        roar = (low_noise.sample(x) * 0.44) + (mid_noise.sample(x * 3.8) * 0.40)
        hiss = high_noise.sample(x * 12.0) * 0.26

        sample = (tone + harmonics) * 0.30
        sample += (roar * 0.56 * chop) + hiss
        samples.append(math.tanh(sample * 1.28))

    return normalize(make_loop_seamless(samples))


def render_missile_flight_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    air_noise = LoopNoise(131, 96)
    whistle_noise = LoopNoise(149, 128)
    detail_noise = LoopNoise(157, 176)
    samples: list[float] = []

    for index in range(sample_count):
        x = index / sample_count
        gust = 0.72 + (0.28 * math.sin(TAU * (2.2 * x + 0.31)))
        whoosh = (air_noise.sample(x) * 0.58) + (detail_noise.sample(x * 5.5) * 0.24)
        whistle = math.sin(TAU * (18.0 * x + whistle_noise.sample(x * 2.0) * 0.08)) * 0.22
        overtone = math.sin(TAU * (31.0 * x + 0.03 * math.sin(TAU * 4.0 * x))) * 0.12
        hiss = detail_noise.sample(x * 10.5) * 0.16

        sample = (whoosh * gust * 0.70) + whistle + overtone + hiss
        samples.append(math.tanh(sample * 1.18))

    return normalize(make_loop_seamless(samples))


def render_flare_launch(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    crack_noise = LoopNoise(163, 128)
    hiss_noise = LoopNoise(167, 192)
    samples: list[float] = []

    for index in range(sample_count):
        t = index / SAMPLE_RATE
        x = index / sample_count
        punch = math.exp(-22.0 * t)
        tail = math.exp(-6.4 * x)
        pop = soft_square(TAU * (118.0 * t), 2.6) * 0.34
        snap = crack_noise.sample(x * 18.0) * math.exp(-34.0 * t) * 0.22
        hiss = hiss_noise.sample(x * 13.0) * tail * 0.36
        tone = math.sin(TAU * (620.0 * t + 18.0 * t * t)) * math.exp(-18.0 * t) * 0.20

        sample = ((pop + snap) * punch) + hiss + tone
        samples.append(math.tanh(sample * 1.42))

    return apply_fade(normalize(samples, peak=0.88), 48, 3600)


def render_flare_burn_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    fizz_noise = LoopNoise(269, 160)
    sparkle_noise = LoopNoise(271, 224)
    body_noise = LoopNoise(277, 96)
    samples: list[float] = []

    for index in range(sample_count):
        x = index / sample_count
        flicker = 0.78 + (0.22 * soft_square(TAU * (6.0 * x + 0.19), 1.9))
        shimmer = 0.84 + (0.16 * math.sin(TAU * (9.0 * x + 0.07)))
        hiss = fizz_noise.sample(x * 22.0) * 0.34
        crackle = sparkle_noise.sample(x * 46.0) * 0.14
        body = body_noise.sample(x * 7.0) * 0.22
        whistle = math.sin(TAU * (720.0 * x + fizz_noise.sample(x * 4.0) * 0.016)) * 0.18
        overtone = triangle_wave(TAU * (1380.0 * x + sparkle_noise.sample(x * 8.0) * 0.010)) * 0.10

        sample = ((hiss + crackle + body) * flicker * shimmer) + whistle + overtone
        samples.append(math.tanh(sample * 1.18))

    return normalize(make_loop_seamless(samples), peak=0.84)


def render_seeker_power_on(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    click_noise = LoopNoise(173, 96)
    shimmer_noise = LoopNoise(179, 128)
    samples: list[float] = []

    for index in range(sample_count):
        t = index / SAMPLE_RATE
        x = index / sample_count
        attack = smoothstep(0.0, 0.05, x)
        decay = math.exp(-4.8 * x)
        chirp_phase = TAU * (
            (420.0 * t)
            + (((1620.0 - 420.0) / max(duration_seconds, 1e-6)) * 0.5 * t * t)
        )
        chirp = math.sin(chirp_phase)
        harmonic = math.sin((chirp_phase * 1.96) + 0.22) * 0.34
        tail = triangle_wave(TAU * (126.0 * x + 0.02 * shimmer_noise.sample(x * 3.0))) * 0.16
        click = click_noise.sample(x * 24.0) * math.exp(-38.0 * t) * 0.16

        sample = ((chirp * 0.72) + harmonic + tail) * attack * decay
        sample += click
        samples.append(math.tanh(sample * 1.36))

    return apply_fade(normalize(samples, peak=0.86), 64, 4096)


def render_seeker_search_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    growl_noise = LoopNoise(181, 96)
    tone_noise = LoopNoise(191, 128)
    hiss_noise = LoopNoise(193, 176)
    samples: list[float] = []

    for index in range(sample_count):
        x = index / sample_count
        roughness = tone_noise.sample(x * 2.0)
        carrier_phase = TAU * (438.0 * x + roughness * 0.018)
        harmonic_phase = TAU * (876.0 * x + tone_noise.sample(x * 4.0) * 0.012)
        bark_phase = TAU * (612.0 * x + growl_noise.sample(x * 1.5) * 0.028)
        flutter = 0.82 + (0.18 * soft_square(TAU * (5.0 * x + 0.09), 1.8))
        scan = 0.88 + (0.12 * math.sin(TAU * (3.0 * x + 0.17)))

        carrier = triangle_wave(carrier_phase) * 0.34
        harmonic = math.sin(harmonic_phase) * 0.22
        bark = soft_square(bark_phase, 2.2) * 0.26
        texture = growl_noise.sample(x * 6.0) * 0.16
        hiss = hiss_noise.sample(x * 14.0) * 0.08

        sample = ((carrier + harmonic + bark) * flutter * scan) + texture + hiss
        samples.append(math.tanh(sample * 1.22))

    return normalize(make_loop_seamless(samples), peak=0.88)


def render_seeker_lock_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    warp_noise = LoopNoise(211, 96)
    grit_noise = LoopNoise(223, 160)
    samples: list[float] = []

    for index in range(sample_count):
        x = index / sample_count
        warp = warp_noise.sample(x * 3.0)
        carrier_phase = TAU * (1468.0 * x + warp * 0.010)
        harmonic_phase = TAU * (2936.0 * x + warp_noise.sample(x * 5.0) * 0.008)
        sideband_phase = TAU * (980.0 * x + warp * 0.020)
        warble = 0.86 + (0.14 * math.sin(TAU * (7.0 * x + 0.11)))

        carrier = math.sin(carrier_phase) * 0.58
        harmonic = math.sin(harmonic_phase + 0.18) * 0.24
        sideband = triangle_wave(sideband_phase) * 0.12
        grit = grit_noise.sample(x * 20.0) * 0.06

        sample = ((carrier + harmonic + sideband) * warble) + grit
        samples.append(math.tanh(sample * 1.18))

    return normalize(make_loop_seamless(samples), peak=0.84)


def render_seeker_lock_acquire(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    click_noise = LoopNoise(227, 112)
    samples: list[float] = []

    for index in range(sample_count):
        t = index / SAMPLE_RATE
        x = index / sample_count
        pulse_a = pulse_envelope(x, 0.00, 0.46, attack_fraction=0.08, release_fraction=0.44)
        pulse_b = pulse_envelope(x, 0.56, 0.28, attack_fraction=0.06, release_fraction=0.40)

        phase_a = TAU * (
            (880.0 * t)
            + (((1880.0 - 880.0) / max(duration_seconds * 0.52, 1e-6)) * 0.5 * t * t)
        )
        local_b = max(t - (duration_seconds * 0.56), 0.0)
        phase_b = TAU * (1440.0 * local_b)

        tone = (math.sin(phase_a) * pulse_a * 0.72) + (math.sin(phase_b) * pulse_b * 0.48)
        click = click_noise.sample(x * 20.0) * math.exp(-34.0 * t) * 0.10
        samples.append(math.tanh((tone + click) * 1.42))

    return apply_fade(normalize(samples, peak=0.88), 48, 4096)


def render_maws_missile_warning_loop(duration_seconds: float) -> list[float]:
    sample_count = int(duration_seconds * SAMPLE_RATE)
    buzz_noise = LoopNoise(239, 160)
    pulse_specs = (
        (0.02, 0.10, 980.0, 1340.0, 1.00),
        (0.16, 0.10, 1020.0, 1400.0, 0.96),
        (0.44, 0.11, 1040.0, 1460.0, 1.00),
        (0.60, 0.11, 1080.0, 1520.0, 0.94),
    )
    samples: list[float] = []

    for index in range(sample_count):
        t = index / SAMPLE_RATE
        x = index / sample_count
        sample = 0.0

        for start, duration, base_hz, top_hz, gain in pulse_specs:
            env = pulse_envelope(x, start, duration, attack_fraction=0.10, release_fraction=0.36)
            if env <= 0.0:
                continue

            local_t = t - (start * duration_seconds)
            sweep = (top_hz - base_hz) / max(duration * duration_seconds, 1e-6)
            phase = TAU * ((base_hz * local_t) + (0.5 * sweep * local_t * local_t))
            tone = soft_square(phase, 2.4) * 0.48
            harmonic = math.sin((phase * 1.48) + 0.14) * 0.20
            buzz = buzz_noise.sample((x - start) * 32.0) * 0.08
            sample += (tone + harmonic + buzz) * env * gain

        samples.append(math.tanh(sample * 1.30))

    return apply_fade(normalize(samples, peak=0.82), 64, 4096)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    assets = {
        "fire_launch.wav": render_fire_launch(1.52),
        "flare_launch.wav": render_flare_launch(0.32),
        "flare_burn_loop.wav": render_flare_burn_loop(1.24),
        "explosion.wav": render_explosion(2.85),
        "jet_afterburner_loop.wav": render_jet_afterburner_loop(3.20),
        "missile_afterburner_loop.wav": render_missile_afterburner_loop(2.40),
        "missile_flight_loop.wav": render_missile_flight_loop(2.20),
        "maws_missile_warning_loop.wav": render_maws_missile_warning_loop(0.92),
        "seeker_power_on.wav": render_seeker_power_on(0.38),
        "seeker_search_loop.wav": render_seeker_search_loop(1.92),
        "seeker_lock_loop.wav": render_seeker_lock_loop(1.48),
        "seeker_lock_acquire.wav": render_seeker_lock_acquire(0.26),
    }

    for filename, samples in assets.items():
        write_wav(OUTPUT_DIR / filename, samples)
        print(f"Wrote {filename}")


if __name__ == "__main__":
    main()
