// Procedural sky, shared by the visible sky (skyboxShader), the lighting
// capture that feeds image-based lighting (buildCubeMapShader) and the water
// reflections (PBRSimpleShader), so all three always agree.
//
// Radiance is linear and HDR. The horizon colour is the scene fog colour, so
// distant land and sea fade into the sky without a seam.

uniform vec3 skySunDirection; // direction the light travels (sun -> scene)
uniform vec3 skySunColor;     // strength-scaled sun colour
uniform vec3 skyHorizonColor; // linear fog colour
uniform float skyTime;        // seconds, drifts the clouds

float skyHash(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float skyNoise(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = skyHash(i);
    float b = skyHash(i + vec2(1.0, 0.0));
    float c = skyHash(i + vec2(0.0, 1.0));
    float d = skyHash(i + vec2(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float skyFbm(vec2 p)
{
    float sum = 0.0;
    float amplitude = 0.5;
    mat2 rotate = mat2(1.6, 1.2, -1.2, 1.6);
    for (int octave = 0; octave < 5; ++octave)
    {
        sum += amplitude * skyNoise(p);
        p = rotate * p + vec2(7.3, -2.9);
        amplitude *= 0.5;
    }
    return sum / 0.96875;
}

vec3 skyToSun()
{
    return normalize(-skySunDirection);
}

// Clear sky: deep blue overhead easing into the haze at the horizon, a glow
// round the sun that warms as the sun gets low.
vec3 skyAtmosphere(vec3 dir)
{
    vec3 toSun = skyToSun();
    float sunHeight = clamp(toSun.y, -0.2, 1.0);
    float up = max(dir.y, 0.0);

    vec3 zenith = vec3(0.040, 0.115, 0.330) * (0.55 + 0.6 * smoothstep(0.0, 0.6, sunHeight));
    vec3 horizon = skyHorizonColor * 1.18;
    float towardHorizon = pow(1.0 - up, 5.0);
    vec3 sky = mix(zenith, horizon, towardHorizon);

    float cosSun = max(dot(dir, toSun), 0.0);
    sky += skySunColor * (pow(cosSun, 6.0) * 0.030 + pow(cosSun, 48.0) * 0.070);

    float lowSun = 1.0 - smoothstep(0.05, 0.50, sunHeight);
    sky += vec3(0.30, 0.15, 0.05) * lowSun * pow(cosSun, 3.0) * towardHorizon;

    // Under the horizon only haze shows (the sea covers the rest).
    sky = mix(sky, skyHorizonColor, smoothstep(0.0, -0.06, dir.y));
    return sky;
}

// A deck of fair-weather cumulus about two kilometres up. rgb is the lit
// cloud colour, a its coverage.
vec4 skyClouds(vec3 dir)
{
    if (dir.y <= 0.0)
    {
        return vec4(0.0);
    }
    vec3 toSun = skyToSun();
    vec2 drift = vec2(skyTime * 0.0035, skyTime * 0.0012);
    vec2 deck = dir.xz / max(dir.y, 0.035) * 2.2 + drift;

    float coverage = skyFbm(deck * 0.33);
    float detail = skyFbm(deck * 1.6 + 3.1);
    float density = smoothstep(0.50, 0.80, coverage * 0.78 + detail * 0.28);
    // Thin out toward the horizon, where the deck is too far to resolve.
    density *= smoothstep(0.015, 0.16, dir.y);

    // Self-shadow: denser cloud toward the sun darkens the near side.
    float towardSun = skyFbm((deck + toSun.xz * 0.30) * 0.33);
    float shade = clamp(1.0 - (towardSun - coverage) * 3.0, 0.40, 1.0);
    vec3 lit = skySunColor * 0.30 * shade + skyHorizonColor * 0.55;
    float silver = pow(max(dot(dir, toSun), 0.0), 10.0);
    lit += skySunColor * silver * 0.30 * (1.0 - density);
    return vec4(lit, density);
}

vec3 skyRadiance(vec3 dir, bool withClouds)
{
    dir = normalize(dir);
    vec3 color = skyAtmosphere(dir);
    if (withClouds)
    {
        vec4 cloud = skyClouds(dir);
        color = mix(color, cloud.rgb, cloud.a * 0.92);
    }
    return color;
}
