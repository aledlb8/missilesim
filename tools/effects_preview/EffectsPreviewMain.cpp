// Effects preview: the countermeasure effects drawn by the game's own
// SceneEffects code in a hidden window, over a flat sky, read back and written
// as PNG files. Chaff moves with the simulation's own chaff model
// (sim/defense/Chaff), so what is drawn is where the radar sees the bundle.
// The sky is a flat colour and the bloom and tone curve are a cheap stand-in
// for the game's, so judge shape, motion and colour here, not exact exposure.
//
// Usage: effects_preview [out-dir]   (default: screenshots/effects_preview)
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include "rendering/SceneEffects.h"
#include "sim/defense/Chaff.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace sim = missilesim::sim;

namespace
{
    constexpr int kWidth = 1280;
    constexpr int kHeight = 720;
    constexpr float kFrameS = 1.0f / 60.0f;
    const glm::vec3 kSky(0.50f, 0.64f, 0.82f);

    // The game's chaff (WorldRadar.cpp): 20 m2, 8 s, 0.4 s drag, 1.5 m/s fall.
    sim::ChaffDispenser gameDispenser()
    {
        sim::ChaffDispenser dispenser;
        dispenser.remaining = 1000;
        dispenser.rcsM2 = 20.0f;
        dispenser.lifetimeS = 8.0;
        dispenser.ejectSpeed = 30.0f;
        dispenser.dragTimeS = 0.4;
        dispenser.fallSpeedMps = 1.5f;
        return dispenser;
    }

    struct Shot
    {
        const char *name;
        glm::vec3 eye;
        glm::vec3 look;
        float fovDeg;
        glm::vec3 background = kSky;
    };

    // A jet flies along +x at 250 m/s and holds a key down: a chaff bundle
    // every 0.2 s (or a flare pair every 0.3 s). The picture is taken `duration`
    // seconds after the first release.
    struct Scenario
    {
        bool chaff = true;
        bool flares = false;
        float duration = 2.0f;
        float releaseFor = 1.0f; // the key is held this long
        glm::vec3 jetStart{-150.0f, 0.0f, 0.0f};
    };

    float aces(float x)
    {
        return std::clamp((x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f), 0.0f, 1.0f);
    }

    // Read back HDR, add a cheap bloom, tone map, write sRGB PNG.
    bool writeFrame(const std::filesystem::path &path)
    {
        std::vector<float> hdr(static_cast<std::size_t>(kWidth) * kHeight * 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_FLOAT, hdr.data());

        // Bright-pass, then a separable blur (two radii) added back.
        std::vector<glm::vec3> bright(static_cast<std::size_t>(kWidth) * kHeight);
        for (std::size_t i = 0; i < bright.size(); ++i)
        {
            const glm::vec3 c(hdr[i * 4], hdr[i * 4 + 1], hdr[i * 4 + 2]);
            const float luma = glm::dot(c, glm::vec3(0.2126f, 0.7152f, 0.0722f));
            bright[i] = c * std::max(0.0f, luma - 1.2f) / std::max(luma, 1.0e-4f);
        }
        auto blur = [&](const std::vector<glm::vec3> &source, int radius)
        {
            std::vector<float> weights(static_cast<std::size_t>(radius) + 1);
            float total = 0.0f;
            for (int k = 0; k <= radius; ++k)
            {
                weights[k] = std::exp(-0.5f * (k * k) / (0.25f * radius * radius));
                total += k == 0 ? weights[k] : 2.0f * weights[k];
            }
            std::vector<glm::vec3> pass(source.size()), result(source.size());
            for (int y = 0; y < kHeight; ++y)
                for (int x = 0; x < kWidth; ++x)
                {
                    glm::vec3 sum(0.0f);
                    for (int k = -radius; k <= radius; ++k)
                        sum += source[y * kWidth + std::clamp(x + k, 0, kWidth - 1)] * weights[std::abs(k)];
                    pass[y * kWidth + x] = sum / total;
                }
            for (int y = 0; y < kHeight; ++y)
                for (int x = 0; x < kWidth; ++x)
                {
                    glm::vec3 sum(0.0f);
                    for (int k = -radius; k <= radius; ++k)
                        sum += pass[std::clamp(y + k, 0, kHeight - 1) * kWidth + x] * weights[std::abs(k)];
                    result[y * kWidth + x] = sum / total;
                }
            return result;
        };
        const std::vector<glm::vec3> tight = blur(bright, 4);
        const std::vector<glm::vec3> wide = blur(bright, 16);

        std::vector<unsigned char> pixels(static_cast<std::size_t>(kWidth) * kHeight * 3);
        for (std::size_t i = 0; i < bright.size(); ++i)
        {
            glm::vec3 c(hdr[i * 4], hdr[i * 4 + 1], hdr[i * 4 + 2]);
            c += tight[i] * 0.6f + wide[i] * 0.35f;
            for (int channel = 0; channel < 3; ++channel)
            {
                const float mapped = std::pow(aces(c[channel] * 0.8f), 1.0f / 2.2f);
                pixels[i * 3 + channel] = static_cast<unsigned char>(std::lround(mapped * 255.0f));
            }
        }
        stbi_flip_vertically_on_write(1);
        return stbi_write_png(path.string().c_str(), kWidth, kHeight, 3, pixels.data(), kWidth * 3) != 0;
    }

    void runScenario(SceneEffects &effects, const Scenario &scenario, const Shot &shot, GLuint framebuffer,
                     const std::filesystem::path &outDir)
    {
        effects.clear();
        sim::ChaffDispenser dispenser = gameDispenser();
        std::vector<sim::ChaffRound> rounds;
        std::vector<std::uint32_t> drawn;
        const glm::vec3 jetVelocity(250.0f, 0.0f, 0.0f);
        const float stepS = 0.01f;
        double time = 0.0;
        double nextChaff = 0.0;
        double nextFlare = 0.0;
        float side = 1.0f;
        std::uint32_t nextId = 100;

        // Flares: the game's cartridge burns 4 s and is drawn by its own effect.
        // Ballistic here (gravity plus drag) is close enough for a look.
        struct FlareBody
        {
            glm::vec3 position;
            glm::vec3 previous;
            glm::vec3 velocity;
            float age;
        };
        std::vector<FlareBody> flareBodies;

        const glm::mat4 view = glm::lookAt(shot.eye, shot.look, glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::mat4 projection = glm::perspective(glm::radians(shot.fovDeg), float(kWidth) / float(kHeight), 0.5f, 20000.0f);
        effects.setViewportSize(kWidth, kHeight);
        effects.setCamera(shot.eye, view, projection);

        const int frames = static_cast<int>(std::lround(scenario.duration / kFrameS));
        for (int frame = 0; frame < frames; ++frame)
        {
            // Simulation steps inside this frame.
            const double frameEnd = (frame + 1) * static_cast<double>(kFrameS);
            while (time + 1.0e-9 < frameEnd)
            {
                const glm::vec3 jet = scenario.jetStart + jetVelocity * static_cast<float>(time);
                if (scenario.chaff && time <= scenario.releaseFor && time + 1.0e-9 >= nextChaff)
                {
                    sim::releaseChaff(dispenser, rounds, sim::EntityId{nextId++}, time, jet + glm::vec3(-4.0f, -0.8f, 0.7f * side),
                                      jetVelocity, glm::vec3(0.0f, -1.0f, 0.35f * side));
                    side = -side;
                    nextChaff += 0.2;
                }
                if (scenario.flares && time <= scenario.releaseFor && time + 1.0e-9 >= nextFlare)
                {
                    for (const float flareSide : {1.0f, -1.0f})
                    {
                        const glm::vec3 eject = glm::normalize(glm::vec3(-0.25f, -0.8f, 0.45f * flareSide)) * 45.0f;
                        const glm::vec3 start = jet + glm::vec3(-4.0f, -0.8f, 0.7f * flareSide);
                        flareBodies.push_back({start, start, jetVelocity + eject, 0.0f});
                    }
                    nextFlare += 0.3;
                }
                time += stepS;
                sim::stepChaff(rounds, time, stepS);
                for (FlareBody &flare : flareBodies)
                {
                    const float speed = glm::length(flare.velocity);
                    flare.velocity += (glm::vec3(0.0f, -9.81f, 0.0f) - flare.velocity * speed * 0.011f) * stepS;
                    flare.position += flare.velocity * stepS;
                    flare.age += stepS;
                }
            }

            effects.beginEngineFrame();
            effects.update(kFrameS);
            for (const sim::ChaffRound &round : rounds)
            {
                if (!round.alive)
                {
                    continue;
                }
                const bool birth = std::find(drawn.begin(), drawn.end(), round.id.value) == drawn.end();
                if (birth)
                {
                    drawn.push_back(round.id.value);
                }
                effects.submitChaffCloud(round.position, round.velocity, static_cast<float>(time - round.birthTime),
                                         static_cast<float>(round.lifetimeS), round.rcsM2 / round.birthRcsM2,
                                         round.id.value, birth);
            }
            for (FlareBody &flare : flareBodies)
            {
                const float heat = std::exp(-1.3f * flare.age);
                if (flare.age < 4.0f)
                {
                    effects.emitFlareEffect(flare.previous, flare.position, flare.velocity, heat);
                }
                flare.previous = flare.position;
            }
        }

        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        glViewport(0, 0, kWidth, kHeight);
        glClearColor(shot.background.r, shot.background.g, shot.background.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        effects.renderParticlesToScene();
        const std::filesystem::path path = outDir / (std::string(shot.name) + ".png");
        std::printf("%s %s\n", writeFrame(path) ? "wrote" : "FAILED", path.string().c_str());
    }
}

int main(int argc, char **argv)
{
    const std::filesystem::path outDir = argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path("screenshots/effects_preview");
    std::error_code error;
    std::filesystem::create_directories(outDir, error);

    if (!glfwInit())
    {
        std::fprintf(stderr, "effects_preview: glfwInit failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(kWidth, kHeight, "effects_preview", nullptr, nullptr);
    if (window == nullptr)
    {
        std::fprintf(stderr, "effects_preview: no OpenGL 4.5 window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::fprintf(stderr, "effects_preview: glad failed\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // HDR target, as the game's scene buffer is.
    GLuint framebuffer = 0;
    GLuint color = 0;
    GLuint depth = 0;
    glGenFramebuffers(1, &framebuffer);
    glGenTextures(1, &color);
    glBindTexture(GL_TEXTURE_2D, color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, kWidth, kHeight, 0, GL_RGBA, GL_FLOAT, nullptr);
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, kWidth, kHeight);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);

    SceneEffects effects;
    effects.initialize();

    // Chase view just after a 1 s burst of chaff: the jet is off to the right.
    Scenario stream;
    stream.duration = 1.4f;
    runScenario(effects, stream, {"chaff_burst_chase", glm::vec3(-60.0f, 12.0f, -70.0f), glm::vec3(30.0f, -4.0f, 0.0f), 60.0f},
                framebuffer, outDir);

    // The same burst 4 s later, seen from the side: clouds hanging and sinking.
    Scenario hanging;
    hanging.duration = 4.0f;
    runScenario(effects, hanging, {"chaff_hanging_side", glm::vec3(0.0f, -6.0f, -170.0f), glm::vec3(0.0f, -14.0f, 0.0f), 50.0f},
                framebuffer, outDir);

    // The same, looking down onto ground from above.
    runScenario(effects, hanging,
                {"chaff_over_ground", glm::vec3(-20.0f, 60.0f, -110.0f), glm::vec3(0.0f, -14.0f, 0.0f), 50.0f,
                 glm::vec3(0.10f, 0.12f, 0.08f)},
                framebuffer, outDir);

    // Flares for comparison: same release, same camera.
    Scenario flares;
    flares.chaff = false;
    flares.flares = true;
    flares.duration = 1.4f;
    runScenario(effects, flares, {"flare_burst_chase", glm::vec3(-60.0f, 12.0f, -70.0f), glm::vec3(30.0f, -4.0f, 0.0f), 60.0f},
                framebuffer, outDir);

    // Both together, far away: does chaff still read at 1 km?
    Scenario both;
    both.flares = true;
    both.duration = 2.0f;
    runScenario(effects, both, {"mixed_far", glm::vec3(0.0f, 30.0f, -1000.0f), glm::vec3(0.0f, -10.0f, 0.0f), 30.0f},
                framebuffer, outDir);

    // One bundle close up, 1.5 s after release.
    Scenario single;
    single.releaseFor = 0.0f;
    single.duration = 1.5f;
    runScenario(effects, single, {"chaff_single_close", glm::vec3(-50.0f, -6.0f, -40.0f), glm::vec3(-52.0f, -13.0f, 0.0f), 50.0f},
                framebuffer, outDir);

    effects.shutdown();
    glDeleteRenderbuffers(1, &depth);
    glDeleteTextures(1, &color);
    glDeleteFramebuffers(1, &framebuffer);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
