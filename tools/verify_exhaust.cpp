// Public-API GPU regression: socket transforms, frame independence, occlusion,
// cutoff and paused rendering. See verify_exhaust.py for the build/run command.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "rendering/Renderer.h"
#include "rendering/SceneEffects.h"
#include "objects/Target.h"
#include "objects/Missile.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
constexpr int width = 1280, height = 900;
const std::string output = "build/exhaust-review/";
using Pixels = std::vector<float>;
class ReviewTarget : public Target {
public:
    using Target::Target;
    glm::vec3 acceleration{0};
    glm::vec3 getRenderAcceleration() const override { return acceleration; }
};
void require(bool ok, const std::string &name) {
    if (!ok) throw std::runtime_error(name);
    std::cout << "PASS " << name << '\n';
}
Pixels readPixels() {
    Pixels pixels(width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, pixels.data());
    return pixels;
}
double energy(const Pixels &pixels) {
    return std::accumulate(pixels.begin(), pixels.end(), 0.0);
}
double difference(const Pixels &a, const Pixels &b) {
    double result = 0;
    for (size_t i = 0; i < a.size(); ++i) result += std::abs(a[i] - b[i]);
    return result;
}
void capture(const std::string &name) {
    glReadBuffer(GL_BACK);
    auto pixels = readPixels();
    std::ofstream file(output + name + ".ppm", std::ios::binary);
    file << "P6\n" << width << ' ' << height << "\n255\n";
    std::vector<unsigned char> row(width * 3);
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width * 3; ++x) {
            row[x] = static_cast<unsigned char>(std::clamp(pixels[y * width * 3 + x], 0.f, 1.f) * 255 + .5f);
        }
        file.write(reinterpret_cast<const char *>(row.data()), row.size());
    }
}
void camera(SceneEffects &effects, glm::vec3 eye, glm::vec3 look) {
    effects.setCamera(eye, glm::lookAt(eye, look, glm::vec3(0, 1, 0)),
                      glm::perspective(glm::radians(45.f), float(width) / height, .01f, 100.f));
}
Pixels volume(SceneEffects &effects, bool emit = true, bool occluded = false) {
    effects.beginEngineFrame();
    effects.beginScene({0, 0, 0});
    if (occluded) {
        glClearDepth(0.001); glClear(GL_DEPTH_BUFFER_BIT); glClearDepth(1);
    }
    if (emit) effects.submitEnginePlume({0, 0, 0}, {0, 0, -1}, .12f, 1, true);
    effects.renderParticlesToScene();
    return readPixels();
}
void volumeChecks() {
    SceneEffects effects;
    effects.setViewportSize(width, height);
    camera(effects, {3, 1, -2}, {0, 0, -1.5f});
    auto lit = volume(effects);
    require(energy(lit) > 100, "side-on volume is visible");
    require(difference(lit, volume(effects)) == 0, "paused frames do not accumulate or flicker");
    require(energy(volume(effects, false)) == 0, "beginEngineFrame removes stale fire immediately");
    require(energy(volume(effects, true, true)) == 0, "foreground depth fully occludes volume (legacy depth snapshot)");
    for (auto eye : {glm::vec3(0, 0, -6), glm::vec3(0, 0, -1)}) {
        camera(effects, eye, eye + glm::vec3(0, 0, 1));
        auto pixels = volume(effects);
        require(energy(pixels) > 100 && std::all_of(pixels.begin(), pixels.end(), [](float v) { return std::isfinite(v); }),
                eye.z == -6 ? "end-on volume finite and visible" : "camera inside volume finite and visible");
    }
    effects.beginEngineFrame(); effects.beginScene({0, 0, 0});
    effects.submitEnginePlume({0, 0, 0}, {0, 0, -1}, .12f, 0, true);
    effects.submitEnginePlume({0, 0, 0}, {0, 0, -1}, -1, 1, true);
    effects.submitEnginePlume({0, 0, 0}, {0, 0, -1}, std::numeric_limits<float>::quiet_NaN(), 1, true);
    effects.renderParticlesToScene();
    require(energy(readPixels()) == 0, "zero throttle and invalid sizes emit nothing");
    Pixels reference;
    for (int fps : {30, 60, 144}) {
        SceneEffects timed;
        timed.setViewportSize(width, height);
        timed.initialize();
        camera(timed, {3, 1, -2}, {0, 0, -1.5f});
        for (int i = 0; i < fps; ++i) timed.update(1.f / fps);
        auto pixels = volume(timed);
        if (reference.empty()) reference = pixels;
        const double error = difference(reference, pixels) / std::max(energy(reference), 1.0);
        require(error < .002, "equal-time image at " + std::to_string(fps) + " FPS (relative error " + std::to_string(error) + ")");
    }
    camera(effects, {6, 2, -6}, {0, 0, -2});
    GLuint query = 0; glGenQueries(1, &query);
    for (int count : {3, 32}) {
        std::vector<double> timings;
        for (int iteration = 0; iteration < 35; ++iteration) {
            effects.beginEngineFrame(); effects.beginScene({0, 0, 0});
            for (int i = 0; i < count; ++i)
                effects.submitEnginePlume({(i % 8 - 3.5f) * .7f, (i / 8) * .6f, 0},
                                          {0, 0, -1}, i == 2 ? .04f : .268f, .9f, i == 2);
            glBeginQuery(GL_TIME_ELAPSED, query);
            effects.renderParticlesToScene();
            glEndQuery(GL_TIME_ELAPSED);
            GLuint64 ns = 0; glGetQueryObjectui64v(query, GL_QUERY_RESULT, &ns);
            if (iteration >= 5) timings.push_back(double(ns) / 1e6);
        }
        std::sort(timings.begin(), timings.end());
        std::cout << "GPU " << count << " visible plumes, " << width << 'x' << height
                  << ": median " << timings[timings.size()/2] << " ms, max " << timings.back() << " ms (includes depth copy)\n";
    }
    glDeleteQueries(1, &query);
    require(glGetError() == GL_NO_ERROR, "volume checks have no OpenGL errors");
}
void render(Renderer &renderer, PhysicsObject &object) {
    renderer.beginSceneFrame({.1f, .1f, .1f});
    renderer.renderEnvironment(); renderer.render(&object);
    renderer.renderSceneEffects(); renderer.presentSceneFrame();
    glReadBuffer(GL_BACK);
}
void sceneChecks(bool motion) {
    Renderer renderer;
    renderer.setViewportSize(width, height);
    renderer.setWorldGuidesEnabled(false); renderer.setPBRFogDensityScale(.2f);
    ReviewTarget jet({0, 100, 0}, 5); jet.setVelocity({0, 0, 240});
    Missile missile({0, 100, 0}, {0, 0, 240});
    missile.setThrustEnabled(true); missile.setThrottle(1); missile.setFuel(100);
    auto sockets = renderer.getExhaustSockets(jet);
    require(sockets.size() == 2 && std::abs(sockets[0].radius - .267966f) < 1e-5f &&
            std::abs(sockets[0].position.z + 4.98782f) < 1e-4f, "jet sockets match normalized outlet geometry");
    jet.acceleration = {22, 0, 0};
    auto banked = renderer.getExhaustSockets(jet);
    require(std::abs(glm::length(sockets[0].position - sockets[1].position) -
                     glm::length(banked[0].position - banked[1].position)) < 1e-5f &&
            glm::distance(sockets[0].position, banked[0].position) > .1f, "bank rotates both nozzle sockets rigidly");
    jet.acceleration = {0, 0, 0};
    auto rocketSockets = renderer.getExhaustSockets(missile);
    require(rocketSockets.size() == 1 && std::abs(rocketSockets[0].position.z + 1) < 1e-5f &&
            std::abs(rocketSockets[0].radius - .0379358f) < 1e-6f, "missile socket matches rear outlet geometry");
    for (auto *object : {static_cast<PhysicsObject *>(&jet), static_cast<PhysicsObject *>(&missile)}) {
        for (int view = 0; view < 4; ++view) {
            bool rocket = object == &missile;
            glm::vec3 look = rocket ? glm::vec3(0, 100, -.8f) : glm::vec3(0, 99.3f, -3.6f);
            glm::vec3 eye = rocket ? glm::vec3(2, 100.8f, -2.8f) : glm::vec3(6, 102, -10);
            if (view == 1) eye = rocket ? glm::vec3(0, 100, -3.2f) : glm::vec3(0, 99.3f, -9);
            if (view == 2) eye = rocket ? glm::vec3(3, 100, -.7f) : glm::vec3(9, 100, -4.5f);
            if (view == 3) {
                jet.acceleration = {22, 0, 0}; eye = {4, 101, -8};
                if (rocket) { missile.setVelocity({0, 240, 0}); look = {0, 99.2f, 0}; eye = {2, 99, 3}; }
            }
            renderer.setCameraPosition(eye); renderer.setCameraTarget(look); renderer.updateEffects(1.f/60);
            render(renderer, *object);
            capture(object->getType() + "-" + std::to_string(view));
        }
    }
    missile.setVelocity({0, 0, 240}); renderer.setCameraPosition({3, 100, -.7f}); renderer.setCameraTarget({0, 100, -.8f});
    render(renderer, missile); auto firing = readPixels();
    render(renderer, missile);
    require(difference(firing, readPixels()) == 0, "PBR paused engine and local lights stay stable");
    missile.setThrottle(0); render(renderer, missile); auto off = readPixels();
    require(difference(firing, off) > 100, "zero missile throttle removes visible fire");
    missile.setThrottle(1); missile.setFuel(0); render(renderer, missile);
    require(difference(off, readPixels()) == 0, "empty fuel removes fire immediately");
    missile.setFuel(100); missile.setThrustEnabled(false); render(renderer, missile);
    require(difference(off, readPixels()) == 0, "engine cutoff removes fire immediately");
    jet.acceleration = {0, 0, 0}; renderer.setCameraPosition({6, 102, -10}); renderer.setCameraTarget({0, 99.3f, -3.6f});
    render(renderer, jet); auto active = readPixels();
    jet.setActive(false); render(renderer, jet);
    require(difference(active, readPixels()) > 100, "inactive jet removes fire");
    jet.setActive(true); missile.setThrustEnabled(true);
    if (motion) {
        std::filesystem::create_directories(output + "motion");
        for (int frame = 0; frame < 180; ++frame) {
            bool rocket = frame >= 90;
            float t = float(frame % 90) / 89.f;
            jet.acceleration = {std::sin(t * 6.2831853f) * 24, 0, 0};
            auto *object = rocket ? static_cast<PhysicsObject *>(&missile) : static_cast<PhysicsObject *>(&jet);
            float angle = .15f + t * 1.3f;
            auto eye = rocket ? glm::vec3(std::sin(angle)*3, 100.5f, -std::cos(angle)*3) :
                                glm::vec3(std::sin(angle)*9, 101, -3-std::cos(angle)*8);
            renderer.setCameraPosition(eye); renderer.setCameraTarget(rocket ? glm::vec3(0,100,-.8f) : glm::vec3(0,99.3f,-3.6f));
            renderer.updateEffects(1.f/30); render(renderer, *object);
            std::ostringstream name; name << "motion/frame-" << std::setw(4) << std::setfill('0') << frame;
            capture(name.str());
        }
    }
    require(glGetError() == GL_NO_ERROR, "PBR views and lifecycle have no OpenGL errors");
}
}
int main(int argc, char **) {
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    auto *window = glfwCreateWindow(width, height, "Exhaust review", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 2; }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) return 3;
    int status = 0;
    try {
        std::cout << "GPU: " << glGetString(GL_RENDERER) << '\n';
        volumeChecks(); sceneChecks(argc > 1);
    } catch (const std::exception &e) { std::cerr << "FAIL " << e.what() << '\n'; status = 1; }
    glfwDestroyWindow(window); glfwTerminate(); return status;
}
