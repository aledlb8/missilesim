// Actual OpenGL renderer + actual Fighter::setAircraft selection regression.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "rendering/Renderer.h"
#include "objects/Fighter.h"
#include "flight/AircraftCatalog.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

namespace {
constexpr int width=1200, height=900;
const std::string output="build/fighter-review/game/";
using Pixels=std::vector<unsigned char>;
void require(bool ok,const std::string &name) {
    if (!ok) throw std::runtime_error(name);
    std::cout << "PASS " << name << '\n';
}
Pixels pixels() {
    Pixels result(width*height*3);
    glReadBuffer(GL_BACK); glPixelStorei(GL_PACK_ALIGNMENT,1);
    glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,result.data());
    return result;
}
size_t changedAircraftPixels(const Pixels &a,const Pixels &b) {
    // The procedural sky deliberately advances on wall time. Compare the
    // central aircraft region, excluding the cloud horizon, with an 8/255
    // color tolerance so sky rounding cannot masquerade as a model change.
    size_t changed=0;
    for (int y=height/5;y<height*4/5;++y)
        for (int x=width/10;x<width*9/10;++x) {
            const size_t offset=(y*width+x)*3;
            if (std::abs(int(a[offset])-int(b[offset]))>8 ||
                std::abs(int(a[offset+1])-int(b[offset+1]))>8 ||
                std::abs(int(a[offset+2])-int(b[offset+2]))>8) ++changed;
        }
    return changed;
}
void save(const Pixels &p,const std::string &name) {
    std::ofstream file(output+name+".ppm",std::ios::binary);
    file << "P6\n" << width << ' ' << height << "\n255\n";
    for (int y=height-1;y>=0;--y)
        file.write(reinterpret_cast<const char *>(p.data()+y*width*3),width*3);
}
void draw(Renderer &renderer,Fighter &fighter) {
    renderer.beginSceneFrame({.025f,.035f,.055f});
    renderer.render(&fighter);
    renderer.renderSceneEffects();renderer.presentSceneFrame();
}
void run() {
    std::filesystem::create_directories(output);
    std::ifstream input("assets/models/fighters/manifest.json");
    nlohmann::json manifest; input>>manifest;
    std::map<std::string,nlohmann::json> assets;
    for (const auto &entry:manifest.at("aircraft")) assets[entry.at("id")]=entry;
    Renderer renderer;
    renderer.setViewportSize(width,height);
    renderer.setPBRFogDensityScale(0);
    renderer.setPBRBloomStrength(0);
    renderer.setCameraFOV(36);
    renderer.setCameraPosition({14,108,17});renderer.setCameraTarget({0,100,0});
    require(renderer.hasPBR(),"real PBR pipeline initialized");
    Fighter fighter;
    fighter.place({0,100,0},{0,0,250},{0,0,1});
    fighter.adjustThrottle(-2);
    std::map<std::string,Pixels> images;
    const auto *catalog=missilesim::flight::aircraftCatalog();
    require(assets.size()==static_cast<size_t>(missilesim::flight::aircraftCatalogCount()),"manifest covers every flight card");
    for (int i=0;i<missilesim::flight::aircraftCatalogCount();++i) {
        const std::string id=catalog[i].id;
        fighter.setAircraft(id.c_str());
        require(fighter.jet().aircraftId()==id && renderer.hasAircraftModel(id.c_str()),id+" selection resolves its own mesh");
        const auto &asset=assets.at(id);
        const float length=asset.at("extent")[1];
        auto sockets=renderer.getExhaustSockets(fighter);
        require(sockets.size()==asset.at("sockets").size(),id+" engine count");
        for (size_t j=0;j<sockets.size();++j) {
            const auto &expected=asset.at("sockets")[j];
            const float x=expected.at("position")[0], y=expected.at("position")[1], z=expected.at("position")[2];
            require(glm::distance(sockets[j].position,glm::vec3(-x,100+z,y+length*.5f))<.002f &&
                    std::abs(sockets[j].radius-float(expected.at("radius")))<1e-5f &&
                    glm::distance(sockets[j].direction,glm::vec3(0,0,-1))<1e-5f,
                    id+" authored metre-scale exhaust transform "+std::to_string(j));
        }
        draw(renderer,fighter);const Pixels frame=pixels();
        for (const auto &[previous,image]:images)
            require(changedAircraftPixels(frame,image)>250,id+" differs visually from "+previous);
        images[id]=frame;save(frame,id);
        draw(renderer,fighter);
        require(changedAircraftPixels(frame,pixels())<20,id+" paused selection is stable");
        renderer.setCameraPosition({-16,105,-22});renderer.setCameraTarget({0,100,0});
        draw(renderer,fighter);save(pixels(),id+"-rear");
        renderer.setCameraPosition({0,140,.01f});renderer.setCameraTarget({0,100,0});
        draw(renderer,fighter);save(pixels(),id+"-top");
        renderer.setCameraPosition({14,108,17});renderer.setCameraTarget({0,100,0});
        require(glGetError()==GL_NO_ERROR,id+" front/rear/top without GL errors");
    }
    // Exercise cached selections in reverse, after every other model was used.
    for (int i=missilesim::flight::aircraftCatalogCount()-1;i>=0;--i) {
        const std::string id=catalog[i].id;fighter.setAircraft(id.c_str());draw(renderer,fighter);
        require(changedAircraftPixels(pixels(),images.at(id))<20,id+" switch-back restores mesh/materials");
    }
    fighter.setAircraft("rafale-c");
    const auto before=renderer.getExhaustSockets(fighter);
    fighter.place({50,140,-30},{250,0,0},{1,0,0});
    const auto after=renderer.getExhaustSockets(fighter);
    require(std::abs(glm::distance(before[0].position,before[1].position)-
                     glm::distance(after[0].position,after[1].position))<1e-4f,
            "yaw and translation preserve nozzle separation");
    fighter.setAircraft("invalid-id");
    require(renderer.hasAircraftModel(fighter.jet().aircraftId()),"unknown selection uses flight catalog fallback");
}
}
int main() {
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,4);glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,5);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);glfwWindowHint(GLFW_VISIBLE,GLFW_FALSE);
    auto *window=glfwCreateWindow(width,height,"Aircraft verification",nullptr,nullptr);
    if (!window) {glfwTerminate();return 2;}
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) return 3;
    int result=0;
    try {run();} catch(const std::exception &error) {std::cerr<<"FAIL "<<error.what()<<'\n';result=1;}
    glfwDestroyWindow(window);glfwTerminate();return result;
}
