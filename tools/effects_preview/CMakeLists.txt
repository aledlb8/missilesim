# Effects preview: draws chaff and flares with the game's SceneEffects code in
# a hidden window and writes one PNG per scripted shot, so their look can be
# reviewed without flying. Usage: effects_preview [out-dir]
file(GLOB EFFECTS_PREVIEW_SCENE_EFFECTS CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/rendering/SceneEffects*.cpp)
add_executable(effects_preview
    EffectsPreviewMain.cpp
    ${EFFECTS_PREVIEW_SCENE_EFFECTS})
target_include_directories(effects_preview PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/rendering
    ${PROJECT_SOURCE_DIR}/third_party)
target_link_libraries(effects_preview PRIVATE missilesim_core glfw glad::glad glm::glm)
