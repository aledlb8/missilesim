# HUD preview: draws the radar scope, the RWR and the fire-control symbols
# with scripted pictures in a hidden window and writes one PNG per picture, so
# their look can be reviewed without flying. Usage: hud_preview [out-dir]
add_executable(hud_preview
    HudPreviewMain.cpp
    ${PROJECT_SOURCE_DIR}/src/ui/Theme.cpp
    ${PROJECT_SOURCE_DIR}/src/ui/Widgets.cpp
    ${PROJECT_SOURCE_DIR}/src/ui/RadarScope.cpp
    ${PROJECT_SOURCE_DIR}/src/ui/RwrScope.cpp
    ${PROJECT_SOURCE_DIR}/src/ui/TargetSymbols.cpp)
target_include_directories(hud_preview PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/third_party)
target_link_libraries(hud_preview PRIVATE imgui glfw glad::glad)
target_compile_definitions(hud_preview PRIVATE
    MISSILESIM_SOURCE_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets")
