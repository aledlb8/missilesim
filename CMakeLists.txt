cmake_minimum_required(VERSION 3.21)
project(MissileSimOpenGL VERSION 0.1.0)

# PORTABLE builds are the ones handed to other people (see the `dist` preset):
# statically linked (no DLLs, no VC++ redistributable), no console window,
# relocatable (no absolute source paths baked in), with an install/CPack step
# that zips the exe together with the assets it needs.
option(MISSILESIM_PORTABLE "Build a self-contained, shareable executable" OFF)
option(MISSILESIM_BUILD_TOOLS "Build developer tools (AudioAudition)" ON)

if(MISSILESIM_PORTABLE AND MSVC)
    # Must match the static vcpkg triplet (x64-windows-static) and be set
    # before any target, including fetched dependencies, is created.
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable experimental GLM extensions
add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

# Ensure RTTI is enabled (may be disabled by default in some compilers)
if(MSVC)
    # For Visual Studio, remove /GR- flag if present and add /GR
    string(REGEX REPLACE "/GR-" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR")
else()
    # For GCC/Clang, remove -fno-rtti if present and add -frtti
    string(REGEX REPLACE "-fno-rtti" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")
endif()

# Set build output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find packages using vcpkg
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# The vcpkg glfw3 package provides a target named 'glfw', not 'glfw3'
# Let's create an alias for clarity
if(TARGET glfw AND NOT TARGET glfw3)
    add_library(glfw3 ALIAS glfw)
endif()

# ImGui
include(FetchContent)
set(MINIAUDIO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MINIAUDIO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MINIAUDIO_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.6
)
FetchContent_Declare(
    miniaudio
    GIT_REPOSITORY https://github.com/mackron/miniaudio.git
    GIT_TAG 0.11.23
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)
FetchContent_MakeAvailable(miniaudio)

# Add ImGui as a library
set(IMGUI_SOURCES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC 
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw3)

# Physical audio engine + synthesis voices (always optimised, see its CMakeLists).
add_subdirectory(src/audio)

# The simulation without windows, rendering or audio: physics, flight models,
# objects and the engagement World. The game and the headless tools link the
# same library, so what the harness checks is what the game runs.
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    "src/physics/*.cpp"
    "src/objects/*.cpp"
    "src/flight/*.cpp"
    "src/sim/*.cpp"
)
add_library(missilesim_core STATIC ${CORE_SOURCES})
target_include_directories(missilesim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(missilesim_core PUBLIC glm::glm PRIVATE nlohmann_json::nlohmann_json)
if(NOT MISSILESIM_PORTABLE)
    # Lets the config loader find assets/ when a tool runs from the build tree.
    target_compile_definitions(missilesim_core PRIVATE
        MISSILESIM_SOURCE_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
endif()

# Headless tools and their checks (CTest).
if(MISSILESIM_BUILD_TOOLS)
    enable_testing()
    add_subdirectory(tools/audition)
    add_subdirectory(tools/fox2_kinematics)
    add_subdirectory(tools/flight_harness)
    add_subdirectory(tools/engagement_harness)
    add_subdirectory(tools/hud_preview)
endif()

# Game sources: everything that is not in the core library or the audio library.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
)
list(FILTER SOURCES EXCLUDE REGEX "/src/audio/(engine|synth)/")
list(FILTER SOURCES EXCLUDE REGEX "/src/(physics|objects|flight|sim)/")

# Include directories for our project
set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/application
    ${CMAKE_CURRENT_SOURCE_DIR}/src/physics
    ${CMAKE_CURRENT_SOURCE_DIR}/src/physics/forces
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objects
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/pbr
    ${CMAKE_CURRENT_SOURCE_DIR}/src/audio
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE missilesim_core glfw glad::glad glm::glm imgui missilesim_audio assimp::assimp nlohmann_json::nlohmann_json)

target_compile_definitions(${PROJECT_NAME} PRIVATE MISSILESIM_VERSION="${PROJECT_VERSION}")

if(MISSILESIM_PORTABLE)
    # MISSILESIM_SOURCE_ASSET_DIR is deliberately left undefined: a shared build
    # must only ever load the assets shipped next to it.
    target_compile_definitions(${PROJECT_NAME} PRIVATE MISSILESIM_PORTABLE=1)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        OUTPUT_NAME "MissileSim"
        WIN32_EXECUTABLE ON
    )
    if(MSVC)
        # GUI subsystem (no console window) while keeping the standard main().
        target_link_options(${PROJECT_NAME} PRIVATE /ENTRY:mainCRTStartup)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        MISSILESIM_SOURCE_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets"
    )
endif()

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${INCLUDES}
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/missile.obj
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/missile.obj
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/jet.obj
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/jet.obj
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/assets.md
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/assets.md
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/config
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/config
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/pbr
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/shaders/pbr
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/skyboxes
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/skyboxes
)

# Fleet assets are build inputs in their own right. Refresh copied runtime
# meshes even when only a Blender export changed and the executable is current.
file(GLOB FIGHTER_RUNTIME_MODELS CONFIGURE_DEPENDS "assets/models/fighters/*.obj")
if(FIGHTER_RUNTIME_MODELS)
    add_custom_target(fighter_assets ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/fighters
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FIGHTER_RUNTIME_MODELS}
                $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/fighters
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/fighters/manifest.json
                $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/fighters/manifest.json
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/jet.obj
                $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/jet.obj
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/assets/assets.md
                $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/assets.md
        COMMENT "Synchronizing fighter runtime meshes")
else()
    # No exported obj yet. An empty copy_if_different list is not a valid cmake -E command.
    add_custom_target(fighter_assets)
endif()
add_dependencies(${PROJECT_NAME} fighter_assets)

# ---------------------------------------------------------------------------
# Packaging: `cmake --install` / `cpack` produce a folder you can zip and send.
# Everything goes into the "game" component so install rules from fetched
# dependencies (miniaudio ships headers + libs) stay out of the package.
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION . COMPONENT game)
install(DIRECTORY assets/
    DESTINATION assets
    COMPONENT game
    # Authoring sources and previews; the runtime only loads the .obj/.hdr files.
    PATTERN "*.blend" EXCLUDE
    PATTERN "*.glb" EXCLUDE
    PATTERN "*.ibl" EXCLUDE
    PATTERN "*preview*" EXCLUDE
)
install(FILES packaging/README.txt DESTINATION . COMPONENT game)

set(CPACK_PACKAGE_NAME "MissileSim")
set(CPACK_PACKAGE_VENDOR "MissileSim")
set(CPACK_PACKAGE_FILE_NAME "MissileSim-${PROJECT_VERSION}-win64")
set(CPACK_GENERATOR "ZIP")
# Wrap everything in one MissileSim-<version>-win64/ folder inside the zip.
set(CPACK_COMPONENT_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
set(CPACK_COMPONENTS_ALL game)
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
include(CPack)
