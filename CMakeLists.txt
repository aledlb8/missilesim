cmake_minimum_required(VERSION 3.10)
project(MissileSimOpenGL)

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

# Project source files
file(GLOB_RECURSE SOURCES
    "src/*.cpp"
)

# Include directories for our project
set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/application
    ${CMAKE_CURRENT_SOURCE_DIR}/src/physics
    ${CMAKE_CURRENT_SOURCE_DIR}/src/physics/forces
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objects
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering
    ${CMAKE_CURRENT_SOURCE_DIR}/src/audio
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE glfw glad::glad glm::glm imgui miniaudio)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    MISSILESIM_SOURCE_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets"
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${INCLUDES}
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/audio
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/missile.obj
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/missile.obj
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/models/jet.obj
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/models/jet.obj
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/audio/explosion.wav
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/audio/explosion.wav
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/audio/fire_launch.wav
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/audio/fire_launch.wav
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/audio/jet_afterburner_loop.wav
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/audio/jet_afterburner_loop.wav
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/audio/missile_afterburner_loop.wav
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/audio/missile_afterburner_loop.wav
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/audio/missile_flight_loop.wav
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/audio/missile_flight_loop.wav
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/assets/assets.md
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets/assets.md
)
