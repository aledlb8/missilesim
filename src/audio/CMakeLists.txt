# Physical audio engine + synthesis voices. A library so the simulator and the
# offline audition tool (tools/audition) render through exactly the same code.
#
# The engine runs a real-time DSP graph on the audio thread, and an
# unoptimised build cannot keep up with a busy engagement, so this library is
# optimised in every configuration. The run-time checks that MSVC adds to
# Debug builds are incompatible with optimisation; they are dropped here only
# (this directory scope does not leak into the rest of the project).
if(MSVC)
    foreach(flag /RTC1 /Od /Ob0)
        string(REPLACE "${flag}" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    endforeach()
    # Replaced by /W4 below.
    string(REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

file(GLOB_RECURSE AUDIO_ENGINE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/synth/*.cpp"
)
add_library(missilesim_audio STATIC ${AUDIO_ENGINE_SOURCES})
target_include_directories(missilesim_audio PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(missilesim_audio PUBLIC glm::glm miniaudio)

if(MSVC)
    target_compile_options(missilesim_audio PRIVATE /W4 $<$<CONFIG:Debug>:/O2 /Ob2>)
else()
    target_compile_options(missilesim_audio PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-O2>)
endif()
