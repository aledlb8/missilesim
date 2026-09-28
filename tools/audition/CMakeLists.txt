# Offline audition renderer: plays scripted scenarios (launches, flybys,
# explosions, chase cameras, headset cues) through the audio engine and writes
# one WAV per scenario. Usage: AudioAudition [out-dir] [filter] [reverb] [trace]
if(MSVC)
    string(REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

add_executable(AudioAudition AuditionMain.cpp)
target_link_libraries(AudioAudition PRIVATE missilesim_audio)
if(MSVC)
    target_compile_options(AudioAudition PRIVATE /W4)
else()
    target_compile_options(AudioAudition PRIVATE -Wall -Wextra)
endif()
