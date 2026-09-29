# No-GL check that the Fox 2 drag scale still reproduces the AIM-9B manual
# ranges. A unit error (feet kept as metres, lbf kept as newtons) fails it.
add_executable(fox2_kinematics
    main.cpp
    ${CMAKE_SOURCE_DIR}/src/sim/Fox2Catalog.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/Atmosphere.cpp)
target_include_directories(fox2_kinematics PRIVATE ${CMAKE_SOURCE_DIR}/src)
if(MSVC)
    target_compile_options(fox2_kinematics PRIVATE /W4)
else()
    target_compile_options(fox2_kinematics PRIVATE -Wall -Wextra)
endif()
