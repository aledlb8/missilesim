# No-GL flight checks: flies scripted mouse-aim and stick scenarios through
# the same Jet (airframe + fly-by-wire + instructor) the game uses and prints
# the handling numbers, so tuning is measured instead of eyeballed.
add_executable(flight_harness
    main.cpp
    HandlingChecks.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/F16Data.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/F16Airframe.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/FlightControl.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/MouseAimInstructor.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/Jet.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/CardAirframe.cpp
    ${CMAKE_SOURCE_DIR}/src/flight/AircraftCatalog.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/Atmosphere.cpp
    ${CMAKE_SOURCE_DIR}/src/objects/Fighter.cpp
    ${CMAKE_SOURCE_DIR}/src/objects/PhysicsObject.cpp)
target_include_directories(flight_harness PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(flight_harness PRIVATE glm::glm)
if(MSVC)
    target_compile_options(flight_harness PRIVATE /W4)
else()
    target_compile_options(flight_harness PRIVATE -Wall -Wextra)
endif()

add_test(NAME flight_handling COMMAND flight_harness --check)
