# Headless engagement harness: runs seeded scenarios through sim::World (the
# same code the game steps), prints their event traces and checks the
# simulation contracts: reproducible traces, independent shots, once-only
# outcomes, swept collisions, safe removal, step-boundary flare births,
# player damage, launch refusals and config validation.
add_executable(engagement_harness
    main.cpp
    Scenarios.cpp
    EngagementChecks.cpp)
target_link_libraries(engagement_harness PRIVATE missilesim_core)
if(MSVC)
    target_compile_options(engagement_harness PRIVATE /W4)
else()
    target_compile_options(engagement_harness PRIVATE -Wall -Wextra)
endif()

add_test(NAME engagement_checks COMMAND engagement_harness --check)
