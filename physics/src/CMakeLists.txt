add_subdirectory(dsa)
add_subdirectory(geometry)
add_subdirectory(systems)
add_subdirectory(types)

# Foreign Function Interface

add_library(
    vstr_c_api SHARED
    c_api.cc
)

target_link_libraries(
    vstr_c_api
    timeline
)

set_target_properties(vstr_c_api PROPERTIES
    VERSION 1
    SOVERSION 1
    PUBLIC_HEADER c_api.h)

# Timeline

add_library(
    timeline
    timeline.cc
)

target_link_libraries(
    timeline
    pipeline
    interval_tree
    object_pool
    absl::status
    absl::statusor
    absl::strings
    absl::span
)

add_executable(
    timeline_test
    timeline_test.cc
)

target_link_libraries(
    timeline_test
    timeline
    gtest_main
    gmock_main
    absl::flat_hash_map
)

add_executable(
    timeline_benchmark
    timeline_benchmark.cc
)

target_link_libraries(
    timeline_benchmark
    timeline
    benchmark::benchmark
)

# Frame Solver

add_library(
    pipeline
    pipeline.cc
)

target_link_libraries(
    pipeline
    motion
    rocket
    frame
    event_effects
    orbit_system
    collision_detector
    glue_system
    collision_rule_set
)

add_executable(
    pipeline_test
    pipeline_test.cc
)

target_link_libraries(
    pipeline_test
    pipeline
    gtest_main
    gmock_main
)

add_executable(
    pipeline_benchmark
    pipeline_benchmark.cc
)

target_link_libraries(
    pipeline_benchmark
    pipeline
    benchmark::benchmark
)
