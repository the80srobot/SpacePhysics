# Component Data Definitions

add_library(
    components
    component_data.cc
)

target_link_libraries(
    components
    absl::span
)

# Glue System

add_library(
    glue_system
    glue_system.cc
)

target_link_libraries(
    glue_system
    geometry
    components
)

# Orbit System

add_library(
    orbit_system
    kepler.cc
)

target_link_libraries(
    orbit_system
    geometry
    components
)

# Motion System

add_library(
    motion
    motion.cc
)

target_link_libraries(
    motion
    geometry
    components
    absl::span
    absl::flat_hash_map
)

add_executable(
    motion_test
    motion_test.cc
)

target_link_libraries(
    motion_test
    motion
    gtest_main
    gmock_main
)

# Rockets

add_library(
    rocket
    rocket.cc
)

target_link_libraries(
    rocket
    geometry
    components
    absl::span
    absl::status
    absl::statusor
)

add_executable(
    rocket_test
    rocket_test.cc
)

target_link_libraries(
    rocket_test
    rocket
    gtest_main
    gmock_main
)

# Collision System

add_library(
    collision_system
    collision_system.cc
)

target_link_libraries(
    collision_system
    geometry
    components
)

add_executable(
    collision_system_test
    collision_system_test.cc
)

target_link_libraries(
    collision_system_test
    collision_system
    gtest_main
    gmock_main
)

add_executable(
    collision_system_benchmark
    collision_system_benchmark.cc
)

target_link_libraries(
    collision_system_benchmark
    collision_system
    benchmark::benchmark
)

add_library(
    rules
    rules.cc
)

target_link_libraries(
    rules
    components
    geometry
    absl::span
    absl::flat_hash_map
)

add_executable(
    rules_test
    rules_test.cc
)

target_link_libraries(
    rules_test
    rules
    gtest_main
    gmock_main
)
