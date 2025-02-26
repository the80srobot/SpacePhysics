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

# Object Pools

add_library(
    object_pool
    object_pool.cc
)

target_link_libraries(
    object_pool
    components
    frame
    absl::status
    absl::statusor
)

add_executable(
    object_pool_test
    object_pool_test.cc
)

target_link_libraries(
    object_pool_test
    geometry
    object_pool
    gtest_main
    gmock_main
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

# Event effects & handlers

add_library(
    event_effects
    event_effects.cc
)

target_link_libraries(
    event_effects
    object_pool
    components
    absl::span
)

# Collision System

add_library(
    collision_detector
    collision_detector.cc
)

target_link_libraries(
    collision_detector
    geometry
    components
)

add_executable(
    collision_detector_test
    collision_detector_test.cc
)

target_link_libraries(
    collision_detector_test
    collision_detector
    gtest_main
    gmock_main
)

add_executable(
    collision_detector_benchmark
    collision_detector_benchmark.cc
)

target_link_libraries(
    collision_detector_benchmark
    collision_detector
    benchmark::benchmark
)

add_library(
    collision_rule_set
    collision_rule_set.cc
)

target_link_libraries(
    collision_rule_set
    components
    geometry
    absl::span
    absl::flat_hash_map
)

add_executable(
    collision_rule_set_test
    collision_rule_set_test.cc
)

target_link_libraries(
    collision_rule_set_test
    collision_rule_set
    gtest_main
    gmock_main
)
