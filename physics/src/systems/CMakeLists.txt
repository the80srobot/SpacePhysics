# Component Data Definitions

add_library(
    components
    component_data.cc
)

# Glue System

add_library(
    glue_system
    glue_system.cc
)

# Motion System

add_library(
    motion_system
    motion_system.cc
)

target_link_libraries(
    motion_system
    geometry
    components
    absl::span
)

add_executable(
    motion_system_test
    motion_system_test.cc
)

target_link_libraries(
    motion_system_test
    motion_system
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
