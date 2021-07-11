cmake_minimum_required(VERSION 3.14)

project(VSTRPhysics)


# GoogleTest requires C++11, but we like std::optional and friends.
set(CMAKE_CXX_STANDARD 17)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

include_directories(src)
add_subdirectory(src)

option(BUILD_TESTS "Build tests" ON)

if(BUILD_TESTS)
    include(FetchContent)

    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/355d57d90d9744c41ac7c99f1e960778f1c63040.zip
    )
    FetchContent_MakeAvailable(googletest)
    include(GoogleTest)
    enable_testing()

    FetchContent_Declare(
        benchmark
        URL https://github.com/google/benchmark/archive/1799e1b9ec02a111c9a1ce79b10c9233dd60db75.zip
    )
    FetchContent_MakeAvailable(benchmark)
    
    FetchContent_Declare(
        absl
        URL https://github.com/abseil/abseil-cpp/archive/a2419e63b8ae3b924152822f3c9f9da67ff6e215.zip
    )
    FetchContent_MakeAvailable(absl)

    # Foreign Function Interface

    add_library(
        vstr_c_api
        src/c_api.cc
    )

    target_link_libraries(
        vstr_c_api
        frame_solver
    )

    # Frame Solver

    add_library(
        frame_solver
        src/frame_solver.cc
    )

    target_link_libraries(
        frame_solver
        motion_system
        collision_system
        glue_system
    )

    # Glue System

    add_library(
        glue_system
        src/glue_system.cc
    )

    # MotionSystem

    add_library(
        motion_system
        src/motion_system.cc
    )

    target_link_libraries(
        motion_system
        geometry
        components
        absl::span
    )

    add_executable(
        motion_system_test
        src/motion_system_test.cc
    )

    target_link_libraries(
        motion_system_test
        motion_system
        gtest_main
        gmock_main
    )

    # CollisionSystem

    add_library(
        collision_system
        src/collision_system.cc
    )

    target_link_libraries(
        collision_system
        bvh
        components
    )

    add_executable(
        collision_system_test
        src/collision_system_test.cc
    )

    target_link_libraries(
        collision_system_test
        collision_system
        gtest_main
        gmock_main
    )

    add_executable(
        collision_system_benchmark
        src/collision_system_benchmark.cc
    )

    target_link_libraries(
        collision_system_benchmark
        collision_system
        benchmark::benchmark
    )

    # IntervalTree

    add_library(
        interval_tree
        src/interval_tree.cc
    )

    target_link_libraries(
        interval_tree
        absl::status
        absl::statusor
    )

    add_executable(
        interval_tree_test
        src/interval_tree_test.cc
    )

    target_link_libraries(
        interval_tree_test
        interval_tree
        gtest_main
        gmock_main
    )

    add_executable(
        interval_tree_benchmark
        src/interval_tree_benchmark.cc
    )

    target_link_libraries(
        interval_tree_benchmark
        interval_tree
        benchmark::benchmark
        absl::status
        absl::statusor
    )

    # BVH

    add_library(
        bvh
        src/bvh.cc
    )

    target_link_libraries(
        bvh
        geometry
    )

    add_executable(
        bvh_test
        src/bvh_test.cc
    )

    target_link_libraries(
        bvh_test
        bvh
        gtest_main
        gmock_main
    )

    add_executable(
        bvh_benchmark
        src/bvh_benchmark.cc
    )

    target_link_libraries(
        bvh_benchmark
        bvh
        benchmark::benchmark
    )

    # Definitions

    add_library(
        components
        src/component_data.cc
    )

    # Geometry

    add_library(
        geometry
        src/aabb.cc
        src/vector3.cc
        src/layer_matrix.cc
    )

    gtest_discover_tests(interval_tree_test)
endif(BUILD_TESTS)
