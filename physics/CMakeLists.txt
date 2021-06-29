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

    add_executable(
        interval_tree_test
        src/interval_tree_test.cc
        src/interval_tree.cc
    )

    target_link_libraries(
        interval_tree_test
        gtest_main
        gmock_main
        absl::status
        absl::statusor
    )

    add_executable(
        bvh_test
        src/bvh_test.cc
        src/bvh.cc
        src/aabb.cc
        src/vector3.cc
    )

    target_link_libraries(
        bvh_test
        gtest_main
        gmock_main
    )

    add_executable(
        interval_tree_benchmark
        src/interval_tree_benchmark.cc
        src/interval_tree.cc
    )

    target_link_libraries(
        interval_tree_benchmark
        benchmark::benchmark
        absl::status
        absl::statusor
    )

    gtest_discover_tests(interval_tree_test)
endif(BUILD_TESTS)
