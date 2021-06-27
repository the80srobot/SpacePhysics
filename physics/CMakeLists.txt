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

    # target_link_libraries(mytest GTest::gtest GTest::gmock GTest::gmock_main)


    enable_testing()

    add_executable(
        interval_tree_test
        src/interval_tree_test.cc
        src/interval_tree.cc
    )

    target_link_libraries(
        interval_tree_test
        gtest_main
        gmock_main
    )

    gtest_discover_tests(interval_tree_test)
endif(BUILD_TESTS)
