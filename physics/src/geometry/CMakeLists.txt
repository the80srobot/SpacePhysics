add_library(
    geometry
    bvh.cc
    aabb.cc
    vector3.cc
    layer_matrix.cc
)

add_executable(
    bvh_test
    bvh_test.cc
)

target_link_libraries(
    bvh_test
    geometry
    gtest_main
    gmock_main
)

add_executable(
    bvh_benchmark
    bvh_benchmark.cc
)

target_link_libraries(
    bvh_benchmark
    geometry
    benchmark::benchmark
)
