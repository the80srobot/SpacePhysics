add_library(
    geometry
    bvh.cc
    aabb.cc
    vector3.cc
    quaternion.cc
    layer_matrix.cc
)

add_executable(
    geometry_test
    bvh_test.cc
    quaternion_test.cc
)

target_link_libraries(
    geometry_test
    geometry
    gtest_main
    gmock_main
)

add_executable(
    geometry_benchmark
    bvh_benchmark.cc
)

target_link_libraries(
    geometry_benchmark
    geometry
    benchmark::benchmark
)
