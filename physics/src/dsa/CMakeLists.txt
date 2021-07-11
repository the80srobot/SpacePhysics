# IntervalTree

add_library(
    interval_tree
    interval_tree.cc
)

target_link_libraries(
    interval_tree
    absl::status
    absl::statusor
)

add_executable(
    interval_tree_test
    interval_tree_test.cc
)

target_link_libraries(
    interval_tree_test
    interval_tree
    gtest_main
    gmock_main
)

add_executable(
    interval_tree_benchmark
    interval_tree_benchmark.cc
)

target_link_libraries(
    interval_tree_benchmark
    interval_tree
    benchmark::benchmark
    absl::status
    absl::statusor
)
