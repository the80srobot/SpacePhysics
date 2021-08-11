add_library(
    test_matchers
)

target_link_libraries(
    test_matchers
    gtest_main
    gmock_main
    geometry
)
