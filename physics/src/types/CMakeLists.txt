add_library(
    components
    required_components.cc
    optional_components.cc
    events.cc
)

target_link_libraries(
    components
    absl::span
)
