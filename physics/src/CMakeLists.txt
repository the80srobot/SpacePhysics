add_subdirectory(dsa)
add_subdirectory(geometry)
add_subdirectory(systems)

# Foreign Function Interface

add_library(
    vstr_c_api
    c_api.cc
)

target_link_libraries(
    vstr_c_api
    frame_solver
)

# Frame Solver

add_library(
    frame_solver
    frame_solver.cc
)

target_link_libraries(
    frame_solver
    motion_system
    collision_system
    glue_system
)
