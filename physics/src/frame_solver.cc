#include "frame_solver.h"

#include "vector3.h"

namespace vstr {
namespace {

// Compute forces
}

void FrameSolver::Step(const float dt, Frame &frame,
                       std::vector<Event> &out_events) {
  std::vector<Input> input{};
  // TODO: compute effective mass
  // TODO: compute orbits
  motion_system_.FirstPass(dt, input, frame);
  glue_system_.Step(frame);
  collision_system_.Solve(frame, dt, collision_events_);
  // TODO: Apply the results of collisions?
  motion_system_.SecondPass(frame);
}

}  // namespace vstr