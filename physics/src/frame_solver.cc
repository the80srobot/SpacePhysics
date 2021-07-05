// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "frame_solver.h"

#include "vector3.h"

namespace vstr {
namespace {

// Compute forces
}

void FrameSolver::Step(const float dt, Frame &frame,
                       std::vector<Event> &out_events) {
  // TODO: compute effective mass
  // TODO: compute orbits
  motion_system_.FirstPass(dt, frame.input, frame.positions, frame.mass,
                           frame.flags, frame.motion);
  glue_system_.Step(frame.positions, frame.glue, frame.motion);
  collision_system_.Solve(frame.positions, frame.colliders, frame.motion,
                          frame.flags, frame.glue, dt, collision_events_);
  // TODO: Apply the results of collisions?
  motion_system_.SecondPass(frame.motion, frame.positions);
}

}  // namespace vstr