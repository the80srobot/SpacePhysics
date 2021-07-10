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

void FrameSolver::Step(const float dt, const int frame_no, Frame &frame,
                       const std::vector<Input> &input,
                       std::vector<Event> &out_events) {
  orbit_system_.Step(dt * frame_no, frame.positions, frame.orbits,
                     frame.motion);
  // TODO: compute effective mass
  motion_system_.FirstPass(dt, input, frame.positions, frame.mass, frame.flags,
                           frame.motion);
  glue_system_.Step(frame.positions, frame.glue, frame.motion);

  collision_system_.Solve(frame.positions, frame.colliders, frame.motion,
                          frame.flags, frame.glue, dt, out_events);
  // TODO: Should collisions be processed here, first?
  motion_system_.SecondPass(frame.motion, frame.positions);
}

}  // namespace vstr