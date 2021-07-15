// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_FRAME_SOLVER
#define VSTR_FRAME_SOLVER

#include <absl/types/span.h>

#include <iostream>

#include "frame.h"
#include "systems/collision_system.h"
#include "systems/component_data.h"
#include "systems/glue_system.h"
#include "systems/motion_system.h"
#include "systems/orbit_system.h"

namespace vstr {

class FrameSolver {
 public:
  explicit FrameSolver(
      LayerMatrix collision_matrix,
      MotionSystem::Integrator integrator = MotionSystem::kVelocityVerlet)
      : collision_system_(collision_matrix), motion_system_(integrator) {}
  void Step(float dt, int frame_no, Frame &frame, absl::Span<const Input> input,
            std::vector<Event> &out_events);

 private:
  MotionSystem motion_system_;
  CollisionSystem collision_system_;
  GlueSystem glue_system_;
  OrbitSystem orbit_system_;
};

}  // namespace vstr

#endif