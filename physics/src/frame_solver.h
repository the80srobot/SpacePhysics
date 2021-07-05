// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_FRAME_SOLVER
#define VSTR_FRAME_SOLVER

#include <iostream>

#include "collision_system.h"
#include "component_data.h"
#include "glue_system.h"
#include "motion_system.h"
#include "orbit_system.h"

namespace vstr {

struct Frame {
  // Core components:
  std::vector<Position> positions;
  std::vector<Mass> mass;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Glue> glue;
  std::vector<Flags> flags;

  // Optional components:
  std::vector<Input> input;
  std::vector<Orbit> orbits;
};

class FrameSolver {
 public:
  void Step(float dt, int frame_no, Frame &frame,
            std::vector<Event> &out_events);

 private:
  MotionSystem motion_system_;
  CollisionSystem collision_system_;
  GlueSystem glue_system_;
  OrbitSystem orbit_system_;
  std::vector<Collision> collision_events_;
};

}  // namespace vstr

#endif