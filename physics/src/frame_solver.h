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

namespace vstr {

class FrameSolver {
 public:
  void Step(float dt, Frame &frame, std::vector<Event> &out_events);

 private:
  MotionSystem motion_system_;
  CollisionSystem collision_system_;
  GlueSystem glue_system_;
  std::vector<Collision> collision_events_;
};

}  // namespace vstr

#endif