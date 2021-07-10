// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TIMELINE
#define VSTR_TIMELINE

#include <vector>

#include "component_data.h"
#include "frame_solver.h"

namespace vstr {

struct KeyFrame {
  Frame frame;
  // Events sorted by time.
  std::vector<Event> events;
};

class Timeline {
 public:
  Timeline(const Frame &frame, int head, LayerMatrix collision_matrix,
           float frame_time = 1.0f / 60, int key_frame_period = 30,
           MotionSystem::Integrator integrator = MotionSystem::kVelocityVerlet)
      : frame_time_(frame_time),
        key_frame_period_(key_frame_period),
        frame_solver_(collision_matrix, integrator),
        key_frames_{KeyFrame{frame, {}}},
        head_(head),
        tail_(head) {}

 private:
  float frame_time_;
  int key_frame_period_;
  int head_;
  int tail_;
  int frame_;
  std::vector<KeyFrame> key_frames_;
  FrameSolver frame_solver_;
};

}  // namespace vstr

#endif
