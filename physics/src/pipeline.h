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
#include "systems/kepler.h"
#include "systems/motion.h"
#include "systems/rules.h"

namespace vstr {

class Pipeline {
 public:
  explicit Pipeline(LayerMatrix collision_matrix,
                    IntegrationMethod integrator = kVelocityVerlet)
      : collision_system_(collision_matrix), integrator_(integrator) {}

  explicit Pipeline(LayerMatrix collision_matrix, const RuleSet &rule_set,
                    IntegrationMethod integrator = kVelocityVerlet)
      : collision_system_(collision_matrix),
        integrator_(integrator),
        rule_set_(rule_set) {}

  void Step(float dt, int frame_no, Frame &frame, absl::Span<Event> input,
            std::vector<Event> &out_events);
  void Replay(float dt, int frame_no, Frame &frame, absl::Span<Event> events);

  inline CollisionSystem &collision_system() { return collision_system_; }

 private:
  IntegrationMethod integrator_;
  CollisionSystem collision_system_;
  GlueSystem glue_system_;
  RuleSet rule_set_;

  std::vector<Event> event_buffer_;
};

}  // namespace vstr

#endif