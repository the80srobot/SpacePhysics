// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_FRAME
#define VSTR_FRAME

#include <absl/types/span.h>

#include <iostream>

#include "systems/collision_system.h"
#include "systems/component_data.h"
#include "systems/glue_system.h"
#include "systems/kepler.h"
#include "systems/motion.h"

namespace vstr {

struct Frame {
  // Core components. Point mass moves clumsily, goes fast.
  std::vector<Transform> positions;
  std::vector<Mass> mass;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Glue> glue;
  std::vector<Flags> flags;

  // Optional components:
  std::vector<Orbit> orbits;
  std::vector<Durability> durability;
  std::vector<Rocket> rockets;
};

}  // namespace vstr
#endif