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
#include "systems/motion.h"
#include "systems/orbit_system.h"

namespace vstr {

struct Frame {
  // Core components. Point mass moves clumsily, goes fast.
  std::vector<Position> positions;
  std::vector<Mass> mass;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Glue> glue;
  std::vector<Flags> flags;

  // Optional components:
  std::vector<Orbit> orbits;
};

}  // namespace vstr
#endif