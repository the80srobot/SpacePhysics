// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "glue_system.h"

namespace vstr {

void GlueSystem::UpdateGluedMotion(const std::vector<Transform> &positions,
                                   const std::vector<Glue> &glue,
                                   const std::vector<Flags> &flags,
                                   std::vector<Motion> &motion) {
  const int count = positions.size();
  for (size_t i = 0; i < count; ++i) {
    const Entity parent_id = glue[i].parent_id;
    if ((flags[i].value & Flags::kGlued) == 0) continue;
    motion[i].velocity = parent_id.Get(motion).velocity;
    motion[i].new_position =
        parent_id.Get(motion).new_position +
        (positions[i].position - parent_id.Get(positions).position);
  }
}

}  // namespace vstr
