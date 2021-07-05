// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "glue_system.h"

#include "component_data.h"

namespace vstr {
namespace {

void FollowParents(const std::vector<Position> &positions,const std::vector<Glue> &glue, std::vector<Motion> &motion) {
  const int count = positions.size();
  for (int i = 0; i < count; ++i) {
    const int parent_id = glue[i].parent_id;
    if (parent_id < 0) continue;
    motion[i].velocity = motion[parent_id].velocity;
    motion[i].new_position = motion[parent_id].new_position +
                             (positions[i].value - positions[parent_id].value);
  }
}

}  // namespace

void GlueSystem::Step(const std::vector<Position> &positions,const std::vector<Glue> &glue,
                      std::vector<Motion> &motion) {
  FollowParents(positions, glue, motion);
}

}  // namespace vstr