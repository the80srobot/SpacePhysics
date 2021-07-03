#include "glue_system.h"

#include "component_data.h"

namespace vstr {
namespace {

void FollowParents(Frame &frame) {
  const int count = frame.positions.size();
  for (int i = 0; i < count; ++i) {
    const int parent_id = frame.glue[i].parent_id;
    if (parent_id < 0) continue;
    frame.motion[i].velocity = frame.motion[parent_id].velocity;
    frame.motion[i].new_position =
        frame.motion[parent_id].new_position +
        (frame.positions[i].value - frame.positions[parent_id].value);
  }
}

}  // namespace

void GlueSystem::Step(Frame &frame) { FollowParents(frame); }

}  // namespace vstr