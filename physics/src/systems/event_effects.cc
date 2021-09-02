// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "event_effects.h"

#include "systems/object_pool.h"
#include "systems/rocket.h"

namespace vstr {
namespace {

void HandleDestroy(int32_t id, Frame &frame) {
  frame.flags[id].value |= Flags::kDestroyed;
  if (frame.flags[id].value & Flags::kReusable)
    ReleaseObject(id, frame.flags, frame.reuse_pools, frame.reuse_tags);
}

void HandleDamage(const Event &event, Frame &frame) {
  int32_t idx = FindOptionalComponent(frame.durability, event.id);
  if (idx >= 0) {
    frame.durability[idx].value -= event.damage.value;
    if (frame.durability[idx].value <= 0) HandleDestroy(event.id, frame);
  }
}

}  // namespace

void ApplyEventEffects(absl::Span<Event> events, Frame &frame) {
  for (const auto &event : events) {
    switch (event.type) {
      case Event::kDestruction:
        HandleDestroy(event.id, frame);
        break;
      case Event::kStick:
        if (event.stick.parent_id >= 0) {
          frame.flags[event.id].value |= Flags::kGlued;
          frame.glue[event.id].parent_id = event.stick.parent_id;
        } else {
          frame.flags[event.id].value &= ~Flags::kGlued;
          frame.glue[event.id].parent_id = 0;
        }
        break;
      case Event::kDamage: {
        HandleDamage(event, frame);
        break;
      }
      case Event::kAcceleration:
        // Nothing to do, acceleration was already used for motion integration.
        break;
      case Event::kCollision:
        // Nothing to do here - collisions effects are already included as other
        // events.
        break;
      case Event::kTeleportation:
        frame.transforms[event.id].position = event.teleportation.new_position;
        frame.motion[event.id].new_position = event.teleportation.new_position;
        frame.motion[event.id].velocity = event.teleportation.new_velocity;
        frame.motion[event.id].spin = event.teleportation.new_spin;
        break;
      case Event::kRocketBurn:
      // Nothing to do - already handled before motion.
      case Event::kRocketRefuel: {
        auto status = ApplyRocketRefuel(event, frame.mass, frame.rockets);
        assert(status.ok());
        break;
      }
      case Event::kSpawn: {
        SpawnObject(event, frame);
        break;
      }
      default:
        break;
    }
  }
}

}  // namespace vstr