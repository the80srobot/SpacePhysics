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

inline bool IsDestroyed(const Entity id, const Frame &frame) {
  return id.Get(frame.flags).value & Flags::kDestroyed;
}

void HandleDestroy(Entity id, Frame &frame) {
  if (IsDestroyed(id, frame)) return;
  id.Get(frame.flags).value |= Flags::kDestroyed;
  if (id.Get(frame.flags).value & Flags::kReusable)
    ReleaseObject(id, frame.flags, frame.reuse_pools, frame.reuse_tags);
}

void HandleDamage(const Event &event, Frame &frame) {
  if (IsDestroyed(event.id, frame)) return;
  ssize_t idx = FindOptionalComponent(frame.durability, event.id);
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
        if (event.stick.parent_id != Entity::Nil()) {
          event.id.Get(frame.flags).value |= Flags::kGlued;
          event.id.Get(frame.glue).parent_id = event.stick.parent_id;
        } else {
          event.id.Get(frame.flags).value &= ~Flags::kGlued;
          event.id.Get(frame.glue).parent_id = Entity::Nil();
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
        event.id.Get(frame.transforms).position =
            event.teleportation.new_position;
        event.id.Get(frame.motion).new_position =
            event.teleportation.new_position;
        event.id.Get(frame.motion).velocity = event.teleportation.new_velocity;
        event.id.Get(frame.motion).spin = event.teleportation.new_spin;
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