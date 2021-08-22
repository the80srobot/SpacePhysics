// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "pipeline.h"

#include <algorithm>

#include "geometry/vector3.h"
#include "systems/rocket.h"

namespace vstr {
namespace {

void ApplyEventEffects(absl::Span<Event> events, Frame &frame) {
  for (const auto &event : events) {
    switch (event.type) {
      case Event::kDestruction:
        if (event.destruction.value) {
          frame.flags[event.id].value |= Flags::kDestroyed;
        } else {
          frame.flags[event.id].value &= ~Flags::kDestroyed;
        }
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
        auto it = std::lower_bound(
            frame.durability.begin(), frame.durability.end(),
            Durability{event.id}, [](const Durability &a, const Durability &b) {
              return a.id < b.id;
            });
        if (it != frame.durability.end() && it->id == event.id) {
          it->value -= event.damage.value;
          if (it->value <= 0) {
            frame.flags[event.id].value |= Flags::kDestroyed;
          }
        }
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
        frame.positions[event.id].position = event.teleportation.new_position;
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
      default:
        break;
    }
  }
}

}  // namespace

void Pipeline::Step(const float dt, const int frame_no, Frame &frame,
                    absl::Span<Event> input, std::vector<Event> &out_events) {
  // The frame pipeline is as follows:
  //
  // 1) Compute closed-form orbital motion
  // 2) Compute acceleration from rockets
  // 3) Compute forces from acceleration input and gravity, from them velocities
  // 4) Compute motion of glued objects
  // 5) Detect collisions <- SKIPPED ON REPLAY
  // 6) Convert collision events to their effects <- SKIPPED ON REPLAY
  // 7) Apply computed velocities and update positions
  // 8) Apply events, including effects of collisions

  UpdateOrbitalMotion(dt * frame_no, frame.positions, frame.orbits,
                      frame.motion);

  auto status =
      ConvertRocketBurnToAcceleration(dt, input, frame.mass, frame.rockets);
  assert(status.ok());

  // The motion system wants input events sorted by ID.
  std::sort(input.begin(), input.end(),
            [](const Event &a, const Event &b) -> bool { return a.id < b.id; });
  IntegrateMotion(integrator_, dt, input, frame.positions, frame.mass,
                  frame.flags, frame.motion);

  // TODO: apply glue motion

  collision_detector_.DetectCollisions(frame.positions, frame.colliders,
                                       frame.motion, frame.flags, frame.glue,
                                       dt, out_events);

  // convert collision events to effects
  rule_set_.Apply(frame.positions, frame.mass, frame.motion, frame.colliders,
                  frame.triggers, out_events);

  UpdatePositions(dt, frame.motion, frame.flags, frame.positions);
  ApplyEventEffects(input, frame);
  ApplyEventEffects(absl::MakeSpan(out_events), frame);
}

void Pipeline::Replay(const float dt, const int frame_no, Frame &frame,
                      absl::Span<Event> events) {
  UpdateOrbitalMotion(dt * frame_no, frame.positions, frame.orbits,
                      frame.motion);

  auto status =
      ConvertRocketBurnToAcceleration(dt, events, frame.mass, frame.rockets);
  assert(status.ok());

  event_buffer_.clear();
  for (const auto &event : events) {
    if (event.type == Event::kAcceleration) event_buffer_.push_back(event);
  }
  std::sort(event_buffer_.begin(), event_buffer_.end(),
            [](const Event &a, const Event &b) -> bool { return a.id < b.id; });
  IntegrateMotion(integrator_, dt, absl::MakeSpan(event_buffer_),
                  frame.positions, frame.mass, frame.flags, frame.motion);

  UpdatePositions(dt, frame.motion, frame.flags, frame.positions);
  ApplyEventEffects(events, frame);
}

}  // namespace vstr