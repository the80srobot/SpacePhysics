// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "pipeline.h"

#include <algorithm>

#include "geometry/vector3.h"
#include "systems/event_effects.h"
#include "systems/object_pool.h"
#include "systems/rocket.h"

namespace vstr {

void Pipeline::Step(const float dt, const int frame_no, Frame &frame,
                    absl::Span<Event> input, std::vector<Event> &out_events) {
  // The frame pipeline is as follows:
  //
  // 0) Convert SpawnAttempt events to Spawns <- SKIPPED ON REPLAY
  // 1) Compute closed-form orbital motion
  // 2) Compute acceleration from rockets
  // 3) Compute forces from acceleration input and gravity, from them velocities
  // 4) Compute motion of glued objects
  // 5) Detect collisions <- SKIPPED ON REPLAY
  // 6) Convert collision events to their effects <- SKIPPED ON REPLAY
  // 7) Apply computed velocities and update positions
  // 8) Apply events, including effects of collisions

  ConvertSpawnAttempts(input, out_events, frame);
  UpdateOrbitalMotion(dt * frame_no, frame.transforms, frame.orbits,
                      frame.motion);

  auto status =
      ConvertRocketBurnToAcceleration(dt, input, frame.mass, frame.rockets);
  assert(status.ok());

  // The motion system wants input events sorted by ID.
  std::sort(input.begin(), input.end(),
            [](const Event &a, const Event &b) -> bool { return a.id < b.id; });
  IntegrateMotion(integrator_, dt, input, frame.transforms, frame.mass,
                  frame.flags, frame.motion);

  // TODO: apply glue motion

  collision_detector_.DetectCollisions(frame.transforms, frame.colliders,
                                       frame.motion, frame.flags, frame.glue,
                                       dt, out_events);

  // convert collision events to effects
  rule_set_.Apply(frame.transforms, frame.mass, frame.motion, frame.colliders,
                  frame.triggers, out_events);

  UpdatePositions(dt, frame.motion, frame.flags, frame.transforms);
  ApplyEventEffects(input, frame);
  ApplyEventEffects(absl::MakeSpan(out_events), frame);
}

void Pipeline::Replay(const float dt, const int frame_no, Frame &frame,
                      absl::Span<Event> events) {
  UpdateOrbitalMotion(dt * frame_no, frame.transforms, frame.orbits,
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
                  frame.transforms, frame.mass, frame.flags, frame.motion);

  UpdatePositions(dt, frame.motion, frame.flags, frame.transforms);
  ApplyEventEffects(events, frame);
}

}  // namespace vstr