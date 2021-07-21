// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "pipeline.h"

#include "geometry/vector3.h"

namespace vstr {
namespace {

// Compute forces
}

void Pipeline::Step(const float dt, const int frame_no, Frame &frame,
                    absl::Span<Event> input, std::vector<Event> &out_events) {
  orbit_system_.Step(dt * frame_no, frame.positions, frame.orbits,
                     frame.motion);
  // TODO: compute effective mass
  std::sort(input.begin(), input.end(),
            [](const Event &a, const Event &b) -> bool { return a.id < b.id; });
  motion_system_.FirstPass(dt, input, frame.positions, frame.mass, frame.flags,
                           frame.motion);
  // glue_system_.Step(frame.positions, frame.glue, frame.motion);

  collision_system_.Solve(frame.positions, frame.colliders, frame.motion,
                          frame.flags, frame.glue, dt, out_events);
  // TODO: Should collisions be processed here, first?
  motion_system_.SecondPass(frame.motion, frame.positions);
}

void Pipeline::Replay(const float dt, const int frame_no, Frame &frame,
                      absl::Span<Event> events) {
  orbit_system_.Step(dt * frame_no, frame.positions, frame.orbits,
                     frame.motion);
  // TODO: compute effective mass

  // The motion system just wants input events sorted by ID.
  event_buffer_.clear();
  for (const auto &event : events) {
    if (event.type == Event::kInput) event_buffer_.push_back(event);
  }
  std::sort(event_buffer_.begin(), event_buffer_.end(),
            [](const Event &a, const Event &b) -> bool { return a.id < b.id; });
  motion_system_.FirstPass(dt, absl::MakeSpan(event_buffer_), frame.positions,
                           frame.mass, frame.flags, frame.motion);

  glue_system_.Step(frame.positions, frame.glue, frame.motion);

  // HERE RESOLVE COLLISION EVENTS

  motion_system_.SecondPass(frame.motion, frame.positions);
}

}  // namespace vstr