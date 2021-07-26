// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "timeline.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "systems/component_data.h"

namespace vstr {
namespace {

TEST(TimelineTest, FallingSphere) {
  const float dt = 0.001;
  // The spheres should take about 111 seconds to come into contact.
  const float duration = 111;

  std::vector<Position> positions{
      Position{Vector3{0, 100, 0}},
      Position{Vector3{0, 200, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{100, 100},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
  };
  std::vector<Collider> colliders{
      Collider{1, 1},
      Collider{1, 1},
  };
  std::vector<Glue> glue{
      Glue{},
      Glue{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  Frame initial_frame{positions, mass, motion, colliders, glue, flags};
  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, dt, 30);

  int frame_no = 0;
  for (float t = 0; t < duration; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  // At this point we should be in the same state, at head frame, as the
  // FallingSphere PipelineTest.
  const Frame* frame = timeline.GetFrame(frame_no);
  EXPECT_NE(frame, nullptr);
  EXPECT_LT(frame->positions[0].value.y, 200);
  EXPECT_GT(frame->positions[0].value.y, 199);

  // A collision event should have been recorded by now.
  std::vector<Event> buffer;
  EXPECT_TRUE(timeline.GetEvents(frame_no, buffer));
  ASSERT_GE(buffer.size(), 1);
  EXPECT_EQ(buffer[0].type, Event::kCollision);
  EXPECT_EQ(buffer[0].id, 0);
  EXPECT_EQ(buffer[0].collision.second_id, 1);
  EXPECT_NE(buffer[0].collision.first_frame_offset_seconds, 0);

  // Simulate some way into the future, then look back.
  for (float t = 0; t < duration; t += dt) {
    timeline.Simulate();
  }

  frame = timeline.GetFrame(frame_no);
  EXPECT_NE(frame, nullptr);
  EXPECT_LT(frame->positions[0].value.y, 200);
  EXPECT_GT(frame->positions[0].value.y, 199);
}

bool FloatEq(const float x, const float y) {
  constexpr float kEpsilon = 0.005f;
  return std::fabs(x - y) < kEpsilon;
}

MATCHER_P(Vector3ApproxEq, other, "") {
  return FloatEq(arg.x, other.x) && FloatEq(arg.y, other.y) &&
         FloatEq(arg.z, other.z);
}

TEST(TimelineTest, AccelerateRewindAccelerate) {
  const float dt = 0.01;

  std::vector<Position> positions{
      Position{Vector3{0, 100, 0}},
      Position{Vector3{0, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
  };
  std::vector<Collider> colliders{
      Collider{1, 1},
      Collider{1, 1},
  };
  std::vector<Glue> glue{
      Glue{},
      Glue{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  Frame initial_frame{positions, mass, motion, colliders, glue, flags};
  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, dt, 30);

  // One-second 10 ms/s/s burn in the direction of sphere 1. After 1 second, the
  // speed of sphere 0 should be 10 m/s.
  timeline.InputEvent(0, 1.0f / dt, Event(0, Acceleration{Vector3{0, -10, 0}}));

  // After two seconds, sphere 0 should be on its way towards sphere 1.
  int frame_no = 0;
  for (float t = 0; t < 2; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  const Frame* frame = timeline.GetFrame(frame_no);
  ASSERT_NE(frame, nullptr);
  EXPECT_THAT(frame->motion[0].velocity, Vector3ApproxEq(Vector3{0, -10, 0}));

  // Rewind the clock to 0.5 second and burn in the opposite direction. The
  // resulting speed should be 0.
  timeline.InputEvent(0.5f / dt, 1.0f / dt,
                      Event(0, Acceleration{Vector3{0, 10, 0}}));
  frame_no = 0.5f / dt;
  for (float t = 0.5; t < 2; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  frame = timeline.GetFrame(frame_no);
  ASSERT_NE(frame, nullptr);
  EXPECT_THAT(frame->motion[0].velocity, Vector3ApproxEq(Vector3{0, 0, 0}));
}

TEST(TimelineTest, DestroyAttractor) {
  const float dt = 1.0f / 30;

  std::vector<Position> positions{
      Position{Vector3{0, 100, 0}},
      Position{Vector3{0, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{100, 100},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
  };
  std::vector<Collider> colliders{
      Collider{1, 1},
      Collider{1, 1},
  };
  std::vector<Glue> glue{
      Glue{},
      Glue{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  Frame initial_frame{positions, mass, motion, colliders, glue, flags};
  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, dt, 30);

  timeline.InputEvent(30.0f / dt, Event(1, SetDestroyed{true}));

  int frame_no = 0;
  for (float t = 0; t < 40.0f; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  // The massive sphere should stop existing after 30 seconds. After that, gone
  // should be the gravitational force, and no collision should occur.
  const Frame* frame = timeline.GetFrame(30.0f / dt);
  ASSERT_NE(frame, nullptr);
  EXPECT_FALSE(frame->flags[1].value & Flags::kDestroyed);
  EXPECT_THAT(
      frame->motion[0].acceleration,
      Vector3ApproxEq(Vector3{
          0,
          -100 / Vector3::SqrMagnitude(positions[0].value - positions[1].value),
          0}));

  frame = timeline.GetFrame(30.0f / dt + 1);
  ASSERT_NE(frame, nullptr);
  EXPECT_TRUE(frame->flags[1].value & Flags::kDestroyed);
  EXPECT_EQ(frame->motion[0].acceleration, (Vector3{0, 0, 0}));

  // Eventually sphere 0 will fall where sphere 1 used to be.
  for (; frame->positions[0].value.y > 1; ++frame_no) {
    timeline.Simulate();
    frame = timeline.GetFrame(frame_no);
  }

  // But no collision should be recorded.
  frame = timeline.GetFrame(frame_no);
  ASSERT_NE(frame, nullptr);
  EXPECT_GT(frame->positions[0].value.y, 0);
  EXPECT_LT(frame->positions[0].value.y, 1);
  std::vector<Event> buffer;
  EXPECT_TRUE(timeline.GetEvents(frame_no, buffer));
  EXPECT_EQ(buffer.size(), 0);

  // Undestroying the sphere should trigger a collision due to overlap.
  timeline.InputEvent(frame_no + 1, Event(1, SetDestroyed{false}));
  timeline.Simulate();
  ++frame_no;
  EXPECT_TRUE(timeline.GetEvents(frame_no, buffer));
  EXPECT_GE(buffer.size(), 1);
  EXPECT_EQ(buffer[0].type, Event::kCollision);
  EXPECT_EQ(buffer[0].id, 0);
  EXPECT_EQ(buffer[0].collision.second_id, 1);
}

}  // namespace
}  // namespace vstr
