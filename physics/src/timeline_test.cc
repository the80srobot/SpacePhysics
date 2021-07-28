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

bool FloatEq(const float x, const float y, const float epsilon = 0.005f) {
  return std::fabs(x - y) < epsilon;
}

MATCHER_P(Vector3ApproxEq, other, "") {
  return FloatEq(arg.x, other.x) && FloatEq(arg.y, other.y) &&
         FloatEq(arg.z, other.z);
}

MATCHER_P2(Vector3ApproxEq, other, epsilon, "") {
  return FloatEq(arg.x, other.x, epsilon) && FloatEq(arg.y, other.y, epsilon) &&
         FloatEq(arg.z, other.z, epsilon);
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

struct TestCase {
  const std::string comment;
  const int resolution;
  // The test will allocate a buffer of appropriate size.
  const std::vector<Timeline::Trajectory> input;
  const std::vector<std::vector<Vector3>> expect;
  const absl::StatusCode expectCode;
};

class QueryTest : public testing::TestWithParam<TestCase> {};

TEST_P(QueryTest, QueryTest) {
  std::vector<Position> positions{
      Position{Vector3{0, 100, 0}},
      Position{Vector3{0, 0, 0}},
      Position{Vector3{100, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{10000, 10000},
      Mass{},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
      Motion{},
  };
  std::vector<Collider> colliders{
      Collider{1, 1},
      Collider{1, 1},
      Collider{1, 1},
  };
  std::vector<Glue> glue{
      Glue{},
      Glue{},
      Glue{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
      Flags{},
  };

  // Allocate the storage for the Trajectory queries.
  std::vector<std::vector<Vector3>> storage;
  std::vector<Timeline::Trajectory> queries = GetParam().input;
  for (auto& query : queries) {
    storage.push_back(std::vector<Vector3>(query.buffer_sz));
    query.buffer = storage.back().data();
  }

  Frame initial_frame{positions, mass, motion, colliders, glue, flags};
  LayerMatrix matrix({{1, 1}});

  const float dt = 0.1;  // 10 FPS
  Timeline timeline(initial_frame, 0, matrix, dt, 30);

  // One-second 1 ms/s/s burn in the direction away from the attractor should
  // exactly cancel the gravitational pull for the fist 10 frames.
  timeline.InputEvent(0, 1.0f / dt, Event(0, Acceleration{Vector3{1, 0, 0}}));

  // Simulate 10 seconds = 100 frames.
  int frame_no = 0;
  for (float t = 0; t < 10; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  absl::Status status =
      timeline.Query(GetParam().resolution, absl::MakeSpan(queries));
  EXPECT_EQ(status.code(), GetParam().expectCode) << status;
  for (int i = 0; i < GetParam().expect.size(); ++i) {
    for (int j = 0; j < queries[i].buffer_sz; ++j) {
      EXPECT_THAT(queries[i].buffer[j],
                  Vector3ApproxEq(GetParam().expect[i][j], 0.1))
          << "query #" << i << " element #" << j;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    QueryTest, QueryTest,
    testing::Values(
        TestCase{
            "empty_query",
            1,
            std::vector<Timeline::Trajectory>{},
            std::vector<std::vector<Vector3>>{},
            absl::StatusCode::kOk,
        },
        TestCase{
            "object_1_position",
            2,
            std::vector<Timeline::Trajectory>{
                Timeline::Trajectory{
                    1,
                    0,
                    Timeline::Trajectory::kPosition,
                    5,
                },
            },
            std::vector<std::vector<Vector3>>{
                std::vector<Vector3>{
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                },
            },
            absl::StatusCode::kOk,
        },
        TestCase{
            "object_2_position_misaligned_query",
            2,
            std::vector<Timeline::Trajectory>{
                Timeline::Trajectory{
                    1,
                    1,
                    Timeline::Trajectory::kPosition,
                    5,
                },
            },
            std::vector<std::vector<Vector3>>{},
            absl::StatusCode::kInvalidArgument,
        },
        TestCase{
            "object_2_position_and_velocity",
            10,
            std::vector<Timeline::Trajectory>{
                Timeline::Trajectory{
                    2,
                    0,
                    static_cast<Timeline::Trajectory::Attribute>(
                        Timeline::Trajectory::kPosition |
                        Timeline::Trajectory::kVelocity),
                    10,
                },
            },
            std::vector<std::vector<Vector3>>{
                std::vector<Vector3>{
                    // These approximate values fall out of the equation of the
                    // falling body.
                    Vector3{100, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{99.5, 0, 0},
                    Vector3{-0.95, 0, 0},
                    Vector3{98, 0, 0},
                    Vector3{-2, 0, 0},
                    Vector3{95.6, 0, 0},
                    Vector3{-3, 0, 0},
                    Vector3{92, 0, 0},
                    Vector3{-4.1, 0, 0},
                },
            },
            absl::StatusCode::kOk,
        }),
    [](const testing::TestParamInfo<QueryTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace
}  // namespace vstr
