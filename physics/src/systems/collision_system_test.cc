// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "collision_system.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "test_matchers/events.h"

namespace vstr {
namespace {

struct TestCase {
  const std::string comment;
  const float deltaTime;
  const std::vector<Position> positions;
  const std::vector<Motion> motion;
  const std::vector<Collider> colliders;
  const std::vector<Glue> glue;
  const std::vector<Flags> flags;
  const LayerMatrix matrix;
  const std::vector<Event> expect;
};

class CollisionSystemTest : public testing::TestWithParam<TestCase> {};

TEST_P(CollisionSystemTest, CollisionSystemTest) {
  CollisionSystem system(GetParam().matrix);
  std::vector<Event> events;
  system.DetectCollisions(GetParam().positions, GetParam().colliders,
                          GetParam().motion, GetParam().flags, GetParam().glue,
                          GetParam().deltaTime, events);

  EXPECT_THAT(events,
              testing::Pointwise(EventMatches(0.005f), GetParam().expect));
}

INSTANTIATE_TEST_SUITE_P(
    CollisionSystemTest, CollisionSystemTest,
    testing::Values(
        TestCase{
            .comment{"basic"},
            .deltaTime = 1.0,
            .positions{
                Position{Vector3{0, 0, 0}},
                Position{Vector3{10, 0, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{0, 0, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            .matrix{LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)})},
            .expect{
                Event(Vector3{9.5, 0, 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds = 0.9}),
            },
        },
        TestCase{
            .comment{"off_center"},
            .deltaTime = 1.0,
            .positions{
                Position{Vector3{0, -10, 0}},
                Position{Vector3{20, 0, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{0, 0, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 10, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{-10, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            .matrix{LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)})},
            .expect{
                Event(Vector3{9.5, 0, 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds = 0.9}),
            },
        },
        TestCase{
            .comment{"fast_mover"},
            .deltaTime = 1.0 / 60,
            .positions{
                Position{Vector3{0, 0, 0}},
                Position{Vector3{10, 0, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{1000000, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{0, 0, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            .expect{
                Event(Vector3{9.5, 0, 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds = 0}),
            },
        },
        TestCase{
            .comment{"both_fast_movers"},
            .deltaTime = 1.0 / 60,
            .positions{
                Position{Vector3{0, 0, 0}},
                Position{Vector3{10, 0, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{1000000, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{-1000000, 0, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            .expect{
                Event(Vector3{5, 0, 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds = 0}),
            },
        },
        TestCase{
            .comment{"slow_orthogonal_movers_collide"},
            .deltaTime = 1.0,
            .positions{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            .expect{
                // At the time of collision, the line connecting the two centers
                // is the hypotenuse of an isosceles right triangle, with the
                // third pivot at {0,0,0}. Both sides are therefore:
                //
                //  1.0/sqrt(2) = 0
                //
                // It takes 1.0 seconds to travel the 10 units, leading to the
                // final formula.
                Event(Vector3{-1.0f / (2 * std::sqrtf(2)),
                              -1.0f / (2 * std::sqrtf(2)), 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds =
                                    1.0f - (1.0f / std::sqrtf(2)) / 10.0f}),
            },
        },
        TestCase{
            .comment{"fast_orthogonal_movers_collide"},
            .deltaTime = 1.0 / 60,
            .positions{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10000000, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10000000, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            .expect{
                Event(Vector3{-1.0f / (2 * std::sqrtf(2)),
                              -1.0f / (2 * std::sqrtf(2)), 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds = 0}),
            },
        },
        TestCase{
            .comment{"destroyed_does_not_collide"},
            .deltaTime = 1.0,
            .positions{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{Flags::kDestroyed},
                Flags{0},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            .expect{},
        },
        TestCase{
            .comment{"layer_mask_no_collision"},
            .deltaTime = 1.0,
            .positions{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10, 0}),
            },
            .colliders{
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
                Collider{.layer = 1, .radius = 0.5, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            .matrix{LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 2)})},
            .expect{},
        },
        TestCase{
            .comment{"unequal_radii"},
            .deltaTime = 1.0,
            .positions{
                {{-10, 0, 0}},
                {{10, 0, 0}},
            },
            .motion{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{-10, 0, 0}),
            },
            .colliders{
                {.layer = 1, .radius = 1, .center{0, 0, 0}},
                {.layer = 1, .radius = 9, .center{0, 0, 0}},
            },
            .glue{
                Glue{-1},
                Glue{-1},
            },
            .flags{
                Flags{0},
                Flags{0},
            },
            .matrix{LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)})},
            .expect{
                Event(Vector3{-4, 0, 0},
                      Collision{.first_id = 0,
                                .second_id = 1,
                                .first_frame_offset_seconds = 0.5}),
            },
        }),
    [](const testing::TestParamInfo<CollisionSystemTest::ParamType>& tc) {
      return tc.param.comment;
    });
}  // namespace
}  // namespace vstr
