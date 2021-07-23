// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "collision_system.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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
  EXPECT_THAT(events, testing::UnorderedElementsAreArray(GetParam().expect));
}

INSTANTIATE_TEST_SUITE_P(
    CollisionSystemTest, CollisionSystemTest,
    testing::Values(
        TestCase{
            "basic",
            1.0,
            std::vector<Position>{
                Position{Vector3{0, 0, 0}},
                Position{Vector3{10, 0, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{0, 0, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{},
                Flags{},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<Event>{
                Collision{0, 1, 0.9},
            },
        },
        TestCase{
            "fast_mover",
            1.0 / 60,
            std::vector<Position>{
                Position{Vector3{0, 0, 0}},
                Position{Vector3{10, 0, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{1000000, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{0, 0, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{false},
                Flags{false},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<Event>{
                Collision{0, 1, 0},
            },
        },
        TestCase{
            "both_fast_movers",
            1.0 / 60,
            std::vector<Position>{
                Position{Vector3{0, 0, 0}},
                Position{Vector3{10, 0, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{0, 0, 0},
                                                Vector3{1000000, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{10, 0, 0},
                                                Vector3{-1000000, 0, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{false},
                Flags{false},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<Event>{
                Collision{0, 1, 0},
            },
        },
        TestCase{
            "slow_orthogonal_movers_collide",
            1.0,
            std::vector<Position>{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{false},
                Flags{false},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<Event>{
                // At the time of collision, the line connecting the two centers
                // is the hypotenuse of an isosceles right triangle, with the
                // third pivot at {0,0,0}. Both sides are therefore:
                //
                //  1.0/sqrt(2) = 0
                //
                // It takes 1.0 seconds to travel the 10 units, leading to the
                // final formula.
                Collision{0, 1, 1.0f - (1.0f / std::sqrtf(2)) / 10.0f},
            },
        },
        TestCase{
            "fast_orthogonal_movers_collide",
            1.0 / 60,
            std::vector<Position>{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10000000, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10000000, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{false},
                Flags{false},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<Event>{
                Collision{0, 1, 0},
            },
        },
        TestCase{
            "destroyed_does_not_collide",
            1.0,
            std::vector<Position>{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{true},
                Flags{false},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<Event>{},
        },
        TestCase{
            "layer_mask_no_collision",
            1.0,
            std::vector<Position>{
                Position{Vector3{-10, 0, 0}},
                Position{Vector3{0, -10, 0}},
            },
            std::vector<Motion>{
                Motion::FromPositionAndVelocity(Vector3{-10, 0, 0},
                                                Vector3{10, 0, 0}),
                Motion::FromPositionAndVelocity(Vector3{0, -10, 0},
                                                Vector3{0, 10, 0}),
            },
            std::vector<Collider>{
                Collider{1, 0.5},
                Collider{1, 0.5},
            },
            std::vector<Glue>{
                Glue{-1},
                Glue{-1},
            },
            std::vector<Flags>{
                Flags{false},
                Flags{false},
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 2)}),
            std::vector<Event>{},
        }),
    [](const testing::TestParamInfo<CollisionSystemTest::ParamType>& tc) {
      return tc.param.comment;
    });
}  // namespace
}  // namespace vstr
