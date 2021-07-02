#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "collision_solver.h"

namespace vstr {
namespace {

struct TestCase {
  const std::string comment;
  const float deltaTime;
  const Frame frame;
  const LayerMatrix matrix;
  const std::vector<CollisionEvent> expect;
};

class CollisionWorldTest : public testing::TestWithParam<TestCase> {};

TEST_P(CollisionWorldTest, CollisionWorldTest) {
  CollisionSolver world(GetParam().matrix);
  std::vector<CollisionEvent> events;
  world.Solve(GetParam().frame, GetParam().deltaTime, events);
  EXPECT_THAT(events, testing::UnorderedElementsAreArray(GetParam().expect));
}

INSTANTIATE_TEST_SUITE_P(
    CollisionWorldTest, CollisionWorldTest,
    testing::Values(
        TestCase{
            "basic",
            1.0,
            Frame{
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
                std::vector<Destroyed>{
                    Destroyed{false},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{
                CollisionEvent{0, 1, 0.9},
            },
        },
        TestCase{
            "fast_mover",
            1.0 / 60,
            Frame{
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
                std::vector<Destroyed>{
                    Destroyed{false},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{
                CollisionEvent{0, 1, 0},
            },
        },
        TestCase{
            "both_fast_movers",
            1.0 / 60,
            Frame{
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
                std::vector<Destroyed>{
                    Destroyed{false},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{
                CollisionEvent{0, 1, 0},
            },
        },
        TestCase{
            "slow_orthogonal_movers_collide",
            1.0,
            Frame{
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
                std::vector<Destroyed>{
                    Destroyed{false},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{
                // At the time of collision, the line connecting the two centers
                // is the hypotenuse of an isosceles right triangle, with the
                // third pivot at {0,0,0}. Both sides are therefore:
                //
                //  1.0/sqrt(2) = 0
                //
                // It takes 1.0 seconds to travel the 10 units, leading to the
                // final formula.
                CollisionEvent{0, 1, 1.0f - (1.0f / std::sqrtf(2)) / 10.0f},
            },
        },
        TestCase{
            "fast_orthogonal_movers_collide",
            1.0 / 60,
            Frame{
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
                std::vector<Destroyed>{
                    Destroyed{false},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{
                CollisionEvent{0, 1, 0},
            },
        },
        TestCase{
            "destroyed_does_not_collide",
            1.0,
            Frame{
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
                std::vector<Destroyed>{
                    Destroyed{true},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{},
        },
        TestCase{
            "layer_mask_no_collision",
            1.0,
            Frame{
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
                    Collider{2, 0.5},
                },
                std::vector<Glue>{
                    Glue{-1},
                    Glue{-1},
                },
                std::vector<Destroyed>{
                    Destroyed{false},
                    Destroyed{false},
                },
            },
            LayerMatrix(std::vector<std::pair<uint32_t, uint32_t>>{
                std::make_pair(1, 1)}),
            std::vector<CollisionEvent>{},
        }),
    [](const testing::TestParamInfo<CollisionWorldTest::ParamType>& tc) {
      return tc.param.comment;
    });
}  // namespace
}  // namespace vstr
