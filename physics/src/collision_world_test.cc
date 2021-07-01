#include "collision_world.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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
  CollisionWorld world(GetParam().matrix);
  std::vector<CollisionEvent> events;
  world.Compute(GetParam().frame, GetParam().deltaTime, events);
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
        }),
    [](const testing::TestParamInfo<CollisionWorldTest::ParamType>& tc) {
      return tc.param.comment;
    });
}  // namespace
}  // namespace vstr
