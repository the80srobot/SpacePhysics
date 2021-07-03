// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "motion_system.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <random>

namespace vstr {
namespace {

struct GravityTestCase {
  const std::string comment;
  const int object_id;
  const Frame frame;
  const Vector3 expect_force;
  const std::vector<std::pair<int, Vector3>> expect_components;
};

class GravityTest : public testing::TestWithParam<GravityTestCase> {};

TEST_P(GravityTest, GravityTest) {
  std::vector<std::pair<int, Vector3>> components;
  Vector3 force = MotionSystem::GravityComponentsOn(
      GetParam().frame, GetParam().object_id, components);
  EXPECT_EQ(force, GetParam().expect_force);
  EXPECT_THAT(components,
              testing::UnorderedElementsAreArray(GetParam().expect_components));
}

INSTANTIATE_TEST_SUITE_P(
    GravityTest, GravityTest,
    testing::Values(GravityTestCase{
        "one_attractor",
        0,
        Frame{
            std::vector<Position>{
                Position{Vector3{0, 100, 0}},
                Position{Vector3{0, 0, 0}},
            },
            std::vector<Mass>{
                Mass{},
                Mass{100, 100},
            },
            std::vector<Acceleration>{
                Acceleration{},
                Acceleration{},
            },
            std::vector<Motion>{
                Motion{},
                Motion{},
            },
            std::vector<Collider>{
                Collider{},
                Collider{},
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
        Vector3{0, -100.0f / (100 * 100), 0},
        std::vector<std::pair<int, Vector3>>{
            std::make_pair(1, Vector3{0, -100.0f / (100 * 100), 0}),
        },
    }),
    [](const testing::TestParamInfo<GravityTest::ParamType>& tc) {
      return tc.param.comment;
    });

struct MotionTestCase {
  const std::string comment;
  const std::vector<Input> input;
  const MotionSystem::Integrator integrator;
  const float deltaTime;
  const int rounds;
  const Frame frame;
  const Frame expect;
};

class MotionSystemTest : public testing::TestWithParam<MotionTestCase> {};

// TODO: this turns out to be a really annoying way to test this system.
// Narrower tests will probably be better.
TEST_P(MotionSystemTest, MotionSystemTest) {
  MotionSystem system(GetParam().integrator);
  Frame frame(GetParam().frame);
  for (int i = 0; i < GetParam().rounds; ++i) {
    system.FirstPass(GetParam().deltaTime, GetParam().input, frame);
    system.SecondPass(frame);
  }
  EXPECT_THAT(frame.positions,
              testing::ElementsAreArray(GetParam().expect.positions));
  EXPECT_THAT(frame.mass, testing::ElementsAreArray(GetParam().expect.mass));
  EXPECT_THAT(frame.acceleration,
              testing::ElementsAreArray(GetParam().expect.acceleration));
  EXPECT_THAT(frame.motion,
              testing::ElementsAreArray(GetParam().expect.motion));
  EXPECT_THAT(frame.colliders,
              testing::ElementsAreArray(GetParam().expect.colliders));
  EXPECT_THAT(frame.glue, testing::ElementsAreArray(GetParam().expect.glue));
  EXPECT_THAT(frame.destroyed,
              testing::ElementsAreArray(GetParam().expect.destroyed));
}

INSTANTIATE_TEST_SUITE_P(
    MotionSystemTest, MotionSystemTest,
    testing::Values(MotionTestCase{
        "euler_falling_in_vaccum_frame_1",
        std::vector<Input>{},
        MotionSystem::kFirstOrderEuler,
        1.0,
        1,
        Frame{
            std::vector<Position>{
                Position{Vector3{0, 100, 0}},
                Position{Vector3{0, 0, 0}},
            },
            std::vector<Mass>{
                Mass{},
                Mass{100, 100},
            },
            std::vector<Acceleration>{
                Acceleration{},
                Acceleration{},
            },
            std::vector<Motion>{
                Motion{},
                Motion{},
            },
            std::vector<Collider>{
                Collider{},
                Collider{},
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
        Frame{
            std::vector<Position>{
                Position{Vector3{0, 100 - 100.0f / (100 * 100), 0}},
                Position{Vector3{0, 0, 0}},
            },
            std::vector<Mass>{
                Mass{},
                Mass{100, 100},
            },
            std::vector<Acceleration>{
                Acceleration{Vector3{0, -100.0f / (100 * 100), 0}},
                Acceleration{},
            },
            std::vector<Motion>{
                Motion{
                    Vector3{0, -100.0f / (100 * 100), 0},
                    Vector3{0, 100 - 100.0f / (100 * 100), 0},
                },
                Motion{},
            },
            std::vector<Collider>{
                Collider{},
                Collider{},
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
    }),
    [](const testing::TestParamInfo<MotionSystemTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace
}  // namespace vstr