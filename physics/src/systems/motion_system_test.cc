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

  const std::vector<Position> positions;
  const std::vector<Mass> mass;
  const std::vector<Flags> flags;

  const Vector3 expect_force;
  const std::vector<std::pair<int, Vector3>> expect_components;
};

class GravityTest : public testing::TestWithParam<GravityTestCase> {};

TEST_P(GravityTest, GravityTest) {
  std::vector<std::pair<int, Vector3>> components;
  Vector3 force = MotionSystem::GravityComponentsOn(
      GetParam().positions, GetParam().mass, GetParam().flags,
      GetParam().object_id, components);
  EXPECT_EQ(force, GetParam().expect_force);
  EXPECT_THAT(components,
              testing::UnorderedElementsAreArray(GetParam().expect_components));
}

INSTANTIATE_TEST_SUITE_P(
    GravityTest, GravityTest,
    testing::Values(GravityTestCase{
        "one_attractor",
        0,
        std::vector<Position>{
            Position{Vector3{0, 100, 0}},
            Position{Vector3{0, 0, 0}},
        },
        std::vector<Mass>{
            Mass{},
            Mass{100, 100},
        },
        std::vector<Flags>{
            Flags{},
            Flags{},
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
  const std::vector<Event> input;
  const MotionSystem::Integrator integrator;
  const float deltaTime;
  const int rounds;

  const std::vector<Position> positions;
  const std::vector<Mass> mass;
  const std::vector<Motion> motion;
  const std::vector<Flags> flags;

  const std::vector<Position> expect_positions;
  const std::vector<Motion> expect_motion;
};

TEST(MotionSystemTest, VerletFalling) {
  // Massless point particle 0 is falling towards a massive point particle 1 in
  // a vaccum. They start out 100 meters apart and particle 1 weighs 100 kg.
  // Note that the motion system sets G=1 for simplicity. (It's actual value is
  // 11 orders of magnitude less.)
  //
  // It should take t = ((pi/2) / sqrt(2(m1 + m2))) * r^(1.5) to close the
  // distance, or about 111 seconds. (Can be derived from more general forms
  // such as
  // https://en.wikipedia.org/wiki/Radial_trajectory#Elliptic_trajectory)
  //
  // Any discrete integration of motion is inaccurate. Verlet integration
  // over-estimates the time needed to fall by a distance, but the error should
  // be smaller with smaller steps.

  MotionSystem system(MotionSystem::kVelocityVerlet);
  const float large_step_dt = 1;
  const float small_step_dt = 0.001;
  const float time_to_fall = 111;

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
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  for (float f = 0; f < time_to_fall; f += large_step_dt) {
    system.FirstPass(large_step_dt, {}, positions, mass, flags, motion);
    system.SecondPass(motion, positions);
  }

  // Integration in large steps should get within the ballpark.
  EXPECT_LT(positions[0].value.y, 20);
  EXPECT_GT(positions[0].value.y, 0);

  // Reset the position and motion.
  positions[0].value.y = 100;
  motion[0] = Motion{};

  // Run again in small steps.
  for (float f = 0; f < time_to_fall; f += small_step_dt) {
    system.FirstPass(small_step_dt, {}, positions, mass, flags, motion);
    system.SecondPass(motion, positions);
  }

  // This should still under-estimate velocities, but the error should be much
  // smaller.
  EXPECT_LT(positions[0].value.y, 1);
  EXPECT_GT(positions[0].value.y, 0);
}

}  // namespace
}  // namespace vstr