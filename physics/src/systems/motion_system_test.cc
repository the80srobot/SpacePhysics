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
  // Point particle 0 of negligible mass is falling towards a massive point
  // particle 1 in a vaccum. They start out 100 meters apart and particle 1
  // weighs 100 kg. Note that the motion system sets G=1 for simplicity. (It's
  // actual value is 11 orders of magnitude less.)
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
  const float coarse_dt = 1;
  const float fine_dt = 0.001;
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

  for (float t = 0; t < time_to_fall; t += coarse_dt) {
    system.FirstPass(coarse_dt, {}, positions, mass, flags, motion);
    system.SecondPass(motion, positions);
  }

  // Integration in large steps should get within the ballpark.
  EXPECT_LT(positions[0].value.y, 20);
  EXPECT_GT(positions[0].value.y, 5);

  // Reset the position and motion.
  positions[0].value.y = 100;
  motion[0] = Motion{};

  // Run again in small steps.
  for (float t = 0; t < time_to_fall; t += fine_dt) {
    system.FirstPass(fine_dt, {}, positions, mass, flags, motion);
    system.SecondPass(motion, positions);
  }

  // This should still under-estimate velocities, but the error should be much
  // smaller.
  EXPECT_LT(positions[0].value.y, 1);
  EXPECT_GT(positions[0].value.y, 0);
}

TEST(MotionSystemTest, VerletHover) {
  // Point particle 0 of neglibile mass is hovering 100 meters over point
  // particle 1 which has 100 kg of mass. Input each frame sets acceleration of
  // point particle 0 to counteract the gravitational influence of particle 1.

  MotionSystem system(MotionSystem::kVelocityVerlet);
  const float dt = 0.001;
  const float duration = 100;

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

  // The acceleration due to gravity at point particle 0 is 100 / 100^2. The
  // inverse input should exactly counter.
  std::vector<Event> input{
      Event(0, Input{Vector3{0, 0.01, 0}}),
  };

  for (float f = 0; f < duration; f += dt) {
    system.FirstPass(dt, absl::MakeSpan(input), positions, mass, flags, motion);
    system.SecondPass(motion, positions);
  }

  EXPECT_EQ(positions[0].value.y, 100);

  // If we now also apply acceleration to particle 1, the force of gravity
  // acting on particle 0 should decrease and its own acceleration should allow
  // it to escape.
  input.push_back(Event(1, Input{Vector3{0, -0.01, 0}}));

  for (float f = 0; f < duration; f += dt) {
    system.FirstPass(dt, absl::MakeSpan(input), positions, mass, flags, motion);
    system.SecondPass(motion, positions);
  }

  EXPECT_GT(positions[0].value.y, 100);
  EXPECT_LT(positions[1].value.y, 0);
}

}  // namespace
}  // namespace vstr