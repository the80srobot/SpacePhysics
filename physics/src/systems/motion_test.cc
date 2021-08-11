// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "motion.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <random>

#include "test_matchers/vector3.h"

namespace vstr {
namespace {

TEST(MotionTest, GravityForceOn) {
  std::vector<Position> positions{
      Position{Vector3{0, 100, 0}},
      Position{Vector3{0, 0, 0}},
      Position{Vector3{0, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{100, 100},
      Mass{100, 100},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
      Flags{Flags::kDestroyed},
  };

  std::vector<std::pair<int, Vector3>> contributions;
  Vector3 force = GravityForceOn(positions, mass, flags, 0, contributions);
  EXPECT_EQ(force, (Vector3{0, -100.0f / (100 * 100), 0}));
  EXPECT_THAT(contributions, testing::UnorderedElementsAre(std::make_pair(
                                 1, Vector3{0, -100.0f / (100 * 100), 0})));
}

// Tests that the Verlet velocity integrator takes velocity input.
TEST(MotionTest, ObjectStaysInMotion) {
  const float dt = 1.0f / 60;
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
      Motion{Vector3{0, 1, 0}},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  for (float t = 0; t < 100; t += dt) {
    IntegrateMotion(kVelocityVerlet, dt, {}, positions, mass, flags, motion);
    UpdatePositions(motion, positions);
  }

  EXPECT_GT(positions[1].value.y, 99.9);
  EXPECT_LT(positions[1].value.y, 100.1);
}

TEST(MotionTest, FallingPointMass) {
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
    IntegrateMotion(kVelocityVerlet, coarse_dt, {}, positions, mass, flags,
                    motion);
    UpdatePositions(motion, positions);
  }

  // Integration in large steps should get within the ballpark.
  EXPECT_LT(positions[0].value.y, 20);
  EXPECT_GT(positions[0].value.y, 5);

  // Reset the position and motion.
  positions[0].value.y = 100;
  motion[0] = Motion{};

  // Run again in small steps.
  for (float t = 0; t < time_to_fall; t += fine_dt) {
    IntegrateMotion(kVelocityVerlet, fine_dt, {}, positions, mass, flags,
                    motion);
    UpdatePositions(motion, positions);
  }

  // This should still under-estimate velocities, but the error should be much
  // smaller.
  EXPECT_LT(positions[0].value.y, 1);
  EXPECT_GT(positions[0].value.y, 0);
}

TEST(MotionTest, PointMassHover) {
  // Point particle 0 of neglibile mass is hovering 100 meters over point
  // particle 1 which has 100 kg of mass. Input each frame sets acceleration of
  // point particle 0 to counteract the gravitational influence of particle 1.

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
      Event(0, {}, Acceleration{Vector3{0, 0.01, 0}, Acceleration::kNone}),
  };

  for (float f = 0; f < duration; f += dt) {
    IntegrateMotion(kVelocityVerlet, dt, absl::MakeSpan(input), positions, mass,
                    flags, motion);
    UpdatePositions(motion, positions);
  }

  EXPECT_EQ(positions[0].value.y, 100);

  // If we now also apply acceleration to particle 1, the force of gravity
  // acting on particle 0 should decrease and its own acceleration should allow
  // it to escape.
  input.push_back(Event(1, {}, Acceleration{Vector3{0, -0.01, 0}}));

  for (float f = 0; f < duration; f += dt) {
    IntegrateMotion(kVelocityVerlet, dt, absl::MakeSpan(input), positions, mass,
                    flags, motion);
    UpdatePositions(motion, positions);
  }

  EXPECT_GT(positions[0].value.y, 100);
  EXPECT_LT(positions[1].value.y, 0);
}

TEST(MotionTest, ForceImpulse) {
  constexpr float dt = 1.0 / 60;

  std::vector<Position> positions{
      Position{Vector3{0, 100, 0}},
      Position{Vector3{0, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{100, 0},
      Mass{},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  for (float t = 0; t < 1; t += dt) {
    IntegrateMotion(kFirstOrderEuler, dt, {}, positions, mass, flags, motion);
    UpdatePositions(motion, positions);
  }

  EXPECT_EQ(positions[0].value, (Vector3{0, 100, 0}));

  std::vector<Event> input{
      Event(0, {},
            Acceleration{Vector3{0, 100, 0},
                         static_cast<Acceleration::Flag>(
                             Acceleration::kImpulse | Acceleration::kForce)}),
  };
  IntegrateMotion(kFirstOrderEuler, dt, absl::MakeSpan(input), positions, mass,
                  flags, motion);
  for (float t = 0; t < 10; t += dt) {
    UpdatePositions(motion, positions);
    IntegrateMotion(kFirstOrderEuler, dt, {}, positions, mass, flags, motion);
  }
  UpdatePositions(motion, positions);

  EXPECT_EQ(motion[0].velocity, (Vector3{0, 1, 0}));
  EXPECT_THAT(positions[0].value, Vector3ApproxEq(Vector3{0, 110, 0}, 0.1));
}

}  // namespace
}  // namespace vstr