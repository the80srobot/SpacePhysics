// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "geometry/quaternion.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "test_matchers/quaternion.h"

namespace vstr {
namespace {

TEST(QuaternionTest, FromAngle) {
  // 180 degrees around principal axes
  EXPECT_THAT(Quaternion::FromAngle({1, 0, 0}, M_PI),
              QuaternionApproxEq(Quaternion{1, 0, 0, 0}));
  EXPECT_THAT(Quaternion::FromAngle({0, 1, 0}, M_PI),
              QuaternionApproxEq(Quaternion{0, 1, 0, 0}));

  // 90 degrees
  EXPECT_THAT(Quaternion::FromAngle({0, 1, 0}, M_PI / 2),
              QuaternionApproxEq(Quaternion{
                  0, std::sinf(90 / 2 * Quaternion::kRadiansPerDeg), 0,
                  std::cosf(90 / 2 * Quaternion::kRadiansPerDeg)}));
}

TEST(QuaternionTest, FromEuler) {
  EXPECT_THAT(Quaternion::FromEulerZXY({M_PI / 2, M_PI / 2, 0}),
              QuaternionApproxEq(Quaternion{0.5, 0.5, -0.5, 0.5}));
  EXPECT_THAT(Quaternion::FromEulerZXY({M_PI / 4, M_PI / 4, 0}),
              QuaternionApproxEq(
                  Quaternion{0.3535534, 0.3535534, -0.1464466, 0.8535535}));
}

TEST(QuaternionTest, MultiplyQuaternion) {
  // Multiplicative identity, both as RHS and LHS, should have no effect.
  EXPECT_THAT(
      Quaternion::Identity() * Quaternion::FromAngle({1, 0, 0}, M_PI / 2),
      QuaternionApproxEq(Quaternion::FromAngle({1, 0, 0}, M_PI / 2)));
  EXPECT_THAT(
      Quaternion::FromAngle({1, 0, 0}, M_PI / 2) * Quaternion::Identity(),
      QuaternionApproxEq(Quaternion::FromAngle({1, 0, 0}, M_PI / 2)));

  // Two turns by 45 degrees should equate one turn by 90 degrees.
  EXPECT_THAT(Quaternion::FromAngle({1, 0, 0}, M_PI / 4) *
                  Quaternion::FromAngle({1, 0, 0}, M_PI / 4),
              QuaternionApproxEq(Quaternion::FromAngle({1, 0, 0}, M_PI / 2)));

  // Incremental turn.
  Quaternion q = Quaternion::Identity();
  Quaternion dq = Quaternion::FromAngle({1, 0, 0}, M_PI / 10);
  for (int i = 0; i < 10; ++i) {
    q *= dq;
  }
  EXPECT_THAT(q, QuaternionApproxEq(Quaternion::FromAngle({1, 0, 0}, M_PI)));
}

}  // namespace
}  // namespace vstr