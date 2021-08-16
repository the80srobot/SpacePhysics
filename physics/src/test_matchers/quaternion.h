// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TEST_MATCHERS_QUATERNION
#define VSTR_TEST_MATCHERS_QUATERNION

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/quaternion.h"

namespace vstr {

MATCHER_P(QuaternionApproxEq, other, "") {
  return Quaternion::Approximately(arg, other);
}

MATCHER_P2(QuaternionApproxEq, other, epsilon, "") {
  return Quaternion::Approximately(arg, other, epsilon);
}

}  // namespace vstr

#endif