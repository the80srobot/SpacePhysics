// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TEST_MATCHERS_VECTOR3
#define VSTR_TEST_MATCHERS_VECTOR3

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/float.h"
#include "geometry/vector3.h"

namespace vstr {

MATCHER_P(Vector3ApproxEq, other, "") {
  return Vector3::Approximately(arg, other);
}

MATCHER_P2(Vector3ApproxEq, other, epsilon, "") {
  return Vector3::Approximately(arg, other, epsilon);
}

}  // namespace vstr

#endif