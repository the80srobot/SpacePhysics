#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/float.h"
#include "geometry/vector3.h"

namespace vstr {

inline bool Vector3Eq(const Vector3 &a, const Vector3 &b,
                      const float epsilon = 0.005f) {
  return FloatEq(a.x, b.x, epsilon) && FloatEq(a.y, b.y, epsilon) &&
         FloatEq(a.z, b.z, epsilon);
}

MATCHER_P(Vector3ApproxEq, other, "") { return Vector3Eq(arg, other); }

MATCHER_P2(Vector3ApproxEq, other, epsilon, "") {
  return Vector3Eq(arg, other, epsilon);
}

}  // namespace vstr