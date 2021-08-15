// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar License:
// http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>
//
// This file provides basic quaternion math. This isn't a complete math library
// - we only define the limited operations needed for interactions with unity
// and to update object angular velocity between frames.

#ifndef VSTR_QUATERNION
#define VSTR_QUATERNION

#include <cmath>
#include <iostream>

#include "geometry/float.h"
#include "geometry/vector3.h"

namespace vstr {

// A quaternion is a compact way of representing orientation in 3D space. It
// requires storing only 4 floats, but is equally expressive to a 3x3 matrix.
//
// In this file:
//
// 1) All angles are in radians, unless noted otherwise.
// 2) Positive angles represent clockwise rotation when observed from the point
//    given by the axis vector towards {0, 0, 0}.
// 3) Unless states otherwise, all operations expect (but do not validate)
//    normalized quaternions.
struct Quaternion {
  // Per Wikipedia: a + bi + cj + dk, where i, j and k are orthogonal unit
  // vectors representing the principal axes. (In game development the real
  // numbers are sometimes called x, y, z and w.)
  //
  // The memory order is B, C, D, then A, for compatibility with Unity.
  float b;  // Unity calls this x, for relation to rotation around the X axis.
  float c;  // Unity component y.
  float d;  // Unity component z.
  float a;  // Unity component w (the scalar).

  // Constructs a quaternion to represent rotation by angle around the axis.
  static Quaternion FromAngle(Vector3 axis, float angle) {
    angle *= 0.5f;
    axis *= std::sinf(angle);
    return Quaternion{axis.x, axis.y, axis.z, std::cosf(angle)};
  }

  static Quaternion Identity() { return Quaternion{0, 0, 0, 1}; }

  // Returns a quaternion to represent an extrensic (using principal axes)
  // rotation around axes X, Y and Z in that order.
  //
  // THIS IS NOT THE ORDER USED IN UNITY - see FromEulerZXY.
  static Quaternion FromEulerXYZ(Vector3 euler);

  // Returns a quaternion to represent an extrensic (using principal axes)
  // rotation around axes Z, X and Y in that order.
  //
  // This order rotation is compatible with Unity.
  static Quaternion FromEulerZXY(Vector3 euler);

  static inline Quaternion FromEulerX(float angle);

  static inline Quaternion FromEulerY(float angle) {
    angle *= 0.5f;
    const float sin = std::sinf(angle);
    const float cos = std::cosf(angle);
    return Quaternion{0, sin, 0, cos};
  }

  static inline Quaternion FromEulerZ(float angle) {
    angle *= 0.5f;
    const float sin = std::sinf(angle);
    const float cos = std::cosf(angle);
    return Quaternion{0, 0, sin, cos};
  }

  static inline float Dot(const Quaternion x, const Quaternion y) {
    return x.a * y.a + x.b * y.b + x.c * y.c + x.d * y.d;
  }

  static inline Quaternion Normalize(Quaternion q) {
    float m = 1 / std::sqrtf(Dot(q, q));
    return Quaternion{q.a * m, q.b * m, q.c * m, q.d * m};
  }
};

Quaternion operator*(const Quaternion lhs, const Quaternion rhs);

Vector3 operator*(const Quaternion q, const Vector3 v);

std::ostream& operator<<(std::ostream& os, Quaternion q);

}  // namespace vstr

#endif
