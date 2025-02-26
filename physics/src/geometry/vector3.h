// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

// This file provides basic 3D vector math.

#ifndef VSTR_VECTOR3
#define VSTR_VECTOR3

#include <cmath>
#include <compare>
#include <iostream>

#include "geometry/float.h"

namespace vstr {

struct Vector3 {
  float x;
  float y;
  float z;

  static Vector3 Zero() { return Vector3{0, 0, 0}; }

  static inline float Dot(const Vector3 x, const Vector3 y) {
    return x.x * y.x + x.y * y.y + x.z * y.z;
  }

  static inline float Magnitude(const Vector3 v) {
    return std::sqrt(Dot(v, v));
  }

  static inline float SqrMagnitude(const Vector3 v) { return Dot(v, v); }

  static inline Vector3 Sqrt(const Vector3 v) {
    return Vector3{std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z)};
  }

  static inline Vector3 Rsqrt(const Vector3 v) {
    return Vector3{1.0f / std::sqrt(v.x), 1.0f / std::sqrt(v.y),
                   1.0f / std::sqrt(v.z)};
  }

  static inline Vector3 Normalize(const Vector3 v) {
    const float m = (1.0f / std::sqrt(Vector3::Dot(v, v)));
    return Vector3{v.x * m, v.y * m, v.z * m};
  }

  static inline Vector3 Max(const Vector3 a, const Vector3 b) {
    return Vector3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
  }

  static inline Vector3 Min(const Vector3 a, const Vector3 b) {
    return Vector3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
  }

  static inline bool Approximately(const Vector3& a, const Vector3& b,
                                   const float epsilon = 0.005f) {
    return FloatEq(a.x, b.x, epsilon) && FloatEq(a.y, b.y, epsilon) &&
           FloatEq(a.z, b.z, epsilon);
  }

  static inline Vector3 Cross(const Vector3 a, const Vector3 b) {
    return Vector3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
  }

  std::partial_ordering operator<=>(const Vector3&) const = default;
};

static_assert(std::is_standard_layout<Vector3>());
static_assert(std::is_trivial<Vector3>());

inline Vector3 operator-(const Vector3 v) { return Vector3{-v.x, -v.y, -v.z}; }

inline Vector3 operator*(const Vector3 lhs, const Vector3 rhs) {
  return Vector3{lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
}

inline Vector3 operator+(const Vector3 lhs, const Vector3 rhs) {
  return Vector3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline void operator+=(Vector3& lhs, const Vector3 rhs) {
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  lhs.z += rhs.z;
}

inline Vector3 operator/(const Vector3 lhs, const Vector3 rhs) {
  return Vector3{lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z};
}

inline Vector3 operator-(const Vector3 lhs, const Vector3 rhs) {
  return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

inline Vector3 operator*(const Vector3 lhs, float rhs) {
  return Vector3{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
}

inline Vector3 operator*(const float lhs, const Vector3 rhs) {
  return Vector3{rhs.x * lhs, rhs.y * lhs, rhs.z * lhs};
}

inline void operator*=(Vector3& lhs, const float rhs) {
  lhs.x *= rhs;
  lhs.y *= rhs;
  lhs.z *= rhs;
}

inline Vector3 operator/(const Vector3 lhs, const float rhs) {
  return Vector3{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
}

inline void operator/=(Vector3& lhs, const float rhs) {
  lhs.x /= rhs;
  lhs.y /= rhs;
  lhs.z /= rhs;
}

std::ostream& operator<<(std::ostream& os, Vector3 v);

}  // namespace vstr

#endif