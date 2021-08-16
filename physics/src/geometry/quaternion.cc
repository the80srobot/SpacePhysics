// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar License:
// http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "quaternion.h"

namespace vstr {

Quaternion Quaternion::FromEulerXYZ(Vector3 euler) {
  const float x_sin = std::sinf(euler.x * 0.5f);
  const float x_cos = std::cosf(euler.x * 0.5f);
  const float y_sin = std::sinf(euler.y * 0.5f);
  const float y_cos = std::cosf(euler.y * 0.5f);
  const float z_sin = std::sinf(euler.z * 0.5f);
  const float z_cos = std::cosf(euler.z * 0.5f);
  return Quaternion{
      x_sin * y_cos * z_cos - y_sin * z_sin * x_cos,
      y_sin * x_cos * z_cos + x_sin * z_sin * y_cos,
      z_sin * x_cos * y_cos - x_sin * y_sin * z_cos,
      x_cos * y_cos * z_cos + y_sin * z_sin * x_sin,
  };
}

Quaternion Quaternion::FromEulerZXY(Vector3 euler) {
  const float x_sin = std::sinf(euler.x * 0.5f);
  const float x_cos = std::cosf(euler.x * 0.5f);
  const float y_sin = std::sinf(euler.y * 0.5f);
  const float y_cos = std::cosf(euler.y * 0.5f);
  const float z_sin = std::sinf(euler.z * 0.5f);
  const float z_cos = std::cosf(euler.z * 0.5f);
  return Quaternion{
      x_sin * y_cos * z_cos + y_sin * z_sin * x_cos,
      y_sin * x_cos * z_cos - x_sin * z_sin * y_cos,
      z_sin * x_cos * y_cos - x_sin * y_sin * z_cos,
      x_cos * y_cos * z_cos + y_sin * z_sin * x_sin,
  };
}

Quaternion operator*(const Quaternion lhs, const Quaternion rhs) {
  return Quaternion{
      lhs.a * rhs.b + lhs.b * rhs.a + lhs.c * rhs.d - lhs.d * rhs.c,
      lhs.a * rhs.c + lhs.c * rhs.a + lhs.d * rhs.b - lhs.b * rhs.d,
      lhs.a * rhs.d + lhs.d * rhs.a + lhs.b * rhs.c - lhs.c * rhs.b,
      lhs.a * rhs.a - lhs.b * rhs.b - lhs.c * rhs.c - lhs.d * rhs.d,
  };
}

Quaternion Quaternion::Normalize(Quaternion q) {
  float m = 1 / std::sqrtf(Dot(q, q));
  return Quaternion{q.a * m, q.b * m, q.c * m, q.d * m};
}

Vector3 operator*(const Quaternion q, const Vector3 v) {
  // Method 1: based Wikipedia reference / naive algorithm

  //   float b = q.b * 2;
  //   float c = q.c * 2;
  //   float d = q.d * 2;
  //   float bb = q.b * b;
  //   float cc = q.c * c;
  //   float dd = q.d * d;
  //   float bc = q.b * c;
  //   float bd = q.b * d;
  //   float cd = q.c * d;
  //   float ab = q.a * b;
  //   float ac = q.a * c;
  //   float ad = q.a * d;

  //   Vector3 result;
  //   result.x = (1.0f - (cc + dd)) * v.x + (bc - ad) * v.y + (bd + ac) * v.z;
  //   result.y = (bc + ad) * v.x + (1.0f - (bb + dd)) * v.y + (cd - ab) * v.z;
  //   result.z = (bd - ac) * v.x + (cd + ab) * v.y + (1.0f - (bb + cc)) * v.z;
  //   return result;

  // Method 2: well-known in the demoscene, of unclear origin:

  //   const Vector3 t = 2 * Vector3::Cross(Vector3{q.b, q.c, q.d}, v);
  //   return v + q.a * t + Vector3::Cross(Vector3{q.b, q.c, q.d}, t);

  // Method 3: derived here:
  // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion

  const Vector3 u{q.b, q.c, q.d};
  return 2 * Vector3::Dot(u, v) * u + (q.a * q.a - Vector3::Dot(u, u)) * v +
         2 * q.a * Vector3::Cross(u, v);
}

Quaternion Quaternion::Interpolate(Quaternion a, Quaternion b, const float t) {
  float dt = Dot(a, b);
  float angle = std::acosf(dt);
  float s = 1 / std::sqrtf(1 - dt * dt);
  float w1 = sin(angle * (1.0f - t)) * s;
  float w2 = sin(angle * t) * s;
  return Quaternion{
      a.b * w1 + b.b * w2,
      a.c * w1 + b.c * w2,
      a.d * w1 + b.d * w2,
      a.a * w1 + b.a * w2,
  };
}

std::ostream &operator<<(std::ostream &os, Quaternion q) {
  return os << "Quaternion{" << q.b << ", " << q.c << ", " << q.d
            << ", /*scalar=*/" << q.a << "}";
}

}  // namespace vstr