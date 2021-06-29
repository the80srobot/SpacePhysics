#ifndef VSTR_AABB
#define VSTR_AABB

#include <iostream>

#include "vector3.h"

namespace vstr {

struct AABB {
  Vector3 min;
  Vector3 max;

  explicit AABB() : min(Vector3()), max(Vector3()) {}
  AABB(Vector3 min, Vector3 max) : min(min), max(max) {}

  bool Overlaps(const AABB &other) const;
  void Encapsulate(const AABB &other);
  void Encapsulate(const Vector3 &other);
};

inline bool operator==(const AABB lhs, const AABB rhs) {
  return lhs.min == rhs.min && lhs.max == rhs.max;
};

std::ostream &operator<<(std::ostream &os, const AABB &aabb);
}  // namespace vstr

#endif