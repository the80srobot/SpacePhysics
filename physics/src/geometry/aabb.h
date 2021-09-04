// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_AABB
#define VSTR_AABB

#include <compare>
#include <iostream>

#include "geometry/vector3.h"

namespace vstr {

// Axis-aligned bounding box. Used to quickly check that two volumes definitely
// don't intersect.
struct AABB {
  Vector3 min;
  Vector3 max;

  explicit AABB() : min(Vector3()), max(Vector3()) {}
  AABB(const Vector3 min, const Vector3 max) : min(min), max(max) {}

  static AABB FromCenterAndExtents(const Vector3 center, const Vector3 extents);
  static AABB FromCenterAndHalfExtents(const Vector3 center,
                                       const Vector3 half_extents);

  bool Overlaps(const AABB &other) const;
  void Encapsulate(const AABB &other);
  void Encapsulate(const Vector3 &other);
  void Sweep(const Vector3 &motion);

  auto operator<=>(const AABB &) const = default;
};

std::ostream &operator<<(std::ostream &os, const AABB &aabb);
}  // namespace vstr

#endif