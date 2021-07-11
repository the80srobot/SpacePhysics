// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "geometry/aabb.h"

namespace vstr {

bool AABB::Overlaps(const AABB &other) const {
  return max >= other.min && min <= other.max;
}

void AABB::Encapsulate(const AABB &other) {
  max = Vector3::Max(max, other.max);
  min = Vector3::Min(min, other.min);
}

void AABB::Encapsulate(const Vector3 &point) {
  max = Vector3::Max(max, point);
  min = Vector3::Min(min, point);
}

void AABB::Sweep(const Vector3 &motion) {
  // TODO: is this right?
  max = Vector3::Max(max, max + motion);
  min = Vector3::Min(min, min + motion);
}

std::ostream &operator<<(std::ostream &os, const AABB &aabb) {
  return os << "AABB{/*min=*/" << aabb.min << ", /*max=*/" << aabb.max << "}";
}

AABB AABB::FromCenterAndExtents(const Vector3 center, const Vector3 extents) {
  Vector3 half = extents / 2;
  return AABB(center - half, center + half);
}

AABB AABB::FromCenterAndHalfExtents(const Vector3 center,
                                    const Vector3 half_extents) {
  return AABB(center - half_extents, center + half_extents);
}

}  // namespace vstr