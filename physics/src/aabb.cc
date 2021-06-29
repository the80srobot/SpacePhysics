#include "aabb.h"

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

std::ostream &operator<<(std::ostream &os, const AABB &aabb) {
  return os << "AABB{/*min=*/" << aabb.min << ", /*max=*/" << aabb.max << "}";
}

}  // namespace vstr