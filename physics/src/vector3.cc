#include "vector3.h"

namespace vstr {

std::ostream& operator<<(std::ostream& os, const Vector3 v) {
  return os << "{" << v.x << ", " << v.y << ", " << v.z << "}";
}

}  // namespace vstr