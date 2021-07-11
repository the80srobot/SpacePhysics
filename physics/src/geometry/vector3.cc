// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>
//
// This file provides basic 3D vector math.

#include "geometry/vector3.h"

namespace vstr {

std::ostream& operator<<(std::ostream& os, const Vector3 v) {
  return os << "{" << v.x << ", " << v.y << ", " << v.z << "}";
}

}  // namespace vstr