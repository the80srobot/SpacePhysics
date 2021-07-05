// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_ORBIT
#define VSTR_ORBIT

#include <assert.h>

#include <cmath>

#include "component_data.h"
#include "vector3.h"

namespace vstr {
class OrbitSystem {
 public:
  static Vector3 EllipticalPosition(const Orbit::Kepler &kepler);

  void Step(float t, const std::vector<Position> &positions,
            const std::vector<Orbit> &orbits, std::vector<Motion> &motion);

 private:
};

}  // namespace vstr

#endif
