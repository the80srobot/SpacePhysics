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

 private:
};

}  // namespace vstr

#endif
