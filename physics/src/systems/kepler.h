// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

// This file implements Keplerian orbital motion.

#ifndef VSTR_ORBIT
#define VSTR_ORBIT

#include <assert.h>

#include <cmath>

#include "geometry/vector3.h"
#include "types/optional_components.h"
#include "types/required_components.h"

namespace vstr {

// Solve the Kepler equations to return the object's position.
Vector3 EllipticalPosition(const Orbit::Kepler &kepler);

// Compute the orbital position at time 't' for each object in orbit, and store
// the results in Motion.next_position. (See UpdatePositions for the pipeline
// step that works with next_position.)
void UpdateOrbitalMotion(float t, const std::vector<Transform> &positions,
                         const std::vector<Orbit> &orbits,
                         std::vector<Motion> &motion);

}  // namespace vstr

#endif
