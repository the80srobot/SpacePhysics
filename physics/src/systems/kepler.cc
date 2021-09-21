// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "kepler.h"

namespace vstr {

Vector3 EllipticalPosition(const Orbit::Kepler &kepler) {
  // It's called elliptical position. We don't take kindly to no parabolas or
  // hyperboles 'round these parts.
  if (kepler.eccentricity >= 1 || kepler.eccentricity < 0) {
    return Vector3::Zero();
  }

  // For explanation, see: https://ssd.jpl.nasa.gov/txt/aprx_pos_planets.pdf

  constexpr float kRadiansPerDeg = 0.0174532924;
  // Everything should be in radians for simplicity. These symbols are chosen to
  // match with literature on Kepler orbits (such as the link above).
  const float a = kepler.semi_major_axis;
  const float e = kepler.eccentricity;
  const float L = kepler.mean_longitude_deg * kRadiansPerDeg;
  const float ɷ = kepler.longitude_of_perihelion_deg * kRadiansPerDeg;
  const float Ω = kepler.longitude_of_ascending_node_deg * kRadiansPerDeg;
  const float I = kepler.inclination_deg * kRadiansPerDeg;

  // Argument of perihelion
  const float ω = ɷ - Ω;
  // Mean anomaly
  const float M =
      std::fmodf(L - ɷ, 360 * kRadiansPerDeg) - 180 * kRadiansPerDeg;

  // Kepler's equation relates mean anomaly to the eccentric anomaly (E):
  //
  //  M = E - (180/π × e) × sin(E)
  //
  // Solve for E using Newton's method with a cap on the number of iterations.
  float E = M;
  float e2 = e;
  for (int i = 0; i < 100; i++) {
    float dE = (E - e2 * std::sinf(E) - M) / (1 - e2 * std::cosf(E));
    E -= dE;
    if (std::abs(dE) < 1e-6) {
      break;
    }
  }

  // Coordinates x and y relative to the focus, in the plane of the focus.
  float x_ = a * (std::cosf(E) - e);
  float y_ = a * std::sqrtf(1 - e * e) * std::sinf(E);

  // Now rotate to the inclined orbital plane.
  float x = (std::cosf(ω) * std::cosf(Ω) -
             std::sinf(ω) * std::sinf(Ω) * std::cosf(I)) *
                x_ +
            (-std::sinf(ω) * std::cosf(Ω) -
             std::cosf(ω) * std::sinf(Ω) * std::cosf(I)) *
                y_;
  float y = (std::cosf(ω) * std::sinf(Ω) -
             std::sinf(ω) * std::cosf(Ω) * std::cosf(I)) *
                x_ +
            (-std::sinf(ω) * std::sinf(Ω) -
             std::cosf(ω) * std::cosf(Ω) * std::cosf(I)) *
                y_;
  float z = std::sinf(ω) * std::sinf(I) * x_ + std::cosf(ω) * std::sinf(I) * y_;

  return Vector3{x, y, z};
}

void UpdateOrbitalMotion(const float t,
                         const std::vector<Transform> &transforms,
                         const std::vector<Orbit> &orbits,
                         std::vector<Motion> &motion) {
  for (const auto &orbit : orbits) {
    const Orbit::Kepler current = orbit.epoch + orbit.delta * t;
    orbit.id.Get(motion).new_position =
        orbit.focus + EllipticalPosition(current);
    orbit.id.Get(motion).velocity =
        orbit.id.Get(motion).new_position - orbit.id.Get(transforms).position;
  }
}

}  // namespace vstr