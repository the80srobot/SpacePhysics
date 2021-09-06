// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_OPTIONAL_COMPONENTS
#define VSTR_OPTIONAL_COMPONENTS

#include <iostream>
#include <vector>

#include "absl/types/span.h"
#include "geometry/aabb.h"
#include "geometry/quaternion.h"
#include "geometry/vector3.h"

namespace vstr {

struct Orbit {
  int32_t id;

  struct Kepler {
    float semi_major_axis;
    float eccentricity;
    float mean_longitude_deg;
    float longitude_of_perihelion_deg;
    float longitude_of_ascending_node_deg;
    float inclination_deg;
  };

  Vector3 focus;
  Kepler epoch;
  Kepler delta;
};

static_assert(std::is_standard_layout<Orbit>());

inline bool operator==(const Orbit::Kepler &a, const Orbit::Kepler &b) {
  return a.semi_major_axis == b.semi_major_axis &&
         a.eccentricity == b.eccentricity &&
         a.mean_longitude_deg == b.mean_longitude_deg &&
         a.longitude_of_perihelion_deg == b.longitude_of_perihelion_deg &&
         a.longitude_of_ascending_node_deg ==
             b.longitude_of_ascending_node_deg &&
         a.inclination_deg == b.inclination_deg;
}

inline Orbit::Kepler operator+(const Orbit::Kepler &a, const Orbit::Kepler &b) {
  return Orbit::Kepler{
      a.semi_major_axis + b.semi_major_axis,
      a.eccentricity + b.eccentricity,
      a.mean_longitude_deg + b.mean_longitude_deg,
      a.longitude_of_perihelion_deg + b.longitude_of_perihelion_deg,
      a.longitude_of_ascending_node_deg + b.longitude_of_ascending_node_deg,
      a.inclination_deg + b.inclination_deg,
  };
}

inline Orbit::Kepler operator*(const Orbit::Kepler &a, const float b) {
  return Orbit::Kepler{
      a.semi_major_axis * b,
      a.eccentricity * b,
      a.mean_longitude_deg * b,
      a.longitude_of_perihelion_deg * b,
      a.longitude_of_ascending_node_deg * b,
      a.inclination_deg * b,
  };
}

std::ostream &operator<<(std::ostream &os, const Orbit::Kepler &kepler);

inline bool operator==(const Orbit &a, const Orbit &b) {
  return a.epoch == b.delta && a.focus == b.focus;
}

std::ostream &operator<<(std::ostream &os, const Orbit &orbit);

struct Durability {
  int32_t id;
  int32_t value;
  int32_t max;
};

inline bool operator==(const Durability &a, const Durability &b) {
  return a.id == b.id && a.value == b.value && a.max == b.max;
}

std::ostream &operator<<(std::ostream &os, const Durability &durability);

struct Rocket {
  static constexpr int kMaxFuelTanks = 8;

  int32_t id;

  struct FuelTank {
    // How much does the fuel in the tank weigh in kg per second of thrust.
    float mass_flow_rate;
    // Fuel in seconds: how long can the tank provide thrust in seconds.
    float fuel;
    // The force the fuel tank can produce in N.
    float thrust;
  };

  int32_t fuel_tank_count;
  FuelTank fuel_tanks[kMaxFuelTanks];
};

inline bool operator==(const Rocket &a, const Rocket &b) {
  return a.id == b.id &&
         absl::MakeConstSpan(a.fuel_tanks) == absl::MakeConstSpan(b.fuel_tanks);
}

inline bool operator==(const Rocket::FuelTank &a, const Rocket::FuelTank &b) {
  return a.mass_flow_rate == b.mass_flow_rate && a.fuel == b.fuel &&
         a.thrust == b.thrust;
}

std::ostream &operator<<(std::ostream &os, const Rocket &rocket);

std::ostream &operator<<(std::ostream &os, const Rocket::FuelTank &fuel_tank);

struct ReuseTag {
  int32_t id;
  int32_t pool_id;
  int32_t next_id;
};

inline bool operator==(const ReuseTag &a, const ReuseTag &b) {
  return a.id == b.id && a.pool_id == b.pool_id && a.next_id == b.next_id;
}

struct ReusePool {
  int32_t id;
  int32_t first_id;

  int32_t in_use_count;
  int32_t free_count;
};

inline bool operator==(const ReusePool &a, const ReusePool &b) {
  return a.id == b.id && a.first_id == b.first_id;
}

}  // namespace vstr

#endif