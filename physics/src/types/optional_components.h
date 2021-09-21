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
#include "types/entity.h"

namespace vstr {

struct Orbit {
  Entity id;

  struct Kepler {
    float semi_major_axis;
    float eccentricity;
    float mean_longitude_deg;
    float longitude_of_perihelion_deg;
    float longitude_of_ascending_node_deg;
    float inclination_deg;

    bool operator==(const Kepler &) const = default;
  };

  Vector3 focus;
  Kepler epoch;
  Kepler delta;

  bool operator==(const Orbit &) const = default;
};

static_assert(std::is_standard_layout<Orbit>());

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

std::ostream &operator<<(std::ostream &os, const Orbit &orbit);

struct Durability {
  Entity id;
  int32_t value;
  int32_t max;

  bool operator==(const Durability &) const = default;
};

std::ostream &operator<<(std::ostream &os, const Durability &durability);

struct Rocket {
  static constexpr int kMaxFuelTanks = 8;

  Entity id;

  struct FuelTank {
    // How much does the fuel in the tank weigh in kg per second of thrust.
    float mass_flow_rate;
    // Fuel in seconds: how long can the tank provide thrust in seconds.
    float fuel;
    // The force the fuel tank can produce in N.
    float thrust;

    bool operator==(const FuelTank &) const = default;
  };

  int32_t fuel_tank_count;
  FuelTank fuel_tanks[kMaxFuelTanks];
};

inline bool operator==(const Rocket &a, const Rocket &b) {
  return a.id == b.id &&
         absl::MakeConstSpan(a.fuel_tanks) == absl::MakeConstSpan(b.fuel_tanks);
}

std::ostream &operator<<(std::ostream &os, const Rocket &rocket);

std::ostream &operator<<(std::ostream &os, const Rocket::FuelTank &fuel_tank);

struct ReuseTag {
  Entity id;
  Entity pool_id;
  Entity next_id;

  bool operator==(const ReuseTag &) const = default;
};

struct ReusePool {
  Entity id;
  Entity first_id;

  int32_t in_use_count;
  int32_t free_count;

  bool operator==(const ReusePool &) const = default;
};

}  // namespace vstr

#endif