// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "optional_components.h"

namespace vstr {

std::ostream &operator<<(std::ostream &os, const Orbit::Kepler &kepler) {
  return os << "Kepler{" << kepler.semi_major_axis << ", "
            << kepler.eccentricity << ", " << kepler.mean_longitude_deg << ", "
            << kepler.longitude_of_perihelion_deg << ", "
            << kepler.longitude_of_ascending_node_deg << ", "
            << kepler.inclination_deg << "}";
}

std::ostream &operator<<(std::ostream &os, const Orbit &orbit) {
  return os << "Orbit{/*id=*/" << orbit.id << "/*focus=*/" << orbit.focus
            << ", /*initial=*/" << orbit.epoch << ", /*delta=*/" << orbit.delta
            << "}";
}

std::ostream &operator<<(std::ostream &os, const Durability &durability) {
  return os << "Durability{/*id=*/" << durability.id << ", /*value=*/"
            << durability.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Rocket &rocket) {
  os << "Rocket{/*id=*/" << rocket.id << ", /*fuel_tank_count=*/"
     << rocket.fuel_tank_count << ", /*fuel_tanks=*/{\n";
  for (int i = 0; i < rocket.fuel_tank_count; ++i) {
    os << "\t{/*mass_flow_rate=*/" << rocket.fuel_tanks[i].mass_flow_rate;
    os << ", /*fuel=*/" << rocket.fuel_tanks[i].fuel;
    os << ", /*thrust=*/" << rocket.fuel_tanks[i].thrust;
    os << "},\n";
  }

  return os << "}}";
}

std::ostream &operator<<(std::ostream &os, const Rocket::FuelTank &fuel_tank) {
  return os << "FuelTank{/*mass_flow_rate=*/" << fuel_tank.mass_flow_rate
            << ", /*fuel=*/" << fuel_tank.fuel << ", /*thrust=*/"
            << fuel_tank.thrust << "}";
}

}  // namespace vstr