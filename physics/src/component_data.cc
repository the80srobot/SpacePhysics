// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "component_data.h"

namespace vstr {

std::ostream &operator<<(std::ostream &os, const Collider &collider) {
  return os << "Collider{/*layer=*/" << collider.layer << ", /*radius=*/"
            << collider.radius << "}";
}

std::ostream &operator<<(std::ostream &os, const Position &position) {
  return os << "Position{" << position.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Acceleration &acceleration) {
  return os << "Acceleration{" << acceleration.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Mass &mass) {
  return os << "Mass{/*rest=*/" << mass.rest << ", /*effective=*/"
            << mass.effective << "}";
}

std::ostream &operator<<(std::ostream &os, const Motion &motion) {
  return os << "Motion{/*velocity=*/" << motion.velocity
            << ", /*new_position=*/" << motion.new_position << "}";
}

std::ostream &operator<<(std::ostream &os, const Orbit::Kepler &kepler) {
  return os << "Kepler{" << kepler.semi_major_axis << ", "
            << kepler.eccentricity << ", " << kepler.mean_longitude_deg << ", "
            << kepler.longitude_of_perihelion_deg << ", "
            << kepler.longitude_of_ascending_node_deg << ", "
            << kepler.inclination_deg << "}";
}

std::ostream &operator<<(std::ostream &os, const Orbit &orbit) {
  return os << "Orbit{/*focus=*/" << orbit.focus << ", /*initial=*/"
            << orbit.initial << ", /*delta=*/" << orbit.delta << "}";
}

std::ostream &operator<<(std::ostream &os, const Glue &glue) {
  return os << "Glue{/*parent_id=*/" << glue.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const Destroyed &destroyed) {
  return os << "Destroyed{" << destroyed.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Collision &event) {
  return os << "CollisionEvent{/*first_body_id=*/" << event.first_body_id
            << ", /*second_body_id=*/" << event.second_body_id
            << ", /*first_frame_offset_seconds=*/"
            << event.first_frame_offset_seconds << "}";
}

}  // namespace vstr