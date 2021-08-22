// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "required_components.h"

namespace vstr {

std::ostream &operator<<(std::ostream &os, const Collider &collider) {
  return os << "Collider{/*layer=*/" << collider.layer << ", /*radius=*/"
            << collider.radius << ", /*center=*/" << collider.center << "}";
}

std::ostream &operator<<(std::ostream &os, const Transform &transform) {
  return os << "Transform{/*position=*/" << transform.position
            << ", /*rotation=*/" << transform.rotation << "}";
}

std::ostream &operator<<(std::ostream &os, const Mass &mass) {
  return os << "Mass{/*inertial=*/" << mass.inertial << ", /*active=*/"
            << mass.active << ", /*cutoff_distance=*/" << mass.cutoff_distance
            << "}";
}

std::ostream &operator<<(std::ostream &os, const Motion &motion) {
  return os << "Motion{/*velocity=*/" << motion.velocity
            << ", /*new_position=*/" << motion.new_position
            << ", /*acceleration=*/" << motion.acceleration << ", /*spin=*/"
            << motion.spin << "}";
}

std::ostream &operator<<(std::ostream &os, const Glue &glue) {
  return os << "Glue{/*parent_id=*/" << glue.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const Flags &flags) {
  return os << "Flags{" << flags.value << "}";
}

}  // namespace vstr