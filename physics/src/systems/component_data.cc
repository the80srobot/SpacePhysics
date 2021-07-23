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
            << orbit.epoch << ", /*delta=*/" << orbit.delta << "}";
}

std::ostream &operator<<(std::ostream &os, const Glue &glue) {
  return os << "Glue{/*parent_id=*/" << glue.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const Flags &flags) {
  return os << "Flags{" << flags.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Collision &collision) {
  return os << "CollisionEvent{/*first_id=*/" << collision.first_id
            << ", /*second_id=*/" << collision.second_id
            << ", /*first_frame_offset_seconds=*/"
            << collision.first_frame_offset_seconds << "}";
}

std::ostream &operator<<(std::ostream &os, const Input &input) {
  return os << "Input{/*acceleration=*/" << input.acceleration << "}";
}

bool operator==(const Event &a, const Event &b) {
  // Intentionally don't check the value of the event - each event can only
  // occur once per unique pair of interval and object.
  if (!(a.id == b.id && a.type == b.type)) {
    return false;
  }

  switch (a.type) {
    case Event::kInput:
      return a.input == b.input;
    case Event::kGlue:
      return a.glue == b.glue;
    case Event::kFlags:
      return a.flags == b.flags;
    case Event::kCollision:
      return a.collision == b.collision;
    default:
      assert(false);  // Programmer error - unreachable.
      return true;
  }
}

bool operator>(const Event &a, const Event &b) {
  return (a.id > b.id) || ((a.id == b.id) && a.type > b.type);
}

bool operator<(const Event &a, const Event &b) {
  return (a.id < b.id) || ((a.id == b.id) && a.type < b.type);
}

std::ostream &operator<<(std::ostream &os, const Event::Type event_type) {
  switch (event_type) {
    case Event::Type::kInput:
      return os << "input";
    case Event::Type::kGlue:
      return os << "glue";
    case Event::Type::kFlags:
      return os << "destroyed";
    case Event::Type::kCollision:
      return os << "collision";
    default:
      assert("not reachable");
  }
}

std::ostream &operator<<(std::ostream &os, const Event &event) {
  os << "Event{/*id=*/" << event.id << ", /*type=*/" << event.type;
  switch (event.type) {
    case Event::Type::kInput:
      return os << ", /*input=*/" << event.input << "}";
    case Event::Type::kGlue:
      return os << ", /*glue=*/" << event.glue << "}";
    case Event::Type::kFlags:
      return os << ", /*destroyed=*/" << event.flags << "}";
    case Event::Type::kCollision:
      return os << ", /*collision=*/" << event.collision << "}";
    default:
      assert("not reachable");
  }
}

}  // namespace vstr