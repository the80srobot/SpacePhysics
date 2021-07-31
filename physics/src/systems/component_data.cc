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

std::ostream &operator<<(std::ostream &os, const Acceleration &acceleration) {
  return os << "Acceleration{/*acceleration=*/" << acceleration.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Collision &collision) {
  return os << "Collision{/*first_id=*/" << collision.first_id
            << ", /*second_id=*/" << collision.second_id
            << ", /*first_frame_offset_seconds=*/"
            << collision.first_frame_offset_seconds << "}";
}

std::ostream &operator<<(std::ostream &os, const AttachTo &attach_to) {
  return os << "AttachTo{/*parent_id=*/" << attach_to.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const SetDestroyed &set_destroyed) {
  return os << "SetDestroyed{/*value=*/" << set_destroyed.value << "}";
}

bool operator==(const Event &a, const Event &b) {
  // TODO(adam): we currently ignore the event position. This is a hack, but it
  // allows MergeInsert to work with Events even when the previous event
  // position is unknown, which is most of the time. Possibly the IntervalTree
  // should support a separate protocol for matching events on their
  // identifiers, without considering metadata.
  if (!(a.id == b.id && a.type == b.type)) {
    return false;
  }

  switch (a.type) {
    case Event::kAcceleration:
      return a.acceleration == b.acceleration;
    case Event::kAttachTo:
      return a.attach_to == b.attach_to;
    case Event::kSetDestroyed:
      return a.set_destroyed == b.set_destroyed;
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
    case Event::Type::kAcceleration:
      return os << "input";
    case Event::Type::kAttachTo:
      return os << "attach_to";
    case Event::Type::kSetDestroyed:
      return os << "set_destroyed";
    case Event::Type::kCollision:
      return os << "collision";
    default:
      assert("not reachable");
  }
}

std::ostream &operator<<(std::ostream &os, const Event &event) {
  os << "Event{/*id=*/" << event.id << ", /*type=*/" << event.type
     << ", /*position=*/" << event.position;
  switch (event.type) {
    case Event::Type::kAcceleration:
      return os << ", /*input=*/" << event.acceleration << "}";
    case Event::Type::kAttachTo:
      return os << ", /*attach_to=*/" << event.attach_to << "}";
    case Event::Type::kSetDestroyed:
      return os << ", /*set_destroyed=*/" << event.set_destroyed << "}";
    case Event::Type::kCollision:
      return os << ", /*collision=*/" << event.collision << "}";
    default:
      assert("not reachable");
  }
}

}  // namespace vstr