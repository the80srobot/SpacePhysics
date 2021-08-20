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

std::ostream &operator<<(std::ostream &os, const Trigger &trigger) {
  return os << "Trigger{/*id=*/" << trigger.id << ", /*condition=*/"
            << trigger.condition << ", /*target=*/" << trigger.target
            << ", /*event=*/" << trigger.event << "}";
}

std::ostream &operator<<(std::ostream &os, const Glue &glue) {
  return os << "Glue{/*parent_id=*/" << glue.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const Flags &flags) {
  return os << "Flags{" << flags.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Acceleration &acceleration) {
  return os << "Acceleration{/*linear=*/" << acceleration.linear
            << ", /*angular=*/" << acceleration.angular << ", /*impulse=*/"
            << (acceleration.flags & Acceleration::Flag::kImpulse)
            << ", /*force=*/"
            << (acceleration.flags & Acceleration::Flag::kForce) << "}";
}

std::ostream &operator<<(std::ostream &os, const Collision &collision) {
  return os << "Collision{/*first_id=*/" << collision.first_id
            << ", /*second_id=*/" << collision.second_id
            << ", /*first_frame_offset_seconds=*/"
            << collision.first_frame_offset_seconds << "}";
}

std::ostream &operator<<(std::ostream &os, const Stick &stick) {
  return os << "Stick{/*parent_id=*/" << stick.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const Destruction &destruction) {
  return os << "Destruction{/*value=*/" << destruction.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Damage &damage) {
  return os << "Damage{/*value=*/" << damage.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Teleportation &teleportation) {
  return os << "Teleportation{/*new_position=*/" << teleportation.new_position
            << ", /*new_velocity=*/" << teleportation.new_velocity << "}";
}

std::ostream &operator<<(std::ostream &os, const RocketBurn &rocket_burn) {
  return os << "RocketBurn{/*fuel_tank=*/" << rocket_burn.fuel_tank
            << ", /*thrust=*/" << rocket_burn.thrust << "}";
}

std::ostream &operator<<(std::ostream &os, const RocketRefuel &rocket_refuel) {
  return os << "RocketBurn{/*fuel_tank_no=*/" << rocket_refuel.fuel_tank_no
            << ", /*fuel_tank=*/" << rocket_refuel.fuel_tank << "}";
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
    case Event::kStick:
      return a.stick == b.stick;
    case Event::kDestruction:
      return a.destruction == b.destruction;
    case Event::kCollision:
      return a.collision == b.collision;
    case Event::kDamage:
      return a.damage == b.damage;
    case Event::kTeleportation:
      return a.teleportation == b.teleportation;
    case Event::kRocketBurn:
      return a.rocket_burn == b.rocket_burn;
    case Event::kRocketRefuel:
      return a.rocket_refuel == b.rocket_refuel;
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
    case Event::Type::kStick:
      return os << "stick";
    case Event::Type::kDestruction:
      return os << "destruction";
    case Event::Type::kCollision:
      return os << "collision";
    case Event::Type::kDamage:
      return os << "damage";
    case Event::Type::kTeleportation:
      return os << "teleportation";
    case Event::Type::kRocketBurn:
      return os << "rocket_burn";
    case Event::Type::kRocketRefuel:
      return os << "rocket_refuel";
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
    case Event::Type::kStick:
      return os << ", /*stick=*/" << event.stick << "}";
    case Event::Type::kDestruction:
      return os << ", /*destruction=*/" << event.destruction << "}";
    case Event::Type::kCollision:
      return os << ", /*collision=*/" << event.collision << "}";
    case Event::Type::kDamage:
      return os << ", /*damage=*/" << event.damage << "}";
    case Event::Type::kTeleportation:
      return os << ", /*teleportation=*/" << event.teleportation << "}";
    case Event::Type::kRocketBurn:
      return os << ", /*rocket_burn=*/" << event.rocket_burn << "}";
    case Event::Type::kRocketRefuel:
      return os << ", /*rocket_refuel=*/" << event.rocket_refuel << "}";
    default:
      assert("not reachable");
  }
}

}  // namespace vstr