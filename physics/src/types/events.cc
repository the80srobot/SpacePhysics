// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "events.h"

namespace vstr {

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
  return os << "Destruction{}";
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

std::ostream &operator<<(std::ostream &os, const Spawn &spawn) {
  return os << "Spawn{/*pool_id=*/" << spawn.pool_id << ", /*velocity=*/"
            << spawn.velocity << ", /*rotation=*/" << spawn.rotation << "}";
}

std::ostream &operator<<(std::ostream &os, const SpawnAttempt &spawn_attempt) {
  return os << "SpawnAttempt{/*velocity=*/" << spawn_attempt.velocity
            << ", /*rotation=*/" << spawn_attempt.rotation << "}";
}

bool Event::operator==(const Event &other) const {
  if (!(id == other.id && type == other.type)) {
    return false;
  }

  switch (type) {
    case Event::kAcceleration:
      return acceleration == other.acceleration;
    case Event::kStick:
      return stick == other.stick;
    case Event::kDestruction:
      return destruction == other.destruction;
    case Event::kCollision:
      return collision == other.collision;
    case Event::kDamage:
      return damage == other.damage;
    case Event::kTeleportation:
      return teleportation == other.teleportation;
    case Event::kRocketBurn:
      return rocket_burn == other.rocket_burn;
    case Event::kRocketRefuel:
      return rocket_refuel == other.rocket_refuel;
    case Event::kSpawn:
      return spawn == other.spawn;
    case Event::kSpawnAttempt:
      return spawn_attempt == other.spawn_attempt;
    default:
      assert(false);  // Programmer error - unreachable.
      return true;
  }
}

std::partial_ordering Event::operator<=>(const Event &other) const {
  std::partial_ordering result = id <=> other.id;
  if (result != std::partial_ordering::equivalent) return result;
  if ((result = (type <=> other.type)) != std::partial_ordering::equivalent)
    return result;

  if ((result = (position <=> other.position)) !=
      std::partial_ordering::equivalent)
    return result;
}

// bool operator>(const Event &a, const Event &b) {
//   if (a.id > b.id) return true;

//   return (a.id > b.id) || ((a.id == b.id) && a.type > b.type);
// }

// bool operator<(const Event &a, const Event &b) {
//   return (a.id < b.id) || ((a.id == b.id) && a.type < b.type);
// }

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
    case Event::Type::kSpawn:
      return os << "spawn";
    case Event::Type::kSpawnAttempt:
      return os << "spawn_attempt";
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
    case Event::Type::kSpawn:
      return os << ", /*spawn=*/" << event.spawn << " }";
    case Event::Type::kSpawnAttempt:
      return os << ", /*spawn_attempt=*/" << event.spawn_attempt << " }";
    default:
      assert("not reachable");
  }
}

std::ostream &operator<<(std::ostream &os, const Trigger &trigger) {
  return os << "Trigger{/*id=*/" << trigger.id << ", /*condition=*/"
            << trigger.condition << ", /*target=*/" << trigger.target
            << ", /*event=*/" << trigger.event << "}";
}

}  // namespace vstr