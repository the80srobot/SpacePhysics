// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_EVENTS
#define VSTR_EVENTS

#include <iostream>
#include <vector>

#include "absl/types/span.h"
#include "geometry/aabb.h"
#include "geometry/quaternion.h"
#include "geometry/vector3.h"
#include "types/optional_components.h"

namespace vstr {

struct Acceleration {
  enum Flag {
    kNone = 0,
    // Apply the entire value on the first frame, insted of dividing by delta t.
    kImpulse = 1 << 0,
    // Divide the value by mass to obtain acceleration.
    kForce = 1 << 1,
  };
  Vector3 linear;
  Flag flags;
  Quaternion angular;
};

static_assert(std::is_standard_layout<Acceleration>());

inline bool operator==(const Acceleration &a, const Acceleration &b) {
  return a.linear == b.linear && a.angular == b.angular;
}

std::ostream &operator<<(std::ostream &os, const Acceleration &acceleration);

struct Collision {
  int32_t first_id;
  int32_t second_id;
  float first_frame_offset_seconds;
};

static_assert(std::is_standard_layout<Collision>());

inline bool operator==(const Collision &a, const Collision &b) {
  // Intentionally don't compare the offset on the first frame. It turns out
  // many operations (like merging event intervals) are greatly simplified if we
  // consider that to be just metadata.
  return a.first_id == b.first_id && a.second_id == b.second_id;
}

std::ostream &operator<<(std::ostream &os, const Collision &collision);

struct Stick {
  int32_t parent_id;
};

static_assert(std::is_standard_layout<Stick>());

inline bool operator==(const Stick &a, const Stick &b) {
  return a.parent_id == b.parent_id;
}

std::ostream &operator<<(std::ostream &os, const Stick &stick);

struct Destruction {
  int32_t value;
};

static_assert(std::is_standard_layout<Destruction>());

inline bool operator==(const Destruction &a, const Destruction &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Destruction &destruction);

struct Damage {
  int32_t value;
};

static_assert(std::is_standard_layout<Damage>());

inline bool operator==(const Damage &a, const Damage &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Damage &damage);

struct Teleportation {
  Vector3 new_position;
  Vector3 new_velocity;
  Quaternion new_spin;
};

static_assert(std::is_standard_layout<Teleportation>());

inline bool operator==(const Teleportation &a, const Teleportation &b) {
  return a.new_position == b.new_position && a.new_velocity == b.new_velocity;
}

std::ostream &operator<<(std::ostream &os, const Teleportation &teleportation);

struct RocketBurn {
  int32_t fuel_tank;
  // The desired thrust as fraction of the rocket's output. (So ranging in
  // magnitude from 0 to 1.)
  Vector3 thrust;
};

static_assert(std::is_standard_layout<RocketBurn>());

inline bool operator==(const RocketBurn &a, const RocketBurn &b) {
  return a.fuel_tank == b.fuel_tank && a.thrust == b.thrust;
}

std::ostream &operator<<(std::ostream &os, const RocketBurn &rocket_burn);

struct RocketRefuel {
  int32_t fuel_tank_no;
  Rocket::FuelTank fuel_tank;
};

static_assert(std::is_standard_layout<RocketRefuel>());

inline bool operator==(const RocketRefuel &a, const RocketRefuel &b) {
  return a.fuel_tank_no == b.fuel_tank_no && a.fuel_tank == b.fuel_tank;
}

std::ostream &operator<<(std::ostream &os, const RocketRefuel &rocket_refuel);

struct Event {
  enum Type {
    kAcceleration = 1,
    kCollision = 2,
    kStick = 3,
    kDestruction = 4,
    kDamage = 5,
    kTeleportation = 6,
    kRocketBurn = 7,
    kRocketRefuel = 8,
  };

  Event() = default;

  explicit Event(Vector3 position, Collision &&collision)
      : type(kCollision),
        id(collision.first_id),
        position(position),
        collision(std::move(collision)) {}

  explicit Event(int id, Vector3 position, Acceleration &&acceleration)
      : id(id),
        type(kAcceleration),
        acceleration(acceleration),
        position(position) {}

  explicit Event(int id, Vector3 position, Destruction &&destruction)
      : id(id),
        type(kDestruction),
        destruction(destruction),
        position(position) {}

  explicit Event(int id, Vector3 position, Damage &&damage)
      : id(id), type(kDamage), damage(damage), position(position) {}

  explicit Event(int id, Vector3 position, Teleportation &&teleportation)
      : id(id),
        position(position),
        type(kTeleportation),
        teleportation(teleportation) {}

  explicit Event(int id, Vector3 position, RocketBurn &&rocket_burn)
      : id(id),
        type(kRocketBurn),
        rocket_burn(rocket_burn),
        position(position) {}

  explicit Event(int id, Vector3 position, RocketRefuel &&rocket_refuel)
      : id(id),
        type(kRocketRefuel),
        rocket_refuel(rocket_refuel),
        position(position) {}

  int32_t id;
  Type type;
  Vector3 position;

  union {
    Acceleration acceleration;
    Collision collision;
    Stick stick;
    Destruction destruction;
    Damage damage;
    Teleportation teleportation;
    RocketBurn rocket_burn;
    RocketRefuel rocket_refuel;
  };
};

static_assert(sizeof(Event) == 60);
static_assert(std::is_standard_layout<Event>());
static_assert(std::is_move_assignable<Event>());
static_assert(std::is_move_constructible<Event>());

bool operator==(const Event &a, const Event &b);
bool operator>=(const Event &a, const Event &b);
bool operator>(const Event &a, const Event &b);
bool operator<(const Event &a, const Event &b);
bool operator<=(const Event &a, const Event &b);

std::ostream &operator<<(std::ostream &os, Event::Type event_type);
std::ostream &operator<<(std::ostream &os, const Event &event);

// Specifies a per-object argument to the per-layer collision rule action
// kTriggerEvent. (Does nothing by itself.)
//
// TODO(adam): this shouldn't be defined here once it no longer inlines the
// event definition.
struct Trigger {
  int32_t id;

  enum Condition {
    kColission,
  };

  enum Target {
    kSelf,
    kCollidingObject,
  };

  enum Flag {
    kDestroyTrigger = 1 << 0,
  };

  Condition condition;
  Target target;
  Flag flags;
  Event event;
};

inline bool operator==(const Trigger &a, const Trigger &b) {
  return a.id == b.id && a.condition == b.condition && a.target == b.target;
}

std::ostream &operator<<(std::ostream &os, const Trigger &trigger);

}  // namespace vstr

#endif