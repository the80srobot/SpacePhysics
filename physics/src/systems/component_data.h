// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_COMPONENT_DATA
#define VSTR_COMPONENT_DATA

#include <iostream>
#include <vector>

#include "geometry/aabb.h"
#include "geometry/vector3.h"

namespace vstr {

// This file defines three flavors of physics data. Core components, optional
// components and events.
//
// Because every object has an instance of each core component, the data for
// core components are stored in vectors, such that the offset into the vector
// is the object ID.
//
// Optional components, on the other hand, include their object ID as the first
// data member.
//
// Finally events occur over some interval (possibly just one frame), during
// which they affect the state of an object. Examples of events are:
// destruction, acceleration from user input and collisions.

// Core components:

struct Position {
  Vector3 value;
};

static_assert(std::is_standard_layout<Position>());

inline bool operator==(const Position &a, const Position &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Position &position);

struct Mass {
  float intertial;
  float active;
};

static_assert(std::is_standard_layout<Mass>());

inline bool operator==(const Mass &a, const Mass &b) {
  return a.intertial == b.intertial && a.active == b.active;
}

std::ostream &operator<<(std::ostream &os, const Mass &mass);

struct Motion {
  Vector3 velocity;
  Vector3 new_position;
  Vector3 acceleration;

  inline static Motion FromPositionAndVelocity(Vector3 position,
                                               Vector3 velocity,
                                               Vector3 acceleration = Vector3{
                                                   0, 0, 0}) {
    return Motion{
        velocity,
        position + velocity,
        acceleration,
    };
  }
};

static_assert(std::is_standard_layout<Motion>());

inline bool operator==(const Motion &a, const Motion &b) {
  return a.velocity == b.velocity && a.new_position == b.new_position;
}

std::ostream &operator<<(std::ostream &os, const Motion &motion);

struct Collider {
  uint32_t layer;
  float radius;
};

static_assert(std::is_standard_layout<Collider>());

inline bool operator==(const Collider &a, const Collider &b) {
  return a.layer == b.layer && a.radius == b.radius;
}

std::ostream &operator<<(std::ostream &os, const Collider &collider);

struct Glue {
  int32_t parent_id;
};

static_assert(std::is_standard_layout<Glue>());

inline bool operator==(const Glue &a, const Glue &b) {
  return a.parent_id == b.parent_id;
}

std::ostream &operator<<(std::ostream &os, const Glue &orbit);

struct Flags {
  uint32_t value;

  static constexpr uint32_t kDestroyed = 1;
  static constexpr uint32_t kGlued = 1 << 1;
  static constexpr uint32_t kOrbiting = 1 << 2;
};

static_assert(std::is_standard_layout<Flags>());

inline bool operator==(const Flags &a, const Flags &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Flags &destroyed);

// Optional components:

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
};

inline bool operator==(const Durability &a, const Durability &b) {
  return a.id == b.id && a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Durability &durability);

// Events:

struct Acceleration {
  enum Flag {
    kNone = 0,
    // Apply the entire value on the first frame, insted of dividing by delta t.
    kImpulse = 1 << 0,
    // Divide the value by mass to obtain acceleration.
    kForce = 1 << 1,
  };
  Vector3 value;
  Flag flags;
};

static_assert(std::is_standard_layout<Acceleration>());

inline bool operator==(const Acceleration &a, const Acceleration &b) {
  return a.value == b.value;
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

struct Event {
  enum Type { kAcceleration = 1, kCollision = 2, kStick = 3, kDestruction = 4 };

  Event(Vector3 position, Collision &&collision)
      : type(kCollision),
        id(collision.first_id),
        position(position),
        collision(std::move(collision)) {}

  Event(int id, Acceleration &&acceleration)
      : id(id), type(kAcceleration), acceleration(acceleration) {}

  Event(int id, Destruction &&destruction)
      : id(id), type(kDestruction), destruction(destruction) {}

  int32_t id;
  Type type;
  Vector3 position;

  union {
    Acceleration acceleration;
    Collision collision;
    Stick stick;
    Destruction destruction;
  };
};

static_assert(sizeof(Event) == 36);
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

}  // namespace vstr

#endif