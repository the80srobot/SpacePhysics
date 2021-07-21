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

struct Collider {
  uint32_t layer;
  float radius;
};

inline bool operator==(const Collider &a, const Collider &b) {
  return a.layer == b.layer && a.radius == b.radius;
}

std::ostream &operator<<(std::ostream &os, const Collider &collider);

struct Position {
  Vector3 value;
};

inline bool operator==(const Position &a, const Position &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Position &position);

struct Mass {
  float rest;
  float effective;
};

inline bool operator==(const Mass &a, const Mass &b) {
  return a.rest == b.rest && a.effective == b.effective;
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

inline bool operator==(const Motion &a, const Motion &b) {
  return a.velocity == b.velocity && a.new_position == b.new_position;
}

std::ostream &operator<<(std::ostream &os, const Motion &motion);

struct Glue {
  int32_t parent_id;
};

inline bool operator==(const Glue &a, const Glue &b) {
  return a.parent_id == b.parent_id;
}

std::ostream &operator<<(std::ostream &os, const Glue &glue);

struct Flags {
  uint32_t value;

  static constexpr uint32_t kDestroyed = 1;
  static constexpr uint32_t kGlued = 1 << 1;
  static constexpr uint32_t kOrbiting = 1 << 2;
};

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

std::ostream &operator<<(std::ostream &os, const Orbit &glue);

// Events:

struct Collision {
  int32_t first_id;
  int32_t second_id;
  float first_frame_offset_seconds;
};

inline bool operator==(const Collision &a, const Collision &b) {
  return a.first_id == b.first_id && a.second_id == b.second_id &&
         a.first_frame_offset_seconds == b.first_frame_offset_seconds;
}

std::ostream &operator<<(std::ostream &os, const Collision &collision);

struct Input {
  Vector3 acceleration;
};

inline bool operator==(const Input &a, const Input &b) {
  return a.acceleration == b.acceleration;
}

std::ostream &operator<<(std::ostream &os, const Input &input);

struct Event {
  enum Type { kInput, kGlue, kDestroyed, kCollision };

  Event(Collision &&collision)
      : type(kCollision),
        id(collision.first_id),
        collision(std::move(collision)) {}

  Event(int id, Input &&input) : id(id), type(kInput), input(input) {}

  int32_t id;
  Type type;

  union {
    Input input;
    Glue glue;
    Flags destroyed;
    Collision collision;
  };
};

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