// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TYPES_REQUIRED_COMPONENTS
#define VSTR_TYPES_REQUIRED_COMPONENTS

#include <iostream>
#include <vector>

#include "absl/types/span.h"
#include "geometry/aabb.h"
#include "geometry/quaternion.h"
#include "geometry/vector3.h"
#include "types/events.h"

namespace vstr {

struct Transform {
  Vector3 position;
  Quaternion rotation;

  bool operator==(const Transform &) const = default;
};

static_assert(std::is_standard_layout<Transform>());

std::ostream &operator<<(std::ostream &os, const Transform &position);

struct Mass {
  float inertial;
  float active;
  float cutoff_distance;

  bool operator==(const Mass &) const = default;
};

static_assert(std::is_standard_layout<Mass>());

std::ostream &operator<<(std::ostream &os, const Mass &mass);

struct Motion {
  Vector3 velocity;
  Vector3 new_position;
  Vector3 acceleration;

  Quaternion spin;

  inline static Motion FromPositionAndVelocity(Vector3 position,
                                               Vector3 velocity,
                                               Vector3 acceleration = Vector3{
                                                   0, 0, 0}) {
    return Motion{
        .velocity{velocity},
        .new_position{position + velocity},
        .acceleration{acceleration},
        .spin{0, 0, 0, 1},
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
  Vector3 center;

  bool operator==(const Collider &) const = default;
};

static_assert(std::is_standard_layout<Collider>());

std::ostream &operator<<(std::ostream &os, const Collider &collider);

struct Glue {
  Entity parent_id;

  bool operator==(const Glue &) const = default;
};

static_assert(std::is_standard_layout<Glue>());

std::ostream &operator<<(std::ostream &os, const Glue &orbit);

struct Flags {
  uint32_t value;

  static constexpr uint32_t kDestroyed = 1;
  static constexpr uint32_t kGlued = 1 << 1;
  static constexpr uint32_t kOrbiting = 1 << 2;
  static constexpr uint32_t kReusable = 1 << 3;

  bool operator==(const Flags &) const = default;

  // TODO(adam): Store the most recent change and frame.
};

static_assert(std::is_standard_layout<Flags>());

std::ostream &operator<<(std::ostream &os, const Flags &destroyed);

}  // namespace vstr

#endif