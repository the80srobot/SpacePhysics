// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_FRAME
#define VSTR_FRAME

#include <absl/types/span.h>

#include <compare>
#include <concepts>
#include <iostream>

#include "systems/collision_detector.h"
#include "systems/glue_system.h"
#include "systems/kepler.h"
#include "systems/motion.h"
#include "types/id.h"
#include "types/optional_components.h"
#include "types/required_components.h"

namespace vstr {

struct Frame {
  static int32_t constexpr kMaxObjects = 10000;

  // Core components. Point mass moves clumsily, goes fast.
  std::vector<Transform> transforms;
  std::vector<Mass> mass;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Glue> glue;
  std::vector<Flags> flags;

  // Optional components:
  std::vector<Orbit> orbits;
  std::vector<Durability> durability;
  std::vector<Rocket> rockets;
  std::vector<Trigger> triggers;
  std::vector<ReusePool> reuse_pools;
  std::vector<ReuseTag> reuse_tags;

  Entity Push();
  Entity Push(Transform &&transform, Mass &&mass, Motion &&motion,
              Collider &&collider, Glue &&glue, Flags &&flags);
};

template <typename T>
concept OptionalComponent = requires(T x) {
  { T().id } -> std::same_as<Entity>;
};

template <OptionalComponent T>
ssize_t FindOptionalComponent(const std::vector<T> &component_data,
                              const Entity id) {
  auto it = std::lower_bound(
      component_data.begin(), component_data.end(), T{.id = id},
      [](const T &a, const T &b) { return a.id < b.id; });
  if (it != component_data.end() && it->id == id) {
    return it - component_data.begin();
  }
  return -1;
}

template <OptionalComponent T>
ssize_t SetOptionalComponent(const Entity id, const T &component,
                             std::vector<T> &component_data) {
  int32_t dst_idx = FindOptionalComponent(component_data, id);

  T cpy = component;
  cpy.id = id;

  if (dst_idx >= 0) {
    component_data[dst_idx] = std::move(component);
    return dst_idx;
  }

  // The optional component isn't set yet – add it in the right place.
  // TODO(Adam): this could be a lot more optimal by reusing the
  // std::lower_bound value from FindOptionalComponent, because the vector
  // starts out sorted.

  component_data.push_back(std::move(cpy));
  // This check: is the component we just added out of order?
  if (component_data.size() > 1 &&
      (component_data.end() - 1)->id < (component_data.end() - 2)->id) {
    // We only hit this codepath when initializing, because components can't be
    // added when the simulation is running.
    std::sort(component_data.begin(), component_data.end(),
              [](const T &a, const T &b) { return a.id < b.id; });
  }

  return FindOptionalComponent(component_data, id);
}

template <OptionalComponent T>
void CopyOptionalComponent(const Entity dst, const Entity src,
                           std::vector<T> &component_data) {
  ssize_t src_idx = FindOptionalComponent(component_data, src);
  if (src_idx < 0) return;
  SetOptionalComponent(dst, component_data[src_idx], component_data);
}

}  // namespace vstr
#endif