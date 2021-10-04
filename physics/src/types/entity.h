// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TYPES_ID
#define VSTR_TYPES_ID

#include <compare>
#include <concepts>
#include <iostream>

namespace vstr {

// Forward declaration for the OptionalComponent concept.
class Entity;

// Optional components are stored in sorted vectors and found using binary
// search. As such, they must specify what entity they belong to in the
// component data.
template <typename T>
concept OptionalComponent = requires(T x) {
  { T().id } -> std::same_as<Entity>;
};

// Clang concept support is missing std::integral as of Clang 12. Defining
// Entity operators to only accept integers is a cheap way to guard against some
// type confusion errors.
template <typename T>
concept integral = std::is_integral_v<T>;

// ID identifies a physics object, and can be used to look up both required and
// optional components. It is intentionally opaque to prevent implicit casts -
// before ID was introduced, bugs were often caused by confusing object ID with
// optional component idx.
class Entity {
 public:
  Entity() : val_(-1) {}
  explicit Entity(int32_t val) : val_(val) { assert(val < kMax); }

  static constexpr int32_t kMax = 1000000;

  auto operator<=>(const Entity &) const = default;

  Entity operator-(Entity) const;
  Entity operator+(Entity) const;

  inline int32_t value() const { return val_; }

  template <typename H>
  friend H AbslHashValue(H h, const Entity &id) {
    return H::combine(std::move(h), id.val_);
  }

  template <integral T>
  Entity operator-(T x) const {
    return Entity{val_ - static_cast<int32_t>(x)};
  }

  template <integral T>
  Entity operator+(T x) const {
    return Entity{val_ + static_cast<int32_t>(x)};
  }

  friend std::ostream &operator<<(std::ostream &os, const Entity id) {
    return os << "ID(" << id.val_ << ")";
  }

  template <typename T>
  inline T &Get(std::vector<T> &component_data) const {
    return component_data[val_];
  }

  template <typename T>
  inline const T &Get(const std::vector<T> &component_data) const {
    return component_data[val_];
  }

  // template <OptionalComponent T>
  // inline void Set(std::vector<T> &component_data, const T &value) const {
  //   SetOptionalComponent(this, value, component_data);
  // }

  template <typename T>
  inline void Set(std::vector<T> &component_data, const T &value) const {
    component_data[val_] = value;
  }

  static Entity Nil() { return Entity(-1); }

 private:
  int32_t val_;
};

static_assert(std::is_standard_layout<Entity>());
static_assert(std::is_default_constructible<Entity>());
static_assert(sizeof(Entity) == sizeof(int32_t));

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