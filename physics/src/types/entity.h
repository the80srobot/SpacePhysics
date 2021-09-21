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

// Clang concept support is missing std::integral as of Clang 12. Defining these
// to only accept integers is a cheap way to guard against some errors from
// other overloaded operators.
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

}  // namespace vstr
#endif