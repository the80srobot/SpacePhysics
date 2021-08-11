#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "systems/component_data.h"
#include "test_matchers/vector3.h"

namespace vstr {

MATCHER_P(EventMatches, epsilon, "approximately matches") {
  const Event& a = std::get<0>(arg);
  const Event& b = std::get<1>(arg);

  if (!(a.id == b.id && a.type == b.type)) {
    return false;
  }

  switch (a.type) {
    case Event::kAcceleration:
      return Vector3Eq(a.acceleration.value, b.acceleration.value, epsilon);
    case Event::kStick:
      return a.stick == b.stick;
    case Event::kDestruction:
      return a.destruction == b.destruction;
    case Event::kCollision:
      return a.collision.first_id == b.collision.first_id &&
             a.collision.second_id == b.collision.second_id &&
             FloatEq(a.collision.first_frame_offset_seconds,
                     b.collision.first_frame_offset_seconds, epsilon);
    case Event::kDamage:
      return a.damage == b.damage;
    case Event::kTeleportation:
      return Vector3Eq(a.teleportation.new_position,
                       b.teleportation.new_position, epsilon) &&
             Vector3Eq(a.teleportation.new_velocity,
                       b.teleportation.new_velocity);
    default:
      assert(false);  // Programmer error - unreachable.
      return true;
  }
}

}  // namespace vstr