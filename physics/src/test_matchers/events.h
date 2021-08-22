// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TEST_MATCHERS_EVENTS
#define VSTR_TEST_MATCHERS_EVENTS

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "test_matchers/vector3.h"
#include "types/required_components.h"

namespace vstr {

MATCHER_P(EventMatches, epsilon, "approximately matches") {
  const Event& a = std::get<0>(arg);
  const Event& b = std::get<1>(arg);

  if (!(a.id == b.id && a.type == b.type &&
        Vector3::Approximately(a.position, b.position, epsilon))) {
    return false;
  }

  switch (a.type) {
    case Event::kAcceleration:
      return Vector3::Approximately(a.acceleration.linear,
                                    b.acceleration.linear, epsilon);
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
      return Vector3::Approximately(a.teleportation.new_position,
                                    b.teleportation.new_position, epsilon) &&
             Vector3::Approximately(a.teleportation.new_velocity,
                                    b.teleportation.new_velocity, epsilon);
    case Event::kRocketBurn:
      return Vector3::Approximately(a.rocket_burn.thrust, b.rocket_burn.thrust,
                                    epsilon) &&
             a.rocket_burn.fuel_tank == b.rocket_burn.fuel_tank;
    case Event::kRocketRefuel:
      return FloatEq(a.rocket_refuel.fuel_tank.fuel,
                     b.rocket_refuel.fuel_tank.fuel) &&
             FloatEq(a.rocket_refuel.fuel_tank.mass_flow_rate,
                     b.rocket_refuel.fuel_tank.mass_flow_rate) &&
             FloatEq(a.rocket_refuel.fuel_tank.thrust,
                     b.rocket_refuel.fuel_tank.thrust) &&
             a.rocket_refuel.fuel_tank_no == b.rocket_refuel.fuel_tank_no;
    default:
      assert(false);  // Programmer error - unreachable.
      return true;
  }
}

}  // namespace vstr

#endif