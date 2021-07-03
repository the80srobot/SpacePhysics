// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_MOTION_SYSTEM
#define VSTR_MOTION_SYSTEM

#include <iostream>

#include "component_data.h"

namespace vstr {

class MotionSystem {
 public:
  enum Integrator {
    kFirstOrderEuler,
    kVelocityVerlet,
  };

  explicit MotionSystem(Integrator integrator) : integrator_(integrator) {}

  // Updates the Motion and Acceleration components, except where Glue, Orbit or
  // Destroyed are in effect. Does not update Position (SecondPass does that).
  // Separate systems update Motion for objects that are otherwise controlled:
  // GlueSystem and OrbitSystem.
  void FirstPass(float dt, const std::vector<Input> &input, Frame &frame);

  // Copies Motion.next_position to Position.value.
  void SecondPass(Frame &frame);

  static Vector3 GravityForceOn(const Frame &frame, int object_id);
  static Vector3 GravityComponentsOn(
      const Frame &frame, int object_id,
      std::vector<std::pair<int, Vector3>> &contributions);

 private:
  Integrator integrator_;
};
}  // namespace vstr

#endif
