// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_MOTION_SYSTEM
#define VSTR_MOTION_SYSTEM

#include <absl/types/span.h>

#include <iostream>

#include "component_data.h"

namespace vstr {

class MotionSystem {
 public:
  enum Integrator {
    kFirstOrderEuler,
    kVelocityVerlet,
  };

  explicit MotionSystem(Integrator integrator = kVelocityVerlet)
      : integrator_(integrator) {}

  // Updates the Motion and Acceleration components, except where kGlued,
  // kOrbiting or kDestroyed are in effect. Does not update Position (SecondPass
  // does that). Separate systems update Motion for objects that are otherwise
  // controlled: GlueSystem and OrbitSystem.
  void FirstPass(float dt, absl::Span<Event> input,
                 const std::vector<Position> &positions,
                 const std::vector<Mass> &mass, const std::vector<Flags> &flags,
                 std::vector<Motion> &motion);

  // Copies Motion.next_position to Position.value.
  void SecondPass(const std::vector<Motion> &motion,
                  std::vector<Position> &positions);

  static Vector3 GravityForceOn(const std::vector<Position> &positions,
                                const std::vector<Mass> &mass,
                                const std::vector<Flags> &flags, int object_id);
  static Vector3 GravityComponentsOn(
      const std::vector<Position> &positions, const std::vector<Mass> &mass,
      const std::vector<Flags> &flags, int object_id,
      std::vector<std::pair<int, Vector3>> &contributions);

  inline const Integrator integrator() const { return integrator_; }

 private:
  Integrator integrator_;
};
}  // namespace vstr

#endif
