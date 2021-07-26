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

enum IntegrationMethod {
  kFirstOrderEuler = 0,
  kVelocityVerlet = 1,
};

// Updates the Motion and Acceleration components, except where kGlued,
// kOrbiting or kDestroyed are in effect. Does not update Position
// (UpdatePositions does that). Call UpdateOrbitalMotion and UpdateGluedMotion
// for objects that don't accelerate freely.
//
// Input must be sorted in ascending order of object ID.
void IntegrateMotion(IntegrationMethod integrator, float dt,
                     absl::Span<Event> input,
                     const std::vector<Position> &positions,
                     const std::vector<Mass> &mass,
                     const std::vector<Flags> &flags,
                     std::vector<Motion> &motion);

// Copies Motion.next_position to Position.value.
void UpdatePositions(const std::vector<Motion> &motion,
                     std::vector<Position> &positions);

Vector3 GravityForceOn(const std::vector<Position> &positions,
                       const std::vector<Mass> &mass,
                       const std::vector<Flags> &flags, int object_id);

Vector3 GravityForceOn(const std::vector<Position> &positions,
                       const std::vector<Mass> &mass,
                       const std::vector<Flags> &flags, int object_id,
                       std::vector<std::pair<int, Vector3>> &contributions);

void IntegrateFirstOrderEuler(float dt, absl::Span<Event> input,
                              const std::vector<Position> &positions,
                              const std::vector<Mass> &mass,
                              const std::vector<Flags> &flags,
                              std::vector<Motion> &motion);

void IntegrateVelocityVerlet(float dt, absl::Span<Event> input,
                             const std::vector<Position> &positions,
                             const std::vector<Mass> &mass,
                             const std::vector<Flags> &flags,
                             std::vector<Motion> &motion);

}  // namespace vstr

#endif
