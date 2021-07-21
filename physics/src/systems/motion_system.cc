// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "motion_system.h"

#include "component_data.h"

namespace vstr {
namespace {

Vector3 GravityContributionFrom(const std::vector<Position> &positions,
                                const std::vector<Mass> &mass,
                                const int attractor_id,
                                const Vector3 other_position,
                                const float other_mass) {
  Vector3 f = positions[attractor_id].value - other_position;
  float rSquare = Vector3::SqrMagnitude(f);
  return Vector3::Normalize(f) *
         ((mass[attractor_id].effective + other_mass) / rSquare);
}

Vector3 GravityAt(const std::vector<Position> &positions,
                  const std::vector<Mass> &mass,
                  const std::vector<Flags> &flags, const int id,
                  std::vector<std::pair<int, Vector3>> *contributions) {
  Vector3 result = Vector3{0, 0, 0};
  const int count = positions.size();
  for (int i = 0; i < count; ++i) {
    if (i == id) continue;
    if (mass[i].effective == 0) continue;
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued)) continue;
    const Vector3 f = GravityContributionFrom(
        positions, mass, i, positions[id].value, mass[id].effective);
    result += f;
    if (contributions != nullptr) {
      contributions->push_back(std::make_pair(i, f));
    }
  }

  return result;
}

Vector3 ComputeAcceleration(
    const std::vector<Position> &positions, const std::vector<Mass> &mass,
    const std::vector<Flags> &flags, const int id,
    absl::Span<Event>::const_iterator &input_iter,
    const absl::Span<Event>::const_iterator &input_end) {
  while (input_iter != input_end && input_iter->id < id) {
    ++input_iter;
  }
  Vector3 result;
  if (input_iter != input_end && input_iter->id == id) {
    result = input_iter->input.acceleration;
  } else {
    result = Vector3{0, 0, 0};
  }

  return result + GravityAt(positions, mass, flags, id, nullptr);
}

void IntegrateFirstOrderEuler(const float dt, absl::Span<const Event> input,
                              const std::vector<Position> &positions,
                              const std::vector<Mass> &mass,
                              const std::vector<Flags> &flags,
                              std::vector<Motion> &motion) {
  const int count = positions.size();
  auto input_iter = input.cbegin();
  auto input_end = input.cend();
  for (int i = 0; i < count; ++i) {
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued | Flags::kOrbiting))
      continue;

    motion[i].acceleration =
        ComputeAcceleration(positions, mass, flags, i, input_iter, input_end);
    motion[i].velocity += motion[i].acceleration * dt;
    motion[i].new_position = positions[i].value + motion[i].velocity * dt;
  }
}

void IntegrateVelocityVerlet(const float dt, absl::Span<const Event> input,
                             const std::vector<Position> &positions,
                             const std::vector<Mass> &mass,
                             const std::vector<Flags> &flags,
                             std::vector<Motion> &motion) {
  const int count = positions.size();
  auto input_iter = input.cbegin();
  auto input_end = input.cend();
  const float half_dt = dt * 0.5;
  for (int i = 0; i < count; ++i) {
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued | Flags::kOrbiting))
      continue;

    motion[i].new_position = positions[i].value + motion[i].velocity * dt +
                             motion[i].acceleration * (dt * half_dt);

    Vector3 new_acceleration =
        ComputeAcceleration(positions, mass, flags, i, input_iter, input_end);
    motion[i].velocity += (new_acceleration + motion[i].acceleration) * half_dt;
    motion[i].acceleration = new_acceleration;
  }
}

}  // namespace

void MotionSystem::FirstPass(float dt, absl::Span<const Event> input,
                             const std::vector<Position> &positions,
                             const std::vector<Mass> &mass,
                             const std::vector<Flags> &flags,
                             std::vector<Motion> &motion) {
  switch (integrator_) {
    case kFirstOrderEuler:
      IntegrateFirstOrderEuler(dt, input, positions, mass, flags, motion);
      break;
    case kVelocityVerlet:
      IntegrateVelocityVerlet(dt, input, positions, mass, flags, motion);
      break;
    default:
      assert("invalid integrator");
  }
}

void MotionSystem::SecondPass(const std::vector<Motion> &motion,
                              std::vector<Position> &positions) {
  const int count = positions.size();
  for (int i = 0; i < count; ++i) {
    positions[i].value = motion[i].new_position;
  }
}

Vector3 MotionSystem::GravityForceOn(const std::vector<Position> &positions,
                                     const std::vector<Mass> &mass,
                                     const std::vector<Flags> &flags,
                                     int object_id) {
  return GravityAt(positions, mass, flags, object_id, nullptr);
}

Vector3 MotionSystem::GravityComponentsOn(
    const std::vector<Position> &positions, const std::vector<Mass> &mass,
    const std::vector<Flags> &flags, const int object_id,
    std::vector<std::pair<int, Vector3>> &contributions) {
  return GravityAt(positions, mass, flags, object_id, &contributions);
}

}  // namespace vstr