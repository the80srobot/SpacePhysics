// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "motion.h"

#include "component_data.h"

namespace vstr {
namespace {

Vector3 GravityContributionFrom(const std::vector<Position> &positions,
                                const std::vector<Mass> &mass,
                                const int attractor_id,
                                const Vector3 other_position) {
  // The force acting on two point masses is F = G×((m_1×m_2) / r²).
  //
  // The acceleration from force on a point mass is a = F / m.
  //
  // As a simplification, we assume G = 1 (the actual value is 11 orders of
  // magnitude less.)
  //
  // So the acceleration of point mass 1 due to gravity from point mass 2 is:
  //
  // a = ((m_1×m_2) / r²) / m_2
  //
  // Which is the same as a = m_1 / r².
  Vector3 f = positions[attractor_id].value - other_position;
  float rSquare = Vector3::SqrMagnitude(f);
  return Vector3::Normalize(f) * ((mass[attractor_id].effective) / rSquare);
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
    const Vector3 f =
        GravityContributionFrom(positions, mass, i, positions[id].value);
    result += f;
    if (contributions != nullptr) {
      contributions->push_back(std::make_pair(i, f));
    }
  }

  return result;
}

Vector3 ComputeAcceleration(const std::vector<Position> &positions,
                            const std::vector<Mass> &mass,
                            const std::vector<Flags> &flags, const int id,
                            absl::Span<Event> input) {
  while (input.size() != 0 && input[0].id < id) {
    input = input.subspan(1);
  }
  Vector3 result;
  if (input.size() != 0 && input[0].id == id) {
    result = input[0].acceleration.value;
  } else {
    result = Vector3{0, 0, 0};
  }

  return result + GravityAt(positions, mass, flags, id, nullptr);
}

}  // namespace

void IntegrateFirstOrderEuler(const float dt, absl::Span<Event> input,
                              const std::vector<Position> &positions,
                              const std::vector<Mass> &mass,
                              const std::vector<Flags> &flags,
                              std::vector<Motion> &motion) {
  const int count = positions.size();
  for (int i = 0; i < count; ++i) {
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued | Flags::kOrbiting))
      continue;

    motion[i].acceleration =
        ComputeAcceleration(positions, mass, flags, i, input);
    motion[i].velocity += motion[i].acceleration * dt;
    motion[i].new_position = positions[i].value + motion[i].velocity * dt;
  }
}

void IntegrateVelocityVerlet(const float dt, absl::Span<Event> input,
                             const std::vector<Position> &positions,
                             const std::vector<Mass> &mass,
                             const std::vector<Flags> &flags,
                             std::vector<Motion> &motion) {
  const int count = positions.size();
  const float half_dt = dt * 0.5;
  for (int i = 0; i < count; ++i) {
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued | Flags::kOrbiting))
      continue;

    motion[i].new_position = positions[i].value + motion[i].velocity * dt +
                             motion[i].acceleration * (dt * half_dt);

    Vector3 new_acceleration =
        ComputeAcceleration(positions, mass, flags, i, input);
    motion[i].velocity += (new_acceleration + motion[i].acceleration) * half_dt;
    motion[i].acceleration = new_acceleration;
  }
}

void Accelerate(IntegrationMethod integrator, float dt, absl::Span<Event> input,
                const std::vector<Position> &positions,
                const std::vector<Mass> &mass, const std::vector<Flags> &flags,
                std::vector<Motion> &motion) {
  switch (integrator) {
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

void UpdatePositions(const std::vector<Motion> &motion,
                     std::vector<Position> &positions) {
  const int count = positions.size();
  for (int i = 0; i < count; ++i) {
    positions[i].value = motion[i].new_position;
  }
}

Vector3 GravityForceOn(const std::vector<Position> &positions,
                       const std::vector<Mass> &mass,
                       const std::vector<Flags> &flags, int object_id) {
  return GravityAt(positions, mass, flags, object_id, nullptr);
}

Vector3 GravityForceOn(const std::vector<Position> &positions,
                       const std::vector<Mass> &mass,
                       const std::vector<Flags> &flags, const int object_id,
                       std::vector<std::pair<int, Vector3>> &contributions) {
  return GravityAt(positions, mass, flags, object_id, &contributions);
}

}  // namespace vstr