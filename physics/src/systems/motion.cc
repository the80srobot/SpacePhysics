// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "motion.h"

namespace vstr {
namespace {

Vector3 GravityContributionFrom(const std::vector<Transform> &positions,
                                const std::vector<Mass> &mass,
                                const Entity attractor,
                                const Vector3 other_position) {
  // The force acting on two point masses is F = G×((m_1×m_2) / r²).
  //
  // The acceleration from force on a point mass is a = F / m.
  //
  // As a simplification, we assume G = 1, where the actual value is 11 orders
  // of magnitude less. If you don't like it call the cops.
  //
  // So the acceleration of point mass 1 due to gravity from point mass 2 is:
  //
  // a = ((m_1×m_2) / r²) / m_2
  //
  // Which is the same as a = m_1 / r².
  Vector3 d = attractor.Get(positions).position - other_position;
  float r_square = Vector3::SqrMagnitude(d);
  if (attractor.Get(mass).cutoff_distance != 0 &&
      r_square > attractor.Get(mass).cutoff_distance *
                     attractor.Get(mass).cutoff_distance) {
    return Vector3::Zero();
  }
  return Vector3::Normalize(d) * ((attractor.Get(mass).active) / r_square);
}

Vector3 GravityAt(const std::vector<Transform> &positions,
                  const std::vector<Mass> &mass,
                  const std::vector<Flags> &flags, const Entity id,
                  std::vector<std::pair<Entity, Vector3>> *contributions) {
  Vector3 result = Vector3{0, 0, 0};
  const size_t count = positions.size();
  for (size_t i = 0; i < count; ++i) {
    Entity candidate = Entity(i);
    if (candidate == id) continue;
    if (candidate.Get(mass).active == 0) continue;
    if (candidate.Get(flags).value & (Flags::kDestroyed | Flags::kGlued))
      continue;
    const Vector3 f = GravityContributionFrom(positions, mass, candidate,
                                              id.Get(positions).position);
    result += f;
    if (contributions != nullptr && f != Vector3::Zero()) {
      contributions->push_back(std::make_pair(candidate, f));
    }
  }

  return result;
}

void ComputeForces(const std::vector<Transform> &positions,
                   const std::vector<Mass> &mass,
                   const std::vector<Flags> &flags, const Entity id,
                   absl::Span<Event> &input, Vector3 &out_linear_acceleration,
                   Vector3 &out_impulse, Quaternion &out_angular) {
  while (input.size() != 0 && input[0].id < id) {
    input = input.subspan(1);
  }
  out_angular = Quaternion::Identity();
  out_linear_acceleration = Vector3{0, 0, 0};
  out_impulse = Vector3{0, 0, 0};
  while (input.size() != 0 && input[0].id == id) {
    if (input[0].type == Event::kAcceleration) {
      Vector3 value = input[0].acceleration.linear;
      if (input[0].acceleration.flags & Acceleration::Flag::kForce &&
          id.Get(mass).inertial != 0) {
        value /= id.Get(mass).inertial;
      }
      if (input[0].acceleration.flags & Acceleration::Flag::kImpulse) {
        out_impulse += value;
      } else {
        out_linear_acceleration += input[0].acceleration.linear;
        out_angular *= input[0].acceleration.angular;
      }
    }
    input = input.subspan(1);
  }

  out_linear_acceleration += GravityAt(positions, mass, flags, id, nullptr);
}

}  // namespace

void IntegrateFirstOrderEuler(const float dt, absl::Span<Event> input,
                              const std::vector<Transform> &positions,
                              const std::vector<Mass> &mass,
                              const std::vector<Flags> &flags,
                              std::vector<Motion> &motion) {
  const size_t count = positions.size();
  for (size_t i = 0; i < count; ++i) {
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued | Flags::kOrbiting))
      continue;

    Vector3 impulse;
    Quaternion angular_acceleration;
    ComputeForces(positions, mass, flags, Entity(i), input,
                  motion[i].acceleration, impulse, angular_acceleration);
    motion[i].velocity += impulse + motion[i].acceleration * dt;
    motion[i].new_position = positions[i].position + motion[i].velocity * dt;
    if (angular_acceleration != Quaternion::Identity()) {
      motion[i].spin *= Quaternion::Interpolate(Quaternion::Identity(),
                                                angular_acceleration, dt);
    }
  }
}

void IntegrateVelocityVerlet(const float dt, absl::Span<Event> input,
                             const std::vector<Transform> &positions,
                             const std::vector<Mass> &mass,
                             const std::vector<Flags> &flags,
                             std::vector<Motion> &motion) {
  const size_t count = positions.size();
  const float half_dt = dt * 0.5;
  for (size_t i = 0; i < count; ++i) {
    if (flags[i].value & (Flags::kDestroyed | Flags::kGlued | Flags::kOrbiting))
      continue;

    motion[i].new_position = positions[i].position + motion[i].velocity * dt +
                             motion[i].acceleration * (dt * half_dt);

    Vector3 new_acceleration;
    Vector3 impulse;
    Quaternion angular_acceleration;
    ComputeForces(positions, mass, flags, Entity(i), input, new_acceleration,
                  impulse, angular_acceleration);
    motion[i].velocity +=
        (new_acceleration + motion[i].acceleration) * half_dt + impulse;
    motion[i].acceleration = new_acceleration;
    if (angular_acceleration != Quaternion::Identity()) {
      motion[i].spin *= Quaternion::Interpolate(Quaternion::Identity(),
                                                angular_acceleration, dt);
    }
  }
}

void IntegrateMotion(IntegrationMethod integrator, const float dt,
                     absl::Span<Event> input,
                     const std::vector<Transform> &positions,
                     const std::vector<Mass> &mass,
                     const std::vector<Flags> &flags,
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

void UpdatePositions(const float dt, const std::vector<Motion> &motion,
                     const std::vector<Flags> &flags,
                     std::vector<Transform> &transforms) {
  const int count = transforms.size();
  for (int i = 0; i < count; ++i) {
    if (flags[i].value & Flags::kDestroyed) continue;
    transforms[i].position = motion[i].new_position;
    if (motion[i].spin != Quaternion::Identity()) {
      transforms[i].rotation *=
          Quaternion::Interpolate(Quaternion::Identity(), motion[i].spin, dt);
    }
  }
}

Vector3 GravityForceOn(const std::vector<Transform> &positions,
                       const std::vector<Mass> &mass,
                       const std::vector<Flags> &flags, Entity object_id) {
  return GravityAt(positions, mass, flags, object_id, nullptr);
}

Vector3 GravityForceOn(const std::vector<Transform> &positions,
                       const std::vector<Mass> &mass,
                       const std::vector<Flags> &flags, const Entity object_id,
                       std::vector<std::pair<Entity, Vector3>> &contributions) {
  return GravityAt(positions, mass, flags, object_id, &contributions);
}

}  // namespace vstr