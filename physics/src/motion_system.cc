#include "motion_system.h"

#include "component_data.h"

namespace vstr {
namespace {

Vector3 GravityContributionFrom(const Frame &frame, const int attractor_id,
                                const Vector3 other_position,
                                const float other_mass) {
  Vector3 f = frame.positions[attractor_id].value - other_position;
  float rSquare = Vector3::SqrMagnitude(f);
  return Vector3::Normalize(f) *
         ((frame.mass[attractor_id].effective + other_mass) / rSquare);
}

Vector3 GravityAt(const Frame &frame, const Vector3 position, const float mass,
                  const int skip_id,
                  std::vector<std::pair<int, Vector3>> *contributions) {
  Vector3 result = Vector3{0, 0, 0};
  const int count = frame.positions.size();
  for (int i = 0; i < count; ++i) {
    if (i == skip_id) continue;
    if (frame.mass[i].effective == 0) continue;
    if (frame.destroyed[i].value) continue;
    if (frame.glue[i].parent_id > 0) continue;
    const Vector3 f = GravityContributionFrom(frame, i, position, mass);
    result += f;
    if (contributions != nullptr) {
      contributions->push_back(std::make_pair(i, f));
    }
  }

  return result;
}

Vector3 ComputeAcceleration(
    const Frame &frame, const int id,
    std::vector<Input>::const_iterator &input_iter,
    const std::vector<Input>::const_iterator &input_end) {
  while (input_iter != input_end && input_iter->object_id < id) {
    ++input_iter;
  }
  Vector3 result;
  if (input_iter != input_end && input_iter->object_id == id) {
    result = input_iter->acceleration;
  } else {
    result = Vector3{0, 0, 0};
  }

  return result + GravityAt(frame, frame.positions[id].value,
                            frame.mass[id].effective, id, nullptr);
}

void IntegrateFirstOrderEuler(const float dt, const std::vector<Input> &input,
                              Frame &frame) {
  const int count = frame.positions.size();
  auto input_iter = input.cbegin();
  auto input_end = input.cend();
  for (int i = 0; i < count; ++i) {
    if (frame.glue[i].parent_id > 0) continue;
    if (frame.destroyed[i].value) continue;
    // TODO skip orbits

    frame.acceleration[i].value =
        ComputeAcceleration(frame, i, input_iter, input_end);
    frame.motion[i].velocity += frame.acceleration[i].value * dt;
    frame.motion[i].new_position =
        frame.positions[i].value + frame.motion[i].velocity * dt;
  }
}

void IntegrateVelocityVerlet(const float dt, const std::vector<Input> &input,
                             Frame &frame) {
  const int count = frame.positions.size();
  auto input_iter = input.cbegin();
  auto input_end = input.cend();
  const float half_dt = dt * 0.5;
  for (int i = 0; i < count; ++i) {
    if (frame.glue[i].parent_id > 0) continue;
    if (frame.destroyed[i].value) continue;
    // TODO skip orbits

    frame.motion[i].new_position = frame.positions[i].value +
                                   frame.motion[i].velocity * dt +
                                   frame.acceleration[i].value * (dt * half_dt);

    Vector3 new_acceleration =
        ComputeAcceleration(frame, i, input_iter, input_end);
    frame.motion[i].velocity +=
        (new_acceleration + frame.acceleration[i].value) * half_dt;
    frame.acceleration[i].value = new_acceleration;
  }
}

void FinalizePositions(Frame &frame) {
  const int count = frame.positions.size();
  for (int i = 0; i < count; ++i) {
    frame.positions[i].value = frame.motion[i].new_position;
  }
}

}  // namespace

void MotionSystem::FirstPass(const float dt, const std::vector<Input> &input,
                             Frame &frame) {
  switch (integrator_) {
    case kFirstOrderEuler:
      IntegrateFirstOrderEuler(dt, input, frame);
      break;
    case kVelocityVerlet:
      IntegrateVelocityVerlet(dt, input, frame);
      break;
    default:
      assert("invalid integrator");
  }
}

void MotionSystem::SecondPass(Frame &frame) { FinalizePositions(frame); }

Vector3 MotionSystem::GravityForceOn(const Frame &frame, const int object_id) {
  return GravityAt(frame, frame.positions[object_id].value,
                   frame.mass[object_id].effective, object_id, nullptr);
}

Vector3 MotionSystem::GravityComponentsOn(
    const Frame &frame, const int object_id,
    std::vector<std::pair<int, Vector3>> &contributions) {
  return GravityAt(frame, frame.positions[object_id].value,
                   frame.mass[object_id].effective, object_id, &contributions);
}

}  // namespace vstr