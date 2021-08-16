// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "rules.h"

namespace vstr {

namespace {

void Bounce(const Event &collision, const BounceParameters params,
            const std::vector<Transform> &transforms,
            const std::vector<Collider> &colliders,
            const std::vector<Motion> &motion, const std::vector<Mass> &mass,
            std::vector<Event> &out_events) {
  const float kSeparationEpsilon = 0.005f;

  // v_a refers to the velocity of the object we're operating on, while v_b the
  // velocity of the object being collided with.
  const float t = collision.collision.first_frame_offset_seconds;
  const Vector3 v_a = motion[collision.collision.first_id].velocity;
  const Vector3 v_b = motion[collision.collision.second_id].velocity;

  // Positions at the time of collision.
  Vector3 a = transforms[collision.collision.first_id].position + v_a * t;
  Vector3 b = transforms[collision.collision.second_id].position + v_b * t;

  // If A and B are very close, or even occupy the same space, most of the below
  // vector operations will be inaccurate or have undefined results. This should
  // basically never happen, because if A and B are set to bounce on contact,
  // then they could only ever be this close if the collider radii are tiny, or
  // if they started out that way. In either case, the best option is to just
  // push them apart.
  if (Vector3::Approximately(a, b)) {
    // Push A away from B. Note that this rule might get applied in both
    // directions, so care must be taken to avoid pushing both in the same
    // direction.
    if (collision.collision.first_id < collision.collision.second_id) {
      a.x += kSeparationEpsilon;
    } else {
      a.x -= kSeparationEpsilon;
    }
  }

  // Since the colliders are spheres, the collision normal lies along the line
  // connecting the second collider's focus with the point of contact.
  Vector3 n = a - b;
  // Closing velocity and the dot product of the the normal and velocity.
  Vector3 v = v_a - v_b;
  float dot = Vector3::Dot(n, v);

  float m_a = mass[collision.collision.first_id].inertial;
  float m_b = mass[collision.collision.second_id].inertial;
  // The direction of the bounce is -R. The magnitude is determined by the ratio
  // of the inertial mass of both objects (the lighter object gets more speed)
  // and the elasticity. Recall that overall momentum is conserved.
  float total_mass = m_a + m_b;

  // If both objects have negligible mass, then treat them as each having
  // equally negligible mass. The specific values below are arbitrary.
  if (total_mass == 0) {
    total_mass = 1;
    m_a = 0.5f;
    m_b = 0.5f;
  }

  // The new velocity vector – momentum is transferred along the line of
  // collision, but not along the tangent.
  const Vector3 new_v =
      v_a - ((2 * m_b) / total_mass) * (dot / Vector3::SqrMagnitude(n)) * n;

  // During off-center collisions, angular momentum is also exchanged. How much
  // depends on the angle between the collision normal and the closing velocity:
  // when the two vectors are parallel no angular momentum is imparted. When
  // they are orthogonal, the entire angular momentum of L = r_a×m_b×|v| will be
  // conferred to object A.
  //
  // In real collisions, conversion of angular momentum into angular velocity
  // requires something called the moment of inertia, or the inertia tensor.
  // This code is basically a big hack to get things looking alright by
  // eyeballing the quantities involved.
  float s = Vector3::Magnitude(v);
  const float r_a = colliders[collision.collision.first_id].radius;
  float angle = std::acosf(dot / (Vector3::Magnitude(n) * s));
  float rate = std::sinf(angle);
  Quaternion spin = motion[collision.collision.first_id].spin;
  if (rate > 0.005f) {
    float L = r_a * m_b * s;
    Vector3 axis = Vector3::Normalize(Vector3::Cross(v, n));
    // Hack alert: this code will be called once for the AxB and once for the
    // BxA side of the collision. The thing is, we don't know which is which,
    // and the normal and closing velocity end up being inverse, so in each case
    // we end up with both objects rotating in the same relative direction (the
    // cross product is always "up"). To get around this, we need to define a
    // global "up" vector, which is what the next three lines amount to.
    //
    // TODO(Adam): How does this behave when the collision is in-plane with the
    // arbitrary "up" vector?
    if (Vector3::Dot({1, 0, 0}, n) > 0) {
      axis *= -1;
    }

    spin *= Quaternion::FromAngle(axis, (L / m_a) * rate);
  }

  out_events.push_back(
      Event(collision.id, collision.position,
            Teleportation{
                .new_position = a + Vector3::Normalize(n) * kSeparationEpsilon,
                .new_velocity = params.elasticity * new_v,
                .new_spin = spin}));
}

void ApplyDamage(const Event &collision, const ApplyDamageParameters params,
                 const float impactor_energy, std::vector<Event> &out_events) {
  out_events.push_back(
      Event(collision.id, collision.position,
            Damage{params.constant +
                   static_cast<int32_t>(params.from_impactor_energy *
                                        impactor_energy)}));
}

Event InvertCollision(const Event &event) {
  Event cpy(event);
  cpy.id = event.collision.second_id;
  cpy.collision.second_id = cpy.collision.first_id;
  cpy.collision.first_id = cpy.id;
  return cpy;
}

}  // namespace

void RuleSet::Add(LayerPair layer_pair, const Action &action) {
  collision_rules_[layer_pair].push_back(action);
}

void RuleSet::Apply(const std::vector<Transform> &transforms,
                    const std::vector<Mass> &mass,
                    const std::vector<Motion> &motion,
                    const std::vector<Collider> &colliders,
                    std::vector<Event> &in_out_events) {
  int limit = in_out_events.size();
  for (int i = 0; i < limit; ++i) {
    const Event &event = in_out_events[i];
    if (event.type != Event::kCollision) continue;
    // Apply once in either direction.
    ApplyToCollision(transforms, mass, motion, colliders, event, in_out_events);
    ApplyToCollision(transforms, mass, motion, colliders,
                     InvertCollision(event), in_out_events);
  }
}

void RuleSet::ApplyToCollision(const std::vector<Transform> &transforms,
                               const std::vector<Mass> &mass,
                               const std::vector<Motion> &motion,
                               const std::vector<Collider> &colliders,
                               const Event &event,
                               std::vector<Event> &out_events) {
  const auto it = collision_rules_.find(
      std::make_pair(colliders[event.collision.first_id].layer,
                     colliders[event.collision.second_id].layer));

  if (it == collision_rules_.end()) return;

  // These are the same for the inverse event. As optimization, we could compute
  // them once and pass to both invocations, if it turns out we need to shave
  // off a sqrt op here.
  const float impact_speed_sqr =
      Vector3::SqrMagnitude(motion[event.collision.first_id].velocity -
                            motion[event.collision.second_id].velocity);
  const float impact_speed = sqrtf(impact_speed_sqr);
  const float impactor_energy =
      0.5 * impact_speed_sqr * mass[event.collision.second_id].inertial;

  for (const Action &action : it->second) {
    if (impact_speed < action.min_speed || impact_speed > action.max_speed)
      continue;
    if (impactor_energy < action.min_impactor_energy ||
        impactor_energy > action.max_impactor_energy)
      continue;

    switch (action.type) {
      case Action::kApplyDamage:
        ApplyDamage(event, action.apply_damage_parameters, impactor_energy,
                    out_events);
        break;
      case Action::kBounce:
        Bounce(event, action.bounce_parameters, transforms, colliders, motion,
               mass, out_events);
        break;
      case Action::kDestroy:
        out_events.push_back(Event(event.id, event.position, Destruction{1}));
        break;
      case Action::kStick:
        // TODO
        break;
      default:
        assert("unreachable");
        break;
    }
  }
}

}  // namespace vstr