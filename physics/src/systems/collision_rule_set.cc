// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "collision_rule_set.h"

namespace vstr {

namespace {

void ApplyTrigger(const Event &event, const std::vector<Trigger> &triggers,
                  std::vector<Event> &out_events) {
  auto it = std::lower_bound(
      triggers.begin(), triggers.end(), Trigger{.id = event.id},
      [](const Trigger &a, const Trigger &b) { return a.id < b.id; });
  if (it != triggers.end() && it->id == event.id) {
    Event new_event = it->event;
    new_event.position = event.position;
    switch (it->target) {
      case Trigger::kSelf:
        new_event.id = event.id;
        break;
      case Trigger::kCollidingObject:
        new_event.id = event.collision.second_id;
        break;
      default:
        assert("not reachable");
        break;
    }
    out_events.push_back(new_event);

    if (it->flags & Trigger::kDestroyTrigger) {
      out_events.push_back(Event(event.id, event.position, Destruction{}));
    }
  }
}

void Bounce(const Event &event, const CollisionEffect::BounceParameters params,
            const std::vector<Transform> &transforms,
            const std::vector<Collider> &colliders,
            const std::vector<Motion> &motion, const std::vector<Mass> &mass,
            std::vector<Event> &out_events) {
  const float kSeparationEpsilon = 0.005f;

  // v_a refers to the velocity of the object we're operating on, while v_b the
  // velocity of the object being collided with.
  const float t = event.collision.first_frame_offset_seconds;
  const Vector3 v_a = event.collision.first_id.Get(motion).velocity;
  const Vector3 v_b = event.collision.second_id.Get(motion).velocity;

  // Positions at the time of collision.
  Vector3 a = event.collision.first_id.Get(transforms).position + v_a * t;
  Vector3 b = event.collision.second_id.Get(transforms).position + v_b * t;

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
    if (event.collision.first_id < event.collision.second_id) {
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

  float m_a = event.collision.first_id.Get(mass).inertial;
  float m_b = event.collision.second_id.Get(mass).inertial;
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
  const float r_a = event.collision.first_id.Get(colliders).radius;
  float angle = std::acosf(dot / (Vector3::Magnitude(n) * s));
  float rate = std::sinf(angle);
  Quaternion spin = event.collision.first_id.Get(motion).spin;
  if (rate > 0.005f) {
    float L = r_a * m_b * s;
    Vector3 axis = Vector3::Normalize(Vector3::Cross(v, n));
    axis = event.collision.first_id.Get(transforms).rotation * axis;
    spin *= Quaternion::FromAngle(axis, (L / m_a) * rate);
  }

  out_events.push_back(
      Event(event.id, event.position,
            Teleportation{
                .new_position = a + Vector3::Normalize(n) * kSeparationEpsilon,
                .new_velocity = params.elasticity * new_v,
                .new_spin = spin}));
}

void ApplyDamage(const Event &collision,
                 const CollisionEffect::ApplyDamageParameters params,
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

void CollisionRuleSet::Add(LayerPair layer_pair,
                           const CollisionEffect &action) {
  collision_rules_[layer_pair].push_back(action);
}

void CollisionRuleSet::Apply(const std::vector<Transform> &transforms,
                             const std::vector<Mass> &mass,
                             const std::vector<Motion> &motion,
                             const std::vector<Collider> &colliders,
                             const std::vector<Trigger> &triggers,
                             std::vector<Event> &in_out_events) {
  int limit = in_out_events.size();
  for (int i = 0; i < limit; ++i) {
    const Event &event = in_out_events[i];
    if (event.type != Event::kCollision) continue;
    // Apply once in either direction.
    ApplyToCollision(transforms, mass, motion, colliders, triggers, event,
                     in_out_events);
    ApplyToCollision(transforms, mass, motion, colliders, triggers,
                     InvertCollision(event), in_out_events);
  }
}

void CollisionRuleSet::ApplyToCollision(
    const std::vector<Transform> &transforms, const std::vector<Mass> &mass,
    const std::vector<Motion> &motion, const std::vector<Collider> &colliders,
    const std::vector<Trigger> &triggers, const Event &event,
    std::vector<Event> &out_events) {
  const auto it = collision_rules_.find(
      std::make_pair(event.collision.first_id.Get(colliders).layer,
                     event.collision.second_id.Get(colliders).layer));

  if (it == collision_rules_.end()) return;

  // These are the same for the inverse event. As optimization, we could compute
  // them once and pass to both invocations, if it turns out we need to shave
  // off a sqrt op here.
  const float impact_speed_sqr =
      Vector3::SqrMagnitude(event.collision.first_id.Get(motion).velocity -
                            event.collision.second_id.Get(motion).velocity);
  const float impact_speed = sqrtf(impact_speed_sqr);
  const float impactor_energy =
      0.5 * impact_speed_sqr * event.collision.second_id.Get(mass).inertial;

  for (const CollisionEffect &action : it->second) {
    if (impact_speed < action.min_speed || impact_speed > action.max_speed)
      continue;
    if (impactor_energy < action.min_impactor_energy ||
        impactor_energy > action.max_impactor_energy)
      continue;

    switch (action.type) {
      case CollisionEffect::kApplyDamage:
        ApplyDamage(event, action.apply_damage_parameters, impactor_energy,
                    out_events);
        break;
      case CollisionEffect::kBounce:
        Bounce(event, action.bounce_parameters, transforms, colliders, motion,
               mass, out_events);
        break;
      case CollisionEffect::kDestroy:
        out_events.push_back(Event(event.id, event.position, Destruction{}));
        break;
      case CollisionEffect::kStick:
        // TODO
        break;
      case CollisionEffect::kTriggerEvent:
        ApplyTrigger(event, triggers, out_events);
        break;
      default:
        assert("unreachable");
        break;
    }
  }
}

}  // namespace vstr