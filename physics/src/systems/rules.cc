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
            const std::vector<Position> &positions,
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
  Vector3 a = positions[collision.collision.first_id].value + v_a * t;
  Vector3 b = positions[collision.collision.second_id].value + v_b * t;

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
  Vector3 n = Vector3::Normalize(a - b);

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
  const Vector3 new_v = v_a - ((2 * m_b) / total_mass) *
                                  (Vector3::Dot(v_a - v_b, a - b) /
                                   Vector3::SqrMagnitude(a - b)) *
                                  (a - b);

  out_events.push_back(
      Event(collision.id, collision.position,
            Teleportation{/*new_position=*/a + n * kSeparationEpsilon,
                          /*new_velocity=*/params.elasticity * new_v}));
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

void RuleSet::Apply(const std::vector<Position> &positions,
                    const std::vector<Mass> &mass,
                    const std::vector<Motion> &motion,
                    const std::vector<Collider> &colliders,
                    std::vector<Event> &in_out_events) {
  int limit = in_out_events.size();
  for (int i = 0; i < limit; ++i) {
    const Event &event = in_out_events[i];
    if (event.type != Event::kCollision) continue;
    // Apply once in either direction.
    ApplyToCollision(positions, mass, motion, colliders, event, in_out_events);
    ApplyToCollision(positions, mass, motion, colliders, InvertCollision(event),
                     in_out_events);
  }
}

void RuleSet::ApplyToCollision(const std::vector<Position> &positions,
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
        Bounce(event, action.bounce_parameters, positions, colliders, motion,
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