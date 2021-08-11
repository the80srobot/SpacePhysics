// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_RULES
#define VSTR_RULES

#include <absl/container/flat_hash_map.h>
#include <absl/types/span.h>

#include "systems/component_data.h"

namespace vstr {

struct ApplyDamageParameters {
  int32_t constant;
  float from_impactor_energy;
};

struct BounceParameters {
  // Otherwise and more technically called Coefficient of Restitution, but
  // that's a terrible name.
  float elasticity;
};

// A recipe to generate other events from a collision event.
struct Action {
  // The rule engine doesn't apply these effects directly, instead it emits
  // events that have the desired effect. (E.g. kDestroy will result in a
  // Destruction event.) This indirection exists to enable replay.
  enum Type {
    // Destroy the target. (Results in a Destruction event.)
    kDestroy,
    // Apply damage to the target, if it has a Durability component. (Otherwise
    // do nothing.) (Results in a Damage event.)
    kApplyDamage,
    // Bounce the object using the Newtonian rules for elastic, or semi-elastic
    // collisions. (Resuls in an Acceleration event.)
    kBounce,
    // Stick the target object to the other object. Careless use could result in
    // invalid events (e.g. two objects attached to each other), as the rule
    // engine performs no validation. (Results in a Stick event.)
    kStick,
  };

  // What to do. Some actions have optional extra parameters in the union below.
  Type type;

  // Filters by collision energy.
  float min_speed;
  float max_speed;
  float min_impactor_energy;
  float max_impactor_energy;

  // Parameters for some actions.
  union {
    ApplyDamageParameters apply_damage_parameters;
    BounceParameters bounce_parameters;
  };
};

class RuleSet {
 public:
  // Rules are not symmetric - a rule will affect an object on the first layer,
  // when the former collides with an object on the second layer. (To express a
  // symmetric rule, e.g. where both objects are destroyed, two rules are
  // needed.)
  using LayerPair = std::pair<uint32_t, uint32_t>;

  void Add(LayerPair layer_pair, const Action &action);
  void Apply(const std::vector<Position> &positions,
             const std::vector<Mass> &mass, const std::vector<Motion> &motion,
             const std::vector<Collider> &colliders,
             std::vector<Event> &in_out_events);

 private:
  absl::flat_hash_map<LayerPair, std::vector<Action>> collision_rules_;

  void ApplyToCollision(const std::vector<Position> &positions,
                        const std::vector<Mass> &mass,
                        const std::vector<Motion> &motion,
                        const std::vector<Collider> &colliders,
                        const Event &event, std::vector<Event> &out_events);
};

}  // namespace vstr

#endif