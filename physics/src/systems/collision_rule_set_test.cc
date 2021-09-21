// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "collision_rule_set.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "test_matchers/events.h"

namespace vstr {
namespace {

struct Rule {
  CollisionRuleSet::LayerPair layer_pair;
  CollisionEffect action;
};

struct TestCase {
  std::string comment;

  std::vector<Rule> rules;

  std::vector<Transform> positions;
  std::vector<Mass> mass;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Trigger> triggers;

  std::vector<Event> input;
  std::vector<Event> output;
};

class RuleSetTest : public testing::TestWithParam<TestCase> {};

TEST_P(RuleSetTest, RuleSetTest) {
  CollisionRuleSet rule_set;
  for (const auto& rule : GetParam().rules) {
    rule_set.Add(rule.layer_pair, rule.action);
  }
  std::vector<Event> events = GetParam().input;
  rule_set.Apply(GetParam().positions, GetParam().mass, GetParam().motion,
                 GetParam().colliders, GetParam().triggers, events);

  std::vector<Event> output(events.begin() + GetParam().input.size(),
                            events.end());

  EXPECT_THAT(output,
              testing::Pointwise(EventMatches(0.005), GetParam().output));
}

INSTANTIATE_TEST_SUITE_P(
    RuleSetTest, RuleSetTest,
    testing::Values(
        TestCase{
            .comment{"empty"},
            .rules{},
            .positions{},
            .mass{},
            .motion{},
            .colliders{},
            .input{},
            .output{},
        },
        TestCase{
            .comment{"destruction"},
            .rules{{
                .layer_pair{0, 1},
                .action{
                    .type = CollisionEffect::kDestroy,
                    .min_speed = 0,
                    .max_speed = std::numeric_limits<float>::infinity(),
                    .min_impactor_energy = 0,
                    .max_impactor_energy =
                        std::numeric_limits<float>::infinity(),
                },
            }},
            .positions{
                {.position{0, 0, 0}},
                {.position{1, 0, 0}},
            },
            .mass{
                {.inertial = 1, .active = 0},
                {.inertial = 1, .active = 0},
            },
            .motion{
                {.velocity{}, .new_position{}, .acceleration{}},
                {.velocity{}, .new_position{}, .acceleration{}},
            },
            .colliders{
                {.layer = 0, .radius = 1},
                {.layer = 1, .radius = 1},
            },
            .input{
                Event(Vector3{0.5, 0, 0},
                      Collision{
                          .first_id = Entity(0),
                          .second_id = Entity(1),
                          .first_frame_offset_seconds = 0,
                      }),
            },
            .output{
                Event(Entity(0), Vector3{0.5, 0, 0}, Destruction{}),
            },
        },
        TestCase{
            .comment{"destroy_both"},
            .rules{
                {
                    .layer_pair{0, 1},
                    .action{
                        .type = CollisionEffect::kDestroy,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                    },
                },
                {
                    .layer_pair{1, 0},
                    .action{
                        .type = CollisionEffect::kDestroy,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                    },
                },
            },
            .positions{
                {.position{0, 0, 0}},
                {.position{1, 0, 0}},
            },
            .mass{
                {.inertial = 1, .active = 0},
                {.inertial = 1, .active = 0},
            },
            .motion{
                {.velocity{}, .new_position{}, .acceleration{}},
                {.velocity{}, .new_position{}, .acceleration{}},
            },
            .colliders{
                {.layer = 0, .radius = 1},
                {.layer = 1, .radius = 1},
            },
            .input{
                Event(Vector3{0.5, 0, 0},
                      Collision{
                          .first_id = Entity(0),
                          .second_id = Entity(1),
                          .first_frame_offset_seconds = 0,
                      }),
            },
            .output{
                Event(Entity(0), Vector3{0.5, 0, 0}, Destruction{}),
                Event(Entity(1), Vector3{0.5, 0, 0}, Destruction{}),
            },
        },
        TestCase{
            .comment{"bounce_1d_elastic"},
            .rules{
                {
                    .layer_pair{0, 1},
                    .action{
                        .type = CollisionEffect::kBounce,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                        .bounce_parameters{
                            .elasticity = 1,
                        },
                    },
                },
            },
            .positions{
                {.position{0, 0, 0}},
                {.position{2, 0, 0}},
            },
            .mass{
                {.inertial = 1, .active = 0},
                {.inertial = 1, .active = 0},
            },
            .motion{
                {.velocity{1, 0, 0}, .new_position{}, .acceleration{}},
                {.velocity{-1, 0, 0}, .new_position{}, .acceleration{}},
            },
            .colliders{
                {.layer = 0, .radius = 1},
                {.layer = 1, .radius = 1},
            },
            .input{
                Event(Vector3{1, 0, 0},
                      Collision{
                          .first_id = Entity(0),
                          .second_id = Entity(1),
                          .first_frame_offset_seconds = 0,
                      }),
            },
            .output{Event(Entity(0), Vector3{1, 0, 0},
                          Teleportation{.new_position{-0.005, 0, 0},
                                        .new_velocity{-1, 0, 0}})},
        },
        TestCase{
            .comment{"bounce_2d_transfer_momentum"},
            .rules{
                {
                    .layer_pair{0, 1},
                    .action{
                        .type = CollisionEffect::kBounce,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                        .bounce_parameters{
                            .elasticity = 1,
                        },
                    },
                },
                {
                    .layer_pair{1, 0},
                    .action{
                        .type = CollisionEffect::kBounce,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                        .bounce_parameters{
                            .elasticity = 1,
                        },
                    },
                },
            },
            .positions{
                {.position{-1, 1, 0}},
                {.position{.5, .5, 0}},
            },
            .mass{
                {.inertial = 1, .active = 0},
                {.inertial = 1, .active = 0},
            },
            .motion{
                {.velocity{1, -1, 0}, .new_position{}, .acceleration{}},
                {.velocity{0, 0, 0}, .new_position{}, .acceleration{}},
            },
            .colliders{
                {.layer = 0, .radius = 0.5},
                {.layer = 1, .radius = 0.5},
            },
            .input{
                Event(Vector3{0, 0.5, 0},
                      Collision{
                          .first_id = Entity(0),
                          .second_id = Entity(1),
                          .first_frame_offset_seconds = 0.5,
                      }),
            },
            .output{Event(Entity(0), Vector3{0, 0.5, 0},
                          Teleportation{.new_position{-0.5, 0.5},
                                        .new_velocity{0, -1, 0}}),
                    Event(Entity(1), Vector3{0, 0.5, 0},
                          Teleportation{.new_position{0.5, 0.5, 0},
                                        .new_velocity{1, 0, 0}})},
        },
        TestCase{
            .comment{"bounce_2d_unequal_mass"},
            .rules{
                {
                    .layer_pair{0, 1},
                    .action{
                        .type = CollisionEffect::kBounce,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                        .bounce_parameters{
                            .elasticity = 1,
                        },
                    },
                },
                {
                    .layer_pair{1, 0},
                    .action{
                        .type = CollisionEffect::kBounce,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                        .bounce_parameters{
                            .elasticity = 1,
                        },
                    },
                },
            },
            .positions{
                {.position{-1, 1, 0}},
                {.position{.5, .5, 0}},
            },
            .mass{
                {.inertial = 1, .active = 0},
                {.inertial = 9, .active = 0},
            },
            .motion{
                {.velocity{1, -1, 0}, .new_position{}, .acceleration{}},
                {.velocity{0, 0, 0}, .new_position{}, .acceleration{}},
            },
            .colliders{
                {.layer = 0, .radius = 0.5},
                {.layer = 1, .radius = 0.5},
            },
            .input{
                Event(Vector3{0, 0.5, 0},
                      Collision{
                          .first_id = Entity(0),
                          .second_id = Entity(1),
                          .first_frame_offset_seconds = 0.5,
                      }),
            },
            .output{Event(Entity(0), Vector3{0, 0.5, 0},
                          Teleportation{.new_position{-0.5, 0.5},
                                        .new_velocity{-0.8, -1, 0}}),
                    Event(Entity(1), Vector3{0, 0.5, 0},
                          Teleportation{.new_position{0.5, 0.5, 0},
                                        .new_velocity{0.2, 0, 0}})},
        },
        TestCase{
            .comment{"trigger_destroy_both"},
            .rules{
                {
                    .layer_pair{0, 1},
                    .action{
                        .type = CollisionEffect::kTriggerEvent,
                        .min_speed = 0,
                        .max_speed = std::numeric_limits<float>::infinity(),
                        .min_impactor_energy = 0,
                        .max_impactor_energy =
                            std::numeric_limits<float>::infinity(),
                    },
                },
            },
            .positions{
                {.position{0, 0, 0}},
                {.position{1, 0, 0}},
            },
            .mass{
                {.inertial = 1, .active = 0},
                {.inertial = 1, .active = 0},
            },
            .motion{
                {.velocity{}, .new_position{}, .acceleration{}},
                {.velocity{}, .new_position{}, .acceleration{}},
            },
            .colliders{
                {.layer = 0, .radius = 1},
                {.layer = 1, .radius = 1},
            },
            .triggers{
                {
                    .id = Entity(0),
                    .condition = Trigger::kColission,
                    .target = Trigger::kCollidingObject,
                    .flags = Trigger::kDestroyTrigger,
                    .event = Event(Entity(0), {}, Destruction{}),
                },
            },
            .input{
                Event(Vector3{0.5, 0, 0},
                      Collision{
                          .first_id = Entity(0),
                          .second_id = Entity(1),
                          .first_frame_offset_seconds = 0,
                      }),
            },
            .output{
                Event(Entity(1), Vector3{0.5, 0, 0}, Destruction{}),
                Event(Entity(0), Vector3{0.5, 0, 0}, Destruction{}),
            },
        }),
    [](const testing::TestParamInfo<RuleSetTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace
}  // namespace vstr