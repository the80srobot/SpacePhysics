// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "rocket.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace vstr {
namespace {

struct TestCase {
  std::string comment;

  std::vector<Mass> mass;
  std::vector<Rocket> rockets;

  std::vector<Mass> expect_mass;
  std::vector<Rocket> expect_rockets;

  Event event;
  absl::StatusCode status_code;
};

class ApplyRocketRefuelTest : public testing::TestWithParam<TestCase> {};

TEST_P(ApplyRocketRefuelTest, ApplyRocketRefuelTest) {
  std::vector<Mass> mass = GetParam().mass;
  std::vector<Rocket> rockets = GetParam().rockets;
  absl::Status status = ApplyRocketRefuel(GetParam().event, mass, rockets);

  EXPECT_EQ(status.code(), GetParam().status_code) << status;
  EXPECT_THAT(mass, testing::ElementsAreArray(GetParam().expect_mass));
  EXPECT_THAT(rockets, testing::ElementsAreArray(GetParam().expect_rockets));
}

INSTANTIATE_TEST_SUITE_P(
    ApplyRocketRefuelTest, ApplyRocketRefuelTest,
    testing::Values(
        TestCase{
            .comment = "blank",
            .mass = {},
            .rockets = {},
            .expect_mass = {},
            .expect_rockets = {},
            .event = Event(0, {}, RocketRefuel{}),
            .status_code = absl::StatusCode::kNotFound,
        },
        TestCase{
            .comment = "refuel_first_empty_simple",
            .mass =
                {
                    {.inertial = 10},
                },
            .rockets = {{
                .id = 0,
                .fuel_tank_count = 1,
                .fuel_tanks{
                    {.mass_flow_rate = 0, .fuel = 0, .thrust = 0},
                },
            }},
            .expect_mass =
                {
                    {.inertial = 20},
                },
            .expect_rockets =
                {
                    {
                        .id = 0,
                        .fuel_tank_count = 1,
                        .fuel_tanks{
                            {.mass_flow_rate = 1, .fuel = 10, .thrust = 10},
                        },
                    },
                },
            .event = Event(
                0, {0},
                RocketRefuel{
                    .fuel_tank_no = -1,
                    .fuel_tank{.mass_flow_rate = 1, .fuel = 10, .thrust = 10},
                }),
            .status_code = absl::StatusCode::kOk,
        }),
    [](const testing::TestParamInfo<ApplyRocketRefuelTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace

}  // namespace vstr
