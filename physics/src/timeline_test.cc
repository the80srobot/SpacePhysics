// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "timeline.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <random>

#include "absl/container/flat_hash_map.h"
#include "systems/object_pool.h"
#include "test_matchers/vector3.h"
#include "types/required_components.h"

namespace vstr {
namespace {

TEST(TimelineTest, FallingSphere) {
  const float dt = 0.001;
  // The spheres should take about 111 seconds to come into contact.
  const float duration = 111;

  Frame initial_frame;
  Entity massless_sphere = initial_frame.Push();
  Entity massive_sphere = initial_frame.Push();

  massless_sphere.Set(initial_frame.transforms,
                      Transform{.position{0, 100, 0}});
  massless_sphere.Set(initial_frame.colliders,
                      Collider{.radius = 1, .layer = 1});
  massive_sphere.Set(initial_frame.transforms, Transform{0, 200, 0});
  massive_sphere.Set(initial_frame.colliders,
                     Collider{.radius = 1, .layer = 1});
  massive_sphere.Set(initial_frame.mass, Mass{.active = 100, .inertial = 100});

  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, {}, dt, 30);

  int frame_no = 0;
  for (float t = 0; t < duration; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  // At this point we should be in the same state, at head frame, as the
  // FallingSphere PipelineTest.
  const Frame* frame = timeline.GetFrame(frame_no);
  EXPECT_NE(frame, nullptr);
  EXPECT_LT(frame->transforms[0].position.y, 200);
  EXPECT_GT(frame->transforms[0].position.y, 199);

  // A collision event should have been recorded by now.
  std::vector<Event> buffer;
  EXPECT_TRUE(timeline.GetEvents(frame_no, buffer));
  ASSERT_GE(buffer.size(), 1);
  EXPECT_EQ(buffer[0].type, Event::kCollision);
  EXPECT_EQ(buffer[0].id, massless_sphere);
  EXPECT_EQ(buffer[0].collision.second_id, massive_sphere);
  EXPECT_NE(buffer[0].collision.first_frame_offset_seconds, 0);

  // Simulate some way into the future, then look back.
  for (float t = 0; t < duration; t += dt) {
    timeline.Simulate();
  }

  frame = timeline.GetFrame(frame_no);
  EXPECT_NE(frame, nullptr);
  EXPECT_LT(massless_sphere.Get(frame->transforms).position.y, 200);
  EXPECT_GT(massless_sphere.Get(frame->transforms).position.y, 199);
}

TEST(TimelineTest, AccelerateRewindAccelerate) {
  const float dt = 0.01;

  Frame initial_frame;
  Entity rocket = initial_frame.Push();
  Entity planet = initial_frame.Push();

  rocket.Set(initial_frame.transforms, Transform{.position{0, 100, 0}});
  rocket.Set(initial_frame.colliders, Collider{.layer = 1, .radius = 1});
  planet.Set(initial_frame.transforms, Transform{.position{0, 0, 0}});
  planet.Set(initial_frame.colliders, Collider{.layer = 1, .radius = 1});

  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, {}, dt, 30);

  // One-second 10 ms/s/s burn in the direction of the planet. After 1 second,
  // the speed should be 10 m/s.
  timeline.InputEvent(1, 1.0f / dt,
                      Event(rocket, {}, Acceleration{Vector3{0, -10, 0}}));

  // After two seconds the rocket should be on its way towards sphere 1.
  int frame_no = 0;
  for (float t = 0; t < 2; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  const Frame* frame = timeline.GetFrame(frame_no);
  ASSERT_NE(frame, nullptr);
  EXPECT_THAT(rocket.Get(frame->motion).velocity,
              Vector3ApproxEq(Vector3{0, -10, 0}));

  // Rewind the clock to 0.5 second and burn in the opposite direction. The
  // resulting speed should be 0.
  timeline.InputEvent(0.5f / dt + 1, 1.0f / dt,
                      Event(rocket, {}, Acceleration{Vector3{0, 10, 0}}));
  // At the two-second mark, the speed should be 0 - simulate until the frame
  // at 2 seconds in is available.
  frame_no = 2.0f / dt;
  for (frame = nullptr; frame == nullptr; frame = timeline.GetFrame(frame_no)) {
    timeline.Simulate();
  }

  ASSERT_NE(frame, nullptr);
  EXPECT_THAT(rocket.Get(frame->motion).velocity,
              Vector3ApproxEq(Vector3{0, 0, 0}));
}

TEST(TimelineTest, TimeTravel) {
  const float dt = 0.1;
  const int key_frame_every = 5;
  Frame initial_frame;

  const Entity spaceship = initial_frame.Push();
  spaceship.Set(
      initial_frame.transforms,
      Transform{.position{100, 0, 0}, .rotation = Quaternion::Identity()});
  spaceship.Set(initial_frame.mass, Mass{.active = 0, .inertial = 10});
  spaceship.Set(initial_frame.colliders,
                Collider{.center{0, 0, 0}, .layer = 1, .radius = 1});
  spaceship.Set(initial_frame.motion, Motion{.velocity{0, 0, 0}});
  spaceship.Set(initial_frame.rockets,
                Rocket{.fuel_tank_count = 2,
                       .fuel_tanks = {
                           {.fuel = 1, .mass_flow_rate = 1, .thrust = 10},
                           {.fuel = 1, .mass_flow_rate = 1, .thrust = 10},
                       }});

  const Entity planet = initial_frame.Push();
  planet.Set(initial_frame.transforms, Transform{.position{0, 0, 0}});
  planet.Set(initial_frame.mass,
             Mass{.active = 1000, .inertial = 1000, .cutoff_distance = 1000});
  planet.Set(initial_frame.colliders,
             Collider{.center{0, 0, 0}, .layer = 2, .radius = 10});
  planet.Set(initial_frame.motion,
             Motion{.spin = Quaternion::FromAngle(
                        {0, 0, 1}, 10 * Quaternion::kRadiansPerDeg / dt)});

  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, {}, dt, key_frame_every,
                    kFirstOrderEuler);

  timeline.InputEvent(
      10, 19,
      Event(spaceship, {}, RocketBurn{.fuel_tank = 0, .thrust{1, 0, 0}}));
  timeline.InputEvent(20, Event(spaceship, {}, TimeTravel{.frame_no = 5}));

  int frame_no;
  for (frame_no = 0; frame_no < 20; ++frame_no) {
    timeline.Simulate();
  }

  const Frame* frame = timeline.GetFrame(19);
  ASSERT_NE(frame, nullptr);
  EXPECT_THAT(spaceship.Get(frame->motion).velocity,
              Vector3ApproxEq(Vector3{10, 0, 0}, 0.5));

  frame = timeline.GetFrame(20);
  ASSERT_NE(frame, nullptr);
  EXPECT_THAT(spaceship.Get(frame->motion).velocity,
              Vector3ApproxEq(Vector3{0, 0, 0}, 0.5));
}

TEST(TimelineTest, DestroyAttractor) {
  const float dt = 1.0f / 30;

  Frame initial_frame;
  Entity rock = initial_frame.Push();
  Entity attractor = initial_frame.Push();

  rock.Set(initial_frame.transforms, Transform{.position{0, 100, 0}});
  rock.Set(initial_frame.colliders, Collider{.layer = 1, .radius = 1});

  attractor.Set(initial_frame.transforms, Transform{.position{0, 0, 0}});
  attractor.Set(initial_frame.colliders, Collider{.layer = 1, .radius = 1});
  attractor.Set(initial_frame.mass, Mass{.active = 100, .inertial = 100});

  LayerMatrix matrix({{1, 1}});

  Timeline timeline(initial_frame, 0, matrix, {}, dt, 30);

  timeline.InputEvent(30.0f / dt, Event(Entity(1), {}, Destruction{}));

  int frame_no = 0;
  for (float t = 0; t < 40.0f; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  // The massive sphere should stop existing after 30 seconds. After that,
  // gone should be the gravitational force, and no collision should occur.
  const Frame* frame = timeline.GetFrame(30.0f / dt);
  ASSERT_NE(frame, nullptr);
  EXPECT_FALSE(attractor.Get(frame->flags).value & Flags::kDestroyed);
  EXPECT_THAT(rock.Get(frame->motion).acceleration,
              Vector3ApproxEq(Vector3{
                  0,
                  -100 / Vector3::SqrMagnitude(
                             rock.Get(initial_frame.transforms).position -
                             attractor.Get(initial_frame.transforms).position),
                  0}));

  frame = timeline.GetFrame(30.0f / dt + 1);
  ASSERT_NE(frame, nullptr);
  EXPECT_TRUE(attractor.Get(frame->flags).value & Flags::kDestroyed);
  EXPECT_EQ(rock.Get(frame->motion).acceleration, (Vector3{0, 0, 0}));

  // Eventually sphere 0 will fall where sphere 1 used to be.
  for (; rock.Get(frame->transforms).position.y > 1; ++frame_no) {
    timeline.Simulate();
    frame = timeline.GetFrame(frame_no);
  }

  // But no collision should be recorded.
  frame = timeline.GetFrame(frame_no);
  ASSERT_NE(frame, nullptr);
  EXPECT_GT(rock.Get(frame->transforms).position.y, 0);
  EXPECT_LT(rock.Get(frame->transforms).position.y, 1);
  std::vector<Event> buffer;
  EXPECT_TRUE(timeline.GetEvents(frame_no, buffer));
  EXPECT_EQ(buffer.size(), 0);

  // TODO(adam): Re-enable once there is a way to undestroy an object again.

  // Undestroying the sphere should trigger a collision due to overlap.
  //   timeline.InputEvent(frame_no + 1, Event(1, {}, Destruction{}));

  // We have to take two steps, because events take effect at the end of the
  // frame, which means the attractor still cannot participate in collisions
  // on the first frame.
  //   timeline.Simulate();
  //   ++frame_no;
  //   timeline.Simulate();
  //   ++frame_no;

  //   EXPECT_TRUE(timeline.GetEvents(frame_no, buffer));
  //   EXPECT_GE(buffer.size(), 1);
  //   EXPECT_EQ(buffer[0].type, Event::kCollision);
  //   EXPECT_EQ(buffer[0].id, 0);
  //   EXPECT_EQ(buffer[0].collision.second_id, 1);
}

// Spawns a bunch of asteroids from a pool and tests that they get correctly
// destroyed on collisions.
TEST(TimelineTest, ObjectPoolCollisions) {
  const float dt = 1.0f / 30;
  Frame initial_frame;
  const Entity asteroid_pool_id = initial_frame.Push();
  const Entity asteroid_prototype_id =
      initial_frame.Push(Transform{}, Mass{.inertial = 10}, Motion{},
                         Collider{.layer = 1, .radius = 0.5}, Glue{}, Flags{});
  asteroid_prototype_id.Set(initial_frame.durability,
                            Durability{.value = 2, .max = 2});
  InitializePool(asteroid_pool_id, asteroid_prototype_id, 8, initial_frame);

  // Big attractor in the middle - asteroids should bounce off its surface,
  // colliding with it and becoming damaged in the process. Any asteroids that
  // manage to move out of max range are also destroyed directly.
  const float kMaxRange = 1000;
  const Entity attractor_id = initial_frame.Push(
      Transform{.position{0, 0, 0}},
      Mass{.inertial = 9999, .active = 9999, .cutoff_distance = kMaxRange},
      Motion{}, Collider{.layer = 1, .radius = 5}, Glue{},
      Flags{.value = Flags::kOrbiting});

  // An empty orbit fixes the attractor in place, even during collisions.
  attractor_id.Set(initial_frame.orbits, Orbit{});

  LayerMatrix matrix({{1, 1}});
  CollisionRuleSet rules;
  rules.Add({1, 1},
            CollisionEffect{
                .type = CollisionEffect::kApplyDamage,
                .apply_damage_parameters{.constant = 1},
                .min_speed = 0,
                .max_speed = std::numeric_limits<float>::infinity(),
                .min_impactor_energy = 0,
                .max_impactor_energy = std::numeric_limits<float>::infinity(),
            });
  rules.Add({1, 1},
            CollisionEffect{
                .type = CollisionEffect::kBounce,
                .bounce_parameters{.elasticity = 1},
                .min_speed = 0,
                .max_speed = std::numeric_limits<float>::infinity(),
                .min_impactor_energy = 0,
                .max_impactor_energy = std::numeric_limits<float>::infinity(),
            });

  Timeline timeline(initial_frame, 0, matrix, rules, dt, 30, kFirstOrderEuler);

  // Run until 100 collisions occur. Whenever a collision happens, check that
  // objects are destroyed after the second time they collide with something,
  // and that this frees up free_count in the pool.
  std::vector<Event> events;
  int collisions = 0;
  int live_asteroids = 0;
  std::mt19937 random_generator;
  std::uniform_real_distribution<float> position_distribution(-10, 10);
  absl::flat_hash_map<Entity, int> asteroids;
  for (int frame_no = 1; collisions < 100; ++frame_no) {
    if (frame_no > 100.0f / dt) {
      FAIL() << "collision_count=" << collisions << " after 100 s "
             << " (waiting for 100 collisions)";
    }

    int pending_spawn_attempts = 0;
    while (live_asteroids < initial_frame.reuse_pools[0].free_count) {
      timeline.InputEvent(
          frame_no, Event(asteroid_pool_id,
                          Vector3{position_distribution(random_generator),
                                  position_distribution(random_generator),
                                  position_distribution(random_generator)},
                          SpawnAttempt{}));
      ++live_asteroids;
      ++pending_spawn_attempts;
    }

    timeline.Simulate();

    events.clear();
    timeline.GetEvents(frame_no, events);

    // Make sure all spawn attempts succeed.
    for (const Event& event : events) {
      if (event.type != Event::kSpawn) continue;
      // Asteroids start with two hit points.
      asteroids[event.id] = 2;
      --pending_spawn_attempts;
    }
    ASSERT_EQ(pending_spawn_attempts, 0);

    // Instead of listening for destruction events, monitor the outcomes -
    // each object should be destroyed after the second collision.
    for (const Event& event : events) {
      if (event.type != Event::kCollision) continue;
      ++collisions;

      auto it = asteroids.find(event.collision.first_id);
      if (it != asteroids.end()) {
        if (--it->second == 0) asteroids.erase(it);
      }

      it = asteroids.find(event.collision.second_id);
      if (it != asteroids.end()) {
        if (--it->second == 0) asteroids.erase(it);
      }
    }
    live_asteroids = asteroids.size();
  }
}

struct TestCase {
  const std::string comment;
  const int resolution;
  // The test will allocate a buffer of appropriate size.
  const std::vector<Timeline::Trajectory> input;
  const std::vector<std::vector<Vector3>> expect;
  const absl::StatusCode expect_code;
};

class QueryTest : public testing::TestWithParam<TestCase> {};

TEST_P(QueryTest, QueryTest) {
  std::vector<Transform> positions{
      Transform{Vector3{0, 100, 0}},
      Transform{Vector3{0, 0, 0}},
      Transform{Vector3{100, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{10000, 10000},
      Mass{},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
      Motion{},
  };
  std::vector<Collider> colliders{
      Collider{1, 1},
      Collider{1, 1},
      Collider{1, 1},
  };
  std::vector<Glue> glue{
      Glue{},
      Glue{},
      Glue{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
      Flags{},
  };

  // Allocate the storage for the Trajectory queries.
  std::vector<std::vector<Vector3>> storage;
  std::vector<Timeline::Trajectory> queries = GetParam().input;
  for (auto& query : queries) {
    storage.push_back(std::vector<Vector3>(query.buffer_sz));
    query.buffer = storage.back().data();
  }

  Frame initial_frame{positions, mass, motion, colliders, glue, flags};
  LayerMatrix matrix({{1, 1}});

  const float dt = 0.1;  // 10 FPS
  Timeline timeline(initial_frame, 0, matrix, {}, dt, 30);

  // One-second 1 ms/s/s burn in the direction away from the attractor should
  // exactly cancel the gravitational pull for the fist 10 frames.
  timeline.InputEvent(1, 1.0f / dt,
                      Event(Entity(0), {}, Acceleration{Vector3{1, 0, 0}}));

  // Simulate 10 seconds = 100 frames.
  int frame_no = 0;
  for (float t = 0; t < 10; t += dt) {
    timeline.Simulate();
    ++frame_no;
  }

  absl::Status status =
      timeline.Query(GetParam().resolution, absl::MakeSpan(queries));
  EXPECT_EQ(status.code(), GetParam().expect_code) << status;
  for (int i = 0; i < GetParam().expect.size(); ++i) {
    for (int j = 0; j < queries[i].buffer_sz; ++j) {
      EXPECT_THAT(queries[i].buffer[j],
                  Vector3ApproxEq(GetParam().expect[i][j], 0.1))
          << "query #" << i << " element #" << j;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    QueryTest, QueryTest,
    testing::Values(
        TestCase{
            "empty_query",
            1,
            std::vector<Timeline::Trajectory>{},
            std::vector<std::vector<Vector3>>{},
            absl::StatusCode::kOk,
        },
        TestCase{
            "object_1_position",
            2,
            std::vector<Timeline::Trajectory>{
                Timeline::Trajectory{
                    1,
                    0,
                    Timeline::Trajectory::kPosition,
                    5,
                },
            },
            std::vector<std::vector<Vector3>>{
                std::vector<Vector3>{
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{0, 0, 0},
                },
            },
            absl::StatusCode::kOk,
        },
        TestCase{
            "object_2_position_misaligned_query",
            2,
            std::vector<Timeline::Trajectory>{
                Timeline::Trajectory{
                    1,
                    1,
                    Timeline::Trajectory::kPosition,
                    5,
                },
            },
            std::vector<std::vector<Vector3>>{},
            absl::StatusCode::kInvalidArgument,
        },
        TestCase{
            "object_2_position_and_velocity",
            10,
            std::vector<Timeline::Trajectory>{
                Timeline::Trajectory{
                    2,
                    0,
                    static_cast<Timeline::Trajectory::Attribute>(
                        Timeline::Trajectory::kPosition |
                        Timeline::Trajectory::kVelocity),
                    10,
                },
            },
            std::vector<std::vector<Vector3>>{
                std::vector<Vector3>{
                    // These approximate values fall out of the equation of the
                    // falling body.
                    Vector3{100, 0, 0},
                    Vector3{0, 0, 0},
                    Vector3{99.5, 0, 0},
                    Vector3{-0.95, 0, 0},
                    Vector3{98, 0, 0},
                    Vector3{-2, 0, 0},
                    Vector3{95.6, 0, 0},
                    Vector3{-3, 0, 0},
                    Vector3{92, 0, 0},
                    Vector3{-4.1, 0, 0},
                },
            },
            absl::StatusCode::kOk,
        }),
    [](const testing::TestParamInfo<QueryTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace
}  // namespace vstr
