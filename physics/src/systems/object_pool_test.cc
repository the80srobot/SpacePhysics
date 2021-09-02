// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "object_pool.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vector>

#include "absl/status/statusor.h"
#include "types/frame.h"

namespace vstr {
namespace {

// Spawns an object in a single step.
absl::StatusOr<int32_t> DoSpawn(Frame &frame, const int32_t pool_id,
                                Vector3 position = {},
                                Quaternion rotation = Quaternion::Identity(),
                                Vector3 velocity = {}) {
  auto spawn_event =
      SpawnEventFromPool(pool_id, position, rotation, velocity, frame);
  if (!spawn_event.ok()) return spawn_event.status();
  SpawnObject(spawn_event.value(), frame);
  return spawn_event.value().id;
}

int32_t PushCannedPrototype(Frame &frame) {
  return frame.Push(
      // Shouldn't matter - spawn should instantiate with new values.
      Transform{
          .position{10, 0, 0},
          .rotation = Quaternion::FromEulerZXY({90, 0, 45}),
      },
      Mass{
          .inertial = 10,
          .active = 0,
      },
      Motion::FromPositionAndVelocity({10, 0, 0}, {-1, 0, 0}),
      Collider{
          .center = {1, 0.5, 0},
          .layer = 10,
          .radius = 3,
      },
      Glue{}, Flags{});
}

// Basic test case, using only core components.
class ObjectPoolBasicTest : public testing::Test {
 protected:
  void SetUp() override {
    pool_id_ = frame_.Push();
    prototype_id_ = PushCannedPrototype(frame_);
    InitializePool(pool_id_, prototype_id_, 8, frame_);
  }

  absl::StatusOr<int32_t> SpawnHelper(
      Vector3 position = {}, Quaternion rotation = Quaternion::Identity(),
      Vector3 velocity = {}) {
    return DoSpawn(frame_, pool_id_, position, rotation, velocity);
  }

  absl::StatusOr<std::vector<int32_t>> SpawnHelper(const int count) {
    std::vector<int32_t> result;
    result.reserve(count);
    for (int i = 0; i < count; ++i) {
      auto id = SpawnHelper();
      if (!id.ok()) return id.status();
      result.push_back(id.value());
    }
    return result;
  }

  Frame frame_;
  int32_t pool_id_;
  int32_t prototype_id_;
};

// Tests that a pool can be initialized and objects from it claimed.
TEST_F(ObjectPoolBasicTest, InitializeWithCapacity) {
  ssize_t pool_idx = FindOptionalComponent(frame_.reuse_pools, pool_id_);
  ASSERT_GE(pool_idx, 0);
  EXPECT_EQ(frame_.reuse_pools[pool_idx].free_count, 8);
  EXPECT_EQ(frame_.reuse_pools[pool_idx].in_use_count, 0);
}

TEST_F(ObjectPoolBasicTest, SpawnMultiple) {
  ssize_t pool_idx = FindOptionalComponent(frame_.reuse_pools, pool_id_);
  ASSERT_GE(pool_idx, 0);

  auto ids = SpawnHelper(8);
  ASSERT_TRUE(ids.ok()) << ids.status();
  EXPECT_THAT(ids.value(), testing::SizeIs(8));
  EXPECT_EQ(frame_.reuse_pools[pool_idx].free_count, 0);
  EXPECT_EQ(frame_.reuse_pools[pool_idx].in_use_count, 8);
  EXPECT_THAT(ids.value(), testing::Contains(prototype_id_));
  EXPECT_THAT(ids.value(), testing::Not(testing::Contains(pool_id_)));
  for (int32_t id : ids.value()) {
    EXPECT_TRUE(frame_.flags[id].value & Flags::kReusable);
    EXPECT_FALSE(frame_.flags[id].value & Flags::kDestroyed);
    EXPECT_EQ(frame_.mass[prototype_id_], frame_.mass[id]);
  }
}

// Tests that objects return to the pool correctly.
TEST_F(ObjectPoolBasicTest, ReleaseMultiple) {
  ssize_t pool_idx = FindOptionalComponent(frame_.reuse_pools, pool_id_);
  ASSERT_GE(pool_idx, 0);

  auto ids = SpawnHelper(8);
  ASSERT_TRUE(ids.ok()) << ids.status();
  EXPECT_THAT(ids.value(), testing::SizeIs(8));

  for (int32_t id : ids.value()) {
    ReleaseObject(id, frame_.flags, frame_.reuse_pools, frame_.reuse_tags);
    // ReleaseObject shouldn't by itself set the object to kDestroyed.
    EXPECT_FALSE(frame_.flags[id].value & Flags::kDestroyed);
  }

  EXPECT_EQ(frame_.reuse_pools[pool_idx].free_count, 8);
  EXPECT_EQ(frame_.reuse_pools[pool_idx].in_use_count, 0);
}

TEST(ObjectPoolTest, OptionalComponents) {
  Frame frame;
  int32_t pool_id = frame.Push();
  int32_t prototype_id = PushCannedPrototype(frame);
  SetOptionalComponent(prototype_id, Orbit{.focus = {33, 66, 99}},
                       frame.orbits);
  frame.flags[prototype_id].value |= Flags::kOrbiting;

  InitializePool(pool_id, prototype_id, 8, frame);
  auto id = DoSpawn(frame, pool_id);
  ASSERT_TRUE(id.ok()) << id.status();
  EXPECT_EQ(frame.orbits[FindOptionalComponent(frame.orbits, prototype_id)],
            frame.orbits[FindOptionalComponent(frame.orbits, id.value())]);
  EXPECT_TRUE(frame.flags[id.value()].value & Flags::kOrbiting);
}

}  // namespace
}  // namespace vstr