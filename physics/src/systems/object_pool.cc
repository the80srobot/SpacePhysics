// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "object_pool.h"

namespace vstr {
namespace {

int32_t ClaimFromPool(ReusePool &pool, std::vector<ReuseTag> &reuse_tags) {
  int32_t id = pool.first_id;
  if (id >= 0) {
    int32_t idx = FindOptionalComponent(reuse_tags, id);
    assert(idx >= 0);
    pool.first_id = reuse_tags[idx].next_id;
    reuse_tags[idx].next_id = -1;
    --pool.free_count;
    ++pool.in_use_count;
  }
  assert(pool.free_count == 0 || pool.first_id >= 0);
  return id;
}

void CopyObject(const int32_t dst, const int32_t src, Frame &frame) {
  frame.mass[dst] = frame.mass[src];
  frame.colliders[dst] = frame.colliders[src];
  frame.glue[dst] = frame.glue[src];
  frame.flags[dst] = frame.flags[src];

  CopyOptionalComponent(dst, src, frame.orbits);
  CopyOptionalComponent(dst, src, frame.durability);
  CopyOptionalComponent(dst, src, frame.rockets);
  CopyOptionalComponent(dst, src, frame.triggers);
  CopyOptionalComponent(dst, src, frame.reuse_tags);
}

void ReturnToPool(const int id, const int tag_idx, ReusePool &pool,
                  std::vector<ReuseTag> &reuse_tags) {
  assert(reuse_tags[tag_idx].next_id == -1);
  reuse_tags[tag_idx].next_id = pool.first_id;
  pool.first_id = id;
  ++pool.free_count;
  --pool.in_use_count;
}

}  // namespace

void InitializePool(const int32_t pool_id, const int32_t prototype_id,
                    const int32_t capacity, Frame &frame) {
  const int32_t pool_idx =
      SetOptionalComponent(pool_id,
                           ReusePool{.id = pool_id,
                                     .free_count = 0,
                                     .in_use_count = capacity,
                                     .first_id = -1},
                           frame.reuse_pools);
  assert(pool_idx >= 0);

  const int32_t tag_idx = SetOptionalComponent(
      prototype_id,
      ReuseTag{.id = prototype_id, .next_id = -1, .pool_id = pool_id},
      frame.reuse_tags);

  frame.flags[prototype_id].value |= Flags::kReusable | Flags::kDestroyed;

  for (int i = 0; i < capacity - 1; ++i) {
    int32_t id = frame.Push();
    CopyObject(id, prototype_id, frame);
    ReleaseObject(id, frame.flags, frame.reuse_pools, frame.reuse_tags);
  }
  ReturnToPool(prototype_id, tag_idx, frame.reuse_pools[pool_idx],
               frame.reuse_tags);

  assert(frame.reuse_pools[pool_idx].free_count == capacity);
  assert(frame.reuse_pools[pool_idx].first_id != pool_id);
}

void ReleaseObject(const int32_t id, const std::vector<Flags> &flags,
                   std::vector<ReusePool> &reuse_pools,
                   std::vector<ReuseTag> &reuse_tags) {
  if (!(flags[id].value & Flags::kReusable)) return;

  const int32_t tag_idx = FindOptionalComponent(reuse_tags, id);
  const int32_t pool_idx =
      FindOptionalComponent(reuse_pools, reuse_tags[tag_idx].pool_id);

  assert(pool_idx >= 0);
  assert(tag_idx >= 0);

  ReturnToPool(id, tag_idx, reuse_pools[pool_idx], reuse_tags);
}

void ConvertSpawnAttempts(absl::Span<Event> in_events,
                          std::vector<Event> &out_events, Frame &frame) {
  for (const Event &event : in_events) {
    if (event.type != Event::kSpawnAttempt) continue;
    auto spawn_event = SpawnEventFromPool(event.id, event.position,
                                          event.spawn_attempt.rotation,
                                          event.spawn_attempt.velocity, frame);
    if (spawn_event.ok()) out_events.push_back(spawn_event.value());
  }
}

absl::StatusOr<Event> SpawnEventFromPool(const int32_t pool_id,
                                         const Vector3 &position,
                                         const Quaternion &rotation,
                                         const Vector3 &velocity,
                                         Frame &frame) {
  const int pool_idx = FindOptionalComponent(frame.reuse_pools, pool_id);
  if (pool_idx < 0)
    return absl::InvalidArgumentError("object has no pool component");

  const int tag_id =
      ClaimFromPool(frame.reuse_pools[pool_idx], frame.reuse_tags);
  if (tag_id < 0) {
    return absl::ResourceExhaustedError(
        "no free objects available in the pool");
  }

  return Event(
      tag_id, position,
      Spawn{.pool_id = pool_id, .rotation = rotation, .velocity = velocity});
}

void SpawnObject(const Event &spawn_event, Frame &frame) {
  const int32_t id = spawn_event.id;
  frame.flags[id].value &= ~Flags::kDestroyed;
  frame.transforms[id].position = spawn_event.position;
  frame.transforms[id].rotation = spawn_event.spawn.rotation;
  frame.motion[id] = Motion::FromPositionAndVelocity(
      spawn_event.position, spawn_event.spawn.velocity);

  const int32_t durability_idx = FindOptionalComponent(frame.durability, id);
  if (durability_idx >= 0) {
    frame.durability[durability_idx].value =
        frame.durability[durability_idx].max;
  }
}

}  // namespace vstr
