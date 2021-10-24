// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "object_pool.h"

namespace vstr {
namespace {

Entity ClaimFromPool(ReusePool &pool, std::vector<ReuseTag> &reuse_tags) {
  Entity id = pool.first_id;
  if (id != Entity::Nil()) {
    ssize_t idx = FindOptionalComponent(reuse_tags, id);
    assert(idx >= 0);
    pool.first_id = reuse_tags[idx].next_id;
    reuse_tags[idx].next_id = Entity::Nil();
    --pool.free_count;
    ++pool.in_use_count;
  }
  assert(pool.free_count == 0 || pool.first_id != Entity::Nil());
  return id;
}

void CopyObject(const Entity dst, const Entity src, Frame &frame) {
  dst.Set(frame.mass, src.Get(frame.mass));
  dst.Set(frame.colliders, src.Get(frame.colliders));
  dst.Set(frame.glue, src.Get(frame.glue));
  dst.Set(frame.flags, src.Get(frame.flags));
  dst.Set(frame.transforms, src.Get(frame.transforms));
  dst.Set(frame.motion, src.Get(frame.motion));

  CopyOptionalComponent(dst, src, frame.orbits);
  CopyOptionalComponent(dst, src, frame.durability);
  CopyOptionalComponent(dst, src, frame.rockets);
  CopyOptionalComponent(dst, src, frame.triggers);
  CopyOptionalComponent(dst, src, frame.reuse_tags);
}

void ReturnToPool(const Entity id, ReuseTag &tag, ReusePool &pool) {
  assert(tag.next_id == Entity::Nil());
  tag.next_id = pool.first_id;
  pool.first_id = id;
  ++pool.free_count;
  --pool.in_use_count;
}

}  // namespace

int32_t InitializePool(const Entity pool_id, const Entity prototype_id,
                       const int32_t capacity, Frame &frame) {
  assert(prototype_id != pool_id);

  ReusePool &pool =
      pool_id.Set(frame.reuse_pools, ReusePool{.free_count = 0,
                                               .in_use_count = capacity,
                                               .first_id = Entity::Nil()});

  prototype_id.Set(frame.reuse_tags,
                   ReuseTag{.next_id = Entity::Nil(), .pool_id = pool_id});

  prototype_id.Get(frame.flags).value |= Flags::kReusable | Flags::kDestroyed;

  for (int i = 0; i < capacity - 1; ++i) {
    Entity id = frame.Push();
    CopyObject(id, prototype_id, frame);
    ReleaseObject(id, frame.flags, frame.reuse_pools, frame.reuse_tags);
  }
  // The dereference of Get's return value is safe because we created the
  // optional component right before the loop. (We cannot reuse the reference
  // returned from Set, because the CopyObject operations in the loop will have
  // invalidated it by now.)
  ReturnToPool(prototype_id, *prototype_id.Get(frame.reuse_tags), pool);

  assert(pool.free_count == capacity);
  assert(pool.first_id != pool_id);

  // TODO: this should return a reference, but the C-compatible API expects an
  // offset. Refactor.
  return FindOptionalComponent(frame.reuse_pools, pool_id);
}

void ReleaseObject(const Entity id, const std::vector<Flags> &flags,
                   std::vector<ReusePool> &reuse_pools,
                   std::vector<ReuseTag> &reuse_tags) {
  assert(id.Get(flags).value & Flags::kReusable);

  ReuseTag *tag = id.Get(reuse_tags);
  assert(tag != nullptr);

  ReusePool *pool = tag->pool_id.Get(reuse_pools);
  assert(pool != nullptr);

  ReturnToPool(id, *tag, *pool);
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

absl::StatusOr<Event> SpawnEventFromPool(const Entity pool_id,
                                         const Vector3 &position,
                                         const Quaternion &rotation,
                                         const Vector3 &velocity,
                                         Frame &frame) {
  ReusePool *pool = pool_id.Get(frame.reuse_pools);
  if (pool == nullptr)
    return absl::InvalidArgumentError("object has no pool component");

  const Entity tag_id = ClaimFromPool(*pool, frame.reuse_tags);
  if (tag_id == Entity::Nil()) {
    return absl::ResourceExhaustedError(
        "no free objects available in the pool");
  }

  return Event(
      tag_id, position,
      Spawn{.pool_id = pool_id, .rotation = rotation, .velocity = velocity});
}

void SpawnObject(const Event &spawn_event, Frame &frame) {
  const Entity id = spawn_event.id;
  id.Get(frame.flags).value &= ~Flags::kDestroyed;
  id.Get(frame.transforms).position = spawn_event.position;
  id.Get(frame.transforms).rotation = spawn_event.spawn.rotation;
  id.Get(frame.motion) = Motion::FromPositionAndVelocity(
      spawn_event.position, spawn_event.spawn.velocity);

  // When spawning objects that have a durability component, heal them to their
  // max hit points.
  Durability *durability = id.Get(frame.durability);
  if (durability != nullptr) {
    durability->value = durability->max;
  }
}

}  // namespace vstr
