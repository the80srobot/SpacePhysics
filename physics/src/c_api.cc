// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "c_api.h"

#include <absl/types/span.h>

#include <chrono>

#include "debug.h"
#include "systems/object_pool.h"

namespace vstr {
extern "C" {

Frame *CreateFrame() { return new Frame(); }

void FrameSyncView(Frame *frame, FrameView *out_view) {
  out_view->transform_data = frame->transforms.data();
  out_view->mass_data = frame->mass.data();
  out_view->motion_data = frame->motion.data();
  out_view->collider_data = frame->colliders.data();
  out_view->glue_data = frame->glue.data();
  out_view->flags_data = frame->flags.data();
  out_view->orbit_data = frame->orbits.data();
  out_view->durability_data = frame->durability.data();
  out_view->rocket_data = frame->rockets.data();
  out_view->trigger_data = frame->triggers.data();
  out_view->reuse_pool_data = frame->reuse_pools.data();
  out_view->reuse_tag_data = frame->reuse_tags.data();

  out_view->object_count = frame->transforms.size();
  out_view->orbit_count = frame->orbits.size();
  out_view->durability_count = frame->durability.size();
  out_view->rocket_count = frame->rockets.size();
  out_view->trigger_count = frame->triggers.size();
  out_view->reuse_pool_count = frame->reuse_pools.size();
  out_view->reuse_tag_count = frame->reuse_tags.size();
}

int32_t FramePush(Frame *frame, Transform transform, Mass mass, Motion motion,
                  Collider collider, Glue glue, Flags flags) {
  return frame->Push(std::move(transform), std::move(mass), std::move(motion),
                     std::move(collider), std::move(glue), std::move(flags));
}

int32_t FramePushObjectPool(Frame *frame, int32_t pool_id, int32_t prototype_id,
                            int32_t capacity, int32_t *obj_ids) {
  if (pool_id == prototype_id) {
    return -1;
  }

  int32_t pool_idx = InitializePool(pool_id, prototype_id, capacity, *frame);

  for (int id = frame->reuse_pools[pool_idx].first_id; id >= 0;
       id = frame->reuse_tags[FindOptionalComponent(frame->reuse_tags, id)]
                .next_id) {
    *obj_ids = id;
    ++obj_ids;
    assert(--capacity >= 0);
  }

  return pool_idx;
}

int32_t FrameSetOrbit(Frame *frame, Orbit orbit) {
  return SetOptionalComponent(orbit.id, orbit, frame->orbits);
}

int32_t FrameSetDurability(Frame *frame, Durability durability) {
  return SetOptionalComponent(durability.id, durability, frame->durability);
}

int32_t FrameSetRocket(Frame *frame, Rocket rocket) {
  return SetOptionalComponent(rocket.id, rocket, frame->rockets);
}

int32_t FrameSetTrigger(Frame *frame, Trigger trigger) {
  return SetOptionalComponent(trigger.id, trigger, frame->triggers);
}

void DestroyFrame(Frame *frame) { delete frame; }

Vector3 KeplerEllipticalPosition(Orbit::Kepler kepler) {
  return EllipticalPosition(kepler);
}

EventBuffer *CreateEventBuffer() { return new EventBuffer(); }

void EventBufferClear(EventBuffer *event_buffer) { event_buffer->clear(); }

IntervalTree<Event>::KV *EventBufferGetEvents(EventBuffer *event_buffer,
                                              int *count) {
  *count = event_buffer->size();
  return event_buffer->data();
}

void DestroyEventBuffer(EventBuffer *event_buffer) { delete event_buffer; }

LayerMatrix *CreateLayerMatrix() { return new LayerMatrix({}); }

void LayerMatrixSet(LayerMatrix *layer_matrix, uint32_t x, uint32_t y) {
  layer_matrix->Set(x, y, true);
}

void DestroyLayerMatrix(LayerMatrix *layer_matrix) { delete layer_matrix; }

Pipeline *CreateFrameSolver(LayerMatrix *collision_matrix,
                            IntegrationMethod integrator) {
  return new Pipeline(*collision_matrix, integrator);
}

CollisionRuleSet *CreateRuleSet() { return new CollisionRuleSet(); }

void RuleSetAdd(CollisionRuleSet *rule_set, uint32_t target_layer,
                uint32_t other_layer, CollisionEffect action) {
  rule_set->Add(CollisionRuleSet::LayerPair(target_layer, other_layer), action);
}

void DestroyRuleSet(CollisionRuleSet *rule_set) { delete (rule_set); }

Timeline *CreateTimeline(Frame *frame, int first_frame_no,
                         LayerMatrix *collision_matrix,
                         CollisionRuleSet *rule_set, float frame_time,
                         int key_frame_period, IntegrationMethod integrator) {
#if !defined(NDEBUG) && defined(VSTR_BREAK_ON_FLOAT_EXC)
  DebugHelper::Singleton()->EnableFloatExceptions();
#endif

  return new Timeline(*frame, first_frame_no, *collision_matrix, *rule_set,
                      frame_time, key_frame_period, integrator);
}

void TimelineInputEvent(Timeline *timeline, int frame_no, Event *event) {
  timeline->InputEvent(frame_no, *event);
}

void TimelineInputEventRange(Timeline *timeline, int first_frame_no,
                             int last_frame_no, Event *event) {
  timeline->InputEvent(first_frame_no, last_frame_no, *event);
}

int TimelineSimulate(Timeline *timeline, float time_budget, int limit,
                     uint64_t *time_spent_nanos) {
  const int max_frames = limit - timeline->head();
  if (max_frames <= 0) return 0;

  // Simulate one frame and measure how long that took us.
  auto now = std::chrono::steady_clock::now();
  auto start = now;
  timeline->Simulate();
  int frames = 1;

  // We really don't want to exceed the time budget, so we assume subsequent
  // frames might take 1.2x as long as the first frame did.
  now = std::chrono::steady_clock::now();
  auto cost = 1.2 * (now - start);
  auto deadline = start + std::chrono::duration<double>(time_budget);

  // Keep going as long as we think simulating the next frame won't exceed the
  // deadline.
  while (now + cost < deadline && frames < max_frames) {
    timeline->Simulate();
    now = std::chrono::steady_clock::now();
    ++frames;
  }

  *time_spent_nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count();
  return frames;
}

int TimelineGetHead(Timeline *timeline) { return timeline->head(); }

int TimelineGetTail(Timeline *timeline) { return timeline->tail(); }

const Frame *TimelineGetFrame(Timeline *timeline, int frame_no) {
  return timeline->GetFrame(frame_no);
}

void TimelineGetEvents(Timeline *timeline, int frame_no, EventBuffer *buffer) {
  timeline->GetEvents(frame_no, *buffer);
}

void TimelineGetEventRange(Timeline *timeline, int first_frame_no,
                           int last_frame_no, EventBuffer *buffer) {
  timeline->GetEvents(first_frame_no, last_frame_no, *buffer);
}

void TimelineSetLabel(Timeline *timeline, const int id,
                      const Timeline::Label label) {
  timeline->SetLabel(id, label);
}

void DestroyTimeline(Timeline *timeline) { delete timeline; }

bool TimelineRunQuery(Timeline *timeline, TimelineQuery *query) {
  auto trajectories =
      absl::MakeSpan(query->trajectory_buffer, query->trajectory_buffer_sz);
  auto status = timeline->Query(query->resolution, trajectories);
  return status.ok();
}
}
}  // namespace vstr