#include "c_api.h"

#include <absl/types/span.h>

#include <chrono>

namespace vstr {
extern "C" {

Frame *CreateFrame() { return new Frame(); }

int FrameSize(Frame *frame) { return frame->positions.size(); }

void FrameResize(Frame *frame, int count) {
  frame->positions.resize(count);
  frame->mass.resize(count);
  frame->motion.resize(count);
  frame->colliders.resize(count);
  frame->glue.resize(count);
  frame->flags.resize(count);
}

Position *FrameGetMutablePositions(Frame *frame, int *count) {
  *count = frame->positions.size();
  return frame->positions.data();
}

Mass *FrameGetMutableMass(Frame *frame, int *count) {
  *count = frame->mass.size();
  return frame->mass.data();
}

Motion *FrameGetMutableMotion(Frame *frame, int *count) {
  *count = frame->motion.size();
  return frame->motion.data();
}

Collider *FrameGetMutableColliders(Frame *frame, int *count) {
  *count = frame->colliders.size();
  return frame->colliders.data();
}

Glue *FrameGetMutableGlue(Frame *frame, int *count) {
  *count = frame->glue.size();
  return frame->glue.data();
}

Flags *FrameGetMutableFlags(Frame *frame, int *count) {
  *count = frame->flags.size();
  return frame->flags.data();
}

void FrameResizeOrbits(Frame *frame, int count) { frame->orbits.resize(count); }

Orbit *FrameGetMutableOrbits(Frame *frame, int *count) {
  *count = frame->orbits.size();
  return frame->orbits.data();
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

RuleSet *CreateRuleSet() { return new RuleSet(); }

void RuleSetAdd(RuleSet *rule_set, uint32_t target_layer, uint32_t other_layer,
                Action action) {
  rule_set->Add(RuleSet::LayerPair(target_layer, other_layer), action);
}

void DestroyRuleSet(RuleSet *rule_set) { delete (rule_set); }

Timeline *CreateTimeline(Frame *frame, int first_frame_no,
                         LayerMatrix *collision_matrix, RuleSet *rule_set,
                         float frame_time, int key_frame_period,
                         IntegrationMethod integrator) {
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

int TimelineSimulate(Timeline *timeline, float time_budget, int limit) {
  const int max_frames = limit - timeline->head();
  if (max_frames <= 0) return 0;
  auto now = std::chrono::steady_clock::now();
  auto start = now;
  timeline->Simulate();
  now = std::chrono::steady_clock::now();
  auto cost = 1.5 * (now - start);
  auto deadline = start + std::chrono::duration<float>(time_budget);
  int frames = 1;
  while (now + cost < deadline && frames < max_frames) {
    timeline->Simulate();
    now = std::chrono::steady_clock::now();
    ++frames;
  }
  return frames - 1;
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