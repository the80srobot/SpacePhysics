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

EventBuffer *CreateEventBuffer() { return new EventBuffer(); }

Event *EventBufferGetEvents(EventBuffer *event_buffer, int *count) {
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
                            MotionSystem::Integrator integrator) {
  return new Pipeline(*collision_matrix, integrator);
}

Timeline *CreateTimeline(Frame *frame, int first_frame_no,
                         LayerMatrix *collision_matrix, float frame_time,
                         int key_frame_period,
                         MotionSystem::Integrator integrator) {
  return new Timeline(*frame, first_frame_no, *collision_matrix, frame_time,
                      key_frame_period, integrator);
}

int TimelineSimulate(Timeline *timeline, float time_budget) {
  auto now = std::chrono::steady_clock::now();
  auto start = now;
  timeline->Simulate();
  now = std::chrono::steady_clock::now();
  auto cost = 1.5 * (now - start);
  auto deadline = start + std::chrono::duration<float>(time_budget);
  int frames = 1;
  while (now + cost < deadline) {
    timeline->Simulate();
    now = std::chrono::steady_clock::now();
    ++frames;
  }
  return frames;
}

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

void DestroyTimeline(Timeline *timeline) { delete timeline; }
}
}  // namespace vstr