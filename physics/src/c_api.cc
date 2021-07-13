#include "c_api.h"

#include <absl/types/span.h>

namespace vstr {
extern "C" {

Frame *CreateFrame() { return new Frame(); }

int FrameCoreCount(Frame *frame) { return frame->positions.size(); }

void FrameResize(Frame *frame, int count) {
  frame->positions.resize(count);
  frame->mass.resize(count);
  frame->motion.resize(count);
  frame->colliders.resize(count);
  frame->glue.resize(count);
  frame->flags.resize(count);
}

Position *FrameGetPositions(Frame *frame, int *count) {
  *count = frame->positions.size();
  return frame->positions.data();
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

FrameSolver *CreateFrameSolver(LayerMatrix *collision_matrix,
                               MotionSystem::Integrator integrator) {
  return new FrameSolver(*collision_matrix, integrator);
}

void FrameSolverStep(FrameSolver *frame_solver, const float frame_time,
                     const int frame_no, Frame *frame, const Input *input,
                     const size_t input_sz, EventBuffer *event_buffer) {
  frame_solver->Step(frame_time, frame_no, *frame,
                     absl::Span<const Input>(input, input_sz), *event_buffer);
}

void DestroyFrameSolver(FrameSolver *frame_solver) { delete frame_solver; }
}
}  // namespace vstr