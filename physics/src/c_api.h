// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_C_API
#define VSTR_C_API

#include "geometry/layer_matrix.h"
#include "geometry/vector3.h"
#include "pipeline.h"
#include "systems/component_data.h"

#if defined(__APPLE__) || defined(__linux__) || defined(ANDROID)
#define EXPORT __attribute__((visibility("default")))
#else
#define EXPORT __declspec(dllexport)
#endif

namespace vstr {

extern "C" {

EXPORT Frame *CreateFrame();
EXPORT int FrameCoreCount(Frame *frame);
EXPORT void FrameResize(Frame *frame, int count);
EXPORT Position *FrameGetPositions(Frame *frame, int *count);
EXPORT Position *FrameSetPosition(Frame *frame, int idx, Position *position);
EXPORT void DestroyFrame(Frame *frame);

using EventBuffer = std::vector<Event>;

EXPORT EventBuffer *CreateEventBuffer();
EXPORT Event *EventBufferGetEvents(EventBuffer *event_buffer, int *count);
EXPORT void DestroyEventBuffer(EventBuffer *event_buffer);

EXPORT LayerMatrix *CreateLayerMatrix();
EXPORT void LayerMatrixSet(LayerMatrix *layer_matrix, uint32_t x, uint32_t y);
EXPORT void DestroyLayerMatrix(LayerMatrix *layer_matrix);

EXPORT Pipeline *CreateFrameSolver(LayerMatrix *collision_matrix,
                                   MotionSystem::Integrator integrator);
EXPORT void FrameSolverStep(Pipeline *frame_solver, float frame_time,
                            int frame_no, Frame *frame, Event *input,
                            size_t input_sz, EventBuffer *event_buffer);
EXPORT void DestroyFrameSolver(Pipeline *frame_solver);
}
}  // namespace vstr

#endif