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
#include "systems/component_data.h"
#include "timeline.h"

#if defined(__APPLE__) || defined(__linux__) || defined(ANDROID)
#define EXPORT __attribute__((visibility("default")))
#else
#define EXPORT __declspec(dllexport)
#endif

namespace vstr {

extern "C" {

// FRAME API //

EXPORT Frame *CreateFrame();

// Work on core components:

EXPORT int FrameSize(Frame *frame);
EXPORT void FrameResize(Frame *frame, int count);
EXPORT Position *FrameGetMutablePositions(Frame *frame, int *count);
EXPORT Mass *FrameGetMutableMass(Frame *frame, int *count);
EXPORT Motion *FrameGetMutableMotion(Frame *frame, int *count);
EXPORT Collider *FrameGetMutableColliders(Frame *frame, int *count);
EXPORT Glue *FrameGetMutableGlue(Frame *frame, int *count);
EXPORT Flags *FrameGetMutableFlags(Frame *frame, int *count);

// Optional components
EXPORT void FrameResizeOrbits(Frame *frame, int count);
EXPORT Orbit *FrameGetMutableOrbits(Frame *frame, int *count);

EXPORT void DestroyFrame(Frame *frame);

// ORBIT API //

EXPORT Vector3 KeplerEllipticalPosition(Orbit::Kepler *kepler);

// EVENT API //

using EventBuffer = std::vector<Event>;

EXPORT EventBuffer *CreateEventBuffer();
EXPORT Event *EventBufferGetEvents(EventBuffer *event_buffer, int *count);
EXPORT void DestroyEventBuffer(EventBuffer *event_buffer);

// LAYER MATRIX API //

EXPORT LayerMatrix *CreateLayerMatrix();
EXPORT void LayerMatrixSet(LayerMatrix *layer_matrix, uint32_t x, uint32_t y);
EXPORT void DestroyLayerMatrix(LayerMatrix *layer_matrix);

// TIMELINE API //

EXPORT Timeline *CreateTimeline(Frame *frame, int first_frame_no,
                                LayerMatrix *collision_matrix, float frame_time,
                                int key_frame_period,
                                IntegrationMethod integrator);
EXPORT void TimelineInputEvent(Timeline *timeline, int frame_no, Event *event);
EXPORT void TimelineInputEventRange(Timeline *timeline, int first_frame_no,
                                    int last_frame_no, Event *event);
EXPORT int TimelineSimulate(Timeline *timeline, float time_budget, int limit);
EXPORT const Frame *TimelineGetFrame(Timeline *timeline, int frame_no);
EXPORT int TimelineGetHead(Timeline *timeline);
EXPORT void TimelineGetEvents(Timeline *timeline, int frame_no,
                              EventBuffer *buffer);
EXPORT void TimelineGetEventRange(Timeline *timeline, int first_frame_no,
                                  int last_frame_no, EventBuffer *buffer);
EXPORT void DestroyTimeline(Timeline *timeline);

// Timeline query API //

struct TimelineQuery {
  int resolution;
  size_t trajectory_buffer_sz;
  Timeline::Trajectory *trajectory_buffer;
};

EXPORT bool TimelineRunQuery(Timeline *timeline, TimelineQuery *query);
}
}  // namespace vstr

#endif