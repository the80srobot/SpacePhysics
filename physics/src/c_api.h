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
#include "timeline.h"
#include "types/required_components.h"

#if defined(__APPLE__) || defined(__linux__) || defined(ANDROID)
#define EXPORT __attribute__((visibility("default")))
#else
#define EXPORT __declspec(dllexport)
#endif

namespace vstr {

extern "C" {

// FRAME API //

EXPORT Frame *CreateFrame();

// CORE COMPONENT DATA

// Every object has all six core components. Consequently, the six arrays that
// hold the component data are all the same size, and each object's ID acts as
// offset into the core component arrays. (It follows that the arrays must not
// be reordered.)

EXPORT int FrameSize(Frame *frame);
EXPORT void FrameResize(Frame *frame, int count);

EXPORT Transform *FrameGetMutablePositions(Frame *frame, int *count);
EXPORT Mass *FrameGetMutableMass(Frame *frame, int *count);
EXPORT Motion *FrameGetMutableMotion(Frame *frame, int *count);
EXPORT Collider *FrameGetMutableColliders(Frame *frame, int *count);
EXPORT Glue *FrameGetMutableGlue(Frame *frame, int *count);
EXPORT Flags *FrameGetMutableFlags(Frame *frame, int *count);

// OPTIONAL COMPONENT DATA

// Optional components are handled differently from core components. Optional
// components include as their first field the ID of the object they belong to.
// (With core components that's not needed, because the array offset is the ID.)
// They must still be kept sorted by object ID, to enable binary search.

EXPORT void FrameResizeOrbits(Frame *frame, int count);
EXPORT Orbit *FrameGetMutableOrbits(Frame *frame, int *count);

EXPORT void FrameResizeDurability(Frame *frame, int count);
EXPORT Durability *FrameGetMutableDurability(Frame *frame, int *count);

EXPORT void FrameResizeRockets(Frame *frame, int count);
EXPORT Rocket *FrameGetMutableRockets(Frame *frame, int *count);

EXPORT void FrameResizeTriggers(Frame *frame, int count);
EXPORT Trigger *FrameGetMutableTriggers(Frame *frame, int *count);

EXPORT void DestroyFrame(Frame *frame);

// ORBIT API //

EXPORT Vector3 KeplerEllipticalPosition(Orbit::Kepler kepler);

// EVENT API //

using EventBuffer = std::vector<IntervalTree<Event>::KV>;

EXPORT EventBuffer *CreateEventBuffer();
EXPORT void EventBufferClear(EventBuffer *event_buffer);
EXPORT IntervalTree<Event>::KV *EventBufferGetEvents(EventBuffer *event_buffer,
                                                     int *count);
EXPORT void DestroyEventBuffer(EventBuffer *event_buffer);

// LAYER MATRIX API //

EXPORT LayerMatrix *CreateLayerMatrix();
EXPORT void LayerMatrixSet(LayerMatrix *layer_matrix, uint32_t x, uint32_t y);
EXPORT void DestroyLayerMatrix(LayerMatrix *layer_matrix);

// RULE SET API //

EXPORT RuleSet *CreateRuleSet();
EXPORT void RuleSetAdd(RuleSet *rule_set, uint32_t target_layer,
                       uint32_t other_layer, Action action);
EXPORT void DestroyRuleSet(RuleSet *rule_set);

// TIMELINE API //

EXPORT Timeline *CreateTimeline(Frame *frame, int first_frame_no,
                                LayerMatrix *collision_matrix,
                                RuleSet *rule_set, float frame_time,
                                int key_frame_period,
                                IntegrationMethod integrator);
EXPORT void TimelineInputEvent(Timeline *timeline, int frame_no, Event *event);
EXPORT void TimelineInputEventRange(Timeline *timeline, int first_frame_no,
                                    int last_frame_no, Event *event);
EXPORT int TimelineSimulate(Timeline *timeline, float time_budget, int limit,
                            uint64_t *time_spent_nanos);
EXPORT const Frame *TimelineGetFrame(Timeline *timeline, int frame_no);
EXPORT int TimelineGetHead(Timeline *timeline);
EXPORT int TimelineGetTail(Timeline *timeline);
EXPORT void TimelineGetEvents(Timeline *timeline, int frame_no,
                              EventBuffer *buffer);
EXPORT void TimelineGetEventRange(Timeline *timeline, int first_frame_no,
                                  int last_frame_no, EventBuffer *buffer);
EXPORT void TimelineSetLabel(Timeline *timeline, int id, Timeline::Label label);
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