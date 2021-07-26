// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TIMELINE
#define VSTR_TIMELINE

#include <vector>

#include "dsa/interval_tree.h"
#include "frame.h"
#include "pipeline.h"
#include "systems/component_data.h"

namespace vstr {

class Timeline {
 public:
  Timeline(const Frame &scene, int first_frame_no, LayerMatrix collision_matrix,
           float frame_time = 1.0f / 60, int key_frame_period = 30,
           IntegrationMethod integrator = kVelocityVerlet)
      : head_(first_frame_no),
        head_frame_(scene),
        tail_(first_frame_no),
        frame_time_(frame_time),
        key_frame_period_(key_frame_period),
        frame_no_{first_frame_no},
        frame_{scene},
        key_frames_{scene},
        pipeline_(std::make_shared<Pipeline>(collision_matrix)) {}
  Timeline() = delete;

  const Frame *GetFrame(int frame_no);
  bool GetEvents(int frame_no, std::vector<Event> &buffer) const;
  bool GetEvents(int first_frame_no, int last_frame_no,
                 std::vector<Event> &buffer) const;
  void Truncate(int new_head);
  void InputEvent(int frame_no, const Event &event);
  void InputEvent(int first_frame_no, int last_frame_no, const Event &event);
  void Simulate();

  inline int head() const { return head_; }
  inline int tail() const { return tail_; }

 private:
  // Timeline(const Frame &scene, int first_frame_no, float frame_time,
  //          int key_frame_period, std::shared_ptr<Pipeline> solver);

  bool Replay(int frame_no);

  int head_;
  Frame head_frame_;

  int tail_;

  float frame_time_;
  int key_frame_period_;

  int frame_no_;
  Frame frame_;

  std::vector<Frame> key_frames_;
  IntervalTree<Event> events_;
  std::shared_ptr<Pipeline> pipeline_;

  std::vector<Event> simulate_buffer_;
  std::vector<Event> replay_buffer_;
  std::vector<Event> input_buffer_;
};

}  // namespace vstr

#endif
