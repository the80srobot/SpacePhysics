// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_TIMELINE
#define VSTR_TIMELINE

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "dsa/interval_tree.h"
#include "pipeline.h"
#include "types/frame.h"
#include "types/required_components.h"

namespace vstr {

class Timeline {
 public:
  Timeline(const Frame &scene, int first_frame_no, LayerMatrix collision_matrix,
           const CollisionRuleSet &rule_set, float frame_time = 1.0f / 60,
           int key_frame_period = 30,
           IntegrationMethod integrator = kVelocityVerlet)
      : head_(first_frame_no),
        head_frame_(scene),
        tail_(first_frame_no),
        frame_time_(frame_time),
        key_frame_period_(key_frame_period),
        frame_no_{first_frame_no},
        frame_{scene},
        key_frames_{scene},
        pipeline_(std::make_shared<Pipeline>(collision_matrix, rule_set,
                                             integrator)) {}
  Timeline() = delete;

  const Frame *GetFrame(int frame_no);
  bool GetEvents(int frame_no, std::vector<Event> &buffer) const;
  bool GetEvents(int first_frame_no, int last_frame_no,
                 std::vector<Event> &buffer) const;
  bool GetEvents(int frame_no,
                 std::vector<IntervalTree<Event>::KV> &buffer) const;
  bool GetEvents(int first_frame_no, int last_frame_no,
                 std::vector<IntervalTree<Event>::KV> &buffer) const;

  // Deletes events occurring after new_head. DOES NOT delete events with the
  // Event::kUserInput flag set. The optional second argument overrides the user
  // input flag for the specified entity.
  //
  // This might look like a fiddly interface, but it naturally allows changing
  // user input for the object under control, while keeping the user input
  // history for all other objects.
  void Truncate(int new_head, Entity user_input_target = Entity::Nil());
  void InputEvent(int frame_no, const Event &event);
  void InputEvent(int first_frame_no, int last_frame_no, const Event &event);
  void Simulate();

  struct Trajectory {
    enum Attribute { kPosition = 1 << 0, kVelocity = 1 << 1 };
    int id;
    int first_frame_no;
    Attribute attribute;
    // This is a size and a pointer for compatibility with the C foreign
    // function API.
    size_t buffer_sz;
    Vector3 *buffer;
  };

  absl::Status Query(int resolution, absl::Span<Trajectory> trajectories);

  inline int head() const { return head_; }
  inline int tail() const { return tail_; }

  struct Label {
    char label[32];
  };

  void SetLabel(int id, Label label);

 private:
  // Labels do nothing - they can be optionally set and then read back out.
  std::vector<Label> labels_;

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
