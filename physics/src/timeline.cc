// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "timeline.h"

namespace vstr {
namespace {}

const Frame *Timeline::GetFrame(const int frame_no) {
  if (frame_no == frame_no_) return &frame_;
  if (frame_no == head_) return &head_frame_;
  if (frame_no < tail_ || frame_no > head_) return nullptr;

  auto d = std::div(frame_no - tail_, key_frame_period_);
  if (d.rem == 0) return &key_frames_[d.quot];

  Replay(frame_no);
  return &frame_;
}

bool Timeline::GetEvents(const int frame_no, std::vector<Event> &buffer) const {
  if (frame_no == head_) {
    buffer.insert(buffer.end(), simulate_buffer_.begin(),
                  simulate_buffer_.end());
    return true;
  }
  if (frame_no < tail_ || frame_no > head_) return false;
  events_.Overlap(frame_no, buffer);
  return true;
}

bool Timeline::GetEvents(const int first_frame_no, const int last_frame_no,
                         std::vector<Event> &buffer) const {
  assert(last_frame_no > first_frame_no);
  if (first_frame_no < tail_ || last_frame_no > head_) return false;
  events_.Overlap(Interval{first_frame_no, last_frame_no}, buffer);
  return true;
}

void Timeline::Truncate(int new_head) {
  if (new_head == head_) return;
  // TODO(adam): this could be about 5-10 times faster and require no allocation
  // if the tree was right-aligned, instead of left-aligned.
  std::vector<IntervalTree<Event>::KV> to_delete;
  events_.Overlap(Interval(new_head, events_.MaxPoint()));
  for (auto &kv : to_delete) {
    events_.Delete(kv);
    if (kv.first.low < new_head) {
      kv.first.high = new_head;
      events_.Insert(kv.first, kv.second);
    }
  }

  auto d = std::div(new_head - tail_, key_frame_period_);
  head_frame_ = key_frames_[d.quot];
}

void Timeline::InputEvent(const int frame_no, const Event &event) {
  Truncate(frame_no);
  events_.MergeInsert(Interval(frame_no, frame_no + 1), event);
}

void Timeline::Simulate() {
  ++head_;
  replay_buffer_.clear();
  simulate_buffer_.clear();

  auto end = events_.End();
  for (auto it = events_.Overlap(head_); it != end; ++it) {
    if (it->second.type == Event::kInput) {
      replay_buffer_.push_back(it->second);
    }
  }
  pipeline_->Step(frame_time_, head_, head_frame_,
                  absl::MakeSpan(replay_buffer_), simulate_buffer_);
  for (const auto &event : simulate_buffer_) {
    events_.MergeInsert(Interval{head_, head_ + 1}, event);
  }
}

bool Timeline::Replay(int frame_no) {
  if (frame_no_ > head_) return false;

  const auto d = std::div(frame_no - tail_, key_frame_period_);
  frame_ = key_frames_[d.quot];

  for (int f = tail_ + d.quot * key_frame_period_; f < frame_no; ++f) {
    replay_buffer_.clear();
    events_.Overlap(f, replay_buffer_);
    pipeline_->Replay(frame_time_, f, frame_, absl::MakeSpan(replay_buffer_));
  }
}

}  // namespace vstr