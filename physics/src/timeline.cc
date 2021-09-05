// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "timeline.h"

#include "systems/object_pool.h"

namespace vstr {
namespace {

bool EventPartialEq(const Event &a, const Event &b) {
  if (a.type != Event::kSpawnAttempt) return a == b;
  return a == b && a.position == b.position;
}

}  // namespace

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
  // TODO(adam): This can be optimized if frame_no == head_, which is a hot
  // path. The simulate_buffer_ might have subtly different contents from what
  // Overlap returns, though.
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

bool Timeline::GetEvents(const int frame_no,
                         std::vector<IntervalTree<Event>::KV> &buffer) const {
  if (frame_no < tail_ || frame_no > head_) return false;
  events_.Overlap(frame_no, buffer);
  return true;
}

bool Timeline::GetEvents(const int first_frame_no, const int last_frame_no,
                         std::vector<IntervalTree<Event>::KV> &buffer) const {
  assert(last_frame_no > first_frame_no);
  if (first_frame_no < tail_ || last_frame_no > head_) return false;
  events_.Overlap(Interval{first_frame_no, last_frame_no}, buffer);
  return true;
}

void Timeline::Truncate(int new_head) {
  if (new_head >= head_) return;

  // TODO(adam): this could be about 5-10 times faster and require no allocation
  // if the tree was right-aligned, instead of left-aligned.
  if (events_.Count() > 0) {
    std::vector<IntervalTree<Event>::KV> to_delete;
    events_.Overlap(Interval(new_head, events_.MaxPoint()), to_delete);
    for (auto &kv : to_delete) {
      events_.Delete(kv);
      if (kv.first.low < new_head) {
        kv.first.high = new_head;
        events_.Insert(kv.first, kv.second);
      }
    }
  }

  auto d = std::div(new_head - tail_, key_frame_period_);
  head_frame_ = key_frames_[d.quot];
  key_frames_.erase(key_frames_.begin() + d.quot + 1, key_frames_.end());

  for (head_ = d.quot * key_frame_period_; head_ < new_head; ++head_) {
    replay_buffer_.clear();
    events_.Overlap(head_, replay_buffer_);
    pipeline_->Replay(frame_time_, head_, head_frame_,
                      absl::MakeSpan(replay_buffer_));
  }
}

void Timeline::InputEvent(const int frame_no, const Event &event) {
  assert(frame_no > tail_);
  Truncate(frame_no - 1);
  events_.MergeInsert(Interval(frame_no, frame_no + 1), event, EventPartialEq);
}

void Timeline::InputEvent(int first_frame_no, int last_frame_no,
                          const Event &event) {
  assert(first_frame_no > tail_);
  Truncate(first_frame_no - 1);
  events_.MergeInsert(Interval(first_frame_no, last_frame_no + 1), event,
                      EventPartialEq);
}

void Timeline::Simulate() {
  ++head_;
  input_buffer_.clear();
  simulate_buffer_.clear();

  events_.Overlap(head_, input_buffer_);
  pipeline_->Step(frame_time_, head_, head_frame_,
                  absl::MakeSpan(input_buffer_), simulate_buffer_);
  for (const auto &event : simulate_buffer_) {
    events_.MergeInsert(Interval{head_, head_ + 1}, event);
  }

  if ((head_ % key_frame_period_) == 0) {
    key_frames_.push_back(head_frame_);
  }
}

bool Timeline::Replay(int frame_no) {
  if (frame_no > head_) return false;

  const auto d = std::div(frame_no - tail_, key_frame_period_);
  assert(key_frames_.size() > d.quot);
  if (d.quot != (frame_no_ - tail_) / key_frame_period_ ||
      frame_no_ > frame_no) {
    frame_ = key_frames_[d.quot];
    frame_no_ = tail_ + d.quot * key_frame_period_;
  }

  for (; frame_no_ < frame_no; ++frame_no_) {
    replay_buffer_.clear();
    events_.Overlap(frame_no_, replay_buffer_);
    pipeline_->Replay(frame_time_, frame_no_, frame_,
                      absl::MakeSpan(replay_buffer_));
  }

  assert(frame_no == frame_no);

  return true;
}

absl::Status Timeline::Query(int resolution,
                             absl::Span<Trajectory> trajectories) {
  if (trajectories.empty()) return absl::OkStatus();

  // AKA the population count. Tells us how many attributes are requested. The
  // required buffer size for each trajectory is 'frame_count' *
  // 'attribute_count'
  int hamming_weights[trajectories.size()];

  // First pass: find the minimum and maximum frame requested.
  int first = head_;
  int last = tail_;
  for (int i = 0; i < trajectories.size(); ++i) {
    const auto &query = trajectories[i];
    hamming_weights[i] =
        std::bitset<sizeof(Trajectory::Attribute)>(query.attribute).count();

    if ((query.first_frame_no % resolution) != 0) {
      return absl::InvalidArgumentError("query not aligned to resolution");
    }

    if (hamming_weights[i] == 0) {
      return absl::InvalidArgumentError("no data requested in query");
    }

    first = std::min(first, query.first_frame_no);
    last = std::max(last,
                    query.first_frame_no + static_cast<int>(query.buffer_sz) *
                                               resolution / hamming_weights[i]);
  }

  if (first < tail_) {
    return absl::OutOfRangeError(
        absl::StrCat("first frame ", first, "< tail ", tail_));
  }
  if (last > head_) {
    return absl::OutOfRangeError(
        absl::StrCat("last frame ", last, " > head ", head_));
  }

  // Second pass: load the attribute data requested.
  for (int frame_no = first; frame_no <= last; frame_no += resolution) {
    Replay(frame_no);
    for (int i = 0; i < trajectories.size(); ++i) {
      auto &query = trajectories[i];
      int buffer_off =
          (frame_no - query.first_frame_no) / resolution * hamming_weights[i];

      if (buffer_off < 0 || buffer_off >= query.buffer_sz) continue;

      if (query.attribute & Trajectory::Attribute::kPosition) {
        query.buffer[buffer_off] = frame_.transforms[query.id].position;
        ++buffer_off;
      }
      if (query.attribute & Trajectory::Attribute::kVelocity) {
        query.buffer[buffer_off] = frame_.motion[query.id].velocity;
        ++buffer_off;
      }
    }
  }

  return absl::OkStatus();
}

void Timeline::SetLabel(const int id, Label label) {
  if (labels_.size() <= id) {
    labels_.reserve(id * 2);
    labels_.resize(id + 1, {0});
  }
  label.label[7] = 0;
  labels_[id] = label;
}

}  // namespace vstr