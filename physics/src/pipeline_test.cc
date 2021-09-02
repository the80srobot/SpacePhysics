// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "pipeline.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace vstr {
namespace {

TEST(PipelineTest, FallingSphere) {
  Pipeline pipeline(LayerMatrix({{1, 1}}));
  const float dt = 0.001;
  // The spheres should take about 111 seconds to come into contact.
  const float duration = 111;

  std::vector<Transform> positions{
      Transform{Vector3{0, 100, 0}},
      Transform{Vector3{0, 0, 0}},
  };
  std::vector<Mass> mass{
      Mass{},
      Mass{100, 100},
  };
  std::vector<Motion> motion{
      Motion{},
      Motion{},
  };
  std::vector<Collider> colliders{
      Collider{1, 1},
      Collider{1, 1},
  };
  std::vector<Glue> glue{
      Glue{},
      Glue{},
  };
  std::vector<Flags> flags{
      Flags{},
      Flags{},
  };

  Frame frame{positions, mass, motion, colliders, glue, flags};

  std::vector<Event> buffer;
  int frame_no = 0;
  for (float t = 0; t < duration; t += dt) {
    pipeline.Step(dt, frame_no, frame, {}, buffer);
    ++frame_no;
  }

  EXPECT_LT(frame.transforms[0].position.y, 1);
  EXPECT_GT(frame.transforms[0].position.y, 0);

  ASSERT_GE(buffer.size(), 1);
  EXPECT_EQ(buffer[0].type, Event::kCollision);
  EXPECT_EQ(buffer[0].id, 0);
  EXPECT_EQ(buffer[0].collision.second_id, 1);
  EXPECT_NE(buffer[0].collision.first_frame_offset_seconds, 0);
}

}  // namespace
}  // namespace vstr