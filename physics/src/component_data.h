#ifndef VSTR_COMPONENT_DATA
#define VSTR_COMPONENT_DATA

#include <iostream>
#include <vector>

#include "aabb.h"
#include "vector3.h"

namespace vstr {

struct Collider {
  uint32_t layer;
  float radius;
};

struct Position {
  Vector3 value;
};

struct Motion {
  Vector3 velocity;
  Vector3 next_position;

  inline static Motion FromPositionAndVelocity(Vector3 position,
                                               Vector3 velocity) {
    return Motion{velocity, position + velocity};
  }
};

struct Glue {
  int32_t parent_id;
};

struct Destroyed {
  bool value;
};

struct CollisionEvent {
  int32_t first_body_id;
  int32_t second_body_id;
  float first_frame_offset_seconds;
};

inline bool operator==(const CollisionEvent &a, const CollisionEvent &b) {
  return a.first_body_id == b.first_body_id &&
         a.second_body_id == b.second_body_id &&
         a.first_frame_offset_seconds == b.first_frame_offset_seconds;
}

std::ostream &operator<<(std::ostream &os, const CollisionEvent &event);

struct Frame {
  std::vector<Position> positions;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Glue> glue;
  std::vector<Destroyed> destroyed;
};

// struct BodyEvent {
//   enum Type { kAcceleration, kGlue, kDestroyed, kCollision };

//   int32_t body_id;
//   Type type;

//   union {
//     AccelerationEvent acceleration;
//     GlueEvent glue;
//     DestroyedEvent destroyed;
//     CollisionEvent collision;
//   };
// };

// bool operator==(const BodyEvent &a, const BodyEvent &b);
// bool operator>=(const BodyEvent &a, const BodyEvent &b);
// bool operator>(const BodyEvent &a, const BodyEvent &b);
// bool operator<(const BodyEvent &a, const BodyEvent &b);
// bool operator<=(const BodyEvent &a, const BodyEvent &b);

}  // namespace vstr

#endif