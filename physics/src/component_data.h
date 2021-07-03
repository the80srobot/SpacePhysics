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

inline bool operator==(const Collider &a, const Collider &b) {
  return a.layer == b.layer && a.radius == b.radius;
}

std::ostream &operator<<(std::ostream &os, const Collider &collider);

struct Position {
  Vector3 value;
};

inline bool operator==(const Position &a, const Position &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Position &position);

struct Acceleration {
  Vector3 value;
};

inline bool operator==(const Acceleration &a, const Acceleration &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Acceleration &acceleration);

struct Mass {
  float rest;
  float effective;
};

inline bool operator==(const Mass &a, const Mass &b) {
  return a.rest == b.rest && a.effective == b.effective;
}

std::ostream &operator<<(std::ostream &os, const Mass &mass);

struct Motion {
  Vector3 velocity;
  Vector3 new_position;

  inline static Motion FromPositionAndVelocity(Vector3 position,
                                               Vector3 velocity) {
    return Motion{velocity, position + velocity};
  }
};

inline bool operator==(const Motion &a, const Motion &b) {
  return a.velocity == b.velocity && a.new_position == b.new_position;
}

std::ostream &operator<<(std::ostream &os, const Motion &motion);

struct Glue {
  int32_t parent_id;
};

inline bool operator==(const Glue &a, const Glue &b) {
  return a.parent_id == b.parent_id;
}

std::ostream &operator<<(std::ostream &os, const Glue &glue);

struct Destroyed {
  bool value;
};

inline bool operator==(const Destroyed &a, const Destroyed &b) {
  return a.value == b.value;
}

std::ostream &operator<<(std::ostream &os, const Destroyed &destroyed);

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
  std::vector<Mass> mass;
  std::vector<Acceleration> acceleration;
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