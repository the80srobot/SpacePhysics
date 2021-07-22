// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "collision_system.h"

#include <limits>

#include "geometry/aabb.h"
#include "geometry/layer_matrix.h"
#include "geometry/vector3.h"

namespace vstr {

namespace {

float DistanceToCollision(const std::vector<Position> &positions,
                          const std::vector<Collider> &colliders,
                          const std::vector<Motion> &motion, const int a,
                          const int b, const float t) {
  Vector3 a_pos = positions[a].value + motion[a].velocity * t;
  Vector3 b_pos = positions[b].value + motion[b].velocity * t;
  return Vector3::Magnitude(a_pos - b_pos) - colliders[a].radius -
         colliders[b].radius;
}

// Returns the earliest time objects a and b will collide based on their current
// velocities. If no such time can be found, returns a time greater than dt.
float CollisionTime(const std::vector<Position> &positions,
                    const std::vector<Collider> &colliders,
                    const std::vector<Motion> &motion, const int a, const int b,
                    const float dt) {
  // The distance between the two objects is a function of time:
  //
  //  d(t) = |(posA + vA * t) - (posB + vB * t)|
  //
  // Distance to collision is also a function of time (recall that the objects
  // are spheres):
  //
  //  dc(t) = d(t) - rA - rB
  //
  // Therefore, we can find the time of collision by solving the following
  // equation for t:
  //
  //  dc = |(posA + vA * t) - (posB + vB * t)| - rA - rB
  //
  // A closed form algebraic solution exists (except for parallel lines), but
  // it's unwieldy and requires multiple expensive operations. Fortunately, we
  // can show that the function can only take one of three forms, which can be
  // solved separately.
  //
  // 1) Linear: distance to collision is directly or inversely proportional to
  //    time.
  // 2) Constant: the lines are parallel
  // 3) Composed of two linear functions: the objects approach and then recede,
  //    resulting in a V-shaped distance function.
  //
  // We take three samples at 0, dt/2 and dt:
  // 1) If d(0) > d(dt/2) > d(dt) or d(0) < d(dt/2) < d(dt) then the function is
  //    linear.
  // 2) If d(0) == d(dt/2) == d(dt) then the lines are parallel.
  // 3) Otherwise the function is V-shaped.
  float d0 = DistanceToCollision(positions, colliders, motion, a, b, 0);

  // The objects are already in collision.
  if (d0 <= 0) {
    return 0;
  }

  float d1 = DistanceToCollision(positions, colliders, motion, a, b, dt / 2);
  float d2 = DistanceToCollision(positions, colliders, motion, a, b, dt);

  // TODO(adam): The floating point comparisons suffer from rounding errors.

  if (d0 == d1 && d0 == d2) {
    // The lines are parallel. The objects are either already in collision, or
    // never will be.
    if (d0 <= 0) {
      return 0;
    }

    return std::numeric_limits<float>::infinity();
  }

  if (d0 < d1 && d0 - d1 == d1 - d2) {
    // The distance function is linear and the objects are receding (d0 is the
    // smallest distance). Therefore, the objects are either already in
    // collision, or never will be.
    if (d0 <= 0) {
      return 0;
    }

    // The objects are already in collision at time 0.
    return std::numeric_limits<float>::infinity();
  }

  float slope;
  if (d0 > d1 && d0 - d1 == d1 - d2) {
    // The distance function is linear and the objects are approaching (d2 is
    // the smallest distance).
    if (d2 > 0) {
      // The objects won't collide before dt.
      return std::numeric_limits<float>::infinity();
    }

    if (d0 <= 0) {
      // The objects are already in collision.
      return 0;
    }

    slope = (d0 - d2) / dt;
    // Solve for t: d0 + slope * t = 0
    // -> t = d0/slope (we know slope != 0).
    return d0 / slope;
  } else {
    // The lines are either parallel, or the function is linear and the objects
    // are receding. Either way, d0 is the closest approach this frame and we
    // already know d0 > 0.
    // return std::numeric_limits<float>::infinity();
  }

  // The function is V-shaped: the objects approach and then recede. The
  // function is a symmetrical, piecewise linear function. This makes it easy to
  // find the slope (same on either side) and compute the intercept under the
  // theory that the left side of the function crosses the x axis. Then we only
  // need to test that at the hypothetical intercept the distance to collision
  // is indeed 0.
  if (d0 > d2) {
    slope = (d1 - d0) / (dt / 2);
  } else {
    slope = (d1 - d2) / (dt / 2);
  }

  // Because of float rounding errors the value at -d0/slope is unlikely to be
  // exactly zero. Advancing the time step slightly pushes the value into the
  // negative if collision occurs. Because we know the function is linear, if
  // the value is negative, we know it will be just on the negative side of
  // zero.
  float t = (-d0 / slope);
  if (DistanceToCollision(positions, colliders, motion, a, b,
                          t + std::numeric_limits<float>::epsilon()) < 0) {
    return t;
  }

  return std::numeric_limits<float>::infinity();
}

bool Eligible(const std::vector<Collider> &colliders,
              const std::vector<Flags> &flags, const std::vector<Glue> &glue,
              const LayerMatrix &matrix, const int a, const int b) {
  if (b <= a) {
    return false;  // Checked in the other direction or self-collision.
  }

  if ((flags[a].value & Flags::kDestroyed) ||
      (flags[b].value & Flags::kDestroyed)) {
    return false;
  }

  if (!matrix.Check(colliders[a].layer, colliders[b].layer)) {
    return false;
  }

  // TODO: recursive glue?
  if (((flags[a].value & Flags::kGlued) && glue[a].parent_id == b) ||
      (flags[b].value & Flags::kGlued) && glue[b].parent_id == a) {
    return false;
  }

  return true;
}

};  // namespace

void CollisionSystem::Solve(const std::vector<Position> &positions,
                            const std::vector<Collider> &colliders,
                            const std::vector<Motion> &motion,
                            const std::vector<Flags> &flags,
                            const std::vector<Glue> &glue, const float dt,
                            std::vector<Event> &out_events) {
  cache_bvh_kvs_.clear();
  cache_object_swept_bounds_.clear();
  for (int id = 0; id < colliders.size(); ++id) {
    float radius = colliders[id].radius;
    AABB bounds = AABB::FromCenterAndHalfExtents(
        positions[id].value, Vector3{radius, radius, radius});
    bounds.Encapsulate(AABB::FromCenterAndHalfExtents(
        motion[id].new_position, Vector3{radius, radius, radius}));
    cache_bvh_kvs_.push_back(BVH::KV(bounds, id));
    cache_object_swept_bounds_.push_back(bounds);
  }
  cache_bvh_.Rebuild(cache_bvh_kvs_);

  std::vector<BVH::KV> buffer;
  for (int id = 0; id < colliders.size(); ++id) {
    buffer.clear();
    cache_bvh_.Overlap(cache_object_swept_bounds_[id], buffer);
    for (const auto &kv : buffer) {
      if (Eligible(colliders, flags, glue, matrix_, id, kv.value)) {
        float t = CollisionTime(positions, colliders, motion, id, kv.value, dt);
        if (t <= dt) {
          out_events.push_back(Event(Collision{id, kv.value, t}));
        }
      }
    }
  }
}
};  // namespace vstr