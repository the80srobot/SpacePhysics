// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_collision_detector
#define VSTR_collision_detector

#include <iostream>

#include "geometry/bvh.h"
#include "geometry/layer_matrix.h"
#include "types/required_components.h"

namespace vstr {

class CollisionDetector {
 public:
  CollisionDetector(LayerMatrix layer_matrix) : matrix_(layer_matrix) {}
  void DetectCollisions(const std::vector<Transform> &positions,
                        const std::vector<Collider> &colliders,
                        const std::vector<Motion> &motion,
                        const std::vector<Flags> &flags,
                        const std::vector<Glue> &glue, float dt,
                        std::vector<Event> &out_events);

  const inline LayerMatrix &matrix() const { return matrix_; }

 private:
  using BVH = BoundingVolumeHierarchy<Entity>;
  LayerMatrix matrix_;
  BVH cache_bvh_;
  std::vector<BVH::KV> cache_bvh_kvs_;
  std::vector<AABB> cache_object_swept_bounds_;
};

}  // namespace vstr

#endif