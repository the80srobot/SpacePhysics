// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_collision_system
#define VSTR_collision_system

#include <iostream>

#include "bvh.h"
#include "component_data.h"
#include "layer_matrix.h"

namespace vstr {

class CollisionSystem {
 public:
  CollisionSystem(LayerMatrix layer_matrix) : matrix_(layer_matrix) {}
  void Solve(const Frame &frame, float dt,
             std::vector<CollisionEvent> &out_events);

 private:
  using BVH = BoundingVolumeHierarchy<int>;
  LayerMatrix matrix_;
  BVH cache_bvh_;
  std::vector<BVH::KV> cache_bvh_kvs_;
  std::vector<AABB> cache_object_swept_bounds_;
};

}  // namespace vstr

#endif