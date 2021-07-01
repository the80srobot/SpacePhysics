#ifndef VSTR_COLLISION_WORLD
#define VSTR_COLLISION_WORLD

#include <iostream>

#include "bvh.h"
#include "component_data.h"
#include "layer_matrix.h"

namespace vstr {

class CollisionWorld {
 public:
  CollisionWorld(LayerMatrix layer_matrix) : matrix_(layer_matrix) {}
  void Compute(const Frame &frame, float dt,
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