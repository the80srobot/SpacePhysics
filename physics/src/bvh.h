#ifndef VSTR_BVH
#define VSTR_BVH

#include <algorithm>
#ifndef NDEBUG
#include <sstream>
#endif
#include "aabb.h"

namespace vstr {

// Basic implemntation of BVH as a binary tree. Each node splits the space along
// the axis where the node's bounding volume has the longest extent. The axis is
// split such that approximately half the data are on either side - this is done
// by finding the median of three elements and then moving elements lesser than
// the median to the left child and other elements to the right child. (A
// modified quickselect.)
//
// Performance of the naive implementation:
//
// Not great. Building a BVH of 1024 elements takes about 1 ms. Each query takes
// about 8000 ns, needing about 250 AABB overlap checks. Consequently, with 1024
// elements in the scene, building the BVH and querying overlap for each element
// needs about 10 ms total.
template <typename T>
class BoundingVolumeHierarchy {
 public:
  using ValueType = T;
  struct KV {
    AABB bounds;
    T value;

    KV(AABB bounds, T value) : bounds(bounds), value(value) {}

    inline bool operator==(const KV &other) const {
      return bounds == other.bounds && value == other.value;
    }
  };

  friend std::ostream &operator<<(std::ostream &os,
                                  const BoundingVolumeHierarchy<T>::KV &kv) {
    return os << "{" << kv.bounds << ", " << kv.value << "}";
  }

  // Clears the BVH and populates it with the new data. This takes about
  // NLog2(N) steps (N = kvs.size()).
  void Rebuild(std::vector<KV> &kvs) {
    nodes_tested_ = 0;
    nodes_.clear();
    const AABB bounds = BoundingVolume(kvs);
    Build(bounds, kvs, 0, kvs.size() - 1);
  }

  void Overlap(AABB needle, std::vector<KV> &hits) { Walk(0, needle, hits); }

  friend std::ostream &operator<<(std::ostream &os,
                                  const BoundingVolumeHierarchy &bvh) {
    os << "BoundingVolumeHierarchy:" << std::endl;
    for (int i = 0; i < bvh.nodes_.size(); ++i) {
      os << "\t #" << i << ": " << bvh.nodes_[i] << std::endl;
    }
    return os;
  }

  int AvgDepth() { return AvgDepth(0, 0).second; }

  std::pair<double, double> AvgDepth(int n, int depth) {
    if (n == kNil) {
      return std::make_pair<double, double>(1, depth);
    }

    std::pair<double, double> l =
        AvgDepth(nodes_[n].children[kLeft], depth + 1);
    std::pair<double, double> r =
        AvgDepth(nodes_[n].children[kRight], depth + 1);

    return std::make_pair(
        l.first + r.first,
        (l.first * l.second + r.first * r.second) / (l.first + r.first));
  }

  int MaxDepth() { return MaxDepth(0); }

  int MaxDepth(int n) {
    if (n == kNil) {
      return 0;
    }
    return 1 + std::max(MaxDepth(nodes_[n].children[kLeft]),
                        MaxDepth(nodes_[n].children[kRight]));
  }

  int MinDepth() { return MinDepth(0); }

  int MinDepth(int n) {
    if (n == kNil) {
      return 0;
    }
    return 1 + std::min(MinDepth(nodes_[n].children[kLeft]),
                        MinDepth(nodes_[n].children[kRight]));
  }

  int NodesTested() { return nodes_tested_; }

 private:
  enum Axis { kXAxis, kYAxis, kZAxis };
  struct Node {
    Node(AABB aabb, T value) : aabb(aabb), value(value) {}
    AABB aabb;
    T value;

    int children[2] = {kNil, kNil};

    inline bool Leaf() const {
      return children[kLeft] == kNil && children[kRight] == kNil;
    }
  };

  friend std::ostream &operator<<(
      std::ostream &os, const BoundingVolumeHierarchy<T>::Node &node) {
    return os << "Node{/*aabb=*/" << node.aabb << ",\t/*value=*/" << node.value
              << ",\t/*children=*/[" << node.children[kLeft] << ",\t"
              << node.children[kRight] << "]}";
  }

  static constexpr int kNil = -1;

  using Direction = int;
  static constexpr int kLeft = 0;
  static constexpr int kRight = 1;

  std::vector<Node> nodes_;

  int nodes_tested_ = 0;

  void Walk(int n, AABB needle, std::vector<KV> &hits) {
    if (n == kNil || nodes_.empty()) {
      return;
    }

    nodes_tested_++;

    if (!nodes_[n].aabb.Overlaps(needle)) {
      return;
    }
    if (nodes_[n].Leaf()) {
      hits.push_back(KV(nodes_[n].aabb, nodes_[n].value));
    } else {
      Walk(nodes_[n].children[kLeft], needle, hits);
      Walk(nodes_[n].children[kRight], needle, hits);
    }
  }

  int Build(const AABB bounds, std::vector<KV> &kvs, int lo, int hi) {
    int n = hi - lo + 1;
    int id = nodes_.size();

    switch (n) {
      case 0:
        return kNil;
      case 1:
        nodes_.push_back(Node(kvs[lo].bounds, kvs[lo].value));
        return id;
      case 2:
        nodes_.push_back(Node(bounds, T()));
        // The order here doesn't matter.
        nodes_[id].children[kLeft] = Build(kvs[lo].bounds, kvs, lo, lo);
        nodes_[id].children[kRight] =
            Build(kvs[lo + 1].bounds, kvs, lo + 1, hi);
        return id;
      default:
        nodes_.push_back(Node(bounds, T()));
        Axis axis = PickSplitAxis(bounds);
        int pivot = HoarePartition(kvs, axis, lo, hi);

        nodes_[id].children[kLeft] =
            Build(BoundingVolume(kvs.cbegin() + lo, kvs.cbegin() + pivot + 1),
                  kvs, lo, pivot);
        nodes_[id].children[kRight] =
            Build(BoundingVolume(kvs.cbegin() + pivot, kvs.cbegin() + hi + 1),
                  kvs, pivot + 1, hi);
        return id;
    }
  }

  static AABB BoundingVolume(const std::vector<KV> &kvs) {
    return BoundingVolume(kvs.cbegin(), kvs.cend());
  }

  static AABB BoundingVolume(typename std::vector<KV>::const_iterator start,
                             typename std::vector<KV>::const_iterator end) {
    AABB result;
    while (start != end) {
      result.Encapsulate(start->bounds);
      ++start;
    }
    return result;
  }

  static Axis PickSplitAxis(AABB bounds) {
    float x = bounds.max.x - bounds.min.x;
    float y = bounds.max.y - bounds.min.y;
    float z = bounds.max.z - bounds.min.z;

    if (x > y) {
      if (x > z) {
        return kXAxis;
      } else {
        return kZAxis;
      }
    } else {
      if (y > z) {
        return kYAxis;
      } else {
        return kZAxis;
      }
    }
  }

  static int HoarePartition(std::vector<KV> &kvs, Axis axis, int lo, int hi) {
    int mid = MedianOfThree(kvs, axis, lo, hi);
    float pivot = Location(kvs[mid].bounds, axis);
    int i = lo - 1;
    int j = hi + 1;
    while (true) {
      do {
        ++i;
      } while (Location(kvs[i].bounds, axis) < pivot);
      do {
        --j;
      } while (Location(kvs[j].bounds, axis) > pivot);
      if (i >= j) {
        return j;
      }
      std::swap(kvs[i], kvs[j]);
    }
  }

  static int MedianOfThree(std::vector<KV> &kvs, Axis axis, int lo, int hi) {
    int mid = (lo + hi) / 2;
    if (Location(kvs[mid].bounds, axis) < Location(kvs[lo].bounds, axis)) {
      std::swap(kvs[mid], kvs[lo]);
    }
    if (Location(kvs[hi].bounds, axis) < Location(kvs[lo].bounds, axis)) {
      std::swap(kvs[hi], kvs[lo]);
    }
    if (Location(kvs[mid].bounds, axis) < Location(kvs[hi].bounds, axis)) {
      std::swap(kvs[mid], kvs[hi]);
    }
    return hi;
  }

  static float Location(AABB bounds, Axis axis) {
    switch (axis) {
      case kXAxis:
        return (bounds.min.x + bounds.max.x) / 2.0f;
      case kYAxis:
        return (bounds.min.y + bounds.max.y) / 2.0f;
      default:
        return (bounds.min.z + bounds.max.z) / 2.0f;
    }
  }
};

}  // namespace vstr

#endif