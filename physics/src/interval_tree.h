#ifndef VSTR_INTERVAL_TREE
#define VSTR_INTERVAL_TREE

#include <assert.h>

#include <iostream>
#include <optional>
#include <vector>

namespace vstr {

struct Interval {
  Interval(int low, int high) : low(low), high(high){};
  int low;
  int high;
};

bool operator<(const Interval& x, const Interval& y);
bool operator>(const Interval& x, const Interval& y);
bool operator==(const Interval& x, const Interval& y);
std::ostream& operator<<(std::ostream& os, const Interval& interval);

// An interval tree implemented as an augmented red-black tree.
//
// If you've seen a red-black tree before, a couple of implementation choices
// might be surprising.
//
// 1) Nodes are kept in a dense vector and edges are stored as offsets into this
//    vector. As long as Delete is only ever called on the last element, the
//    nodes will be sorted in insertion order. It so happens, that we only ever
//    need to delete the last element, so this allows for great data locality.
//    As a bonus, shallow copies of the tree just work. Returning a subtree
//    would require linear time, however.
//
// 2) The RBT algorithms favor clarity over micro-optimization, which makes them
//    look considerably different from what people implement based on (I think)
//    the wikipedia article. It still performs the same number of rotations, and
//    the performance compares favorably to other versions. (I guess compilers
//    don't need you to reuse variables - who knew.)
//
// For intervals, we define a total order based on the 3-tuple (low, high,
// value). Consequently, T must implement lt, gt and eq comparisons. Multiple
// elements can be inserted for the same interval with different values, and the
// same value can be inserted for different intervals, but inserting the same
// pair of interval and value will have no effect after the first time.
//
// Somewhat obvious: it's usually faster to use one big tree than multiple small
// trees. For this reason, T is usually a discriminated union type of different
// kinds of events in a timeline. In this case, lt, gt and eq operators should
// first compare the enum value used to discriminate the union, then - if more
// than one instance of each is allowed, sort on the actual value.
template <typename T>
class IntervalTree {
 public:
  explicit IntervalTree() = default;
  // IntervalTree can be copied because all of its data are stored inside the
  // class and referenced by index only. However, if T is a move-only type, such
  // as unique_ptr, then IntervalTree is also move-only.
  IntervalTree(const IntervalTree& other) = default;
  IntervalTree& operator=(const IntervalTree& other) = default;

  using ValueType = T;
  using KV = std::pair<Interval, ValueType>;

  int Count() const { return nodes_.size(); }

  bool Insert(const Interval interval, const T value) {
    int node = BstInsert(interval, value);
    if (node != kNil) {
      FixInsert(node);
      return true;
    }
    return false;
  }

  void Overlap(const int point, std::vector<KV>& hits) {
    return SearchPoint(root_, point, hits);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const IntervalTree<T>& tree) {
    int sz = tree.Count();
    os << "IntervalTree of " << sz << " nodes, root=" << tree.root_
       << std::endl;
    for (int i = 0; i < sz; ++i) {
      Node node = tree.nodes_[i];
      os << "\tnode " << i << ": ";
      if (i == tree.root_) {
        os << "(ROOT) ";
      }
      os << node << std::endl;
    }
    return os;
  }

 private:
  static constexpr int kNil = -1;
  enum Direction { kLeft = 0, kRight = 1 };
  enum Color { kRed, kBlack };

  friend std::ostream& operator<<(std::ostream& os, const Color& color) {
    switch (color) {
      case kRed:
        return os << "red";
      case kBlack:
        return os << "black";
      default:
        return os << "!invalid " << static_cast<int>(color);
    }
  }

  struct Node {
    explicit Node(Interval interval, T value)
        : color(kRed),
          parent(kNil),
          children{kNil, kNil},
          interval(interval),
          max(interval.high),
          value(value) {}

    int parent;
    int children[2];
    Color color;
    Interval interval;
    int max;
    T value;
  };

  friend std::ostream& operator<<(std::ostream& os,
                                  const IntervalTree<T>::Node& node) {
    return os << "parent=" << node.parent << " left=" << node.children[kLeft]
              << " right=" << node.children[kRight] << " color=" << node.color
              << " max=" << node.max << " interval=" << node.interval
              << " value=" << node.value;
  }

  void SearchPoint(int node, const int point, std::vector<KV>& hits) {
    if (node == kNil) {
      return;
    }

    if (point > nodes_[node].max) {
      return;
    }

    SearchPoint(nodes_[node].children[kLeft], point, hits);

    if (nodes_[node].interval.low <= point &&
        nodes_[node].interval.high > point) {
      hits.push_back(std::make_pair(nodes_[node].interval, nodes_[node].value));
    }

    if (point >= nodes_[node].interval.low) {
      SearchPoint(nodes_[node].children[kRight], point, hits);
    }
  }

  void FixInsert(int n) {
    if (n == kNil) {
      // No new node inserted (key is duplicate).
      return;
    }
    int p = nodes_[n].parent;

    while (n != root_ && nodes_[p].color == kRed) {
      // node and parent are both red, which violates invariant 2.

      // Because parent is red, it cannot be root and so must have a non-nil
      // parent.
      int g = nodes_[p].parent;
      // The uncle: the other child of g.
      int u = Sibling(p, g);

      // The uncle is red - we can fix the local violation of invariant 2 by
      // recoloring, but this may introduce another violation further up.
      if (u != kNil && nodes_[u].color == kRed) {
        nodes_[u].color = kBlack;
        nodes_[p].color = kBlack;
        nodes_[g].color = kRed;
        // Go up the tree until we reach root.
        n = g;
        p = nodes_[n].parent;
        continue;
      }

      // The uncle is black. There are 4 possible arrangements, which we solve
      // by one or more rotations. Note that rotations maintain the BST
      // invariant, because the conditions for inclusion in the left or right
      // subtree are mirrored. For example, in case 1, P is the left child of G
      // (meaning P < G), G becomes the right child of P (meaning G > P).

      if (p == nodes_[g].children[kLeft] && n == nodes_[p].children[kLeft]) {
        // 1: Both p and n are left children: rotate right about g.
        //       (G-b)              (P-b)
        //       /   \              /   \
        //    (P-r) (U-b)   =>   (N-r) (G-r)
        //    /                            \
        // (N-r)                          (U-b)
        Rotate(kRight, g);
        nodes_[g].color = kRed;
        nodes_[p].color = kBlack;
      } else if (p == nodes_[g].children[kLeft] &&
                 n == nodes_[p].children[kRight]) {
        // 2: p is the left child, n the right child: rotate left about p, then
        // rotate right about g.
        //       (G-b)              (G-b)             (N-b)
        //       /   \              /   \             /   \
        //    (P-r) (U-b)   =>   (N-r) (U-b)   =>  (P-r) (G-r)
        //        \              /                           \
        //       (N-r)         (P-r)                        (U-b)
        //
        // BST invariant: N >= P becomes P < N. Possible violation.
        Rotate(kLeft, p);
        Rotate(kRight, g);
        nodes_[n].color = kBlack;
        nodes_[g].color = kRed;
      } else if (p == nodes_[g].children[kRight] &&
                 n == nodes_[p].children[kRight]) {
        // 3: Both p and n are right children: rotate left about g.
        //       (G-b)              (P-b)
        //       /   \              /   \
        //    (U-b) (P-r)   =>   (G-r) (N-r)
        //              \        /
        //             (N-r)  (U-b)
        Rotate(kLeft, g);
        nodes_[p].color = kBlack;
        nodes_[g].color = kRed;
      } else if (p == nodes_[g].children[kRight] &&
                 n == nodes_[p].children[kLeft]) {
        // 4: p is the right child, n the left child: rotate right about p, then
        // rotate left about g.
        //       (G-b)              (G-b)             (N-b)
        //       /   \              /   \             /   \
        //    (U-b) (P-r)   =>   (U-b) (N-r)   =>  (G-r) (P-r)
        //          /                     \        /
        //       (N-r)                    (P-r) (U-b)
        Rotate(kRight, p);
        Rotate(kLeft, g);
        nodes_[n].color = kBlack;
        nodes_[g].color = kRed;
      } else {
        assert(false);  // Programmer error: logically impossible to reach.
      }
      break;
    }
    nodes_[root_].color = kBlack;
  }

  int BstInsert(const Interval interval, const T value) {
    Node node(interval, value);
    int n = nodes_.size();
    if (n == 0) {
      root_ = 0;
      node.color = kBlack;
      nodes_.push_back(std::move(node));
      return 0;
    }

    int p = root_;
    int c = kNil;
    KV interval_value(interval, value);
    for (;;) {
      Direction direction;
      KV node_iv(nodes_[p].interval, nodes_[p].value);
      if (interval_value < node_iv) {
        direction = kLeft;
      } else if (interval_value > node_iv) {
        direction = kRight;
      } else {
        // Nothing to do: the interval already exists and has the same value.
        return kNil;
      }

      c = nodes_[p].children[direction];
      if (c == kNil) {
        node.parent = p;
        nodes_.push_back(std::move(node));
        nodes_[p].children[direction] = n;
        FixBranchMax(n);
        return n;
      }
      p = c;
    }
  }

  // Rotates a subtree about N in the given direction, while maintaining the BST
  // invariant. One of N's children becomes the new root of this subtree and N
  // becomes its child.
  //
  // When rotating right, the left child (L) MUST NOT be nil, when rotating left
  // the right child MUST NOT be nil.
  //
  // For example: in this subtree K < L < M < N < O. We rotate right.
  //
  //  BEFORE          =>    AFTER
  //        (N)                (L)
  //       /   \              /   \
  //     (L)   (O)   =>     (K)   (N)
  //    /   \                    /   \
  //  (K)   (M)                (M)   (O)
  void Rotate(Direction dir, int n) {
    // Could be nil if n is root
    int parent = nodes_[n].parent;
    int l = nodes_[n].children[1 - dir];
    int m = nodes_[l].children[dir];

    nodes_[n].children[1 - dir] = m;
    if (m != kNil) {
      nodes_[m].parent = n;
    }

    nodes_[l].children[dir] = n;
    nodes_[n].parent = l;

    nodes_[l].parent = parent;
    FixMax(n);
    FixMax(l);

    if (parent == kNil) {
      // n was root
      root_ = l;
    } else if (nodes_[parent].children[kLeft] == n) {
      nodes_[parent].children[kLeft] = l;
      FixMax(parent);
    } else if (nodes_[parent].children[kRight] == n) {
      nodes_[parent].children[kRight] = l;
      FixMax(parent);
    } else {
      // cannot be reached unless the parent of n has n as neither of its
      // children
      assert(nodes_[parent].children[kLeft] == n ||
             nodes_[parent].children[kRight] == n);
    }
  }

  void FixBranchMax(int n) {
    while (n != kNil) {
      FixMax(n);
      n = nodes_[n].parent;
    }
  }

  void FixMax(int n) {
    int l = nodes_[n].children[kLeft];
    int r = nodes_[n].children[kRight];
    if (l == kNil) {
      if (r == kNil) {
        nodes_[n].max = nodes_[n].interval.high;
      } else {
        nodes_[n].max = std::max(nodes_[n].interval.high, nodes_[r].max);
      }
    } else if (r == kNil) {
      nodes_[n].max = std::max(nodes_[n].interval.high, nodes_[l].max);
    } else {  // Neither child is nil
      nodes_[n].max = std::max(nodes_[n].interval.high,
                               std::max(nodes_[l].max, nodes_[r].max));
    }
  }

  Direction NodeDirection(int node, int parent) {
    assert(parent != kNil);
    Direction direction =
        (nodes_[parent].children[kLeft] == node ? kLeft : kRight);
    assert(nodes_[parent].children[direction] == node);
    return direction;
  }

  int Sibling(int node, int parent) {
    return nodes_[parent].children[1 - NodeDirection(node, parent)];
  }

  int root_ = kNil;

  // Nodes are kept together in this vector. Deleting an element from the middle
  // replaces it with the last element, which is then deleted instead.
  std::vector<Node> nodes_;
};
}  // namespace vstr

#endif