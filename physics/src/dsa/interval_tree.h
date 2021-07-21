// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_INTERVAL_TREE
#define VSTR_INTERVAL_TREE

#include <absl/status/statusor.h>
#include <assert.h>

#include <iostream>
#include <optional>
#include <vector>

namespace vstr {

// Half-open interval [low, high) – up to, but excluding the high point.
struct Interval {
  Interval(int low, int high) : low(low), high(high){};
  int low;
  int high;

  inline bool Overlap(const Interval other) const {
    // [0, 1) x [1, 2) => false
    // [0, 2) x [1, 2) => true
    // [0, 0) x [0, 0) => false
    return low < other.high && other.low < high;
  }

  inline bool Empty() const { return low >= high; }
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
//    vector. As long as elements are inserted in increasing order and Delete is
//    only ever called on the last-inserted element, the nodes remain sorted in
//    insertion order.
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
//
// Performance:
//
// Depending on the size and density of the tree, a point overlap query takes
// between 150 and 1200 ns on 2.3 GHz 8-Core Intel Core i9. The throughput is
// betwen 3 and 12 million items looked up per second.
//
// When running at 60 FPS, an application has about 16 ms to process a frame.
// The 16 ms budget is enough to retrieve about 170,000 intervals from a tree
// containing about 60,000 intervals.
//
// Benchmarks are in interval_tree_benchmark.cc. The tree compares favorably to
// std::map when inserting and deleting. (Obviously interval overlap queries are
// not supported by std::map and so can't be compared.)
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

  // DFS iterator that filters by interval overlap.
  class Iterator {
   public:
    using iterator_category = std::input_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = KV;
    using pointer = const KV*;
    using reference = const KV&;

    reference operator*() const { return tree_->nodes_[node_].kv; }
    pointer operator->() { return &tree_->nodes_[node_].kv; }

    Iterator& operator++() {
      node_ = kNil;
      while (!stack_.empty()) {
        const int node = stack_.back();
        stack_.pop_back();
        if (node == kNil) continue;
        if (interval_.low > tree_->nodes_[node].max) continue;

        stack_.push_back(tree_->nodes_[node].children[kLeft]);
        if (interval_.high >= tree_->nodes_[node].interval().low) {
          stack_.push_back(tree_->nodes_[node].children[kRight]);
        }

        // A hit: pause the iterator here.
        if (interval_.Overlap(tree_->nodes_[node].interval())) {
          node_ = node;
          break;
        }
      }
      return *this;
    }

    Iterator operator++(int) {
      auto tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const Iterator& a, const Iterator& b) {
      return a.tree_ == b.tree_ && a.node_ == b.node_;
    }

    friend bool operator!=(const Iterator& a, const Iterator& b) {
      return a.tree_ != b.tree_ || a.node_ != b.node_;
    }

   private:
    Iterator(const IntervalTree* tree, int node, Interval interval)
        : tree_(tree), node_(kNil), interval_(interval), stack_{node} {
      ++(*this);
    }

    Iterator(const IntervalTree* tree, int node)
        : tree_(tree), node_(node), interval_(0, -1), stack_{} {}

    const IntervalTree* tree_;
    int node_;
    const Interval interval_;
    std::vector<int> stack_;

    friend class IntervalTree;  // To call the constructor.
  };

  int Count() const { return nodes_.size(); }

  // Returns the maximum point held in the tree. Must only be called if Count is
  // > 0 (otherwise throws range exception).
  int MaxPoint() const { return nodes_[root_].max; }

  bool Insert(const Interval interval, const T value) {
    int node = BstInsert(interval, value);
    if (node != kNil) {
      FixInsert(node);
      return true;
    }
    return false;
  }

  void MergeInsert(Interval interval, const T value) {
    auto end = End();
    for (auto it = Overlap(Interval(interval.low - 1, interval.high));
         it != end; ++it) {
      if (it->second == value) {
        interval.low = std::min(it->first.low, interval.low);
        interval.high = std::max(it->first.high, interval.high);
        Delete(it);
      }
    }

    Insert(interval, value);
  }

  void Overlap(const int point, std::vector<KV>& hits) const {
    Overlap(Interval{point, point + 1}, hits);
  }

  void Overlap(const Interval interval, std::vector<KV>& hits) const {
    auto end = End();
    for (auto it = Overlap(interval); it != end; ++it) {
      hits.push_back(*it);
    }
  }

  void Overlap(const int point, std::vector<ValueType>& hits) const {
    Overlap(Interval{point, point + 1}, hits);
  }

  void Overlap(const Interval interval, std::vector<ValueType>& hits) const {
    auto end = End();
    for (auto it = Overlap(interval); it != end; ++it) {
      hits.push_back(it->second);
    }
  }

  Iterator Overlap(const Interval interval) const {
    return Iterator(this, root_, interval);
  }

  Iterator Overlap(const int point) const {
    return Iterator(this, root_, Interval{point, point + 1});
  }

  // Returns an iterator pointing to the lowest element in the tree. Note that
  // Iterator is in DFS order and the minimum element, by BFS invariant, has no
  // children. Consequently, this iterator can be dereferenced or passed to
  // Delete, but not usefully incremented.
  Iterator Min() const { return Iterator(this, MinNode(root_)); }

  // Returns an iterator pointing to the highest element in the tree. This
  // iterator can be passed to Delete to efficiently truncate the tree, but
  // cannot be incremented for the same reason as apply to Min.
  Iterator Max() const { return Iterator(this, MaxNode(root_)); }

  Iterator End() const { return Iterator(this, kNil); }

  bool Delete(const KV& interval_value) {
    int n = root_;
    while (n != kNil) {
      KV node_iv = nodes_[n].kv;
      if (interval_value < node_iv) {
        n = nodes_[n].children[kLeft];
      } else if (interval_value > node_iv) {
        n = nodes_[n].children[kRight];
      } else {
        DeleteNode(n);
        return true;
      }
    }

    return false;
  }

  void Delete(const Iterator& it) { DeleteNode(it.node_); }

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

  using Direction = int;
  static constexpr int kLeft = 0;
  static constexpr int kRight = 1;

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
          kv(interval, value),
          max(interval.high) {}

    int parent;
    int children[2];
    Color color;
    KV kv;
    int max;

    const inline Interval& interval() const { return kv.first; }
    const inline T& value() const { return kv.second; }
  };

  friend std::ostream& operator<<(std::ostream& os,
                                  const IntervalTree<T>::Node& node) {
    return os << "parent=" << node.parent << " left=" << node.children[kLeft]
              << " right=" << node.children[kRight] << " color=" << node.color
              << " max=" << node.max << " interval=" << node.interval()
              << " value=" << node.value();
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
      KV node_iv = nodes_[p].kv;
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

  int DeleteNode(const int n) {
    int l = nodes_[n].children[kLeft];
    int r = nodes_[n].children[kRight];

    if (l != kNil && r != kNil) {
      int successor = MinNode(r);
      nodes_[n].kv = nodes_[successor].kv;
      FixBranchMax(n);
      return DeleteNode(successor);
    } else if (l != kNil) {
      // Invariant 4 states that every path from root of any subtree to a leaf
      // passes through the same number of black nodes. Consequently, if a
      // node has only one child, then it must be a red child. This also means
      // that node n must be black, because red nodes are not allowed to have
      // red children by invariant 2. We can replace n with its child, color
      // the latter black and by so doing maintain all RBT invariants.
      nodes_[l].color = kBlack;
      Replace(n, l);
      FixBranchMax(l);
      return l;
    } else if (r != kNil) {
      nodes_[r].color = kBlack;
      Replace(n, r);
      FixBranchMax(r);
      return r;
    } else {
      // This is the only tricky case: removing a black node without any
      // children affects the black depth of this subtree, violating property
      // 4.
      int p = nodes_[n].parent;
      // Pretend the node is already deleted as we fix up the max values in its
      // ancestor nodes.
      nodes_[n].max = nodes_[p].interval().low;
      FixBranchMax(p);
      if (nodes_[n].color == kBlack) {
        FixDoubleBlackNode(n);
      }
      Replace(n, kNil);
      return kNil;
    }
  }

  int MinNode(int n) const {
    while (nodes_[n].children[kLeft] != kNil) {
      n = nodes_[n].children[kLeft];
    }
    return n;
  }

  int MaxNode(int n) const {
    while (nodes_[n].children[kRight] != kNil) {
      n = nodes_[n].children[kRight];
    }
    return n;
  }

  void Replace(const int node, const int newNode) {
    int p = nodes_[node].parent;
    if (p != kNil) {
      if (node == nodes_[p].children[kLeft]) {
        nodes_[p].children[kLeft] = newNode;
      } else {
        nodes_[p].children[kRight] = newNode;
      }
    } else {
      // The node being replaced is the root.
      root_ = newNode;
    }
    if (newNode != kNil) {
      nodes_[newNode].parent = p;
    }
    DeleteStorage(node);
  }

  void FixDoubleBlackNode(const int n) {
    int p = nodes_[n].parent;
    if (p == kNil) {
      return;
    }
    Direction d;
    int s;
    if (n == nodes_[p].children[kLeft]) {
      d = kLeft;
      s = nodes_[p].children[kRight];
    } else {
      d = kRight;
      s = nodes_[p].children[kLeft];
    }

    // If the sibling is red then it must have two or zero black children. So a
    // rotation about the parent will give n a black sibling (possibly the
    // sibling will be nil, which is black).
    if (nodes_[s].color == kRed) {
      Rotate(d, p);
      nodes_[s].color = kBlack;
      nodes_[p].color = kRed;
      s = nodes_[p].children[1 - d];
    }

    // The sibling is black.
    int closeNephew = nodes_[s].children[d];
    int distantNephew = nodes_[s].children[1 - d];

    if (distantNephew != kNil && nodes_[distantNephew].color == kRed) {
      // The distant child of the sibling node is red. After a rotation about
      // the parent node, the sibling node becomes the new root of this
      // subtree, and we keep it at the same color as the original parent.
      // Other nodes are colored black. The subtree now looks topologically
      // the same as before removal.
      Rotate(d, p);
      nodes_[s].color = nodes_[p].color;
      nodes_[p].color = kBlack;
      nodes_[distantNephew].color = kBlack;
    } else if (closeNephew != kNil && nodes_[closeNephew].color == kRed) {
      Rotate(1 - d, s);
      nodes_[closeNephew].color = kBlack;
      nodes_[s].color = kRed;
      // This reduces to the case above - the close nephew is the new sibling
      // node and its distant child is red.
      s = closeNephew;
      distantNephew = nodes_[s].children[1 - d];
      Rotate(d, p);
      nodes_[s].color = nodes_[p].color;
      nodes_[p].color = kBlack;
      nodes_[distantNephew].color = kBlack;
    } else {  // Sibling node and both its children are black.
      nodes_[s].color = kRed;
      if (nodes_[p].color == kRed) {
        nodes_[p].color = kBlack;
      } else {
        FixDoubleBlackNode(p);
      }
    }
  }

  void DeleteStorage(int n) {
    int count = nodes_.size();
    if (n != (count - 1)) {
      nodes_[n] = nodes_[count - 1];
      int p = nodes_[n].parent;
      int l = nodes_[n].children[kLeft];
      int r = nodes_[n].children[kRight];

      if (p == kNil) {
        root_ = n;
      } else {
        Direction d = NodeDirection(count - 1, p);
        nodes_[p].children[d] = n;
      }

      if (l != kNil) {
        nodes_[l].parent = n;
      }

      if (r != kNil) {
        nodes_[r].parent = n;
      }
    }
    nodes_.pop_back();
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
    // TODO: this function can terminate early if we get to a node that's
    // already got the correct values.
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
        nodes_[n].max = nodes_[n].interval().high;
      } else {
        nodes_[n].max = std::max(nodes_[n].interval().high, nodes_[r].max);
      }
    } else if (r == kNil) {
      nodes_[n].max = std::max(nodes_[n].interval().high, nodes_[l].max);
    } else {  // Neither child is nil
      nodes_[n].max = std::max(nodes_[n].interval().high,
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

 public:
  absl::Status Validate() { return Validate(root_, 0).status(); }

 private:
  absl::StatusOr<int> Validate(int n, int blackDepth) {
    if (nodes_.size() == 0) {
      return 0;
    }

    if (root_ >= nodes_.size()) {
      return absl::InternalError(absl::StrCat("storage error: root ", root_,
                                              " is out of range (",
                                              nodes_.size(), "nodes)"));
    }

    if (n == kNil) {
      return blackDepth + 1;
    }

    if (nodes_[n].color == kBlack) {
      blackDepth++;
    }

    int l = nodes_[n].children[kLeft];
    int r = nodes_[n].children[kRight];
    if (l != kNil) {
      auto x = nodes_[n].kv;
      auto y = nodes_[l].kv;

      if (x <= y) {
        return absl::InternalError(absl::StrCat("BST violation: node ", n));
      }
    }

    if (r != kNil) {
      auto x = nodes_[n].kv;
      auto y = nodes_[r].kv;
      if (x >= y) {
        return absl::InternalError(absl::StrCat("BST violation: node ", n));
      }
    }

    if (n == root_ && nodes_[n].color != kBlack) {
      return absl::InternalError(
          absl::StrCat("RB violation 1: node ", n, "is a red root"));
    }

    int p = nodes_[n].parent;
    if (p != kNil && nodes_[n].color == kRed && nodes_[p].color == kRed) {
      return absl::InternalError(
          absl::StrCat("RB violation 2: red node ", n, "has a red parent ", p));
    }

    auto leftDepth = Validate(l, blackDepth);
    if (!leftDepth.ok()) {
      return leftDepth;
    }

    auto rightDepth = Validate(r, blackDepth);
    if (!rightDepth.ok()) {
      return rightDepth;
    }

    if (leftDepth.value() != rightDepth.value()) {
      return absl::InternalError(absl::StrCat(
          "left subtree of node ", n, " has black depth ", leftDepth.value(),
          ", right subtree", rightDepth.value()));
    }

    return leftDepth.value();
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const typename std::pair<Interval, T>& kv) {
  return os << "<interval=" << kv.first << ", value=" << kv.second << ">";
}

}  // namespace vstr

#endif
