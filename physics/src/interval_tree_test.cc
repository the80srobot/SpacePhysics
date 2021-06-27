#include "interval_tree.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace vstr {
namespace {

TEST(Interval, Comparisons) {
  EXPECT_EQ(Interval(1, 2), Interval(1, 2));
  EXPECT_LT(Interval(1, 2), Interval(1, 3));
  EXPECT_GT(Interval(1, 2), Interval(0, 3));
  EXPECT_GT(Interval(1, 2), Interval(1, 1));
}

using IntTree = IntervalTree<int>;

TEST(IntervalValue, Comparisons) {
  EXPECT_EQ(IntTree::KV(Interval(1, 2), 1), IntTree::KV(Interval(1, 2), 1));
  EXPECT_LT(IntTree::KV(Interval(1, 2), 1), IntTree::KV(Interval(1, 2), 2));
  EXPECT_GT(IntTree::KV(Interval(1, 3), 1), IntTree::KV(Interval(1, 2), 2));
}

const std::vector<IntTree::KV> tree_empty{};

const std::vector<const IntTree::KV> kTreeOne{
    IntTree::KV(Interval{0, 1}, 1),
};

const std::vector<const IntTree::KV> kTreeTwo{
    IntTree::KV(Interval{0, 1}, 1),
    IntTree::KV(Interval{1, 2}, 2),
};

const std::vector<const IntTree::KV> kTreeDuplicate{
    IntTree::KV(Interval{1, 2}, 2),
    IntTree::KV(Interval{1, 2}, 2),
    IntTree::KV(Interval{1, 2}, 2),
};

const std::vector<const IntTree::KV> kTreeMany{
    IntTree::KV(Interval{0, 3}, 0),  IntTree::KV(Interval{2, 3}, 1),
    IntTree::KV(Interval{1, 4}, 2),  IntTree::KV(Interval{0, 10}, 3),
    IntTree::KV(Interval{3, 8}, 4),  IntTree::KV(Interval{3, 8}, 5),
    IntTree::KV(Interval{3, 8}, 6),  IntTree::KV(Interval{3, 8}, 7),
    IntTree::KV(Interval{3, 8}, 7),   // duplicate
    IntTree::KV(Interval{0, 10}, 3),  // duplicate
    IntTree::KV(Interval{1, 2}, 9),
};

struct PointQueryTestCase {
  const std::string comment;
  const std::vector<const IntTree::KV> data;
  const int point;
  const std::vector<const IntTree::KV> expect;
};

class TreeDumper : public testing::EmptyTestEventListener {
 private:
  // Fired after the test ends.
  void OnTestEnd(const testing::TestInfo& test_info) override {
    const testing::TestResult* result = test_info.result();
    if (result->Failed()) {
      return;
    }
  }
};

class PointQueryTest : public testing::TestWithParam<PointQueryTestCase> {};

TEST_P(PointQueryTest, PointQueryTest) {
  IntTree tree;
  for (auto kv : GetParam().data) {
    tree.Insert(kv.first, kv.second);
  }
  //   RecordProperty()
  std::vector<IntTree::KV> results;
  tree.Overlap(GetParam().point, results);
  EXPECT_THAT(results, testing::ElementsAreArray(GetParam().expect))
      << "called tree.Overlap(" << GetParam().point << ", #vector_reference). "
      << "Tree printout follows: " << tree;
}

INSTANTIATE_TEST_SUITE_P(
    PointQueryTest, PointQueryTest,
    testing::Values(
        PointQueryTestCase{
            "empty",
            std::vector<const IntTree::KV>{},
            0,
            std::vector<const IntTree::KV>{},
        },
        PointQueryTestCase{
            "one_element_hit",
            kTreeOne,
            0,
            std::vector<const IntTree::KV>{
                IntTree::KV(Interval(0, 1), 1),
            },
        },
        PointQueryTestCase{
            "one_element_miss",
            kTreeOne,
            1,
            std::vector<const IntTree::KV>{},
        },
        PointQueryTestCase{
            "two_element_hit_1",
            kTreeTwo,
            0,
            std::vector<const IntTree::KV>{
                IntTree::KV(Interval(0, 1), 1),
            },
        },
        PointQueryTestCase{
            "two_element_hit_2",
            kTreeTwo,
            1,
            std::vector<const IntTree::KV>{
                IntTree::KV(Interval(1, 2), 2),
            },
        },
        PointQueryTestCase{
            "two_element_miss",
            kTreeTwo,
            2,
            std::vector<const IntTree::KV>{},
        },
        PointQueryTestCase{
            "duplicates",
            kTreeDuplicate,
            1,
            std::vector<const IntTree::KV>{
                IntTree::KV(Interval(1, 2), 2),
            },
        },
        PointQueryTestCase{
            "tree_many_hits_1",
            kTreeMany,
            0,
            std::vector<const IntTree::KV>{
                IntTree::KV(Interval(0, 3), 0),
                IntTree::KV(Interval(0, 10), 3),
            },
        }),
    [](const testing::TestParamInfo<PointQueryTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace
}  // namespace vstr