// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "interval_tree.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <random>

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

class PointQueryTest : public testing::TestWithParam<PointQueryTestCase> {};

TEST_P(PointQueryTest, PointQueryTest) {
  IntTree tree;
  for (auto kv : GetParam().data) {
    tree.Insert(kv.first, kv.second);

    auto status = tree.Validate();
    EXPECT_TRUE(status.ok()) << status.message();
  }
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

struct DeleteTestCase {
  const std::string comment;
  const std::vector<const IntTree::KV> data;
  const std::vector<const std::pair<const IntTree::KV, bool>> delete_calls;
};

class DeleteTest : public testing::TestWithParam<DeleteTestCase> {};

TEST_P(DeleteTest, DeleteTest) {
  IntTree tree;
  for (auto kv : GetParam().data) {
    tree.Insert(kv.first, kv.second);
  }

  for (auto delete_call : GetParam().delete_calls) {
    bool found = tree.Delete(delete_call.first);
    EXPECT_EQ(found, delete_call.second)
        << "tree.Delete(" << delete_call.first << ") should return "
        << delete_call.second << ". Tree printout follows: " << tree;

    auto status = tree.Validate();
    EXPECT_TRUE(status.ok()) << status.message();
  }
}

INSTANTIATE_TEST_SUITE_P(
    DeleteTest, DeleteTest,
    testing::Values(
        DeleteTestCase{
            "empty",
            std::vector<const IntTree::KV>{},
            std::vector<const std::pair<const IntTree::KV, bool>>{},
        },
        DeleteTestCase{
            "tree_one",
            kTreeOne,
            std::vector<const std::pair<const IntTree::KV, bool>>{
                std::make_pair(IntTree::KV(Interval{0, 1}, 2), false),
                std::make_pair(IntTree::KV(Interval{0, 1}, 1), true),
                std::make_pair(IntTree::KV(Interval{0, 1}, 1), false),
            },
        },
        DeleteTestCase{
            "tree_two",
            kTreeTwo,
            std::vector<const std::pair<const IntTree::KV, bool>>{
                std::make_pair(IntTree::KV(Interval{0, 1}, 2), false),
                std::make_pair(IntTree::KV(Interval{0, 1}, 1), true),
                std::make_pair(IntTree::KV(Interval{0, 1}, 1), false),
                std::make_pair(IntTree::KV(Interval{1, 2}, 2), true),
            },
        },
        DeleteTestCase{
            "tree_many",
            kTreeMany,
            std::vector<const std::pair<const IntTree::KV, bool>>{
                std::make_pair(IntTree::KV(Interval{0, 3}, 0), true),
                std::make_pair(IntTree::KV(Interval{2, 3}, 1), true),
                std::make_pair(IntTree::KV(Interval{1, 4}, 2), true),
                std::make_pair(IntTree::KV(Interval{0, 10}, 3), true),
                std::make_pair(IntTree::KV(Interval{3, 8}, 4), true),
                std::make_pair(IntTree::KV(Interval{3, 8}, 5), true),
                std::make_pair(IntTree::KV(Interval{3, 8}, 6), true),
                std::make_pair(IntTree::KV(Interval{3, 8}, 7), true),
                std::make_pair(IntTree::KV(Interval{3, 8}, 7), false),
                std::make_pair(IntTree::KV(Interval{0, 10}, 3), false),
                std::make_pair(IntTree::KV(Interval{1, 2}, 9), true),
            },
        }),
    [](const testing::TestParamInfo<DeleteTest::ParamType>& tc) {
      return tc.param.comment;
    });

class TreeFuzzTest : public testing::TestWithParam<int> {};

TEST_P(TreeFuzzTest, TreeFuzzTest) {
  constexpr int sz = 10000;
  std::mt19937 random_generator;
  std::uniform_int_distribution low_dist(-sz, sz);
  std::uniform_int_distribution len_dist(1, sz / 100);
  random_generator.seed(GetParam());
  IntTree tree;

  std::vector<IntTree::KV> data;
  data.reserve(sz);

  for (int i = 0; i < sz; ++i) {
    int x = low_dist(random_generator);
    int y = x + len_dist(random_generator);
    tree.Insert(Interval(x, y), i);
  }

  auto status = tree.Validate();
  EXPECT_TRUE(status.ok()) << status.message();

  std::shuffle(data.begin(), data.end(), random_generator);

  for (auto kv : data) {
    EXPECT_TRUE(tree.Delete(kv));
  }
}

INSTANTIATE_TEST_SUITE_P(TreeFuzzTest, TreeFuzzTest, testing::Range(1, 10));

}  // namespace
}  // namespace vstr