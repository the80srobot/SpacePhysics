#include "bvh.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <random>

namespace vstr {
namespace {

using IntBVH = BoundingVolumeHierarchy<int>;

struct TestCase {
  const std::string comment;
  std::vector<IntBVH::KV> data;
  const AABB needle;
  std::vector<IntBVH::KV> expect;
};

class BVHTest : public testing::TestWithParam<TestCase> {};

TEST_P(BVHTest, BVHTest) {
  IntBVH bvh;
  std::vector<IntBVH::KV> data(GetParam().data);
  bvh.Rebuild(data);

  std::vector<IntBVH::KV> hits;
  bvh.Overlap(GetParam().needle, hits);
  EXPECT_THAT(hits, testing::UnorderedElementsAreArray(GetParam().expect))
      << "called bvh.Find(" << GetParam().needle << ", #vector_reference). "
      << bvh.DebugOverlap(GetParam().needle);
}

INSTANTIATE_TEST_SUITE_P(
    BVHTest, BVHTest,
    testing::Values(
        TestCase{
            "empty",
            std::vector<IntBVH::KV>{},
            AABB(Vector3(), Vector3()),
            std::vector<IntBVH::KV>{},
        },
        TestCase{
            "one",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
            },
            AABB(Vector3{0, 0, 0}, Vector3{0, 0, 0}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
            },
        },
        TestCase{
            "one_miss",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{1, 1, 1}), 1),
            },
            AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}),
            std::vector<IntBVH::KV>{},
        },
        TestCase{
            "two_hit_miss",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{1, 0, 0}, Vector3{2, 2, 2}), 1),
            },
            AABB(Vector3{0, 0, 0}, Vector3{0, 0, 0}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
            },
        },
        TestCase{
            "two_hit_hit",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{1, 0, 0}, Vector3{2, 2, 2}), 1),
            },
            AABB(Vector3{-10, -10, -10}, Vector3{10, 10, 10}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{1, 0, 0}, Vector3{2, 2, 2}), 1),
            },
        },
        TestCase{
            "intervals_are_closed",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
            },
            AABB(Vector3{3, 0, 0}, Vector3{4, 8, 8}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
            },
        },
        TestCase{
            "zero_size_volume_found",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{2, 2, 2}), 1),
            },
            AABB(Vector3{-10, -10, -10}, Vector3{10, 10, 10}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{2, 2, 2}), 1),
            },
        },
        TestCase{
            "zero_size_needle_match",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{2, 2, 2}), 1),
            },
            AABB(Vector3{2, 2, 2}, Vector3{2, 2, 2}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{2, 2, 2}), 1),
            },
        },
        TestCase{
            "three_hits",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, -1, -1}, Vector3{1, 1, 1}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{0, 0, 0}), 1),
            },
            AABB(Vector3{-10, -10, -10}, Vector3{10, 10, 10}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, -1, -1}, Vector3{1, 1, 1}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{0, 0, 0}), 1),
            },
        },
        TestCase{
            "duplicates_are_preserved",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, -1, -1}, Vector3{1, 1, 1}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{0, 0, 0}), 1),
            },
            AABB(Vector3{-10, -10, -10}, Vector3{10, 10, 10}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{-1, -1, -1}, Vector3{1, 1, 1}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{3, 3, 3}), 1),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{0, 0, 0}), 1),
            },
        },
        TestCase{
            "larger_bvh",
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{2, 2, 2}), 1),
                IntBVH::KV(AABB(Vector3{-10, -5, 0}, Vector3{-9, -4, 1}), 2),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{2, 2, 2}), 3),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{100, 0.5, 1}), 4),
                IntBVH::KV(AABB(Vector3{-50, -50, -50}, Vector3{-49, -49, -49}),
                           5),
                IntBVH::KV(AABB(Vector3{20, 0, 0}, Vector3{21, 5, 1}), 6),
                IntBVH::KV(AABB(Vector3{-2, -2, -2}, Vector3{2, 2, 2}), 7),
                IntBVH::KV(AABB(Vector3{2, 2, 2}, Vector3{2, 2, 2}), 8),
                IntBVH::KV(AABB(Vector3{0, 0, 0}, Vector3{1000, 0, 0}), 9),
            },
            AABB(Vector3{5, 5, 0}, Vector3{21.5, 10, 10}),
            std::vector<IntBVH::KV>{
                IntBVH::KV(AABB(Vector3{20, 0, 0}, Vector3{21, 5, 1}), 6),
            },
        }),
    [](const testing::TestParamInfo<BVHTest::ParamType>& tc) {
      return tc.param.comment;
    });

class BVHFuzzTest : public testing::TestWithParam<int> {};

TEST_P(BVHFuzzTest, BVHFuzzTest) {
  const int count = GetParam();

  std::mt19937 random_generator;
  random_generator.seed(GetParam());
  std::uniform_real_distribution<float> center_rg(-100, 100);
  std::uniform_real_distribution<float> side_rg(0, 20);

  std::vector<IntBVH::KV> data;
  for (int i = 0; i < count; ++i) {
    Vector3 center{center_rg(random_generator), center_rg(random_generator),
                   center_rg(random_generator)};
    Vector3 extents{side_rg(random_generator), side_rg(random_generator),
                    side_rg(random_generator)};
    data.push_back(IntBVH::KV(AABB::FromCenterAndExtents(center, extents), i));
  }
  IntBVH bvh;
  bvh.Rebuild(data);

  // All the stuff we instered should match when used as a needle.
  std::vector<IntBVH::KV> buffer;
  for (const auto& kv : data) {
    buffer.clear();
    bvh.Overlap(kv.bounds, buffer);
    EXPECT_THAT(buffer, testing::Contains(kv)) << bvh.DebugOverlap(kv.bounds);
  }
}

INSTANTIATE_TEST_SUITE_P(BVHFuzzTest, BVHFuzzTest,
                         testing::Values(1, 2, 3, 4, 5, 6, 7, 8, 1 << 4, 1 << 5,
                                         1 << 6, 1 << 7, 1 << 8, 1 << 9,
                                         1 << 10));

}  // namespace
}  // namespace vstr