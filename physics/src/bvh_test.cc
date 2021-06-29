#include "bvh.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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
  bvh.Find(GetParam().needle, hits);
  EXPECT_THAT(hits, testing::ElementsAreArray(GetParam().expect))
      << "called bvh.Find(" << GetParam().needle << ", #vector_reference).";
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
        }),
    [](const testing::TestParamInfo<BVHTest::ParamType>& tc) {
      return tc.param.comment;
    });

}  // namespace
}  // namespace vstr