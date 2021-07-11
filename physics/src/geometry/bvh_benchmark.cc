// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include <benchmark/benchmark.h>

#include <random>

#include "geometry/bvh.h"

namespace {

using IntBVH = ::vstr::BoundingVolumeHierarchy<int>;
using ::vstr::AABB;
using ::vstr::Vector3;

std::vector<IntBVH::KV> GenerateData(const int count, const int center_max,
                                     const int side_max,
                                     std::mt19937& random_generator) {
  std::uniform_real_distribution<float> center_rg(-center_max, center_max);
  std::uniform_real_distribution<float> side_rg(0, side_max);

  std::vector<IntBVH::KV> data;
  for (int i = 0; i < count; ++i) {
    Vector3 center{center_rg(random_generator), center_rg(random_generator),
                   center_rg(random_generator)};
    Vector3 extents{side_rg(random_generator), side_rg(random_generator),
                    side_rg(random_generator)};
    data.push_back(IntBVH::KV(AABB::FromCenterAndExtents(center, extents), i));
  }
  return data;
}

void BM_BVHRebuild(benchmark::State& state) {
  std::mt19937 random_generator;

  const int count = state.range(0);
  const int center_max = state.range(1);
  const int side_max = state.range(2);

  std::vector<IntBVH::KV> data =
      GenerateData(count, center_max, side_max, random_generator);

  IntBVH bvh;
  int i = 0;
  for (auto _ : state) {
    bvh.Rebuild(data);
    state.PauseTiming();
    std::shuffle(data.begin(), data.end(), random_generator);
    state.ResumeTiming();
    ++i;
  }

  state.SetItemsProcessed(i * count);
  state.SetComplexityN(count);
}
BENCHMARK(BM_BVHRebuild)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 16, /*multi=*/2),  // count
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // center spread
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // size spread
    })
    ->Complexity();

void BM_BVHOverlap(benchmark::State& state) {
  std::mt19937 random_generator;

  const int count = state.range(0);
  const int center_max = state.range(1);
  const int side_max = state.range(2);

  std::vector<IntBVH::KV> data =
      GenerateData(count, center_max, side_max, random_generator);

  IntBVH bvh;
  bvh.Rebuild(data);
  int i = 0;
  int hits = 0;
  std::vector<IntBVH::KV> buffer;
  for (auto _ : state) {
    const AABB needle = data[i % data.size()].bounds;
    buffer.clear();
    bvh.Overlap(needle, buffer);
    hits += buffer.size();
    ++i;
  }

  state.counters["max_depth"] = bvh.MaxDepth();
  state.counters["avg_depth"] = bvh.AvgDepth();
  state.counters["min_depth"] = bvh.MinDepth();
  state.counters["hits"] = hits;
  state.counters["hits_per_query"] = float(hits) / i;
  state.counters["queries"] = i;
  state.counters["checks"] = bvh.NodesTested();
  state.counters["checks_per_query"] = float(bvh.NodesTested()) / i;
  state.counters["pcnt_checked_per_query"] =
      100 * (float(bvh.NodesTested() / i) / float(count));
  state.counters["checks_per_hit"] = float(bvh.NodesTested()) / hits;
  state.counters["pcnt_checked_per_hit"] =
      100 * (float(bvh.NodesTested() / hits) / float(count));
  state.SetComplexityN(count);
  state.SetItemsProcessed(hits);
}
BENCHMARK(BM_BVHOverlap)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 18, /*multi=*/2),  // count
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // center spread
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // size spread
    })
    ->Complexity();

}  // namespace

BENCHMARK_MAIN();