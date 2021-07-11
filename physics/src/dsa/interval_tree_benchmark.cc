// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include <benchmark/benchmark.h>

#include <random>

#include "interval_tree.h"

static std::vector<vstr::IntervalTree<int>::KV> GenerateData(
    const int size, const int overlap, const int spread = 0) {
  std::vector<vstr::IntervalTree<int>::KV> data;
  data.reserve(size);
  const int length = overlap + 1;

  for (int i = 0; i < size; ++i) {
    int low = i;
    int high = low + length;
    if (spread != 0) {
      low += (i % spread) / 2;
      high += (i % spread) / 2;
    }
    data.push_back(vstr::IntervalTree<int>::KV(vstr::Interval(low, high), i));
  }
  return data;
}

static void BM_IntervalTreeBuild(benchmark::State& state) {
  const int size = state.range(0);
  const int overlap = state.range(1);
  const int spread = state.range(2);
  auto insert_data = GenerateData(size, overlap, spread);

  for (auto _ : state) {
    vstr::IntervalTree<int> tree;
    for (const auto& kv : insert_data) {
      tree.Insert(kv.first, kv.second);
    }
  }

  state.SetItemsProcessed(state.iterations() * size);
  state.SetBytesProcessed(state.iterations() * size *
                          sizeof(vstr::IntervalTree<int>::KV));
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_IntervalTreeBuild)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 16, /*multi=*/2),  // size
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // overlap
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // spread
    })
    ->Complexity();

static void BM_StdMapBuild(benchmark::State& state) {
  const int size = state.range(0);
  const int overlap = state.range(1);
  const int spread = state.range(2);
  auto insert_data = GenerateData(size, overlap, spread);

  for (auto _ : state) {
    std::map<vstr::Interval, int> map;
    for (const auto& kv : insert_data) {
      map.insert(kv);
    }
  }

  state.SetItemsProcessed(state.iterations() * size);
  state.SetBytesProcessed(state.iterations() * size *
                          sizeof(vstr::IntervalTree<int>::KV));
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_StdMapBuild)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 16, /*multi=*/2),  // size
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // overlap
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // spread
    })
    ->Complexity();

static void BM_IntervalTreeOverlapPointDense(benchmark::State& state) {
  const int size = state.range(0);
  const int overlap = state.range(1);
  const int spread = state.range(2);
  auto insert_data = GenerateData(size, overlap, spread);
  vstr::IntervalTree<int> tree;
  for (const auto& kv : insert_data) {
    tree.Insert(kv.first, kv.second);
  }

  std::vector<int> point_queries;
  point_queries.reserve(size);
  for (const auto& kv : insert_data) {
    point_queries.push_back((kv.first.low + kv.first.high) / 2);
  }

  std::vector<vstr::IntervalTree<int>::KV> buffer;
  int i = 0;
  int hits = 0;
  for (auto _ : state) {
    ++i;
    buffer.clear();
    tree.Overlap(point_queries[i % size], buffer);
    hits += buffer.size();
  }
  state.SetItemsProcessed(hits);
  state.SetComplexityN(size);
  state.SetBytesProcessed(hits * sizeof(vstr::IntervalTree<int>::KV));
}
BENCHMARK(BM_IntervalTreeOverlapPointDense)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 16, /*multi=*/2),  // size
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // overlap
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // spread
    })
    ->Complexity();

static void BM_IntervalTreeDeleteReinsertRandom(benchmark::State& state) {
  const int size = state.range(0);
  const int overlap = state.range(1);
  const int spread = state.range(2);
  auto insert_data = GenerateData(size, overlap, spread);
  vstr::IntervalTree<int> tree;
  for (const auto& kv : insert_data) {
    tree.Insert(kv.first, kv.second);
  }

  std::mt19937 random_generator;
  std::vector<vstr::IntervalTree<int>::KV> delete_ops(insert_data);
  std::shuffle(delete_ops.begin(), delete_ops.end(), random_generator);

  int i = 0;
  for (auto _ : state) {
    ++i;
    auto entry = delete_ops[i % delete_ops.size()];
    tree.Delete(entry);
    tree.Insert(entry.first, entry.second);
  }
  state.SetItemsProcessed(i);
  state.SetComplexityN(size);
  state.SetBytesProcessed(i * sizeof(vstr::IntervalTree<int>::KV));
}
BENCHMARK(BM_IntervalTreeDeleteReinsertRandom)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 16, /*multi=*/2),  // size
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // overlap
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // spread
    })
    ->Complexity();

static void BM_StdMapDeleteReinsertRandom(benchmark::State& state) {
  const int size = state.range(0);
  const int overlap = state.range(1);
  const int spread = state.range(2);
  auto insert_data = GenerateData(size, overlap, spread);
  std::map<vstr::Interval, int> map;

  for (const auto& kv : insert_data) {
    map.insert(kv);
  }

  std::mt19937 random_generator;
  std::vector<vstr::IntervalTree<int>::KV> delete_ops(insert_data);
  std::shuffle(delete_ops.begin(), delete_ops.end(), random_generator);

  int i = 0;
  for (auto _ : state) {
    ++i;
    auto entry = delete_ops[i % delete_ops.size()];
    map.erase(entry.first);
    map.insert(entry);
  }
  state.SetItemsProcessed(i);
  state.SetComplexityN(size);
  state.SetBytesProcessed(i * sizeof(vstr::IntervalTree<int>::KV));
}
BENCHMARK(BM_StdMapDeleteReinsertRandom)
    ->ArgsProduct({
        benchmark::CreateRange(1 << 2, 1 << 16, /*multi=*/2),  // size
        benchmark::CreateDenseRange(0, 8, /*step=*/8),         // overlap
        benchmark::CreateRange(1, 1 << 8, /*multi=*/16),       // spread
    })
    ->Complexity();

BENCHMARK_MAIN();
