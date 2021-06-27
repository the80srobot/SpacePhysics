#include <benchmark/benchmark.h>

#include "interval_tree.h"

static void BM_IntervalTreeInsert(benchmark::State& state) {
  vstr::IntervalTree<int> tree;
  int i = 0;
  for (auto _ : state) {
    ++i;
    tree.Insert(vstr::Interval(0, i), i);
  }
}
BENCHMARK(BM_IntervalTreeInsert);

BENCHMARK_MAIN();
