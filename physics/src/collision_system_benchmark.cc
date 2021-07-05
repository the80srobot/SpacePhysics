// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include <benchmark/benchmark.h>

#include <random>

#include "collision_system.h"

namespace vstr {
namespace {

struct Frame {
  std::vector<Position> positions;
  std::vector<Mass> mass;
  std::vector<Motion> motion;
  std::vector<Collider> colliders;
  std::vector<Glue> glue;
  std::vector<Flags> flags;
};
constexpr int kDeltaTime = 1.0 / 60;

void GenerateCluster(const int size, const bool collision,
                     std::mt19937& random_generator, Frame& frame) {
  std::uniform_real_distribution<float> center_rg(-1e6, 1e6);
  std::uniform_real_distribution<float> spread_rg(-100, 100);
  std::uniform_real_distribution<float> radius_rg(0, 100);

  Vector3 cluster_center{center_rg(random_generator),
                         center_rg(random_generator),
                         center_rg(random_generator)};

  for (int i = 0; i < size; ++i) {
    Vector3 offset{spread_rg(random_generator), spread_rg(random_generator),
                   spread_rg(random_generator)};
    Vector3 center = cluster_center + offset;
    float radius = radius_rg(random_generator);
    Vector3 velocity = (collision ? -offset : offset);
    frame.colliders.push_back(Collider{1, radius});
    frame.flags.push_back(Flags{false});
    frame.glue.push_back(Glue{-1});
    frame.positions.push_back(Position{center});
    frame.motion.push_back(Motion::FromPositionAndVelocity(center, velocity));
  }
}

Frame Generate(const int clusters, const int collision_every_n_clusters,
               const int cluster_size, std::mt19937& random_generator) {
  Frame frame;
  for (int i = 0; i < clusters; ++i) {
    GenerateCluster(cluster_size, ((i % collision_every_n_clusters) == 0),
                    random_generator, frame);
  }

  return frame;
}

void BM_CollisionSystem(benchmark::State& state) {
  const int clusters = state.range(0);
  const int collision_every_n_clusters = state.range(1);
  const int cluster_size = state.range(2);
  std::mt19937 random_generator;

  const Frame frame = Generate(clusters, collision_every_n_clusters,
                               cluster_size, random_generator);

  CollisionSystem solver(LayerMatrix(
      std::vector<std::pair<uint32_t, uint32_t>>{std::make_pair(1, 1)}));
  std::vector<Collision> buffer;
  int collisions = 0;
  for (auto _ : state) {
    solver.Solve(frame.positions, frame.colliders, frame.motion, frame.flags,
                 frame.glue, kDeltaTime, buffer);
    collisions += buffer.size();
    buffer.clear();
  }

  state.counters["avg_collisions"] = float(collisions) / state.iterations();
  state.SetItemsProcessed(collisions);
  state.SetComplexityN(clusters * cluster_size);
}
BENCHMARK(BM_CollisionSystem)
    ->ArgsProduct({
        // clusters
        benchmark::CreateRange(1 << 2, 1 << 10, /*multi=*/2),
        // collision_every_n_clusters
        benchmark::CreateRange(1, 8, /*multi=*/2),
        // cluster_size
        benchmark::CreateRange(2, 16, /*multi=*/2),
    })
    ->Complexity();

}  // namespace
}  // namespace vstr

BENCHMARK_MAIN();