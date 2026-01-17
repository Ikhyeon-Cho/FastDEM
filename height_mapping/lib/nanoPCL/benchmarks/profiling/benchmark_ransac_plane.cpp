// Benchmark: RANSAC Plane Segmentation Performance
//
// Measures:
//   - Throughput at different point cloud sizes
//   - OpenMP parallelization speedup
//   - Adaptive iteration effectiveness
//
// Build:
//   g++ -O3 -march=native -fopenmp -std=c++17 bench_ransac_plane.cpp \
//       -o bench_ransac_plane -I../../include -I/usr/include/eigen3
//
// Run:
//   ./bench_ransac_plane

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/segmentation/ransac_plane.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace npcl;

// =============================================================================
// Test Data Generation
// =============================================================================

PointCloud generateTestCloud(size_t n_ground, size_t n_obstacles, float noise = 0.02f) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> xy_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> noise_dist(-noise, noise);
  std::uniform_real_distribution<float> z_dist(0.5f, 3.0f);

  PointCloud cloud;
  cloud.reserve(n_ground + n_obstacles);

  // Ground plane (z ≈ 0)
  for (size_t i = 0; i < n_ground; ++i) {
    float x = xy_dist(rng);
    float y = xy_dist(rng);
    float z = noise_dist(rng);
    cloud.add(Point(x, y, z));
  }

  // Random obstacles above ground
  for (size_t i = 0; i < n_obstacles; ++i) {
    float x = xy_dist(rng);
    float y = xy_dist(rng);
    float z = z_dist(rng);
    cloud.add(Point(x, y, z));
  }

  return cloud;
}

// =============================================================================
// Benchmark Runner
// =============================================================================

struct BenchResult {
  double time_ms;
  size_t inliers;
  int iterations;
  double fitness;
};

template <typename Func>
BenchResult benchmark(Func func, int warmup = 3, int iterations = 10) {
  // Warmup
  for (int i = 0; i < warmup; ++i) {
    func();
  }

  // Measure
  double total_ms = 0.0;
  BenchResult result{};

  for (int i = 0; i < iterations; ++i) {
    auto start = Clock::now();
    auto res = func();
    auto end = Clock::now();

    total_ms += std::chrono::duration<double, std::milli>(end - start).count();
    result.inliers = res.inliers.size();
    result.iterations = res.iterations;
    result.fitness = res.fitness;
  }

  result.time_ms = total_ms / iterations;
  return result;
}

// =============================================================================
// Main Benchmark
// =============================================================================

int main() {
  std::cout << "=== RANSAC Plane Segmentation Benchmark ===" << std::endl;
  std::cout << std::endl;

#ifdef _OPENMP
  std::cout << "OpenMP: Enabled (max threads: " << omp_get_max_threads() << ")"
            << std::endl;
#else
  std::cout << "OpenMP: Disabled" << std::endl;
#endif
  std::cout << std::endl;

  // Test configurations
  std::vector<std::pair<size_t, size_t>> configs = {
      {8000, 2000},      // 10K points (80% ground)
      {40000, 10000},    // 50K points
      {80000, 20000},    // 100K points
      {400000, 100000},  // 500K points
  };

  segmentation::RansacConfig config;
  config.distance_threshold = 0.1f;
  config.max_iterations = 1000;
  config.probability = 0.99;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << std::setw(10) << "Points" << std::setw(12) << "Time(ms)"
            << std::setw(12) << "Inliers" << std::setw(10) << "Fitness"
            << std::setw(10) << "Iters" << std::setw(15) << "Throughput"
            << std::endl;
  std::cout << std::string(69, '-') << std::endl;

  for (const auto& [n_ground, n_obstacles] : configs) {
    size_t total = n_ground + n_obstacles;
    auto cloud = generateTestCloud(n_ground, n_obstacles);

    auto result = benchmark([&]() {
      return segmentation::segmentPlane(cloud, config.distance_threshold,
                                        config.max_iterations, config.probability);
    });

    double throughput = total / result.time_ms / 1000.0;  // M points/sec

    std::cout << std::setw(10) << total << std::setw(12) << result.time_ms
              << std::setw(12) << result.inliers << std::setw(9)
              << result.fitness * 100 << "%" << std::setw(10) << result.iterations
              << std::setw(12) << throughput << " Mp/s" << std::endl;
  }

  std::cout << std::endl;

  // ==========================================================================
  // Adaptive vs Fixed Iteration Comparison
  // ==========================================================================
  std::cout << "=== Adaptive vs Fixed Iterations (100K points) ===" << std::endl;
  std::cout << std::endl;

  auto cloud_100k = generateTestCloud(80000, 20000);

  // With adaptive (probability = 0.99)
  config.probability = 0.99;
  auto adaptive_result = benchmark([&]() {
    return segmentation::segmentPlane(cloud_100k, config.distance_threshold,
                                      config.max_iterations, config.probability);
  });

  // Fixed max iterations (disable adaptive by setting probability very high)
  config.probability = 0.9999999;
  auto fixed_result = benchmark([&]() {
    return segmentation::segmentPlane(cloud_100k, config.distance_threshold,
                                      100,  // Force exactly 100 iterations
                                      config.probability);
  });

  std::cout << "Adaptive (p=0.99):  " << std::setw(8) << adaptive_result.time_ms
            << " ms, " << adaptive_result.iterations << " iterations" << std::endl;
  std::cout << "Fixed (100 iters):  " << std::setw(8) << fixed_result.time_ms
            << " ms, " << fixed_result.iterations << " iterations" << std::endl;
  std::cout << "Speedup: " << fixed_result.time_ms / adaptive_result.time_ms << "x"
            << std::endl;

  std::cout << std::endl;

  // ==========================================================================
  // Varying Inlier Ratios
  // ==========================================================================
  std::cout << "=== Varying Inlier Ratios (100K points) ===" << std::endl;
  std::cout << std::endl;

  std::vector<std::pair<size_t, size_t>> ratio_configs = {
      {90000, 10000},  // 90% inliers
      {70000, 30000},  // 70% inliers
      {50000, 50000},  // 50% inliers
      {30000, 70000},  // 30% inliers
  };

  config.probability = 0.99;
  config.max_iterations = 1000;

  std::cout << std::setw(15) << "Ground Ratio" << std::setw(12) << "Time(ms)"
            << std::setw(10) << "Iters" << std::setw(12) << "Fitness"
            << std::endl;
  std::cout << std::string(49, '-') << std::endl;

  for (const auto& [n_ground, n_obstacles] : ratio_configs) {
    auto cloud = generateTestCloud(n_ground, n_obstacles);
    double ratio = 100.0 * n_ground / (n_ground + n_obstacles);

    auto result = benchmark([&]() {
      return segmentation::segmentPlane(cloud, config.distance_threshold,
                                        config.max_iterations, config.probability);
    });

    std::cout << std::setw(14) << ratio << "%" << std::setw(12) << result.time_ms
              << std::setw(10) << result.iterations << std::setw(11)
              << result.fitness * 100 << "%" << std::endl;
  }

  std::cout << std::endl;
  std::cout << "Benchmark complete." << std::endl;

  return 0;
}
