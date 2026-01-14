// Benchmark: nanoPCL vs PCL RANSAC Plane Segmentation
//
// Compares:
//   - nanoPCL (OpenMP enabled)
//   - nanoPCL (OpenMP disabled / single thread)
//   - PCL SACSegmentation
//
// Build:
//   g++ -O3 -march=native -fopenmp -std=c++17 bench_ransac_plane_vs_pcl.cpp \
//       -o bench_ransac_plane_vs_pcl \
//       -I../../include -I/usr/include/eigen3 \
//       -I/usr/include/pcl-1.10 \
//       $(pkg-config --libs pcl_segmentation-1.10 pcl_common-1.10 pcl_sample_consensus-1.10)
//
// Run:
//   ./bench_ransac_plane_vs_pcl

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/segmentation/ransac_plane.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#ifdef _OPENMP
#include <omp.h>
#endif

using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Test Data Generation
// =============================================================================

void generateTestData(size_t n_ground, size_t n_obstacles, float noise,
                      npcl::PointCloud& nano_cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> xy_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> noise_dist(-noise, noise);
  std::uniform_real_distribution<float> z_dist(0.5f, 3.0f);

  size_t total = n_ground + n_obstacles;

  // nanoPCL
  nano_cloud.clear();
  nano_cloud.reserve(total);

  // PCL
  pcl_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->reserve(total);

  // Ground plane (z ≈ 0)
  for (size_t i = 0; i < n_ground; ++i) {
    float x = xy_dist(rng);
    float y = xy_dist(rng);
    float z = noise_dist(rng);
    nano_cloud.add(npcl::Point(x, y, z));
    pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
  }

  // Random obstacles above ground
  for (size_t i = 0; i < n_obstacles; ++i) {
    float x = xy_dist(rng);
    float y = xy_dist(rng);
    float z = z_dist(rng);
    nano_cloud.add(npcl::Point(x, y, z));
    pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
  }
}

// =============================================================================
// Benchmark Helpers
// =============================================================================

struct BenchResult {
  double time_ms;
  size_t inliers;
  int iterations;
};

template <typename Func>
BenchResult benchmark(Func func, int warmup = 3, int runs = 10) {
  // Warmup
  for (int i = 0; i < warmup; ++i) func();

  // Measure
  double total_ms = 0.0;
  BenchResult result{};

  for (int i = 0; i < runs; ++i) {
    auto start = Clock::now();
    auto [inliers, iters] = func();
    auto end = Clock::now();

    total_ms += std::chrono::duration<double, std::milli>(end - start).count();
    result.inliers = inliers;
    result.iterations = iters;
  }

  result.time_ms = total_ms / runs;
  return result;
}

// =============================================================================
// nanoPCL Benchmark (OpenMP controlled)
// =============================================================================

std::pair<size_t, int> runNanoPCL(const npcl::PointCloud& cloud,
                                   float threshold, int max_iters,
                                   int num_threads) {
#ifdef _OPENMP
  omp_set_num_threads(num_threads);
#endif

  auto result = npcl::segmentation::segmentPlane(cloud, threshold, max_iters, 0.99);
  return {result.inliers.size(), result.iterations};
}

// =============================================================================
// PCL Benchmark
// =============================================================================

std::pair<size_t, int> runPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               float threshold, int max_iters) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iters);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // PCL doesn't expose iteration count easily, return max_iters as proxy
  return {inliers->indices.size(), max_iters};
}

// =============================================================================
// Main Benchmark
// =============================================================================

int main() {
  std::cout << "=== nanoPCL vs PCL: RANSAC Plane Segmentation ===" << std::endl;
  std::cout << std::endl;

#ifdef _OPENMP
  int max_threads = omp_get_max_threads();
  std::cout << "OpenMP: Enabled (max threads: " << max_threads << ")" << std::endl;
#else
  int max_threads = 1;
  std::cout << "OpenMP: Disabled" << std::endl;
#endif
  std::cout << std::endl;

  // Parameters
  float threshold = 0.1f;
  int max_iters = 1000;

  // Test configurations
  std::vector<std::pair<size_t, size_t>> configs = {
      {8000, 2000},      // 10K points
      {40000, 10000},    // 50K points
      {80000, 20000},    // 100K points
      {400000, 100000},  // 500K points
  };

  std::cout << std::fixed << std::setprecision(3);

  // Header
  std::cout << std::setw(8) << "Points"
            << std::setw(14) << "nanoPCL(1T)"
            << std::setw(14) << "nanoPCL(MT)"
            << std::setw(12) << "PCL"
            << std::setw(12) << "vs PCL(1T)"
            << std::setw(12) << "vs PCL(MT)"
            << std::endl;
  std::cout << std::string(72, '-') << std::endl;

  for (const auto& [n_ground, n_obstacles] : configs) {
    size_t total = n_ground + n_obstacles;

    // Generate test data
    npcl::PointCloud nano_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
    generateTestData(n_ground, n_obstacles, 0.02f, nano_cloud, pcl_cloud);

    // nanoPCL single thread
    auto nano_1t = benchmark([&]() {
      return runNanoPCL(nano_cloud, threshold, max_iters, 1);
    });

    // nanoPCL multi-thread
    auto nano_mt = benchmark([&]() {
      return runNanoPCL(nano_cloud, threshold, max_iters, max_threads);
    });

    // PCL
    auto pcl_result = benchmark([&]() {
      return runPCL(pcl_cloud, threshold, max_iters);
    });

    // Speedup calculations
    double speedup_1t = pcl_result.time_ms / nano_1t.time_ms;
    double speedup_mt = pcl_result.time_ms / nano_mt.time_ms;

    std::cout << std::setw(8) << total
              << std::setw(12) << nano_1t.time_ms << "ms"
              << std::setw(12) << nano_mt.time_ms << "ms"
              << std::setw(10) << pcl_result.time_ms << "ms"
              << std::setw(11) << speedup_1t << "x"
              << std::setw(11) << speedup_mt << "x"
              << std::endl;
  }

  std::cout << std::endl;

  // ==========================================================================
  // Detailed comparison at 100K points
  // ==========================================================================
  std::cout << "=== Detailed Comparison (100K points) ===" << std::endl;
  std::cout << std::endl;

  npcl::PointCloud nano_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
  generateTestData(80000, 20000, 0.02f, nano_cloud, pcl_cloud);

  // nanoPCL 1 thread
  auto nano_1t = benchmark([&]() {
    return runNanoPCL(nano_cloud, threshold, max_iters, 1);
  }, 5, 20);

  // nanoPCL multi-thread
  auto nano_mt = benchmark([&]() {
    return runNanoPCL(nano_cloud, threshold, max_iters, max_threads);
  }, 5, 20);

  // PCL
  auto pcl_result = benchmark([&]() {
    return runPCL(pcl_cloud, threshold, max_iters);
  }, 5, 20);

  std::cout << "nanoPCL (1 thread):   " << std::setw(8) << nano_1t.time_ms
            << " ms, " << nano_1t.inliers << " inliers, "
            << nano_1t.iterations << " iters (adaptive)" << std::endl;

  std::cout << "nanoPCL (" << max_threads << " threads):  " << std::setw(8)
            << nano_mt.time_ms << " ms, " << nano_mt.inliers << " inliers, "
            << nano_mt.iterations << " iters (adaptive)" << std::endl;

  std::cout << "PCL:                  " << std::setw(8) << pcl_result.time_ms
            << " ms, " << pcl_result.inliers << " inliers" << std::endl;

  std::cout << std::endl;
  std::cout << "Speedup vs PCL:" << std::endl;
  std::cout << "  nanoPCL (1T):  " << pcl_result.time_ms / nano_1t.time_ms << "x faster" << std::endl;
  std::cout << "  nanoPCL (MT):  " << pcl_result.time_ms / nano_mt.time_ms << "x faster" << std::endl;
  std::cout << "  OpenMP gain:   " << nano_1t.time_ms / nano_mt.time_ms << "x" << std::endl;

  std::cout << std::endl;
  std::cout << "Benchmark complete." << std::endl;

  return 0;
}
