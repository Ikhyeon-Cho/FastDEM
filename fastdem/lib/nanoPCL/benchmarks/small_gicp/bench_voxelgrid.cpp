// nanoPCL vs small_gicp Benchmark: Voxel Grid Downsampling
//
// Build:
//   g++ -O3 -march=native -fopenmp -I../../include \
//       -I/home/ikhyeon/ros/test_ws/src/small_gicp/include \
//       bench_voxelgrid.cpp -o bench_voxelgrid

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/filters/downsample.hpp>

// small_gicp
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/downsampling.hpp>

using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Helpers
// =============================================================================

nanopcl::PointCloud generateNanoPCL(size_t n, unsigned seed = 42) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

  nanopcl::PointCloud cloud;
  cloud.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    cloud.add(dist(rng), dist(rng), dist(rng));
  }
  return cloud;
}

small_gicp::PointCloud generateSmallGicp(size_t n, unsigned seed = 42) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-100.0, 100.0);

  small_gicp::PointCloud cloud;
  cloud.points.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    cloud.points.emplace_back(dist(rng), dist(rng), dist(rng), 1.0);
  }
  return cloud;
}

template <typename Func>
double benchmark(Func&& func, int warmup = 2, int iterations = 10) {
  // Warmup
  for (int i = 0; i < warmup; ++i) {
    func();
  }

  // Measure
  double total_ms = 0;
  for (int i = 0; i < iterations; ++i) {
    auto start = Clock::now();
    func();
    auto end = Clock::now();
    total_ms += std::chrono::duration<double, std::milli>(end - start).count();
  }
  return total_ms / iterations;
}

// =============================================================================
// Benchmarks
// =============================================================================

void benchmarkVoxelGrid(size_t n, float voxel_size) {
  std::cout << "\n=== Voxel Grid Downsampling ===" << std::endl;
  std::cout << "Points: " << n << ", Voxel size: " << voxel_size << "m\n"
            << std::endl;

  // Generate data with same seed
  auto cloud_npcl = generateNanoPCL(n, 42);
  auto cloud_sg = generateSmallGicp(n, 42);

  size_t result_npcl = 0, result_sg = 0;

  // nanoPCL (CENTROID mode - matches small_gicp behavior)
  double time_npcl = benchmark([&]() {
    auto result = nanopcl::filters::voxelGrid(cloud_npcl, voxel_size,
                                           nanopcl::filters::VoxelMode::CENTROID);
    result_npcl = result.size();
  });

  // small_gicp
  double time_sg = benchmark([&]() {
    auto result = small_gicp::voxelgrid_sampling(cloud_sg, voxel_size);
    result_sg = result->size();
  });

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  nanoPCL:     " << std::setw(8) << time_npcl << " ms  ("
            << result_npcl << " points)\n";
  std::cout << "  small_gicp:  " << std::setw(8) << time_sg << " ms  ("
            << result_sg << " points)\n";
  std::cout << "  Speedup:     " << std::setw(8) << (time_sg / time_npcl)
            << "x\n";
}

void benchmarkVoxelGridModes(size_t n, float voxel_size) {
  std::cout << "\n=== nanoPCL VoxelMode Comparison ===" << std::endl;
  std::cout << "Points: " << n << ", Voxel size: " << voxel_size << "m\n"
            << std::endl;

  auto cloud = generateNanoPCL(n, 42);

  auto runMode = [&](nanopcl::filters::VoxelMode mode, const char* name) {
    size_t result_size = 0;
    double time = benchmark([&]() {
      auto result = nanopcl::filters::voxelGrid(cloud, voxel_size, mode);
      result_size = result.size();
    });
    std::cout << "  " << std::setw(10) << name << ": " << std::setw(8) << time
              << " ms  (" << result_size << " points)\n";
    return time;
  };

  runMode(nanopcl::filters::VoxelMode::CENTROID, "CENTROID");
  runMode(nanopcl::filters::VoxelMode::NEAREST, "NEAREST");
  runMode(nanopcl::filters::VoxelMode::ANY, "ANY");
  runMode(nanopcl::filters::VoxelMode::CENTER, "CENTER");
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
  size_t n = 1000000; // 1M points default
  float voxel_size = 0.5f;

  if (argc > 1)
    n = std::stoull(argv[1]);
  if (argc > 2)
    voxel_size = std::stof(argv[2]);

  std::cout << "========================================" << std::endl;
  std::cout << "  nanoPCL vs small_gicp Benchmark" << std::endl;
  std::cout << "========================================" << std::endl;

  // Warm up allocators
  {
    auto _ = generateNanoPCL(1000);
    auto __ = generateSmallGicp(1000);
  }

  // Run benchmarks
  benchmarkVoxelGrid(n, voxel_size);
  benchmarkVoxelGridModes(n, voxel_size);

  // Different sizes
  std::cout << "\n=== Scaling Test ===" << std::endl;
  for (size_t pts : {100000, 500000, 1000000, 2000000}) {
    auto cloud_npcl = generateNanoPCL(pts, 42);
    auto cloud_sg = generateSmallGicp(pts, 42);

    double t_npcl = benchmark([&]() {
      nanopcl::filters::voxelGrid(cloud_npcl, voxel_size,
                               nanopcl::filters::VoxelMode::CENTROID);
    });
    double t_sg = benchmark(
        [&]() { small_gicp::voxelgrid_sampling(cloud_sg, voxel_size); });

    std::cout << std::setw(8) << pts << " pts: nanopcl=" << std::setw(7) << t_npcl
              << "ms, sg=" << std::setw(7) << t_sg
              << "ms, ratio=" << (t_sg / t_npcl) << "x\n";
  }

  return 0;
}
