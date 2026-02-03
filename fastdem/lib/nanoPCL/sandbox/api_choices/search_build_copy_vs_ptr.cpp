// nanoPCL Benchmark: VoxelHash build - Copy vs Pointer
// Measures the overhead of cloud copying during VoxelHash::build()
//
// Question: Is the cloud copy in build(const PointCloud&) significant
// compared to the actual hash table construction?

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include <nanopcl/core/point_cloud.hpp>
#include <nanopcl/search/voxel_hash.hpp>

using namespace nanopcl;

// =============================================================================
// Timer
// =============================================================================
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()) {}

  double elapsed_ms() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }
};

template <typename T>
void doNotOptimize(T& value) {
#if defined(__GNUC__) || defined(__clang__)
  asm volatile(""
               :
               : "g"(&value)
               : "memory");
#else
  volatile auto unused = &value;
  (void)unused;
#endif
}

// =============================================================================
// Stats
// =============================================================================
struct Stats {
  double mean, stddev, min, max;
};

Stats computeStats(const std::vector<double>& data) {
  if (data.empty())
    return {0, 0, 0, 0};
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();
  double sq_sum = 0;
  for (double v : data)
    sq_sum += (v - mean) * (v - mean);
  double stddev = std::sqrt(sq_sum / data.size());
  auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
  return {mean, stddev, *min_it, *max_it};
}

// =============================================================================
// Cloud Generator
// =============================================================================
PointCloud generateCloud(size_t num_points, float range) {
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-range, range);

  PointCloud cloud("test");
  cloud.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(Point(dist(gen), dist(gen), dist(gen)));
  }
  return cloud;
}

// =============================================================================
// Benchmark
// =============================================================================
void runBenchmark(size_t cloud_size, float resolution, int rounds) {
  std::cout << "\n"
            << std::string(70, '=') << "\n";
  std::cout << "Cloud: " << cloud_size << " pts, Resolution: " << resolution << "m\n";
  std::cout << "Memory: " << (cloud_size * sizeof(Point) / 1024.0 / 1024.0)
            << " MB (XYZ only)\n";
  std::cout << std::string(70, '=') << "\n";

  // Generate cloud
  PointCloud cloud = generateCloud(cloud_size, 50.0f);
  auto cloud_ptr = std::make_shared<PointCloud>(cloud);

  std::vector<double> times_copy, times_ptr, times_pure_copy;

  for (int r = 0; r < rounds; ++r) {
    // Method 1: build(const PointCloud&) - current, causes copy
    {
      search::VoxelHash searcher(resolution);
      Timer t;
      searcher.build(cloud);
      times_copy.push_back(t.elapsed_ms());
      doNotOptimize(searcher);
    }

    // Method 2: build(shared_ptr) - no copy
    {
      search::VoxelHash searcher(resolution);
      Timer t;
      searcher.build(cloud_ptr);
      times_ptr.push_back(t.elapsed_ms());
      doNotOptimize(searcher);
    }

    // Method 3: Pure copy cost (baseline)
    {
      Timer t;
      auto copied = std::make_shared<PointCloud>(cloud);
      times_pure_copy.push_back(t.elapsed_ms());
      doNotOptimize(copied);
    }
  }

  auto stats_copy = computeStats(times_copy);
  auto stats_ptr = computeStats(times_ptr);
  auto stats_pure = computeStats(times_pure_copy);

  std::cout << std::left << std::setw(35) << "Method"
            << std::right << std::setw(12) << "Mean(ms)"
            << std::setw(12) << "Stddev"
            << "\n";
  std::cout << std::string(70, '-') << "\n";

  std::cout << std::left << std::setw(35) << "build(cloud) [copy + hash]"
            << std::right << std::fixed << std::setprecision(3)
            << std::setw(12) << stats_copy.mean
            << std::setw(12) << stats_copy.stddev << "\n";

  std::cout << std::left << std::setw(35) << "build(shared_ptr) [hash only]"
            << std::right << std::setw(12) << stats_ptr.mean
            << std::setw(12) << stats_ptr.stddev << "\n";

  std::cout << std::left << std::setw(35) << "Pure copy (make_shared)"
            << std::right << std::setw(12) << stats_pure.mean
            << std::setw(12) << stats_pure.stddev << "\n";

  std::cout << std::string(70, '-') << "\n";

  double copy_overhead = stats_copy.mean - stats_ptr.mean;
  double copy_percent = (copy_overhead / stats_copy.mean) * 100;

  std::cout << "Copy overhead: " << std::fixed << std::setprecision(3)
            << copy_overhead << " ms (" << std::setprecision(1)
            << copy_percent << "% of total build time)\n";
}

// =============================================================================
// Main
// =============================================================================
int main() {
  std::cout << "nanoPCL Benchmark: VoxelHash build - Copy vs Pointer\n";
  std::cout << std::string(70, '=') << "\n";
  std::cout << R"(
Question: How much overhead does cloud copying add to VoxelHash::build()?

Methods:
  1. build(cloud)      - Current API, copies cloud internally
  2. build(shared_ptr) - Zero-copy, user provides shared_ptr
  3. Pure copy         - Just the copy cost (baseline)

)"
            << "\n";

  const int ROUNDS = 30;

  // Various cloud sizes
  runBenchmark(50000, 0.3f, ROUNDS);   // 50K - small
  runBenchmark(100000, 0.3f, ROUNDS);  // 100K - typical
  runBenchmark(300000, 0.5f, ROUNDS);  // 300K - large
  runBenchmark(500000, 0.5f, ROUNDS);  // 500K - very large
  runBenchmark(1000000, 1.0f, ROUNDS); // 1M - stress test

  std::cout << "\n"
            << std::string(70, '=') << "\n";
  std::cout << "CONCLUSION\n";
  std::cout << std::string(70, '=') << "\n";
  std::cout << R"(
If copy overhead is < 20% of build time:
  -> Keep current API for simplicity

If copy overhead is > 50% of build time:
  -> Consider adding raw pointer option for internal use

)"
            << "\n";

  return 0;
}
