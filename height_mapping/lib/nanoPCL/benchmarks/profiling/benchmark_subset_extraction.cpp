// Benchmark: Subset Extraction (operator[](indices))
// Compare: Interleaved gather vs Loop Fission
//
// Tests if Loop Fission (separate loop per channel) is faster than
// the current interleaved approach.

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

using namespace npcl;
using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Test Data Generation
// =============================================================================

PointCloud createTestCloud(size_t n) {
  PointCloud cloud("test_frame");
  cloud.reserve(n);
  cloud.enableIntensity();
  cloud.enableRing();
  cloud.enableTime();

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring(0, 63);

  for (size_t i = 0; i < n; ++i) {
    cloud.add(Point(pos(gen), pos(gen), pos(gen)), Intensity(intensity(gen)),
              Ring(ring(gen)), Time(static_cast<float>(i) * 0.0001f));
  }
  return cloud;
}

// Sequential indices: [start, start+1, ..., start+count-1]
std::vector<size_t> sequentialIndices(size_t start, size_t count) {
  std::vector<size_t> indices(count);
  std::iota(indices.begin(), indices.end(), start);
  return indices;
}

// Random indices (sorted for fair comparison)
std::vector<size_t> randomIndices(size_t cloud_size, size_t count,
                                  unsigned seed) {
  std::mt19937 gen(seed);
  std::uniform_int_distribution<size_t> dist(0, cloud_size - 1);
  std::vector<size_t> indices;
  indices.reserve(count);

  // Use set-like approach to avoid duplicates
  std::vector<bool> used(cloud_size, false);
  while (indices.size() < count) {
    size_t idx = dist(gen);
    if (!used[idx]) {
      used[idx] = true;
      indices.push_back(idx);
    }
  }
  std::sort(indices.begin(), indices.end());
  return indices;
}

// Sparse indices: every k-th element
std::vector<size_t> sparseIndices(size_t cloud_size, size_t stride) {
  std::vector<size_t> indices;
  indices.reserve(cloud_size / stride + 1);
  for (size_t i = 0; i < cloud_size; i += stride) {
    indices.push_back(i);
  }
  return indices;
}

// =============================================================================
// Implementation A: Current (Interleaved + push_back)
// =============================================================================

PointCloud subsetInterleaved(const PointCloud& cloud,
                             const std::vector<size_t>& indices) {
  PointCloud result(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  result.reserve(indices.size());

  if (cloud.hasIntensity()) result.enableIntensity();
  if (cloud.hasTime()) result.enableTime();
  if (cloud.hasRing()) result.enableRing();

  for (size_t idx : indices) {
    result.xyz().push_back(cloud.xyz()[idx]);
    if (cloud.hasIntensity())
      result.intensity().push_back(cloud.intensity()[idx]);
    if (cloud.hasTime()) result.time().push_back(cloud.time()[idx]);
    if (cloud.hasRing()) result.ring().push_back(cloud.ring()[idx]);
  }

  return result;
}

// =============================================================================
// Implementation B: Loop Fission + resize
// =============================================================================

PointCloud subsetLoopFission(const PointCloud& cloud,
                             const std::vector<size_t>& indices) {
  PointCloud result(cloud.frameId());
  result.setTimestamp(cloud.timestamp());

  const size_t n = indices.size();

  // 1. Pre-allocate all channels
  result.resize(n);
  if (cloud.hasIntensity()) result.enableIntensity();
  if (cloud.hasTime()) result.enableTime();
  if (cloud.hasRing()) result.enableRing();

  // 2. Gather XYZ (separate loop)
  {
    const Point* src = cloud.xyz().data();
    Point* dst = result.xyz().data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  // 3. Gather Intensity (separate loop)
  if (cloud.hasIntensity()) {
    const float* src = cloud.intensity().data();
    float* dst = result.intensity().data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  // 4. Gather Time (separate loop)
  if (cloud.hasTime()) {
    const float* src = cloud.time().data();
    float* dst = result.time().data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  // 5. Gather Ring (separate loop)
  if (cloud.hasRing()) {
    const uint16_t* src = cloud.ring().data();
    uint16_t* dst = result.ring().data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  return result;
}

// =============================================================================
// Benchmarking
// =============================================================================

struct BenchResult {
  double interleaved_ms;
  double fission_ms;
};

BenchResult benchmark(const PointCloud& cloud,
                      const std::vector<size_t>& indices, int iterations) {
  BenchResult result{0, 0};

  // Warmup
  volatile size_t dummy = 0;
  for (int i = 0; i < 2; ++i) {
    auto r1 = subsetInterleaved(cloud, indices);
    auto r2 = subsetLoopFission(cloud, indices);
    dummy += r1.size() + r2.size();
  }

  // Benchmark Interleaved
  {
    auto start = Clock::now();
    for (int i = 0; i < iterations; ++i) {
      auto r = subsetInterleaved(cloud, indices);
      dummy += r.size();
    }
    auto end = Clock::now();
    result.interleaved_ms =
        std::chrono::duration<double, std::milli>(end - start).count() /
        iterations;
  }

  // Benchmark Loop Fission
  {
    auto start = Clock::now();
    for (int i = 0; i < iterations; ++i) {
      auto r = subsetLoopFission(cloud, indices);
      dummy += r.size();
    }
    auto end = Clock::now();
    result.fission_ms =
        std::chrono::duration<double, std::milli>(end - start).count() /
        iterations;
  }

  return result;
}

// =============================================================================
// Implementation C: Actual operator[] (after optimization)
// =============================================================================

double benchActual(const PointCloud& cloud, const std::vector<size_t>& indices,
                   int iterations) {
  volatile size_t dummy = 0;
  // Warmup
  for (int i = 0; i < 2; ++i) {
    auto r = cloud[indices];
    dummy += r.size();
  }

  auto start = Clock::now();
  for (int i = 0; i < iterations; ++i) {
    auto r = cloud[indices];
    dummy += r.size();
  }
  auto end = Clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count() /
         iterations;
}

void runTest(const std::string& name, const PointCloud& cloud,
             const std::vector<size_t>& indices, int iterations = 10) {
  auto result = benchmark(cloud, indices, iterations);
  double actual_ms = benchActual(cloud, indices, iterations);

  printf("  %-15s | %6zu pts | Old: %7.3f ms | New: %7.3f ms | Actual: %7.3f ms\n",
         name.c_str(), indices.size(), result.interleaved_ms, result.fission_ms,
         actual_ms);
}

void runSuite(size_t cloud_size) {
  std::cout << "\n========================================\n";
  std::cout << "Cloud Size: " << cloud_size << " points\n";
  std::cout << "Channels: XYZ + Intensity + Ring + Time\n";
  std::cout << "========================================\n";

  auto cloud = createTestCloud(cloud_size);

  // Test different extraction ratios
  size_t sizes[] = {100, 1000, 10000, cloud_size / 2};

  for (size_t extract_size : sizes) {
    if (extract_size > cloud_size) continue;

    std::cout << "\n--- Extract " << extract_size << " points ---\n";

    // Sequential (best case for prefetcher)
    auto seq = sequentialIndices(0, extract_size);
    runTest("Sequential", cloud, seq);

    // Random (worst case)
    auto rnd = randomIndices(cloud_size, extract_size, 12345);
    runTest("Random (sorted)", cloud, rnd);

    // Sparse (every 10th)
    if (cloud_size >= extract_size * 10) {
      auto sparse = sparseIndices(cloud_size, cloud_size / extract_size);
      runTest("Sparse (strided)", cloud, sparse);
    }
  }
}

int main() {
  std::cout << "==============================================\n";
  std::cout << "Subset Extraction Benchmark\n";
  std::cout << "Interleaved (current) vs Loop Fission\n";
  std::cout << "==============================================\n";

  runSuite(10000);
  runSuite(100000);
  runSuite(500000);

  std::cout << "\n==============================================\n";
  std::cout << "Done.\n";

  return 0;
}
