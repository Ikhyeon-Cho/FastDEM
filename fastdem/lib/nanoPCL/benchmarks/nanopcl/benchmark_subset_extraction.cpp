// Benchmark: Subset Extraction (extract by indices)
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

using namespace nanopcl;
using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Test Data Generation
// =============================================================================

PointCloud createTestCloud(size_t n) {
  PointCloud cloud;
  cloud.reserve(n);
  cloud.useIntensity();
  cloud.useRing();
  cloud.useTime();

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring(0, 63);

  for (size_t i = 0; i < n; ++i) {
    cloud.add(pos(gen), pos(gen), pos(gen),
              Intensity(intensity(gen)),
              Ring(ring(gen)),
              Time(static_cast<float>(i) * 0.0001f));
  }
  return cloud;
}

std::vector<size_t> sequentialIndices(size_t start, size_t count) {
  std::vector<size_t> indices(count);
  std::iota(indices.begin(), indices.end(), start);
  return indices;
}

std::vector<size_t> randomIndices(size_t cloud_size, size_t count, unsigned seed) {
  std::mt19937 gen(seed);
  std::uniform_int_distribution<size_t> dist(0, cloud_size - 1);
  std::vector<size_t> indices;
  indices.reserve(count);

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

std::vector<size_t> sparseIndices(size_t cloud_size, size_t stride) {
  std::vector<size_t> indices;
  indices.reserve(cloud_size / stride + 1);
  for (size_t i = 0; i < cloud_size; i += stride) {
    indices.push_back(i);
  }
  return indices;
}

// =============================================================================
// Implementation A: Interleaved (per-point copy)
// =============================================================================

PointCloud subsetInterleaved(const PointCloud& cloud,
                             const std::vector<size_t>& indices) {
  PointCloud result;
  result.setFrameId(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  result.reserve(indices.size());

  if (cloud.hasIntensity())
    result.useIntensity();
  if (cloud.hasTime())
    result.useTime();
  if (cloud.hasRing())
    result.useRing();

  for (size_t idx : indices) {
    // Copy point
    const auto& src = cloud[idx];
    result.add(src.x(), src.y(), src.z());
    size_t dst = result.size() - 1;

    // Copy channels
    if (cloud.hasIntensity())
      result.intensity(dst) = cloud.intensity(idx);
    if (cloud.hasTime())
      result.time(dst) = cloud.time(idx);
    if (cloud.hasRing())
      result.ring(dst) = cloud.ring(idx);
  }

  return result;
}

// =============================================================================
// Implementation B: Loop Fission (separate loop per channel)
// =============================================================================

PointCloud subsetLoopFission(const PointCloud& cloud,
                             const std::vector<size_t>& indices) {
  PointCloud result;
  result.setFrameId(cloud.frameId());
  result.setTimestamp(cloud.timestamp());

  const size_t n = indices.size();

  // Pre-allocate
  result.resize(n);
  if (cloud.hasIntensity())
    result.useIntensity();
  if (cloud.hasTime())
    result.useTime();
  if (cloud.hasRing())
    result.useRing();

  // Gather points (separate loop)
  {
    const auto& src = cloud.points();
    auto& dst = result.points();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  // Gather Intensity
  if (cloud.hasIntensity()) {
    const auto& src = cloud.intensities();
    auto& dst = result.intensities();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  // Gather Time
  if (cloud.hasTime()) {
    const auto& src = cloud.times();
    auto& dst = result.times();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  // Gather Ring
  if (cloud.hasRing()) {
    const auto& src = cloud.rings();
    auto& dst = result.rings();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  return result;
}

// =============================================================================
// Implementation C: Using cloud.extract() (actual API)
// =============================================================================

PointCloud subsetExtract(const PointCloud& cloud,
                         const std::vector<size_t>& indices) {
  return cloud.extract(indices);
}

// =============================================================================
// Benchmarking
// =============================================================================

struct BenchResult {
  double interleaved_ms;
  double fission_ms;
  double extract_ms;
};

BenchResult benchmark(const PointCloud& cloud,
                      const std::vector<size_t>& indices,
                      int iterations) {
  BenchResult result{0, 0, 0};

  // Warmup
  volatile size_t dummy = 0;
  for (int i = 0; i < 2; ++i) {
    auto r1 = subsetInterleaved(cloud, indices);
    auto r2 = subsetLoopFission(cloud, indices);
    auto r3 = subsetExtract(cloud, indices);
    dummy += r1.size() + r2.size() + r3.size();
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

  // Benchmark Extract (actual API)
  {
    auto start = Clock::now();
    for (int i = 0; i < iterations; ++i) {
      auto r = subsetExtract(cloud, indices);
      dummy += r.size();
    }
    auto end = Clock::now();
    result.extract_ms =
        std::chrono::duration<double, std::milli>(end - start).count() /
        iterations;
  }

  return result;
}

void runTest(const std::string& name, const PointCloud& cloud,
             const std::vector<size_t>& indices, int iterations = 10) {
  auto result = benchmark(cloud, indices, iterations);

  printf("  %-15s | %6zu pts | Interleaved: %7.3f ms | Fission: %7.3f ms | "
         "Extract: %7.3f ms\n",
         name.c_str(), indices.size(), result.interleaved_ms, result.fission_ms,
         result.extract_ms);
}

void runSuite(size_t cloud_size) {
  std::cout << "\n========================================\n";
  std::cout << "Cloud Size: " << cloud_size << " points\n";
  std::cout << "Channels: XYZ + Intensity + Ring + Time\n";
  std::cout << "========================================\n";

  auto cloud = createTestCloud(cloud_size);

  size_t sizes[] = {100, 1000, 10000, cloud_size / 2};

  for (size_t extract_size : sizes) {
    if (extract_size > cloud_size)
      continue;

    std::cout << "\n--- Extract " << extract_size << " points ---\n";

    // Sequential (best case for prefetcher)
    auto seq = sequentialIndices(0, extract_size);
    runTest("Sequential", cloud, seq);

    // Random (worst case)
    auto rnd = randomIndices(cloud_size, extract_size, 12345);
    runTest("Random (sorted)", cloud, rnd);

    // Sparse (every k-th)
    if (cloud_size >= extract_size * 10) {
      auto sparse = sparseIndices(cloud_size, cloud_size / extract_size);
      runTest("Sparse (strided)", cloud, sparse);
    }
  }
}

int main() {
  std::cout << "==============================================\n";
  std::cout << "Subset Extraction Benchmark (Point4 API)\n";
  std::cout << "Interleaved vs Loop Fission vs extract()\n";
  std::cout << "==============================================\n";

  runSuite(10000);
  runSuite(100000);
  runSuite(500000);

  std::cout << "\n==============================================\n";
  std::cout << "Done.\n";

  return 0;
}
