// Benchmark: Single erase (O(n*k)) vs Batch erase (O(n + k log k))

#include <chrono>
#include <iostream>
#include <random>
#include <set>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

using namespace nanopcl;
using Clock = std::chrono::high_resolution_clock;

std::vector<size_t> randomIndices(size_t cloud_size, size_t num_indices, unsigned seed) {
  std::mt19937 gen(seed);
  std::uniform_int_distribution<size_t> dist(0, cloud_size - 1);
  std::set<size_t> unique;
  while (unique.size() < num_indices)
    unique.insert(dist(gen));
  return std::vector<size_t>(unique.begin(), unique.end());
}

PointCloud createTestCloud(size_t n) {
  PointCloud cloud;
  cloud.reserve(n);
  cloud.enableIntensity();
  cloud.enableRing();
  cloud.enableTime();
  for (size_t i = 0; i < n; ++i) {
    float fi = static_cast<float>(i);
    cloud.add(Point(fi, fi * 2, fi * 3), Intensity(fi * 0.1f), Ring(i % 64), Time(fi * 0.001f));
  }
  return cloud;
}

double benchSingleErase(size_t cloud_size, const std::vector<size_t>& indices) {
  PointCloud cloud = createTestCloud(cloud_size);
  std::vector<size_t> sorted = indices;
  std::sort(sorted.rbegin(), sorted.rend()); // descending to avoid shift
  auto start = Clock::now();
  for (size_t idx : sorted)
    cloud.erase(idx);
  auto end = Clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double benchBatchErase(size_t cloud_size, const std::vector<size_t>& indices) {
  PointCloud cloud = createTestCloud(cloud_size);
  auto start = Clock::now();
  cloud.erase(indices);
  auto end = Clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void run(size_t n, size_t k, int iter = 5) {
  std::cout << "\n=== " << n << " pts, erase " << k << " ===\n";
  double s = 0, b = 0;
  for (int i = 0; i < iter; ++i) {
    auto idx = randomIndices(n, k, i * 42);
    s += benchSingleErase(n, idx);
    b += benchBatchErase(n, idx);
  }
  s /= iter;
  b /= iter;
  printf("  Single: %8.2f ms\n", s);
  printf("  Batch:  %8.2f ms\n", b);
  printf("  Speedup: %.1fx\n", s / b);
}

int main() {
  std::cout << "Batch Erase Benchmark\n";
  std::cout << "=====================\n";

  run(10000, 100);
  run(10000, 1000);
  run(100000, 1000);
  run(100000, 10000);
  run(500000, 5000);
  run(500000, 50000);

  return 0;
}
