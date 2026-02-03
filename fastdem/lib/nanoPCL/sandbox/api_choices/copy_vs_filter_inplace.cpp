// nanoPCL Benchmark: Copy Strategy Comparison
// Compares: "Full copy + in-place filter" vs "Copy only passing points"
//
// Question: Is PointCloud(cloud) + filterInPlace faster/slower than filterByPointCopy?

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"

using namespace nanopcl;

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
               : "+r"(value)
               :
               : "memory");
#else
  volatile auto unused = value;
  (void)unused;
#endif
}

PointCloud generateCloud(size_t num_points, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> pos(-50.0f, 50.0f);
  std::uniform_real_distribution<float> val(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring(0, 127);

  PointCloud cloud("benchmark");
  cloud.reserve(num_points);
  cloud.enableIntensity();
  cloud.enableTime();
  cloud.enableRing();

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(Point(pos(gen), pos(gen), pos(gen)));
    cloud.intensity().back() = val(gen);
    cloud.time().back() = val(gen);
    cloud.ring().back() = ring(gen);
  }
  return cloud;
}

// Strategy A: Full copy + in-place filter
template <typename Pred>
PointCloud strategyFullCopyThenFilter(const PointCloud& cloud, Pred pred) {
  PointCloud copy(cloud); // Full copy

  // In-place filter
  Point* pts = copy.xyz().data();
  float* intensity = copy.hasIntensity() ? copy.intensity().data() : nullptr;
  float* time = copy.hasTime() ? copy.time().data() : nullptr;
  uint16_t* ring = copy.hasRing() ? copy.ring().data() : nullptr;

  size_t write = 0;
  const size_t n = copy.size();
  for (size_t read = 0; read < n; ++read) {
    if (pred(pts[read])) {
      if (write != read) {
        pts[write] = pts[read];
        if (intensity)
          intensity[write] = intensity[read];
        if (time)
          time[write] = time[read];
        if (ring)
          ring[write] = ring[read];
      }
      ++write;
    }
  }
  copy.resize(write);
  return copy;
}

// Strategy B: Copy only passing points
template <typename Pred>
PointCloud strategyCopyOnlyPassing(const PointCloud& cloud, Pred pred) {
  PointCloud result(cloud.frameId());
  result.reserve(cloud.size());

  if (cloud.hasIntensity())
    result.enableIntensity();
  if (cloud.hasTime())
    result.enableTime();
  if (cloud.hasRing())
    result.enableRing();

  const Point* pts = cloud.xyz().data();
  const float* intensity = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  const float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  const uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;

  const size_t n = cloud.size();
  for (size_t i = 0; i < n; ++i) {
    if (pred(pts[i])) {
      result.xyz().push_back(pts[i]);
      if (intensity)
        result.intensity().push_back(intensity[i]);
      if (time)
        result.time().push_back(time[i]);
      if (ring)
        result.ring().push_back(ring[i]);
    }
  }
  return result;
}

void runBenchmark(size_t num_points, float pass_rate, int iterations) {
  // Z-axis filter predicate
  float z_threshold = -50.0f + 100.0f * pass_rate; // Adjust for pass rate
  auto pred = [z_threshold](const Point& pt) { return pt.z() < z_threshold; };

  std::cout << "\n=== Points: " << num_points
            << ", Pass rate: ~" << int(pass_rate * 100) << "% ===\n";

  // Strategy A
  double total_a = 0;
  size_t size_a = 0;
  for (int i = 0; i < iterations; ++i) {
    PointCloud cloud = generateCloud(num_points, 42 + i);
    Timer t;
    auto result = strategyFullCopyThenFilter(cloud, pred);
    total_a += t.elapsed_ms();
    size_a = result.size();
    doNotOptimize(size_a);
  }

  // Strategy B
  double total_b = 0;
  size_t size_b = 0;
  for (int i = 0; i < iterations; ++i) {
    PointCloud cloud = generateCloud(num_points, 42 + i);
    Timer t;
    auto result = strategyCopyOnlyPassing(cloud, pred);
    total_b += t.elapsed_ms();
    size_b = result.size();
    doNotOptimize(size_b);
  }

  double avg_a = total_a / iterations;
  double avg_b = total_b / iterations;
  double ratio = avg_a / avg_b;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  [A] Full copy + filter:    " << avg_a << " ms\n";
  std::cout << "  [B] Copy only passing:     " << avg_b << " ms\n";
  std::cout << "  Ratio (A/B):               " << std::setprecision(2) << ratio << "x\n";
  std::cout << "  Result size:               " << size_a << " / " << num_points
            << " (" << (100.0 * size_a / num_points) << "%)\n";

  if (ratio > 1.1) {
    std::cout << "  --> B is faster (copy only passing wins)\n";
  } else if (ratio < 0.9) {
    std::cout << "  --> A is faster (full copy + filter wins)\n";
  } else {
    std::cout << "  --> Similar performance\n";
  }
}

int main() {
  std::cout << "=======================================================\n";
  std::cout << "Copy Strategy Benchmark\n";
  std::cout << "  [A] Full copy + in-place filter\n";
  std::cout << "  [B] Copy only points that pass predicate\n";
  std::cout << "=======================================================\n";

  const int ITER = 20;

  // Low pass rate (10%) - B should win (less copying)
  runBenchmark(500000, 0.1f, ITER);

  // Medium pass rate (50%)
  runBenchmark(500000, 0.5f, ITER);

  // High pass rate (90%) - A might win (sequential memory access)
  runBenchmark(500000, 0.9f, ITER);

  // Very high pass rate (99%)
  runBenchmark(500000, 0.99f, ITER);

  std::cout << "\n=======================================================\n";
  std::cout << "Conclusion:\n";
  std::cout << "  If B is consistently faster -> use filterByPointCopy\n";
  std::cout << "  If A is faster at high pass rates -> consider hybrid\n";
  std::cout << "=======================================================\n";

  return 0;
}
