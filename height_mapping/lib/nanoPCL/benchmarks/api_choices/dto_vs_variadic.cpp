// Benchmark: DTO vs Variadic vs Direct channel access
#include <chrono>
#include <iostream>
#include <nanopcl/core.hpp>

using namespace npcl;

constexpr size_t N = 1000000;  // 1M points
constexpr int RUNS = 5;

template <typename Func>
double benchmark(const char* name, Func&& func) {
  double total_ms = 0;
  for (int run = 0; run < RUNS; ++run) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();
    total_ms +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count() /
        1000.0;
  }
  double avg_ms = total_ms / RUNS;
  std::cout << "  " << name << ": " << avg_ms << " ms\n";
  return avg_ms;
}

int main() {
  std::cout << "=== DTO vs Variadic vs Direct Benchmark ===\n";
  std::cout << "Points: " << N << ", Runs: " << RUNS << "\n\n";

  // =========================================================================
  // Test 1: PointXYZI (xyz + intensity)
  // =========================================================================
  std::cout << "[Test 1] PointXYZI (xyz + intensity)\n";

  double dto_time = benchmark("DTO        ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    for (size_t i = 0; i < N; ++i) {
      cloud.add(PointXYZI(i * 0.1f, i * 0.2f, i * 0.3f, i * 0.001f));
    }
  });

  double variadic_time = benchmark("Variadic   ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    for (size_t i = 0; i < N; ++i) {
      cloud.add(Point(i * 0.1f, i * 0.2f, i * 0.3f), Intensity(i * 0.001f));
    }
  });

  double direct_time = benchmark("Direct     ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    auto& xyz = cloud.xyz();
    auto& intensity = cloud.intensity();
    for (size_t i = 0; i < N; ++i) {
      xyz.push_back(Point(i * 0.1f, i * 0.2f, i * 0.3f));
      intensity.push_back(i * 0.001f);
    }
  });

  std::cout << "  Ratio (DTO/Direct): " << dto_time / direct_time << "x\n\n";

  // =========================================================================
  // Test 2: PointXYZIR (xyz + intensity + ring)
  // =========================================================================
  std::cout << "[Test 2] PointXYZIR (xyz + intensity + ring)\n";

  dto_time = benchmark("DTO        ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    cloud.enableRing();
    for (size_t i = 0; i < N; ++i) {
      cloud.add(PointXYZIR(i * 0.1f, i * 0.2f, i * 0.3f, i * 0.001f, i % 64));
    }
  });

  variadic_time = benchmark("Variadic   ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    cloud.enableRing();
    for (size_t i = 0; i < N; ++i) {
      cloud.add(Point(i * 0.1f, i * 0.2f, i * 0.3f), Intensity(i * 0.001f),
                Ring(i % 64));
    }
  });

  direct_time = benchmark("Direct     ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    cloud.enableRing();
    auto& xyz = cloud.xyz();
    auto& intensity = cloud.intensity();
    auto& ring = cloud.ring();
    for (size_t i = 0; i < N; ++i) {
      xyz.push_back(Point(i * 0.1f, i * 0.2f, i * 0.3f));
      intensity.push_back(i * 0.001f);
      ring.push_back(i % 64);
    }
  });

  std::cout << "  Ratio (DTO/Direct): " << dto_time / direct_time << "x\n\n";

  // =========================================================================
  // Test 3: PointXYZIRT (xyz + intensity + ring + time)
  // =========================================================================
  std::cout << "[Test 3] PointXYZIRT (xyz + intensity + ring + time)\n";

  dto_time = benchmark("DTO        ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    cloud.enableRing();
    cloud.enableTime();
    for (size_t i = 0; i < N; ++i) {
      cloud.add(
          PointXYZIRT(i * 0.1f, i * 0.2f, i * 0.3f, i * 0.001f, i % 64, i * 1e-6f));
    }
  });

  variadic_time = benchmark("Variadic   ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    cloud.enableRing();
    cloud.enableTime();
    for (size_t i = 0; i < N; ++i) {
      cloud.add(Point(i * 0.1f, i * 0.2f, i * 0.3f), Intensity(i * 0.001f),
                Ring(i % 64), Time(i * 1e-6f));
    }
  });

  direct_time = benchmark("Direct     ", []() {
    PointCloud cloud;
    cloud.reserve(N);
    cloud.enableIntensity();
    cloud.enableRing();
    cloud.enableTime();
    auto& xyz = cloud.xyz();
    auto& intensity = cloud.intensity();
    auto& ring = cloud.ring();
    auto& time = cloud.time();
    for (size_t i = 0; i < N; ++i) {
      xyz.push_back(Point(i * 0.1f, i * 0.2f, i * 0.3f));
      intensity.push_back(i * 0.001f);
      ring.push_back(i % 64);
      time.push_back(i * 1e-6f);
    }
  });

  std::cout << "  Ratio (DTO/Direct): " << dto_time / direct_time << "x\n\n";

  std::cout << "=== Summary ===\n";
  std::cout << "DTO and Variadic should be nearly identical (both delegate to same code)\n";
  std::cout << "Direct is fastest but less convenient\n";

  return 0;
}
