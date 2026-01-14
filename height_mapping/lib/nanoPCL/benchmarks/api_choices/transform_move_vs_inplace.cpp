// Benchmark: transformCloud with Move vs transformCloudInPlace
//
// Question: Is InPlace API necessary when we have Move semantics?
//
// Hypothesis:
//   - Move: creates new cloud, transforms into it, swaps pointers
//   - InPlace: transforms points directly in existing cloud
//   - Both should be O(N), but InPlace might have slight cache advantage

#include <chrono>
#include <cmath>
#include <iostream>
#include <nanopcl/common.hpp>
#include <vector>

using namespace npcl;

// Local InPlace implementation for benchmarking comparison
// (Not exposed in public API - Move semantics is preferred)
template <typename Scalar>
void transformCloudInPlace(PointCloud& cloud, const Transform_<Scalar>& tf) {
  if (!tf.isValid()) {
    throw std::runtime_error("Transform not initialized");
  }
  if (tf.childFrame() != cloud.frameId()) {
    throw std::runtime_error("Frame mismatch");
  }

  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  for (auto& point : cloud.xyz()) {
    point = R * point + t;
  }
  if (cloud.hasNormal()) {
    for (auto& n : cloud.normal()) {
      n = R * n;
    }
  }
  cloud.setFrameId(tf.parentFrame());
}

constexpr size_t N = 1000000;  // 1M points
constexpr int WARMUP = 2;
constexpr int RUNS = 10;

// Generate test cloud
PointCloud generateCloud(size_t count) {
  PointCloud cloud("source");
  cloud.enableIntensity();
  cloud.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    float x = std::sin(i * 0.001f) * 10.0f;
    float y = std::cos(i * 0.001f) * 10.0f;
    float z = (i % 100) * 0.1f;
    cloud.add(PointXYZI(x, y, z, i * 0.0001f));
  }
  return cloud;
}

template <typename Func>
double benchmark(const char* name, Func&& func) {
  // Warmup
  for (int i = 0; i < WARMUP; ++i) {
    func();
  }

  // Measure
  std::vector<double> times;
  for (int run = 0; run < RUNS; ++run) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();
    double ms =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count() /
        1000.0;
    times.push_back(ms);
  }

  // Calculate stats
  double sum = 0, min_t = times[0], max_t = times[0];
  for (double t : times) {
    sum += t;
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
  }
  double avg = sum / RUNS;

  std::cout << "  " << name << ": " << avg << " ms (min=" << min_t
            << ", max=" << max_t << ")\n";
  return avg;
}

int main() {
  std::cout << "=== Transform: Move vs InPlace Benchmark ===\n";
  std::cout << "Points: " << N << ", Warmup: " << WARMUP << ", Runs: " << RUNS
            << "\n\n";

  // Create transform (rotation + translation)
  Transformd T = Transformd::fromRPY("target", "source", 0.1, 0.2, 0.3,
                                     Eigen::Vector3d(1.0, 2.0, 3.0));

  // =========================================================================
  // Test 1: Copy (baseline - creates new cloud, preserves original)
  // =========================================================================
  std::cout << "[Test 1] Copy (baseline)\n";
  double copy_time = benchmark("Copy       ", [&]() {
    PointCloud cloud = generateCloud(N);
    PointCloud result = transformCloud(cloud, T);
    (void)result;
  });

  // =========================================================================
  // Test 2: Move semantics (cloud = transformCloud(std::move(cloud), T))
  // =========================================================================
  std::cout << "\n[Test 2] Move semantics\n";
  double move_time = benchmark("Move       ", [&]() {
    PointCloud cloud = generateCloud(N);
    cloud = transformCloud(std::move(cloud), T);
  });

  // =========================================================================
  // Test 3: InPlace (transformCloudInPlace modifies in-place)
  // =========================================================================
  std::cout << "\n[Test 3] InPlace\n";
  double inplace_time = benchmark("InPlace    ", [&]() {
    PointCloud cloud = generateCloud(N);
    transformCloudInPlace(cloud, T);
  });

  // =========================================================================
  // Test 4: Transform only (exclude cloud generation)
  // =========================================================================
  std::cout << "\n[Test 4] Transform only (pre-generated cloud)\n";

  PointCloud base_cloud = generateCloud(N);

  double move_only = benchmark("Move only  ", [&]() {
    PointCloud cloud = base_cloud;  // Copy
    cloud = transformCloud(std::move(cloud), T);
  });

  double inplace_only = benchmark("InPlace only", [&]() {
    PointCloud cloud = base_cloud;  // Copy
    transformCloudInPlace(cloud, T);
  });

  // =========================================================================
  // Summary
  // =========================================================================
  std::cout << "\n=== Summary ===\n";
  std::cout << "Move vs InPlace difference: "
            << std::abs(move_time - inplace_time) << " ms ("
            << (100.0 * std::abs(move_time - inplace_time) /
                std::max(move_time, inplace_time))
            << "%)\n";
  std::cout << "Transform-only difference:  "
            << std::abs(move_only - inplace_only) << " ms ("
            << (100.0 * std::abs(move_only - inplace_only) /
                std::max(move_only, inplace_only))
            << "%)\n";

  if (std::abs(move_only - inplace_only) / std::max(move_only, inplace_only) <
      0.1) {
    std::cout << "\nConclusion: Move and InPlace have similar performance.\n";
    std::cout << "            InPlace API is redundant - Move is sufficient.\n";
  } else if (inplace_only < move_only) {
    std::cout << "\nConclusion: InPlace is faster by "
              << (100.0 * (move_only - inplace_only) / move_only) << "%\n";
  } else {
    std::cout << "\nConclusion: Move is faster by "
              << (100.0 * (inplace_only - move_only) / inplace_only) << "%\n";
  }

  return 0;
}
