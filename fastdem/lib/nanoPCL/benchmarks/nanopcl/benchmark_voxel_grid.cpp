// nanoPCL Benchmark: VoxelGrid Performance
//
// Purpose:
//   Compare copy vs move versions of voxelGrid filter
//
// Build:
//   g++ -O3 -march=native -std=c++17 benchmark_voxel_grid.cpp \
//       -o benchmark_voxel_grid -I../../include -I/usr/include/eigen3
//
// Run:
//   ./benchmark_voxel_grid

#include "benchmark_common.hpp"
#include <nanopcl/common.hpp>

using namespace nanopcl;

// =============================================================================
// Test Data Generation
// =============================================================================

PointCloud generateCloud(size_t num_points, int num_channels, uint32_t seed = 42) {
  benchmark::resetRng(seed);
  auto& rng = benchmark::getRng();
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> val_dist(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring_dist(0, 127);

  PointCloud cloud;
  cloud.reserve(num_points);

  if (num_channels >= 1)
    cloud.useIntensity();
  if (num_channels >= 2)
    cloud.useTime();
  if (num_channels >= 3)
    cloud.useRing();

  for (size_t i = 0; i < num_points; ++i) {
    float x = pos_dist(rng), y = pos_dist(rng), z = pos_dist(rng);
    if (num_channels >= 3)
      cloud.add(x, y, z, Intensity(val_dist(rng)), Time(val_dist(rng)), Ring(ring_dist(rng)));
    else if (num_channels >= 2)
      cloud.add(x, y, z, Intensity(val_dist(rng)), Time(val_dist(rng)));
    else if (num_channels >= 1)
      cloud.add(x, y, z, Intensity(val_dist(rng)));
    else
      cloud.add(x, y, z);
  }

  return cloud;
}

// =============================================================================
// Benchmark Scenarios
// =============================================================================

struct Scenario {
  size_t num_points;
  float voxel_size;
  std::string description;
};

void runScenario(const Scenario& scenario, int num_channels) {
  const size_t num_points = scenario.num_points;
  const float voxel_size = scenario.voxel_size;

  benchmark::printSection(scenario.description);

  std::cout << "Points: " << num_points << ", Voxel: " << voxel_size
            << "m, Channels: " << num_channels << "\n\n";

  // Warmup and get expected output size
  size_t output_size = 0;
  {
    PointCloud test = generateCloud(num_points, num_channels);
    PointCloud result = filters::voxelGrid(test, voxel_size);
    output_size = result.size();
  }

  double reduction = 100.0 * output_size / num_points;
  std::cout << "Reduction: " << num_points << " -> " << output_size
            << " (" << std::fixed << std::setprecision(1) << reduction << "%)\n\n";

  // Column header
  std::cout << std::left << std::setw(32) << "Method"
            << std::right << std::setw(14) << "Time (ms)"
            << std::setw(14) << "Throughput"
            << std::setw(10) << "Speedup"
            << "\n";
  std::cout << std::string(70, '-') << "\n";

  // 1. Copy version (baseline)
  auto stats_copy = benchmark::run([&]() {
    PointCloud cloud = generateCloud(num_points, num_channels);
    return filters::voxelGrid(cloud, voxel_size);
  });

  double throughput_copy = num_points / stats_copy.mean / 1000.0;
  std::cout << std::left << std::setw(32) << "voxelGrid(const&)"
            << std::right << std::fixed << std::setprecision(3)
            << std::setw(10) << stats_copy.mean << " ± "
            << std::setw(5) << stats_copy.ci_95()
            << std::setprecision(2) << std::setw(10) << throughput_copy << " Mp/s"
            << std::setw(10) << "1.00x"
            << "\n";

  // 2. Move version
  auto stats_move = benchmark::run([&]() {
    PointCloud cloud = generateCloud(num_points, num_channels);
    return filters::voxelGrid(std::move(cloud), voxel_size);
  });

  double throughput_move = num_points / stats_move.mean / 1000.0;
  double speedup = stats_copy.mean / stats_move.mean;
  std::cout << std::left << std::setw(32) << "voxelGrid(&&)"
            << std::right << std::fixed << std::setprecision(3)
            << std::setw(10) << stats_move.mean << " ± "
            << std::setw(5) << stats_move.ci_95()
            << std::setprecision(2) << std::setw(10) << throughput_move << " Mp/s"
            << std::setw(9) << speedup << "x"
            << "\n";

  // 3. Reference: allocation + data generation overhead
  auto stats_gen = benchmark::run([&]() {
    return generateCloud(num_points, num_channels);
  });

  std::cout << std::left << std::setw(32) << "[REF] data generation"
            << std::right << std::fixed << std::setprecision(3)
            << std::setw(10) << stats_gen.mean << " ± "
            << std::setw(5) << stats_gen.ci_95()
            << std::setw(10) << "-"
            << std::setw(10) << "-"
            << "\n";

  // Compute actual voxelGrid processing time (excluding data gen)
  double voxel_only_copy = stats_copy.mean - stats_gen.mean;
  double voxel_only_move = stats_move.mean - stats_gen.mean;
  std::cout << "\nNet voxelGrid time (excluding data gen):\n";
  std::cout << "  const&: " << std::fixed << std::setprecision(3) << voxel_only_copy << " ms\n";
  std::cout << "  &&:     " << voxel_only_move << " ms\n";

  // Summary for this scenario
  std::cout << "\nStatistics: n=" << stats_copy.n;
  if (stats_copy.n_outliers > 0) {
    std::cout << " (" << stats_copy.n_outliers << " outliers removed)";
  }
  std::cout << "\n";
}

void verifyCorrectness(float voxel_size) {
  std::cout << "\nCorrectness verification: ";

  PointCloud test1 = generateCloud(1000, 3, 999);
  PointCloud test2 = generateCloud(1000, 3, 999);

  PointCloud result1 = filters::voxelGrid(test1, voxel_size);
  PointCloud result2 = filters::voxelGrid(std::move(test2), voxel_size);

  if (result1.size() != result2.size()) {
    std::cout << "FAILED (size: " << result1.size() << " vs " << result2.size() << ")\n";
    return;
  }

  // Sort and compare
  auto getSortedPoints = [](PointCloud& c) {
    std::vector<std::tuple<float, float, float>> pts;
    pts.reserve(c.size());
    for (size_t i = 0; i < c.size(); ++i) {
      pts.emplace_back(c[i].x(), c[i].y(), c[i].z());
    }
    std::sort(pts.begin(), pts.end());
    return pts;
  };

  auto pts1 = getSortedPoints(result1);
  auto pts2 = getSortedPoints(result2);

  float max_diff = 0;
  for (size_t i = 0; i < pts1.size(); ++i) {
    float dx = std::abs(std::get<0>(pts1[i]) - std::get<0>(pts2[i]));
    float dy = std::abs(std::get<1>(pts1[i]) - std::get<1>(pts2[i]));
    float dz = std::abs(std::get<2>(pts1[i]) - std::get<2>(pts2[i]));
    max_diff = std::max({max_diff, dx, dy, dz});
  }

  if (max_diff < 1e-4f) {
    std::cout << "PASSED (max diff: " << max_diff << ")\n";
  } else {
    std::cout << "FAILED (max diff: " << max_diff << ")\n";
  }
}

// =============================================================================
// Main
// =============================================================================

int main() {
  benchmark::printHeader("VoxelGrid Benchmark: Copy vs Move Semantics");

  // Platform info
  benchmark::PlatformInfo::capture().print();
  std::cout << "\n";

  // Benchmark configuration
  std::cout << "Configuration:\n";
  std::cout << "  Iterations: " << benchmark::IterationPolicy::MEDIUM << "\n";
  std::cout << "  Warmup:     " << benchmark::IterationPolicy::WARMUP << "\n";
  std::cout << "  Outliers:   IQR method (auto-removed)\n";

  // Test scenarios
  std::vector<Scenario> scenarios = {
      {100000, 0.5f, "100K points, 0.5m voxel (high reduction)"},
      {100000, 0.1f, "100K points, 0.1m voxel (low reduction)"},
      {500000, 0.5f, "500K points, 0.5m voxel"},
      {500000, 0.1f, "500K points, 0.1m voxel"},
  };

  for (const auto& scenario : scenarios) {
    runScenario(scenario, 3); // 3 channels: intensity, time, ring
  }

  verifyCorrectness(0.5f);

  // Conclusion
  benchmark::printFooter(
      "Move semantics benefit depends on reduction ratio.\n"
      "           High reduction (large voxels): Move saves allocation.\n"
      "           Low reduction (small voxels): Minimal benefit.");

  return 0;
}
