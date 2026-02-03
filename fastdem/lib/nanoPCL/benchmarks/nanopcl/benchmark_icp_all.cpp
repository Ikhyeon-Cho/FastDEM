// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Benchmark: P2P ICP vs P2Plane ICP vs GICP vs VGICP

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <nanopcl/common.hpp>
#include <nanopcl/geometry/normal_estimation.hpp>
#include <nanopcl/registration.hpp>

using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Benchmark Result Structure
// =============================================================================

struct BenchResult {
  double total_ms;
  double per_iter_ms;
  size_t iterations;
  double fitness;
  double rmse;
  bool converged;
};

template <typename F>
BenchResult measure(F&& func, int runs = 5) {
  auto result = func();  // Warmup

  double total = 0;
  for (int i = 0; i < runs; ++i) {
    auto start = Clock::now();
    result = func();
    auto end = Clock::now();
    total += std::chrono::duration<double, std::milli>(end - start).count();
  }

  double avg_ms = total / runs;
  return {avg_ms,
          result.iterations > 0 ? avg_ms / result.iterations : avg_ms,
          result.iterations,
          result.fitness,
          result.rmse,
          result.converged};
}

// =============================================================================
// Point Cloud Generation - Realistic LiDAR Simulation
// =============================================================================

nanopcl::PointCloud generateVLP16Scan(
    std::mt19937& gen,
    const Eigen::Vector3f& sensor_pos = Eigen::Vector3f::Zero()) {
  const int num_channels = 16;
  const int points_per_channel = 1800;

  const float vertical_fov_min = -15.0f * M_PI / 180.0f;
  const float vertical_fov_max = 15.0f * M_PI / 180.0f;
  const float vertical_step = (vertical_fov_max - vertical_fov_min) / (num_channels - 1);

  std::uniform_real_distribution<float> range_dist(5.0f, 50.0f);
  std::normal_distribution<float> range_noise(0.0f, 0.02f);
  std::normal_distribution<float> angle_noise(0.0f, 0.001f);

  nanopcl::PointCloud cloud;
  cloud.reserve(num_channels * points_per_channel);

  for (int ch = 0; ch < num_channels; ++ch) {
    float elevation = vertical_fov_min + ch * vertical_step;

    for (int i = 0; i < points_per_channel; ++i) {
      float azimuth = 2.0f * M_PI * static_cast<float>(i) / points_per_channel;
      azimuth += angle_noise(gen);

      float range = range_dist(gen) + range_noise(gen);
      float elev_noisy = elevation + angle_noise(gen);

      float cos_elev = std::cos(elev_noisy);
      float x = sensor_pos.x() + range * cos_elev * std::cos(azimuth);
      float y = sensor_pos.y() + range * cos_elev * std::sin(azimuth);
      float z = sensor_pos.z() + range * std::sin(elev_noisy);

      cloud.add(x, y, z);
    }
  }

  return cloud;
}

nanopcl::PointCloud generateAccumulatedMap(int num_scans, std::mt19937& gen) {
  nanopcl::PointCloud map;
  map.reserve(num_scans * 16 * 1800);

  std::normal_distribution<float> drift(0.0f, 0.1f);

  for (int i = 0; i < num_scans; ++i) {
    Eigen::Vector3f sensor_pos(static_cast<float>(i) * 2.0f + drift(gen),
                               drift(gen) * 2.0f,
                               0.0f);

    auto scan = generateVLP16Scan(gen, sensor_pos);
    map += scan;
  }

  return map;
}

nanopcl::PointCloud applyTransform(const nanopcl::PointCloud& cloud,
                                   const Eigen::Isometry3d& T) {
  nanopcl::PointCloud result;
  result.reserve(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    Eigen::Vector3f pt = (T * cloud[i].head<3>().cast<double>()).cast<float>();
    result.add(pt.x(), pt.y(), pt.z());
  }
  return result;
}

// =============================================================================
// Print Utilities
// =============================================================================

void printHeader(const std::string& title) {
  std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
               "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  " << title << "\n";
  std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
               "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
}

void printResults(const BenchResult& p2p, const BenchResult& p2plane,
                  const BenchResult& gicp, const BenchResult& vgicp) {
  std::cout << std::fixed << std::setprecision(3);

  std::cout << "\n  ┌─────────────────┬────────────────┬────────────────┬──────────"
               "──────┬────────────────┐\n";
  std::cout << "  │     Metric      │  Point-to-Point │  Point-to-Plane │      "
               "GICP      │     VGICP      │\n";
  std::cout << "  ├─────────────────┼────────────────┼────────────────┼──────────"
               "──────┼────────────────┤\n";

  std::cout << "  │ Total time (ms) │ " << std::setw(14) << p2p.total_ms
            << " │ " << std::setw(14) << p2plane.total_ms << " │ "
            << std::setw(14) << gicp.total_ms << " │ " << std::setw(14)
            << vgicp.total_ms << " │\n";

  std::cout << "  │ Per-iter (ms)   │ " << std::setw(14) << p2p.per_iter_ms
            << " │ " << std::setw(14) << p2plane.per_iter_ms << " │ "
            << std::setw(14) << gicp.per_iter_ms << " │ " << std::setw(14)
            << vgicp.per_iter_ms << " │\n";

  std::cout << "  │ Iterations      │ " << std::setw(14) << p2p.iterations
            << " │ " << std::setw(14) << p2plane.iterations << " │ "
            << std::setw(14) << gicp.iterations << " │ " << std::setw(14)
            << vgicp.iterations << " │\n";

  std::cout << "  │ Fitness         │ " << std::setw(14) << p2p.fitness << " │ "
            << std::setw(14) << p2plane.fitness << " │ " << std::setw(14)
            << gicp.fitness << " │ " << std::setw(14) << vgicp.fitness << " │\n";

  std::cout << "  │ RMSE            │ " << std::setw(14) << p2p.rmse << " │ "
            << std::setw(14) << p2plane.rmse << " │ " << std::setw(14)
            << gicp.rmse << " │ " << std::setw(14) << vgicp.rmse << " │\n";

  std::cout << "  │ Converged       │ " << std::setw(14)
            << (p2p.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (p2plane.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (gicp.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (vgicp.converged ? "yes" : "no") << " │\n";

  std::cout << "  └─────────────────┴────────────────┴────────────────┴──────────"
               "──────┴────────────────┘\n";

  std::cout << "\n  Per-iteration speedup (VGICP vs others):\n";
  std::cout << std::setprecision(2);
  std::cout << "    - vs P2P:     " << p2p.per_iter_ms / vgicp.per_iter_ms << "x\n";
  std::cout << "    - vs P2Plane: " << p2plane.per_iter_ms / vgicp.per_iter_ms << "x\n";
  std::cout << "    - vs GICP:    " << gicp.per_iter_ms / vgicp.per_iter_ms << "x\n";
}

// =============================================================================
// Scan-to-Scan Benchmark
// =============================================================================

void runScanToScanBenchmark() {
  printHeader("SCAN-TO-SCAN BENCHMARK (Odometry Scenario)");

  std::cout << "\n  Scenario: Two consecutive VLP-16 LiDAR scans\n";
  std::cout << "  Transform: 0.5m translation + 2deg rotation\n";

  std::mt19937 gen(42);

  std::cout << "\n  Generating data:\n";
  std::cout << "    - Target scan... " << std::flush;
  auto target = generateVLP16Scan(gen);
  std::cout << "done (" << target.size() << " points)\n";

  Eigen::Isometry3d true_T = Eigen::Isometry3d::Identity();
  true_T.translation() << 0.5, 0.1, 0.02;
  Eigen::AngleAxisd rot(0.035, Eigen::Vector3d::UnitZ());
  true_T.linear() = rot.toRotationMatrix();

  std::cout << "    - Source scan... " << std::flush;
  auto source = applyTransform(target, true_T);
  std::cout << "done (" << source.size() << " points)\n";

  // Preprocessing
  std::cout << "\n  Preprocessing:\n";
  const float neighbor_radius = 1.0f;

  auto target_with_normals = target;
  std::cout << "    - Estimating normals... " << std::flush;
  auto t0 = Clock::now();
  nanopcl::geometry::estimateNormals(target_with_normals, neighbor_radius);
  auto t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)\n";

  auto source_with_cov = source;
  auto target_with_cov = target;
  std::cout << "    - Estimating covariances (source)... " << std::flush;
  t0 = Clock::now();
  nanopcl::geometry::estimateCovariances(source_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)\n";

  std::cout << "    - Estimating covariances (target)... " << std::flush;
  t0 = Clock::now();
  nanopcl::geometry::estimateCovariances(target_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)\n";

  // Settings
  nanopcl::registration::AlignSettings settings;
  settings.max_iterations = 50;
  settings.max_correspondence_dist = 2.0f;
  settings.covariance_epsilon = 1e-3;

  const float vgicp_voxel_res = 0.5f;

  // Run Benchmarks
  std::cout << "\n  Running algorithms:\n";

  std::cout << "    - Point-to-Point ICP... " << std::flush;
  auto p2p_result = measure(
      [&]() {
        return nanopcl::registration::alignICP(
            source, target, Eigen::Isometry3d::Identity(), settings);
      },
      5);
  std::cout << "done\n";

  std::cout << "    - Point-to-Plane ICP... " << std::flush;
  auto p2plane_result = measure(
      [&]() {
        return nanopcl::registration::alignPlaneICP(
            source, target_with_normals, Eigen::Isometry3d::Identity(), settings);
      },
      5);
  std::cout << "done\n";

  std::cout << "    - GICP... " << std::flush;
  auto gicp_result = measure(
      [&]() {
        return nanopcl::registration::alignGICP(
            source_with_cov, target_with_cov, Eigen::Isometry3d::Identity(), settings);
      },
      5);
  std::cout << "done\n";

  std::cout << "    - VGICP... " << std::flush;
  auto vgicp_result = measure(
      [&]() {
        return nanopcl::registration::alignVGICP(
            source_with_cov, target, vgicp_voxel_res, Eigen::Isometry3d::Identity(), settings);
      },
      5);
  std::cout << "done\n";

  printResults(p2p_result, p2plane_result, gicp_result, vgicp_result);
}

// =============================================================================
// Scan-to-Map Benchmark
// =============================================================================

void runScanToMapBenchmark(int num_map_scans) {
  std::stringstream title;
  title << "SCAN-TO-MAP BENCHMARK (" << num_map_scans << " scans, ~"
        << num_map_scans * 28800 / 1000 << "K pts)";
  printHeader(title.str());

  std::cout << "\n  Scenario: Current scan to accumulated SLAM map\n";

  std::mt19937 gen(42);

  std::cout << "\n  Generating data:\n";
  std::cout << "    - Accumulated map (" << num_map_scans << " scans)... " << std::flush;
  auto t0 = Clock::now();
  auto map = generateAccumulatedMap(num_map_scans, gen);
  auto t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms, " << map.size() << " points)\n";

  Eigen::Vector3f scan_position(static_cast<float>(num_map_scans) * 2.0f, 0.0f, 0.0f);
  std::cout << "    - Current scan... " << std::flush;
  auto scan = generateVLP16Scan(gen, scan_position);
  std::cout << "done (" << scan.size() << " points)\n";

  Eigen::Isometry3d true_T = Eigen::Isometry3d::Identity();
  true_T.translation() << 0.5, 0.1, 0.02;
  Eigen::AngleAxisd rot(0.035, Eigen::Vector3d::UnitZ());
  true_T.linear() = rot.toRotationMatrix();

  auto source = applyTransform(scan, true_T);

  // Preprocessing
  std::cout << "\n  Preprocessing:\n";
  const float neighbor_radius = 1.0f;

  auto source_with_cov = source;
  std::cout << "    - Estimating covariances (source)... " << std::flush;
  t0 = Clock::now();
  nanopcl::geometry::estimateCovariances(source_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)\n";

  // Settings
  nanopcl::registration::AlignSettings settings;
  settings.max_iterations = 50;
  settings.max_correspondence_dist = 2.0f;
  settings.covariance_epsilon = 1e-3;

  const float vgicp_voxel_res = 0.5f;

  // Run Benchmarks
  std::cout << "\n  Running algorithms:\n";

  std::cout << "    - P2P ICP (scan-to-map)... " << std::flush;
  auto p2p_s2m = measure(
      [&]() {
        return nanopcl::registration::alignICP(
            source, map, Eigen::Isometry3d::Identity(), settings);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - VGICP (scan-to-map)... " << std::flush;
  auto vgicp_s2m = measure(
      [&]() {
        return nanopcl::registration::alignVGICP(
            source_with_cov, map, vgicp_voxel_res, Eigen::Isometry3d::Identity(), settings);
      },
      3);
  std::cout << "done\n";

  // Results
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n  Scan-to-Map Results:\n";
  std::cout << "  ┌────────────────────┬────────────────┬────────────────┐\n";
  std::cout << "  │      Method        │  Time (ms)     │  Iterations    │\n";
  std::cout << "  ├────────────────────┼────────────────┼────────────────┤\n";
  std::cout << "  │ P2P (scan-to-map)  │ " << std::setw(14) << p2p_s2m.total_ms
            << " │ " << std::setw(14) << p2p_s2m.iterations << " │\n";
  std::cout << "  │ VGICP (scan-to-map)│ " << std::setw(14) << vgicp_s2m.total_ms
            << " │ " << std::setw(14) << vgicp_s2m.iterations << " │\n";
  std::cout << "  └────────────────────┴────────────────┴────────────────┘\n";

  std::cout << "\n  Speedup: " << std::setprecision(2)
            << p2p_s2m.total_ms / vgicp_s2m.total_ms << "x\n";
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
  std::cout << "╔═══════════════════════════════════════════════════════════"
               "═══════════════════════════════╗\n";
  std::cout << "║         ICP Registration Benchmark: P2P vs P2Plane vs GICP "
               "vs VGICP                    ║\n";
  std::cout << "╚═══════════════════════════════════════════════════════════"
               "═══════════════════════════════╝\n";

#ifdef _OPENMP
  std::cout << "\n  [OpenMP ENABLED]\n";
#else
  std::cout << "\n  [OpenMP DISABLED]\n";
#endif

  // Scan-to-Scan
  std::cout << "\n\n════════════════════════════════════════════════════════════"
               "══════════════════════════════\n";
  std::cout << "  PART 1: SCAN-TO-SCAN (Frame-to-Frame Odometry)\n";
  std::cout << "════════════════════════════════════════════════════════════════"
               "══════════════════════════\n";
  runScanToScanBenchmark();

  // Scan-to-Map
  std::cout << "\n\n════════════════════════════════════════════════════════════"
               "══════════════════════════════\n";
  std::cout << "  PART 2: SCAN-TO-MAP (SLAM Localization)\n";
  std::cout << "════════════════════════════════════════════════════════════════"
               "══════════════════════════\n";

  std::vector<int> map_sizes = {5, 10};
  if (argc > 1) {
    map_sizes.clear();
    for (int i = 1; i < argc; ++i) {
      map_sizes.push_back(std::stoi(argv[i]));
    }
  }

  for (int n : map_sizes) {
    runScanToMapBenchmark(n);
  }

  std::cout << "\n══════════════════════════════════════════════════════════════"
               "════════════════════════════\n";

  return 0;
}
