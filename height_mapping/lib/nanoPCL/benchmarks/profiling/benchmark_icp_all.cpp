// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Benchmark: P2P ICP vs P2Plane ICP vs GICP vs VGICP
//
// This benchmark compares all four registration algorithms in two scenarios:
//   1. Scan-to-Scan: Frame-to-frame registration (typical odometry)
//   2. Scan-to-Map: Current scan to accumulated map (SLAM localization)
//
// Algorithms:
//   - P2P ICP (Point-to-Point)
//   - P2Plane ICP (Point-to-Plane, requires normals)
//   - GICP (Generalized ICP, requires covariances)
//   - VGICP (Voxelized GICP, O(1) voxel lookup - best for scan-to-map)

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <nanopcl/common.hpp>
#include <nanopcl/geometry/local_surface.hpp>
#include <nanopcl/registration.hpp>
#include <nanopcl/search.hpp>

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
  // Warmup
  auto result = func();

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

/**
 * @brief Generate realistic VLP-16 LiDAR scan
 *
 * VLP-16 specifications:
 *   - 16 channels, vertical FOV: -15° to +15°
 *   - Horizontal resolution: ~0.2° (1800 points per channel)
 *   - Range: 1m to 100m
 *   - ~28,800 points per scan
 */
npcl::PointCloud generateVLP16Scan(
    std::mt19937& gen,
    const Eigen::Vector3f& sensor_pos = Eigen::Vector3f::Zero()) {
  const int num_channels = 16;
  const int points_per_channel = 1800;

  const float vertical_fov_min = -15.0f * M_PI / 180.0f;
  const float vertical_fov_max = 15.0f * M_PI / 180.0f;
  const float vertical_step =
      (vertical_fov_max - vertical_fov_min) / (num_channels - 1);

  // Typical outdoor environment: 5-50m range
  std::uniform_real_distribution<float> range_dist(5.0f, 50.0f);
  std::normal_distribution<float> range_noise(0.0f, 0.02f);  // 2cm accuracy
  std::normal_distribution<float> angle_noise(0.0f, 0.001f);

  npcl::PointCloud cloud;
  cloud.reserve(num_channels * points_per_channel);

  for (int ch = 0; ch < num_channels; ++ch) {
    float elevation = vertical_fov_min + ch * vertical_step;

    for (int i = 0; i < points_per_channel; ++i) {
      float azimuth =
          2.0f * M_PI * static_cast<float>(i) / points_per_channel;
      azimuth += angle_noise(gen);

      float range = range_dist(gen) + range_noise(gen);
      float elev_noisy = elevation + angle_noise(gen);

      float cos_elev = std::cos(elev_noisy);
      float x = sensor_pos.x() + range * cos_elev * std::cos(azimuth);
      float y = sensor_pos.y() + range * cos_elev * std::sin(azimuth);
      float z = sensor_pos.z() + range * std::sin(elev_noisy);

      cloud.add(npcl::Point(x, y, z));
    }
  }

  return cloud;
}

/**
 * @brief Generate accumulated map from multiple scans (SLAM scenario)
 *
 * Simulates robot moving forward, accumulating LiDAR scans into a map.
 * Each scan adds ~28,800 points at a different position.
 */
npcl::PointCloud generateAccumulatedMap(int num_scans, std::mt19937& gen) {
  npcl::PointCloud map;
  map.reserve(num_scans * 16 * 1800);

  std::normal_distribution<float> drift(0.0f, 0.1f);

  for (int i = 0; i < num_scans; ++i) {
    // Robot moves ~2m forward per scan with small lateral drift
    Eigen::Vector3f sensor_pos(static_cast<float>(i) * 2.0f + drift(gen),
                               drift(gen) * 2.0f, 0.0f);

    auto scan = generateVLP16Scan(gen, sensor_pos);
    map += scan;
  }

  return map;
}

/**
 * @brief Transform point cloud by given transformation
 */
npcl::PointCloud transformCloud(const npcl::PointCloud& cloud,
                                   const Eigen::Isometry3d& T) {
  npcl::PointCloud result;
  result.reserve(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    result.add((T * cloud[i].cast<double>()).cast<float>());
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
                  const BenchResult& gicp, const BenchResult& vgicp,
                  bool is_scan_to_map = false) {
  std::cout << std::fixed << std::setprecision(3);

  const char* col_headers =
      is_scan_to_map
          ? "│     Metric      │  P2P (s2s)     │ P2Plane (s2s)  │  GICP "
            "(s2s)    │  VGICP (s2m)   │"
          : "│     Metric      │  Point-to-Point │  Point-to-Plane │      "
            "GICP      │     VGICP      │";

  std::cout
      << "\n  "
         "┌─────────────────┬────────────────┬────────────────┬──────────"
         "──────┬────────────────┐\n";
  std::cout << "  " << col_headers << "\n";
  std::cout
      << "  "
         "├─────────────────┼────────────────┼────────────────┼──────────"
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
            << gicp.fitness << " │ " << std::setw(14) << vgicp.fitness
            << " │\n";

  std::cout << "  │ RMSE            │ " << std::setw(14) << p2p.rmse << " │ "
            << std::setw(14) << p2plane.rmse << " │ " << std::setw(14)
            << gicp.rmse << " │ " << std::setw(14) << vgicp.rmse << " │\n";

  std::cout << "  │ Converged       │ " << std::setw(14)
            << (p2p.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (p2plane.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (gicp.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (vgicp.converged ? "yes" : "no") << " │\n";

  std::cout
      << "  "
         "└─────────────────┴────────────────┴────────────────┴──────────"
         "──────┴────────────────┘\n";

  // Speedup analysis
  std::cout << "\n  Per-iteration speedup (VGICP vs others):\n";
  std::cout << std::setprecision(2);
  std::cout << "    - vs P2P:     "
            << p2p.per_iter_ms / vgicp.per_iter_ms << "x\n";
  std::cout << "    - vs P2Plane: "
            << p2plane.per_iter_ms / vgicp.per_iter_ms << "x\n";
  std::cout << "    - vs GICP:    "
            << gicp.per_iter_ms / vgicp.per_iter_ms << "x\n";
}

// =============================================================================
// Scan-to-Scan Benchmark (Odometry Scenario)
// =============================================================================

/**
 * @brief Scan-to-Scan registration benchmark
 *
 * Simulates frame-to-frame LiDAR odometry:
 *   - Two consecutive VLP-16 scans
 *   - Small inter-frame displacement (~0.5m translation, ~2° rotation)
 *   - All algorithms use identical scan pair
 *
 * This is the typical odometry use case where:
 *   - Point density is similar in both scans
 *   - k-NN search is effective (similar point distributions)
 */
void runScanToScanBenchmark() {
  printHeader("SCAN-TO-SCAN BENCHMARK (Odometry Scenario)");

  std::cout << "\n  Scenario: Two consecutive VLP-16 LiDAR scans\n";
  std::cout << "  Transform: 0.5m translation + 2° rotation (typical "
               "inter-frame at 10Hz)\n";

  std::mt19937 gen(42);

  // Generate target scan (previous frame)
  std::cout << "\n  Generating data:\n";
  std::cout << "    - Target scan (previous frame)... " << std::flush;
  auto target = generateVLP16Scan(gen);
  std::cout << "done (" << target.size() << " points)\n";

  // Known transform - typical inter-frame displacement
  // At 10Hz with ~5m/s velocity: ~0.5m translation
  Eigen::Isometry3d true_T = Eigen::Isometry3d::Identity();
  true_T.translation() << 0.5, 0.1, 0.02;  // ~0.5m forward motion
  Eigen::AngleAxisd rot(0.035, Eigen::Vector3d::UnitZ());  // ~2° yaw
  true_T.linear() = rot.toRotationMatrix();

  // Generate source scan (current frame = transformed target)
  std::cout << "    - Source scan (current frame)... " << std::flush;
  auto source = transformCloud(target, true_T);
  std::cout << "done (" << source.size() << " points)\n";

  // ===========================================================================
  // Preprocessing
  // ===========================================================================
  std::cout << "\n  Preprocessing:\n";

  // Neighborhood radius for local geometry estimation
  // Rule of thumb: 5-10x point spacing for reliable surface estimation
  const float neighbor_radius = 1.0f;

  // For P2Plane: estimate normals
  auto target_with_normals = target;
  std::cout << "    - Estimating normals... " << std::flush;
  auto t0 = Clock::now();
  npcl::geometry::estimateNormals(target_with_normals, neighbor_radius);
  auto t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // For GICP: estimate covariances on both
  auto source_with_cov = source;
  auto target_with_cov = target;
  std::cout << "    - Estimating covariances (source)... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateCovariances(source_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  std::cout << "    - Estimating covariances (target)... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateCovariances(target_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // Build KdTree indices
  npcl::search::KdTree tree_basic, tree_normals, tree_cov;
  std::cout << "    - Building KdTree... " << std::flush;
  t0 = Clock::now();
  tree_basic.build(target);
  tree_normals.build(target_with_normals);
  tree_cov.build(target_with_cov);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // For VGICP: build voxel map from target scan
  const float vgicp_voxel_resolution = 0.5f;
  npcl::registration::VoxelDistributionMap voxel_map(vgicp_voxel_resolution);
  std::cout << "    - Building VoxelDistributionMap (res="
            << vgicp_voxel_resolution << "m)... " << std::flush;
  t0 = Clock::now();
  voxel_map.build(target);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms, " << voxel_map.numVoxels() << " voxels)\n";

  // ===========================================================================
  // Configuration
  // ===========================================================================
  const float max_corr_dist = 2.0f;

  npcl::registration::ICPConfig cfg;
  cfg.max_iterations = 50;
  cfg.max_correspondence_dist = max_corr_dist;

  npcl::registration::GICPConfig gicp_cfg;
  gicp_cfg.max_iterations = 50;
  gicp_cfg.max_correspondence_dist = max_corr_dist;
  gicp_cfg.covariance_epsilon = 1e-3;

  npcl::registration::VGICPConfig vgicp_cfg;
  vgicp_cfg.max_iterations = 50;
  vgicp_cfg.covariance_epsilon = 1e-3;

  // ===========================================================================
  // Run Benchmarks
  // ===========================================================================
  std::cout << "\n  Running algorithms:\n";

  std::cout << "    - Point-to-Point ICP... " << std::flush;
  auto p2p_result = measure(
      [&]() {
        return npcl::registration::icp(source, target, tree_basic,
                                          Eigen::Isometry3d::Identity(), cfg);
      },
      5);
  std::cout << "done\n";

  std::cout << "    - Point-to-Plane ICP... " << std::flush;
  auto p2plane_result = measure(
      [&]() {
        return npcl::registration::icpPlane(
            source, target_with_normals, tree_normals,
            Eigen::Isometry3d::Identity(),
            npcl::registration::ICPPlaneConfig{cfg});
      },
      5);
  std::cout << "done\n";

  std::cout << "    - GICP... " << std::flush;
  auto gicp_result = measure(
      [&]() {
        return npcl::registration::gicp(source_with_cov, target_with_cov,
                                           tree_cov,
                                           Eigen::Isometry3d::Identity(),
                                           gicp_cfg);
      },
      5);
  std::cout << "done\n";

  std::cout << "    - VGICP... " << std::flush;
  auto vgicp_result = measure(
      [&]() {
        return npcl::registration::vgicp(source_with_cov, voxel_map,
                                            Eigen::Isometry3d::Identity(),
                                            vgicp_cfg);
      },
      5);
  std::cout << "done\n";

  printResults(p2p_result, p2plane_result, gicp_result, vgicp_result, false);

  std::cout << "\n  Note: In scan-to-scan, k-NN search is effective because\n";
  std::cout << "        both point clouds have similar density.\n";
}

// =============================================================================
// Scan-to-Map Benchmark (SLAM Localization Scenario)
// =============================================================================

/**
 * @brief Scan-to-Map registration benchmark
 *
 * Simulates SLAM localization:
 *   - Source: Current VLP-16 scan (~28K points)
 *   - Target: Accumulated map from multiple scans (dense, 100K+ points)
 *
 * This is where VGICP shines:
 *   - k-NN search on large maps is expensive
 *   - Voxelized map enables O(1) correspondence lookup
 *   - Map voxels are pre-computed, amortizing covariance cost
 *
 * @param num_map_scans Number of scans to accumulate into map
 */
void runScanToMapBenchmark(int num_map_scans) {
  std::stringstream title;
  title << "SCAN-TO-MAP BENCHMARK (" << num_map_scans
        << " scans accumulated, ~" << num_map_scans * 28800 / 1000 << "K pts)";
  printHeader(title.str());

  std::cout << "\n  Scenario: Current scan to accumulated SLAM map\n";
  std::cout << "  Transform: 0.5m translation + 2° rotation\n";

  std::mt19937 gen(42);

  // ===========================================================================
  // Generate Data
  // ===========================================================================
  std::cout << "\n  Generating data:\n";

  // Generate accumulated map (target)
  std::cout << "    - Accumulated map (" << num_map_scans << " scans)... "
            << std::flush;
  auto t0 = Clock::now();
  auto map = generateAccumulatedMap(num_map_scans, gen);
  auto t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms, " << map.size() << " points)\n";

  // Generate current scan near end of trajectory
  Eigen::Vector3f scan_position(static_cast<float>(num_map_scans) * 2.0f, 0.0f,
                                0.0f);
  std::cout << "    - Current scan... " << std::flush;
  auto scan = generateVLP16Scan(gen, scan_position);
  std::cout << "done (" << scan.size() << " points)\n";

  // Known transform
  Eigen::Isometry3d true_T = Eigen::Isometry3d::Identity();
  true_T.translation() << 0.5, 0.1, 0.02;
  Eigen::AngleAxisd rot(0.035, Eigen::Vector3d::UnitZ());
  true_T.linear() = rot.toRotationMatrix();

  // Source = transformed scan
  auto source = transformCloud(scan, true_T);

  // ===========================================================================
  // Preprocessing
  // ===========================================================================
  std::cout << "\n  Preprocessing:\n";

  const float neighbor_radius = 1.0f;

  // For P2Plane/GICP: estimate on scan (not map - too expensive)
  auto scan_with_normals = scan;
  std::cout << "    - Estimating normals (scan)... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateNormals(scan_with_normals, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  auto source_with_cov = source;
  auto scan_with_cov = scan;
  std::cout << "    - Estimating covariances (source)... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateCovariances(source_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  std::cout << "    - Estimating covariances (scan)... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateCovariances(scan_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // Build KdTree on SCAN (for scan-to-scan comparison baseline)
  npcl::search::KdTree tree_scan, tree_scan_normals, tree_scan_cov;
  std::cout << "    - Building KdTree (scan)... " << std::flush;
  t0 = Clock::now();
  tree_scan.build(scan);
  tree_scan_normals.build(scan_with_normals);
  tree_scan_cov.build(scan_with_cov);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // Build KdTree on MAP (for scan-to-map with k-NN methods)
  npcl::search::KdTree tree_map;
  std::cout << "    - Building KdTree (map, " << map.size() << " pts)... "
            << std::flush;
  t0 = Clock::now();
  tree_map.build(map);
  t1 = Clock::now();
  double kdtree_map_time =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "done (" << kdtree_map_time << " ms)\n";

  // Build VoxelDistributionMap on MAP (for VGICP)
  const float vgicp_voxel_resolution = 0.5f;
  npcl::registration::VoxelDistributionMap voxel_map(vgicp_voxel_resolution);
  std::cout << "    - Building VoxelDistributionMap (map, res="
            << vgicp_voxel_resolution << "m)... " << std::flush;
  t0 = Clock::now();
  voxel_map.build(map);
  t1 = Clock::now();
  double voxel_map_time =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "done (" << voxel_map_time << " ms, " << voxel_map.numVoxels()
            << " voxels)\n";

  // ===========================================================================
  // Configuration
  // ===========================================================================
  const float max_corr_dist = 2.0f;

  npcl::registration::ICPConfig cfg;
  cfg.max_iterations = 50;
  cfg.max_correspondence_dist = max_corr_dist;

  npcl::registration::GICPConfig gicp_cfg;
  gicp_cfg.max_iterations = 50;
  gicp_cfg.max_correspondence_dist = max_corr_dist;
  gicp_cfg.covariance_epsilon = 1e-3;

  npcl::registration::VGICPConfig vgicp_cfg;
  vgicp_cfg.max_iterations = 50;
  vgicp_cfg.covariance_epsilon = 1e-3;

  // ===========================================================================
  // Run Benchmarks
  // ===========================================================================
  std::cout << "\n  Running algorithms:\n";

  // Scan-to-Scan (baseline for comparison)
  std::cout << "    - P2P ICP (scan-to-scan)... " << std::flush;
  auto p2p_s2s = measure(
      [&]() {
        return npcl::registration::icp(source, scan, tree_scan,
                                          Eigen::Isometry3d::Identity(), cfg);
      },
      5);
  std::cout << "done\n";

  std::cout << "    - P2Plane ICP (scan-to-scan)... " << std::flush;
  auto p2plane_s2s = measure(
      [&]() {
        return npcl::registration::icpPlane(
            source, scan_with_normals, tree_scan_normals,
            Eigen::Isometry3d::Identity(),
            npcl::registration::ICPPlaneConfig{cfg});
      },
      5);
  std::cout << "done\n";

  std::cout << "    - GICP (scan-to-scan)... " << std::flush;
  auto gicp_s2s = measure(
      [&]() {
        return npcl::registration::gicp(source_with_cov, scan_with_cov,
                                           tree_scan_cov,
                                           Eigen::Isometry3d::Identity(),
                                           gicp_cfg);
      },
      5);
  std::cout << "done\n";

  // Scan-to-Map (VGICP's strength)
  std::cout << "    - VGICP (scan-to-MAP)... " << std::flush;
  auto vgicp_s2m = measure(
      [&]() {
        return npcl::registration::vgicp(source_with_cov, voxel_map,
                                            Eigen::Isometry3d::Identity(),
                                            vgicp_cfg);
      },
      5);
  std::cout << "done\n";

  // Also run P2P on MAP for comparison (shows k-NN scaling issue)
  std::cout << "    - P2P ICP (scan-to-MAP)... " << std::flush;
  auto p2p_s2m = measure(
      [&]() {
        return npcl::registration::icp(source, map, tree_map,
                                          Eigen::Isometry3d::Identity(), cfg);
      },
      5);
  std::cout << "done\n";

  // ===========================================================================
  // Results
  // ===========================================================================
  std::cout << "\n  Scan-to-Scan Results (baseline):\n";
  printResults(p2p_s2s, p2plane_s2s, gicp_s2s, vgicp_s2m, true);

  std::cout << "\n  Scan-to-Map Comparison:\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout
      << "  "
         "┌────────────────────┬────────────────┬────────────────┬──────────"
         "────────────────────┐\n";
  std::cout << "  │      Method        │  Time (ms)     │  Iterations    │  "
               "Notes                        │\n";
  std::cout
      << "  "
         "├────────────────────┼────────────────┼────────────────┼──────────"
         "────────────────────┤\n";
  std::cout << "  │ P2P (scan-to-scan) │ " << std::setw(14) << p2p_s2s.total_ms
            << " │ " << std::setw(14) << p2p_s2s.iterations
            << " │ Baseline (28K vs 28K)        │\n";
  std::cout << "  │ P2P (scan-to-MAP)  │ " << std::setw(14) << p2p_s2m.total_ms
            << " │ " << std::setw(14) << p2p_s2m.iterations << " │ k-NN on "
            << map.size() / 1000 << "K pts (slow!)     │\n";
  std::cout << "  │ VGICP (scan-to-MAP)│ " << std::setw(14) << vgicp_s2m.total_ms
            << " │ " << std::setw(14) << vgicp_s2m.iterations
            << " │ O(1) voxel lookup            │\n";
  std::cout
      << "  "
         "└────────────────────┴────────────────┴────────────────┴──────────"
         "────────────────────┘\n";

  std::cout << "\n  Key Insight:\n";
  std::cout << "    - VGICP scan-to-MAP: " << vgicp_s2m.total_ms << " ms\n";
  std::cout << "    - P2P scan-to-MAP:   " << p2p_s2m.total_ms << " ms\n";
  std::cout << "    - Speedup:           "
            << p2p_s2m.total_ms / vgicp_s2m.total_ms << "x\n";
  std::cout << "\n    VGICP enables scan-to-map registration at scan-to-scan "
               "speed!\n";

  std::cout << "\n  Index Build Time Comparison:\n";
  std::cout << "    - KdTree (map):           " << kdtree_map_time << " ms\n";
  std::cout << "    - VoxelDistributionMap:   " << voxel_map_time << " ms\n";
  std::cout << "    Note: Voxel map build amortizes covariance computation.\n";
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
  std::cout << "\n  [OpenMP ENABLED - Parallel execution]\n";
#else
  std::cout << "\n  [OpenMP DISABLED - Sequential execution]\n";
#endif

  // =========================================================================
  // Scan-to-Scan Benchmark
  // =========================================================================
  std::cout
      << "\n\n════════════════════════════════════════════════════════════"
         "══════════════════════════════\n";
  std::cout << "  PART 1: SCAN-TO-SCAN (Frame-to-Frame Odometry)\n";
  std::cout << "════════════════════════════════════════════════════════════════"
               "══════════════════════════\n";
  runScanToScanBenchmark();

  // =========================================================================
  // Scan-to-Map Benchmark
  // =========================================================================
  std::cout
      << "\n\n════════════════════════════════════════════════════════════"
         "══════════════════════════════\n";
  std::cout << "  PART 2: SCAN-TO-MAP (SLAM Localization)\n";
  std::cout << "════════════════════════════════════════════════════════════════"
               "══════════════════════════\n";

  // Test with different map sizes
  std::vector<int> map_sizes = {5, 10, 20};
  if (argc > 1) {
    map_sizes.clear();
    for (int i = 1; i < argc; ++i) {
      map_sizes.push_back(std::stoi(argv[i]));
    }
  }

  for (int n : map_sizes) {
    runScanToMapBenchmark(n);
  }

  // =========================================================================
  // Summary
  // =========================================================================
  std::cout
      << "\n\n════════════════════════════════════════════════════════════"
         "══════════════════════════════\n";
  std::cout << "  SUMMARY\n";
  std::cout << "════════════════════════════════════════════════════════════════"
               "══════════════════════════\n";
  std::cout << "\n  Algorithm Selection Guide:\n";
  std::cout << "  ┌──────────────────┬──────────────────────────────────────────"
               "───────────────────┐\n";
  std::cout << "  │ Scenario         │ Recommended Algorithm                    "
               "                   │\n";
  std::cout << "  ├──────────────────┼──────────────────────────────────────────"
               "───────────────────┤\n";
  std::cout << "  │ LiDAR Odometry   │ P2Plane ICP (fast, accurate with "
               "normals)                   │\n";
  std::cout << "  │ SLAM Localization│ VGICP (O(1) map lookup, scan-to-map at "
               "scan-to-scan speed)│\n";
  std::cout << "  │ Loop Closure     │ GICP (most accurate, can afford more "
               "computation)         │\n";
  std::cout << "  │ Simple/Fast      │ P2P ICP (no preprocessing required)      "
               "                   │\n";
  std::cout << "  └──────────────────┴──────────────────────────────────────────"
               "───────────────────┘\n";

  std::cout << "\n  VGICP Key Advantage:\n";
  std::cout << "    - Per-iteration: O(1) voxel lookup vs O(log N) k-NN\n";
  std::cout << "    - Map size independent: 100K or 1M points same speed\n";
  std::cout << "    - Ideal for incremental SLAM with growing maps\n";

  std::cout
      << "\n══════════════════════════════════════════════════════════════"
         "════════════════════════════\n";

  return 0;
}
