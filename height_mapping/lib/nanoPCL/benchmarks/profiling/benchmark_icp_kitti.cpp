// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Benchmark: ICP algorithms on real KITTI LiDAR data
//
// This benchmark uses actual KITTI Velodyne scans to evaluate
// registration performance in realistic conditions.

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <nanopcl/common.hpp>
#include <nanopcl/geometry/local_surface.hpp>
#include <nanopcl/io.hpp>
#include <nanopcl/registration.hpp>
#include <nanopcl/search.hpp>

using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// KITTI Data Loading
// =============================================================================

/**
 * @brief Load KITTI Velodyne binary point cloud
 *
 * KITTI format: N x 4 float32 (x, y, z, intensity)
 */
npcl::PointCloud loadKittiBin(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open: " + path);
  }

  // Get file size
  file.seekg(0, std::ios::end);
  size_t file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  // Each point: 4 floats (x, y, z, intensity) = 16 bytes
  size_t num_points = file_size / (4 * sizeof(float));

  npcl::PointCloud cloud;
  cloud.reserve(num_points);
  cloud.enableIntensity();

  std::vector<float> buffer(4);
  for (size_t i = 0; i < num_points; ++i) {
    file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float));
    cloud.add(npcl::Point(buffer[0], buffer[1], buffer[2]),
              npcl::Intensity{buffer[3]});
  }

  return cloud;
}

/**
 * @brief Load KITTI poses (4x4 transformation matrices)
 *
 * Format: 12 floats per line (row-major 3x4 matrix)
 */
std::vector<Eigen::Isometry3d> loadKittiPoses(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open: " + path);
  }

  std::vector<Eigen::Isometry3d> poses;
  std::string line;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        iss >> mat(i, j);
      }
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = mat.block<3, 3>(0, 0);
    pose.translation() = mat.block<3, 1>(0, 3);
    poses.push_back(pose);
  }

  return poses;
}

// =============================================================================
// Benchmark Utilities
// =============================================================================

struct BenchResult {
  double total_ms;
  double per_iter_ms;
  size_t iterations;
  double fitness;
  double rmse;
  bool converged;
  Eigen::Isometry3d estimated_T;
};

template <typename F>
BenchResult measure(F&& func, int runs = 3) {
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
          result.converged,
          result.transform};
}

void printTransformError(const Eigen::Isometry3d& estimated,
                         const Eigen::Isometry3d& ground_truth) {
  Eigen::Isometry3d error = ground_truth.inverse() * estimated;
  double trans_error = error.translation().norm();
  double rot_error =
      std::abs(Eigen::AngleAxisd(error.rotation()).angle()) * 180.0 / M_PI;

  std::cout << "    Estimated trans:   " << estimated.translation().transpose() << " m\n";
  std::cout << "    Ground truth:      " << ground_truth.translation().transpose() << " m\n";
  std::cout << "    Translation error: " << std::fixed << std::setprecision(4)
            << trans_error << " m\n";
  std::cout << "    Rotation error:    " << rot_error << " deg\n";
}

// =============================================================================
// Main Benchmark
// =============================================================================

void runKittiBenchmark(const std::string& data_dir) {
  std::cout
      << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
         "━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  KITTI REAL DATA BENCHMARK\n";
  std::cout
      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
         "━━━━━━━━━━━━━━━━━━━━━━\n";

  // Load poses
  std::cout << "\n  Loading data:\n";
  auto poses = loadKittiPoses(data_dir + "/poses.txt");
  std::cout << "    - Loaded " << poses.size() << " poses\n";

  // Load scans (0-5)
  std::vector<npcl::PointCloud> scans;
  for (int i = 0; i <= 5; ++i) {
    std::ostringstream oss;
    oss << data_dir << "/" << std::setfill('0') << std::setw(6) << i << ".bin";
    std::cout << "    - Loading scan " << i << "... " << std::flush;
    auto scan = loadKittiBin(oss.str());
    std::cout << "done (" << scan.size() << " points)\n";
    scans.push_back(std::move(scan));
  }

  // ==========================================================================
  // Test 1: Scan-to-Scan (consecutive frames)
  // ==========================================================================
  std::cout
      << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
         "━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  TEST 1: SCAN-TO-SCAN (Frame 0 → Frame 1)\n";
  std::cout
      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
         "━━━━━━━━━━━━━━━━━━━━━━\n";

  // ===========================================================================
  // IMPORTANT: Coordinate frame transformation
  // ===========================================================================
  // KITTI poses.txt is in CAMERA 0 coordinate system (X-right, Y-down, Z-forward)
  // KITTI Velodyne scans are in LIDAR coordinate system (X-forward, Y-left, Z-up)
  //
  // T_cam0_velo (Velodyne to Camera 0) transforms: cam = R * velo
  // where R rotates: velo_X -> cam_Z, velo_Y -> -cam_X, velo_Z -> -cam_Y
  //
  // We need to convert ground truth from camera frame to Velodyne frame.
  // ===========================================================================

  // Standard KITTI T_cam0_velo rotation (approximate, actual is in calib file)
  Eigen::Matrix3d R_cam_velo;
  R_cam_velo << 0, -1, 0,   // cam_X = -velo_Y
                0,  0, -1,  // cam_Y = -velo_Z
                1,  0,  0;  // cam_Z = +velo_X

  // Ground truth in camera frame: T_cam = poses[0]^-1 * poses[1]
  // converts point from frame1 to frame0 (both in camera coordinates)
  Eigen::Isometry3d T_cam = poses[0].inverse() * poses[1];

  // Convert to Velodyne frame: T_velo = R_cam_velo^T * T_cam * R_cam_velo
  Eigen::Isometry3d T_gt_velo = Eigen::Isometry3d::Identity();
  T_gt_velo.linear() = R_cam_velo.transpose() * T_cam.linear() * R_cam_velo;
  T_gt_velo.translation() = R_cam_velo.transpose() * T_cam.translation();

  std::cout << "\n  Ground truth (converted to Velodyne frame):\n";
  std::cout << "    Translation: " << T_gt_velo.translation().transpose()
            << " m (magnitude: " << T_gt_velo.translation().norm() << " m)\n";
  std::cout << "    Rotation: "
            << Eigen::AngleAxisd(T_gt_velo.rotation()).angle() * 180.0 / M_PI
            << " deg\n";

  // For comparison, keep original
  Eigen::Isometry3d T_1_to_0 = T_gt_velo;  // Use Velodyne-frame ground truth

  auto& target = scans[0];
  auto& source = scans[1];

  // Preprocessing
  std::cout << "\n  Preprocessing:\n";
  const float neighbor_radius = 0.5f;  // 50cm for KITTI outdoor

  auto target_with_normals = target;
  std::cout << "    - Estimating normals (target)... " << std::flush;
  auto t0 = Clock::now();
  npcl::geometry::estimateNormals(target_with_normals, neighbor_radius);
  auto t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  auto source_with_cov = source;
  auto target_with_cov = target;
  std::cout << "    - Estimating covariances... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateCovariances(source_with_cov, neighbor_radius);
  npcl::geometry::estimateCovariances(target_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // Build indices
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

  const float voxel_res = 0.5f;
  npcl::registration::VoxelDistributionMap voxel_map(voxel_res);
  std::cout << "    - Building VoxelDistributionMap (res=" << voxel_res
            << "m)... " << std::flush;
  t0 = Clock::now();
  voxel_map.build(target);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms, " << voxel_map.numVoxels() << " voxels)\n";

  // Config
  npcl::registration::ICPConfig cfg;
  cfg.max_iterations = 50;
  cfg.max_correspondence_dist = 1.0f;

  npcl::registration::GICPConfig gicp_cfg;
  gicp_cfg.max_iterations = 50;
  gicp_cfg.max_correspondence_dist = 1.0f;
  gicp_cfg.covariance_epsilon = 1e-3;

  npcl::registration::VGICPConfig vgicp_cfg;
  vgicp_cfg.max_iterations = 50;
  vgicp_cfg.covariance_epsilon = 1e-3;

  // ===========================================================================
  // Test A: Without initial guess (identity)
  // ===========================================================================
  std::cout << "\n  [A] WITHOUT Initial Guess (starting from Identity):\n";

  // Run algorithms
  std::cout << "\n  Running algorithms:\n";

  std::cout << "    - P2P ICP... " << std::flush;
  auto p2p = measure(
      [&]() {
        return npcl::registration::icp(source, target, tree_basic,
                                          Eigen::Isometry3d::Identity(), cfg);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - P2Plane ICP... " << std::flush;
  auto p2plane = measure(
      [&]() {
        return npcl::registration::icpPlane(
            source, target_with_normals, tree_normals,
            Eigen::Isometry3d::Identity(),
            npcl::registration::ICPPlaneConfig{cfg});
      },
      3);
  std::cout << "done\n";

  std::cout << "    - GICP... " << std::flush;
  auto gicp = measure(
      [&]() {
        return npcl::registration::gicp(source_with_cov, target_with_cov,
                                           tree_cov,
                                           Eigen::Isometry3d::Identity(),
                                           gicp_cfg);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - VGICP... " << std::flush;
  auto vgicp = measure(
      [&]() {
        return npcl::registration::vgicp(source_with_cov, voxel_map,
                                            Eigen::Isometry3d::Identity(),
                                            vgicp_cfg);
      },
      3);
  std::cout << "done\n";

  // Results
  std::cout << std::fixed << std::setprecision(3);
  std::cout
      << "\n  "
         "┌─────────────────┬────────────────┬────────────────┬────────────────"
         "┬────────────────┐\n";
  std::cout << "  │     Metric      │      P2P       │    P2Plane     │      "
               "GICP      │     VGICP      │\n";
  std::cout
      << "  "
         "├─────────────────┼────────────────┼────────────────┼────────────────"
         "┼────────────────┤\n";

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
         "└─────────────────┴────────────────┴────────────────┴────────────────"
         "┴────────────────┘\n";

  // Transform accuracy
  std::cout << "\n  Transform Accuracy (vs Ground Truth):\n";
  std::cout << "\n    P2P ICP:\n";
  printTransformError(p2p.estimated_T, T_1_to_0);
  std::cout << "\n    P2Plane ICP:\n";
  printTransformError(p2plane.estimated_T, T_1_to_0);
  std::cout << "\n    GICP:\n";
  printTransformError(gicp.estimated_T, T_1_to_0);
  std::cout << "\n    VGICP:\n";
  printTransformError(vgicp.estimated_T, T_1_to_0);

  // ===========================================================================
  // Test B: WITH initial guess (simulating motion prediction from IMU/odometry)
  // ===========================================================================
  std::cout << "\n  ─────────────────────────────────────────────────────────────────\n";
  std::cout << "  [B] WITH Initial Guess (80% of ground truth, simulating odometry):\n";

  // Create a noisy initial guess: 80% of ground truth translation
  // This simulates wheel odometry or constant velocity model prediction
  Eigen::Isometry3d noisy_init = Eigen::Isometry3d::Identity();
  noisy_init.translation() = T_1_to_0.translation() * 0.8;
  noisy_init.linear() = T_1_to_0.linear();  // Keep rotation (small anyway)

  std::cout << "\n  Initial guess: " << noisy_init.translation().transpose()
            << " m (error from GT: "
            << (noisy_init.translation() - T_1_to_0.translation()).norm() << " m)\n";

  std::cout << "\n  Running algorithms with initial guess:\n";

  std::cout << "    - P2P ICP... " << std::flush;
  auto p2p_init = measure(
      [&]() {
        return npcl::registration::icp(source, target, tree_basic,
                                          noisy_init, cfg);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - P2Plane ICP... " << std::flush;
  auto p2plane_init = measure(
      [&]() {
        return npcl::registration::icpPlane(
            source, target_with_normals, tree_normals,
            noisy_init,
            npcl::registration::ICPPlaneConfig{cfg});
      },
      3);
  std::cout << "done\n";

  std::cout << "    - GICP... " << std::flush;
  auto gicp_init = measure(
      [&]() {
        return npcl::registration::gicp(source_with_cov, target_with_cov,
                                           tree_cov,
                                           noisy_init,
                                           gicp_cfg);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - VGICP... " << std::flush;
  auto vgicp_init = measure(
      [&]() {
        return npcl::registration::vgicp(source_with_cov, voxel_map,
                                            noisy_init,
                                            vgicp_cfg);
      },
      3);
  std::cout << "done\n";

  // Comparison table
  std::cout << std::fixed << std::setprecision(3);
  std::cout
      << "\n  "
         "┌─────────────────┬────────────────┬────────────────┬────────────────"
         "┬────────────────┐\n";
  std::cout << "  │     Metric      │      P2P       │    P2Plane     │      "
               "GICP      │     VGICP      │\n";
  std::cout
      << "  "
         "├─────────────────┼────────────────┼────────────────┼────────────────"
         "┼────────────────┤\n";

  std::cout << "  │ Total time (ms) │ " << std::setw(14) << p2p_init.total_ms
            << " │ " << std::setw(14) << p2plane_init.total_ms << " │ "
            << std::setw(14) << gicp_init.total_ms << " │ " << std::setw(14)
            << vgicp_init.total_ms << " │\n";

  std::cout << "  │ Iterations      │ " << std::setw(14) << p2p_init.iterations
            << " │ " << std::setw(14) << p2plane_init.iterations << " │ "
            << std::setw(14) << gicp_init.iterations << " │ " << std::setw(14)
            << vgicp_init.iterations << " │\n";

  std::cout << "  │ Fitness         │ " << std::setw(14) << p2p_init.fitness << " │ "
            << std::setw(14) << p2plane_init.fitness << " │ " << std::setw(14)
            << gicp_init.fitness << " │ " << std::setw(14) << vgicp_init.fitness
            << " │\n";

  std::cout
      << "  "
         "└─────────────────┴────────────────┴────────────────┴────────────────"
         "┴────────────────┘\n";

  // Transform accuracy with initial guess
  std::cout << "\n  Transform Accuracy (with initial guess):\n";
  std::cout << "\n    P2P ICP:\n";
  printTransformError(p2p_init.estimated_T, T_1_to_0);
  std::cout << "\n    P2Plane ICP:\n";
  printTransformError(p2plane_init.estimated_T, T_1_to_0);
  std::cout << "\n    GICP:\n";
  printTransformError(gicp_init.estimated_T, T_1_to_0);
  std::cout << "\n    VGICP:\n";
  printTransformError(vgicp_init.estimated_T, T_1_to_0);

  // Summary comparison
  std::cout << "\n  ═══════════════════════════════════════════════════════════════\n";
  std::cout << "  IMPROVEMENT SUMMARY (Identity → With Prior):\n";
  std::cout << "  ═══════════════════════════════════════════════════════════════\n";

  auto printImprovement = [](const std::string& name, const BenchResult& without,
                             const BenchResult& with,
                             const Eigen::Isometry3d& gt) {
    Eigen::Isometry3d err1 = gt.inverse() * without.estimated_T;
    Eigen::Isometry3d err2 = gt.inverse() * with.estimated_T;
    double trans_err1 = err1.translation().norm();
    double trans_err2 = err2.translation().norm();

    std::cout << "  " << std::setw(10) << name << ": "
              << "Time " << std::setw(6) << without.total_ms << " → "
              << std::setw(6) << with.total_ms << " ms ("
              << std::setw(5) << std::setprecision(1)
              << (without.total_ms / with.total_ms) << "x), "
              << "Error " << std::setprecision(3) << trans_err1 << " → "
              << trans_err2 << " m ("
              << std::setprecision(1) << ((trans_err1 - trans_err2) / trans_err1 * 100)
              << "% ↓)\n";
  };

  printImprovement("P2P", p2p, p2p_init, T_1_to_0);
  printImprovement("P2Plane", p2plane, p2plane_init, T_1_to_0);
  printImprovement("GICP", gicp, gicp_init, T_1_to_0);
  printImprovement("VGICP", vgicp, vgicp_init, T_1_to_0);

  // ==========================================================================
  // Test 2: Scan-to-Map (accumulated map)
  // ==========================================================================
  std::cout
      << "\n\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
         "━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  TEST 2: SCAN-TO-MAP (Frame 5 → Accumulated Map from 0-4)\n";
  std::cout
      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
         "━━━━━━━━━━━━━━━━━━━━━━\n";

  // Build accumulated map from frames 0-4 (transform to Velodyne world frame)
  // Convert poses to Velodyne frame for consistency
  std::cout << "\n  Building accumulated map (frames 0-4 in Velodyne frame):\n";
  npcl::PointCloud map;

  // Convert camera poses to Velodyne-world frame:
  // T_velo_world = R_cam_velo^T * T_cam_world
  auto poseToVeloWorld = [&R_cam_velo](const Eigen::Isometry3d& cam_pose) {
    Eigen::Isometry3d velo_pose = Eigen::Isometry3d::Identity();
    velo_pose.linear() = R_cam_velo.transpose() * cam_pose.linear() * R_cam_velo;
    velo_pose.translation() = R_cam_velo.transpose() * cam_pose.translation();
    return velo_pose;
  };

  std::vector<Eigen::Isometry3d> velo_poses;
  for (int i = 0; i <= 5; ++i) {
    velo_poses.push_back(poseToVeloWorld(poses[i]));
  }

  for (int i = 0; i <= 4; ++i) {
    std::cout << "    - Adding frame " << i << " (" << scans[i].size()
              << " pts)... " << std::flush;

    // Transform scan to Velodyne world frame using converted pose
    for (size_t j = 0; j < scans[i].size(); ++j) {
      Eigen::Vector3f pt_world =
          (velo_poses[i] * scans[i][j].cast<double>()).cast<float>();
      map.add(pt_world);
    }
    std::cout << "done\n";
  }
  std::cout << "    Total map size: " << map.size() << " points\n";

  // Source: frame 5 (in its own Velodyne frame)
  auto& source_scan = scans[5];

  // Ground truth: transform from frame 5 (Velodyne) to Velodyne-world
  Eigen::Isometry3d T_5_to_world = velo_poses[5];
  std::cout << "\n  Ground truth (frame 5 → Velodyne world):\n";
  std::cout << "    Translation: " << T_5_to_world.translation().transpose()
            << " m\n";

  // Preprocessing for source
  std::cout << "\n  Preprocessing:\n";
  auto source_scan_cov = source_scan;
  std::cout << "    - Estimating covariances (source)... " << std::flush;
  t0 = Clock::now();
  npcl::geometry::estimateCovariances(source_scan_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done ("
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms)\n";

  // Build KdTree on map
  npcl::search::KdTree tree_map;
  std::cout << "    - Building KdTree (map)... " << std::flush;
  t0 = Clock::now();
  tree_map.build(map);
  t1 = Clock::now();
  double kdtree_time =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "done (" << kdtree_time << " ms)\n";

  // Build VoxelDistributionMap on map
  npcl::registration::VoxelDistributionMap voxel_map_large(voxel_res);
  std::cout << "    - Building VoxelDistributionMap (map)... " << std::flush;
  t0 = Clock::now();
  voxel_map_large.build(map);
  t1 = Clock::now();
  double voxel_time =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "done (" << voxel_time << " ms, " << voxel_map_large.numVoxels()
            << " voxels)\n";

  // Use pose[4] as initial guess (nearby frame, in Velodyne frame)
  Eigen::Isometry3d initial_guess = velo_poses[4];

  // Run P2P on map
  std::cout << "\n  Running algorithms:\n";
  std::cout << "    - P2P ICP (scan-to-map)... " << std::flush;
  auto p2p_s2m = measure(
      [&]() {
        return npcl::registration::icp(source_scan, map, tree_map,
                                          initial_guess, cfg);
      },
      3);
  std::cout << "done\n";

  // Run VGICP on map
  std::cout << "    - VGICP (scan-to-map)... " << std::flush;
  auto vgicp_s2m = measure(
      [&]() {
        return npcl::registration::vgicp(source_scan_cov, voxel_map_large,
                                            initial_guess, vgicp_cfg);
      },
      3);
  std::cout << "done\n";

  // Results
  std::cout << "\n  Scan-to-Map Results:\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout
      << "  "
         "┌────────────────────┬────────────────┬────────────────┬────────────"
         "────────────────────┐\n";
  std::cout << "  │      Method        │  Time (ms)     │  Iterations    │  "
               "Notes                        │\n";
  std::cout
      << "  "
         "├────────────────────┼────────────────┼────────────────┼────────────"
         "────────────────────┤\n";
  std::cout << "  │ P2P (scan-to-map)  │ " << std::setw(14) << p2p_s2m.total_ms
            << " │ " << std::setw(14) << p2p_s2m.iterations << " │ KdTree on "
            << map.size() / 1000 << "K pts          │\n";
  std::cout << "  │ VGICP (scan-to-map)│ " << std::setw(14)
            << vgicp_s2m.total_ms << " │ " << std::setw(14)
            << vgicp_s2m.iterations << " │ O(1) voxel lookup ("
            << voxel_map_large.numVoxels() << " vox) │\n";
  std::cout
      << "  "
         "└────────────────────┴────────────────┴────────────────┴────────────"
         "────────────────────┘\n";

  std::cout << "\n  Speedup: " << std::setprecision(2)
            << p2p_s2m.total_ms / vgicp_s2m.total_ms << "x\n";

  std::cout << "\n  Transform Accuracy:\n";
  std::cout << "\n    P2P (scan-to-map):\n";
  std::cout << "      Fitness: " << p2p_s2m.fitness
            << ", RMSE: " << p2p_s2m.rmse << "\n";
  printTransformError(p2p_s2m.estimated_T, T_5_to_world);

  std::cout << "\n    VGICP (scan-to-map):\n";
  std::cout << "      Fitness: " << vgicp_s2m.fitness
            << ", RMSE: " << vgicp_s2m.rmse << "\n";
  printTransformError(vgicp_s2m.estimated_T, T_5_to_world);

  std::cout << "\n  Index Build Time:\n";
  std::cout << "    - KdTree:              " << kdtree_time << " ms\n";
  std::cout << "    - VoxelDistributionMap: " << voxel_time << " ms\n";
}

int main(int argc, char** argv) {
  std::cout << "╔═══════════════════════════════════════════════════════════"
               "═══════════════════════════════╗\n";
  std::cout << "║              KITTI Real Data ICP Benchmark                 "
               "                           ║\n";
  std::cout << "╚═══════════════════════════════════════════════════════════"
               "═══════════════════════════════╝\n";

#ifdef _OPENMP
  std::cout << "\n  [OpenMP ENABLED]\n";
#else
  std::cout << "\n  [OpenMP DISABLED]\n";
#endif

  std::string data_dir = "../data/kitti";
  if (argc > 1) {
    data_dir = argv[1];
  }

  try {
    runKittiBenchmark(data_dir);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  std::cout
      << "\n══════════════════════════════════════════════════════════════"
         "════════════════════════════\n";

  return 0;
}
