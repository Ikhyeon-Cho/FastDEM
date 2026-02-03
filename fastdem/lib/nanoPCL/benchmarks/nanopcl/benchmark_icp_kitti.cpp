// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Benchmark: ICP algorithms on real KITTI LiDAR data

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <nanopcl/common.hpp>
#include <nanopcl/geometry/normal_estimation.hpp>
#include <nanopcl/registration.hpp>

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#endif

using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// KITTI Data Loading
// =============================================================================

nanopcl::PointCloud loadKittiBin(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open: " + path);
  }

  file.seekg(0, std::ios::end);
  size_t file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  size_t num_points = file_size / (4 * sizeof(float));

  nanopcl::PointCloud cloud;
  cloud.reserve(num_points);
  cloud.useIntensity();

  std::vector<float> buffer(4);
  for (size_t i = 0; i < num_points; ++i) {
    file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float));
    cloud.add(buffer[0], buffer[1], buffer[2], nanopcl::Intensity{buffer[3]});
  }

  return cloud;
}

#ifdef HAVE_PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr nanoToPCL(const nanopcl::PointCloud& cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    pcl_cloud->points[i].x = cloud.points()[i].x();
    pcl_cloud->points[i].y = cloud.points()[i].y();
    pcl_cloud->points[i].z = cloud.points()[i].z();
  }
  return pcl_cloud;
}
#endif

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

#ifdef HAVE_PCL
BenchResult measurePCL(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
                       int runs = 3) {
  pcl::PointCloud<pcl::PointXYZ> final;
  icp.setInputSource(source);
  icp.align(final); // Warmup

  double total = 0;
  for (int i = 0; i < runs; ++i) {
    auto start = Clock::now();
    icp.setInputSource(source); // Reset source to initial state (though PCL copies internally)
    icp.align(final);
    auto end = Clock::now();
    total += std::chrono::duration<double, std::milli>(end - start).count();
  }
  
  double avg_ms = total / runs;
  
  // Convert result to common format
  Eigen::Isometry3d estimated_T = Eigen::Isometry3d::Identity();
  estimated_T.matrix() = icp.getFinalTransformation().cast<double>();
  
  // PCL hides iteration count if converged early, but we can't easily get it without hacking.
  // We'll trust its convergence.
  return {avg_ms, avg_ms, 0, icp.getFitnessScore(), 0.0, icp.hasConverged(), estimated_T}; 
}
#endif

void printTransformError(const Eigen::Isometry3d& estimated,
                         const Eigen::Isometry3d& ground_truth) {
  Eigen::Isometry3d error = ground_truth.inverse() * estimated;
  double trans_error = error.translation().norm();
  double rot_error = std::abs(Eigen::AngleAxisd(error.rotation()).angle()) * 180.0 / M_PI;

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
  std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
               "━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  KITTI REAL DATA BENCHMARK\n";
  std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
               "━━━━━━━━━━━━━━━━━━━━━━\n";

  // Load poses
  std::cout << "\n  Loading data:\n";
  auto poses = loadKittiPoses(data_dir + "/poses.txt");
  std::cout << "    - Loaded " << poses.size() << " poses\n";

  // Load scans
  std::vector<nanopcl::PointCloud> scans;
  for (int i = 0; i <= 2; ++i) {
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
  std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
               "━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  TEST 1: SCAN-TO-SCAN (Frame 0 -> Frame 1)\n";
  std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
               "━━━━━━━━━━━━━━━━━━━━━━\n";

  // KITTI coordinate frame transformation
  Eigen::Matrix3d R_cam_velo;
  R_cam_velo << 0, -1, 0,
                0, 0, -1,
                1, 0, 0;

  Eigen::Isometry3d T_cam = poses[0].inverse() * poses[1];
  Eigen::Isometry3d T_gt_velo = Eigen::Isometry3d::Identity();
  T_gt_velo.linear() = R_cam_velo.transpose() * T_cam.linear() * R_cam_velo;
  T_gt_velo.translation() = R_cam_velo.transpose() * T_cam.translation();

  std::cout << "\n  Ground truth (Velodyne frame):\n";
  std::cout << "    Translation: " << T_gt_velo.translation().transpose()
            << " m (magnitude: " << T_gt_velo.translation().norm() << " m)\n";

  Eigen::Isometry3d T_1_to_0 = T_gt_velo;

  auto& target = scans[0];
  auto& source = scans[1];

#ifdef HAVE_PCL
  std::cout << "    - Converting to PCL... " << std::flush;
  auto pcl_target = nanoToPCL(target);
  auto pcl_source = nanoToPCL(source);
  std::cout << "done\n";
#endif

  // Preprocessing
  std::cout << "\n  Preprocessing:\n";
  const float neighbor_radius = 0.5f;

  auto target_with_normals = target;
  std::cout << "    - Estimating normals (target)... " << std::flush;
  auto t0 = Clock::now();
  nanopcl::geometry::estimateNormals(target_with_normals, neighbor_radius);
  auto t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)\n";

  auto source_with_cov = source;
  auto target_with_cov = target;
  std::cout << "    - Estimating covariances... " << std::flush;
  t0 = Clock::now();
  nanopcl::geometry::estimateCovariances(source_with_cov, neighbor_radius);
  nanopcl::geometry::estimateCovariances(target_with_cov, neighbor_radius);
  t1 = Clock::now();
  std::cout << "done (" << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms)\n";

  // Settings
  nanopcl::registration::AlignSettings settings;
  settings.max_iterations = 50;
  settings.max_correspondence_dist = 1.0f;
  settings.covariance_epsilon = 1e-3;

  const float voxel_res = 0.5f;

  // Run algorithms
  std::cout << "\n  Running algorithms:\n";

#ifdef HAVE_PCL
  std::cout << "    - PCL ICP... " << std::flush;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_icp;
  pcl_icp.setMaximumIterations(50);
  pcl_icp.setMaxCorrespondenceDistance(1.0);
  pcl_icp.setTransformationEpsilon(1e-8);
  pcl_icp.setEuclideanFitnessEpsilon(1e-5);
  pcl_icp.setInputTarget(pcl_target);
  auto pcl_res = measurePCL(pcl_icp, pcl_source, 3);
  std::cout << "done\n";
#endif

  std::cout << "    - P2P ICP... " << std::flush;
  auto p2p = measure(
      [&]() {
        return nanopcl::registration::alignICP(
            source, target, Eigen::Isometry3d::Identity(), settings);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - P2Plane ICP... " << std::flush;
  auto p2plane = measure(
      [&]() {
        return nanopcl::registration::alignPlaneICP(
            source, target_with_normals, Eigen::Isometry3d::Identity(), settings);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - GICP... " << std::flush;
  auto gicp = measure(
      [&]() {
        return nanopcl::registration::alignGICP(
            source_with_cov, target_with_cov, Eigen::Isometry3d::Identity(), settings);
      },
      3);
  std::cout << "done\n";

  std::cout << "    - VGICP... " << std::flush;
  auto vgicp = measure(
      [&]() {
        return nanopcl::registration::alignVGICP(
            source_with_cov, target, voxel_res, Eigen::Isometry3d::Identity(), settings);
      },
      3);
  std::cout << "done\n";

  // Results
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "\n  ┌─────────────────┬────────────────┬────────────────┬────────────────"
               "┬────────────────┬────────────────┐\n";
  std::cout << "  │     Metric      │    PCL ICP     │    nanoP2P     │    P2Plane     │      "
               "GICP      │     VGICP      │\n";
  std::cout << "  ├─────────────────┼────────────────┼────────────────┼────────────────┼────────────────"
               "┼────────────────┤\n";

#ifdef HAVE_PCL
  std::cout << "  │ Total time (ms) │ " << std::setw(14) << pcl_res.total_ms << " │ ";
#else
  std::cout << "  │ Total time (ms) │ " << std::setw(14) << "N/A" << " │ ";
#endif
  std::cout << std::setw(14) << p2p.total_ms
            << " │ " << std::setw(14) << p2plane.total_ms << " │ "
            << std::setw(14) << gicp.total_ms << " │ " << std::setw(14)
            << vgicp.total_ms << " │\n";

#ifdef HAVE_PCL
  std::cout << "  │ Iterations      │ " << std::setw(14) << "-" << " │ ";
#else
  std::cout << "  │ Iterations      │ " << std::setw(14) << "N/A" << " │ ";
#endif
  std::cout << std::setw(14) << p2p.iterations
            << " │ " << std::setw(14) << p2plane.iterations << " │ "
            << std::setw(14) << gicp.iterations << " │ " << std::setw(14)
            << vgicp.iterations << " │\n";

#ifdef HAVE_PCL
  std::cout << "  │ Fitness         │ " << std::setw(14) << pcl_res.fitness << " │ ";
#else
  std::cout << "  │ Fitness         │ " << std::setw(14) << "N/A" << " │ ";
#endif
  std::cout << std::setw(14) << p2p.fitness << " │ "
            << std::setw(14) << p2plane.fitness << " │ " << std::setw(14)
            << gicp.fitness << " │ " << std::setw(14) << vgicp.fitness << " │\n";

  std::cout << "  │ RMSE            │ " << std::setw(14) << "-" << " │ "
            << std::setw(14) << p2p.rmse << " │ "
            << std::setw(14) << p2plane.rmse << " │ " << std::setw(14)
            << gicp.rmse << " │ " << std::setw(14) << vgicp.rmse << " │\n";

#ifdef HAVE_PCL
  std::cout << "  │ Converged       │ " << std::setw(14) << (pcl_res.converged ? "yes" : "no") << " │ ";
#else
  std::cout << "  │ Converged       │ " << std::setw(14) << "N/A" << " │ ";
#endif
  std::cout << std::setw(14)
            << (p2p.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (p2plane.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (gicp.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (vgicp.converged ? "yes" : "no") << " │\n";

  std::cout << "  └─────────────────┴────────────────┴────────────────┴────────────────"
               "┴────────────────┴────────────────┘\n";

  // Transform accuracy
  std::cout << "\n  Transform Accuracy (vs Ground Truth):\n";
#ifdef HAVE_PCL
  std::cout << "\n    PCL ICP:\n";
  printTransformError(pcl_res.estimated_T, T_1_to_0);
#endif
  std::cout << "\n    nanoPCL P2P ICP:\n";
  printTransformError(p2p.estimated_T, T_1_to_0);
  std::cout << "\n    nanoPCL P2Plane ICP:\n";
  printTransformError(p2plane.estimated_T, T_1_to_0);
  std::cout << "\n    nanoPCL GICP:\n";
  printTransformError(gicp.estimated_T, T_1_to_0);
  std::cout << "\n    nanoPCL VGICP:\n";
  printTransformError(vgicp.estimated_T, T_1_to_0);
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

  std::cout << "\n══════════════════════════════════════════════════════════════"
               "════════════════════════════\n";

  return 0;
}
