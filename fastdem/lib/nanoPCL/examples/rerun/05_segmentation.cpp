// nanoPCL Rerun Example 05: Segmentation Visualization
//
// Rerun version of examples/05_segmentation.cpp
// Visualizes ground removal and clustering with KITTI data.
//
// Build: cmake -DNANOPCL_BUILD_EXAMPLES=ON -DNANOPCL_USE_RERUN=ON ..
// Run:   RERUN_SERVE=1 ./rerun_05_segmentation

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/io.hpp>
#include <nanopcl/segmentation.hpp>
#include <nanopcl/bridge/rerun.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Segmentation (Rerun) ===\n\n";
  std::cout << std::fixed << std::setprecision(2);

  // Load KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input] " << cloud.size() << " points (KITTI)\n\n";

  // 1. Ground Segmentation
  std::cout << "[1] Ground Segmentation\n";
  auto ground_result = segmentation::segmentGround(cloud);
  std::cout << "    Ground: " << ground_result.ground.size() << " pts\n";
  std::cout << "    Obstacles: " << ground_result.obstacles.size() << " pts\n\n";

  // 2. Euclidean Clustering on obstacles
  std::cout << "[2] Euclidean Clustering\n";
  auto clusters = segmentation::euclideanCluster(cloud, ground_result.obstacles, 0.5f);
  std::cout << "    Found " << clusters.numClusters() << " clusters\n";

  // 3. RANSAC Plane Fitting
  std::cout << "\n[3] RANSAC Plane Fitting\n";
  auto plane = segmentation::segmentPlane(cloud, 0.2f);
  std::cout << "    Plane inliers: " << plane.inliers.size() << " pts\n\n";

  // Visualize
  std::cout << "[Rerun] Visualizing segmentation results...\n";

  rr::showCloud("segmentation/0_input", cloud, rr::ColorMode::ByZ);
  rr::showGroundSegmentation("segmentation/1_ground", cloud, ground_result.ground);
  rr::showClusters("segmentation/2_clusters", cloud, clusters);
  rr::showPlaneSegmentation("segmentation/3_ransac", cloud, plane.inliers);

  std::cout << "\nPress Ctrl+C to exit...\n";
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
