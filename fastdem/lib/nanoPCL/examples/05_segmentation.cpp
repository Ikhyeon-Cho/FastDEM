// nanoPCL Example 05: Segmentation
//
// Ground removal and object clustering with KITTI data.

#include <chrono>
#include <iomanip>
#include <iostream>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/io.hpp>
#include <nanopcl/segmentation.hpp>

using namespace nanopcl;
using Clock = std::chrono::high_resolution_clock;

int main() {
  std::cout << "=== nanoPCL Segmentation ===\n\n";
  std::cout << std::fixed << std::setprecision(2);

  // Load KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input] " << cloud.size() << " points (KITTI)\n\n";

  // 1. Ground Segmentation
  std::cout << "[1] Ground Segmentation\n";
  auto t1 = Clock::now();
  auto ground_result = segmentation::segmentGround(cloud);
  auto t2 = Clock::now();
  double dt1 = std::chrono::duration<double, std::milli>(t2 - t1).count();

  std::cout << "    Ground points:   " << ground_result.ground.size() << "\n";
  std::cout << "    Obstacle points: " << ground_result.obstacles.size() << "\n";
  std::cout << "    Time: " << dt1 << " ms\n\n";

  // Extract obstacles for clustering
  PointCloud obstacles = cloud.extract(
      std::vector<size_t>(ground_result.obstacles.begin(), ground_result.obstacles.end()));

  // 2. Euclidean Clustering
  std::cout << "[2] Euclidean Clustering\n";
  auto t3 = Clock::now();
  auto clusters = segmentation::euclideanCluster(obstacles, 0.5f);
  auto t4 = Clock::now();
  double dt2 = std::chrono::duration<double, std::milli>(t4 - t3).count();

  std::cout << "    Found " << clusters.numClusters() << " clusters\n";
  for (size_t i = 0; i < std::min(clusters.numClusters(), size_t(5)); ++i) {
    std::cout << "    Cluster " << i << ": " << clusters.clusterSize(i) << " points\n";
  }
  if (clusters.numClusters() > 5) {
    std::cout << "    ... and " << (clusters.numClusters() - 5) << " more\n";
  }
  std::cout << "    Time: " << dt2 << " ms\n\n";

  // 3. RANSAC Plane Fitting
  std::cout << "[3] RANSAC Plane Fitting\n";
  auto t5 = Clock::now();
  auto plane = segmentation::segmentPlane(cloud, 0.2f);
  auto t6 = Clock::now();
  double dt3 = std::chrono::duration<double, std::milli>(t6 - t5).count();

  std::cout << "    Plane inliers: " << plane.inliers.size() << "\n";
  std::cout << "    Plane: [" << plane.model.coefficients.transpose() << "]\n";
  std::cout << "    Time: " << dt3 << " ms\n";

  return 0;
}
