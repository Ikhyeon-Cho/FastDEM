// Quick compile test for plane_segmentation.hpp
#include <iostream>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/segmentation/ransac_plane.hpp"

int main() {
  using namespace npcl;

  // Create test cloud with ground plane + noise
  PointCloud cloud;
  cloud.reserve(10000);

  // Ground plane (z ≈ 0)
  for (int i = 0; i < 8000; ++i) {
    float x = (i % 100) * 0.1f - 5.0f;
    float y = (i / 100) * 0.1f - 4.0f;
    float z = 0.0f + ((i % 7) - 3) * 0.02f;  // Small noise
    cloud.add(Point(x, y, z));
  }

  // Random obstacles
  for (int i = 0; i < 2000; ++i) {
    float x = (i % 50) * 0.2f - 5.0f;
    float y = (i / 50) * 0.2f - 4.0f;
    float z = 0.5f + (i % 10) * 0.1f;
    cloud.add(Point(x, y, z));
  }

  std::cout << "Cloud size: " << cloud.size() << std::endl;

  // Test segmentPlane
  auto result = segmentation::segmentPlane(cloud, 0.1f);

  std::cout << "=== Single Plane Segmentation ===" << std::endl;
  std::cout << "Success: " << (result.success() ? "yes" : "no") << std::endl;
  std::cout << "Inliers: " << result.inliers.size() << std::endl;
  std::cout << "Fitness: " << result.fitness * 100 << "%" << std::endl;
  std::cout << "Iterations: " << result.iterations << std::endl;
  std::cout << "Normal: [" << result.model.normal.transpose() << "]"
            << std::endl;
  std::cout << "d: " << result.model.d << std::endl;

  // Test outliers
  auto outliers = result.outliers(cloud.size());
  std::cout << "Outliers: " << outliers.size() << std::endl;

  // Test segmentMultiplePlanes
  segmentation::RansacConfig config;
  config.distance_threshold = 0.1f;
  config.max_iterations = 500;

  auto planes = segmentation::segmentMultiplePlanes(cloud, config, 3, 0.05);
  std::cout << "\n=== Multiple Plane Segmentation ===" << std::endl;
  std::cout << "Found " << planes.size() << " planes" << std::endl;
  for (size_t i = 0; i < planes.size(); ++i) {
    std::cout << "  Plane " << i << ": " << planes[i].inliers.size()
              << " points, fitness=" << planes[i].fitness * 100 << "%"
              << std::endl;
  }

  

  return 0;
}
