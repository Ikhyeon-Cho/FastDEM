// nanoPCL Deskewing Example
//
// Demonstrates motion distortion correction for spinning LiDAR data.
// During a scan, the sensor moves, causing geometric distortion.
// deskew() corrects this by transforming all points to a common reference frame.

#include <iostream>
#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters/deskew.hpp>

int main() {
  using namespace nanopcl;

  // ==========================================================================
  // 1. Create a "distorted" point cloud
  // ==========================================================================
  // Scenario: Sensor moves 1m in +X direction while scanning a vertical pillar
  // at world X=5.0. Due to motion, sensor measures pillar at varying relative X.

  PointCloud cloud;
  cloud.useTime();

  // Pillar at X=5.0m in world frame
  // Sensor: starts at (0,0,0) at t=0, ends at (1,0,0) at t=1
  for (int i = 0; i < 100; ++i) {
    float ratio = static_cast<float>(i) / 99.0f;
    float world_x = 5.0f;
    float sensor_x_at_t = world_x - ratio * 1.0f; // Relative to moving sensor

    cloud.add(sensor_x_at_t, 0.0f, static_cast<float>(i) * 0.1f);
    cloud.time(i) = ratio; // Normalized time [0, 1]
  }

  std::cout << "=== nanoPCL Deskewing Example ===\n\n";
  std::cout << "Original (Distorted):\n";
  std::cout << "  X-range: " << cloud[0].x() << " to " << cloud[99].x() << "\n";

  // ==========================================================================
  // 2. Deskew using linear interpolation
  // ==========================================================================
  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.translation() = Eigen::Vector3d(1.0, 0, 0);

  auto corrected = filters::deskew(cloud, T_start, T_end);

  std::cout << "\nAfter Deskewing (to T_end frame):\n";
  std::cout << "  X-range: " << corrected[0].x() << " to " << corrected[99].x()
            << "\n";
  std::cout << "  Expected: all points at X = 4.0\n";

  // ==========================================================================
  // 3. Alternative: Callback-based deskew
  // ==========================================================================
  // For complex trajectories or TF integration:
  //
  // auto corrected = filters::deskew(cloud, [&](double t) {
  //     return trajectory.poseAt(t);  // or tf_buffer.lookupTransform(...)
  // });

  return 0;
}
