// nanoPCL Basic Example
#include <nanopcl/nanopcl.hpp>
#include <iostream>
#include <cmath>

int main() {
  using namespace nanopcl;

  std::cout << "nanoPCL version: " << version() << "\n\n";

  // Create point cloud with frame metadata
  PointCloud cloud("lidar");

  // Add some points
  for (int i = 0; i < 100; ++i) {
    float x = static_cast<float>(i) * 0.1f;
    float y = std::sin(x);
    float z = 0.0f;
    cloud.push_back(Point(x, y, z));
  }

  std::cout << "Original cloud: " << cloud.size() << " points\n";
  std::cout << "Frame: " << cloud.frameId() << "\n\n";

  // Apply voxel filter
  filters::VoxelGrid voxel({.voxel_size = 0.5f});
  auto stats = voxel.filterInPlace(cloud);

  std::cout << "After VoxelGrid (0.5m):\n";
  std::cout << "  Input: " << stats.input_size << "\n";
  std::cout << "  Output: " << stats.output_size << "\n";
  std::cout << "  Voxels: " << stats.voxel_count << "\n\n";

  // Transform the cloud
  SE3d T = SE3d::from2D(1.0, 0.0, M_PI/4);  // translate + rotate 45 deg
  cloud = T * cloud;

  std::cout << "After transform:\n";
  std::cout << "  First point: ("
            << cloud[0].x() << ", "
            << cloud[0].y() << ", "
            << cloud[0].z() << ")\n";

  return 0;
}
