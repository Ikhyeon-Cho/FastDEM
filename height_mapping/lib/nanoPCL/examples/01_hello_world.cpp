// nanoPCL Example 01: Hello World
//
// Minimal example showing core PointCloud usage.

#include <iostream>
#include <nanopcl/core.hpp>

using namespace npcl;

int main() {
  // 1. Create a point cloud
  PointCloud cloud;

  // 2. Add points (PCL-style)
  cloud.add(PointXYZI{1.0f, 2.0f, 3.0f, 0.5f});
  cloud.add(PointXYZI{4.0f, 5.0f, 6.0f, 0.8f});
  cloud.add(PointXYZI{7.0f, 8.0f, 9.0f, 0.3f});

  // 3a. Index-based iteration with PointRef
  std::cout << "Points (index-based):\n";
  for (size_t i = 0; i < cloud.size(); ++i) {
    auto pt = cloud.point(i);
    std::cout << "  [" << i << "] (" << pt.x() << ", " << pt.y() << ", "
              << pt.z() << ") i=" << pt.intensity() << "\n";
  }

  // 3b. Range-based for (returns PointRef)
  std::cout << "\nPoints (range-based):\n";
  for (auto pt : cloud) {
    std::cout << "  (" << pt.x() << ", " << pt.y() << ", " << pt.z() << ")\n";
  }

  // 4. Direct channel access (fast, cache-friendly)
  std::cout << "\nDirect channel access - double all intensities:\n";
  std::cout << "  Before: ";
  for (float i : cloud.intensity()) std::cout << i << " ";
  std::cout << "\n";

  for (float& i : cloud.intensity()) {
    i *= 2.0f;
  }

  std::cout << "  After:  ";
  for (float i : cloud.intensity()) std::cout << i << " ";
  std::cout << "\n";

  // 5. Extract subset by indices
  std::vector<size_t> keep;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud.point(i).intensity() > 1.0f) keep.push_back(i);
  }
  PointCloud filtered = cloud[keep];
  std::cout << "\nFiltered: " << filtered.size() << " points\n";

  // 6. Frame and timestamp (metadata)
  PointCloud cloud2("lidar");  // via constructor
  cloud2.setTimestamp(time::now());

  cloud.setFrameId("lidar");  // via setter
  cloud.setTimestamp(time::now());

  std::cout << "\nMetadata:\n";
  std::cout << "  Frame: " << cloud.frameId() << "\n";
  std::cout << "  Time:  " << time::toSec(cloud.timestamp()) << " sec\n";

  return 0;
}
