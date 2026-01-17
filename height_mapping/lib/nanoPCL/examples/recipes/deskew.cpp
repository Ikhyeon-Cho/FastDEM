// nanoPCL Deskewing Example
#include <iostream>
#include <nanopcl/common.hpp>
#include <vector>

int main() {
  using namespace npcl;

  // 1. Create a "distorted" line
  // Assume a sensor is moving at 10m/s in +X direction while scanning a
  // vertical pillar at X=5.0 pillar points should all be at X=5.0 in world
  // frame. But due to motion, the sensor measures them at different relative X.

  PointCloud cloud("sensor");
  cloud.enableTime();

  // Pillar at X=5.0m in world frame
  // Sensor starts at (0,0,0) at t=0, ends at (1,0,0) at t=1 (10Hz, 10m/s)
  for (int i = 0; i < 100; ++i) {
    float ratio = i / 99.0f;
    float world_x = 5.0f;
    float sensor_x_at_t =
        world_x - ratio * 1.0f;  // Sensor has moved by 'ratio'

    cloud.add(Point(sensor_x_at_t, 0, float(i) * 0.1f));
    cloud.time()[i] = ratio;
  }

  std::cout << "Original (Distorted) Pillar X-range: " << cloud[0].x() << " to "
            << cloud[99].x() << "\n";

  // 2. Deskew
  // T_start = Identity
  // T_end = Translation(1, 0, 0)
  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.translation() = Eigen::Vector3d(1.0, 0, 0);

  deskew(cloud, T_start, T_end);

  std::cout << "After Deskewing (Target frame = T_end):\n";
  std::cout << "  Pillar X-range: " << cloud[0].x() << " to " << cloud[99].x()
            << "\n";

  // In T_end frame, the pillar at world_x=5.0 should be at x = 5.0 - 1.0 = 4.0
  std::cout << "  Expected X: 4.0\n";

  return 0;
}
