// nanoPCL Example 02: Channels
//
// nanoPCL uses Structure-of-Arrays (SoA) design.
// Channels are created automatically when you use them.

#include <iostream>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/io.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Channels ===\n\n";

  // [1] Load KITTI data (has intensity channel)
  std::cout << "[1] Load KITTI Data\n";
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);

  std::cout << "    Loaded " << cloud.size() << " points\n";
  std::cout << "    hasIntensity: " << cloud.hasIntensity() << "\n";
  std::cout << "    hasRing:      " << cloud.hasRing() << "\n\n";

  // [2] Access channels
  std::cout << "[2] Channel Access\n";
  std::cout << "    First 5 points:\n";
  for (size_t i = 0; i < 5; ++i) {
    std::cout << "    [" << i << "] xyz=(" << cloud.point(i).head<3>().transpose() << ")";
    std::cout << " intensity=" << cloud.intensity(i) << "\n";
  }

  // [3] Bulk channel access (cache-friendly)
  std::cout << "\n[3] Bulk Channel Access\n";
  float sum = 0;
  for (float val : cloud.intensities()) {
    sum += val;
  }
  std::cout << "    Sum of intensities: " << sum << "\n";
  std::cout << "    Mean intensity: " << sum / cloud.size() << "\n";

  // [4] Add new channel to existing cloud
  std::cout << "\n[4] Add Ring Channel\n";
  cloud.useRing();
  for (size_t i = 0; i < cloud.size(); ++i) {
    cloud.ring(i) = i % 64;  // Simulate 64-channel LiDAR
  }
  std::cout << "    hasRing: " << cloud.hasRing() << "\n";

  // [5] Available channels
  std::cout << "\n[Available Channels]\n";
  std::cout << "    Intensity(float)  Ring(uint16)  Time(float)\n";
  std::cout << "    Color(r,g,b)      Label(uint32)\n";
  std::cout << "    Normal vector     Covariance matrix\n";

  return 0;
}
