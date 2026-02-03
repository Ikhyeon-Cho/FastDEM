// nanoPCL Example 01: Quick Start
//
// Load KITTI data and apply basic filtering.
// Build and run to see results in seconds.

#include <chrono>
#include <iomanip>
#include <iostream>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters.hpp>
#include <nanopcl/io.hpp>

using namespace nanopcl;
using Clock = std::chrono::high_resolution_clock;

int main() {
  std::cout << "=== nanoPCL Quick Start ===\n\n";

  // 1. Load KITTI LiDAR scan
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input]  KITTI scan: " << cloud.size() << " points\n\n";

  // 2. VoxelGrid downsampling
  auto t1 = Clock::now();
  auto downsampled = filters::voxelGrid(cloud, 0.2f);
  auto t2 = Clock::now();
  double dt1 = std::chrono::duration<double, std::milli>(t2 - t1).count();

  double reduction = 100.0 * (1.0 - double(downsampled.size()) / cloud.size());
  std::cout << "[Step 1] VoxelGrid (0.2m)\n";
  std::cout << "         Output: " << downsampled.size() << " points ";
  std::cout << "(" << std::fixed << std::setprecision(1) << reduction << "% reduction)\n";
  std::cout << "         Time:   " << dt1 << " ms\n\n";

  // 3. CropBox filtering (keep points within 20m x 20m x 4m region)
  auto t3 = Clock::now();
  auto cropped = filters::cropBox(downsampled, Point(-20, -20, -2), Point(20, 20, 2));
  auto t4 = Clock::now();
  double dt2 = std::chrono::duration<double, std::milli>(t4 - t3).count();

  std::cout << "[Step 2] CropBox (40x40x4m region)\n";
  std::cout << "         Output: " << cropped.size() << " points\n";
  std::cout << "         Time:   " << dt2 << " ms\n\n";

  // 4. Summary
  std::cout << "[Summary]\n";
  std::cout << "         Total:  " << cloud.size() << " -> " << cropped.size() << " points\n";
  std::cout << "         Time:   " << (dt1 + dt2) << " ms\n";

  return 0;
}
