// nanoPCL Rerun Example 01: Quick Start Visualization
//
// Rerun version of examples/01_quick_start.cpp
// Visualizes KITTI LiDAR scan with filtering results.
//
// Build: cmake -DNANOPCL_BUILD_EXAMPLES=ON -DNANOPCL_USE_RERUN=ON ..
// Run:   RERUN_SERVE=1 ./rerun_01_quick_start

#include <chrono>
#include <iostream>
#include <thread>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters.hpp>
#include <nanopcl/io.hpp>
#include <nanopcl/bridge/rerun.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Quick Start (Rerun) ===\n\n";

  // 1. Load KITTI scan
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input] " << cloud.size() << " points (KITTI)\n";

  // 2. VoxelGrid downsampling
  auto downsampled = filters::voxelGrid(cloud, 0.2f);
  std::cout << "[VoxelGrid] " << downsampled.size() << " points\n";

  // 3. CropBox filtering
  auto cropped = filters::cropBox(downsampled, Point(-20, -20, -2), Point(20, 20, 2));
  std::cout << "[CropBox] " << cropped.size() << " points\n\n";

  // Visualize with Rerun
  std::cout << "[Rerun] Sending to viewer...\n";
  rr::showCloud("lidar/1_input", cloud, rr::ColorMode::ByZ);
  rr::showCloud("lidar/2_downsampled", downsampled, rr::ColorMode::ByZ);
  rr::showCloud("lidar/3_cropped", cropped, rr::ColorMode::ByZ);

  std::cout << "\nPress Ctrl+C to exit...\n";
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
