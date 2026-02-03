// nanoPCL Rerun Example 03: Filtering Visualization
//
// Rerun version of examples/03_filtering.cpp
// Visualizes various filtering results with KITTI data.
//
// Build: cmake -DNANOPCL_BUILD_EXAMPLES=ON -DNANOPCL_USE_RERUN=ON ..
// Run:   RERUN_SERVE=1 ./rerun_03_filtering

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <nanopcl/bridge/rerun.hpp>
#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters.hpp>
#include <nanopcl/io.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Filtering (Rerun) ===\n\n";

  // Load KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input] " << cloud.size() << " points\n\n";

  // Apply filters
  auto boxed = filters::cropBox(cloud, Point(-20, -20, -2), Point(20, 20, 2));
  std::cout << "[CropBox]    " << boxed.size() << " pts\n";

  auto ranged = filters::cropRange(cloud, 5.0f, 30.0f);
  std::cout << "[CropRange]  " << ranged.size() << " pts\n";

  auto front = filters::cropAngle(cloud, -M_PI / 4, M_PI / 4);
  std::cout << "[CropAngle]  " << front.size() << " pts (front 90 deg)\n";

  auto down = filters::voxelGrid(cloud, 0.3f);
  std::cout << "[VoxelGrid]  " << down.size() << " pts\n\n";

  // Visualize
  std::cout << "[Rerun] Sending to viewer...\n";

  rr::showCloud("filter/0_input", cloud, rr::ColorMode::ByZ);
  rr::showCloud("filter/1_cropBox", boxed, rr::ColorMode::ByZ);
  rr::showCloud("filter/2_cropRange", ranged, rr::ColorMode::ByZ);
  rr::showCloud("filter/3_cropAngle", front, rr::ColorMode::ByZ);
  rr::showCloud("filter/4_voxelGrid", down, rr::ColorMode::ByZ);

  std::cout << "\nPress Ctrl+C to exit...\n";
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
