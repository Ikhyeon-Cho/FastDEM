// nanoPCL Rerun Example 02: Channels Visualization
//
// Rerun version of examples/02_channels.cpp
// Visualizes different channel coloring modes with KITTI data.
//
// Build: cmake -DNANOPCL_BUILD_EXAMPLES=ON -DNANOPCL_USE_RERUN=ON ..
// Run:   RERUN_SERVE=1 ./rerun_02_channels

#include <chrono>
#include <iostream>
#include <thread>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/io.hpp>
#include <nanopcl/bridge/rerun.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Channels (Rerun) ===\n\n";

  // Load KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input] " << cloud.size() << " points\n";
  std::cout << "  hasIntensity: " << cloud.hasIntensity() << "\n\n";

  // Visualize with different color modes
  std::cout << "[Rerun] Visualizing channel coloring modes...\n";

  rr::showCloud("channels/1_z", cloud, rr::ColorMode::ByZ);
  rr::showCloud("channels/2_intensity", cloud, rr::ColorMode::ByIntensity);
  rr::showCloud("channels/3_range", cloud, rr::ColorMode::ByRange);

  std::cout << "  1_z         - Color by Z value\n";
  std::cout << "  2_intensity - Color by intensity channel\n";
  std::cout << "  3_range     - Color by distance from origin\n";

  std::cout << "\nPress Ctrl+C to exit...\n";
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
