// nanoPCL Example 03: Filtering
//
// Common filtering operations with KITTI data.

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters.hpp>
#include <nanopcl/io.hpp>

using namespace nanopcl;
using Clock = std::chrono::high_resolution_clock;

int main() {
  std::cout << "=== nanoPCL Filtering ===\n\n";

  // Load KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto cloud = io::loadKITTI(path);
  std::cout << "[Input] " << cloud.size() << " points\n\n";
  std::cout << std::fixed << std::setprecision(2);

  auto bench = [](auto&& fn) {
    auto t1 = Clock::now();
    auto result = fn();
    auto t2 = Clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    return std::make_pair(std::move(result), ms);
  };

  // 1. CropBox
  auto [boxed, t1] = bench([&]() {
    return filters::cropBox(cloud, Point(-20, -20, -2), Point(20, 20, 2));
  });
  std::cout << "[CropBox]    40x40x4m       " << boxed.size() << " pts  (" << t1 << " ms)\n";

  // 2. CropRange
  auto [ranged, t2] = bench([&]() {
    return filters::cropRange(cloud, 5.0f, 30.0f);
  });
  std::cout << "[CropRange]  [5m, 30m]      " << ranged.size() << " pts  (" << t2 << " ms)\n";

  // 3. CropZ
  auto [zfiltered, t3] = bench([&]() {
    return filters::cropZ(cloud, -1.5f, 1.0f);
  });
  std::cout << "[CropZ]      [-1.5m, 1m]    " << zfiltered.size() << " pts  (" << t3 << " ms)\n";

  // 4. CropAngle (front 90 degrees)
  auto [front, t4] = bench([&]() {
    return filters::cropAngle(cloud, -M_PI / 4, M_PI / 4);
  });
  std::cout << "[CropAngle]  [-45, +45 deg] " << front.size() << " pts  (" << t4 << " ms)\n";

  // 5. VoxelGrid
  auto [down, t5] = bench([&]() {
    return filters::voxelGrid(cloud, 0.3f);
  });
  std::cout << "[VoxelGrid]  0.3m           " << down.size() << " pts  (" << t5 << " ms)\n";

  // 6. Pipeline
  std::cout << "\n[Pipeline] cropRange -> cropZ -> voxelGrid\n";
  auto [result, t6] = bench([&]() {
    return filters::voxelGrid(
        filters::cropZ(
            filters::cropRange(cloud, 3.0f, 40.0f),
            -2.0f, 2.0f),
        0.2f);
  });
  std::cout << "           " << cloud.size() << " -> " << result.size() << " pts  (" << t6 << " ms)\n";

  return 0;
}
