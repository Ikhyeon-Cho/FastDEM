// nanoPCL Example 05: I/O
//
// Save and load point clouds: PCD (ASCII) and BIN (KITTI binary) formats.

#include <cmath>
#include <iostream>
#include <nanopcl/core.hpp>
#include <nanopcl/io.hpp>

using namespace npcl;

int main() {
  // 1. Create a point cloud with intensity
  PointCloud cloud("lidar");
  cloud.enableIntensity();
  for (int i = 0; i < 100; ++i) {
    cloud.add(PointXYZI{float(i), float(i) * 0.1f, std::sin(float(i)),
                        float(i) / 100.0f});
  }
  std::cout << "Created cloud with " << cloud.size() << " points.\n";

  // 2. Save/Load PCD
  try {
    io::savePCD("test.pcd", cloud);
    std::cout << "Saved test.pcd\n";

    PointCloud loaded_pcd = io::loadPCD("test.pcd");
    std::cout << "Loaded test.pcd (" << loaded_pcd.size() << " points)\n";
    std::cout << "  Point 0: " << loaded_pcd[0].transpose() << "\n";
    float diff = (cloud[0] - loaded_pcd[0]).norm();
    std::cout << "  Diff with original: " << diff << "\n";
  } catch (const std::exception& e) {
    std::cerr << "PCD error: " << e.what() << "\n";
  }

  // 3. Save/Load BIN
  try {
    io::saveBIN("test.bin", cloud);
    std::cout << "Saved test.bin\n";

    PointCloud loaded_bin = io::loadBIN("test.bin");
    std::cout << "Loaded test.bin (" << loaded_bin.size() << " points)\n";
    std::cout << "  Point 0: " << loaded_bin[0].transpose() << "\n";
    float diff = (cloud[0] - loaded_bin[0]).norm();
    std::cout << "  Diff with original: " << diff << "\n";
  } catch (const std::exception& e) {
    std::cerr << "BIN error: " << e.what() << "\n";
  }

  return 0;
}
