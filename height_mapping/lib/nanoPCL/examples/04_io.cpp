// nanoPCL I/O Example
#include <nanopcl/nanopcl.hpp>
#include <iostream>
#include <vector>

int main() {
  using namespace nanopcl;

  // 1. Create a point cloud with intensity
  PointCloud cloud("lidar");
  cloud.enableIntensity();

  for (int i = 0; i < 100; ++i) {
    cloud.push_back(Point(float(i), float(i) * 0.1f, std::sin(float(i))),
                    Intensity(float(i) / 100.0f));
  }
  std::cout << "Created cloud with " << cloud.size() << " points.\n";

  // 2. Save/Load PCD
  if (io::savePCD("test.pcd", cloud)) {
    std::cout << "Saved test.pcd\n";
  } else {
    std::cerr << "Failed to save test.pcd\n";
  }

  PointCloud loaded_pcd;
  if (io::loadPCD("test.pcd", loaded_pcd)) {
    std::cout << "Loaded test.pcd (" << loaded_pcd.size() << " points)\n";
    std::cout << "  Point 0: " << loaded_pcd[0].transpose() << "\n";
  } else {
    std::cerr << "Failed to load test.pcd\n";
  }

  // 3. Save/Load BIN
  if (io::saveBIN("test.bin", cloud)) {
    std::cout << "Saved test.bin\n";
  } else {
    std::cerr << "Failed to save test.bin\n";
  }

  PointCloud loaded_bin;
  if (io::loadBIN("test.bin", loaded_bin)) {
    std::cout << "Loaded test.bin (" << loaded_bin.size() << " points)\n";
    std::cout << "  Point 0: " << loaded_bin[0].transpose() << "\n";
    // Check approximate equality (float precision)
    float diff = (cloud[0] - loaded_bin[0]).norm();
    std::cout << "  Diff with original: " << diff << "\n";
  } else {
    std::cerr << "Failed to load test.bin\n";
  }

  return 0;
}
