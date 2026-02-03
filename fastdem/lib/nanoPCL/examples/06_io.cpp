// nanoPCL Example 06: File I/O
//
// Load and save point clouds in PCD and BIN formats.

#include <filesystem>
#include <iostream>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/io.hpp>

namespace fs = std::filesystem;
using namespace nanopcl;

#ifndef NANOPCL_EXAMPLES_DIR
#define NANOPCL_EXAMPLES_DIR "."
#endif

int main() {
  std::cout << "=== nanoPCL I/O ===\n\n";

  // Create output directory (always under examples/results/)
  fs::path results_dir = fs::path(NANOPCL_EXAMPLES_DIR) / "results";
  fs::create_directories(results_dir);

  // Create sample cloud
  PointCloud cloud;
  for (int i = 0; i < 1000; ++i) {
    float x = (i % 10) * 0.1f;
    float y = (i / 10 % 10) * 0.1f;
    float z = (i / 100) * 0.1f;
    cloud.add(x, y, z, Intensity(float(i) * 0.001f));
  }
  std::cout << "[Created] " << cloud.size() << " points with intensity\n\n";

  // 1. Save as PCD
  std::cout << "[1] PCD Format\n";
  auto pcd_ascii = results_dir / "cloud_ascii.pcd";
  auto pcd_binary = results_dir / "cloud_binary.pcd";

  io::savePCD(pcd_ascii.string(), cloud, io::PCDFormat::ASCII);
  std::cout << "    Saved: " << pcd_ascii << " (ASCII)\n";

  io::savePCD(pcd_binary.string(), cloud, io::PCDFormat::BINARY);
  std::cout << "    Saved: " << pcd_binary << " (Binary)\n";

  // Load back
  auto loaded_pcd = io::loadPCD(pcd_binary.string());
  std::cout << "    Loaded: " << loaded_pcd.size() << " points\n";
  std::cout << "    hasIntensity: " << loaded_pcd.hasIntensity() << "\n\n";

  // 2. Save as BIN (KITTI format)
  std::cout << "[2] BIN Format (KITTI)\n";
  auto bin_file = results_dir / "cloud.bin";

  io::saveBIN(bin_file.string(), cloud);
  std::cout << "    Saved: " << bin_file << "\n";

  auto loaded_bin = io::loadBIN(bin_file.string());
  std::cout << "    Loaded: " << loaded_bin.size() << " points\n\n";

  // 3. Verify data integrity
  std::cout << "[3] Data Integrity Check\n";
  bool match = true;
  for (size_t i = 0; i < cloud.size() && match; ++i) {
    auto p1 = cloud.point(i);
    auto p2 = loaded_pcd.point(i);
    if ((p1 - p2).norm() > 1e-5f) match = false;
    if (std::abs(cloud.intensity(i) - loaded_pcd.intensity(i)) > 1e-5f) match = false;
  }
  std::cout << "    PCD round-trip: " << (match ? "PASS" : "FAIL") << "\n";

  std::cout << "\n[Output] " << results_dir << "\n";
  std::cout << "    cloud_ascii.pcd\n";
  std::cout << "    cloud_binary.pcd\n";
  std::cout << "    cloud.bin\n";

  return 0;
}
