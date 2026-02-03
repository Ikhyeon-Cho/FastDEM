// nanoPCL Rerun Example 04: Registration Visualization
//
// Rerun version of examples/04_registration.cpp
// Visualizes ICP alignment with KITTI data.
//
// Build: cmake -DNANOPCL_BUILD_EXAMPLES=ON -DNANOPCL_USE_RERUN=ON ..
// Run:   RERUN_SERVE=1 ./rerun_04_registration

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include <nanopcl/bridge/rerun.hpp>
#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters.hpp>
#include <nanopcl/geometry.hpp>
#include <nanopcl/io.hpp>
#include <nanopcl/registration.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Registration (Rerun) ===\n\n";
  std::cout << std::fixed << std::setprecision(4);

  // Load and downsample KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto raw = io::loadKITTI(path);
  auto target = filters::voxelGrid(raw, 0.3f);
  std::cout << "[Target] " << target.size() << " points\n";

  // Create source by transforming target
  Eigen::Isometry3d T_true = Eigen::Isometry3d::Identity();
  T_true.translate(Eigen::Vector3d(0.5, 0.3, 0.1));
  T_true.rotate(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));

  auto source = transformCloud(target, T_true);
  std::cout << "[Source] " << source.size() << " points (transformed)\n";
  std::cout << "[Ground Truth] t=(" << T_true.translation().transpose() << ")\n\n";

  // Show initial state
  rr::showCloud("registration/target", target, rerun::Color(100, 100, 255));
  rr::showCloud("registration/source_initial", source, rerun::Color(255, 100, 100));

  // Run GICP
  geometry::estimateNormals(target, 0.5f);
  geometry::estimateCovariances(source, 0.5f);
  geometry::estimateCovariances(target, 0.5f);

  auto result = registration::alignGICP(source, target);

  std::cout << "[GICP Result]\n";
  std::cout << "  Converged: " << (result.converged ? "yes" : "no") << "\n";
  std::cout << "  Iterations: " << result.iterations << "\n";
  std::cout << "  Translation: (" << result.transform.translation().transpose() << ")\n\n";

  // Transform source with result
  auto aligned = transformCloud(source, result.transform);

  // Visualize result
  std::cout << "[Rerun] Visualizing alignment result...\n";
  rr::showCloud("registration/source_aligned", aligned, rerun::Color(100, 255, 100));

  std::cout << "\nPress Ctrl+C to exit...\n";
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
