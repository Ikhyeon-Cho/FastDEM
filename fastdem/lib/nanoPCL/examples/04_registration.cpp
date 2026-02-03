// nanoPCL Example 04: Registration
//
// Point cloud alignment using ICP variants with KITTI data.

#include <iomanip>
#include <iostream>

#include <nanopcl/core.hpp>
#include "fastdem/point_types.hpp"
#include <nanopcl/filters.hpp>
#include <nanopcl/geometry.hpp>
#include <nanopcl/io.hpp>
#include <nanopcl/registration.hpp>

using namespace nanopcl;

int main() {
  std::cout << "=== nanoPCL Registration ===\n\n";
  std::cout << std::fixed << std::setprecision(2);

  // Load and downsample KITTI data
  std::string path = std::string(NANOPCL_DATA_DIR) + "/kitti/000000.bin";
  auto raw = io::loadKITTI(path);
  auto target = filters::voxelGrid(raw, 0.3f);
  std::cout << "[Target] " << target.size() << " points\n";

  // Create source by applying a transformation to target
  Eigen::Isometry3d T_source = Eigen::Isometry3d::Identity();
  T_source.translate(Eigen::Vector3d(0.5, 0.3, 0.1));
  T_source.rotate(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

  auto source = transformCloud(target, T_source);
  std::cout << "[Source] " << source.size() << " points\n";

  // ICP finds T such that T * source = target, so T = T_source^(-1)
  Eigen::Isometry3d T_gt = T_source.inverse();
  std::cout << "[Ground Truth] t=(" << T_gt.translation().transpose() << ")\n\n";

  // 1. Point-to-Point ICP
  std::cout << "[1] Point-to-Point ICP\n";
  auto result1 = registration::alignICP(source, target);
  std::cout << "    Converged: " << (result1.converged ? "yes" : "no") << "\n";
  std::cout << "    Iterations: " << result1.iterations << "\n";
  std::cout << "    Translation: (" << result1.transform.translation().transpose() << ")\n\n";

  // 2. Point-to-Plane ICP (requires normals)
  std::cout << "[2] Point-to-Plane ICP\n";
  geometry::estimateNormals(target, 1.0f);
  auto result2 = registration::alignPlaneICP(source, target);
  std::cout << "    Converged: " << (result2.converged ? "yes" : "no") << "\n";
  std::cout << "    Iterations: " << result2.iterations << "\n";
  std::cout << "    Translation: (" << result2.transform.translation().transpose() << ")\n\n";

  // 3. GICP (requires covariances)
  std::cout << "[3] Generalized ICP (GICP)\n";
  geometry::estimateCovariances(source, 1.0f);
  geometry::estimateCovariances(target, 1.0f);
  auto result3 = registration::alignGICP(source, target);
  std::cout << "    Converged: " << (result3.converged ? "yes" : "no") << "\n";
  std::cout << "    Iterations: " << result3.iterations << "\n";
  std::cout << "    Translation: (" << result3.transform.translation().transpose() << ")\n";

  return 0;
}
