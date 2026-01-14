// nanoPCL Example 12: Point Cloud Registration
//
// Demonstrates 4 registration algorithms with increasing sophistication:
//   1. Point-to-Point ICP (baseline)
//   2. Point-to-Plane ICP (requires target normals)
//   3. GICP (requires covariances on both clouds)
//   4. VGICP (fastest - voxelized target)
//
// Each method builds on the previous, showing the accuracy/speed tradeoffs.

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <nanopcl/common.hpp>
#include <nanopcl/geometry/local_surface.hpp>
#include <nanopcl/registration.hpp>
#include <random>

using namespace npcl;

// Ground truth transformation for verification
Eigen::Isometry3d g_ground_truth;

// =============================================================================
// Helper: Generate target point cloud (wavy surface)
// =============================================================================
PointCloud generateTarget(size_t count) {
  PointCloud cloud("map");
  cloud.reserve(count);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> xy(-10.0f, 10.0f);
  std::normal_distribution<float> noise(0.0f, 0.02f);

  for (size_t i = 0; i < count; ++i) {
    float x = xy(gen);
    float y = xy(gen);
    // Wavy surface: z = sin(x/3) * cos(y/3) + noise
    float z = std::sin(x / 3.0f) * std::cos(y / 3.0f) + noise(gen);
    cloud.add(Point(x, y, z));
  }
  return cloud;
}

// =============================================================================
// Helper: Generate source by transforming target + adding noise
// =============================================================================
PointCloud generateSource(const PointCloud& target) {
  PointCloud source("lidar");
  source.reserve(target.size());

  // Known transformation: rotation + translation
  g_ground_truth = Eigen::Isometry3d::Identity();
  g_ground_truth.rotate(
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));  // ~5.7 degrees
  g_ground_truth.pretranslate(Eigen::Vector3d(0.5, 0.3, 0.1));

  Eigen::Isometry3d T_inv = g_ground_truth.inverse();

  std::mt19937 gen(123);
  std::normal_distribution<float> noise(0.0f, 0.01f);
  std::uniform_real_distribution<float> keep(0.0f, 1.0f);

  for (size_t i = 0; i < target.size(); ++i) {
    // Random subsampling (~80% of points)
    if (keep(gen) > 0.8f) continue;

    Eigen::Vector3d pt = target[i].cast<double>();
    Eigen::Vector3d transformed = T_inv * pt;

    // Add measurement noise
    source.add(Point(static_cast<float>(transformed.x()) + noise(gen),
                     static_cast<float>(transformed.y()) + noise(gen),
                     static_cast<float>(transformed.z()) + noise(gen)));
  }
  return source;
}

// =============================================================================
// Helper: Compute transformation error
// =============================================================================
std::pair<double, double> computeError(const Eigen::Isometry3d& estimated) {
  Eigen::Isometry3d error = g_ground_truth.inverse() * estimated;
  double trans_err = error.translation().norm();
  double rot_err =
      Eigen::AngleAxisd(error.rotation()).angle() * 180.0 / M_PI;  // degrees
  return {trans_err, rot_err};
}

// =============================================================================
// Helper: Print result row
// =============================================================================
void printResult(const std::string& name,
                 const registration::RegistrationResult& result,
                 double elapsed_ms) {
  auto [trans_err, rot_err] = computeError(result.transform);

  std::cout << "   " << std::setw(10) << std::left << name << std::right
            << " | " << std::setw(5) << result.iterations << " | "
            << std::setw(7) << std::fixed << std::setprecision(3)
            << result.fitness << " | " << std::setw(8) << std::setprecision(5)
            << result.rmse << " | " << std::setw(7) << std::setprecision(4)
            << trans_err << " | " << std::setw(7) << std::setprecision(3)
            << rot_err << " | " << std::setw(8) << std::setprecision(2)
            << elapsed_ms << " | " << (result.converged ? "Yes" : "No ")
            << "\n";
}

int main() {
  std::cout << "=== nanoPCL Registration Example ===\n\n";

  // ===========================================================================
  // 1. Generate Test Data
  // ===========================================================================
  std::cout << "1. Generating test data...\n";

  PointCloud target = generateTarget(20000);
  PointCloud source = generateSource(target);

  std::cout << "   Target: " << target.size() << " points\n";
  std::cout << "   Source: " << source.size() << " points\n";
  std::cout << "   Ground truth: t=(" << std::fixed << std::setprecision(2)
            << g_ground_truth.translation().x() << ", "
            << g_ground_truth.translation().y() << ", "
            << g_ground_truth.translation().z() << "), ";
  std::cout << "R=" << std::setprecision(1)
            << (Eigen::AngleAxisd(g_ground_truth.rotation()).angle() * 180.0 /
                M_PI)
            << " deg\n\n";

  // Results table header
  std::cout << "2. Registration Results:\n";
  std::cout << "   " << std::string(82, '-') << "\n";
  std::cout << "   " << std::setw(10) << std::left << "Method" << std::right
            << " | " << std::setw(5) << "Iters"
            << " | " << std::setw(7) << "Fitness"
            << " | " << std::setw(8) << "RMSE"
            << " | " << std::setw(7) << "T_err"
            << " | " << std::setw(7) << "R_err"
            << " | " << std::setw(8) << "Time(ms)"
            << " | Conv\n";
  std::cout << "   " << std::string(82, '-') << "\n";

  // ===========================================================================
  // 2. Point-to-Point ICP (baseline - no preprocessing required)
  // ===========================================================================
  {
    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = registration::icp(source, target);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printResult("ICP", result, ms);
  }

  // ===========================================================================
  // 3. Point-to-Plane ICP (requires normals on target)
  // ===========================================================================
  {
    // Preprocessing: estimate target normals
    geometry::estimateNormals(target, 0.5f);

    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = registration::icpPlane(source, target);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printResult("ICP-Plane", result, ms);
  }

  // ===========================================================================
  // 4. GICP (requires covariances on both clouds)
  // ===========================================================================
  {
    // Preprocessing: estimate covariances
    geometry::estimateCovariances(source, 0.5f);
    geometry::estimateCovariances(target, 0.5f);

    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = registration::gicp(source, target);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printResult("GICP", result, ms);
  }

  // ===========================================================================
  // 5. VGICP (fastest - voxelized target, reuse source covariances)
  // ===========================================================================
  {
    // Preprocessing: build voxel map (source already has covariances)
    registration::VoxelDistributionMap voxel_map(0.5f);
    voxel_map.build(target);

    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = registration::vgicp(source, voxel_map);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printResult("VGICP", result, ms);
  }

  std::cout << "   " << std::string(82, '-') << "\n";
  std::cout << "   T_err: translation error (m), R_err: rotation error (deg)\n";
  std::cout << "\n";

  // ===========================================================================
  // 6. Advanced: Custom Configuration
  // ===========================================================================
  std::cout << "3. Custom configuration example (stricter ICP):\n";
  {
    registration::ICPConfig config;
    config.max_iterations = 100;
    config.max_correspondence_dist = 0.5f;
    config.translation_threshold = 1e-8;
    config.rotation_threshold = 1e-8;

    auto result = registration::icp(source, target,
                                    Eigen::Isometry3d::Identity(), config);
    auto [trans_err, rot_err] = computeError(result.transform);

    std::cout << "   Iterations: " << result.iterations << "\n";
    std::cout << "   Fitness: " << std::setprecision(3) << result.fitness
              << "\n";
    std::cout << "   Translation error: " << std::setprecision(5) << trans_err
              << " m\n";
    std::cout << "   Rotation error: " << std::setprecision(3) << rot_err
              << " deg\n\n";
  }

  // ===========================================================================
  // 7. Result Usage
  // ===========================================================================
  std::cout << "4. Using registration result:\n";
  {
    auto result = registration::icpPlane(source, target);

    if (result.success(0.5)) {  // converged AND fitness >= 0.5
      // Apply transform to source
      Transform tf("map", "lidar", result.transform);
      PointCloud aligned = transformCloud(source, tf);

      std::cout << "   Registration successful!\n";
      std::cout << "   Aligned cloud frame: " << aligned.frameId() << "\n";
      std::cout << "   Aligned cloud size: " << aligned.size() << "\n";
    } else {
      std::cout << "   Registration failed (low fitness)\n";
    }
  }

  std::cout << "\nDone!\n";
  return 0;
}
