// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Benchmark: Point-to-Point ICP vs Point-to-Plane ICP

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include <nanopcl/common.hpp>
#include <nanopcl/geometry/normal_estimation.hpp>
#include <nanopcl/registration.hpp>

using Clock = std::chrono::high_resolution_clock;

struct BenchResult {
  double total_ms;
  double per_iter_ms;
  size_t iterations;
  double fitness;
  double rmse;
  bool converged;
};

template <typename F>
BenchResult measure(F&& func, int runs = 5) {
  auto result = func();  // Warmup

  double total = 0;
  for (int i = 0; i < runs; ++i) {
    auto start = Clock::now();
    result = func();
    auto end = Clock::now();
    total += std::chrono::duration<double, std::milli>(end - start).count();
  }

  double avg_ms = total / runs;
  return {avg_ms,
          result.iterations > 0 ? avg_ms / result.iterations : avg_ms,
          result.iterations,
          result.fitness,
          result.rmse,
          result.converged};
}

nanopcl::PointCloud generatePlaneCloud(size_t num_points, std::mt19937& gen) {
  std::uniform_real_distribution<float> xy_dist(-10.0f, 10.0f);
  std::normal_distribution<float> noise(0.0f, 0.02f);

  nanopcl::PointCloud cloud;
  cloud.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    float x = xy_dist(gen);
    float y = xy_dist(gen);
    float z = 0.1f * x + 0.05f * y + noise(gen);
    cloud.add(x, y, z);
  }

  return cloud;
}

nanopcl::PointCloud generate3DCloud(size_t num_points, std::mt19937& gen) {
  std::uniform_real_distribution<float> theta_dist(0.0f, 2.0f * M_PI);
  std::uniform_real_distribution<float> phi_dist(0.0f, M_PI);
  std::uniform_real_distribution<float> r_dist(4.0f, 6.0f);
  std::normal_distribution<float> noise(0.0f, 0.05f);

  nanopcl::PointCloud cloud;
  cloud.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    float theta = theta_dist(gen);
    float phi = phi_dist(gen);
    float r = r_dist(gen);

    float x = r * std::sin(phi) * std::cos(theta) + noise(gen);
    float y = r * std::sin(phi) * std::sin(theta) + noise(gen);
    float z = r * std::cos(phi) + noise(gen);
    cloud.add(x, y, z);
  }

  return cloud;
}

enum class CloudType { PLANAR, SPHERICAL };

void run_benchmark(size_t num_points, CloudType cloud_type = CloudType::SPHERICAL) {
  std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  Point Cloud Size: " << num_points;
  std::cout << " [" << (cloud_type == CloudType::PLANAR ? "PLANAR" : "3D SPHERICAL") << "]\n";
  std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

  std::mt19937 gen(42);

  // Known transform
  Eigen::Isometry3d true_T = Eigen::Isometry3d::Identity();
  true_T.translation() << 0.3, 0.2, 0.1;
  Eigen::AngleAxisd rot(0.05, Eigen::Vector3d::UnitZ());
  true_T.linear() = rot.toRotationMatrix();

  // Generate target
  auto target = (cloud_type == CloudType::PLANAR)
                    ? generatePlaneCloud(num_points, gen)
                    : generate3DCloud(num_points, gen);

  // Generate source (transformed target)
  nanopcl::PointCloud source;
  source.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector3f pt = (true_T * target[i].head<3>().cast<double>()).cast<float>();
    source.add(pt.x(), pt.y(), pt.z());
  }

  // Estimate normals for P2Plane
  auto target_with_normals = target;
  std::cout << "\n  Estimating normals... " << std::flush;
  auto norm_start = Clock::now();
  nanopcl::geometry::estimateNormals(target_with_normals, 0.5f);
  auto norm_end = Clock::now();
  double norm_ms = std::chrono::duration<double, std::milli>(norm_end - norm_start).count();
  std::cout << "done (" << std::fixed << std::setprecision(2) << norm_ms << " ms)\n";

  // Config
  nanopcl::registration::AlignSettings settings;
  settings.max_iterations = 50;
  settings.max_correspondence_dist = 1.0f;

  // --------------------------------------------------------------------------
  // Point-to-Point ICP
  // --------------------------------------------------------------------------
  std::cout << "\n  Running Point-to-Point ICP... " << std::flush;
  auto p2p_result = measure(
      [&]() {
        return nanopcl::registration::alignICP(
            source, target, Eigen::Isometry3d::Identity(), settings);
      },
      5);
  std::cout << "done\n";

  // --------------------------------------------------------------------------
  // Point-to-Plane ICP
  // --------------------------------------------------------------------------
  std::cout << "  Running Point-to-Plane ICP... " << std::flush;
  auto p2plane_result = measure(
      [&]() {
        return nanopcl::registration::alignPlaneICP(
            source, target_with_normals, Eigen::Isometry3d::Identity(), settings);
      },
      5);
  std::cout << "done\n";

  // --------------------------------------------------------------------------
  // Results
  // --------------------------------------------------------------------------
  std::cout << std::fixed << std::setprecision(3);

  std::cout << "\n  ┌─────────────────┬────────────────┬────────────────┐\n";
  std::cout << "  │     Metric      │  Point-to-Point │  Point-to-Plane │\n";
  std::cout << "  ├─────────────────┼────────────────┼────────────────┤\n";

  std::cout << "  │ Total time (ms) │ " << std::setw(14) << p2p_result.total_ms
            << " │ " << std::setw(14) << p2plane_result.total_ms << " │\n";

  std::cout << "  │ Per-iter (ms)   │ " << std::setw(14) << p2p_result.per_iter_ms
            << " │ " << std::setw(14) << p2plane_result.per_iter_ms << " │\n";

  std::cout << "  │ Iterations      │ " << std::setw(14) << p2p_result.iterations
            << " │ " << std::setw(14) << p2plane_result.iterations << " │\n";

  std::cout << "  │ Fitness         │ " << std::setw(14) << p2p_result.fitness
            << " │ " << std::setw(14) << p2plane_result.fitness << " │\n";

  std::cout << "  │ RMSE            │ " << std::setw(14) << p2p_result.rmse
            << " │ " << std::setw(14) << p2plane_result.rmse << " │\n";

  std::cout << "  │ Converged       │ " << std::setw(14)
            << (p2p_result.converged ? "yes" : "no") << " │ " << std::setw(14)
            << (p2plane_result.converged ? "yes" : "no") << " │\n";

  std::cout << "  └─────────────────┴────────────────┴────────────────┘\n";

  // Speedup analysis
  double speedup = p2p_result.per_iter_ms / p2plane_result.per_iter_ms;
  std::cout << "\n  Per-iteration speedup: ";
  if (speedup > 1.0) {
    std::cout << speedup << "x (P2Plane faster per iteration)\n";
  } else {
    std::cout << 1.0 / speedup << "x (P2P faster per iteration)\n";
  }

  double total_speedup = p2p_result.total_ms / p2plane_result.total_ms;
  std::cout << "  Total speedup: ";
  if (total_speedup > 1.0) {
    std::cout << total_speedup << "x (P2Plane faster overall)\n";
  } else {
    std::cout << 1.0 / total_speedup << "x (P2P faster overall)\n";
  }
}

int main(int argc, char** argv) {
  std::cout << "╔═══════════════════════════════════════════════════════════════════════════╗\n";
  std::cout << "║         ICP Benchmark: Point-to-Point vs Point-to-Plane                   ║\n";
  std::cout << "╚═══════════════════════════════════════════════════════════════════════════╝\n";

#ifdef _OPENMP
  std::cout << "\n  [OpenMP ENABLED]\n";
#else
  std::cout << "\n  [OpenMP DISABLED]\n";
#endif

  std::vector<size_t> sizes = {25000, 50000};

  if (argc > 1) {
    sizes.clear();
    for (int i = 1; i < argc; ++i) {
      sizes.push_back(std::stoul(argv[i]));
    }
  }

  // Test 1: 3D spherical structure
  std::cout << "\n\n══════════════════════════════════════════════════════════════════════════\n";
  std::cout << "  TEST 1: 3D SPHERICAL STRUCTURE\n";
  std::cout << "══════════════════════════════════════════════════════════════════════════\n";
  for (size_t n : sizes) {
    run_benchmark(n, CloudType::SPHERICAL);
  }

  // Test 2: Planar structure
  std::cout << "\n\n══════════════════════════════════════════════════════════════════════════\n";
  std::cout << "  TEST 2: PLANAR STRUCTURE (P2Plane advantage)\n";
  std::cout << "══════════════════════════════════════════════════════════════════════════\n";
  for (size_t n : sizes) {
    run_benchmark(n, CloudType::PLANAR);
  }

  std::cout << "\n═══════════════════════════════════════════════════════════════════════════\n";

  return 0;
}
