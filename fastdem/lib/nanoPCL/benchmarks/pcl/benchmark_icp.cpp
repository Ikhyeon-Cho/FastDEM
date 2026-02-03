// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Benchmark: ICP registration nanoPCL vs PCL
//
// Build:
//   g++ -std=c++17 -O3 -fopenmp \
//       -I ../../include -I ../../thirdparty -I /usr/include/eigen3 \
//       $(pkg-config --cflags pcl_registration-1.10) \
//       -o benchmark_icp benchmark_icp.cpp \
//       $(pkg-config --libs pcl_registration-1.10) -pthread

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

// nanoPCL
#include <nanopcl/common.hpp>
#include <nanopcl/registration.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using Clock = std::chrono::high_resolution_clock;

template <typename F>
double measure_ms(F&& func, int runs = 5) {
  func(); // Warmup
  double total = 0;
  for (int i = 0; i < runs; ++i) {
    auto start = Clock::now();
    func();
    auto end = Clock::now();
    total += std::chrono::duration<double, std::milli>(end - start).count();
  }
  return total / runs;
}

void run_benchmark(size_t num_points) {
  std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
  std::cout << "  Point Cloud Size: " << num_points << "\n";
  std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

  // Generate random 3D point cloud
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

  // Known transform to apply
  Eigen::Isometry3d true_T = Eigen::Isometry3d::Identity();
  true_T.translation() << 0.3, 0.2, 0.1;
  Eigen::AngleAxisd rot(0.05, Eigen::Vector3d::UnitZ());
  true_T.linear() = rot.toRotationMatrix();

  // Create nanoPCL clouds
  nanopcl::PointCloud nano_target, nano_source;
  nano_target.reserve(num_points);
  nano_source.reserve(num_points);
  
  for (size_t i = 0; i < num_points; ++i) {
    nanopcl::Point pt(dist(gen), dist(gen), dist(gen));
    nano_target.add(pt.x(), pt.y(), pt.z());
    
    nanopcl::Point src_pt = (true_T * pt.cast<double>()).cast<float>();
    nano_source.add(src_pt.x(), src_pt.y(), src_pt.z());
  }

  // Create PCL clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < num_points; ++i) {
    pcl_target->push_back(pcl::PointXYZ(nano_target[i].x(), nano_target[i].y(), nano_target[i].z()));
    pcl_source->push_back(pcl::PointXYZ(nano_source[i].x(), nano_source[i].y(), nano_source[i].z()));
  }

  // --------------------------------------------------------------------------
  // nanoPCL ICP
  // --------------------------------------------------------------------------
  nanopcl::registration::AlignSettings settings;
  settings.max_iterations = 50;
  settings.max_correspondence_dist = 1.0f;

  nanopcl::registration::RegistrationResult nano_res;
  double nano_icp = measure_ms([&]() {
    nano_res = nanopcl::registration::alignICP(nano_source, nano_target, Eigen::Isometry3d::Identity(), settings);
  },
                               5);

  // --------------------------------------------------------------------------
  // PCL ICP
  // --------------------------------------------------------------------------
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_icp;
  pcl_icp.setMaximumIterations(50);
  pcl_icp.setMaxCorrespondenceDistance(1.0);
  pcl_icp.setTransformationEpsilon(1e-8); // Explicit termination condition
  pcl_icp.setEuclideanFitnessEpsilon(1e-5);
  pcl_icp.setInputTarget(pcl_target);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  double pcl_icp_ms = measure_ms([&]() {
    pcl_icp.setInputSource(pcl_source);
    pcl_icp.align(*aligned);
  },
                                 5);

  // --------------------------------------------------------------------------
  // Results
  // --------------------------------------------------------------------------
  double speedup = pcl_icp_ms / nano_icp;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n  nanoPCL:\n";
  std::cout << "    ICP:           " << std::setw(10) << nano_icp << " ms";
  std::cout << " (" << nano_res.iterations << " iterations)\n";

  std::cout << "\n  PCL:\n";
  std::cout << "    ICP:           " << std::setw(10) << pcl_icp_ms << " ms\n";

  std::cout << "\n  Speedup:         " << std::setw(10) << speedup << "x";
  if (speedup > 1.0) {
    std::cout << " (nanoPCL is faster)\n";
  } else {
    std::cout << " (PCL is faster)\n";
  }
}

int main(int argc, char** argv) {
  std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
  std::cout << "║           ICP Benchmark: nanoPCL vs PCL                          ║\n";
  std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";

#ifdef _OPENMP
  std::cout << "\n  [nanoPCL compiled WITH OpenMP]\n";
#else
  std::cout << "\n  [nanoPCL compiled WITHOUT OpenMP]\n";
#endif

  std::vector<size_t> sizes = {10000, 25000, 50000, 100000};

  if (argc > 1) {
    sizes.clear();
    for (int i = 1; i < argc; ++i) {
      sizes.push_back(std::stoul(argv[i]));
    }
  }

  for (size_t n : sizes) {
    run_benchmark(n);
  }

  std::cout << "\n═══════════════════════════════════════════════════════════════════\n";

  return 0;
}
