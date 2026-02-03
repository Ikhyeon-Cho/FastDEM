// nanoPCL vs small_gicp Benchmark: Normal & Covariance Estimation
// Fair comparison: Both use OpenMP with same thread count
//
// Build:
//   g++ -O3 -march=native -fopenmp -I../../include \
//       -I/home/ikhyeon/ros/test_ws/src/small_gicp/include \
//       bench_normal_estimation.cpp -o bench_normal_estimation

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

// nanoPCL
#include <nanopcl/geometry/normal_estimation.hpp>

// small_gicp (OMP version for fair comparison)
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Helpers
// =============================================================================

nanopcl::PointCloud generateNanoPCL(size_t n, unsigned seed = 42) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-50.0f, 50.0f);

  nanopcl::PointCloud cloud;
  cloud.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    float x = dist(rng);
    float y = dist(rng);
    // Create a noisy plane: z = 0.1*x + 0.2*y + noise
    float z = 0.1f * x + 0.2f * y + dist(rng) * 0.01f;
    cloud.add(x, y, z);
  }
  return cloud;
}

small_gicp::PointCloud generateSmallGicp(size_t n, unsigned seed = 42) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-50.0, 50.0);

  small_gicp::PointCloud cloud;
  cloud.points.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double z = 0.1 * x + 0.2 * y + dist(rng) * 0.01;
    cloud.points.emplace_back(x, y, z, 1.0);
  }
  return cloud;
}

template <typename Func>
double benchmark(Func&& func, int warmup = 1, int iterations = 5) {
  for (int i = 0; i < warmup; ++i) {
    func();
  }

  double total_ms = 0;
  for (int i = 0; i < iterations; ++i) {
    auto start = Clock::now();
    func();
    auto end = Clock::now();
    total_ms += std::chrono::duration<double, std::milli>(end - start).count();
  }
  return total_ms / iterations;
}

// =============================================================================
// Benchmarks
// =============================================================================

void benchmarkNormalEstimation(size_t n, int k, int num_threads) {
  std::cout << "\n=== Normal Estimation (OpenMP) ===" << std::endl;
  std::cout << "Points: " << n << ", k: " << k << ", threads: " << num_threads
            << "\n"
            << std::endl;

  // nanoPCL (uses OpenMP automatically when compiled with -fopenmp)
  double time_npcl;
  {
    auto cloud = generateNanoPCL(n, 42);
    time_npcl =
        benchmark([&]() { nanopcl::geometry::estimateNormals(cloud, k); });
    std::cout << "  nanoPCL:     " << std::setw(8) << std::fixed
              << std::setprecision(2) << time_npcl << " ms\n";
  }

  // small_gicp OMP
  double time_sg;
  {
    auto cloud = generateSmallGicp(n, 42);
    time_sg = benchmark(
        [&]() { small_gicp::estimate_normals_omp(cloud, k, num_threads); });
    std::cout << "  small_gicp:  " << std::setw(8) << time_sg << " ms\n";
  }

  std::cout << "  Ratio:       " << std::setw(8) << std::setprecision(2)
            << (time_sg / time_npcl) << "x\n";
}

void benchmarkCovarianceEstimation(size_t n, int k, int num_threads) {
  std::cout << "\n=== Covariance Estimation (OpenMP) ===" << std::endl;
  std::cout << "Points: " << n << ", k: " << k << ", threads: " << num_threads
            << "\n"
            << std::endl;

  double time_npcl;
  {
    auto cloud = generateNanoPCL(n, 42);
    time_npcl =
        benchmark([&]() { nanopcl::geometry::estimateCovariances(cloud, k); });
    std::cout << "  nanoPCL:     " << std::setw(8) << std::fixed
              << std::setprecision(2) << time_npcl << " ms\n";
  }

  double time_sg;
  {
    auto cloud = generateSmallGicp(n, 42);
    time_sg = benchmark(
        [&]() { small_gicp::estimate_covariances_omp(cloud, k, num_threads); });
    std::cout << "  small_gicp:  " << std::setw(8) << time_sg << " ms\n";
  }

  std::cout << "  Ratio:       " << std::setw(8) << std::setprecision(2)
            << (time_sg / time_npcl) << "x\n";
}

void benchmarkCombinedEstimation(size_t n, int k, int num_threads) {
  std::cout << "\n=== Normal + Covariance Combined (OpenMP) ===" << std::endl;
  std::cout << "Points: " << n << ", k: " << k << ", threads: " << num_threads
            << "\n"
            << std::endl;

  double time_npcl;
  {
    auto cloud = generateNanoPCL(n, 42);
    time_npcl = benchmark(
        [&]() { nanopcl::geometry::estimateNormalsCovariances(cloud, k); });
    std::cout << "  nanoPCL:     " << std::setw(8) << std::fixed
              << std::setprecision(2) << time_npcl << " ms\n";
  }

  double time_sg;
  {
    auto cloud = generateSmallGicp(n, 42);
    time_sg = benchmark([&]() {
      small_gicp::estimate_normals_covariances_omp(cloud, k, num_threads);
    });
    std::cout << "  small_gicp:  " << std::setw(8) << time_sg << " ms\n";
  }

  std::cout << "  Ratio:       " << std::setw(8) << std::setprecision(2)
            << (time_sg / time_npcl) << "x\n";
}

void benchmarkScaling(int k, int num_threads) {
  std::cout << "\n=== Scaling Test (Normal+Cov, OpenMP) ===" << std::endl;
  std::cout << "k: " << k << ", threads: " << num_threads << "\n" << std::endl;

  std::cout << std::setw(10) << "Points" << std::setw(12) << "nanoPCL"
            << std::setw(12) << "small_gicp" << std::setw(10) << "Ratio"
            << std::endl;
  std::cout << std::string(44, '-') << std::endl;

  for (size_t pts : {10000, 50000, 100000, 200000}) {
    auto cloud_npcl = generateNanoPCL(pts, 42);
    auto cloud_sg = generateSmallGicp(pts, 42);

    double t_npcl = benchmark(
        [&]() { nanopcl::geometry::estimateNormalsCovariances(cloud_npcl, k); }, 1,
        3);
    double t_sg = benchmark(
        [&]() {
          small_gicp::estimate_normals_covariances_omp(cloud_sg, k,
                                                       num_threads);
        },
        1, 3);

    std::cout << std::setw(10) << pts << std::setw(10) << std::fixed
              << std::setprecision(1) << t_npcl << "ms" << std::setw(10) << t_sg
              << "ms" << std::setw(10) << std::setprecision(2)
              << (t_sg / t_npcl) << "x" << std::endl;
  }
}

void benchmarkSingleThread(size_t n, int k) {
  std::cout << "\n=== Single-threaded Comparison ===" << std::endl;
  std::cout << "Points: " << n << ", k: " << k << "\n" << std::endl;

#ifdef _OPENMP
  int original_threads = omp_get_max_threads();
  omp_set_num_threads(1);
#endif

  double time_npcl;
  {
    auto cloud = generateNanoPCL(n, 42);
    time_npcl = benchmark(
        [&]() { nanopcl::geometry::estimateNormalsCovariances(cloud, k); });
    std::cout << "  nanoPCL:     " << std::setw(8) << std::fixed
              << std::setprecision(2) << time_npcl << " ms\n";
  }

  double time_sg;
  {
    auto cloud = generateSmallGicp(n, 42);
    time_sg = benchmark([&]() {
      small_gicp::estimate_normals_covariances_omp(cloud, k, 1);
    });
    std::cout << "  small_gicp:  " << std::setw(8) << time_sg << " ms\n";
  }

  std::cout << "  Ratio:       " << std::setw(8) << std::setprecision(2)
            << (time_sg / time_npcl) << "x\n";

#ifdef _OPENMP
  omp_set_num_threads(original_threads);
#endif
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
  size_t n = 100000;
  int k = 20;
  int num_threads = 4;

  if (argc > 1)
    n = std::stoull(argv[1]);
  if (argc > 2)
    k = std::stoi(argv[2]);
  if (argc > 3)
    num_threads = std::stoi(argv[3]);

#ifdef _OPENMP
  omp_set_num_threads(num_threads);
  std::cout << "OpenMP enabled, max threads: " << omp_get_max_threads()
            << std::endl;
#else
  std::cout << "OpenMP NOT enabled" << std::endl;
#endif

  std::cout << "================================================" << std::endl;
  std::cout << "  nanoPCL vs small_gicp: Normal/Cov Estimation" << std::endl;
  std::cout << "  (Fair comparison: Both using OpenMP)" << std::endl;
  std::cout << "================================================" << std::endl;

  benchmarkNormalEstimation(n, k, num_threads);
  benchmarkCovarianceEstimation(n, k, num_threads);
  benchmarkCombinedEstimation(n, k, num_threads);
  benchmarkScaling(k, num_threads);
  benchmarkSingleThread(n, k);

  return 0;
}
