// nanoPCL Transform Implementation Benchmark
// Compares different transform strategies for Point4 (SoA) layout
//
// Point4 = (x, y, z, 1) - homogeneous coordinates
// Transform: p' = R * p + t (applied to xyz only)

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nanopcl/core/point_cloud.hpp>

using namespace nanopcl;

// =============================================================================
// Timer utility
// =============================================================================
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()) {}

  double elapsed_ms() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }
};

template <typename T>
void doNotOptimize(const T& value) {
  asm volatile(""
               :
               : "g"(&value)
               : "memory");
}

void printHeader(const std::string& title) {
  std::cout << "\n" << std::string(70, '=') << "\n";
  std::cout << title << "\n";
  std::cout << std::string(70, '=') << "\n";
}

void printSubHeader(const std::string& title) {
  std::cout << "\n--- " << title << " ---\n";
}

// =============================================================================
// Test cloud generator
// =============================================================================
PointCloud generateCloud(size_t num_points, bool with_normals = false, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> norm_dist(-1.0f, 1.0f);

  PointCloud cloud;
  cloud.reserve(num_points);
  cloud.useIntensity();

  if (with_normals) {
    cloud.useNormal();
  }

  for (size_t i = 0; i < num_points; ++i) {
    float x = pos_dist(gen), y = pos_dist(gen), z = pos_dist(gen);
    cloud.add(x, y, z, Intensity(0.5f));
    if (with_normals) {
      Eigen::Vector3f n(norm_dist(gen), norm_dist(gen), norm_dist(gen));
      cloud.normal(cloud.size() - 1) = n.normalized();
    }
  }

  return cloud;
}

Eigen::Matrix3f generateRotation(unsigned seed = 123) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> angle_dist(-M_PI, M_PI);

  float roll = angle_dist(gen);
  float pitch = angle_dist(gen);
  float yaw = angle_dist(gen);

  Eigen::AngleAxisf r(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf p(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf y(yaw, Eigen::Vector3f::UnitZ());

  return (y * p * r).toRotationMatrix();
}

Eigen::Vector3f generateTranslation(unsigned seed = 456) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  return Eigen::Vector3f(dist(gen), dist(gen), dist(gen));
}

// =============================================================================
// Transform implementations (Point4 compatible)
// =============================================================================

// --- Implementation 1: Per-point loop using point(i) accessor ---
void transformImpl1_PointAccessor(PointCloud& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  for (size_t i = 0; i < cloud.size(); ++i) {
    Eigen::Vector3f p = cloud.point(i);  // Copy xyz
    cloud.point(i) = R * p + t;          // Write back
  }

  if (cloud.hasNormal()) {
    for (size_t i = 0; i < cloud.size(); ++i) {
      Eigen::Vector3f n = cloud.normal(i);
      cloud.normal(i) = R * n;
    }
  }
}

// --- Implementation 2: Direct Point4 manipulation with head<3>() ---
void transformImpl2_Head3Direct(PointCloud& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  auto& pts = cloud.points();
  for (size_t i = 0; i < pts.size(); ++i) {
    pts[i].head<3>() = R * pts[i].head<3>() + t;
  }

  if (cloud.hasNormal()) {
    auto& norms = cloud.normals();
    for (size_t i = 0; i < norms.size(); ++i) {
      norms[i].head<3>() = R * norms[i].head<3>();
    }
  }
}

// --- Implementation 3: 4x4 homogeneous matrix (single multiply) ---
void transformImpl3_Homogeneous4x4(PointCloud& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  // Build 4x4 homogeneous transform
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;

  auto& pts = cloud.points();
  for (size_t i = 0; i < pts.size(); ++i) {
    pts[i] = T * pts[i];  // 4x4 * 4x1 = 4x1
  }

  if (cloud.hasNormal()) {
    // Normals: only rotation, no translation
    Eigen::Matrix4f Rn = Eigen::Matrix4f::Identity();
    Rn.block<3, 3>(0, 0) = R;

    auto& norms = cloud.normals();
    for (size_t i = 0; i < norms.size(); ++i) {
      norms[i] = Rn * norms[i];
    }
  }
}

// --- Implementation 4: Pointer-based with cached data ---
void transformImpl4_PointerCached(PointCloud& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  Point4* pts = cloud.points().data();
  const size_t n = cloud.size();

  for (size_t i = 0; i < n; ++i) {
    pts[i].head<3>() = R * pts[i].head<3>() + t;
  }

  if (cloud.hasNormal()) {
    Normal4* norms = cloud.normals().data();
    for (size_t i = 0; i < n; ++i) {
      norms[i].head<3>() = R * norms[i].head<3>();
    }
  }
}

// --- Implementation 5: Eigen::Map batch on 4xN matrix ---
void transformImpl5_EigenMapBatch(PointCloud& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  if (cloud.empty())
    return;

  const Eigen::Index n = static_cast<Eigen::Index>(cloud.size());

  // Map Point4 array as 4xN matrix (column-major: each column is a point)
  Eigen::Map<Eigen::Matrix4Xf> points(cloud.points().data()->data(), 4, n);

  // Build 4x4 homogeneous transform
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;

  // Batch transform: P' = T * P
  points = T * points;

  if (cloud.hasNormal()) {
    Eigen::Map<Eigen::Matrix4Xf> normals(cloud.normals().data()->data(), 4, n);
    Eigen::Matrix4f Rn = Eigen::Matrix4f::Identity();
    Rn.block<3, 3>(0, 0) = R;
    normals = Rn * normals;
  }
}

// =============================================================================
// Benchmark runner
// =============================================================================

struct BenchmarkResult {
  std::string name;
  double avg_ms;
  double min_ms;
  double max_ms;
  double throughput_mpts;
};

template <typename Func>
BenchmarkResult runBenchmark(const std::string& name, Func&& func,
                             const PointCloud& original,
                             const Eigen::Matrix3f& R, const Eigen::Vector3f& t,
                             int iterations = 100, int warmup = 10) {
  // Warmup
  for (int i = 0; i < warmup; ++i) {
    PointCloud cloud = original;
    func(cloud, R, t);
    doNotOptimize(cloud);
  }

  // Benchmark
  std::vector<double> times;
  times.reserve(iterations);

  for (int i = 0; i < iterations; ++i) {
    PointCloud cloud = original;
    Timer timer;
    func(cloud, R, t);
    times.push_back(timer.elapsed_ms());
    doNotOptimize(cloud);
  }

  double sum = 0, min_t = times[0], max_t = times[0];
  for (double t : times) {
    sum += t;
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
  }
  double avg = sum / iterations;
  double throughput = (original.size() / 1e6) / (avg / 1000.0);

  return {name, avg, min_t, max_t, throughput};
}

void printResults(const std::vector<BenchmarkResult>& results) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << std::left << std::setw(35) << "Implementation" << std::right
            << std::setw(12) << "Avg (ms)" << std::setw(12) << "Min (ms)"
            << std::setw(12) << "Max (ms)" << std::setw(15) << "Mpts/sec"
            << "\n";
  std::cout << std::string(86, '-') << "\n";

  double baseline = results[0].avg_ms;
  for (const auto& r : results) {
    double speedup = baseline / r.avg_ms;
    std::cout << std::left << std::setw(35) << r.name << std::right
              << std::setw(12) << r.avg_ms << std::setw(12) << r.min_ms
              << std::setw(12) << r.max_ms << std::setw(12) << r.throughput_mpts
              << "  (" << std::setprecision(2) << speedup << "x)\n";
    std::cout << std::setprecision(3);
  }
}

// =============================================================================
// Correctness verification
// =============================================================================
bool verifyCorrectness(const PointCloud& original, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  constexpr float EPSILON = 1e-4f;

  PointCloud cloud1 = original;
  PointCloud cloud2 = original;
  PointCloud cloud3 = original;
  PointCloud cloud4 = original;
  PointCloud cloud5 = original;

  transformImpl1_PointAccessor(cloud1, R, t);
  transformImpl2_Head3Direct(cloud2, R, t);
  transformImpl3_Homogeneous4x4(cloud3, R, t);
  transformImpl4_PointerCached(cloud4, R, t);
  transformImpl5_EigenMapBatch(cloud5, R, t);

  for (size_t i = 0; i < original.size(); ++i) {
    Eigen::Vector3f p1 = cloud1.point(i);
    Eigen::Vector3f p2 = cloud2.point(i);
    Eigen::Vector3f p3 = cloud3.point(i);
    Eigen::Vector3f p4 = cloud4.point(i);
    Eigen::Vector3f p5 = cloud5.point(i);

    if ((p1 - p2).norm() > EPSILON || (p1 - p3).norm() > EPSILON ||
        (p1 - p4).norm() > EPSILON || (p1 - p5).norm() > EPSILON) {
      std::cerr << "ERROR: Results differ at index " << i << "\n";
      std::cerr << "  Impl1: " << p1.transpose() << "\n";
      std::cerr << "  Impl2: " << p2.transpose() << "\n";
      std::cerr << "  Impl3: " << p3.transpose() << "\n";
      std::cerr << "  Impl4: " << p4.transpose() << "\n";
      std::cerr << "  Impl5: " << p5.transpose() << "\n";
      return false;
    }
  }

  std::cout << "Correctness verification: PASSED\n";
  return true;
}

// =============================================================================
// Main
// =============================================================================
int main() {
  std::cout << "nanoPCL Transform Benchmark (Point4 API)\n";
  std::cout << "Comparing transform implementation strategies\n";
  std::cout << "Compiler: " << __VERSION__ << "\n";
#ifdef __AVX__
  std::cout << "SIMD: AVX enabled\n";
#elif defined(__SSE4_2__)
  std::cout << "SIMD: SSE4.2 enabled\n";
#else
  std::cout << "SIMD: Basic\n";
#endif
  std::cout << "sizeof(Point4) = " << sizeof(Point4) << " bytes\n";

  const Eigen::Matrix3f R = generateRotation();
  const Eigen::Vector3f t = generateTranslation();

  std::vector<size_t> sizes = {10000, 50000, 100000, 500000};

  // =========================================================================
  // Benchmark: XYZ only
  // =========================================================================
  printHeader("Transform Benchmark: XYZ Only");

  for (size_t num_points : sizes) {
    printSubHeader("Cloud size: " + std::to_string(num_points) + " points");

    PointCloud cloud = generateCloud(num_points, false);

    if (!verifyCorrectness(cloud, R, t)) {
      return 1;
    }

    std::vector<BenchmarkResult> results;

    results.push_back(runBenchmark("1. point(i) accessor", transformImpl1_PointAccessor, cloud, R, t));
    results.push_back(runBenchmark("2. head<3>() direct", transformImpl2_Head3Direct, cloud, R, t));
    results.push_back(runBenchmark("3. Homogeneous 4x4", transformImpl3_Homogeneous4x4, cloud, R, t));
    results.push_back(runBenchmark("4. Pointer cached", transformImpl4_PointerCached, cloud, R, t));
    results.push_back(runBenchmark("5. Eigen::Map 4xN batch", transformImpl5_EigenMapBatch, cloud, R, t));

    printResults(results);
  }

  // =========================================================================
  // Benchmark: XYZ + Normals
  // =========================================================================
  printHeader("Transform Benchmark: XYZ + Normals");

  for (size_t num_points : sizes) {
    printSubHeader("Cloud size: " + std::to_string(num_points) + " points");

    PointCloud cloud = generateCloud(num_points, true);

    std::vector<BenchmarkResult> results;

    results.push_back(runBenchmark("1. point(i) accessor", transformImpl1_PointAccessor, cloud, R, t));
    results.push_back(runBenchmark("2. head<3>() direct", transformImpl2_Head3Direct, cloud, R, t));
    results.push_back(runBenchmark("3. Homogeneous 4x4", transformImpl3_Homogeneous4x4, cloud, R, t));
    results.push_back(runBenchmark("4. Pointer cached", transformImpl4_PointerCached, cloud, R, t));
    results.push_back(runBenchmark("5. Eigen::Map 4xN batch", transformImpl5_EigenMapBatch, cloud, R, t));

    printResults(results);
  }

  // =========================================================================
  // Summary
  // =========================================================================
  printHeader("Summary");
  std::cout << R"(
Implementation notes:
1. point(i) accessor  : Uses cloud.point(i) which returns head<3>() view
2. head<3>() direct   : Directly manipulates Point4.head<3>()
3. Homogeneous 4x4    : Uses 4x4 matrix multiply (exploits w=1)
4. Pointer cached     : Caches data pointer, uses head<3>()
5. Eigen::Map batch   : Maps to 4xN matrix, single batch GEMM

Key observations:
- Point4 layout allows efficient 4x4 homogeneous transforms
- Batch operations may benefit from SIMD when data is contiguous
- head<3>() creates a view (no copy) but requires per-element R*p+t
)";

  return 0;
}
