// Benchmark: Plane distance calculation methods
//
// Compares:
//   1. Scalar operations: a*x + b*y + c*z + d
//   2. Eigen dot product: normal.dot(point) + d
//   3. Eigen Map batch: (points * normal).array() + d
//
// Build:
//   g++ -O3 -march=native -fopenmp bench_plane_distance.cpp -o bench_plane_distance
//
// Run:
//   ./bench_plane_distance

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

using Point = Eigen::Vector3f;
using Clock = std::chrono::high_resolution_clock;

// =============================================================================
// Method 1: Scalar operations (manual)
// =============================================================================
inline float distanceScalar(float x, float y, float z, float a, float b, float c, float d) {
  return a * x + b * y + c * z + d;
}

size_t countInliersScalar(const std::vector<Point>& points, float a, float b, float c, float d, float threshold) {
  size_t count = 0;
  for (const auto& p : points) {
    float dist = std::abs(a * p.x() + b * p.y() + c * p.z() + d);
    if (dist < threshold)
      ++count;
  }
  return count;
}

// =============================================================================
// Method 2: Eigen dot product (per-point)
// =============================================================================
size_t countInliersDot(const std::vector<Point>& points,
                       const Eigen::Vector3f& normal,
                       float d,
                       float threshold) {
  size_t count = 0;
  for (const auto& p : points) {
    float dist = std::abs(normal.dot(p) + d);
    if (dist < threshold)
      ++count;
  }
  return count;
}

// =============================================================================
// Method 3: Eigen dot with pointer caching
// =============================================================================
size_t countInliersDotCached(const std::vector<Point>& points,
                             const Eigen::Vector3f& normal,
                             float d,
                             float threshold) {
  size_t count = 0;
  const float nx = normal.x(), ny = normal.y(), nz = normal.z();
  const Point* ptr = points.data();
  const size_t n = points.size();

  for (size_t i = 0; i < n; ++i) {
    float dist = std::abs(nx * ptr[i].x() + ny * ptr[i].y() + nz * ptr[i].z() + d);
    if (dist < threshold)
      ++count;
  }
  return count;
}

// =============================================================================
// Method 4: OpenMP parallel (scalar)
// =============================================================================
size_t countInliersParallel(const std::vector<Point>& points, float a, float b, float c, float d, float threshold) {
  size_t count = 0;
  const size_t n = points.size();

#pragma omp parallel for reduction(+ \
                                   : count)
  for (size_t i = 0; i < n; ++i) {
    const auto& p = points[i];
    float dist = std::abs(a * p.x() + b * p.y() + c * p.z() + d);
    if (dist < threshold)
      ++count;
  }
  return count;
}

// =============================================================================
// Method 5: Eigen Map batch operation
// =============================================================================
size_t countInliersBatch(const std::vector<Point>& points,
                         const Eigen::Vector3f& normal,
                         float d,
                         float threshold) {
  // Map points as Nx3 matrix (row-major due to vector<Vector3f> layout)
  // Actually Vector3f is column vector, so we need to be careful
  const size_t n = points.size();

  // Compute all distances at once using Eigen operations
  Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic>> pts(
      points[0].data(), 3, n);

  // distances = normal^T * pts + d (1xN)
  Eigen::VectorXf distances = (normal.transpose() * pts).array() + d;

  return (distances.array().abs() < threshold).count();
}

// =============================================================================
// Benchmark runner
// =============================================================================
template <typename Func>
double benchmark(const std::string& name, Func func, int iterations = 100) {
  // Warmup
  for (int i = 0; i < 10; ++i)
    func();

  auto start = Clock::now();
  for (int i = 0; i < iterations; ++i) {
    func();
  }
  auto end = Clock::now();

  double ms =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() /
      1000.0 / iterations;

  std::cout << "  " << name << ": " << ms << " ms" << std::endl;
  return ms;
}

int main() {
  std::cout << "=== Plane Distance Calculation Benchmark ===" << std::endl;
  std::cout << std::endl;

  // Generate random points
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-50.0f, 50.0f);

  for (size_t N : {10000, 50000, 100000, 500000}) {
    std::cout << "N = " << N << " points:" << std::endl;

    std::vector<Point> points(N);
    for (auto& p : points) {
      p = Point(dist(rng), dist(rng), dist(rng));
    }

    // Plane: z = 0 (ground plane)
    float a = 0.0f, b = 0.0f, c = 1.0f, d = 0.0f;
    Eigen::Vector3f normal(a, b, c);
    float threshold = 0.5f;

    size_t result1 = 0, result2 = 0, result3 = 0, result4 = 0, result5 = 0;

    double t1 = benchmark("Scalar", [&]() {
      result1 = countInliersScalar(points, a, b, c, d, threshold);
    });

    double t2 = benchmark("Eigen dot", [&]() {
      result2 = countInliersDot(points, normal, d, threshold);
    });

    double t3 = benchmark("Dot+Cache", [&]() {
      result3 = countInliersDotCached(points, normal, d, threshold);
    });

    double t4 = benchmark("Parallel ", [&]() {
      result4 = countInliersParallel(points, a, b, c, d, threshold);
    });

    double t5 = benchmark("Batch Map", [&]() {
      result5 = countInliersBatch(points, normal, d, threshold);
    });

    // Verify results match
    if (result1 != result2 || result2 != result3 || result3 != result4 ||
        result4 != result5) {
      std::cerr << "ERROR: Results mismatch!" << std::endl;
      std::cerr << "  Scalar: " << result1 << ", Dot: " << result2
                << ", Cached: " << result3 << ", Parallel: " << result4
                << ", Batch: " << result5 << std::endl;
    }

    std::cout << "  -> Inliers: " << result1 << std::endl;
    std::cout << "  -> Speedup (Parallel vs Scalar): " << t1 / t4 << "x"
              << std::endl;
    std::cout << std::endl;
  }

  return 0;
}
