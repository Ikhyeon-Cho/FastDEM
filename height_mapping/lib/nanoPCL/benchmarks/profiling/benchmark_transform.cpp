// nanoPCL Transform Implementation Benchmark
// Compares different transform strategies for SoA layout
//
// Implementations compared:
// 1. Per-point loop (current implementation)
// 2. Eigen::Map batch processing (proposed optimization)
// 3. Per-point loop with manual unroll hint

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nanopcl/core/point_cloud.hpp>

using namespace npcl;

// =============================================================================
// Timer utility
// =============================================================================
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

 public:
  Timer() : start_(std::chrono::high_resolution_clock::now()) {}

  double elapsed_ms() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }
};

// Prevent compiler optimization
template <typename T>
void doNotOptimize(const T& value) {
  asm volatile("" : : "g"(&value) : "memory");
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
PointCloud generateCloud(size_t num_points, bool with_normals = false,
                         unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> norm_dist(-1.0f, 1.0f);

  PointCloud cloud("test_frame");
  cloud.reserve(num_points);
  cloud.enableIntensity();

  if (with_normals) {
    cloud.enableNormal();
  }

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)),
              Intensity(0.5f));
    if (with_normals) {
      Eigen::Vector3f n(norm_dist(gen), norm_dist(gen), norm_dist(gen));
      cloud.normal().back() = n.normalized();
    }
  }

  return cloud;
}

// Generate random transform
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
// Transform implementations to benchmark
// =============================================================================

// --- Implementation 1: Per-point loop (current) ---
void transformImpl1_PerPointLoop(PointCloud& cloud, const Eigen::Matrix3f& R,
                                  const Eigen::Vector3f& t) {
  for (auto& point : cloud.xyz()) {
    point = R * point + t;
  }

  if (cloud.hasNormal()) {
    for (auto& n : cloud.normal()) {
      n = R * n;
    }
  }
}

// --- Implementation 2: Eigen::Map batch processing (proposed) ---
void transformImpl2_EigenMapBatch(PointCloud& cloud, const Eigen::Matrix3f& R,
                                   const Eigen::Vector3f& t) {
  if (cloud.empty()) return;

  static_assert(sizeof(Eigen::Vector3f) == 3 * sizeof(float),
                "Vector3f must be tightly packed for Map to work");

  const size_t n = cloud.size();

  // Zero-copy Map: vector<Vector3f> -> 3xN Matrix (ColMajor)
  Eigen::Map<Eigen::Matrix3Xf> points(cloud.xyz().data()->data(), 3, n);

  // Batch transform: P' = R*P + t
  points = (R * points).colwise() + t;

  if (cloud.hasNormal()) {
    Eigen::Map<Eigen::Matrix3Xf> normals(cloud.normal().data()->data(), 3, n);
    normals = R * normals;
  }
}

// --- Implementation 3: Index-based loop (alternative baseline) ---
void transformImpl3_IndexLoop(PointCloud& cloud, const Eigen::Matrix3f& R,
                               const Eigen::Vector3f& t) {
  auto& pts = cloud.xyz();
  const size_t n = pts.size();

  for (size_t i = 0; i < n; ++i) {
    pts[i] = R * pts[i] + t;
  }

  if (cloud.hasNormal()) {
    auto& norms = cloud.normal();
    for (size_t i = 0; i < n; ++i) {
      norms[i] = R * norms[i];
    }
  }
}

// --- Implementation 4: Eigen::Map with noalias (avoid temporary) ---
void transformImpl4_EigenMapNoAlias(PointCloud& cloud, const Eigen::Matrix3f& R,
                                     const Eigen::Vector3f& t) {
  if (cloud.empty()) return;

  static_assert(sizeof(Eigen::Vector3f) == 3 * sizeof(float),
                "Vector3f must be tightly packed for Map to work");

  const Eigen::Index n = static_cast<Eigen::Index>(cloud.size());

  Eigen::Map<Eigen::Matrix3Xf> points(cloud.xyz().data()->data(), 3, n);

  // Process column by column to avoid temporary matrix
  for (Eigen::Index i = 0; i < n; ++i) {
    points.col(i) = R * points.col(i) + t;
  }

  if (cloud.hasNormal()) {
    Eigen::Map<Eigen::Matrix3Xf> normals(cloud.normal().data()->data(), 3, n);
    for (Eigen::Index i = 0; i < n; ++i) {
      normals.col(i) = R * normals.col(i);
    }
  }
}

// --- Implementation 5: Eigen::Map batch with pre-allocated temp ---
void transformImpl5_EigenMapPrealloc(PointCloud& cloud, const Eigen::Matrix3f& R,
                                      const Eigen::Vector3f& t) {
  if (cloud.empty()) return;

  static_assert(sizeof(Eigen::Vector3f) == 3 * sizeof(float),
                "Vector3f must be tightly packed for Map to work");

  const size_t n = cloud.size();

  Eigen::Map<Eigen::Matrix3Xf> points(cloud.xyz().data()->data(), 3, n);

  // Pre-allocate temporary and use noalias
  Eigen::Matrix3Xf temp(3, n);
  temp.noalias() = R * points;
  points = temp.colwise() + t;

  if (cloud.hasNormal()) {
    Eigen::Map<Eigen::Matrix3Xf> normals(cloud.normal().data()->data(), 3, n);
    temp.noalias() = R * normals;
    normals = temp;
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
  double throughput_mpts;  // million points per second
};

// Cache flusher - evict data from cache by touching large buffer
class CacheFlusher {
  static constexpr size_t FLUSH_SIZE = 32 * 1024 * 1024;  // 32MB > L3 cache
  std::vector<char> buffer_;

 public:
  CacheFlusher() : buffer_(FLUSH_SIZE) {
    // Initialize to prevent lazy allocation
    std::fill(buffer_.begin(), buffer_.end(), 0);
  }

  void flush() {
    // Touch every cache line (64 bytes)
    volatile char sum = 0;
    for (size_t i = 0; i < buffer_.size(); i += 64) {
      buffer_[i] = static_cast<char>(i);
      sum += buffer_[i];
    }
    doNotOptimize(sum);
  }
};

// Global cache flusher
static CacheFlusher g_cache_flusher;

template <typename Func>
BenchmarkResult runBenchmark(const std::string& name, Func&& func,
                             const PointCloud& original,
                             const Eigen::Matrix3f& R,
                             const Eigen::Vector3f& t, int iterations = 100,
                             int warmup = 10, bool cold_cache = false) {
  // Warmup
  for (int i = 0; i < warmup; ++i) {
    PointCloud cloud = original;  // copy
    func(cloud, R, t);
    doNotOptimize(cloud);
  }

  // Benchmark
  std::vector<double> times;
  times.reserve(iterations);

  for (int i = 0; i < iterations; ++i) {
    PointCloud cloud = original;  // copy

    if (cold_cache) {
      g_cache_flusher.flush();  // Evict from cache
    }

    Timer timer;
    func(cloud, R, t);
    times.push_back(timer.elapsed_ms());
    doNotOptimize(cloud);
  }

  // Calculate statistics
  double sum = 0, min_t = times[0], max_t = times[0];
  for (double t : times) {
    sum += t;
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
  }
  double avg = sum / iterations;

  // Throughput: points per second
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
bool verifyCorrectness(const PointCloud& original, const Eigen::Matrix3f& R,
                       const Eigen::Vector3f& t) {
  constexpr float EPSILON = 1e-5f;

  // Test all implementations produce same result
  PointCloud cloud1 = original;
  PointCloud cloud2 = original;
  PointCloud cloud3 = original;
  PointCloud cloud4 = original;
  PointCloud cloud5 = original;

  transformImpl1_PerPointLoop(cloud1, R, t);
  transformImpl2_EigenMapBatch(cloud2, R, t);
  transformImpl3_IndexLoop(cloud3, R, t);
  transformImpl4_EigenMapNoAlias(cloud4, R, t);
  transformImpl5_EigenMapPrealloc(cloud5, R, t);

  // Compare results
  for (size_t i = 0; i < original.size(); ++i) {
    if ((cloud1[i] - cloud2[i]).norm() > EPSILON ||
        (cloud1[i] - cloud3[i]).norm() > EPSILON ||
        (cloud1[i] - cloud4[i]).norm() > EPSILON ||
        (cloud1[i] - cloud5[i]).norm() > EPSILON) {
      std::cerr << "ERROR: Results differ at index " << i << "\n";
      std::cerr << "  Impl1: " << cloud1[i].transpose() << "\n";
      std::cerr << "  Impl2: " << cloud2[i].transpose() << "\n";
      std::cerr << "  Impl3: " << cloud3[i].transpose() << "\n";
      std::cerr << "  Impl4: " << cloud4[i].transpose() << "\n";
      std::cerr << "  Impl5: " << cloud5[i].transpose() << "\n";
      return false;
    }
  }

  std::cout << "Correctness verification: PASSED\n";
  return true;
}

// =============================================================================
// Main benchmark
// =============================================================================
int main() {
  std::cout << "nanoPCL Transform Benchmark\n";
  std::cout << "Comparing transform implementation strategies\n";
  std::cout << "Compiler: " << __VERSION__ << "\n";
#ifdef __AVX__
  std::cout << "SIMD: AVX enabled\n";
#elif defined(__SSE4_2__)
  std::cout << "SIMD: SSE4.2 enabled\n";
#else
  std::cout << "SIMD: Basic\n";
#endif
  std::cout << "sizeof(Vector3f) = " << sizeof(Eigen::Vector3f) << " bytes\n";

  // Generate transform
  const Eigen::Matrix3f R = generateRotation();
  const Eigen::Vector3f t = generateTranslation();

  // Test sizes
  std::vector<size_t> sizes = {1000, 10000, 50000, 100000, 500000};

  // =========================================================================
  // Benchmark: XYZ only
  // =========================================================================
  printHeader("Transform Benchmark: XYZ Only");

  for (size_t num_points : sizes) {
    printSubHeader("Cloud size: " + std::to_string(num_points) + " points");

    PointCloud cloud = generateCloud(num_points, false);

    // Verify correctness first
    if (!verifyCorrectness(cloud, R, t)) {
      return 1;
    }

    std::vector<BenchmarkResult> results;

    results.push_back(
        runBenchmark("1. Per-point loop (current)", transformImpl1_PerPointLoop,
                     cloud, R, t));

    results.push_back(runBenchmark("2. Eigen::Map batch (proposed)",
                                   transformImpl2_EigenMapBatch, cloud, R, t));

    results.push_back(runBenchmark("3. Index-based loop",
                                   transformImpl3_IndexLoop, cloud, R, t));

    results.push_back(runBenchmark("4. Eigen::Map col-by-col",
                                   transformImpl4_EigenMapNoAlias, cloud, R, t));

    results.push_back(runBenchmark("5. Eigen::Map + prealloc temp",
                                   transformImpl5_EigenMapPrealloc, cloud, R, t));

    printResults(results);
  }

  // =========================================================================
  // Benchmark: XYZ + Normals
  // =========================================================================
  printHeader("Transform Benchmark: XYZ + Normals");

  for (size_t num_points : sizes) {
    printSubHeader("Cloud size: " + std::to_string(num_points) + " points");

    PointCloud cloud = generateCloud(num_points, true);  // with normals

    std::vector<BenchmarkResult> results;

    results.push_back(
        runBenchmark("1. Per-point loop (current)", transformImpl1_PerPointLoop,
                     cloud, R, t));

    results.push_back(runBenchmark("2. Eigen::Map batch (proposed)",
                                   transformImpl2_EigenMapBatch, cloud, R, t));

    results.push_back(runBenchmark("3. Index-based loop",
                                   transformImpl3_IndexLoop, cloud, R, t));

    results.push_back(runBenchmark("4. Eigen::Map col-by-col",
                                   transformImpl4_EigenMapNoAlias, cloud, R, t));

    results.push_back(runBenchmark("5. Eigen::Map + prealloc temp",
                                   transformImpl5_EigenMapPrealloc, cloud, R, t));

    printResults(results);
  }

  // =========================================================================
  // Benchmark: Cold Cache (realistic scenario)
  // =========================================================================
  printHeader("Transform Benchmark: Cold Cache (XYZ Only)");
  std::cout << "Note: Cache is flushed before each iteration to simulate real usage\n";

  // Only test larger sizes where cache effects matter
  std::vector<size_t> cold_sizes = {50000, 100000, 500000};

  for (size_t num_points : cold_sizes) {
    printSubHeader("Cloud size: " + std::to_string(num_points) + " points (COLD CACHE)");

    PointCloud cloud = generateCloud(num_points, false);

    std::vector<BenchmarkResult> results;

    // Fewer iterations for cold cache (slower)
    constexpr int COLD_ITERS = 30;
    constexpr int COLD_WARMUP = 3;
    constexpr bool COLD = true;

    results.push_back(
        runBenchmark("1. Per-point loop (current)", transformImpl1_PerPointLoop,
                     cloud, R, t, COLD_ITERS, COLD_WARMUP, COLD));

    results.push_back(runBenchmark("2. Eigen::Map batch (proposed)",
                                   transformImpl2_EigenMapBatch, cloud, R, t,
                                   COLD_ITERS, COLD_WARMUP, COLD));

    results.push_back(runBenchmark("3. Index-based loop",
                                   transformImpl3_IndexLoop, cloud, R, t,
                                   COLD_ITERS, COLD_WARMUP, COLD));

    results.push_back(runBenchmark("4. Eigen::Map col-by-col",
                                   transformImpl4_EigenMapNoAlias, cloud, R, t,
                                   COLD_ITERS, COLD_WARMUP, COLD));

    printResults(results);
  }

  // =========================================================================
  // Warm vs Cold comparison
  // =========================================================================
  printHeader("Warm Cache vs Cold Cache Comparison (100K points)");

  {
    PointCloud cloud = generateCloud(100000, false);

    std::cout << "\n--- WARM CACHE ---\n";
    std::vector<BenchmarkResult> warm_results;
    warm_results.push_back(
        runBenchmark("Per-point loop", transformImpl1_PerPointLoop,
                     cloud, R, t, 100, 10, false));
    warm_results.push_back(
        runBenchmark("Eigen::Map batch", transformImpl2_EigenMapBatch,
                     cloud, R, t, 100, 10, false));
    printResults(warm_results);

    std::cout << "\n--- COLD CACHE ---\n";
    std::vector<BenchmarkResult> cold_results;
    cold_results.push_back(
        runBenchmark("Per-point loop", transformImpl1_PerPointLoop,
                     cloud, R, t, 30, 3, true));
    cold_results.push_back(
        runBenchmark("Eigen::Map batch", transformImpl2_EigenMapBatch,
                     cloud, R, t, 30, 3, true));
    printResults(cold_results);
  }

  // =========================================================================
  // Summary
  // =========================================================================
  printHeader("Summary");
  std::cout << R"(
Implementation notes:
1. Per-point loop     : Current implementation, simple for-each
2. Eigen::Map batch   : Zero-copy map to 3xN matrix, single GEMM operation
3. Index-based loop   : Explicit indexing (baseline alternative)
4. Eigen::Map col-col : Map with column-by-column processing (no temp)
5. Eigen::Map prealloc: Map with pre-allocated temporary matrix

Cache effects:
- WARM cache: Data already in L1/L2/L3 cache (best case)
- COLD cache: Data must be fetched from RAM (realistic scenario)

Key observations:
- Per-point loop benefits from sequential memory access (prefetching)
- Batch operations may suffer from larger working set (cache misses)
- Real-world performance depends on surrounding code and cache state
)";

  return 0;
}
