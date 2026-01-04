// nanoPCL Filter Implementation Benchmark
// Compares different in-place filter strategies for SoA layout
//
// Implementations compared:
// 1. Direct if-chain (baseline)
// 2. Pointer caching
// 3. copyPointFrom method
// 4. ConstPointRef predicate
// 5. Raw Point& predicate

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <nanopcl/nanopcl.hpp>

using namespace nanopcl;

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
};

// Prevent compiler optimization
template <typename T>
void doNotOptimize(T& value) {
  asm volatile("" : "+r"(value) : : "memory");
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
PointCloud generateCloud(size_t num_points, int num_channels, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> val_dist(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring_dist(0, 127);

  PointCloud cloud("benchmark");
  cloud.reserve(num_points);

  if (num_channels >= 1) cloud.enableIntensity();
  if (num_channels >= 2) cloud.enableTime();
  if (num_channels >= 3) cloud.enableRing();
  if (num_channels >= 4) cloud.enableColor();
  if (num_channels >= 5) cloud.enableLabel();

  for (size_t i = 0; i < num_points; ++i) {
    cloud.push_back(Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)));
    if (num_channels >= 1) cloud.intensity().back() = val_dist(gen);
    if (num_channels >= 2) cloud.time().back() = val_dist(gen);
    if (num_channels >= 3) cloud.ring().back() = ring_dist(gen);
    if (num_channels >= 4) cloud.color().back() = Color(128, 128, 128);
    if (num_channels >= 5) cloud.label().back() = Label(SemanticClass(10));
  }

  return cloud;
}

// =============================================================================
// Filter implementations to benchmark
// =============================================================================

// Distance filter condition
inline bool passesDistance(const Point& p, float min_sq, float max_sq) {
  float dist_sq = p.squaredNorm();
  return dist_sq >= min_sq && dist_sq <= max_sq;
}

// --- Implementation 1: Direct if-chain (baseline) ---
size_t filterImpl1_DirectIfChain(PointCloud& cloud, float min_dist, float max_dist) {
  const float min_sq = min_dist * min_dist;
  const float max_sq = max_dist * max_dist;

  size_t write = 0;
  for (size_t read = 0; read < cloud.size(); ++read) {
    if (passesDistance(cloud[read], min_sq, max_sq)) {
      if (write != read) {
        cloud[write] = cloud[read];
        if (cloud.hasIntensity()) cloud.intensity()[write] = cloud.intensity()[read];
        if (cloud.hasTime()) cloud.time()[write] = cloud.time()[read];
        if (cloud.hasRing()) cloud.ring()[write] = cloud.ring()[read];
        if (cloud.hasColor()) cloud.color()[write] = cloud.color()[read];
        if (cloud.hasLabel()) cloud.label()[write] = cloud.label()[read];
      }
      ++write;
    }
  }
  size_t removed = cloud.size() - write;
  cloud.resize(write);
  return removed;
}

// --- Implementation 2: Pointer caching ---
size_t filterImpl2_PointerCaching(PointCloud& cloud, float min_dist, float max_dist) {
  const float min_sq = min_dist * min_dist;
  const float max_sq = max_dist * max_dist;

  // Cache pointers outside loop
  Point* pts = cloud.xyz().data();
  float* intensity = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;

  size_t write = 0;
  const size_t n = cloud.size();
  for (size_t read = 0; read < n; ++read) {
    if (passesDistance(pts[read], min_sq, max_sq)) {
      if (write != read) {
        pts[write] = pts[read];
        if (intensity) intensity[write] = intensity[read];
        if (time) time[write] = time[read];
        if (ring) ring[write] = ring[read];
        if (color) color[write] = color[read];
        if (label) label[write] = label[read];
      }
      ++write;
    }
  }
  size_t removed = cloud.size() - write;
  cloud.resize(write);
  return removed;
}

// --- Implementation 3: copyPointFrom method (simulated) ---
// We simulate this by inlining what copyPointFrom would do
size_t filterImpl3_CopyPointFrom(PointCloud& cloud, float min_dist, float max_dist) {
  const float min_sq = min_dist * min_dist;
  const float max_sq = max_dist * max_dist;

  // Simulate copyPointFrom by caching everything upfront
  auto& pts = cloud.xyz();
  const bool has_i = cloud.hasIntensity();
  const bool has_t = cloud.hasTime();
  const bool has_r = cloud.hasRing();
  const bool has_c = cloud.hasColor();
  const bool has_l = cloud.hasLabel();

  size_t write = 0;
  for (size_t read = 0; read < cloud.size(); ++read) {
    if (passesDistance(pts[read], min_sq, max_sq)) {
      if (write != read) {
        // This is what copyPointFrom would do
        pts[write] = pts[read];
        if (has_i) cloud.intensity()[write] = cloud.intensity()[read];
        if (has_t) cloud.time()[write] = cloud.time()[read];
        if (has_r) cloud.ring()[write] = cloud.ring()[read];
        if (has_c) cloud.color()[write] = cloud.color()[read];
        if (has_l) cloud.label()[write] = cloud.label()[read];
      }
      ++write;
    }
  }
  size_t removed = cloud.size() - write;
  cloud.resize(write);
  return removed;
}

// --- Implementation 4: ConstPointRef predicate ---
template <typename Predicate>
size_t filterImpl4_ConstPointRef(PointCloud& cloud, Predicate pred) {
  auto& pts = cloud.xyz();
  const bool has_i = cloud.hasIntensity();
  const bool has_t = cloud.hasTime();
  const bool has_r = cloud.hasRing();
  const bool has_c = cloud.hasColor();
  const bool has_l = cloud.hasLabel();

  size_t write = 0;
  for (size_t read = 0; read < cloud.size(); ++read) {
    if (pred(cloud.point(read))) {  // ConstPointRef creation
      if (write != read) {
        pts[write] = pts[read];
        if (has_i) cloud.intensity()[write] = cloud.intensity()[read];
        if (has_t) cloud.time()[write] = cloud.time()[read];
        if (has_r) cloud.ring()[write] = cloud.ring()[read];
        if (has_c) cloud.color()[write] = cloud.color()[read];
        if (has_l) cloud.label()[write] = cloud.label()[read];
      }
      ++write;
    }
  }
  size_t removed = cloud.size() - write;
  cloud.resize(write);
  return removed;
}

// --- Implementation 5: Raw Point& predicate (no channel access in predicate) ---
template <typename Predicate>
size_t filterImpl5_RawPoint(PointCloud& cloud, Predicate pred) {
  auto& pts = cloud.xyz();
  const bool has_i = cloud.hasIntensity();
  const bool has_t = cloud.hasTime();
  const bool has_r = cloud.hasRing();
  const bool has_c = cloud.hasColor();
  const bool has_l = cloud.hasLabel();

  size_t write = 0;
  for (size_t read = 0; read < cloud.size(); ++read) {
    if (pred(pts[read])) {  // Raw Point reference
      if (write != read) {
        pts[write] = pts[read];
        if (has_i) cloud.intensity()[write] = cloud.intensity()[read];
        if (has_t) cloud.time()[write] = cloud.time()[read];
        if (has_r) cloud.ring()[write] = cloud.ring()[read];
        if (has_c) cloud.color()[write] = cloud.color()[read];
        if (has_l) cloud.label()[write] = cloud.label()[read];
      }
      ++write;
    }
  }
  size_t removed = cloud.size() - write;
  cloud.resize(write);
  return removed;
}

// =============================================================================
// Benchmark runner
// =============================================================================
struct BenchmarkResult {
  std::string name;
  double time_ms;
  size_t remaining;
};

void runBenchmark(size_t num_points, int num_channels, float pass_rate, int iterations) {
  // Calculate distance thresholds for desired pass rate
  // Points are uniform in [-50, 50]^3, max distance from origin is sqrt(3)*50 ~ 86.6
  float max_possible = std::sqrt(3.0f) * 50.0f;
  float threshold = max_possible * std::pow(pass_rate, 1.0f/3.0f);  // Approximate for sphere
  float min_dist = 0.0f;
  float max_dist = threshold;

  const float min_sq = min_dist * min_dist;
  const float max_sq = max_dist * max_dist;

  std::cout << std::fixed << std::setprecision(1);
  std::cout << "\nPoints: " << num_points
            << ", Channels: " << num_channels
            << ", Target pass rate: " << (pass_rate * 100) << "%"
            << ", Iterations: " << iterations << "\n";
  std::cout << std::string(70, '-') << "\n";

  std::vector<BenchmarkResult> results;

  // Warm up and verify all implementations produce same result
  {
    PointCloud verify = generateCloud(num_points, num_channels);
    size_t expected = filterImpl2_PointerCaching(verify, min_dist, max_dist);
    std::cout << "Actual pass rate: " << (100.0 * verify.size() / num_points) << "% ";
    std::cout << "(" << verify.size() << "/" << num_points << " points)\n\n";
  }

  // Benchmark 1: Direct if-chain
  {
    double total_ms = 0;
    size_t remaining = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      remaining = cloud.size() - filterImpl1_DirectIfChain(cloud, min_dist, max_dist);
      total_ms += t.elapsed_ms();
      doNotOptimize(remaining);
    }
    results.push_back({"1. Direct if-chain", total_ms / iterations, remaining});
  }

  // Benchmark 2: Pointer caching
  {
    double total_ms = 0;
    size_t remaining = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      remaining = cloud.size() - filterImpl2_PointerCaching(cloud, min_dist, max_dist);
      total_ms += t.elapsed_ms();
      doNotOptimize(remaining);
    }
    results.push_back({"2. Pointer caching", total_ms / iterations, remaining});
  }

  // Benchmark 3: copyPointFrom (simulated)
  {
    double total_ms = 0;
    size_t remaining = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      remaining = cloud.size() - filterImpl3_CopyPointFrom(cloud, min_dist, max_dist);
      total_ms += t.elapsed_ms();
      doNotOptimize(remaining);
    }
    results.push_back({"3. copyPointFrom", total_ms / iterations, remaining});
  }

  // Benchmark 4: PointRef predicate (via auto)
  {
    double total_ms = 0;
    size_t remaining = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      auto pred = [min_sq, max_sq](auto p) {  // accepts PointRef
        float dist_sq = p.point().squaredNorm();
        return dist_sq >= min_sq && dist_sq <= max_sq;
      };
      remaining = cloud.size() - filterImpl4_ConstPointRef(cloud, pred);
      total_ms += t.elapsed_ms();
      doNotOptimize(remaining);
    }
    results.push_back({"4. PointRef pred", total_ms / iterations, remaining});
  }

  // Benchmark 5: Raw Point& predicate
  {
    double total_ms = 0;
    size_t remaining = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      auto pred = [min_sq, max_sq](const Point& p) {
        float dist_sq = p.squaredNorm();
        return dist_sq >= min_sq && dist_sq <= max_sq;
      };
      remaining = cloud.size() - filterImpl5_RawPoint(cloud, pred);
      total_ms += t.elapsed_ms();
      doNotOptimize(remaining);
    }
    results.push_back({"5. Raw Point& pred", total_ms / iterations, remaining});
  }

  // Print results
  std::cout << std::left << std::setw(30) << "Implementation"
            << std::right << std::setw(12) << "Time (ms)"
            << std::setw(15) << "Throughput"
            << std::setw(10) << "Relative" << "\n";
  std::cout << std::string(70, '-') << "\n";

  double baseline = results[0].time_ms;
  for (const auto& r : results) {
    double throughput = num_points / r.time_ms / 1000.0;  // Million pts/sec
    std::cout << std::left << std::setw(30) << r.name
              << std::right << std::fixed << std::setprecision(3)
              << std::setw(12) << r.time_ms
              << std::setprecision(2) << std::setw(12) << throughput << " Mpts/s"
              << std::setprecision(2) << std::setw(9) << (baseline / r.time_ms) << "x\n";
  }
}

// =============================================================================
// Main
// =============================================================================
int main() {
  std::cout << "nanoPCL Filter Implementation Benchmark\n";
  std::cout << "Comparing in-place filter strategies for SoA layout\n";
  std::cout << std::string(70, '=') << "\n";

  const int ITERATIONS = 20;

  // Test matrix
  std::vector<size_t> point_counts = {100000, 500000, 1000000};
  std::vector<int> channel_counts = {0, 3, 5};
  std::vector<float> pass_rates = {0.1f, 0.5f, 0.9f};

  for (size_t num_points : point_counts) {
    printHeader("Point Count: " + std::to_string(num_points));

    for (int num_channels : channel_counts) {
      printSubHeader("Channels: " + std::to_string(num_channels));

      for (float pass_rate : pass_rates) {
        runBenchmark(num_points, num_channels, pass_rate, ITERATIONS);
      }
    }
  }

  // Summary
  printHeader("ANALYSIS");
  std::cout << R"(
Key observations to look for:
1. Pointer caching vs Direct if-chain: How much does branch prediction cost?
2. ConstPointRef vs Raw Point&: What's the overhead of PointRef creation?
3. Effect of channel count: Does copying more channels dominate?
4. Effect of pass rate: High pass = more copies, Low pass = more branches

Based on results, choose implementation for production.
)";

  return 0;
}
