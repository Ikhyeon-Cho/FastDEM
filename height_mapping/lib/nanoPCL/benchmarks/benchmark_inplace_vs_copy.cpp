// nanoPCL Benchmark: In-place vs Copy filter API
// Compares performance of in-place modification vs returning new PointCloud
//
// Key questions:
// 1. Is Move semantics as fast as In-place? (API design decision)
// 2. Are pre-built filters (cropDistance) as fast as baseline? (optimization validation)
// 3. What's the overhead of ConstPointRef for user-defined predicates?

#include <chrono>
#include <cmath>
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

template <typename T>
void doNotOptimize(T& value) {
#if defined(__GNUC__) || defined(__clang__)
  asm volatile("" : "+r"(value) : : "memory");
#else
  volatile auto unused = value;
  (void)unused;
#endif
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

  for (size_t i = 0; i < num_points; ++i) {
    cloud.push_back(Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)));
    if (num_channels >= 1) cloud.intensity().back() = val_dist(gen);
    if (num_channels >= 2) cloud.time().back() = val_dist(gen);
    if (num_channels >= 3) cloud.ring().back() = ring_dist(gen);
  }

  return cloud;
}

// =============================================================================
// Baseline Implementations (for theoretical comparison)
// =============================================================================

// --- In-place filter (theoretical best for modify-in-place) ---
template <typename Predicate>
void baselineInPlace(PointCloud& cloud, Predicate pred) {
  Point* pts = cloud.xyz().data();
  float* intensity = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;

  size_t write = 0;
  const size_t n = cloud.size();
  for (size_t read = 0; read < n; ++read) {
    if (pred(pts[read])) {
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
  cloud.resize(write);
}

// --- Copy filter (theoretical best for preserve-original) ---
template <typename Predicate>
PointCloud baselineCopy(const PointCloud& cloud, Predicate pred) {
  PointCloud result(cloud.frameId());
  result.reserve(cloud.size());

  if (cloud.hasIntensity()) result.enableIntensity();
  if (cloud.hasTime()) result.enableTime();
  if (cloud.hasRing()) result.enableRing();
  if (cloud.hasColor()) result.enableColor();
  if (cloud.hasLabel()) result.enableLabel();

  const Point* pts = cloud.xyz().data();
  const float* intensity = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  const float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  const uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  const Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  const Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;

  const size_t n = cloud.size();
  for (size_t i = 0; i < n; ++i) {
    if (pred(pts[i])) {
      result.xyz().push_back(pts[i]);
      if (intensity) result.intensity().push_back(intensity[i]);
      if (time) result.time().push_back(time[i]);
      if (ring) result.ring().push_back(ring[i]);
      if (color) result.color().push_back(color[i]);
      if (label) result.label().push_back(label[i]);
    }
  }

  return result;
}

// --- Move pattern (in-place + move return) ---
template <typename Predicate>
PointCloud baselineMove(PointCloud&& cloud, Predicate pred) {
  baselineInPlace(cloud, pred);
  return std::move(cloud);
}

// =============================================================================
// Benchmark Infrastructure
// =============================================================================

struct BenchResult {
  std::string name;
  double time_ms;
  size_t result_size;
};

void printResults(const std::vector<BenchResult>& results, size_t num_points,
                  const std::string& baseline_name = "") {
  std::cout << std::left << std::setw(40) << "Method" << std::right
            << std::setw(12) << "Time (ms)" << std::setw(15) << "Throughput"
            << std::setw(12) << "vs Baseline"
            << "\n";
  std::cout << std::string(80, '-') << "\n";

  double baseline = results[0].time_ms;
  for (const auto& r : results) {
    double throughput = num_points / r.time_ms / 1000.0;
    double ratio = baseline / r.time_ms;
    std::string ratio_str =
        (ratio >= 1.0) ? std::to_string(ratio).substr(0, 4) + "x"
                       : std::to_string(1.0 / ratio).substr(0, 4) + "x slower";

    std::cout << std::left << std::setw(40) << r.name << std::right
              << std::fixed << std::setprecision(3) << std::setw(12)
              << r.time_ms << std::setprecision(2) << std::setw(12)
              << throughput << " Mpts/s" << std::setw(11) << ratio_str << "\n";
  }
}

void runScenario(const std::string& title, size_t num_points, int num_channels,
                 float pass_rate, int iterations) {
  // Distance threshold for pass rate
  float max_possible = std::sqrt(3.0f) * 50.0f;
  float threshold = max_possible * std::pow(pass_rate, 1.0f / 3.0f);
  float max_dist = threshold;

  std::cout << "\n" << std::string(80, '=') << "\n";
  std::cout << title << "\n";
  std::cout << "Points: " << num_points << ", Channels: " << num_channels
            << ", Pass rate: ~" << int(pass_rate * 100) << "%"
            << ", Distance threshold: " << max_dist << "m\n";
  std::cout << std::string(80, '=') << "\n";

  std::vector<BenchResult> results;

  // =========================================================================
  // PART 1: BASELINE (Theoretical performance bounds)
  // =========================================================================
  std::cout << "\n[PART 1] Baseline - Theoretical Performance\n";
  std::cout << std::string(80, '-') << "\n";

  // Use same predicate as cropDistance for fair comparison
  const float min_dist = 0.0f;
  const float min_dist_sq = min_dist * min_dist;
  const float max_dist_sq = max_dist * max_dist;
  const Point origin = Point::Zero();

  auto pred_point = [=](const Point& p) {
    float dist_sq = (p - origin).squaredNorm();
    return dist_sq >= min_dist_sq && dist_sq <= max_dist_sq;
  };

  // 1-1. In-place baseline
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      baselineInPlace(cloud, pred_point);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"[BASE] In-place", total_ms / iterations, result_size});
  }

  // 1-2. Copy baseline
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      PointCloud filtered = baselineCopy(cloud, pred_point);
      total_ms += t.elapsed_ms();
      result_size = filtered.size();
      doNotOptimize(result_size);
    }
    results.push_back({"[BASE] Copy", total_ms / iterations, result_size});
  }

  // 1-3. Move baseline (key question: is this as fast as in-place?)
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = baselineMove(std::move(cloud), pred_point);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"[BASE] Move (in-place + return)", total_ms / iterations, result_size});
  }

  printResults(results, num_points);
  results.clear();

  // =========================================================================
  // PART 2: USER API - Generic filter (ConstPointRef predicate)
  // For user-defined custom predicates that need attribute access
  // =========================================================================
  std::cout << "\n[PART 2] User API - filters::filter (ConstPointRef)\n";
  std::cout << "Use case: User-defined predicates with attribute access\n";
  std::cout << std::string(80, '-') << "\n";

  // Same condition as baseline, but using ConstPointRef
  auto pred_ref = [=](auto p) {
    float dist_sq = (p.point() - origin).squaredNorm();
    return dist_sq >= min_dist_sq && dist_sq <= max_dist_sq;
  };

  // Re-add baseline for comparison
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = baselineMove(std::move(cloud), pred_point);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"[BASE] Move (reference)", total_ms / iterations, result_size});
  }

  // 2-1. filters::filter(&&) - Move version
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = filters::filter(std::move(cloud), pred_ref);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"filters::filter(&&, ConstPointRef)", total_ms / iterations, result_size});
  }

  // 2-2. filters::filter(const&) - Copy version
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      PointCloud filtered = filters::filter(cloud, pred_ref);
      total_ms += t.elapsed_ms();
      result_size = filtered.size();
      doNotOptimize(result_size);
    }
    results.push_back({"filters::filter(const&, ConstPointRef)", total_ms / iterations, result_size});
  }

  printResults(results, num_points);
  results.clear();

  // =========================================================================
  // PART 3: USER API - Pre-built filters (Point& optimized)
  // Library-provided filters should match baseline performance
  // =========================================================================
  std::cout << "\n[PART 3] User API - Pre-built Filters (Optimized)\n";
  std::cout << "Use case: Library-provided filters (cropDistance, cropBox, etc.)\n";
  std::cout << std::string(80, '-') << "\n";

  // Re-add baseline for comparison
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = baselineMove(std::move(cloud), pred_point);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"[BASE] Move (reference)", total_ms / iterations, result_size});
  }

  // 3-1. cropRange(&&) - Same condition as baseline
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = filters::cropRange(std::move(cloud), 0.0f, max_dist);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"filters::cropRange(&&)", total_ms / iterations, result_size});
  }

  // 3-2. cropRange(const&) - Copy version
  {
    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      PointCloud filtered = filters::cropRange(cloud, 0.0f, max_dist);
      total_ms += t.elapsed_ms();
      result_size = filtered.size();
      doNotOptimize(result_size);
    }
    results.push_back({"filters::cropRange(const&)", total_ms / iterations, result_size});
  }

  // 3-3. cropBox(&&) - Alternative pre-built filter
  {
    // Calculate box that approximates the sphere
    float box_half = max_dist / std::sqrt(3.0f);  // inscribed box
    Point box_min(-box_half, -box_half, -box_half);
    Point box_max(box_half, box_half, box_half);

    double total_ms = 0;
    size_t result_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = filters::cropBox(std::move(cloud), box_min, box_max);
      total_ms += t.elapsed_ms();
      result_size = cloud.size();
      doNotOptimize(result_size);
    }
    results.push_back({"filters::cropBox(&&)", total_ms / iterations, result_size});
  }

  printResults(results, num_points);

  // Verify correctness
  std::cout << "\nExpected pass rate: ~" << int(pass_rate * 100) << "%\n";
}

// =============================================================================
// Main
// =============================================================================
int main() {
  std::cout << "nanoPCL Benchmark: In-place vs Copy Filter API\n";
  std::cout << std::string(80, '=') << "\n";
  std::cout << R"(
Purpose:
  1. Verify Move semantics matches In-place performance
  2. Validate pre-built filters (cropDistance) are optimized
  3. Measure ConstPointRef overhead for user-defined predicates

API Design Decision:
  - If Move == In-place: No separate in-place API needed
  - If Pre-built == Baseline: Internal optimization is working
  - ConstPointRef overhead is acceptable for user convenience
)" << "\n";

  const int ITERATIONS = 20;

  runScenario("Small cloud, few channels, low pass rate",
              100000, 1, 0.1f, ITERATIONS);

  runScenario("Medium cloud, all channels, medium pass rate",
              500000, 3, 0.5f, ITERATIONS);

  runScenario("Large cloud, all channels, high pass rate",
              1000000, 3, 0.9f, ITERATIONS);

  // Summary
  std::cout << "\n" << std::string(80, '=') << "\n";
  std::cout << "CONCLUSION\n";
  std::cout << std::string(80, '=') << "\n";
  std::cout << R"(
Check the results above:

1. [BASE] Move vs [BASE] In-place
   - If ~1.0x: Move semantics is sufficient, no in-place API needed

2. filters::cropRange(&&) vs [BASE] Move
   - If ~1.0x: Pre-built filters are properly optimized

3. filters::filter(&&) vs [BASE] Move
   - Difference shows ConstPointRef overhead (acceptable for user convenience)
)";

  return 0;
}
