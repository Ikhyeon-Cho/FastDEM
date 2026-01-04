// nanoPCL Benchmark: VoxelGrid Move Semantics
// Compares performance of copy vs move versions of voxelGrid filter
//
// Key questions:
// 1. Can move semantics improve voxelGrid performance?
// 2. What's the theoretical limit (output buffer reuse)?
// 3. Is it worth adding a move version to the API?

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
// VoxelGrid implementations to benchmark
// =============================================================================

namespace benchmark_detail {

// Voxel coordinate limits (same as in voxel_grid_impl.hpp)
constexpr int32_t VOXEL_COORD_OFFSET = 1 << 20;
constexpr int32_t VOXEL_COORD_MIN = -VOXEL_COORD_OFFSET;
constexpr int32_t VOXEL_COORD_MAX = VOXEL_COORD_OFFSET - 1;
constexpr uint64_t VOXEL_COORD_MASK = 0x1FFFFF;

struct IndexedPoint {
  uint64_t key;
  uint32_t index;
  bool operator<(const IndexedPoint& other) const { return key < other.key; }
};

inline uint64_t computeVoxelKey(const Point& p, float inv_voxel_size) {
  int32_t ix = static_cast<int32_t>(std::floor(p.x() * inv_voxel_size));
  int32_t iy = static_cast<int32_t>(std::floor(p.y() * inv_voxel_size));
  int32_t iz = static_cast<int32_t>(std::floor(p.z() * inv_voxel_size));

  ix = std::clamp(ix, VOXEL_COORD_MIN, VOXEL_COORD_MAX);
  iy = std::clamp(iy, VOXEL_COORD_MIN, VOXEL_COORD_MAX);
  iz = std::clamp(iz, VOXEL_COORD_MIN, VOXEL_COORD_MAX);

  uint64_t ux = static_cast<uint64_t>(ix + VOXEL_COORD_OFFSET);
  uint64_t uy = static_cast<uint64_t>(iy + VOXEL_COORD_OFFSET);
  uint64_t uz = static_cast<uint64_t>(iz + VOXEL_COORD_OFFSET);

  return (ux << 42) | (uy << 21) | uz;
}

}  // namespace benchmark_detail

// --- Version 1: Copy version (const&) ---
PointCloud voxelGridCopy(const PointCloud& cloud, float voxel_size) {
  return filters::voxelGrid(cloud, voxel_size, filters::VoxelMethod::CENTROID);
}

// --- Version 2: Move version (official API) ---
PointCloud voxelGridMoveAPI(PointCloud&& cloud, float voxel_size) {
  return filters::voxelGrid(std::move(cloud), voxel_size, filters::VoxelMethod::CENTROID);
}

// --- Version 2: Move version - reuses input's memory for output ---
// Correct implementation: compute results first, then write to input buffer
PointCloud voxelGridMove(PointCloud&& cloud, float voxel_size) {
  if (cloud.empty()) {
    return std::move(cloud);
  }

  const float inv_voxel_size = 1.0f / voxel_size;

  // Cache channel presence
  const bool has_intensity = cloud.hasIntensity();
  const bool has_time = cloud.hasTime();
  const bool has_ring = cloud.hasRing();
  const bool has_color = cloud.hasColor();
  const bool has_label = cloud.hasLabel();

  // Step 1: Compute voxel keys
  std::vector<benchmark_detail::IndexedPoint> indexed_points;
  indexed_points.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    uint64_t key = benchmark_detail::computeVoxelKey(point, inv_voxel_size);
    indexed_points.push_back({key, static_cast<uint32_t>(i)});
  }

  if (indexed_points.empty()) {
    cloud.clear();
    return std::move(cloud);
  }

  // Step 2: Sort by voxel key
  std::sort(indexed_points.begin(), indexed_points.end());

  // Step 3: Compute all centroids FIRST (read all original data)
  struct VoxelResult {
    Point centroid;
    float intensity;
    float time;
    uint16_t ring;
    Color color;
    Label label;
  };

  // Estimate output size
  size_t estimated_voxels = indexed_points.size() / 4;
  if (estimated_voxels < 100) estimated_voxels = 100;
  std::vector<VoxelResult> results;
  results.reserve(estimated_voxels);

  size_t group_start = 0;
  while (group_start < indexed_points.size()) {
    uint64_t current_key = indexed_points[group_start].key;

    size_t group_end = group_start + 1;
    while (group_end < indexed_points.size() &&
           indexed_points[group_end].key == current_key) {
      ++group_end;
    }

    // Compute centroid from ORIGINAL data
    Point centroid = Point::Zero();
    float avg_intensity = 0.0f;
    float avg_time = 0.0f;
    float avg_r = 0, avg_g = 0, avg_b = 0;
    const float n = static_cast<float>(group_end - group_start);

    for (size_t i = group_start; i < group_end; ++i) {
      uint32_t idx = indexed_points[i].index;
      centroid += cloud[idx];
      if (has_intensity) avg_intensity += cloud.intensity()[idx];
      if (has_time) avg_time += cloud.time()[idx];
      if (has_color) {
        avg_r += cloud.color()[idx][0];
        avg_g += cloud.color()[idx][1];
        avg_b += cloud.color()[idx][2];
      }
    }

    VoxelResult res;
    res.centroid = centroid / n;
    res.intensity = avg_intensity / n;
    res.time = avg_time / n;
    res.ring = has_ring ? cloud.ring()[indexed_points[group_start].index] : 0;
    res.color = has_color ? Color(static_cast<uint8_t>(avg_r / n),
                                  static_cast<uint8_t>(avg_g / n),
                                  static_cast<uint8_t>(avg_b / n))
                          : Color();
    res.label = has_label ? cloud.label()[indexed_points[group_start].index] : Label();
    results.push_back(res);

    group_start = group_end;
  }

  // Step 4: Write results to input cloud's buffer (reuse memory)
  cloud.resize(results.size());
  Point* pts = cloud.xyz().data();
  float* intensity = has_intensity ? cloud.intensity().data() : nullptr;
  float* time = has_time ? cloud.time().data() : nullptr;
  uint16_t* ring = has_ring ? cloud.ring().data() : nullptr;
  Color* color = has_color ? cloud.color().data() : nullptr;
  Label* label = has_label ? cloud.label().data() : nullptr;

  for (size_t i = 0; i < results.size(); ++i) {
    pts[i] = results[i].centroid;
    if (intensity) intensity[i] = results[i].intensity;
    if (time) time[i] = results[i].time;
    if (ring) ring[i] = results[i].ring;
    if (color) color[i] = results[i].color;
    if (label) label[i] = results[i].label;
  }

  return std::move(cloud);
}

// --- Version 3: Move with pre-allocated indexed_points ---
// Hypothesis: Reusing indexed_points buffer helps?
thread_local std::vector<benchmark_detail::IndexedPoint> g_indexed_points;

struct VoxelResultTLS {
  Point centroid;
  float intensity;
  float time;
  uint16_t ring;
  Color color;
  Label label;
};
thread_local std::vector<VoxelResultTLS> g_results;

PointCloud voxelGridMovePrealloc(PointCloud&& cloud, float voxel_size) {
  if (cloud.empty()) {
    return std::move(cloud);
  }

  const float inv_voxel_size = 1.0f / voxel_size;

  const bool has_intensity = cloud.hasIntensity();
  const bool has_time = cloud.hasTime();
  const bool has_ring = cloud.hasRing();
  const bool has_color = cloud.hasColor();
  const bool has_label = cloud.hasLabel();

  // Reuse thread-local buffer
  g_indexed_points.clear();
  g_indexed_points.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    uint64_t key = benchmark_detail::computeVoxelKey(point, inv_voxel_size);
    g_indexed_points.push_back({key, static_cast<uint32_t>(i)});
  }

  if (g_indexed_points.empty()) {
    cloud.clear();
    return std::move(cloud);
  }

  std::sort(g_indexed_points.begin(), g_indexed_points.end());

  // Compute all results first (using TLS buffer)
  g_results.clear();
  g_results.reserve(g_indexed_points.size() / 4 + 100);

  size_t group_start = 0;
  while (group_start < g_indexed_points.size()) {
    uint64_t current_key = g_indexed_points[group_start].key;

    size_t group_end = group_start + 1;
    while (group_end < g_indexed_points.size() &&
           g_indexed_points[group_end].key == current_key) {
      ++group_end;
    }

    Point centroid = Point::Zero();
    float avg_intensity = 0.0f;
    float avg_time = 0.0f;
    float avg_r = 0, avg_g = 0, avg_b = 0;
    const float n = static_cast<float>(group_end - group_start);

    for (size_t i = group_start; i < group_end; ++i) {
      uint32_t idx = g_indexed_points[i].index;
      centroid += cloud[idx];
      if (has_intensity) avg_intensity += cloud.intensity()[idx];
      if (has_time) avg_time += cloud.time()[idx];
      if (has_color) {
        avg_r += cloud.color()[idx][0];
        avg_g += cloud.color()[idx][1];
        avg_b += cloud.color()[idx][2];
      }
    }

    VoxelResultTLS res;
    res.centroid = centroid / n;
    res.intensity = avg_intensity / n;
    res.time = avg_time / n;
    res.ring = has_ring ? cloud.ring()[g_indexed_points[group_start].index] : 0;
    res.color = has_color ? Color(static_cast<uint8_t>(avg_r / n),
                                  static_cast<uint8_t>(avg_g / n),
                                  static_cast<uint8_t>(avg_b / n))
                          : Color();
    res.label = has_label ? cloud.label()[g_indexed_points[group_start].index] : Label();
    g_results.push_back(res);

    group_start = group_end;
  }

  // Write results to input cloud's buffer
  cloud.resize(g_results.size());
  Point* pts = cloud.xyz().data();
  float* intensity = has_intensity ? cloud.intensity().data() : nullptr;
  float* time = has_time ? cloud.time().data() : nullptr;
  uint16_t* ring = has_ring ? cloud.ring().data() : nullptr;
  Color* color = has_color ? cloud.color().data() : nullptr;
  Label* label = has_label ? cloud.label().data() : nullptr;

  for (size_t i = 0; i < g_results.size(); ++i) {
    pts[i] = g_results[i].centroid;
    if (intensity) intensity[i] = g_results[i].intensity;
    if (time) time[i] = g_results[i].time;
    if (ring) ring[i] = g_results[i].ring;
    if (color) color[i] = g_results[i].color;
    if (label) label[i] = g_results[i].label;
  }

  return std::move(cloud);
}

// =============================================================================
// Benchmark Infrastructure
// =============================================================================

struct BenchResult {
  std::string name;
  double time_ms;
  size_t output_size;
  double reduction_ratio;
};

void printResults(const std::vector<BenchResult>& results, size_t num_points) {
  std::cout << std::left << std::setw(45) << "Method" << std::right
            << std::setw(12) << "Time (ms)" << std::setw(15) << "Throughput"
            << std::setw(12) << "vs Baseline" << std::setw(12) << "Reduction"
            << "\n";
  std::cout << std::string(95, '-') << "\n";

  double baseline = results[0].time_ms;
  for (const auto& r : results) {
    double throughput = num_points / r.time_ms / 1000.0;
    double ratio = baseline / r.time_ms;

    std::cout << std::left << std::setw(45) << r.name << std::right
              << std::fixed << std::setprecision(3) << std::setw(12)
              << r.time_ms << std::setprecision(2) << std::setw(12)
              << throughput << " Mpts/s" << std::setw(11) << ratio << "x"
              << std::setw(11) << (r.reduction_ratio * 100) << "%\n";
  }
}

void runBenchmark(size_t num_points, int num_channels, float voxel_size,
                  int iterations) {
  std::cout << "\n" << std::string(95, '=') << "\n";
  std::cout << "VoxelGrid Benchmark\n";
  std::cout << "Points: " << num_points << ", Channels: " << num_channels
            << ", Voxel size: " << voxel_size << "m"
            << ", Iterations: " << iterations << "\n";
  std::cout << std::string(95, '=') << "\n";

  std::vector<BenchResult> results;

  // Warm up and get expected output size
  size_t expected_size = 0;
  {
    PointCloud test = generateCloud(num_points, num_channels);
    PointCloud result = voxelGridCopy(test, voxel_size);
    expected_size = result.size();
    std::cout << "Expected reduction: " << num_points << " -> " << expected_size
              << " points (" << std::fixed << std::setprecision(1)
              << (100.0 * expected_size / num_points) << "%)\n\n";
  }

  // 1. Current implementation (const& copy version)
  {
    double total_ms = 0;
    size_t output_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      PointCloud result = voxelGridCopy(cloud, voxel_size);
      total_ms += t.elapsed_ms();
      output_size = result.size();
      doNotOptimize(output_size);
    }
    results.push_back({"[CURRENT] voxelGrid(const&)",
                       total_ms / iterations,
                       output_size,
                       static_cast<double>(output_size) / num_points});
  }

  // 2. Move version (official API)
  {
    double total_ms = 0;
    size_t output_size = 0;
    for (int i = 0; i < iterations; ++i) {
      PointCloud cloud = generateCloud(num_points, num_channels, 42 + i);
      Timer t;
      cloud = voxelGridMoveAPI(std::move(cloud), voxel_size);
      total_ms += t.elapsed_ms();
      output_size = cloud.size();
      doNotOptimize(output_size);
    }
    results.push_back({"[MOVE] voxelGrid(&&) - Official API",
                       total_ms / iterations,
                       output_size,
                       static_cast<double>(output_size) / num_points});
  }

  // 4. Measure allocation overhead separately
  {
    double total_ms = 0;
    for (int i = 0; i < iterations; ++i) {
      Timer t;
      PointCloud cloud(num_points);
      cloud.enableIntensity();
      cloud.enableTime();
      cloud.enableRing();
      cloud.reserve(num_points);
      total_ms += t.elapsed_ms();
      size_t sz = cloud.capacity();
      doNotOptimize(sz);
    }
    results.push_back({"[REF] PointCloud allocation only",
                       total_ms / iterations,
                       expected_size,
                       static_cast<double>(expected_size) / num_points});
  }

  printResults(results, num_points);

  // Verify correctness (compare sorted points since order may differ)
  std::cout << "\nCorrectness check: ";
  PointCloud test1 = generateCloud(1000, 3, 999);
  PointCloud test2 = generateCloud(1000, 3, 999);
  PointCloud result1 = voxelGridCopy(test1, voxel_size);
  PointCloud result2 = voxelGridMoveAPI(std::move(test2), voxel_size);

  if (result1.size() != result2.size()) {
    std::cout << "FAILED (size mismatch: " << result1.size() << " vs "
              << result2.size() << ")\n";
  } else {
    // Sort points for comparison (order may differ between implementations)
    auto getSortedPoints = [](PointCloud& c) {
      std::vector<std::tuple<float, float, float>> pts;
      pts.reserve(c.size());
      for (size_t i = 0; i < c.size(); ++i) {
        pts.emplace_back(c[i].x(), c[i].y(), c[i].z());
      }
      std::sort(pts.begin(), pts.end());
      return pts;
    };

    auto pts1 = getSortedPoints(result1);
    auto pts2 = getSortedPoints(result2);

    float maxDiff = 0;
    for (size_t i = 0; i < pts1.size(); ++i) {
      float dx = std::abs(std::get<0>(pts1[i]) - std::get<0>(pts2[i]));
      float dy = std::abs(std::get<1>(pts1[i]) - std::get<1>(pts2[i]));
      float dz = std::abs(std::get<2>(pts1[i]) - std::get<2>(pts2[i]));
      maxDiff = std::max({maxDiff, dx, dy, dz});
    }

    if (maxDiff < 1e-4f) {
      std::cout << "PASSED (max diff: " << maxDiff << ")\n";
    } else {
      std::cout << "FAILED (max diff: " << maxDiff << ")\n";
    }
  }
}

// =============================================================================
// Main
// =============================================================================
int main() {
  std::cout << "nanoPCL Benchmark: VoxelGrid Move Semantics\n";
  std::cout << std::string(95, '=') << "\n";
  std::cout << R"(
Purpose:
  1. Test if move semantics can improve voxelGrid performance
  2. Measure the benefit of in-place buffer reuse
  3. Decide whether to add move version to public API

Hypothesis:
  - VoxelGrid creates new data (centroids), so input buffer reuse may be limited
  - However, avoiding output allocation could still help
  - Thread-local buffer for indexed_points may reduce allocation overhead

)" << "\n";

  const int ITERATIONS = 20;

  // Test matrix: different point counts and voxel sizes
  std::vector<std::tuple<size_t, float, std::string>> scenarios = {
      {100000, 0.5f, "Small cloud, large voxels (high reduction)"},
      {100000, 0.1f, "Small cloud, small voxels (low reduction)"},
      {500000, 0.5f, "Medium cloud, large voxels"},
      {500000, 0.1f, "Medium cloud, small voxels"},
      {1000000, 0.5f, "Large cloud, large voxels"},
      {1000000, 0.1f, "Large cloud, small voxels"},
  };

  for (const auto& [num_points, voxel_size, desc] : scenarios) {
    std::cout << "\n>>> " << desc << " <<<\n";
    runBenchmark(num_points, 3, voxel_size, ITERATIONS);
  }

  // Summary
  std::cout << "\n" << std::string(95, '=') << "\n";
  std::cout << "CONCLUSION\n";
  std::cout << std::string(95, '=') << "\n";
  std::cout << R"(
Analyze the results above:

1. [MOVE] vs [CURRENT]:
   - If > 1.1x speedup: Add move version to API
   - If ~1.0x: Move semantics provides no benefit, keep const& only

2. [MOVE+PREALLOC] vs [MOVE]:
   - Shows benefit of avoiding indexed_points reallocation
   - If significant: Consider internal buffer pooling

3. [REF] Allocation overhead:
   - Shows how much time is spent on output allocation
   - This is the theoretical maximum gain from move semantics

Key insight: VoxelGrid fundamentally creates NEW point data (centroids),
so we cannot avoid the computation cost. The only potential gain is from:
  a) Reusing input buffer for output (avoiding allocation)
  b) Reusing indexed_points buffer (thread-local)

If gains are minimal, keep the simple const& API only.
)";

  return 0;
}
