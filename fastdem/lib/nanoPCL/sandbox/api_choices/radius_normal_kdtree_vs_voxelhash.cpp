// nanoPCL Benchmark: Radius-based Normal Estimation
// KdTree vs VoxelHash for radius search in normal estimation
//
// Build: g++ -std=c++17 -O3 -fopenmp -I../../include radius_normal_kdtree_vs_voxelhash.cpp -o radius_normal_bench

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include <nanopcl/core/point_cloud.hpp>
#include <nanopcl/search.hpp>
#include <nanopcl/geometry/impl/pca.hpp>

using namespace nanopcl;

// =============================================================================
// Timer & Utilities
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
};

template <typename T>
void doNotOptimize(T& value) {
#if defined(__GNUC__) || defined(__clang__)
  asm volatile(""
               :
               : "g"(&value)
               : "memory");
#else
  volatile auto unused = &value;
  (void)unused;
#endif
}

constexpr size_t CACHE_FLUSH_SIZE = 64 * 1024 * 1024;
static std::vector<uint8_t> g_cache_flush_buffer;

void initCacheFlushBuffer() {
  if (g_cache_flush_buffer.empty()) {
    g_cache_flush_buffer.resize(CACHE_FLUSH_SIZE);
    std::mt19937 gen(12345);
    std::uniform_int_distribution<uint8_t> dist(0, 255);
    for (auto& b : g_cache_flush_buffer)
      b = dist(gen);
  }
}

volatile uint64_t g_cache_sink = 0;
void flushCache() {
  uint64_t sum = 0;
  for (size_t i = 0; i < g_cache_flush_buffer.size(); i += 64)
    sum += g_cache_flush_buffer[i];
  g_cache_sink = sum;
}

struct Stats {
  double mean, stddev, min, max;
};

Stats computeStats(const std::vector<double>& data) {
  if (data.empty())
    return {0, 0, 0, 0};
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();
  double sq_sum = 0;
  for (double v : data)
    sq_sum += (v - mean) * (v - mean);
  auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
  return {mean, std::sqrt(sq_sum / data.size()), *min_it, *max_it};
}

// =============================================================================
// LiDAR Scan Generator
// =============================================================================
struct LidarSpec {
  std::string name;
  int num_rings;
  int points_per_ring;
  float vertical_fov_min;
  float vertical_fov_max;
};

const LidarSpec VLP16 = {"VLP-16", 16, 1800, -15.0f, 15.0f};
const LidarSpec OS1_64 = {"OS1-64", 64, 1024, -22.5f, 22.5f};
const LidarSpec OS1_128 = {"OS1-128", 128, 1024, -22.5f, 22.5f};

PointCloud generateLidarScan(const LidarSpec& spec, float max_range, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> noise(-0.02f, 0.02f);
  std::uniform_real_distribution<float> range_var(0.3f, 1.0f);
  std::uniform_real_distribution<float> dropout(0.0f, 1.0f);

  PointCloud cloud;
  cloud.reserve(spec.num_rings * spec.points_per_ring);

  float vert_step = (spec.vertical_fov_max - spec.vertical_fov_min) / (spec.num_rings - 1);
  float horiz_step = 360.0f / spec.points_per_ring;

  for (int ring = 0; ring < spec.num_rings; ++ring) {
    float elevation = (spec.vertical_fov_min + ring * vert_step) * M_PI / 180.0f;
    for (int h = 0; h < spec.points_per_ring; ++h) {
      if (dropout(gen) < 0.1f)
        continue; // 10% dropout
      float azimuth = h * horiz_step * M_PI / 180.0f;
      float range = max_range * range_var(gen);

      float x = range * std::cos(elevation) * std::cos(azimuth) + noise(gen);
      float y = range * std::cos(elevation) * std::sin(azimuth) + noise(gen);
      float z = range * std::sin(elevation) + noise(gen);
      cloud.add(x, y, z);
    }
  }
  return cloud;
}

// =============================================================================
// Normal Estimation Implementations (for benchmarking)
// =============================================================================
constexpr size_t MIN_NEIGHBORS = 3;

struct NormalEstimationResult {
  size_t valid_count = 0;
  size_t invalid_count = 0;
  double build_time_ms = 0;
  double query_time_ms = 0;
  double total_time_ms = 0;
};

// KdTree-based radius normal estimation
NormalEstimationResult estimateNormalsKdTree(PointCloud& cloud, float radius) {
  NormalEstimationResult result;
  cloud.useNormal();
  const size_t n = cloud.size();

  Timer build_timer;
  search::KdTree tree;
  tree.build(cloud);
  result.build_time_ms = build_timer.elapsed_ms();

  size_t valid_count = 0;
  size_t invalid_count = 0;

  Timer query_timer;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+:valid_count, invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    const Point query = cloud.point(i);
    std::vector<uint32_t> indices = tree.radius(query, radius);

    if (indices.size() < MIN_NEIGHBORS) {
      cloud.normals()[i] = Normal4::Zero();
      invalid_count++;
      continue;
    }

    auto pca = geometry::detail::computePCA(cloud, indices);
    if (!pca.valid) {
      cloud.normals()[i] = Normal4::Zero();
      invalid_count++;
      continue;
    }

    Eigen::Vector3f normal = pca.eigenvectors.col(0);
    // Orient towards origin (viewpoint)
    if (normal.dot(-cloud.point(i)) < 0)
      normal = -normal;

    cloud.normals()[i] = Normal4(normal.x(), normal.y(), normal.z(), 0.0f);
    valid_count++;
  }
  result.query_time_ms = query_timer.elapsed_ms();
  result.total_time_ms = result.build_time_ms + result.query_time_ms;
  result.valid_count = valid_count;
  result.invalid_count = invalid_count;

  return result;
}

// VoxelHash-based radius normal estimation
NormalEstimationResult estimateNormalsVoxelHash(PointCloud& cloud, float radius) {
  NormalEstimationResult result;
  cloud.useNormal();
  const size_t n = cloud.size();

  Timer build_timer;
  search::VoxelHash hash(radius);  // resolution = radius
  hash.build(cloud);
  result.build_time_ms = build_timer.elapsed_ms();

  size_t valid_count = 0;
  size_t invalid_count = 0;

  Timer query_timer;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+:valid_count, invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    const Point query = cloud.point(i);
    std::vector<uint32_t> indices = hash.radius(query, radius);

    if (indices.size() < MIN_NEIGHBORS) {
      cloud.normals()[i] = Normal4::Zero();
      invalid_count++;
      continue;
    }

    auto pca = geometry::detail::computePCA(cloud, indices);
    if (!pca.valid) {
      cloud.normals()[i] = Normal4::Zero();
      invalid_count++;
      continue;
    }

    Eigen::Vector3f normal = pca.eigenvectors.col(0);
    if (normal.dot(-cloud.point(i)) < 0)
      normal = -normal;

    cloud.normals()[i] = Normal4(normal.x(), normal.y(), normal.z(), 0.0f);
    valid_count++;
  }
  result.query_time_ms = query_timer.elapsed_ms();
  result.total_time_ms = result.build_time_ms + result.query_time_ms;
  result.valid_count = valid_count;
  result.invalid_count = invalid_count;

  return result;
}

// =============================================================================
// Print Helpers
// =============================================================================
void printHeader(const std::string& title) {
  std::cout << "\n"
            << std::string(80, '=') << "\n"
            << title << "\n"
            << std::string(80, '=') << "\n";
}

void printTableHeader() {
  std::cout << std::left << std::setw(20) << "Method"
            << std::right << std::setw(12) << "Build(ms)"
            << std::setw(12) << "Query(ms)"
            << std::setw(12) << "Total(ms)"
            << std::setw(12) << "Invalid(%)"
            << std::setw(12) << "Speedup"
            << "\n";
  std::cout << std::string(80, '-') << "\n";
}

void printRow(const std::string& name, const NormalEstimationResult& r,
              size_t total_pts, double baseline) {
  double invalid_pct = 100.0 * r.invalid_count / total_pts;
  std::cout << std::left << std::setw(20) << name
            << std::right << std::fixed << std::setprecision(2)
            << std::setw(12) << r.build_time_ms
            << std::setw(12) << r.query_time_ms
            << std::setw(12) << r.total_time_ms
            << std::setw(12) << std::setprecision(1) << invalid_pct
            << std::setw(12) << std::setprecision(2) << (baseline / r.total_time_ms) << "x"
            << "\n";
}

// =============================================================================
// Benchmark: Radius Sensitivity
// =============================================================================
void benchmarkRadiusSensitivity(const LidarSpec& spec, float max_range, int rounds) {
  printHeader("RADIUS SENSITIVITY: " + spec.name + " (max_range=" +
              std::to_string((int)max_range) + "m)");

  PointCloud cloud = generateLidarScan(spec, max_range, 42);
  std::cout << "Cloud size: " << cloud.size() << " points\n\n";

  std::vector<float> radii = {0.1f, 0.2f, 0.3f, 0.5f, 1.0f, 1.5f, 2.0f};

  std::cout << std::left << std::setw(10) << "Radius"
            << std::setw(15) << "KdTree(ms)"
            << std::setw(15) << "VoxelHash(ms)"
            << std::setw(12) << "Winner"
            << std::setw(12) << "Speedup"
            << std::setw(15) << "Invalid(%)"
            << "\n";
  std::cout << std::string(79, '-') << "\n";

  for (float radius : radii) {
    std::vector<double> kd_times, vh_times;
    NormalEstimationResult kd_result, vh_result;

    for (int r = 0; r < rounds; ++r) {
      PointCloud cloud1 = generateLidarScan(spec, max_range, 100 + r);
      PointCloud cloud2 = generateLidarScan(spec, max_range, 100 + r);

      flushCache();
      kd_result = estimateNormalsKdTree(cloud1, radius);
      kd_times.push_back(kd_result.total_time_ms);

      flushCache();
      vh_result = estimateNormalsVoxelHash(cloud2, radius);
      vh_times.push_back(vh_result.total_time_ms);
    }

    auto kd_stats = computeStats(kd_times);
    auto vh_stats = computeStats(vh_times);

    bool kd_wins = kd_stats.mean < vh_stats.mean;
    double speedup = kd_wins ? vh_stats.mean / kd_stats.mean
                             : kd_stats.mean / vh_stats.mean;
    double invalid_pct = 100.0 * kd_result.invalid_count / cloud.size();

    std::cout << std::fixed << std::setprecision(2) << std::left
              << std::setw(10) << radius
              << std::setw(15) << kd_stats.mean
              << std::setw(15) << vh_stats.mean
              << std::setw(12) << (kd_wins ? "KdTree" : "VoxelHash")
              << std::setw(12) << std::setprecision(2) << speedup << "x"
              << std::setw(15) << std::setprecision(1) << invalid_pct
              << "\n";
  }
}

// =============================================================================
// Benchmark: LiDAR Comparison
// =============================================================================
void benchmarkLidarComparison(float radius, int rounds) {
  printHeader("LIDAR COMPARISON (radius=" + std::to_string(radius).substr(0, 4) + "m)");

  std::vector<std::pair<LidarSpec, float>> configs = {
    {VLP16, 30.0f},
    {VLP16, 50.0f},
    {VLP16, 100.0f},
    {OS1_64, 50.0f},
    {OS1_128, 50.0f},
  };

  std::cout << std::left << std::setw(20) << "Config"
            << std::setw(10) << "Points"
            << std::setw(15) << "KdTree(ms)"
            << std::setw(15) << "VoxelHash(ms)"
            << std::setw(12) << "Winner"
            << std::setw(12) << "Speedup"
            << "\n";
  std::cout << std::string(84, '-') << "\n";

  for (const auto& [spec, max_range] : configs) {
    std::string config_name = spec.name + "@" + std::to_string((int)max_range) + "m";

    std::vector<double> kd_times, vh_times;
    size_t pts = 0;

    for (int r = 0; r < rounds; ++r) {
      PointCloud cloud1 = generateLidarScan(spec, max_range, 100 + r);
      PointCloud cloud2 = generateLidarScan(spec, max_range, 100 + r);
      pts = cloud1.size();

      flushCache();
      auto kd_result = estimateNormalsKdTree(cloud1, radius);
      kd_times.push_back(kd_result.total_time_ms);

      flushCache();
      auto vh_result = estimateNormalsVoxelHash(cloud2, radius);
      vh_times.push_back(vh_result.total_time_ms);
    }

    auto kd_stats = computeStats(kd_times);
    auto vh_stats = computeStats(vh_times);

    bool kd_wins = kd_stats.mean < vh_stats.mean;
    double speedup = kd_wins ? vh_stats.mean / kd_stats.mean
                             : kd_stats.mean / vh_stats.mean;

    std::cout << std::fixed << std::setprecision(2) << std::left
              << std::setw(20) << config_name
              << std::setw(10) << pts
              << std::setw(15) << kd_stats.mean
              << std::setw(15) << vh_stats.mean
              << std::setw(12) << (kd_wins ? "KdTree" : "VoxelHash")
              << std::setw(12) << speedup << "x"
              << "\n";
  }
}

// =============================================================================
// Benchmark: Build vs Query Time Breakdown
// =============================================================================
void benchmarkTimeBreakdown(const LidarSpec& spec, float max_range, float radius, int rounds) {
  printHeader("TIME BREAKDOWN: " + spec.name + " (radius=" +
              std::to_string(radius).substr(0, 4) + "m)");

  PointCloud sample = generateLidarScan(spec, max_range, 0);
  std::cout << "Cloud size: " << sample.size() << " points\n\n";

  printTableHeader();

  NormalEstimationResult kd_avg{}, vh_avg{};

  for (int r = 0; r < rounds; ++r) {
    PointCloud cloud1 = generateLidarScan(spec, max_range, 100 + r);
    PointCloud cloud2 = generateLidarScan(spec, max_range, 100 + r);

    flushCache();
    auto kd_result = estimateNormalsKdTree(cloud1, radius);
    kd_avg.build_time_ms += kd_result.build_time_ms;
    kd_avg.query_time_ms += kd_result.query_time_ms;
    kd_avg.total_time_ms += kd_result.total_time_ms;
    kd_avg.invalid_count = kd_result.invalid_count;

    flushCache();
    auto vh_result = estimateNormalsVoxelHash(cloud2, radius);
    vh_avg.build_time_ms += vh_result.build_time_ms;
    vh_avg.query_time_ms += vh_result.query_time_ms;
    vh_avg.total_time_ms += vh_result.total_time_ms;
    vh_avg.invalid_count = vh_result.invalid_count;
  }

  kd_avg.build_time_ms /= rounds;
  kd_avg.query_time_ms /= rounds;
  kd_avg.total_time_ms /= rounds;
  vh_avg.build_time_ms /= rounds;
  vh_avg.query_time_ms /= rounds;
  vh_avg.total_time_ms /= rounds;

  double baseline = std::max(kd_avg.total_time_ms, vh_avg.total_time_ms);
  printRow("KdTree", kd_avg, sample.size(), baseline);
  printRow("VoxelHash", vh_avg, sample.size(), baseline);

  std::cout << "\nAnalysis:\n";
  std::cout << "  KdTree   - Build: " << std::fixed << std::setprecision(1)
            << (100.0 * kd_avg.build_time_ms / kd_avg.total_time_ms) << "%, "
            << "Query: " << (100.0 * kd_avg.query_time_ms / kd_avg.total_time_ms) << "%\n";
  std::cout << "  VoxelHash - Build: "
            << (100.0 * vh_avg.build_time_ms / vh_avg.total_time_ms) << "%, "
            << "Query: " << (100.0 * vh_avg.query_time_ms / vh_avg.total_time_ms) << "%\n";
}

// =============================================================================
// Benchmark: Neighbor Count Statistics
// =============================================================================
void benchmarkNeighborStats(const LidarSpec& spec, float max_range, float radius) {
  printHeader("NEIGHBOR COUNT STATISTICS: " + spec.name);

  PointCloud cloud = generateLidarScan(spec, max_range, 42);
  std::cout << "Cloud size: " << cloud.size() << " points, radius: " << radius << "m\n\n";

  search::KdTree tree;
  tree.build(cloud);

  std::vector<size_t> neighbor_counts;
  neighbor_counts.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    auto indices = tree.radius(cloud.point(i), radius);
    neighbor_counts.push_back(indices.size());
  }

  std::sort(neighbor_counts.begin(), neighbor_counts.end());

  size_t n = neighbor_counts.size();
  double sum = std::accumulate(neighbor_counts.begin(), neighbor_counts.end(), 0.0);

  std::cout << "Neighbor count distribution:\n";
  std::cout << "  Min:    " << neighbor_counts[0] << "\n";
  std::cout << "  P10:    " << neighbor_counts[n * 10 / 100] << "\n";
  std::cout << "  P25:    " << neighbor_counts[n * 25 / 100] << "\n";
  std::cout << "  Median: " << neighbor_counts[n / 2] << "\n";
  std::cout << "  P75:    " << neighbor_counts[n * 75 / 100] << "\n";
  std::cout << "  P90:    " << neighbor_counts[n * 90 / 100] << "\n";
  std::cout << "  Max:    " << neighbor_counts[n - 1] << "\n";
  std::cout << "  Mean:   " << std::fixed << std::setprecision(1) << (sum / n) << "\n";

  size_t invalid = std::count_if(neighbor_counts.begin(), neighbor_counts.end(),
                                  [](size_t c) { return c < MIN_NEIGHBORS; });
  std::cout << "  Invalid (<" << MIN_NEIGHBORS << "): " << invalid
            << " (" << std::setprecision(2) << (100.0 * invalid / n) << "%)\n";
}

// =============================================================================
// Benchmark: Sparse vs Dense (by range)
// =============================================================================
void benchmarkSparsityEffect(const LidarSpec& spec, float radius, int rounds) {
  printHeader("SPARSITY EFFECT: " + spec.name + " (radius=" +
              std::to_string(radius).substr(0, 4) + "m)");

  std::cout << "Testing different max ranges (farther = sparser)\n\n";

  std::vector<float> ranges = {20.0f, 50.0f, 100.0f, 150.0f};

  std::cout << std::left << std::setw(15) << "MaxRange"
            << std::setw(10) << "Points"
            << std::setw(15) << "KdTree(ms)"
            << std::setw(15) << "VoxelHash(ms)"
            << std::setw(12) << "Winner"
            << std::setw(15) << "Invalid(%)"
            << "\n";
  std::cout << std::string(82, '-') << "\n";

  for (float max_range : ranges) {
    std::vector<double> kd_times, vh_times;
    size_t pts = 0;
    size_t invalid = 0;

    for (int r = 0; r < rounds; ++r) {
      PointCloud cloud1 = generateLidarScan(spec, max_range, 100 + r);
      PointCloud cloud2 = generateLidarScan(spec, max_range, 100 + r);
      pts = cloud1.size();

      flushCache();
      auto kd_result = estimateNormalsKdTree(cloud1, radius);
      kd_times.push_back(kd_result.total_time_ms);
      invalid = kd_result.invalid_count;

      flushCache();
      auto vh_result = estimateNormalsVoxelHash(cloud2, radius);
      vh_times.push_back(vh_result.total_time_ms);
    }

    auto kd_stats = computeStats(kd_times);
    auto vh_stats = computeStats(vh_times);

    bool kd_wins = kd_stats.mean < vh_stats.mean;
    double invalid_pct = 100.0 * invalid / pts;

    std::cout << std::fixed << std::setprecision(2) << std::left
              << std::setw(15) << (std::to_string((int)max_range) + "m")
              << std::setw(10) << pts
              << std::setw(15) << kd_stats.mean
              << std::setw(15) << vh_stats.mean
              << std::setw(12) << (kd_wins ? "KdTree" : "VoxelHash")
              << std::setw(15) << std::setprecision(1) << invalid_pct
              << "\n";
  }
}

// =============================================================================
// Main
// =============================================================================
int main() {
  initCacheFlushBuffer();

  std::cout << "╔══════════════════════════════════════════════════════════════════════════════╗\n";
  std::cout << "║     nanoPCL Benchmark: Radius-based Normal Estimation                        ║\n";
  std::cout << "║     KdTree vs VoxelHash                                                      ║\n";
  std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";

#ifdef _OPENMP
  std::cout << "\nOpenMP enabled: " << omp_get_max_threads() << " threads\n";
#else
  std::cout << "\nOpenMP disabled (single-threaded)\n";
#endif

  const int ROUNDS = 5;

  // 1. Neighbor statistics (to understand the data)
  std::cout << "\n>>> NEIGHBOR STATISTICS\n";
  benchmarkNeighborStats(VLP16, 50.0f, 0.5f);
  benchmarkNeighborStats(VLP16, 50.0f, 1.0f);

  // 2. Radius sensitivity
  std::cout << "\n>>> RADIUS SENSITIVITY\n";
  benchmarkRadiusSensitivity(VLP16, 50.0f, ROUNDS);

  // 3. LiDAR comparison
  std::cout << "\n>>> LIDAR SENSOR COMPARISON\n";
  benchmarkLidarComparison(0.5f, ROUNDS);

  // 4. Time breakdown
  std::cout << "\n>>> TIME BREAKDOWN\n";
  benchmarkTimeBreakdown(VLP16, 50.0f, 0.5f, ROUNDS);
  benchmarkTimeBreakdown(OS1_64, 50.0f, 0.5f, ROUNDS);

  // 5. Sparsity effect
  std::cout << "\n>>> SPARSITY EFFECT (range variation)\n";
  benchmarkSparsityEffect(VLP16, 0.5f, ROUNDS);

  // Summary
  std::cout << "\n"
            << std::string(80, '=') << "\n";
  std::cout << "SUMMARY & RECOMMENDATIONS\n";
  std::cout << std::string(80, '=') << "\n";
  std::cout << R"(
Key factors for choosing between KdTree and VoxelHash:

1. Build Time:
   - VoxelHash: O(n) - faster build
   - KdTree: O(n log n) - slower build

2. Query Time (radius search):
   - VoxelHash: O(1) lookup but O(r³/res³) voxels to check
   - KdTree: O(log n + k) where k = neighbors found

3. When radius >> voxel_resolution:
   - VoxelHash degrades (many voxels to check)
   - KdTree more stable

4. LiDAR characteristics:
   - Near range: dense → more neighbors → VoxelHash may struggle with many voxels
   - Far range: sparse → fewer neighbors → both should be fast

RECOMMENDATION:
- Small radius (< 0.5m): Benchmark your specific data
- Large radius (> 1.0m): Prefer KdTree
- One-shot estimation: Consider build cost
- Repeated queries (same structure): Query cost dominates

)" << "\n";

  return 0;
}
