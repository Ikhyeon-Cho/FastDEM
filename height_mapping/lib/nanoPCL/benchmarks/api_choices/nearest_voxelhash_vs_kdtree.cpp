// nanoPCL Benchmark: Nearest Search (1-NN) - Realistic Scenarios
// VoxelHash vs KdTree with simulated LiDAR data

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

using namespace npcl;

// =============================================================================
// Timer & Utilities
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
  asm volatile("" : : "g"(&value) : "memory");
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
    for (auto& b : g_cache_flush_buffer) b = dist(gen);
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
  if (data.empty()) return {0, 0, 0, 0};
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();
  double sq_sum = 0;
  for (double v : data) sq_sum += (v - mean) * (v - mean);
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

  PointCloud cloud("lidar");
  cloud.reserve(spec.num_rings * spec.points_per_ring);

  float vert_step = (spec.vertical_fov_max - spec.vertical_fov_min) / (spec.num_rings - 1);
  float horiz_step = 360.0f / spec.points_per_ring;

  for (int ring = 0; ring < spec.num_rings; ++ring) {
    float elevation = (spec.vertical_fov_min + ring * vert_step) * M_PI / 180.0f;
    for (int h = 0; h < spec.points_per_ring; ++h) {
      if (dropout(gen) < 0.1f) continue;  // 10% dropout
      float azimuth = h * horiz_step * M_PI / 180.0f;
      float range = max_range * range_var(gen);

      float x = range * std::cos(elevation) * std::cos(azimuth) + noise(gen);
      float y = range * std::cos(elevation) * std::sin(azimuth) + noise(gen);
      float z = range * std::sin(elevation) + noise(gen);
      cloud.add(Point(x, y, z));
    }
  }
  return cloud;
}

PointCloud generateNextScan(const LidarSpec& spec, float max_range,
                            float dx, float dy, float dyaw, unsigned seed) {
  PointCloud cloud = generateLidarScan(spec, max_range, seed);
  float cos_yaw = std::cos(dyaw), sin_yaw = std::sin(dyaw);
  for (size_t i = 0; i < cloud.size(); ++i) {
    Point& p = cloud[i];
    p = Point(cos_yaw * p.x() - sin_yaw * p.y() + dx,
              sin_yaw * p.x() + cos_yaw * p.y() + dy,
              p.z());
  }
  return cloud;
}

// =============================================================================
// Print Helpers
// =============================================================================
void printHeader(const std::string& title) {
  std::cout << "\n" << std::string(70, '=') << "\n" << title << "\n"
            << std::string(70, '=') << "\n";
}

void printRow(const std::string& name, const Stats& s, double baseline) {
  std::cout << std::left << std::setw(30) << name << std::right << std::fixed
            << std::setprecision(2) << std::setw(10) << s.mean << std::setw(10)
            << s.max << std::setw(10) << (baseline / s.mean) << "x\n";
}

// =============================================================================
// Scenario 1: Frame-to-Frame ICP
// =============================================================================
void benchmarkFrameToFrame(const LidarSpec& spec, float max_range,
                           float search_radius, int rounds) {
  printHeader("FRAME-TO-FRAME ICP: " + spec.name + " (r=" +
              std::to_string(search_radius).substr(0, 4) + "m)");

  PointCloud sample = generateLidarScan(spec, max_range, 0);
  std::cout << "Points/scan: ~" << sample.size() << "\n\n";

  std::cout << std::left << std::setw(30) << "Method" << std::right
            << std::setw(10) << "Mean(ms)" << std::setw(10) << "Max(ms)"
            << std::setw(10) << "Speedup" << "\n";
  std::cout << std::string(60, '-') << "\n";

  std::vector<double> voxel_times, kdtree_times;

  for (int r = 0; r < rounds; ++r) {
    PointCloud source = generateLidarScan(spec, max_range, 100 + r);
    PointCloud target = generateNextScan(spec, max_range, 0.1f, 0.05f, 0.02f, 200 + r);

    flushCache();
    {
      Timer t;
      search::VoxelHash searcher(search_radius);
      searcher.build(target);
      size_t found = 0;
      for (size_t i = 0; i < source.size(); ++i) {
        if (searcher.nearest(source[i], search_radius)) found++;
      }
      voxel_times.push_back(t.elapsed_ms());
      doNotOptimize(found);
    }

    flushCache();
    {
      Timer t;
      search::KdTree searcher;
      searcher.build(target);
      size_t found = 0;
      for (size_t i = 0; i < source.size(); ++i) {
        if (searcher.nearest(source[i], search_radius)) found++;
      }
      kdtree_times.push_back(t.elapsed_ms());
      doNotOptimize(found);
    }
  }

  auto vs = computeStats(voxel_times);
  auto ks = computeStats(kdtree_times);
  printRow("VoxelHash (build+query)", vs, vs.mean);
  printRow("KdTree (build+query)", ks, vs.mean);
}

// =============================================================================
// Scenario 2: Scan-to-Map
// =============================================================================
void benchmarkScanToMap(const LidarSpec& spec, size_t map_scans,
                        float max_range, float search_radius, int rounds) {
  printHeader("SCAN-TO-MAP: " + spec.name + " (map=" + std::to_string(map_scans) +
              " scans, r=" + std::to_string(search_radius).substr(0, 4) + "m)");

  // Build map
  PointCloud map("map");
  for (size_t i = 0; i < map_scans; ++i) {
    PointCloud scan = generateNextScan(spec, max_range, i * 0.5f, i * 0.1f, i * 0.05f, 1000 + i);
    for (size_t j = 0; j < scan.size(); ++j) map.add(scan[j]);
  }
  std::cout << "Map: " << map.size() << " pts, Query: ~"
            << (spec.num_rings * spec.points_per_ring * 0.9) << " pts\n\n";

  // Build once
  search::VoxelHash voxel_searcher(search_radius);
  search::KdTree kdtree_searcher;

  Timer tb1; voxel_searcher.build(map); double vb = tb1.elapsed_ms();
  Timer tb2; kdtree_searcher.build(map); double kb = tb2.elapsed_ms();
  std::cout << "Build: VoxelHash=" << std::fixed << std::setprecision(1)
            << vb << "ms, KdTree=" << kb << "ms\n\n";

  std::cout << std::left << std::setw(30) << "Method" << std::right
            << std::setw(10) << "Mean(ms)" << std::setw(10) << "Max(ms)"
            << std::setw(10) << "Speedup" << "\n";
  std::cout << std::string(60, '-') << "\n";

  std::vector<double> voxel_times, kdtree_times;

  for (int r = 0; r < rounds; ++r) {
    PointCloud scan = generateLidarScan(spec, max_range, 5000 + r);

    flushCache();
    {
      Timer t;
      size_t found = 0;
      for (size_t i = 0; i < scan.size(); ++i) {
        if (voxel_searcher.nearest(scan[i], search_radius)) found++;
      }
      voxel_times.push_back(t.elapsed_ms());
      doNotOptimize(found);
    }

    flushCache();
    {
      Timer t;
      size_t found = 0;
      for (size_t i = 0; i < scan.size(); ++i) {
        if (kdtree_searcher.nearest(scan[i], search_radius)) found++;
      }
      kdtree_times.push_back(t.elapsed_ms());
      doNotOptimize(found);
    }
  }

  auto vs = computeStats(voxel_times);
  auto ks = computeStats(kdtree_times);
  printRow("VoxelHash (query only)", vs, vs.mean);
  printRow("KdTree (query only)", ks, vs.mean);
}

// =============================================================================
// Scenario 3: Radius Sensitivity
// =============================================================================
void benchmarkRadiusSensitivity(const LidarSpec& spec, float max_range, int rounds) {
  printHeader("RADIUS SENSITIVITY: " + spec.name);

  PointCloud cloud = generateLidarScan(spec, max_range, 42);
  std::cout << "Cloud: " << cloud.size() << " pts\n\n";

  std::vector<float> radii = {0.3f, 0.5f, 1.0f, 1.5f, 2.0f};

  std::cout << std::left << std::setw(10) << "Radius"
            << std::setw(15) << "VoxelHash"
            << std::setw(15) << "KdTree"
            << std::setw(10) << "Ratio" << "\n";
  std::cout << std::string(50, '-') << "\n";

  for (float radius : radii) {
    search::VoxelHash vs(radius); vs.build(cloud);
    search::KdTree ks; ks.build(cloud);

    std::vector<double> vt, kt;
    for (int r = 0; r < rounds; ++r) {
      flushCache();
      { Timer t; size_t f = 0;
        for (size_t i = 0; i < cloud.size(); ++i) if (vs.nearest(cloud[i], radius)) f++;
        vt.push_back(t.elapsed_ms()); doNotOptimize(f); }

      flushCache();
      { Timer t; size_t f = 0;
        for (size_t i = 0; i < cloud.size(); ++i) if (ks.nearest(cloud[i], radius)) f++;
        kt.push_back(t.elapsed_ms()); doNotOptimize(f); }
    }

    auto vss = computeStats(vt), kss = computeStats(kt);
    std::cout << std::fixed << std::setprecision(1) << std::left
              << std::setw(10) << radius
              << std::setw(15) << vss.mean
              << std::setw(15) << kss.mean
              << std::setw(10) << std::setprecision(2)
              << (vss.mean / kss.mean) << "x\n";
  }
}

// =============================================================================
// Main
// =============================================================================
int main() {
  initCacheFlushBuffer();

  std::cout << "nanoPCL Benchmark: Nearest Search - Realistic LiDAR Scenarios\n";
  std::cout << std::string(70, '=') << "\n\n";

  std::cout << "Sensors: VLP-16 (~26K pts), OS1-64 (~59K pts), OS1-128 (~118K pts)\n\n";

  const int ROUNDS = 10;

  // Frame-to-Frame
  std::cout << ">>> FRAME-TO-FRAME ICP (build every frame)\n";
  benchmarkFrameToFrame(VLP16, 50.0f, 0.5f, ROUNDS);
  benchmarkFrameToFrame(OS1_64, 50.0f, 0.5f, ROUNDS);
  benchmarkFrameToFrame(OS1_128, 50.0f, 0.5f, ROUNDS);

  // Scan-to-Map
  std::cout << "\n>>> SCAN-TO-MAP (static map, query only)\n";
  benchmarkScanToMap(VLP16, 10, 50.0f, 1.0f, ROUNDS);
  benchmarkScanToMap(OS1_64, 10, 50.0f, 1.0f, ROUNDS);

  // Radius Sensitivity
  std::cout << "\n>>> RADIUS SENSITIVITY\n";
  benchmarkRadiusSensitivity(VLP16, 50.0f, 5);

  // Summary
  std::cout << "\n" << std::string(70, '=') << "\n";
  std::cout << "SUMMARY\n" << std::string(70, '=') << "\n";
  std::cout << R"(
Frame-to-Frame ICP: Build cost matters → check which wins
Scan-to-Map:        Query cost dominates → KdTree likely better
Radius < 0.5m:      VoxelHash competitive
Radius > 1.0m:      KdTree significantly faster
)" << "\n";

  return 0;
}
