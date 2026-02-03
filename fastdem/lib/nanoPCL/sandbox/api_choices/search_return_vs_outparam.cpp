// nanoPCL Benchmark: Search API - Return Value (RVO) vs Out Parameter
// Realistic benchmark using actual VoxelHash with real point cloud data
//
// Scenario: Height Mapping application
// - LiDAR point cloud (100k - 1M points)
// - Grid-based queries (thousands of radius searches)
// - Variable result sizes based on local density

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>
#include <vector>

#include <nanopcl/core/point_cloud.hpp>
#include <nanopcl/search.hpp>

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

// =============================================================================
// Statistics helper
// =============================================================================
struct Stats {
  double mean;
  double stddev;
  double min;
  double max;
};

Stats computeStats(const std::vector<double>& data) {
  if (data.empty())
    return {0, 0, 0, 0};

  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();

  double sq_sum = 0;
  for (double v : data) {
    sq_sum += (v - mean) * (v - mean);
  }
  double stddev = std::sqrt(sq_sum / data.size());

  auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());

  return {mean, stddev, *min_it, *max_it};
}

// =============================================================================
// Point Cloud Generator (Simulated LiDAR)
// =============================================================================
PointCloud generateLidarCloud(size_t num_points, float range, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> xy_dist(-range, range);
  std::uniform_real_distribution<float> z_dist(-2.0f, 3.0f); // Ground to above

  PointCloud cloud("base_link");
  cloud.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(Point(xy_dist(gen), xy_dist(gen), z_dist(gen)));
  }

  return cloud;
}

// Generate grid query points (like height map cells)
std::vector<Point> generateGridQueries(float range, float resolution) {
  std::vector<Point> queries;
  int cells = static_cast<int>(2 * range / resolution);

  for (int i = 0; i < cells; ++i) {
    for (int j = 0; j < cells; ++j) {
      float x = -range + (i + 0.5f) * resolution;
      float y = -range + (j + 0.5f) * resolution;
      queries.emplace_back(x, y, 0.0f);
    }
  }

  return queries;
}

// =============================================================================
// Search API Variants
// =============================================================================

// API 1: Out parameter (reusable vector)
size_t searchOutParam(const search::VoxelHash& searcher,
                      const Point& query,
                      float radius,
                      std::vector<uint32_t>& out) {
  searcher.radius(query, radius, out);
  return out.size();
}

// API 2: Return value (RVO)
std::vector<uint32_t> searchReturn(const search::VoxelHash& searcher,
                                   const Point& query,
                                   float radius) {
  return searcher.radius(query, radius);
}

// =============================================================================
// Benchmark Infrastructure
// =============================================================================

void printHeader(const std::string& scenario, size_t cloud_size, size_t num_queries, float radius) {
  std::cout << "\n"
            << std::string(75, '=') << "\n";
  std::cout << scenario << "\n";
  std::cout << "Cloud: " << cloud_size << " pts, "
            << "Queries: " << num_queries << ", "
            << "Radius: " << radius << "m\n";
  std::cout << std::string(75, '=') << "\n";
}

void printResults(const std::vector<std::string>& names,
                  const std::vector<Stats>& stats,
                  const std::vector<size_t>& total_results) {
  std::cout << std::left << std::setw(35) << "Method" << std::right
            << std::setw(12) << "Mean(ms)" << std::setw(10) << "Stddev"
            << std::setw(15) << "Results/query"
            << "\n";
  std::cout << std::string(75, '-') << "\n";

  double baseline = stats[0].mean;
  for (size_t i = 0; i < names.size(); ++i) {
    double ratio = baseline / stats[i].mean;
    std::string ratio_str;
    if (std::abs(ratio - 1.0) < 0.05) {
      ratio_str = "~1.0x";
    } else if (ratio >= 1.0) {
      ratio_str = std::to_string(ratio).substr(0, 4) + "x";
    } else {
      ratio_str = std::to_string(1.0 / ratio).substr(0, 4) + "x slower";
    }

    std::cout << std::left << std::setw(35) << names[i] << std::right
              << std::fixed << std::setprecision(2) << std::setw(12)
              << stats[i].mean << std::setw(10) << stats[i].stddev
              << std::setw(12) << "~" << (total_results[i] > 0 ? std::to_string(total_results[i] / 30) : "0") // avg over rounds
              << "  " << ratio_str << "\n";
  }
}

// =============================================================================
// Main Benchmark
// =============================================================================
void runRealisticBenchmark(size_t cloud_size, float cloud_range, float grid_resolution, float search_radius, int rounds) {
  // Generate data
  PointCloud cloud = generateLidarCloud(cloud_size, cloud_range);
  std::vector<Point> queries = generateGridQueries(cloud_range, grid_resolution);

  // Build search index
  search::VoxelHash searcher(search_radius);
  searcher.build(cloud);

  printHeader("Height Mapping Scenario", cloud_size, queries.size(), search_radius);

  const int NUM_METHODS = 3;
  std::vector<std::string> names = {
      "Out param (reuse vector)",
      "Out param (new each query)",
      "Return value (RVO)",
  };
  std::vector<std::vector<double>> times(NUM_METHODS);
  std::vector<size_t> total_results(NUM_METHODS, 0);

  // Warmup
  {
    std::vector<uint32_t> warmup;
    for (size_t i = 0; i < std::min(queries.size(), size_t(100)); ++i) {
      searcher.radius(queries[i], search_radius, warmup);
    }
  }

  // Interleaved benchmark rounds
  for (int round = 0; round < rounds; ++round) {
    size_t result_count = 0;

    // Method 0: Out param with reuse
    {
      std::vector<uint32_t> indices;
      indices.reserve(500); // Pre-allocate reasonable size

      Timer t;
      for (const auto& query : queries) {
        searchOutParam(searcher, query, search_radius, indices);
        result_count += indices.size();
      }
      times[0].push_back(t.elapsed_ms());
      total_results[0] += result_count;
    }
    doNotOptimize(result_count);

    // Method 1: Out param without reuse
    {
      result_count = 0;
      Timer t;
      for (const auto& query : queries) {
        std::vector<uint32_t> indices;
        searchOutParam(searcher, query, search_radius, indices);
        result_count += indices.size();
      }
      times[1].push_back(t.elapsed_ms());
      total_results[1] += result_count;
    }
    doNotOptimize(result_count);

    // Method 2: Return value (RVO)
    {
      result_count = 0;
      Timer t;
      for (const auto& query : queries) {
        auto indices = searchReturn(searcher, query, search_radius);
        result_count += indices.size();
      }
      times[2].push_back(t.elapsed_ms());
      total_results[2] += result_count;
    }
    doNotOptimize(result_count);
  }

  // Compute and print stats
  std::vector<Stats> stats;
  for (int i = 0; i < NUM_METHODS; ++i) {
    stats.push_back(computeStats(times[i]));
  }

  // Normalize total_results by rounds for display
  for (auto& tr : total_results) {
    tr /= rounds;
  }

  printResults(names, stats, total_results);

  // Additional info
  size_t avg_results = total_results[0] / queries.size();
  std::cout << "\nAvg results per query: " << avg_results << " points\n";
}

// =============================================================================
// Main
// =============================================================================
int main() {
  std::cout << "nanoPCL Benchmark: Search API - Realistic Height Mapping Scenario\n";
  std::cout << std::string(75, '=') << "\n";
  std::cout << R"(
Scenario: Height Mapping with LiDAR
  - Point cloud simulating LiDAR scan
  - Grid-based queries (like height map cells)
  - Variable result sizes based on local density

Key Questions:
  1. How much does vector reuse help in real applications?
  2. Is RVO sufficient for practical use cases?
)"
            << "\n";

  const int ROUNDS = 30;

  // Scenario 1: Small area, high density (indoor/close range)
  std::cout << "\n>>> SCENARIO 1: Small area, high density (indoor)\n";
  runRealisticBenchmark(
      100000, // 100k points
      10.0f,  // 10m range
      0.5f,   // 0.5m grid resolution (400 queries)
      0.3f,   // 0.3m search radius
      ROUNDS);

  // Scenario 2: Medium area, medium density (typical outdoor)
  std::cout << "\n>>> SCENARIO 2: Medium area, medium density (outdoor)\n";
  runRealisticBenchmark(
      300000, // 300k points
      25.0f,  // 25m range
      0.5f,   // 0.5m grid resolution (2500 queries)
      0.5f,   // 0.5m search radius
      ROUNDS);

  // Scenario 3: Large area, sparse (long range LiDAR)
  std::cout << "\n>>> SCENARIO 3: Large area, sparse (long range)\n";
  runRealisticBenchmark(
      500000, // 500k points
      50.0f,  // 50m range
      1.0f,   // 1.0m grid resolution (2500 queries)
      1.0f,   // 1.0m search radius
      ROUNDS);

  // Summary
  std::cout << "\n"
            << std::string(75, '=') << "\n";
  std::cout << "CONCLUSION\n";
  std::cout << std::string(75, '=') << "\n";
  std::cout << "\nInterpretation:\n"
            << "  - Out param (reuse): best-case with vector reuse\n"
            << "  - Out param (new) and Return (RVO): should be similar\n"
            << "  - Difference shows malloc overhead in real scenario\n"
            << "\nAPI Recommendation:\n"
            << "  - If reuse >> RVO: Provide both APIs\n"
            << "  - If reuse ~= RVO: Return-only API is sufficient\n";

  return 0;
}
