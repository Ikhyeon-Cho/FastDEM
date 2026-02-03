// nanoPCL Benchmark: Search API - Callback vs Vector
// Compares zero-allocation callback approach vs vector-based approach
//
// Key insight: Callback avoids vector allocation AND index-to-point lookup
//
// Scenario: Normal Estimation (covariance accumulation)
// - Many radius searches (one per point)
// - Results used immediately for computation, not stored
// - This is where callback shines

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
// Cache control
// =============================================================================

// Large buffer to flush CPU cache (should be larger than L3 cache)
// Typical L3: 8-32MB, we use 64MB to be safe
constexpr size_t CACHE_FLUSH_SIZE = 64 * 1024 * 1024; // 64MB
static std::vector<uint8_t> g_cache_flush_buffer;

void initCacheFlushBuffer() {
  if (g_cache_flush_buffer.empty()) {
    g_cache_flush_buffer.resize(CACHE_FLUSH_SIZE);
    // Initialize with random data to prevent any clever optimization
    std::mt19937 gen(12345);
    std::uniform_int_distribution<uint8_t> dist(0, 255);
    for (auto& b : g_cache_flush_buffer) {
      b = dist(gen);
    }
  }
}

// Flush cache by reading large buffer (pollutes cache lines)
volatile uint64_t g_cache_sink = 0;
void flushCache() {
  uint64_t sum = 0;
  for (size_t i = 0; i < g_cache_flush_buffer.size(); i += 64) { // cache line = 64B
    sum += g_cache_flush_buffer[i];
  }
  g_cache_sink = sum; // Prevent optimization
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
// Point Cloud Generator
// =============================================================================
PointCloud generateCloud(size_t num_points, float range, unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> xy_dist(-range, range);
  std::uniform_real_distribution<float> z_dist(-2.0f, 3.0f);

  PointCloud cloud("base_link");
  cloud.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(Point(xy_dist(gen), xy_dist(gen), z_dist(gen)));
  }

  return cloud;
}

// =============================================================================
// Covariance Accumulator (simulates Normal Estimation workload)
// =============================================================================
struct CovarianceAccumulator {
  Point sum = Point::Zero();
  float cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
  size_t count = 0;

  void add(const Point& pt) {
    sum += pt;
    cxx += pt.x() * pt.x();
    cxy += pt.x() * pt.y();
    cxz += pt.x() * pt.z();
    cyy += pt.y() * pt.y();
    cyz += pt.y() * pt.z();
    czz += pt.z() * pt.z();
    count++;
  }

  void reset() {
    sum = Point::Zero();
    cxx = cxy = cxz = cyy = cyz = czz = 0;
    count = 0;
  }

  // Compute covariance trace (just to use the result)
  float trace() const {
    if (count < 3)
      return 0;
    float n = static_cast<float>(count);
    Point mean = sum / n;
    float var_x = cxx / n - mean.x() * mean.x();
    float var_y = cyy / n - mean.y() * mean.y();
    float var_z = czz / n - mean.z() * mean.z();
    return var_x + var_y + var_z;
  }
};

// =============================================================================
// Search Methods
// =============================================================================

// Method 1: Vector-based (traditional approach)
float searchVector(const PointCloud& cloud, const search::VoxelHash& searcher, const std::vector<Point>& queries, float radius, std::vector<uint32_t>& indices_buffer) {
  CovarianceAccumulator acc;
  float total_trace = 0;

  for (const auto& query : queries) {
    acc.reset();

    // Get indices
    searcher.radius(query, radius, indices_buffer);

    // Lookup points by index and accumulate
    for (uint32_t idx : indices_buffer) {
      acc.add(cloud[idx]);
    }

    total_trace += acc.trace();
  }

  return total_trace;
}

// Method 2: Callback-based (zero-allocation)
float searchCallback(const PointCloud& cloud, const search::VoxelHash& searcher, const std::vector<Point>& queries, float radius) {
  (void)cloud; // Not needed - callback receives point directly

  float total_trace = 0;

  for (const auto& query : queries) {
    CovarianceAccumulator acc;

    // Direct accumulation - no vector, no index lookup
    searcher.radius(query, radius, [&acc](uint32_t /*idx*/, const Point& pt, float /*dist_sq*/) {
      acc.add(pt);
    });

    total_trace += acc.trace();
  }

  return total_trace;
}

// Method 3: Callback with std::ref (to see if function wrapper has overhead)
float searchCallbackRef(const PointCloud& cloud, const search::VoxelHash& searcher, const std::vector<Point>& queries, float radius) {
  (void)cloud;

  float total_trace = 0;

  for (const auto& query : queries) {
    struct Acc {
      Point sum = Point::Zero();
      float cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
      size_t count = 0;

      void operator()(uint32_t, const Point& pt, float) {
        sum += pt;
        cxx += pt.x() * pt.x();
        cxy += pt.x() * pt.y();
        cxz += pt.x() * pt.z();
        cyy += pt.y() * pt.y();
        cyz += pt.y() * pt.z();
        czz += pt.z() * pt.z();
        count++;
      }

      float trace() const {
        if (count < 3)
          return 0;
        float n = static_cast<float>(count);
        Point mean = sum / n;
        return (cxx / n - mean.x() * mean.x()) +
               (cyy / n - mean.y() * mean.y()) +
               (czz / n - mean.z() * mean.z());
      }
    } acc;

    searcher.radius(query, radius, std::ref(acc));
    total_trace += acc.trace();
  }

  return total_trace;
}

// =============================================================================
// Benchmark Runner
// =============================================================================
void printHeader(const std::string& scenario, size_t cloud_size, size_t num_queries, float radius, const std::string& cache_state) {
  std::cout << "\n"
            << std::string(80, '=') << "\n";
  std::cout << scenario << " [" << cache_state << "]\n";
  std::cout << "Cloud: " << cloud_size << " pts, "
            << "Queries: " << num_queries << ", "
            << "Radius: " << radius << "m\n";
  std::cout << std::string(80, '=') << "\n";
}

void printResults(const std::vector<std::string>& names,
                  const std::vector<Stats>& stats) {
  std::cout << std::left << std::setw(40) << "Method" << std::right
            << std::setw(12) << "Mean(ms)" << std::setw(12) << "Stddev"
            << std::setw(12) << "Min" << std::setw(12) << "Max"
            << std::setw(10) << "Speedup"
            << "\n";
  std::cout << std::string(80, '-') << "\n";

  double baseline = stats[0].mean;
  for (size_t i = 0; i < names.size(); ++i) {
    double speedup = baseline / stats[i].mean;

    std::cout << std::left << std::setw(40) << names[i] << std::right
              << std::fixed << std::setprecision(2) << std::setw(12)
              << stats[i].mean << std::setw(12) << stats[i].stddev
              << std::setw(12) << stats[i].min << std::setw(12) << stats[i].max
              << std::setw(10) << speedup << "x\n";
  }
}

void runBenchmark(size_t cloud_size, float cloud_range, size_t num_queries, float search_radius, int rounds, bool cold_cache) {
  // Generate data
  PointCloud cloud = generateCloud(cloud_size, cloud_range);

  // Generate random query points (subset of cloud)
  std::vector<Point> queries;
  queries.reserve(num_queries);
  std::mt19937 gen(12345);
  std::uniform_int_distribution<size_t> idx_dist(0, cloud.size() - 1);
  for (size_t i = 0; i < num_queries; ++i) {
    queries.push_back(cloud[idx_dist(gen)]);
  }

  // Build search index
  search::VoxelHash searcher(search_radius);
  searcher.build(cloud);

  printHeader("Normal Estimation Workload", cloud_size, num_queries, search_radius, cold_cache ? "COLD CACHE" : "WARM CACHE");

  const int NUM_METHODS = 3;
  std::vector<std::string> names = {
      "Vector (indices -> lookup)",
      "Callback (lambda)",
      "Callback (std::ref functor)",
  };
  std::vector<std::vector<double>> times(NUM_METHODS);

  // Pre-allocate buffer for vector method
  std::vector<uint32_t> indices_buffer;
  indices_buffer.reserve(1000);

  // Warmup (only for warm cache scenario)
  if (!cold_cache) {
    float warmup_result = 0;
    warmup_result += searchVector(cloud, searcher, queries, search_radius, indices_buffer);
    warmup_result += searchCallback(cloud, searcher, queries, search_radius);
    warmup_result += searchCallbackRef(cloud, searcher, queries, search_radius);
    doNotOptimize(warmup_result);
  }

  // Benchmark rounds
  for (int round = 0; round < rounds; ++round) {
    float result;

    // Method 0: Vector-based
    if (cold_cache)
      flushCache();
    {
      Timer t;
      result = searchVector(cloud, searcher, queries, search_radius, indices_buffer);
      times[0].push_back(t.elapsed_ms());
    }
    doNotOptimize(result);

    // Method 1: Callback lambda
    if (cold_cache)
      flushCache();
    {
      Timer t;
      result = searchCallback(cloud, searcher, queries, search_radius);
      times[1].push_back(t.elapsed_ms());
    }
    doNotOptimize(result);

    // Method 2: Callback std::ref
    if (cold_cache)
      flushCache();
    {
      Timer t;
      result = searchCallbackRef(cloud, searcher, queries, search_radius);
      times[2].push_back(t.elapsed_ms());
    }
    doNotOptimize(result);
  }

  // Compute and print stats
  std::vector<Stats> stats;
  for (int i = 0; i < NUM_METHODS; ++i) {
    stats.push_back(computeStats(times[i]));
  }

  printResults(names, stats);
}

// =============================================================================
// Main
// =============================================================================
int main() {
  initCacheFlushBuffer();

  std::cout << "nanoPCL Benchmark: Search Callback vs Vector\n";
  std::cout << std::string(80, '=') << "\n";
  std::cout << R"(
Scenario: Normal Estimation (covariance accumulation per query point)

Methods compared:
  1. Vector: searcher.radius() -> indices -> cloud[idx] lookup -> accumulate
  2. Callback lambda: searcher.radius(query, r, [&](idx, pt, dist){...})
  3. Callback std::ref: searcher.radius(query, r, std::ref(functor))

Cache states:
  - WARM: Sequential runs, data likely in cache
  - COLD: Cache flushed before each method (simulates real-world interleaved use)

)"
            << "\n";

  const int ROUNDS = 20;

  // Scenario 1: Small cloud, many queries (typical normal estimation)
  std::cout << "\n>>> SCENARIO 1: Dense queries (like normal estimation)\n";
  runBenchmark(50000, 10.0f, 10000, 0.3f, ROUNDS, false); // warm
  runBenchmark(50000, 10.0f, 10000, 0.3f, ROUNDS, true);  // cold

  // Scenario 2: Large cloud, many queries
  std::cout << "\n>>> SCENARIO 2: Large cloud, many queries\n";
  runBenchmark(200000, 20.0f, 20000, 0.5f, ROUNDS, false); // warm
  runBenchmark(200000, 20.0f, 20000, 0.5f, ROUNDS, true);  // cold

  // Scenario 3: Sparse results (small radius)
  std::cout << "\n>>> SCENARIO 3: Sparse results (small radius)\n";
  runBenchmark(100000, 15.0f, 10000, 0.1f, ROUNDS, false); // warm
  runBenchmark(100000, 15.0f, 10000, 0.1f, ROUNDS, true);  // cold

  // Scenario 4: Dense results (large radius)
  std::cout << "\n>>> SCENARIO 4: Dense results (large radius)\n";
  runBenchmark(100000, 15.0f, 5000, 1.0f, ROUNDS, false); // warm
  runBenchmark(100000, 15.0f, 5000, 1.0f, ROUNDS, true);  // cold

  // Summary
  std::cout << "\n"
            << std::string(80, '=') << "\n";
  std::cout << "INTERPRETATION\n";
  std::cout << std::string(80, '=') << "\n";
  std::cout << R"(
Expected results:
  - Callback should be faster due to:
    1. No vector allocation (malloc overhead)
    2. No index storage/retrieval
    3. Direct point access (pt already dereferenced in search)

  - Cold cache difference:
    - Vector: cloud[idx] causes cache miss for each point lookup
    - Callback: pt is already in register from search traversal

  - Lambda vs std::ref should be similar (both inline well)

If Vector is competitive:
  - For small result sets, allocation overhead is minimal
  - Modern allocators (jemalloc, tcmalloc) are very fast

Recommendation:
  - Use Callback for hot paths (normal estimation, ICP correspondence)
  - Use Vector when indices need to be stored/reused
)"
            << "\n";

  return 0;
}
