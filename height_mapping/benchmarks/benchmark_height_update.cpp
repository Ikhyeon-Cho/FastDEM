// Height Map Update Benchmark
//
// Compares different update strategies:
//   1. Point-wise (current): getIndex() per point, sequential update
//   2. Cell-first grouping: group by cell, then batch update per cell
//   3. Batch mean: scatter-add sum/count, then divide (simple mean only)
//   4. Batch + variance: one-pass variance via sum of squares
//
// =============================================================================
// BENCHMARK RESULTS (Ryzen 9 5900X, -O2)
// =============================================================================
//
// Key Finding: Point-wise (current implementation) is optimal for most cases.
//
// 1. VARYING POINT COUNT (0.1m resolution, 1000x1000 cells)
// ----------------------------------------------------------------
// | Points | Point-wise | Batch+var |  Speedup  |
// |--------|------------|-----------|-----------|
// |   10K  |    0.45ms  |   3.66ms  |  8.1x     |
// |   50K  |    1.56ms  |   4.73ms  |  3.0x     |
// |  125K  |    3.68ms  |   6.69ms  |  1.8x     |
// |  250K  |    7.40ms  |   9.80ms  |  1.3x     |
// |  500K  |   14.46ms  |  15.66ms  |  1.1x     |
// ----------------------------------------------------------------
// → Point-wise is faster across all point counts
// → Gap narrows as points increase (batch overhead amortized)
//
// 2. VARYING RESOLUTION (125K points)
// ----------------------------------------------------------------
// | Resolution | Grid Size  | Point-wise | Batch+var | Winner     |
// |------------|------------|------------|-----------|------------|
// |   0.05m    | 2000x2000  |    5.7ms   |  23.1ms   | Point 4.0x |
// |   0.1m     | 1000x1000  |    3.5ms   |   4.1ms   | Point 1.2x |
// |   0.2m     |  500x500   |    3.4ms   |   3.1ms   | Batch 1.1x |
// |   0.5m     |  200x200   |    3.3ms   |   2.7ms   | Batch 1.2x |
// ----------------------------------------------------------------
// → High resolution (many cells): Point-wise wins
// → Low resolution (few cells): Batch wins
//
// 3. COMPLEXITY ANALYSIS
// ----------------------------------------------------------------
// Point-wise:  O(points)           - scales with input size
// Batch:       O(points + cells)   - fixed overhead per cell
//
// Crossover: Batch is faster when points/cells ratio > ~1
//
// 4. WHY CELL-FIRST GROUPING IS SLOW
// ----------------------------------------------------------------
// - Hash map insertion overhead > cache locality benefit
// - unordered_map memory allocation per cell
// - No SIMD benefit from grouping
//
// 5. WHY EIGEN VECTORIZED IS SLOW
// ----------------------------------------------------------------
// - isNaN() checks prevent vectorization
// - Conditional select() has high overhead
// - Scalar loop with branch prediction is more efficient
//
// RECOMMENDATION:
// - Keep Point-wise for general use (simpler, faster for typical cases)
// - Consider Batch+var only for: low resolution + high point density
// =============================================================================
//
// Build:
//   g++ -O2 -std=c++17 benchmark_height_update.cpp \
//       -o benchmark_height_update \
//       -I../lib/nanoPCL/include -I../include \
//       -I/usr/include/eigen3 \
//       $(pkg-config --cflags --libs grid_map_core)
//
// Run:
//   ./benchmark_height_update [path/to/kitti.bin]

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <nanopcl/io.hpp>
#include <unordered_map>

#include "../lib/nanoPCL/benchmarks/benchmark_common.hpp"

using namespace npcl;

// =============================================================================
// Configuration
// =============================================================================

struct MapConfig {
  float width = 100.0f;
  float height = 100.0f;
  float resolution = 0.1f;
};

// =============================================================================
// Index Hash for unordered_map
// =============================================================================

struct IndexHash {
  std::size_t operator()(const grid_map::Index& idx) const {
    return std::hash<int>()(idx(0)) ^ (std::hash<int>()(idx(1)) << 16);
  }
};

struct IndexEqual {
  bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
    return a(0) == b(0) && a(1) == b(1);
  }
};

// =============================================================================
// Method 1: Point-wise Update (Current Implementation)
// =============================================================================

void updatePointWise(grid_map::GridMap& map, const PointCloud& cloud) {
  auto& elevation = map.get("elevation");
  auto& variance = map.get("variance");
  auto& count = map.get("count");

  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }

    float& elev = elevation(idx(0), idx(1));
    float& var = variance(idx(0), idx(1));
    float& cnt = count(idx(0), idx(1));
    float z = point.z();

    // Welford's online algorithm (simplified)
    if (std::isnan(elev)) {
      elev = z;
      var = 0.0f;
      cnt = 1.0f;
    } else {
      cnt += 1.0f;
      float delta = z - elev;
      elev += delta / cnt;
      float delta2 = z - elev;
      if (cnt > 2.0f) {
        float m2 = var * (cnt - 2.0f);
        m2 += delta * delta2;
        var = m2 / (cnt - 1.0f);
      } else {
        var = delta * delta2;
      }
    }
  }
}

// =============================================================================
// Method 2: Cell-First Grouping
// =============================================================================

void updateCellFirstGrouping(grid_map::GridMap& map, const PointCloud& cloud) {
  auto& elevation = map.get("elevation");
  auto& variance = map.get("variance");
  auto& count = map.get("count");

  // Phase 1: Group points by cell
  std::unordered_map<grid_map::Index, std::vector<float>, IndexHash, IndexEqual>
      cell_points;
  cell_points.reserve(cloud.size() / 4);  // Estimate ~4 points per cell

  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }
    cell_points[idx].push_back(point.z());
  }

  // Phase 2: Update each cell with its grouped points
  for (auto& [idx, heights] : cell_points) {
    float& elev = elevation(idx(0), idx(1));
    float& var = variance(idx(0), idx(1));
    float& cnt = count(idx(0), idx(1));

    for (float z : heights) {
      if (std::isnan(elev)) {
        elev = z;
        var = 0.0f;
        cnt = 1.0f;
      } else {
        cnt += 1.0f;
        float delta = z - elev;
        elev += delta / cnt;
        float delta2 = z - elev;
        if (cnt > 2.0f) {
          float m2 = var * (cnt - 2.0f);
          m2 += delta * delta2;
          var = m2 / (cnt - 1.0f);
        } else {
          var = delta * delta2;
        }
      }
    }
  }
}

// =============================================================================
// Method 3: Batch Mean (Scatter-Add)
// - Only computes mean (no incremental variance)
// - Good for simple averaging use cases
// =============================================================================

void updateBatchMean(grid_map::GridMap& map, const PointCloud& cloud) {
  const auto size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);

  // Temporary accumulators (initialized to zero)
  Eigen::MatrixXf sum = Eigen::MatrixXf::Zero(rows, cols);
  Eigen::MatrixXf cnt = Eigen::MatrixXf::Zero(rows, cols);

  // Scatter-add phase
  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }
    sum(idx(0), idx(1)) += point.z();
    cnt(idx(0), idx(1)) += 1.0f;
  }

  // Batch divide (Eigen vectorized)
  auto& elevation = map.get("elevation");
  auto& count = map.get("count");

  // Only update cells that received new measurements
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (cnt(i, j) > 0) {
        float new_mean = sum(i, j) / cnt(i, j);
        float& elev = elevation(i, j);
        float& old_cnt = count(i, j);

        if (std::isnan(elev)) {
          elev = new_mean;
          old_cnt = cnt(i, j);
        } else {
          // Merge old and new means
          float total_cnt = old_cnt + cnt(i, j);
          elev = (elev * old_cnt + sum(i, j)) / total_cnt;
          old_cnt = total_cnt;
        }
      }
    }
  }
}

// =============================================================================
// Method 4: Batch Mean Pure (Single-scan, no accumulation)
// - Simplest case: just compute mean from current scan
// =============================================================================

void updateBatchMeanPure(grid_map::GridMap& map, const PointCloud& cloud) {
  const auto size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);

  Eigen::MatrixXf sum = Eigen::MatrixXf::Zero(rows, cols);
  Eigen::MatrixXf cnt = Eigen::MatrixXf::Zero(rows, cols);

  // Scatter-add
  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }
    sum(idx(0), idx(1)) += point.z();
    cnt(idx(0), idx(1)) += 1.0f;
  }

  // Batch divide (Eigen vectorized) - overwrites previous values
  auto& elevation = map.get("elevation");
  auto& count = map.get("count");

  // Use Eigen array operations
  elevation = (cnt.array() > 0).select(sum.array() / cnt.array(), elevation);
  count = (cnt.array() > 0).select(cnt, count);
}

// =============================================================================
// Method 5: Batch with Variance (One-pass, sum of squares)
// - Collects sum, sum_sq, count in single pass
// - Computes mean and variance using: var = (sum_sq - sum^2/n) / (n-1)
// =============================================================================

void updateBatchWithVariance(grid_map::GridMap& map, const PointCloud& cloud) {
  const auto size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);

  // Accumulators for this scan
  Eigen::MatrixXf sum = Eigen::MatrixXf::Zero(rows, cols);
  Eigen::MatrixXf sum_sq = Eigen::MatrixXf::Zero(rows, cols);
  Eigen::MatrixXf cnt = Eigen::MatrixXf::Zero(rows, cols);

  // Single pass: collect sum, sum_sq, count
  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }
    float z = point.z();
    sum(idx(0), idx(1)) += z;
    sum_sq(idx(0), idx(1)) += z * z;
    cnt(idx(0), idx(1)) += 1.0f;
  }

  auto& elevation = map.get("elevation");
  auto& variance = map.get("variance");
  auto& count = map.get("count");

  // Batch compute mean and variance, then merge with existing
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      float n2 = cnt(i, j);
      if (n2 < 1.0f) continue;

      // Scan statistics
      float mean2 = sum(i, j) / n2;
      float var2 = 0.0f;
      if (n2 > 1.0f) {
        // var = (sum_sq - sum^2/n) / (n-1)
        var2 = (sum_sq(i, j) - sum(i, j) * sum(i, j) / n2) / (n2 - 1.0f);
        var2 = std::max(var2, 0.0f);  // Numerical stability
      }

      float& elev = elevation(i, j);
      float& var = variance(i, j);
      float& n1 = count(i, j);

      if (std::isnan(elev)) {
        // First measurement for this cell
        elev = mean2;
        var = var2;
        n1 = n2;
      } else {
        // Merge two distributions (parallel Welford algorithm)
        float n = n1 + n2;
        float delta = mean2 - elev;

        // Combined mean
        float new_mean = elev + delta * (n2 / n);

        // Combined variance
        // M2_combined = M2_1 + M2_2 + delta^2 * n1 * n2 / n
        float m2_1 = (n1 > 1.0f) ? var * (n1 - 1.0f) : 0.0f;
        float m2_2 = (n2 > 1.0f) ? var2 * (n2 - 1.0f) : 0.0f;
        float m2_combined = m2_1 + m2_2 + delta * delta * n1 * n2 / n;
        float new_var = (n > 1.0f) ? m2_combined / (n - 1.0f) : 0.0f;

        elev = new_mean;
        var = new_var;
        n1 = n;
      }
    }
  }
}

// =============================================================================
// Method 6: Batch Variance with Eigen (vectorized merge)
// - Same as Method 5 but uses Eigen operations for merge step
// =============================================================================

void updateBatchVarianceEigen(grid_map::GridMap& map, const PointCloud& cloud) {
  const auto size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);

  // Accumulators
  Eigen::MatrixXf sum = Eigen::MatrixXf::Zero(rows, cols);
  Eigen::MatrixXf sum_sq = Eigen::MatrixXf::Zero(rows, cols);
  Eigen::MatrixXf cnt = Eigen::MatrixXf::Zero(rows, cols);

  // Scatter phase
  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }
    float z = point.z();
    sum(idx(0), idx(1)) += z;
    sum_sq(idx(0), idx(1)) += z * z;
    cnt(idx(0), idx(1)) += 1.0f;
  }

  // Compute scan statistics (vectorized)
  Eigen::ArrayXXf n2 = cnt.array();
  Eigen::ArrayXXf mean2 = (n2 > 0).select(sum.array() / n2, 0.0f);
  Eigen::ArrayXXf var2 = (n2 > 1).select(
      (sum_sq.array() - sum.array().square() / n2) / (n2 - 1.0f), 0.0f);
  var2 = var2.max(0.0f);  // Clamp negative values

  auto& elevation = map.get("elevation");
  auto& variance = map.get("variance");
  auto& count = map.get("count");

  Eigen::ArrayXXf n1 = count.array();
  Eigen::ArrayXXf elev = elevation.array();
  Eigen::ArrayXXf var = variance.array();

  // Mask for cells with new measurements
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> has_new = (n2 > 0);
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> is_first =
      has_new && elev.isNaN();
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> is_merge =
      has_new && !elev.isNaN();

  // For first measurements: just copy
  elev = is_first.select(mean2, elev);
  var = is_first.select(var2, var);
  n1 = is_first.select(n2, n1);

  // For merging: parallel Welford
  Eigen::ArrayXXf n_total = n1 + n2;
  Eigen::ArrayXXf delta = mean2 - elev;

  // Combined mean: elev + delta * n2 / n_total
  Eigen::ArrayXXf new_mean =
      is_merge.select(elev + delta * n2 / n_total.max(1.0f), elev);

  // Combined variance
  Eigen::ArrayXXf m2_1 = (n1 > 1).select(var * (n1 - 1.0f), 0.0f);
  Eigen::ArrayXXf m2_2 = (n2 > 1).select(var2 * (n2 - 1.0f), 0.0f);
  Eigen::ArrayXXf m2_combined =
      m2_1 + m2_2 + delta.square() * n1 * n2 / n_total.max(1.0f);
  Eigen::ArrayXXf new_var = is_merge.select(
      (n_total > 1).select(m2_combined / (n_total - 1.0f), 0.0f), var);

  // Write back
  elevation = is_merge.select(new_mean, elev).matrix();
  variance = is_merge.select(new_var, var).matrix();
  count = is_merge.select(n_total, n1).matrix();
}

// =============================================================================
// Map Setup Helper
// =============================================================================

grid_map::GridMap createMap(const MapConfig& config) {
  grid_map::GridMap map({"elevation", "variance", "count"});
  map.setGeometry(grid_map::Length(config.width, config.height),
                  config.resolution);
  map.get("elevation").setConstant(NAN);
  map.get("variance").setConstant(0.0f);
  map.get("count").setConstant(0.0f);
  return map;
}

void resetMap(grid_map::GridMap& map) {
  map.get("elevation").setConstant(NAN);
  map.get("variance").setConstant(0.0f);
  map.get("count").setConstant(0.0f);
}

// =============================================================================
// Benchmark Runner
// =============================================================================

void runBenchmark(const PointCloud& cloud, const MapConfig& config) {
  const size_t num_points = cloud.size();

  benchmark::printSection("Map Update Strategies");
  std::cout << "Points: " << num_points << "\n";
  std::cout << "Map: " << config.width << "x" << config.height << "m @ "
            << config.resolution << "m resolution\n";
  std::cout << "Grid: " << static_cast<int>(config.width / config.resolution)
            << "x" << static_cast<int>(config.height / config.resolution)
            << " cells\n\n";

  // Column header
  std::cout << std::left << std::setw(32) << "Method" << std::right
            << std::setw(14) << "Time (ms)" << std::setw(14) << "Throughput"
            << std::setw(10) << "Speedup"
            << "\n";
  std::cout << std::string(70, '-') << "\n";

  // 1. Point-wise (baseline)
  grid_map::GridMap map1 = createMap(config);
  auto stats_pointwise = benchmark::runVoid(
      [&]() {
        resetMap(map1);
        updatePointWise(map1, cloud);
      },
      benchmark::IterationPolicy::MEDIUM);

  double throughput_pw = num_points / stats_pointwise.mean / 1000.0;
  std::cout << std::left << std::setw(32) << "Point-wise (current)"
            << std::right << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_pointwise.mean << " ± " << std::setw(5)
            << stats_pointwise.ci_95() << std::setprecision(2) << std::setw(10)
            << throughput_pw << " Mp/s" << std::setw(10) << "1.00x"
            << "\n";

  // 2. Cell-first grouping
  grid_map::GridMap map2 = createMap(config);
  auto stats_grouping = benchmark::runVoid(
      [&]() {
        resetMap(map2);
        updateCellFirstGrouping(map2, cloud);
      },
      benchmark::IterationPolicy::MEDIUM);

  double throughput_grp = num_points / stats_grouping.mean / 1000.0;
  double speedup_grp = stats_pointwise.mean / stats_grouping.mean;
  std::cout << std::left << std::setw(32) << "Cell-first grouping" << std::right
            << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_grouping.mean << " ± " << std::setw(5)
            << stats_grouping.ci_95() << std::setprecision(2) << std::setw(10)
            << throughput_grp << " Mp/s" << std::setw(9) << speedup_grp << "x"
            << "\n";

  // 3. Batch mean (with accumulation)
  grid_map::GridMap map3 = createMap(config);
  auto stats_batch = benchmark::runVoid(
      [&]() {
        resetMap(map3);
        updateBatchMean(map3, cloud);
      },
      benchmark::IterationPolicy::MEDIUM);

  double throughput_bat = num_points / stats_batch.mean / 1000.0;
  double speedup_bat = stats_pointwise.mean / stats_batch.mean;
  std::cout << std::left << std::setw(32) << "Batch mean (accumulate)"
            << std::right << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_batch.mean << " ± " << std::setw(5) << stats_batch.ci_95()
            << std::setprecision(2) << std::setw(10) << throughput_bat
            << " Mp/s" << std::setw(9) << speedup_bat << "x"
            << "\n";

  // 4. Batch mean pure (no accumulation, simplest)
  grid_map::GridMap map4 = createMap(config);
  auto stats_pure = benchmark::runVoid(
      [&]() {
        resetMap(map4);
        updateBatchMeanPure(map4, cloud);
      },
      benchmark::IterationPolicy::MEDIUM);

  double throughput_pure = num_points / stats_pure.mean / 1000.0;
  double speedup_pure = stats_pointwise.mean / stats_pure.mean;
  std::cout << std::left << std::setw(32) << "Batch mean (no var)" << std::right
            << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_pure.mean << " ± " << std::setw(5) << stats_pure.ci_95()
            << std::setprecision(2) << std::setw(10) << throughput_pure
            << " Mp/s" << std::setw(9) << speedup_pure << "x"
            << "\n";

  // 5. Batch with variance (one-pass sum of squares)
  grid_map::GridMap map5 = createMap(config);
  auto stats_batch_var = benchmark::runVoid(
      [&]() {
        resetMap(map5);
        updateBatchWithVariance(map5, cloud);
      },
      benchmark::IterationPolicy::MEDIUM);

  double throughput_bv = num_points / stats_batch_var.mean / 1000.0;
  double speedup_bv = stats_pointwise.mean / stats_batch_var.mean;
  std::cout << std::left << std::setw(32) << "Batch + var (sum_sq)"
            << std::right << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_batch_var.mean << " ± " << std::setw(5)
            << stats_batch_var.ci_95() << std::setprecision(2) << std::setw(10)
            << throughput_bv << " Mp/s" << std::setw(9) << speedup_bv << "x"
            << "\n";

  // 6. Batch variance Eigen (vectorized merge)
  grid_map::GridMap map6 = createMap(config);
  auto stats_eigen_var = benchmark::runVoid(
      [&]() {
        resetMap(map6);
        updateBatchVarianceEigen(map6, cloud);
      },
      benchmark::IterationPolicy::MEDIUM);

  double throughput_ev = num_points / stats_eigen_var.mean / 1000.0;
  double speedup_ev = stats_pointwise.mean / stats_eigen_var.mean;
  std::cout << std::left << std::setw(32) << "Batch + var (Eigen)" << std::right
            << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_eigen_var.mean << " ± " << std::setw(5)
            << stats_eigen_var.ci_95() << std::setprecision(2) << std::setw(10)
            << throughput_ev << " Mp/s" << std::setw(9) << speedup_ev << "x"
            << "\n";

  // Reference: getIndex overhead
  grid_map::GridMap map_ref = createMap(config);
  size_t valid_count = 0;
  auto stats_getindex = benchmark::runVoid(
      [&]() {
        valid_count = 0;
        for (const auto& point : cloud) {
          grid_map::Index idx;
          if (map_ref.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
            ++valid_count;
          }
        }
      },
      benchmark::IterationPolicy::MEDIUM);

  std::cout << std::left << std::setw(32) << "[REF] getIndex() only"
            << std::right << std::fixed << std::setprecision(3) << std::setw(10)
            << stats_getindex.mean << " ± " << std::setw(5)
            << stats_getindex.ci_95() << std::setw(10) << "-" << std::setw(10)
            << "-"
            << "\n";

  std::cout << "\nValid points: " << valid_count << " / " << num_points << " ("
            << std::fixed << std::setprecision(1)
            << (100.0 * valid_count / num_points) << "%)\n";
}

// =============================================================================
// Point Cloud Manipulation
// =============================================================================

PointCloud subsample(const PointCloud& cloud, size_t target_size) {
  if (target_size >= cloud.size()) return cloud;

  PointCloud result;
  result.reserve(target_size);

  size_t step = cloud.size() / target_size;
  for (size_t i = 0; i < cloud.size() && result.size() < target_size;
       i += step) {
    result.add(cloud[i]);
  }
  return result;
}

PointCloud replicate(const PointCloud& cloud, size_t target_size) {
  if (target_size <= cloud.size()) return cloud;

  PointCloud result;
  result.reserve(target_size);

  // Copy original
  for (size_t i = 0; i < cloud.size(); ++i) {
    result.add(cloud[i]);
  }

  // Add with small offset to simulate denser scan
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> offset(-0.01f, 0.01f);

  while (result.size() < target_size) {
    size_t idx = result.size() % cloud.size();
    Point p = cloud[idx];
    result.add(
        Point(p.x() + offset(rng), p.y() + offset(rng), p.z() + offset(rng)));
  }
  return result;
}

PointCloud resize(const PointCloud& cloud, size_t target_size) {
  if (target_size < cloud.size()) {
    return subsample(cloud, target_size);
  } else {
    return replicate(cloud, target_size);
  }
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
  benchmark::printHeader("Height Map Update Benchmark");
  benchmark::PlatformInfo::capture().print();

  // Default data path
  std::string data_path = "../lib/nanoPCL/data/kitti/000000.bin";
  if (argc > 1) {
    data_path = argv[1];
  }

  std::cout << "\nLoading: " << data_path << "\n";

  // Load KITTI data
  PointCloud cloud_original = io::loadBIN(data_path);
  if (cloud_original.empty()) {
    std::cerr << "Failed to load point cloud from: " << data_path << "\n";
    return 1;
  }

  std::cout << "Loaded " << cloud_original.size() << " points\n";

  // Test with different point counts
  std::vector<size_t> point_counts = {
      10000,   // 10K
      50000,   // 50K
      124668,  // Original (~125K)
      250000,  // 250K
      500000,  // 500K
  };

  // Fixed resolution for point count comparison
  MapConfig config{100.0f, 100.0f, 0.1f};

  std::cout << "\n";
  std::cout << "==============================================================="
               "========\n";
  std::cout << " Varying Point Count (fixed resolution: " << config.resolution
            << "m)\n";
  std::cout << "==============================================================="
               "========\n";

  for (size_t target : point_counts) {
    PointCloud cloud = resize(cloud_original, target);
    runBenchmark(cloud, config);
  }

  // Also test resolution scaling with original cloud
  std::cout << "\n";
  std::cout << "==============================================================="
               "========\n";
  std::cout << " Varying Resolution (fixed points: " << cloud_original.size()
            << ")\n";
  std::cout << "==============================================================="
               "========\n";

  std::vector<MapConfig> configs = {
      {100.0f, 100.0f, 0.05f},  // Very fine
      {100.0f, 100.0f, 0.1f},   // Fine
      {100.0f, 100.0f, 0.2f},   // Medium
      {100.0f, 100.0f, 0.5f},   // Coarse
  };

  for (const auto& cfg : configs) {
    runBenchmark(cloud_original, cfg);
  }

  benchmark::printFooter(
      "Point-wise: Simple, works with any estimator.\n"
      "           Cell-first: Group by cell, then sequential update.\n"
      "           Batch mean: Fast but no variance.\n"
      "           Batch + var: One-pass variance via sum of squares.\n"
      "           Batch Eigen: Vectorized merge step.");

  return 0;
}
