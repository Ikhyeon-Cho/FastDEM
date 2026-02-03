// nanoPCL Benchmark: Normal Estimation - RVO vs Out Parameter
// Compares return value (RVO) vs out parameter for normal estimation

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include <nanopcl/common.hpp>
#include <nanopcl/search/voxel_hash.hpp>

using namespace nanopcl;

// =============================================================================
// Utilities
// =============================================================================

struct Stats {
  double mean;
  double stddev;
};

Stats computeStats(const std::vector<double>& data) {
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();
  double sq_sum = 0;
  for (double v : data)
    sq_sum += (v - mean) * (v - mean);
  return {mean, std::sqrt(sq_sum / data.size())};
}

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
// Normal Estimation Core (shared logic)
// =============================================================================

namespace bench_detail {
constexpr float TWO_PI_OVER_THREE = 2.0943951023931953f;

inline Eigen::Vector3f computeNormal(const Point& query_pt,
                                     const search::VoxelHash& searcher,
                                     float radius,
                                     size_t min_neighbors) {
  if (!query_pt.allFinite())
    return Eigen::Vector3f::Zero();

  struct Accumulator {
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
  } acc;

  searcher.radius(query_pt, radius, std::ref(acc));

  if (acc.count < min_neighbors)
    return Eigen::Vector3f::Zero();

  float n = static_cast<float>(acc.count);
  Point mean = acc.sum / n;

  Eigen::Matrix3f cov;
  cov(0, 0) = acc.cxx / n - mean.x() * mean.x();
  cov(0, 1) = acc.cxy / n - mean.x() * mean.y();
  cov(0, 2) = acc.cxz / n - mean.x() * mean.z();
  cov(1, 1) = acc.cyy / n - mean.y() * mean.y();
  cov(1, 2) = acc.cyz / n - mean.y() * mean.z();
  cov(2, 2) = acc.czz / n - mean.z() * mean.z();
  cov(1, 0) = cov(0, 1);
  cov(2, 0) = cov(0, 2);
  cov(2, 1) = cov(1, 2);

  float scale = cov.cwiseAbs().maxCoeff();
  if (scale <= std::numeric_limits<float>::min())
    return Eigen::Vector3f::Zero();

  Eigen::Matrix3f scaled_cov = cov / scale;
  float m = (scaled_cov(0, 0) + scaled_cov(1, 1) + scaled_cov(2, 2)) / 3.0f;
  float q = (scaled_cov - m * Eigen::Matrix3f::Identity()).squaredNorm() / 6.0f;
  float det_b = (scaled_cov - m * Eigen::Matrix3f::Identity()).determinant();

  Point normal;
  if (q <= std::numeric_limits<float>::min()) {
    normal = Point::UnitX();
  } else {
    float phi = std::atan2(
                    std::sqrt(std::max(0.0f, 4.0f * q * q * q - det_b * det_b)), det_b) /
                3.0f;
    float eigenvalue = m + 2.0f * std::sqrt(q) * std::cos(phi + TWO_PI_OVER_THREE);

    Eigen::Matrix3f r = scaled_cov - eigenvalue * Eigen::Matrix3f::Identity();
    Point r0 = r.row(0), r1 = r.row(1), r2 = r.row(2);
    Point n0 = r0.cross(r1), n1 = r0.cross(r2), n2 = r1.cross(r2);
    float d0 = n0.squaredNorm(), d1 = n1.squaredNorm(), d2 = n2.squaredNorm();

    float max_d = d0;
    normal = n0;
    if (d1 > max_d) {
      max_d = d1;
      normal = n1;
    }
    if (d2 > max_d) {
      max_d = d2;
      normal = n2;
    }

    if (max_d <= std::numeric_limits<float>::min()) {
      normal = Point::UnitX();
    } else {
      normal.normalize();
    }
  }

  if (normal.dot(-query_pt) < 0)
    normal = -normal;
  return normal;
}
} // namespace bench_detail

// =============================================================================
// Method 1: RVO (Return Value Optimization)
// =============================================================================

std::vector<Eigen::Vector3f> estimateNormals_RVO(const PointCloud& cloud,
                                                 float radius,
                                                 size_t min_neighbors = 5) {
  std::vector<Eigen::Vector3f> normals(cloud.size(), Eigen::Vector3f::Zero());

  search::VoxelHash searcher(radius);
  searcher.build(cloud);

  for (size_t i = 0; i < cloud.size(); ++i) {
    normals[i] = bench_detail::computeNormal(cloud[i], searcher, radius, min_neighbors);
  }

  return normals;
}

// =============================================================================
// Method 2: Out Parameter
// =============================================================================

void estimateNormals_OutParam(const PointCloud& cloud, float radius, std::vector<Eigen::Vector3f>& normals, size_t min_neighbors = 5) {
  normals.resize(cloud.size());
  std::fill(normals.begin(), normals.end(), Eigen::Vector3f::Zero());

  search::VoxelHash searcher(radius);
  searcher.build(cloud);

  for (size_t i = 0; i < cloud.size(); ++i) {
    normals[i] = bench_detail::computeNormal(cloud[i], searcher, radius, min_neighbors);
  }
}

// =============================================================================
// Data Generation
// =============================================================================

PointCloud generateCloud(size_t num_points, float range) {
  std::mt19937 gen(12345);
  std::uniform_real_distribution<float> xy_dist(-range, range);
  std::uniform_real_distribution<float> z_dist(-range * 0.1f, range * 0.1f);

  PointCloud cloud("bench");
  cloud.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(Point(xy_dist(gen), xy_dist(gen), z_dist(gen)));
  }

  return cloud;
}

// =============================================================================
// Benchmark
// =============================================================================

void runBenchmark(size_t num_points, float radius, int rounds) {
  PointCloud cloud = generateCloud(num_points, 10.0f);

  std::vector<double> rvo_times, out_times, out_reuse_times;

  // Pre-allocate for reuse test
  std::vector<Eigen::Vector3f> normals_buffer;

  // Warmup
  {
    auto n1 = estimateNormals_RVO(cloud, radius);
    doNotOptimize(n1);
    std::vector<Eigen::Vector3f> n2;
    estimateNormals_OutParam(cloud, radius, n2);
    doNotOptimize(n2);
  }

  for (int r = 0; r < rounds; ++r) {
    // RVO
    {
      auto start = std::chrono::high_resolution_clock::now();
      auto normals = estimateNormals_RVO(cloud, radius);
      auto end = std::chrono::high_resolution_clock::now();
      doNotOptimize(normals);
      rvo_times.push_back(
          std::chrono::duration<double, std::milli>(end - start).count());
    }

    // Out param (new allocation each time)
    {
      auto start = std::chrono::high_resolution_clock::now();
      std::vector<Eigen::Vector3f> normals;
      estimateNormals_OutParam(cloud, radius, normals);
      auto end = std::chrono::high_resolution_clock::now();
      doNotOptimize(normals);
      out_times.push_back(
          std::chrono::duration<double, std::milli>(end - start).count());
    }

    // Out param (reuse buffer)
    {
      auto start = std::chrono::high_resolution_clock::now();
      estimateNormals_OutParam(cloud, radius, normals_buffer);
      auto end = std::chrono::high_resolution_clock::now();
      doNotOptimize(normals_buffer);
      out_reuse_times.push_back(
          std::chrono::duration<double, std::milli>(end - start).count());
    }
  }

  Stats rvo_stats = computeStats(rvo_times);
  Stats out_stats = computeStats(out_times);
  Stats out_reuse_stats = computeStats(out_reuse_times);

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n"
            << num_points / 1000 << "K points, radius=" << radius << "\n";
  std::cout << std::string(55, '-') << "\n";
  std::cout << std::left << std::setw(30) << "Method" << std::right
            << std::setw(12) << "Mean(ms)" << std::setw(12) << "Stddev"
            << "\n";
  std::cout << std::string(55, '-') << "\n";
  std::cout << std::left << std::setw(30) << "RVO (return value)" << std::right
            << std::setw(12) << rvo_stats.mean << std::setw(12) << rvo_stats.stddev
            << "\n";
  std::cout << std::left << std::setw(30) << "Out param (new each time)" << std::right
            << std::setw(12) << out_stats.mean << std::setw(12) << out_stats.stddev
            << "\n";
  std::cout << std::left << std::setw(30) << "Out param (reuse buffer)" << std::right
            << std::setw(12) << out_reuse_stats.mean << std::setw(12)
            << out_reuse_stats.stddev << "\n";

  double rvo_vs_out = out_stats.mean / rvo_stats.mean;
  double rvo_vs_reuse = out_reuse_stats.mean / rvo_stats.mean;

  std::cout << "\nRVO vs Out(new):   " << rvo_vs_out << "x\n";
  std::cout << "RVO vs Out(reuse): " << rvo_vs_reuse << "x\n";
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "nanoPCL Benchmark: Normal Estimation - RVO vs Out Parameter\n";
  std::cout << std::string(55, '=') << "\n";
  std::cout << R"(
Comparing:
  1. RVO (return value)      - auto normals = estimateNormals(cloud, r);
  2. Out param (new)         - vector<> normals; estimateNormals(..., normals);
  3. Out param (reuse)       - reuse pre-allocated buffer across calls

Expected: All three should be nearly identical because:
  - C++17 guarantees RVO (no copy/move)
  - Computation time dominates allocation time
)"
            << "\n";

  const int ROUNDS = 10;

  runBenchmark(50000, 0.1f, ROUNDS);
  runBenchmark(100000, 0.1f, ROUNDS);
  runBenchmark(200000, 0.1f, ROUNDS);

  std::cout << "\n"
            << std::string(55, '=') << "\n";
  std::cout << "Conclusion:\n";
  std::cout << "  If ratios are ~1.0x, RVO is sufficient.\n";
  std::cout << "  If reuse is significantly faster, consider out param API.\n";

  return 0;
}
