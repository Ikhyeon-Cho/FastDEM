// nanoPCL Example 10: Spatial Search
//
// Two search structures: VoxelHash (fast build) vs KdTree (knn support).
// Choose based on your use case.

#include <chrono>
#include <iomanip>
#include <iostream>
#include <nanopcl/core.hpp>
#include <nanopcl/search.hpp>
#include <random>

using namespace npcl;

// Helper: Generate random point cloud
PointCloud generateCloud(size_t count, float range = 10.0f) {
  PointCloud cloud("map");
  cloud.reserve(count);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-range, range);

  for (size_t i = 0; i < count; ++i) {
    cloud.add(Point(dist(gen), dist(gen), dist(gen)));
  }
  return cloud;
}

// Helper: Timer
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

 public:
  Timer() : start_(std::chrono::high_resolution_clock::now()) {}
  double ms() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }
};

int main() {
  std::cout << std::fixed << std::setprecision(2);

  // ===========================================================================
  // 1. VoxelHash - Fast Build, Radius Search
  // ===========================================================================
  std::cout << "=== 1. VoxelHash ===\n";

  PointCloud cloud = generateCloud(100000);
  std::cout << "Cloud: " << cloud.size() << " points\n";

  // Build
  Timer t;
  search::VoxelHash voxel_hash(0.5f);  // resolution = 0.5m
  voxel_hash.build(cloud);
  std::cout << "VoxelHash build: " << t.ms() << " ms\n";

  // Radius search
  Point query(0.0f, 0.0f, 0.0f);
  float radius = 2.0f;

  t = Timer();
  auto indices = voxel_hash.radius(query, radius);
  std::cout << "radius(" << radius << "m): " << indices.size() << " points, "
            << t.ms() << " ms\n";

  // Nearest search
  t = Timer();
  auto nearest = voxel_hash.nearest(query, 10.0f);
  if (nearest) {
    std::cout << "nearest: idx=" << nearest->index
              << ", dist=" << std::sqrt(nearest->dist_sq) << "m\n";
  }
  std::cout << "\n";

  // ===========================================================================
  // 2. KdTree - KNN Support
  // ===========================================================================
  std::cout << "=== 2. KdTree ===\n";

  // Build
  t = Timer();
  search::KdTree kdtree;
  kdtree.build(cloud);
  std::cout << "KdTree build: " << t.ms() << " ms\n";

  // Radius search
  t = Timer();
  indices = kdtree.radius(query, radius);
  std::cout << "radius(" << radius << "m): " << indices.size() << " points, "
            << t.ms() << " ms\n";

  // KNN search (KdTree only!)
  t = Timer();
  size_t k = 10;
  auto knn_result = kdtree.knn(query, k);
  std::cout << "knn(k=" << k << "): ";
  for (size_t i = 0; i < std::min(knn_result.size(), size_t(3)); ++i) {
    std::cout << "idx=" << knn_result[i].index << " ";
  }
  if (knn_result.size() > 3) std::cout << "...";
  std::cout << "\n\n";

  // ===========================================================================
  // 3. Build Time Comparison
  // ===========================================================================
  std::cout << "=== 3. Build Time Comparison ===\n";

  size_t sizes[] = {10000, 50000, 100000, 200000};
  std::cout << std::left << std::setw(12) << "Points" << std::setw(15)
            << "VoxelHash" << std::setw(15) << "KdTree"
            << "Ratio\n";
  std::cout << std::string(50, '-') << "\n";

  for (size_t n : sizes) {
    PointCloud c = generateCloud(n);

    t = Timer();
    search::VoxelHash v(0.5f);
    v.build(c);
    double voxel_time = t.ms();

    t = Timer();
    search::KdTree kd;
    kd.build(c);
    double kdtree_time = t.ms();

    std::cout << std::left << std::setw(12) << n << std::setw(15) << voxel_time
              << std::setw(15) << kdtree_time << kdtree_time / voxel_time
              << "x\n";
  }
  std::cout << "\n";

  // ===========================================================================
  // 4. Query Time Comparison
  // ===========================================================================
  std::cout << "=== 4. Query Time Comparison ===\n";

  cloud = generateCloud(100000);
  search::VoxelHash v2(0.5f);
  v2.build(cloud);
  search::KdTree kd2;
  kd2.build(cloud);

  // Generate random queries
  std::vector<Point> queries;
  std::mt19937 gen(123);
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  for (int i = 0; i < 1000; ++i) {
    queries.emplace_back(dist(gen), dist(gen), dist(gen));
  }

  // VoxelHash radius
  t = Timer();
  size_t total = 0;
  for (const auto& q : queries) {
    total += v2.radius(q, 1.0f).size();
  }
  double voxel_query = t.ms();

  // KdTree radius
  t = Timer();
  total = 0;
  for (const auto& q : queries) {
    total += kd2.radius(q, 1.0f).size();
  }
  double kdtree_query = t.ms();

  std::cout << "1000 radius queries (r=1.0m):\n";
  std::cout << "  VoxelHash: " << voxel_query << " ms\n";
  std::cout << "  KdTree:    " << kdtree_query << " ms\n\n";

  // ===========================================================================
  // 5. When to Use What
  // ===========================================================================
  std::cout << "=== 5. When to Use What ===\n\n";

  std::cout << "VoxelHash:\n";
  std::cout << "  + Fast build O(N) - good for streaming/real-time\n";
  std::cout << "  + Fast radius search O(1) average\n";
  std::cout << "  + Zero-allocation callback API\n";
  std::cout << "  - No KNN support\n";
  std::cout << "  Use for: Normal estimation, real-time mapping\n\n";

  std::cout << "KdTree:\n";
  std::cout << "  + KNN search support\n";
  std::cout << "  + Better for sparse clouds\n";
  std::cout << "  - Slower build O(N log N)\n";
  std::cout << "  Use for: ICP, feature matching, static maps\n";

  return 0;
}
