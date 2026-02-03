// nanoPCL - Test: Search Module

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include <nanopcl/search/kdtree.hpp>
#include <nanopcl/search/voxel_hash.hpp>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))
#define ASSERT_TRUE(cond) assert(cond)
#define ASSERT_FALSE(cond) assert(!(cond))

// =============================================================================
// Brute-force Ground Truth (for verification)
// =============================================================================

std::vector<uint32_t> bruteForceRadius(const PointCloud& cloud, const Point& center, float r) {
  std::vector<uint32_t> indices;
  float r_sq = r * r;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if ((cloud.point(i) - center).squaredNorm() <= r_sq) {
      indices.push_back(static_cast<uint32_t>(i));
    }
  }
  std::sort(indices.begin(), indices.end());
  return indices;
}

std::optional<search::NearestResult> bruteForceNearest(const PointCloud& cloud, const Point& center, float max_r) {
  float max_r_sq = max_r * max_r;
  float best_dist_sq = max_r_sq;
  uint32_t best_idx = std::numeric_limits<uint32_t>::max();

  for (size_t i = 0; i < cloud.size(); ++i) {
    float d2 = (cloud.point(i) - center).squaredNorm();
    if (d2 < best_dist_sq) {
      best_dist_sq = d2;
      best_idx = static_cast<uint32_t>(i);
    }
  }

  if (best_idx == std::numeric_limits<uint32_t>::max()) {
    return std::nullopt;
  }
  return search::NearestResult{best_idx, best_dist_sq};
}

std::vector<search::NearestResult> bruteForceKNN(const PointCloud& cloud, const Point& center, size_t k) {
  std::vector<search::NearestResult> all;
  all.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    float d2 = (cloud.point(i) - center).squaredNorm();
    all.push_back({static_cast<uint32_t>(i), d2});
  }

  std::sort(all.begin(), all.end(), [](const auto& a, const auto& b) {
    return a.dist_sq < b.dist_sq;
  });

  if (all.size() > k) all.resize(k);
  return all;
}

// =============================================================================
// Helper: Create test point clouds
// =============================================================================

PointCloud createGrid3x3x3() {
  PointCloud cloud;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 3; ++z) {
        cloud.add(float(x), float(y), float(z));
      }
    }
  }
  return cloud; // 27 points
}

PointCloud createRandomCloud(size_t n, float range = 10.0f, unsigned seed = 42) {
  PointCloud cloud;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-range, range);

  for (size_t i = 0; i < n; ++i) {
    cloud.add(dist(rng), dist(rng), dist(rng));
  }
  return cloud;
}

// =============================================================================
// VoxelHash Tests
// =============================================================================

void test_voxel_hash_empty_cloud() {
  PointCloud cloud;
  search::VoxelHash voxel_hash(0.5f);
  voxel_hash.build(cloud);

  ASSERT_TRUE(voxel_hash.empty());
  ASSERT_EQ(voxel_hash.size(), 0u);

  auto result = voxel_hash.radius(Point(0, 0, 0), 1.0f);
  ASSERT_TRUE(result.empty());

  auto nearest = voxel_hash.nearest(Point(0, 0, 0), 1.0f);
  ASSERT_FALSE(nearest.has_value());
}

void test_voxel_hash_radius_accuracy() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::VoxelHash voxel_hash(1.0f);
  voxel_hash.build(cloud);

  Point query(0, 0, 0);
  float radius = 3.0f;

  auto result = voxel_hash.radius(query, radius);
  auto expected = bruteForceRadius(cloud, query, radius);

  std::sort(result.begin(), result.end());

  ASSERT_EQ(result.size(), expected.size());
  for (size_t i = 0; i < result.size(); ++i) {
    ASSERT_EQ(result[i], expected[i]);
  }
}

void test_voxel_hash_radius_callback() {
  auto cloud = createGrid3x3x3();
  search::VoxelHash voxel_hash(0.5f);
  voxel_hash.build(cloud);

  Point query(1, 1, 1);
  float radius = 1.5f;

  std::vector<uint32_t> indices;
  std::vector<float> distances;

  voxel_hash.radius(query, radius, [&](uint32_t idx, const Point& pt, float dist_sq) {
    indices.push_back(idx);
    distances.push_back(dist_sq);

    // Verify distance matches
    float expected_dist_sq = (cloud.point(idx) - query).squaredNorm();
    ASSERT_NEAR(dist_sq, expected_dist_sq, 1e-5f);
  });

  auto expected = bruteForceRadius(cloud, query, radius);
  ASSERT_EQ(indices.size(), expected.size());
}

void test_voxel_hash_nearest() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::VoxelHash voxel_hash(1.0f);
  voxel_hash.build(cloud);

  Point query(0, 0, 0);
  float max_r = 5.0f;

  auto result = voxel_hash.nearest(query, max_r);
  auto expected = bruteForceNearest(cloud, query, max_r);

  if (expected.has_value()) {
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->index, expected->index);
    ASSERT_NEAR(result->dist_sq, expected->dist_sq, 1e-5f);
  } else {
    ASSERT_FALSE(result.has_value());
  }
}

void test_voxel_hash_nearest_out_of_range() {
  PointCloud cloud;
  cloud.add(100, 100, 100); // Far from origin

  search::VoxelHash voxel_hash(1.0f);
  voxel_hash.build(cloud);

  auto result = voxel_hash.nearest(Point(0, 0, 0), 1.0f);
  ASSERT_FALSE(result.has_value());
}

void test_voxel_hash_resolution_large() {
  auto cloud = createGrid3x3x3();
  search::VoxelHash voxel_hash(100.0f); // All points in one voxel
  voxel_hash.build(cloud);

  auto result = voxel_hash.radius(Point(1, 1, 1), 2.0f);
  auto expected = bruteForceRadius(cloud, Point(1, 1, 1), 2.0f);

  std::sort(result.begin(), result.end());
  ASSERT_EQ(result.size(), expected.size());
}

void test_voxel_hash_resolution_small() {
  auto cloud = createGrid3x3x3();
  search::VoxelHash voxel_hash(0.1f); // Each point in separate voxel
  voxel_hash.build(cloud);

  auto result = voxel_hash.radius(Point(1, 1, 1), 0.5f);
  auto expected = bruteForceRadius(cloud, Point(1, 1, 1), 0.5f);

  std::sort(result.begin(), result.end());
  ASSERT_EQ(result.size(), expected.size());
}

void test_voxel_hash_coordinate_limits() {
  // Test 21-bit coordinate limits (±2^20 voxels)
  // At resolution 1.0, max range is ~±1,048,576 meters
  PointCloud cloud;

  // Points at extreme coordinates (clamped by voxel::pack)
  cloud.add(0, 0, 0);
  cloud.add(1000, 1000, 1000);    // Normal range
  cloud.add(-1000, -1000, -1000); // Normal range (negative)
  cloud.add(100000, 0, 0);        // Large but within 21-bit range at res=1.0

  search::VoxelHash voxel_hash(1.0f);
  voxel_hash.build(cloud);

  // Should find the origin point
  auto result = voxel_hash.radius(Point(0, 0, 0), 1.5f);
  ASSERT_EQ(result.size(), 1u);
  ASSERT_EQ(result[0], 0u);

  // Should find the large coordinate point
  auto result2 = voxel_hash.nearest(Point(100000, 0, 0), 2.0f);
  ASSERT_TRUE(result2.has_value());
  ASSERT_EQ(result2->index, 3u);
}

// =============================================================================
// KdTree Tests
// =============================================================================

void test_kdtree_empty_cloud() {
  PointCloud cloud;
  search::KdTree kdtree;
  kdtree.build(cloud);

  ASSERT_TRUE(kdtree.empty());
  ASSERT_EQ(kdtree.size(), 0u);

  auto result = kdtree.radius(Point(0, 0, 0), 1.0f);
  ASSERT_TRUE(result.empty());

  auto nearest = kdtree.nearest(Point(0, 0, 0), 1.0f);
  ASSERT_FALSE(nearest.has_value());

  auto knn_result = kdtree.knn(Point(0, 0, 0), 5);
  ASSERT_TRUE(knn_result.empty());
}

void test_kdtree_radius_accuracy() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::KdTree kdtree;
  kdtree.build(cloud);

  Point query(0, 0, 0);
  float radius = 3.0f;

  auto result = kdtree.radius(query, radius);
  auto expected = bruteForceRadius(cloud, query, radius);

  std::sort(result.begin(), result.end());

  ASSERT_EQ(result.size(), expected.size());
  for (size_t i = 0; i < result.size(); ++i) {
    ASSERT_EQ(result[i], expected[i]);
  }
}

void test_kdtree_nearest() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::KdTree kdtree;
  kdtree.build(cloud);

  Point query(0, 0, 0);
  float max_r = 5.0f;

  auto result = kdtree.nearest(query, max_r);
  auto expected = bruteForceNearest(cloud, query, max_r);

  if (expected.has_value()) {
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->index, expected->index);
    ASSERT_NEAR(result->dist_sq, expected->dist_sq, 1e-5f);
  } else {
    ASSERT_FALSE(result.has_value());
  }
}

void test_kdtree_nearest_out_of_range() {
  PointCloud cloud;
  cloud.add(100, 100, 100); // Far from origin

  search::KdTree kdtree;
  kdtree.build(cloud);

  auto result = kdtree.nearest(Point(0, 0, 0), 1.0f);
  ASSERT_FALSE(result.has_value());
}

void test_kdtree_knn_k1() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::KdTree kdtree;
  kdtree.build(cloud);

  Point query(0, 0, 0);
  auto result = kdtree.knn(query, 1);
  auto expected = bruteForceKNN(cloud, query, 1);

  ASSERT_EQ(result.size(), 1u);
  ASSERT_EQ(result[0].index, expected[0].index);
  ASSERT_NEAR(result[0].dist_sq, expected[0].dist_sq, 1e-5f);
}

void test_kdtree_knn_k5() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::KdTree kdtree;
  kdtree.build(cloud);

  Point query(0, 0, 0);
  auto result = kdtree.knn(query, 5);
  auto expected = bruteForceKNN(cloud, query, 5);

  ASSERT_EQ(result.size(), 5u);
  for (size_t i = 0; i < 5; ++i) {
    ASSERT_EQ(result[i].index, expected[i].index);
    ASSERT_NEAR(result[i].dist_sq, expected[i].dist_sq, 1e-5f);
  }
}

void test_kdtree_knn_k_exceeds_size() {
  auto cloud = createGrid3x3x3(); // 27 points
  search::KdTree kdtree;
  kdtree.build(cloud);

  auto result = kdtree.knn(Point(1, 1, 1), 100); // Request more than available
  ASSERT_EQ(result.size(), 27u);
}

void test_kdtree_knn_sorted() {
  auto cloud = createRandomCloud(100, 10.0f);
  search::KdTree kdtree;
  kdtree.build(cloud);

  auto result = kdtree.knn(Point(0, 0, 0), 10);

  // Verify sorted by distance
  for (size_t i = 1; i < result.size(); ++i) {
    ASSERT_TRUE(result[i - 1].dist_sq <= result[i].dist_sq);
  }
}

// =============================================================================
// Cross-validation: VoxelHash vs KdTree
// =============================================================================

void test_cross_validation_radius() {
  auto cloud = createRandomCloud(200, 10.0f, 123);

  search::VoxelHash voxel_hash(0.5f);
  search::KdTree kdtree;

  voxel_hash.build(cloud);
  kdtree.build(cloud);

  Point query(2, 3, 1);
  float radius = 2.0f;

  auto vh_result = voxel_hash.radius(query, radius);
  auto kd_result = kdtree.radius(query, radius);

  std::sort(vh_result.begin(), vh_result.end());
  std::sort(kd_result.begin(), kd_result.end());

  ASSERT_EQ(vh_result.size(), kd_result.size());
  for (size_t i = 0; i < vh_result.size(); ++i) {
    ASSERT_EQ(vh_result[i], kd_result[i]);
  }
}

void test_cross_validation_nearest() {
  auto cloud = createRandomCloud(200, 10.0f, 456);

  search::VoxelHash voxel_hash(0.5f);
  search::KdTree kdtree;

  voxel_hash.build(cloud);
  kdtree.build(cloud);

  Point query(-1, 2, 3);
  float max_r = 3.0f;

  auto vh_result = voxel_hash.nearest(query, max_r);
  auto kd_result = kdtree.nearest(query, max_r);

  ASSERT_EQ(vh_result.has_value(), kd_result.has_value());
  if (vh_result.has_value()) {
    ASSERT_EQ(vh_result->index, kd_result->index);
    ASSERT_NEAR(vh_result->dist_sq, kd_result->dist_sq, 1e-5f);
  }
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "=== VoxelHash Tests ===\n";
  TEST(voxel_hash_empty_cloud);
  TEST(voxel_hash_radius_accuracy);
  TEST(voxel_hash_radius_callback);
  TEST(voxel_hash_nearest);
  TEST(voxel_hash_nearest_out_of_range);
  TEST(voxel_hash_resolution_large);
  TEST(voxel_hash_resolution_small);
  TEST(voxel_hash_coordinate_limits);

  std::cout << "\n=== KdTree Tests ===\n";
  TEST(kdtree_empty_cloud);
  TEST(kdtree_radius_accuracy);
  TEST(kdtree_nearest);
  TEST(kdtree_nearest_out_of_range);
  TEST(kdtree_knn_k1);
  TEST(kdtree_knn_k5);
  TEST(kdtree_knn_k_exceeds_size);
  TEST(kdtree_knn_sorted);

  std::cout << "\n=== Cross-validation Tests ===\n";
  TEST(cross_validation_radius);
  TEST(cross_validation_nearest);

  std::cout << "\n[PASS] All search tests passed.\n";
  return 0;
}
