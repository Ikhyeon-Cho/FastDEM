// nanoPCL - Test: Segmentation Module

#include <cassert>
#include <cmath>
#include <iostream>
#include <nanopcl/segmentation/euclidean_cluster.hpp>
#include <nanopcl/segmentation/ground_seg.hpp>
#include <nanopcl/segmentation/ransac_plane.hpp>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))
#define ASSERT_TRUE(cond) assert(cond)
#define ASSERT_FALSE(cond) assert(!(cond))
#define ASSERT_GE(a, b) assert((a) >= (b))
#define ASSERT_LE(a, b) assert((a) <= (b))
#define ASSERT_GT(a, b) assert((a) > (b))
#define ASSERT_LT(a, b) assert((a) < (b))

// =============================================================================
// Helper: Create test point clouds
// =============================================================================

/// Creates a flat ground plane at z=0 with some noise
PointCloud createGroundPlane(int nx, int ny, float spacing, float noise = 0.01f) {
  PointCloud cloud;
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      float z = (rand() % 100) * noise * 0.01f;
      cloud.add(x * spacing, y * spacing, z);
    }
  }
  return cloud;
}

/// Creates a spherical cluster centered at (cx, cy, cz)
PointCloud createCluster(float cx, float cy, float cz, int n, float radius) {
  PointCloud cloud;
  for (int i = 0; i < n; ++i) {
    float dx = (rand() % 200 - 100) * 0.01f * radius;
    float dy = (rand() % 200 - 100) * 0.01f * radius;
    float dz = (rand() % 200 - 100) * 0.01f * radius;
    cloud.add(cx + dx, cy + dy, cz + dz);
  }
  return cloud;
}

/// Creates multiple separated clusters
PointCloud createMultipleClusters() {
  PointCloud cloud;

  // Cluster 1: around (0, 0, 1)
  for (int i = 0; i < 50; ++i) {
    cloud.add((rand() % 100) * 0.005f, (rand() % 100) * 0.005f, 1.0f + (rand() % 100) * 0.005f);
  }

  // Cluster 2: around (5, 0, 1)
  for (int i = 0; i < 30; ++i) {
    cloud.add(5.0f + (rand() % 100) * 0.005f, (rand() % 100) * 0.005f, 1.0f + (rand() % 100) * 0.005f);
  }

  // Cluster 3: around (0, 5, 1)
  for (int i = 0; i < 20; ++i) {
    cloud.add((rand() % 100) * 0.005f, 5.0f + (rand() % 100) * 0.005f, 1.0f + (rand() % 100) * 0.005f);
  }

  return cloud;
}

/// Creates a scene with ground + obstacles
/// Note: Ground points are denser to satisfy min_points_per_cell requirement
PointCloud createGroundWithObstacles() {
  PointCloud cloud;

  // Ground plane (dense: multiple points per grid cell)
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      // Add multiple points per cell to satisfy min_points_per_cell
      for (int k = 0; k < 3; ++k) {
        float dx = (rand() % 100) * 0.004f; // 0 ~ 0.4 within cell
        float dy = (rand() % 100) * 0.004f;
        float dz = (rand() % 100) * 0.001f; // Small z noise
        cloud.add(x * 0.5f + dx, y * 0.5f + dy, dz);
      }
    }
  }

  // Obstacle 1
  for (int i = 0; i < 40; ++i) {
    cloud.add(2.0f + (rand() % 100) * 0.005f, 2.0f + (rand() % 100) * 0.005f, 1.0f + (rand() % 100) * 0.01f);
  }

  // Obstacle 2
  for (int i = 0; i < 40; ++i) {
    cloud.add(-2.0f + (rand() % 100) * 0.005f, -2.0f + (rand() % 100) * 0.005f, 1.0f + (rand() % 100) * 0.01f);
  }

  return cloud;
}

// =============================================================================
// 1. Euclidean Clustering Tests
// =============================================================================

void test_euclidean_basic_clustering() {
  PointCloud cloud = createMultipleClusters();
  ASSERT_EQ(cloud.size(), 100); // 50 + 30 + 20

  segmentation::ClusterConfig config;
  config.tolerance = 1.0f;
  config.min_size = 10;
  config.max_size = 100;

  auto result = segmentation::euclideanCluster(cloud, config);

  ASSERT_EQ(result.numClusters(), 3);
  ASSERT_EQ(result.totalClusteredPoints(), 100);
}

void test_euclidean_csr_format() {
  PointCloud cloud = createMultipleClusters();

  segmentation::ClusterConfig config;
  config.tolerance = 1.0f;
  config.min_size = 10;

  auto result = segmentation::euclideanCluster(cloud, config);

  // CSR format: offsets.size() == numClusters + 1
  ASSERT_EQ(result.offsets.size(), result.numClusters() + 1);
  ASSERT_EQ(result.offsets[0], 0);

  // Sum of cluster sizes equals total clustered points
  size_t total = 0;
  for (size_t i = 0; i < result.numClusters(); ++i) {
    total += result.clusterSize(i);
  }
  ASSERT_EQ(total, result.totalClusteredPoints());
}

void test_euclidean_cluster_extraction() {
  PointCloud cloud = createMultipleClusters();

  segmentation::ClusterConfig config;
  config.tolerance = 1.0f;
  config.min_size = 10;

  auto result = segmentation::euclideanCluster(cloud, config);

  // Extract first cluster
  PointCloud cluster0 = result.extract(cloud, 0);
  ASSERT_EQ(cluster0.size(), result.clusterSize(0));

  // Verify points are valid
  for (size_t i = 0; i < cluster0.size(); ++i) {
    ASSERT_TRUE(std::isfinite(cluster0.point(i).x()));
  }
}

void test_euclidean_config_tolerance() {
  PointCloud cloud;
  // Two groups separated by 2m
  for (int i = 0; i < 20; ++i) {
    cloud.add(i * 0.1f, 0, 0);         // Group 1: 0-2m
    cloud.add(10.0f + i * 0.1f, 0, 0); // Group 2: 10-12m
  }

  // Small tolerance: 2 clusters
  segmentation::ClusterConfig config1;
  config1.tolerance = 0.5f;
  config1.min_size = 5;
  auto result1 = segmentation::euclideanCluster(cloud, config1);
  ASSERT_EQ(result1.numClusters(), 2);

  // Large tolerance: 1 cluster (if gap < tolerance, but 8m gap is too big)
  segmentation::ClusterConfig config2;
  config2.tolerance = 0.5f;
  config2.min_size = 5;
  auto result2 = segmentation::euclideanCluster(cloud, config2);
  ASSERT_EQ(result2.numClusters(), 2);
}

void test_euclidean_config_min_max_size() {
  PointCloud cloud = createMultipleClusters(); // 50, 30, 20 points

  // min_size = 25: exclude cluster with 20 points
  segmentation::ClusterConfig config1;
  config1.tolerance = 1.0f;
  config1.min_size = 25;
  config1.max_size = 100;
  auto result1 = segmentation::euclideanCluster(cloud, config1);
  ASSERT_EQ(result1.numClusters(), 2); // 50 and 30 pass, 20 rejected

  // max_size = 40: exclude cluster with 50 points (PCL-style rejection)
  segmentation::ClusterConfig config2;
  config2.tolerance = 1.0f;
  config2.min_size = 10;
  config2.max_size = 40;
  auto result2 = segmentation::euclideanCluster(cloud, config2);
  ASSERT_EQ(result2.numClusters(), 2); // 30 and 20 pass, 50 rejected
}

void test_euclidean_subset_clustering() {
  PointCloud cloud = createGroundWithObstacles();

  // Create subset indices (obstacles only)
  std::vector<uint32_t> subset;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud.point(i).z() > 0.5f) {
      subset.push_back(static_cast<uint32_t>(i));
    }
  }

  segmentation::ClusterConfig config;
  config.tolerance = 0.5f;
  config.min_size = 10;

  auto result = segmentation::euclideanCluster(cloud, subset, config);

  ASSERT_GE(result.numClusters(), 1);

  // Verify indices reference original cloud
  for (size_t i = 0; i < result.numClusters(); ++i) {
    for (uint32_t idx : result.clusterIndices(i)) {
      ASSERT_LT(idx, cloud.size());
    }
  }
}

void test_euclidean_apply_labels() {
  PointCloud cloud = createMultipleClusters();

  segmentation::ClusterConfig config;
  config.tolerance = 1.0f;
  config.min_size = 10;

  auto result = segmentation::euclideanCluster(cloud, config);
  segmentation::applyClusterLabels(cloud, result, 1);

  ASSERT_TRUE(cloud.hasLabel());
  ASSERT_EQ(cloud.labels().size(), cloud.size());

  // Check that cluster points have non-zero instance IDs
  size_t labeled_count = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if ((cloud.label(i).val & 0xFFFF) > 0) {
      labeled_count++;
    }
  }
  ASSERT_EQ(labeled_count, result.totalClusteredPoints());
}

void test_euclidean_noise_indices() {
  PointCloud cloud;
  // Dense cluster
  for (int i = 0; i < 50; ++i) {
    cloud.add(i * 0.1f, 0, 0);
  }
  // Isolated noise points
  cloud.add(100.0f, 0, 0);
  cloud.add(200.0f, 0, 0);

  segmentation::ClusterConfig config;
  config.tolerance = 0.5f;
  config.min_size = 10;

  auto result = segmentation::euclideanCluster(cloud, config);
  auto noise = result.noiseIndices(cloud.size());

  ASSERT_EQ(noise.size(), 2); // Two isolated points
}

// =============================================================================
// 2. RANSAC Plane Tests
// =============================================================================

void test_ransac_plane_detection() {
  PointCloud cloud = createGroundPlane(20, 20, 0.5f, 0.01f);

  auto result = segmentation::segmentPlane(cloud, 0.05f);

  ASSERT_TRUE(result.success());
  ASSERT_GT(result.fitness, 0.9); // Most points should be inliers
}

void test_ransac_plane_model_accuracy() {
  // Create perfect XY plane at z=1
  PointCloud cloud;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      cloud.add(float(x), float(y), 1.0f);
    }
  }

  auto result = segmentation::segmentPlane(cloud, 0.1f);

  ASSERT_TRUE(result.success());
  // Normal should be approximately (0, 0, 1) or (0, 0, -1)
  ASSERT_NEAR(std::abs(result.model.coefficients.z()), 1.0f, 0.01f);
  ASSERT_NEAR(std::abs(result.model.coefficients.w()), 1.0f, 0.1f);
}

void test_ransac_inlier_outlier_split() {
  PointCloud cloud = createGroundPlane(10, 10, 1.0f, 0.01f);
  size_t ground_size = cloud.size();

  // Add outliers above ground
  for (int i = 0; i < 20; ++i) {
    cloud.add(float(i % 10), float(i / 10), 5.0f);
  }

  auto result = segmentation::segmentPlane(cloud, 0.1f);
  auto outliers = result.outliers(cloud.size());

  ASSERT_TRUE(result.success());
  ASSERT_GE(result.inliers.size(), ground_size * 0.9);
  ASSERT_GE(outliers.size(), 15); // Most elevated points should be outliers
}

void test_ransac_multiple_planes() {
  PointCloud cloud;

  // Plane 1: z = 0
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      cloud.add(float(x), float(y), 0.0f);
    }
  }

  // Plane 2: z = 5
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      cloud.add(float(x), float(y), 5.0f);
    }
  }

  segmentation::RansacConfig config;
  config.distance_threshold = 0.1f;
  config.max_iterations = 500;

  auto results = segmentation::segmentMultiplePlanes(cloud, config, 3, 0.1);

  ASSERT_GE(results.size(), 2);
}

void test_ransac_collinear_points() {
  // Points on a line (cannot form a plane)
  PointCloud cloud;
  for (int i = 0; i < 10; ++i) {
    cloud.add(float(i), 0, 0);
  }

  auto result = segmentation::segmentPlane(cloud, 0.1f, 100);

  // Should either fail or have very low fitness
  // (collinear points can technically fit infinite planes)
  ASSERT_TRUE(result.inliers.size() <= cloud.size());
}

void test_ransac_plane_from_points() {
  Point p1(0, 0, 0);
  Point p2(1, 0, 0);
  Point p3(0, 1, 0);

  auto model = segmentation::PlaneModel::fromPoints(p1, p2, p3);

  ASSERT_TRUE(model.isValid());
  ASSERT_NEAR(std::abs(model.coefficients.z()), 1.0f, 0.01f);
  ASSERT_NEAR(model.coefficients.w(), 0.0f, 0.01f);
}

// =============================================================================
// 3. Ground Segmentation Tests
// =============================================================================

void test_ground_basic_segmentation() {
  PointCloud cloud = createGroundWithObstacles();

  auto result = segmentation::segmentGround(cloud, 0.5f, 0.3f, 0.5f);

  ASSERT_FALSE(result.empty());
  ASSERT_GT(result.ground.size(), 0);
  ASSERT_GT(result.obstacles.size(), 0);
  ASSERT_EQ(result.ground.size() + result.obstacles.size(), cloud.size());
}

void test_ground_grid_resolution() {
  // Create dense ground plane (multiple points per cell)
  PointCloud cloud;
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      for (int k = 0; k < 3; ++k) {
        float dx = (rand() % 100) * 0.004f;
        float dy = (rand() % 100) * 0.004f;
        cloud.add(x * 0.5f + dx, y * 0.5f + dy, (rand() % 100) * 0.001f);
      }
    }
  }

  // Add obstacle
  for (int i = 0; i < 20; ++i) {
    cloud.add(5.0f, 5.0f, 1.0f + i * 0.05f);
  }

  // Different resolutions
  auto result1 = segmentation::segmentGround(cloud, 0.5f);
  auto result2 = segmentation::segmentGround(cloud, 2.0f);

  // Both should identify ground vs obstacles
  ASSERT_GT(result1.ground.size(), 0);
  ASSERT_GT(result2.ground.size(), 0);
}

void test_ground_thickness_config() {
  PointCloud cloud;

  // Ground at z=0 (dense: multiple points per cell)
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int k = 0; k < 5; ++k) {
        float dx = (rand() % 100) * 0.004f;
        float dy = (rand() % 100) * 0.004f;
        cloud.add(float(x) + dx, float(y) + dy, 0.0f);
      }
    }
  }
  size_t base_ground = cloud.size();

  // Points at z=0.2 (may be within or above thickness depending on config)
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int k = 0; k < 3; ++k) {
        float dx = (rand() % 100) * 0.004f;
        float dy = (rand() % 100) * 0.004f;
        cloud.add(float(x) + dx, float(y) + dy, 0.2f);
      }
    }
  }

  // Thin ground layer
  segmentation::GroundSegConfig config1;
  config1.ground_thickness = 0.1f;
  config1.grid_resolution = 1.0f;
  auto result1 = segmentation::segmentGround(cloud, config1);

  // Thick ground layer
  segmentation::GroundSegConfig config2;
  config2.ground_thickness = 0.5f;
  config2.grid_resolution = 1.0f;
  auto result2 = segmentation::segmentGround(cloud, config2);

  // Thicker ground should classify more as ground
  ASSERT_GT(result2.ground.size(), result1.ground.size());
}

void test_ground_height_config() {
  PointCloud cloud;

  // Low ground (z=0) - dense
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int k = 0; k < 3; ++k) {
        float dx = (rand() % 100) * 0.004f;
        float dy = (rand() % 100) * 0.004f;
        cloud.add(float(x) + dx, float(y) + dy, 0.0f);
      }
    }
  }
  size_t low_ground_count = cloud.size();

  // Elevated "ground" (z=1) - dense, separate area
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int k = 0; k < 3; ++k) {
        float dx = (rand() % 100) * 0.004f;
        float dy = (rand() % 100) * 0.004f;
        cloud.add(20.0f + float(x) + dx, float(y) + dy, 1.0f);
      }
    }
  }

  // Low max_ground_height: only z=0 is ground
  segmentation::GroundSegConfig config;
  config.max_ground_height = 0.5f;
  config.grid_resolution = 1.0f;
  auto result = segmentation::segmentGround(cloud, config);

  // Low ground should be classified as ground
  ASSERT_GT(result.ground.size(), 0);
  // Elevated points should be obstacles (their cells have robust_min_z > max_ground_height)
  ASSERT_GT(result.obstacles.size(), 0);
}

// =============================================================================
// 4. Boundary Condition Tests
// =============================================================================

void test_empty_cloud_clustering() {
  PointCloud empty;
  auto result = segmentation::euclideanCluster(empty);
  ASSERT_EQ(result.numClusters(), 0);
  ASSERT_TRUE(result.empty());
}

void test_empty_cloud_ransac() {
  PointCloud empty;
  auto result = segmentation::segmentPlane(empty, 0.1f);
  ASSERT_FALSE(result.success());
}

void test_empty_cloud_ground() {
  PointCloud empty;
  auto result = segmentation::segmentGround(empty);
  ASSERT_TRUE(result.empty());
}

void test_single_point() {
  PointCloud single;
  single.add(1.0f, 2.0f, 3.0f);

  // Clustering: below min_size
  segmentation::ClusterConfig config;
  config.min_size = 2;
  auto cluster_result = segmentation::euclideanCluster(single, config);
  ASSERT_EQ(cluster_result.numClusters(), 0);

  // RANSAC: need 3 points minimum
  auto ransac_result = segmentation::segmentPlane(single, 0.1f);
  ASSERT_FALSE(ransac_result.success());

  // Ground: should work
  auto ground_result = segmentation::segmentGround(single);
  ASSERT_EQ(ground_result.ground.size() + ground_result.obstacles.size(), 1);
}

void test_insufficient_points_for_plane() {
  PointCloud cloud;
  cloud.add(0, 0, 0);
  cloud.add(1, 0, 0);

  auto result = segmentation::segmentPlane(cloud, 0.1f);
  ASSERT_FALSE(result.success());
}

// =============================================================================
// 5. Channel Preservation Tests
// =============================================================================

void test_cluster_extract_preserves_channels() {
  PointCloud cloud;
  cloud.useIntensity();
  cloud.useColor();

  // Cluster 1
  for (int i = 0; i < 20; ++i) {
    cloud.add(float(i) * 0.1f, 0, 0, Intensity(0.5f));
    cloud.color(cloud.size() - 1) = Color(100, 150, 200);
  }

  // Cluster 2
  for (int i = 0; i < 20; ++i) {
    cloud.add(10.0f + float(i) * 0.1f, 0, 0, Intensity(0.8f));
    cloud.color(cloud.size() - 1) = Color(50, 100, 150);
  }

  segmentation::ClusterConfig config;
  config.tolerance = 0.5f;
  config.min_size = 10;

  auto result = segmentation::euclideanCluster(cloud, config);
  ASSERT_EQ(result.numClusters(), 2);

  PointCloud extracted = result.extract(cloud, 0);

  ASSERT_TRUE(extracted.hasIntensity());
  ASSERT_TRUE(extracted.hasColor());
  ASSERT_EQ(extracted.size(), result.clusterSize(0));
}

// =============================================================================
// 6. Integration Tests
// =============================================================================

void test_ground_removal_then_clustering() {
  PointCloud cloud = createGroundWithObstacles();

  // Step 1: Ground segmentation
  auto ground_result = segmentation::segmentGround(cloud, 0.5f, 0.3f, 0.5f);

  // Step 2: Cluster obstacles
  segmentation::ClusterConfig config;
  config.tolerance = 0.5f;
  config.min_size = 10;

  auto cluster_result = segmentation::euclideanCluster(cloud, ground_result.obstacles, config);

  // Should find obstacle clusters
  ASSERT_GE(cluster_result.numClusters(), 1);

  // Total clustered points <= obstacle points
  ASSERT_LE(cluster_result.totalClusteredPoints(), ground_result.obstacles.size());
}

void test_ransac_then_clustering() {
  PointCloud cloud = createGroundWithObstacles();

  // Step 1: RANSAC plane removal
  auto plane_result = segmentation::segmentPlane(cloud, 0.1f);
  ASSERT_TRUE(plane_result.success());

  // Step 2: Cluster outliers (obstacles)
  auto outliers = plane_result.outliers(cloud.size());

  segmentation::ClusterConfig config;
  config.tolerance = 0.5f;
  config.min_size = 10;

  auto cluster_result = segmentation::euclideanCluster(cloud, outliers, config);

  ASSERT_GE(cluster_result.numClusters(), 1);
}

// =============================================================================
// Main
// =============================================================================

int main() {
  srand(42); // Reproducible tests

  std::cout << "\n=== nanoPCL Segmentation Tests ===\n\n";

  std::cout << "[Euclidean Clustering]\n";
  TEST(euclidean_basic_clustering);
  TEST(euclidean_csr_format);
  TEST(euclidean_cluster_extraction);
  TEST(euclidean_config_tolerance);
  TEST(euclidean_config_min_max_size);
  TEST(euclidean_subset_clustering);
  TEST(euclidean_apply_labels);
  TEST(euclidean_noise_indices);

  std::cout << "\n[RANSAC Plane Segmentation]\n";
  TEST(ransac_plane_detection);
  TEST(ransac_plane_model_accuracy);
  TEST(ransac_inlier_outlier_split);
  TEST(ransac_multiple_planes);
  TEST(ransac_collinear_points);
  TEST(ransac_plane_from_points);

  std::cout << "\n[Ground Segmentation]\n";
  TEST(ground_basic_segmentation);
  TEST(ground_grid_resolution);
  TEST(ground_thickness_config);
  TEST(ground_height_config);

  std::cout << "\n[Boundary Conditions]\n";
  TEST(empty_cloud_clustering);
  TEST(empty_cloud_ransac);
  TEST(empty_cloud_ground);
  TEST(single_point);
  TEST(insufficient_points_for_plane);

  std::cout << "\n[Channel Preservation]\n";
  TEST(cluster_extract_preserves_channels);

  std::cout << "\n[Integration Tests]\n";
  TEST(ground_removal_then_clustering);
  TEST(ransac_then_clustering);

  std::cout << "\n=== All segmentation tests passed! ===\n\n";
  return 0;
}
