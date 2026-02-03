// nanoPCL - Test: Geometry Module (Normal/Covariance Estimation)

#include <cassert>
#include <cmath>
#include <iostream>
#include <nanopcl/geometry/normal_estimation.hpp>

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
// Helper: Create test point clouds
// =============================================================================

PointCloud createPlaneXY(int n = 10, float z = 0.0f) {
  // Points on XY plane at z=z
  PointCloud cloud;
  for (int x = 0; x < n; ++x) {
    for (int y = 0; y < n; ++y) {
      cloud.add(float(x), float(y), z);
    }
  }
  return cloud;
}

PointCloud createPlaneXZ(int n = 10, float y = 0.0f) {
  // Points on XZ plane at y=y
  PointCloud cloud;
  for (int x = 0; x < n; ++x) {
    for (int z = 0; z < n; ++z) {
      cloud.add(float(x), y, float(z));
    }
  }
  return cloud;
}

PointCloud createTiltedPlane(int n = 10) {
  // Plane: z = 0.5*x + 0.3*y
  // Normal should be proportional to (-0.5, -0.3, 1) normalized
  PointCloud cloud;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      float x = float(i);
      float y = float(j);
      float z = 0.5f * x + 0.3f * y;
      cloud.add(x, y, z);
    }
  }
  return cloud;
}

PointCloud createSparseCloud() {
  // Only 2 points - insufficient for PCA
  PointCloud cloud;
  cloud.add(0, 0, 0);
  cloud.add(1, 0, 0);
  return cloud;
}

PointCloud createCollinearPoints() {
  // Points on a line - degenerate plane
  PointCloud cloud;
  for (int i = 0; i < 10; ++i) {
    cloud.add(float(i), 0, 0);
  }
  return cloud;
}

// =============================================================================
// Normal Estimation Tests
// =============================================================================

void test_normals_plane_xy() {
  auto cloud = createPlaneXY(5, 0.0f);
  geometry::estimateNormals(cloud, 10);

  ASSERT_TRUE(cloud.hasNormal());

  // Check center point normal (avoid boundary effects)
  size_t center_idx = 12; // (2,2) in 5x5 grid
  auto n = cloud.normal(center_idx);

  // Normal should be close to (0, 0, ±1)
  ASSERT_NEAR(std::abs(n.x()), 0.0f, 0.1f);
  ASSERT_NEAR(std::abs(n.y()), 0.0f, 0.1f);
  ASSERT_NEAR(std::abs(n.z()), 1.0f, 0.1f);
}

void test_normals_plane_xz() {
  auto cloud = createPlaneXZ(5, 0.0f);
  geometry::estimateNormals(cloud, 10);

  ASSERT_TRUE(cloud.hasNormal());

  size_t center_idx = 12;
  auto n = cloud.normal(center_idx);

  // Normal should be close to (0, ±1, 0)
  ASSERT_NEAR(std::abs(n.x()), 0.0f, 0.1f);
  ASSERT_NEAR(std::abs(n.y()), 1.0f, 0.1f);
  ASSERT_NEAR(std::abs(n.z()), 0.0f, 0.1f);
}

void test_normals_plane_arbitrary() {
  auto cloud = createTiltedPlane(5);
  geometry::estimateNormals(cloud, 10);

  ASSERT_TRUE(cloud.hasNormal());

  // Expected normal direction: (-0.5, -0.3, 1) normalized
  Eigen::Vector3f expected(-0.5f, -0.3f, 1.0f);
  expected.normalize();

  size_t center_idx = 12;
  Eigen::Vector3f n = cloud.normal(center_idx);

  // Check alignment (dot product close to ±1)
  float dot = std::abs(n.dot(expected));
  ASSERT_NEAR(dot, 1.0f, 0.1f);
}

void test_normals_viewpoint_orientation() {
  auto cloud = createPlaneXY(5, 0.0f);

  // Viewpoint above the plane
  Point viewpoint(2.0f, 2.0f, 10.0f);
  geometry::estimateNormals(cloud, 10, viewpoint);

  size_t center_idx = 12;
  Eigen::Vector3f n = cloud.normal(center_idx);
  Eigen::Vector3f to_viewpoint = viewpoint - cloud.point(center_idx);

  // Normal should point towards viewpoint
  ASSERT_TRUE(n.dot(to_viewpoint) > 0);
}

void test_normals_viewpoint_below() {
  auto cloud = createPlaneXY(5, 0.0f);

  // Viewpoint below the plane
  Point viewpoint(2.0f, 2.0f, -10.0f);
  geometry::estimateNormals(cloud, 10, viewpoint);

  size_t center_idx = 12;
  Eigen::Vector3f n = cloud.normal(center_idx);

  // Normal should point downward (negative z)
  ASSERT_TRUE(n.z() < 0);
}

void test_normals_sparse_cloud() {
  auto cloud = createSparseCloud();
  geometry::estimateNormals(cloud, 10);

  ASSERT_TRUE(cloud.hasNormal());

  // With only 2 points, normals should be invalid (zero)
  auto n0 = cloud.normal(0);
  auto n1 = cloud.normal(1);

  ASSERT_NEAR(n0.norm(), 0.0f, 1e-5f);
  ASSERT_NEAR(n1.norm(), 0.0f, 1e-5f);
}

void test_normals_with_external_kdtree() {
  auto cloud = createPlaneXY(5, 0.0f);

  search::KdTree tree;
  tree.build(cloud);

  geometry::estimateNormals(cloud, tree, 10);

  ASSERT_TRUE(cloud.hasNormal());

  size_t center_idx = 12;
  auto n = cloud.normal(center_idx);

  ASSERT_NEAR(std::abs(n.z()), 1.0f, 0.1f);
}

// =============================================================================
// Covariance Estimation Tests
// =============================================================================

void test_covariances_planar() {
  auto cloud = createPlaneXY(5, 0.0f);
  geometry::estimateCovariances(cloud, 10);

  ASSERT_TRUE(cloud.hasCovariance());

  size_t center_idx = 12;
  const Covariance& cov = cloud.covariance(center_idx);

  // Eigendecompose to check regularization
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  Eigen::Vector3f eigenvalues = solver.eigenvalues();

  // GICP regularization: smallest eigenvalue should be ~epsilon (1e-3)
  ASSERT_NEAR(eigenvalues(0), 1e-3f, 1e-4f);
  // Larger eigenvalues should be ~1
  ASSERT_NEAR(eigenvalues(1), 1.0f, 0.1f);
  ASSERT_NEAR(eigenvalues(2), 1.0f, 0.1f);
}

void test_covariances_gicp_regularization() {
  auto cloud = createPlaneXY(5, 0.0f);
  geometry::estimateCovariances(cloud, 10);

  size_t center_idx = 12;
  const Covariance& cov = cloud.covariance(center_idx);

  // Covariance should be symmetric
  ASSERT_NEAR(cov(0, 1), cov(1, 0), 1e-6f);
  ASSERT_NEAR(cov(0, 2), cov(2, 0), 1e-6f);
  ASSERT_NEAR(cov(1, 2), cov(2, 1), 1e-6f);

  // Covariance should be positive semi-definite
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(solver.eigenvalues()(i) >= 0);
  }
}

void test_covariances_invalid_returns_identity() {
  auto cloud = createSparseCloud();
  geometry::estimateCovariances(cloud, 10);

  ASSERT_TRUE(cloud.hasCovariance());

  // Invalid covariance should be identity
  const Covariance& cov = cloud.covariance(0);
  ASSERT_NEAR(cov(0, 0), 1.0f, 1e-5f);
  ASSERT_NEAR(cov(1, 1), 1.0f, 1e-5f);
  ASSERT_NEAR(cov(2, 2), 1.0f, 1e-5f);
  ASSERT_NEAR(cov(0, 1), 0.0f, 1e-5f);
}

void test_covariances_with_external_kdtree() {
  auto cloud = createPlaneXY(5, 0.0f);

  search::KdTree tree;
  tree.build(cloud);

  geometry::estimateCovariances(cloud, tree, 10);

  ASSERT_TRUE(cloud.hasCovariance());

  // Should produce valid covariance
  const Covariance& cov = cloud.covariance(12);
  ASSERT_TRUE(cov.trace() > 0);
}

// =============================================================================
// Combined Normal + Covariance Tests
// =============================================================================

void test_normals_covariances_single_pass() {
  auto cloud1 = createPlaneXY(5, 0.0f);
  auto cloud2 = createPlaneXY(5, 0.0f);

  // Method 1: Combined
  geometry::estimateNormalsCovariances(cloud1, 10);

  // Method 2: Separate
  geometry::estimateNormals(cloud2, 10);
  geometry::estimateCovariances(cloud2, 10);

  ASSERT_TRUE(cloud1.hasNormal());
  ASSERT_TRUE(cloud1.hasCovariance());

  // Results should match
  for (size_t i = 0; i < cloud1.size(); ++i) {
    Eigen::Vector3f n1 = cloud1.normal(i);
    Eigen::Vector3f n2 = cloud2.normal(i);

    // Normals should be identical
    ASSERT_NEAR((n1 - n2).norm(), 0.0f, 1e-5f);

    // Covariances should be identical
    const Covariance& c1 = cloud1.covariance(i);
    const Covariance& c2 = cloud2.covariance(i);
    ASSERT_NEAR((c1 - c2).norm(), 0.0f, 1e-5f);
  }
}

void test_normals_covariances_with_external_kdtree() {
  auto cloud = createPlaneXY(5, 0.0f);

  search::KdTree tree;
  tree.build(cloud);

  geometry::estimateNormalsCovariances(cloud, tree, 10);

  ASSERT_TRUE(cloud.hasNormal());
  ASSERT_TRUE(cloud.hasCovariance());
}

// =============================================================================
// Edge Cases
// =============================================================================

void test_empty_cloud() {
  PointCloud cloud;

  geometry::estimateNormals(cloud, 10);
  ASSERT_FALSE(cloud.hasNormal());

  geometry::estimateCovariances(cloud, 10);
  ASSERT_FALSE(cloud.hasCovariance());

  geometry::estimateNormalsCovariances(cloud, 10);
  ASSERT_FALSE(cloud.hasNormal());
  ASSERT_FALSE(cloud.hasCovariance());
}

void test_single_point() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  geometry::estimateNormals(cloud, 10);

  ASSERT_TRUE(cloud.hasNormal());
  // Invalid normal (insufficient neighbors)
  ASSERT_NEAR(cloud.normal(0).norm(), 0.0f, 1e-5f);
}

void test_collinear_points() {
  auto cloud = createCollinearPoints();
  geometry::estimateNormals(cloud, 5);

  ASSERT_TRUE(cloud.hasNormal());

  // Collinear points have degenerate covariance
  // Normal may be invalid or arbitrary
  // Just verify no crash and channel is set
}

void test_normal_unit_length() {
  auto cloud = createPlaneXY(5, 0.0f);
  geometry::estimateNormals(cloud, 10);

  for (size_t i = 0; i < cloud.size(); ++i) {
    Eigen::Vector3f n = cloud.normal(i);
    float len = n.norm();
    // Either unit length or zero (invalid)
    ASSERT_TRUE(len < 1e-5f || std::abs(len - 1.0f) < 1e-5f);
  }
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "=== test_geometry ===\n";

  std::cout << "Normal Estimation:\n";
  TEST(normals_plane_xy);
  TEST(normals_plane_xz);
  TEST(normals_plane_arbitrary);
  TEST(normals_viewpoint_orientation);
  TEST(normals_viewpoint_below);
  TEST(normals_sparse_cloud);
  TEST(normals_with_external_kdtree);

  std::cout << "Covariance Estimation:\n";
  TEST(covariances_planar);
  TEST(covariances_gicp_regularization);
  TEST(covariances_invalid_returns_identity);
  TEST(covariances_with_external_kdtree);

  std::cout << "Combined:\n";
  TEST(normals_covariances_single_pass);
  TEST(normals_covariances_with_external_kdtree);

  std::cout << "Edge Cases:\n";
  TEST(empty_cloud);
  TEST(single_point);
  TEST(collinear_points);
  TEST(normal_unit_length);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
