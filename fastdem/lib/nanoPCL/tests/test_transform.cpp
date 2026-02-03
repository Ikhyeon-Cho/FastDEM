// nanoPCL - Test: Transform

#include <cassert>
#include <cmath>
#include <iostream>
#include <nanopcl/core/transform.hpp>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))
#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_TRUE(cond) assert(cond)

constexpr float EPS = 1e-5f;

// =============================================================================
// Basic Transform
// =============================================================================

void test_identity() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  auto result = transformCloud(cloud, Eigen::Matrix4f::Identity());

  ASSERT_NEAR(result.point(0).x(), 1.0f, EPS);
  ASSERT_NEAR(result.point(0).y(), 2.0f, EPS);
  ASSERT_NEAR(result.point(0).z(), 3.0f, EPS);
}

void test_translation() {
  PointCloud cloud;
  cloud.add(0, 0, 0);

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 10.0f;
  T(1, 3) = 20.0f;
  T(2, 3) = 30.0f;

  auto result = transformCloud(cloud, T);

  ASSERT_NEAR(result.point(0).x(), 10.0f, EPS);
  ASSERT_NEAR(result.point(0).y(), 20.0f, EPS);
  ASSERT_NEAR(result.point(0).z(), 30.0f, EPS);
}

void test_rotation_z_90() {
  PointCloud cloud;
  cloud.add(1, 0, 0);

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()).matrix();

  auto result = transformCloud(cloud, T);

  // (1,0,0) rotated 90 deg around Z -> (0,1,0)
  ASSERT_NEAR(result.point(0).x(), 0.0f, EPS);
  ASSERT_NEAR(result.point(0).y(), 1.0f, EPS);
  ASSERT_NEAR(result.point(0).z(), 0.0f, EPS);
}

void test_combined() {
  PointCloud cloud;
  cloud.add(1, 0, 0);

  // Rotate 90 deg around Z, then translate +10 in Y
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
  T.translation() = Eigen::Vector3f(0, 10, 0);

  auto result = transformCloud(cloud, T);

  // (1,0,0) -> rotate -> (0,1,0) -> translate -> (0,11,0)
  ASSERT_NEAR(result.point(0).x(), 0.0f, EPS);
  ASSERT_NEAR(result.point(0).y(), 11.0f, EPS);
  ASSERT_NEAR(result.point(0).z(), 0.0f, EPS);
}

// =============================================================================
// Copy vs In-place
// =============================================================================

void test_copy_preserves_original() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 100.0f;

  auto result = transformCloud(cloud, T);

  // Original unchanged
  ASSERT_NEAR(cloud.point(0).x(), 1.0f, EPS);
  ASSERT_NEAR(cloud.point(0).y(), 2.0f, EPS);
  ASSERT_NEAR(cloud.point(0).z(), 3.0f, EPS);

  // Result transformed
  ASSERT_NEAR(result.point(0).x(), 101.0f, EPS);
}

void test_inplace_modifies() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 100.0f;

  cloud = transformCloud(std::move(cloud), T);

  ASSERT_NEAR(cloud.point(0).x(), 101.0f, EPS);
  ASSERT_NEAR(cloud.point(0).y(), 2.0f, EPS);
  ASSERT_NEAR(cloud.point(0).z(), 3.0f, EPS);
}

// =============================================================================
// Channel Handling
// =============================================================================

void test_normals_rotation_only() {
  PointCloud cloud;
  cloud.useNormal();
  cloud.add(0, 0, 0);
  cloud.normals()[0] = Normal4(1, 0, 0, 0);

  // Translate + Rotate 90 deg around Z
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()).matrix();
  T(0, 3) = 100.0f; // translation should be ignored for normals

  auto result = transformCloud(cloud, T);

  // Normal (1,0,0) -> (0,1,0), translation ignored
  ASSERT_NEAR(result.normal(0).x(), 0.0f, EPS);
  ASSERT_NEAR(result.normal(0).y(), 1.0f, EPS);
  ASSERT_NEAR(result.normal(0).z(), 0.0f, EPS);
}

void test_covariances_unchanged() {
  PointCloud cloud;
  cloud.useCovariance();
  cloud.add(0, 0, 0);

  Covariance cov = Covariance::Identity() * 2.0f;
  cloud.covariances()[0] = cov;

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(M_PI / 4, Eigen::Vector3f::UnitX()).matrix();

  auto result = transformCloud(cloud, T);

  // Covariance should be unchanged
  ASSERT_NEAR((result.covariance(0) - cov).norm(), 0.0f, EPS);
}

// =============================================================================
// Overloads
// =============================================================================

void test_isometry3f_overload() {
  PointCloud cloud;
  cloud.add(0, 0, 0);

  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.translate(Eigen::Vector3f(5, 6, 7));

  auto result = transformCloud(cloud, T);

  ASSERT_NEAR(result.point(0).x(), 5.0f, EPS);
  ASSERT_NEAR(result.point(0).y(), 6.0f, EPS);
  ASSERT_NEAR(result.point(0).z(), 7.0f, EPS);
}

void test_isometry3d_overload() {
  PointCloud cloud;
  cloud.add(1, 0, 0);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  T.translation() = Eigen::Vector3d(0, 10, 0);

  auto result = transformCloud(cloud, T);

  // (1,0,0) -> rotate -> (0,1,0) -> translate -> (0,11,0)
  ASSERT_NEAR(result.point(0).x(), 0.0f, EPS);
  ASSERT_NEAR(result.point(0).y(), 11.0f, EPS);
  ASSERT_NEAR(result.point(0).z(), 0.0f, EPS);
}

void test_isometry3d_inplace() {
  PointCloud cloud;
  cloud.add(0, 0, 0);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(1.5, 2.5, 3.5);

  cloud = transformCloud(std::move(cloud), T);

  ASSERT_NEAR(cloud.point(0).x(), 1.5f, EPS);
  ASSERT_NEAR(cloud.point(0).y(), 2.5f, EPS);
  ASSERT_NEAR(cloud.point(0).z(), 3.5f, EPS);
}

// =============================================================================
// Edge Cases
// =============================================================================

void test_empty_cloud() {
  PointCloud cloud;

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 100.0f;

  auto result = transformCloud(cloud, T);

  ASSERT_TRUE(result.empty());
}

void test_multiple_points() {
  PointCloud cloud;
  cloud.add(1, 0, 0);
  cloud.add(0, 1, 0);
  cloud.add(0, 0, 1);

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 10.0f;

  auto result = transformCloud(cloud, T);

  ASSERT_EQ(result.size(), 3u);
  ASSERT_NEAR(result.point(0).x(), 11.0f, EPS);
  ASSERT_NEAR(result.point(1).x(), 10.0f, EPS);
  ASSERT_NEAR(result.point(2).x(), 10.0f, EPS);
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "=== test_transform ===\n";

  std::cout << "Basic:\n";
  TEST(identity);
  TEST(translation);
  TEST(rotation_z_90);
  TEST(combined);

  std::cout << "Copy vs In-place:\n";
  TEST(copy_preserves_original);
  TEST(inplace_modifies);

  std::cout << "Channels:\n";
  TEST(normals_rotation_only);
  TEST(covariances_unchanged);

  std::cout << "Overloads:\n";
  TEST(isometry3f_overload);
  TEST(isometry3d_overload);
  TEST(isometry3d_inplace);

  std::cout << "Edge Cases:\n";
  TEST(empty_cloud);
  TEST(multiple_points);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
