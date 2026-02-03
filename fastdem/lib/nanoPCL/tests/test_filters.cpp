// nanoPCL - Test: Filters Module

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <nanopcl/filters/core.hpp>
#include <nanopcl/filters/crop.hpp>
#include <nanopcl/filters/deskew.hpp>
#include <nanopcl/filters/downsample.hpp>
#include <nanopcl/filters/outlier_removal.hpp>

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

PointCloud createWithChannels() {
  PointCloud cloud;
  cloud.useIntensity();
  cloud.useColor();
  cloud.useCovariance();

  for (int i = 0; i < 10; ++i) {
    cloud.add(float(i), float(i), float(i), Intensity(float(i) * 0.1f));
    cloud.color(i) = Color(i * 10, i * 20, i * 30);
    cloud.covariance(i) = Covariance::Identity() * float(i);
  }
  return cloud;
}

// =============================================================================
// 1. Core Filter Tests
// =============================================================================

void test_filter_basic() {
  PointCloud cloud = createGrid3x3x3();
  ASSERT_EQ(cloud.size(), 27);

  // Keep only points with x > 0
  auto filtered =
      filters::filter(cloud, [&](size_t i) { return cloud.point(i).x() > 0; });

  ASSERT_EQ(filtered.size(), 18); // 2 * 3 * 3 = 18
}

void test_filter_move_semantics() {
  PointCloud cloud = createGrid3x3x3();

  // Filter using move
  auto filtered = filters::filter(
      std::move(cloud), [&](size_t i) { return cloud.point(i).x() > 0; });

  ASSERT_EQ(filtered.size(), 18);
}

void test_filter_channel_preservation() {
  PointCloud cloud = createWithChannels();
  ASSERT_EQ(cloud.size(), 10);

  // Keep points with x >= 5
  auto filtered = filters::filter(
      cloud, [&](size_t i) { return cloud.point(i).x() >= 5.0f; });

  ASSERT_EQ(filtered.size(), 5);
  ASSERT_TRUE(filtered.hasIntensity());
  ASSERT_TRUE(filtered.hasColor());
  ASSERT_TRUE(filtered.hasCovariance());

  // Check first point (was index 5 in original)
  ASSERT_NEAR(filtered.intensity(0), 0.5f, 0.01f);
  ASSERT_EQ(filtered.color(0).r, 50);
  ASSERT_NEAR(filtered.covariance(0)(0, 0), 5.0f, 0.01f);
}

void test_removeInvalid() {
  PointCloud cloud;
  cloud.add(1.0f, 2.0f, 3.0f);
  cloud.add(std::numeric_limits<float>::quiet_NaN(), 0, 0);
  cloud.add(0, std::numeric_limits<float>::infinity(), 0);
  cloud.add(4.0f, 5.0f, 6.0f);

  ASSERT_EQ(cloud.size(), 4);

  auto cleaned = filters::removeInvalid(cloud);
  ASSERT_EQ(cleaned.size(), 2);
  ASSERT_NEAR(cleaned.point(0).x(), 1.0f, 0.01f);
  ASSERT_NEAR(cleaned.point(1).x(), 4.0f, 0.01f);
}

// =============================================================================
// 2. Crop Filter Tests
// =============================================================================

void test_cropBox_inside() {
  PointCloud cloud = createGrid3x3x3();

  auto cropped =
      filters::cropBox(cloud, Point(0.5f, 0.5f, 0.5f), Point(1.5f, 1.5f, 1.5f), filters::FilterMode::INSIDE);

  ASSERT_EQ(cropped.size(), 1); // Only (1,1,1)
  ASSERT_NEAR(cropped.point(0).x(), 1.0f, 0.01f);
}

void test_cropBox_outside() {
  PointCloud cloud = createGrid3x3x3();

  auto cropped =
      filters::cropBox(cloud, Point(0.5f, 0.5f, 0.5f), Point(1.5f, 1.5f, 1.5f), filters::FilterMode::OUTSIDE);

  ASSERT_EQ(cropped.size(), 26); // 27 - 1 = 26
}

void test_cropRange() {
  PointCloud cloud;
  cloud.add(0, 0, 0);  // dist = 0
  cloud.add(1, 0, 0);  // dist = 1
  cloud.add(2, 0, 0);  // dist = 2
  cloud.add(10, 0, 0); // dist = 10

  auto cropped = filters::cropRange(cloud, 0.5f, 5.0f);
  ASSERT_EQ(cropped.size(), 2); // dist 1 and 2
}

void test_cropZ() {
  PointCloud cloud = createGrid3x3x3();

  auto cropped = filters::cropZ(cloud, 1.0f, 1.0f);
  ASSERT_EQ(cropped.size(), 9); // 3 * 3 = 9 points at z=1
}

// =============================================================================
// 3. VoxelGrid Tests
// =============================================================================

void test_voxelGrid_centroid() {
  PointCloud cloud;
  // 4 points in same voxel (voxel_size = 2)
  cloud.add(0.0f, 0.0f, 0.0f);
  cloud.add(1.0f, 0.0f, 0.0f);
  cloud.add(0.0f, 1.0f, 0.0f);
  cloud.add(1.0f, 1.0f, 0.0f);

  auto downsampled =
      filters::voxelGrid(cloud, 2.0f, filters::VoxelMode::CENTROID);

  ASSERT_EQ(downsampled.size(), 1);
  ASSERT_NEAR(downsampled.point(0).x(), 0.5f, 0.01f);
  ASSERT_NEAR(downsampled.point(0).y(), 0.5f, 0.01f);
  ASSERT_NEAR(downsampled.point(0).z(), 0.0f, 0.01f);
}

void test_voxelGrid_nearest() {
  PointCloud cloud;
  // 4 points in same voxel
  cloud.add(0.1f, 0.1f, 0.0f);
  cloud.add(0.9f, 0.1f, 0.0f);
  cloud.add(0.1f, 0.9f, 0.0f);
  cloud.add(0.5f, 0.5f, 0.0f); // Closest to center (0.5, 0.5, 0.5)

  auto downsampled =
      filters::voxelGrid(cloud, 1.0f, filters::VoxelMode::NEAREST);

  ASSERT_EQ(downsampled.size(), 1);
  // Should pick point closest to voxel center
  ASSERT_NEAR(downsampled.point(0).x(), 0.5f, 0.01f);
  ASSERT_NEAR(downsampled.point(0).y(), 0.5f, 0.01f);
}

void test_voxelGrid_channel_averaging() {
  PointCloud cloud;
  cloud.useIntensity();

  cloud.add(0.0f, 0.0f, 0.0f);
  cloud.intensity(0) = 0.2f;
  cloud.add(0.5f, 0.5f, 0.0f);
  cloud.intensity(1) = 0.8f;

  auto downsampled =
      filters::voxelGrid(cloud, 1.0f, filters::VoxelMode::CENTROID);

  ASSERT_EQ(downsampled.size(), 1);
  ASSERT_TRUE(downsampled.hasIntensity());
  ASSERT_NEAR(downsampled.intensity(0), 0.5f, 0.01f); // Average of 0.2 and 0.8
}

void test_voxelGrid_move_semantics() {
  PointCloud cloud = createGrid3x3x3();
  size_t original_size = cloud.size();

  auto downsampled = filters::voxelGrid(std::move(cloud), 1.5f);

  // Should have fewer points
  ASSERT_TRUE(downsampled.size() < original_size);
  ASSERT_TRUE(downsampled.size() > 0);
}

void test_voxelGrid_covariance_preservation() {
  PointCloud cloud;
  cloud.useCovariance();

  cloud.add(0.0f, 0.0f, 0.0f);
  cloud.covariance(0) = Covariance::Identity() * 2.0f;
  cloud.add(0.5f, 0.5f, 0.0f);
  cloud.covariance(1) = Covariance::Identity() * 4.0f;

  auto downsampled =
      filters::voxelGrid(cloud, 1.0f, filters::VoxelMode::CENTROID);

  ASSERT_EQ(downsampled.size(), 1);
  ASSERT_TRUE(downsampled.hasCovariance());
  // Covariance uses representative (first point's value)
  ASSERT_NEAR(downsampled.covariance(0)(0, 0), 2.0f, 0.01f);
}

void test_voxelGrid_symmetry() {
  // Regression test for in-place overwrite bug
  // Create symmetric grid
  PointCloud cloud;
  for (float x = -10.0f; x <= 10.0f; x += 0.3f) {
    for (float y = -10.0f; y <= 10.0f; y += 0.3f) {
      cloud.add(x, y, 0.0f);
    }
  }

  size_t orig_neg_x = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud[i].x() < 0) orig_neg_x++;
  }

  auto downsampled = filters::voxelGrid(cloud, 0.3f);

  // Count negative x points after downsampling
  size_t down_neg_x = 0;
  for (size_t i = 0; i < downsampled.size(); ++i) {
    if (downsampled[i].x() < 0) down_neg_x++;
  }

  // Check symmetry: ratio should be approximately same
  float orig_ratio = static_cast<float>(orig_neg_x) / cloud.size();
  float down_ratio = static_cast<float>(down_neg_x) / downsampled.size();

  // Allow 5% deviation for edge effects
  ASSERT_TRUE(std::abs(orig_ratio - down_ratio) < 0.05f);
  ASSERT_TRUE(down_ratio > 0.45f && down_ratio < 0.55f); // Should be ~0.5
}

// =============================================================================
// 4. GridMaxZ Tests
// =============================================================================

void test_gridMaxZ_basic() {
  PointCloud cloud;
  // Same (x, y), different z
  cloud.add(0.0f, 0.0f, 1.0f);
  cloud.add(0.0f, 0.0f, 5.0f); // Max
  cloud.add(0.0f, 0.0f, 3.0f);

  auto result = filters::gridMaxZ(cloud, 1.0f);

  ASSERT_EQ(result.size(), 1);
  ASSERT_NEAR(result.point(0).z(), 5.0f, 0.01f);
}

void test_gridMaxZ_multiple_cells() {
  PointCloud cloud;
  // Cell 1: (0,0)
  cloud.add(0.0f, 0.0f, 1.0f);
  cloud.add(0.0f, 0.0f, 3.0f); // Max for cell 1

  // Cell 2: (2,0)
  cloud.add(2.0f, 0.0f, 5.0f);
  cloud.add(2.0f, 0.0f, 2.0f); // Max for cell 2 is 5.0

  auto result = filters::gridMaxZ(cloud, 1.0f);

  ASSERT_EQ(result.size(), 2);
}

void test_gridMaxZ_channel_preservation() {
  PointCloud cloud;
  cloud.useIntensity();

  cloud.add(0.0f, 0.0f, 1.0f);
  cloud.intensity(0) = 0.1f;
  cloud.add(0.0f, 0.0f, 5.0f); // Max z
  cloud.intensity(1) = 0.9f;

  auto result = filters::gridMaxZ(cloud, 1.0f);

  ASSERT_EQ(result.size(), 1);
  ASSERT_TRUE(result.hasIntensity());
  ASSERT_NEAR(result.intensity(0), 0.9f, 0.01f); // Intensity of max-z point
}

// =============================================================================
// 5. Outlier Removal Tests
// =============================================================================

void test_radiusOutlierRemoval_basic() {
  PointCloud cloud;
  // Dense cluster
  for (int i = 0; i < 10; ++i) {
    cloud.add(float(i) * 0.1f, 0.0f, 0.0f);
  }
  // Isolated outlier (far away)
  cloud.add(100.0f, 0.0f, 0.0f);

  ASSERT_EQ(cloud.size(), 11);

  auto filtered = filters::radiusOutlierRemoval(cloud, 0.5f, 2);

  // Outlier should be removed (has no neighbors within radius)
  ASSERT_EQ(filtered.size(), 10);
}

void test_radiusOutlierRemoval_all_inliers() {
  PointCloud cloud;
  // Dense grid - all points have neighbors
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      cloud.add(float(x) * 0.5f, float(y) * 0.5f, 0.0f);
    }
  }

  auto filtered = filters::radiusOutlierRemoval(cloud, 1.0f, 1);

  // All points should remain
  ASSERT_EQ(filtered.size(), 9);
}

void test_radiusOutlierRemoval_channel_preservation() {
  PointCloud cloud;
  cloud.useIntensity();

  // Dense cluster
  for (int i = 0; i < 5; ++i) {
    cloud.add(float(i) * 0.1f, 0.0f, 0.0f, Intensity(float(i) * 0.2f));
  }
  // Outlier
  cloud.add(100.0f, 0.0f, 0.0f, Intensity(0.99f));

  auto filtered = filters::radiusOutlierRemoval(cloud, 0.5f, 2);

  ASSERT_TRUE(filtered.hasIntensity());
  ASSERT_EQ(filtered.size(), 5);
  // Check intensity preserved
  ASSERT_NEAR(filtered.intensity(0), 0.0f, 0.01f);
  ASSERT_NEAR(filtered.intensity(4), 0.8f, 0.01f);
}

void test_radiusOutlierRemoval_move() {
  PointCloud cloud;
  for (int i = 0; i < 10; ++i) {
    cloud.add(float(i) * 0.1f, 0.0f, 0.0f);
  }
  cloud.add(100.0f, 0.0f, 0.0f); // outlier

  auto filtered = filters::radiusOutlierRemoval(std::move(cloud), 0.5f, 2);
  ASSERT_EQ(filtered.size(), 10);
}

void test_statisticalOutlierRemoval_all_inliers() {
  PointCloud cloud;
  // Uniform grid - all points have similar neighbor distances
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 3; ++z) {
        cloud.add(float(x), float(y), float(z));
      }
    }
  }

  // With high std_mul, all points should remain
  auto filtered = filters::statisticalOutlierRemoval(cloud, 4, 3.0f);

  ASSERT_EQ(filtered.size(), 27); // All points remain
}

void test_statisticalOutlierRemoval_basic() {
  PointCloud cloud;
  // Dense cluster with consistent spacing
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      cloud.add(float(x), float(y), 0.0f);
    }
  }
  // Isolated outlier (much farther than average neighbor distance)
  cloud.add(100.0f, 100.0f, 0.0f);

  ASSERT_EQ(cloud.size(), 26);

  // k=4 neighbors, std_mul=1.0 (strict)
  auto filtered = filters::statisticalOutlierRemoval(cloud, 4, 1.0f);

  // Outlier should be removed
  ASSERT_TRUE(filtered.size() < 26);
  ASSERT_TRUE(filtered.size() >= 24); // Most inliers should remain
}

void test_statisticalOutlierRemoval_channel_preservation() {
  PointCloud cloud;
  cloud.useIntensity();
  cloud.useColor();

  for (int i = 0; i < 9; ++i) {
    cloud.add(float(i % 3), float(i / 3), 0.0f, Intensity(float(i) * 0.1f));
    cloud.color(i) = Color(i * 10, i * 20, i * 30);
  }
  // Outlier
  cloud.add(100.0f, 100.0f, 0.0f, Intensity(0.99f));
  cloud.color(9) = Color(255, 255, 255);

  auto filtered = filters::statisticalOutlierRemoval(cloud, 3, 1.0f);

  ASSERT_TRUE(filtered.hasIntensity());
  ASSERT_TRUE(filtered.hasColor());
  // Verify some channel data preserved
  if (filtered.size() > 0) {
    ASSERT_TRUE(filtered.color(0).r < 200); // Not the outlier's color
  }
}

void test_statisticalOutlierRemoval_move() {
  PointCloud cloud;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      cloud.add(float(x), float(y), 0.0f);
    }
  }
  cloud.add(100.0f, 100.0f, 0.0f);

  auto filtered = filters::statisticalOutlierRemoval(std::move(cloud), 4, 1.0f);
  ASSERT_TRUE(filtered.size() >= 24);
}

void test_outlierRemoval_empty_cloud() {
  PointCloud empty;

  auto radius_filtered = filters::radiusOutlierRemoval(empty, 1.0f, 3);
  ASSERT_TRUE(radius_filtered.empty());

  auto stat_filtered = filters::statisticalOutlierRemoval(empty, 5, 1.0f);
  ASSERT_TRUE(stat_filtered.empty());
}

void test_outlierRemoval_single_point() {
  PointCloud single;
  single.add(1.0f, 2.0f, 3.0f);

  // Radius: single point has no neighbors
  auto radius_filtered = filters::radiusOutlierRemoval(single, 1.0f, 1);
  ASSERT_TRUE(radius_filtered.empty());

  // Statistical: k=0 or k>=n returns original
  auto stat_filtered = filters::statisticalOutlierRemoval(single, 0, 1.0f);
  ASSERT_TRUE(stat_filtered.empty());
}

// =============================================================================
// 6. Deskew Tests
// =============================================================================

// Helper: Create a "distorted" line (simulating motion during scan)
PointCloud createDistortedLine() {
  PointCloud cloud;
  cloud.useTime();

  // Pillar at X=5.0m in world frame
  // Sensor moves from (0,0,0) to (1,0,0) during scan
  // Due to motion, sensor measures pillar at different relative X positions
  for (int i = 0; i < 100; ++i) {
    float ratio = static_cast<float>(i) / 99.0f;
    float world_x = 5.0f;
    float sensor_x_at_t = world_x - ratio * 1.0f; // Sensor moved by 'ratio' meters

    cloud.add(sensor_x_at_t, 0.0f, static_cast<float>(i) * 0.1f);
    cloud.time(i) = ratio; // Normalized time [0, 1]
  }
  return cloud;
}

void test_deskew_linear_basic() {
  PointCloud cloud = createDistortedLine();
  ASSERT_EQ(cloud.size(), 100);

  // Original: X ranges from 5.0 (t=0) to 4.0 (t=1)
  ASSERT_NEAR(cloud[0].x(), 5.0f, 0.01f);
  ASSERT_NEAR(cloud[99].x(), 4.0f, 0.01f);

  // Poses
  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.translation() = Eigen::Vector3d(1.0, 0, 0);

  // Deskew to T_end frame
  auto corrected = filters::deskew(cloud, T_start, T_end);

  // After deskewing to T_end frame:
  // All points should be at X = 5.0 - 1.0 = 4.0 (world pillar minus sensor end position)
  for (size_t i = 0; i < corrected.size(); ++i) {
    ASSERT_NEAR(corrected[i].x(), 4.0f, 0.02f);
  }
}

void test_deskew_linear_with_rotation() {
  PointCloud cloud;
  cloud.useTime();

  // Single point at (1, 0, 0) in sensor frame
  cloud.add(1.0f, 0.0f, 0.0f);
  cloud.time(0) = 0.0f; // At start time

  // Sensor rotates 90 degrees around Z during scan
  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  // Use explicit time range (single point can't auto-detect range)
  auto corrected = filters::deskew(cloud, T_start, T_end, 0.0, 1.0);

  // Point was at (1,0,0) at t=0 in T_start frame
  // In T_end frame (rotated 90 deg), this point should be at (0, -1, 0)
  // Because T_end^-1 * T_start = rotation by -90 deg
  ASSERT_NEAR(corrected[0].x(), 0.0f, 0.01f);
  ASSERT_NEAR(corrected[0].y(), -1.0f, 0.01f);
}

void test_deskew_index_strategy() {
  // Same scenario as deskew_linear_basic, but without time channel
  // Use INDEX strategy: index 0~99 maps to ratio 0~1

  PointCloud cloud;
  // No time channel - use INDEX strategy

  // Pillar at X=5.0m in world frame
  // Sensor moves from (0,0,0) to (1,0,0) during scan
  for (int i = 0; i < 100; ++i) {
    float ratio = static_cast<float>(i) / 99.0f;
    float world_x = 5.0f;
    float sensor_x_at_t = world_x - ratio * 1.0f;
    cloud.add(sensor_x_at_t, 0.0f, static_cast<float>(i) * 0.1f);
  }

  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.translation() = Eigen::Vector3d(1.0, 0, 0);

  auto corrected =
      filters::deskew(cloud, T_start, T_end, filters::TimeStrategy::INDEX);

  // After deskewing to T_end frame: all points at X=4.0
  ASSERT_EQ(corrected.size(), 100);
  for (size_t i = 0; i < corrected.size(); ++i) {
    ASSERT_NEAR(corrected[i].x(), 4.0f, 0.02f);
  }
}

void test_deskew_callback_api() {
  PointCloud cloud;
  cloud.useTime();

  // Two points: one at mid-scan, one at end
  cloud.add(1.0f, 0.0f, 0.0f);
  cloud.time(0) = 0.5; // Mid-scan

  cloud.add(0.0f, 0.0f, 0.0f); // Reference point at end
  cloud.time(1) = 1.0;         // End of scan

  // Custom pose lookup: linear motion
  auto pose_lookup = [](double t) -> Eigen::Isometry3d {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(t * 2.0, 0, 0); // Move 2m over t=[0,1]
    return T;
  };

  auto corrected = filters::deskew(cloud, pose_lookup);

  // Point at (1,0,0) measured at t=0.5 when sensor at x=1.0
  // World position: 1 + 1 = 2
  // T_end (t=1.0) sensor at x=2.0
  // In T_end frame: 2 - 2 = 0
  ASSERT_NEAR(corrected[0].x(), 0.0f, 0.01f);
}

void test_deskew_explicit_time_range() {
  PointCloud cloud;
  cloud.useTime();

  // Time values in absolute seconds
  cloud.add(1.0f, 0.0f, 0.0f);
  cloud.time(0) = 100.05f; // Mid-way between 100.0 and 100.1

  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.translation() = Eigen::Vector3d(1.0, 0, 0);

  auto corrected = filters::deskew(cloud, T_start, T_end, 100.0, 100.1);

  // Point measured at t=100.05 (ratio 0.5) at (1,0,0)
  // Sensor at ratio 0.5: position (0.5, 0, 0)
  // World position: 1 + 0.5 = 1.5
  // T_end sensor at (1,0,0)
  // In T_end frame: 1.5 - 1 = 0.5
  ASSERT_NEAR(corrected[0].x(), 0.5f, 0.01f);
}

void test_deskew_move_semantics() {
  PointCloud cloud = createDistortedLine();

  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.translation() = Eigen::Vector3d(1.0, 0, 0);

  auto corrected = filters::deskew(std::move(cloud), T_start, T_end);

  ASSERT_EQ(corrected.size(), 100);
  // All points should be corrected
  for (size_t i = 0; i < corrected.size(); ++i) {
    ASSERT_NEAR(corrected[i].x(), 4.0f, 0.02f);
  }
}

void test_deskew_with_normals() {
  PointCloud cloud;
  cloud.useTime();
  cloud.useNormal();

  // Point with normal pointing in +X direction
  cloud.add(1.0f, 0.0f, 0.0f);
  cloud.time(0) = 0.0f;
  cloud.normal(0) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  // Sensor rotates 90 degrees
  Eigen::Isometry3d T_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_end = Eigen::Isometry3d::Identity();
  T_end.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  // Use explicit time range (single point can't auto-detect range)
  auto corrected = filters::deskew(cloud, T_start, T_end, 0.0, 1.0);

  // Normal was (1,0,0) at T_start
  // In T_end frame (rotated 90 deg), normal should be (0,-1,0)
  ASSERT_NEAR(corrected.normal(0).x(), 0.0f, 0.01f);
  ASSERT_NEAR(corrected.normal(0).y(), -1.0f, 0.01f);
}

void test_deskew_empty_cloud() {
  PointCloud empty;

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  auto result1 = filters::deskew(empty, T, T);
  ASSERT_TRUE(result1.empty());

  auto result2 = filters::deskew(empty, [](double) { return Eigen::Isometry3d::Identity(); });
  ASSERT_TRUE(result2.empty());
}

void test_deskew_no_motion() {
  PointCloud cloud = createDistortedLine();

  // No motion: T_start == T_end
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  auto corrected = filters::deskew(cloud, T, T);

  // Points should remain unchanged
  ASSERT_NEAR(corrected[0].x(), cloud[0].x(), 0.01f);
  ASSERT_NEAR(corrected[99].x(), cloud[99].x(), 0.01f);
}

// =============================================================================
// 7. Boundary Condition Tests
// =============================================================================

void test_empty_cloud() {
  PointCloud empty;

  auto filtered = filters::filter(empty, [](size_t) { return true; });
  ASSERT_TRUE(filtered.empty());

  auto cropped = filters::cropBox(empty, Point::Zero(), Point::Ones());
  ASSERT_TRUE(cropped.empty());

  auto voxelized = filters::voxelGrid(empty, 1.0f);
  ASSERT_TRUE(voxelized.empty());

  auto maxz = filters::gridMaxZ(empty, 1.0f);
  ASSERT_TRUE(maxz.empty());
}

void test_single_point() {
  PointCloud single;
  single.add(1.0f, 2.0f, 3.0f);

  auto voxelized = filters::voxelGrid(single, 1.0f);
  ASSERT_EQ(voxelized.size(), 1);

  auto maxz = filters::gridMaxZ(single, 1.0f);
  ASSERT_EQ(maxz.size(), 1);
}

void test_nan_handling_in_voxelGrid() {
  PointCloud cloud;
  cloud.add(1.0f, 2.0f, 3.0f);
  cloud.add(std::numeric_limits<float>::quiet_NaN(), 0, 0);
  cloud.add(4.0f, 5.0f, 6.0f);

  auto voxelized = filters::voxelGrid(cloud, 10.0f);
  ASSERT_EQ(voxelized.size(), 1); // NaN skipped, others merged
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "\n=== nanoPCL Filters Tests ===\n\n";

  std::cout << "[Core Filters]\n";
  TEST(filter_basic);
  TEST(filter_move_semantics);
  TEST(filter_channel_preservation);
  TEST(removeInvalid);

  std::cout << "\n[Crop Filters]\n";
  TEST(cropBox_inside);
  TEST(cropBox_outside);
  TEST(cropRange);
  TEST(cropZ);

  std::cout << "\n[VoxelGrid]\n";
  TEST(voxelGrid_centroid);
  TEST(voxelGrid_nearest);
  TEST(voxelGrid_channel_averaging);
  TEST(voxelGrid_move_semantics);
  TEST(voxelGrid_covariance_preservation);
  TEST(voxelGrid_symmetry);

  std::cout << "\n[GridMaxZ]\n";
  TEST(gridMaxZ_basic);
  TEST(gridMaxZ_multiple_cells);
  TEST(gridMaxZ_channel_preservation);

  std::cout << "\n[Outlier Removal]\n";
  TEST(radiusOutlierRemoval_basic);
  TEST(radiusOutlierRemoval_all_inliers);
  TEST(radiusOutlierRemoval_channel_preservation);
  TEST(radiusOutlierRemoval_move);
  TEST(statisticalOutlierRemoval_all_inliers);
  TEST(statisticalOutlierRemoval_basic);
  TEST(statisticalOutlierRemoval_channel_preservation);
  TEST(statisticalOutlierRemoval_move);
  TEST(outlierRemoval_empty_cloud);
  TEST(outlierRemoval_single_point);

  std::cout << "\n[Deskew]\n";
  TEST(deskew_linear_basic);
  TEST(deskew_linear_with_rotation);
  TEST(deskew_index_strategy);
  TEST(deskew_callback_api);
  TEST(deskew_explicit_time_range);
  TEST(deskew_move_semantics);
  TEST(deskew_with_normals);
  TEST(deskew_empty_cloud);
  TEST(deskew_no_motion);

  std::cout << "\n[Boundary Conditions]\n";
  TEST(empty_cloud);
  TEST(single_point);
  TEST(nan_handling_in_voxelGrid);

  std::cout << "\n=== All filters tests passed! ===\n\n";
  return 0;
}
