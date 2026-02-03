// nanoPCL - Test: IO Module
// Tests for PCD, BIN, and Trajectory file I/O.

#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

#include <nanopcl/io.hpp>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_TRUE(cond) assert(cond)
#define ASSERT_FALSE(cond) assert(!(cond))
#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))

constexpr float EPS = 1e-5f;
constexpr double EPSD = 1e-9;

// =============================================================================
// PCD I/O Tests
// =============================================================================

void test_pcd_ascii_roundtrip() {
  // Create test cloud
  PointCloud cloud;
  cloud.useIntensity();
  cloud.add(1.0f, 2.0f, 3.0f, Intensity(100.0f));
  cloud.add(4.0f, 5.0f, 6.0f, Intensity(200.0f));

  // Save to stream
  std::stringstream ss;
  io::savePCD(ss, cloud, {.format = io::PCDFormat::ASCII});

  // Load back
  ss.seekg(0);
  io::PCDMetadata metadata;
  PointCloud loaded = io::loadPCD(ss, metadata);

  // Verify
  ASSERT_EQ(loaded.size(), 2u);
  ASSERT_NEAR(loaded.point(0).x(), 1.0f, EPS);
  ASSERT_NEAR(loaded.point(0).y(), 2.0f, EPS);
  ASSERT_NEAR(loaded.point(0).z(), 3.0f, EPS);
  ASSERT_NEAR(loaded.point(1).x(), 4.0f, EPS);

  ASSERT_TRUE(loaded.hasIntensity());
  ASSERT_NEAR(loaded.intensity(0), 100.0f, EPS);
  ASSERT_NEAR(loaded.intensity(1), 200.0f, EPS);
}

void test_pcd_binary_roundtrip() {
  // Create test cloud with multiple channels
  PointCloud cloud;
  cloud.useIntensity();
  cloud.useNormal();

  cloud.add(1.5f, 2.5f, 3.5f);
  cloud.intensities().back() = 50.0f;
  cloud.normals().back() = Normal4(0.0f, 0.0f, 1.0f, 0.0f);

  cloud.add(-1.0f, -2.0f, -3.0f);
  cloud.intensities().back() = 75.0f;
  cloud.normals().back() = Normal4(1.0f, 0.0f, 0.0f, 0.0f);

  // Save to binary stream
  std::stringstream ss(std::ios::binary | std::ios::in | std::ios::out);
  io::savePCD(ss, cloud, {.format = io::PCDFormat::BINARY});

  // Load back
  ss.seekg(0);
  io::PCDMetadata metadata;
  PointCloud loaded = io::loadPCD(ss, metadata);

  // Verify
  ASSERT_EQ(loaded.size(), 2u);
  ASSERT_NEAR(loaded.point(0).x(), 1.5f, EPS);
  ASSERT_NEAR(loaded.point(1).z(), -3.0f, EPS);

  ASSERT_TRUE(loaded.hasIntensity());
  ASSERT_NEAR(loaded.intensity(0), 50.0f, EPS);

  ASSERT_TRUE(loaded.hasNormal());
  ASSERT_NEAR(loaded.normal(0).z(), 1.0f, EPS);
  ASSERT_NEAR(loaded.normal(1).x(), 1.0f, EPS);
}

void test_pcd_viewpoint() {
  PointCloud cloud;
  cloud.add(0, 0, 0);

  // Set viewpoint
  io::PCDSaveOptions opts;
  opts.format = io::PCDFormat::ASCII;
  opts.viewpoint = Eigen::Isometry3d::Identity();
  opts.viewpoint.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  opts.viewpoint.rotate(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));

  std::stringstream ss;
  io::savePCD(ss, cloud, opts);

  // Load and check viewpoint
  ss.seekg(0);
  io::PCDMetadata meta;
  io::loadPCD(ss, meta);

  ASSERT_NEAR(meta.viewpoint.translation().x(), 1.0, EPSD);
  ASSERT_NEAR(meta.viewpoint.translation().y(), 2.0, EPSD);
  ASSERT_NEAR(meta.viewpoint.translation().z(), 3.0, EPSD);
}

void test_pcd_rgb_channel() {
  PointCloud cloud;
  cloud.useColor();
  cloud.add(0, 0, 0);
  cloud.colors().back() = Color(255, 128, 64);

  std::stringstream ss;
  io::savePCD(ss, cloud, {.format = io::PCDFormat::BINARY});

  ss.seekg(0);
  io::PCDMetadata meta;
  PointCloud loaded = io::loadPCD(ss, meta);

  ASSERT_TRUE(loaded.hasColor());
  ASSERT_EQ(static_cast<int>(loaded.color(0).r), 255);
  ASSERT_EQ(static_cast<int>(loaded.color(0).g), 128);
  ASSERT_EQ(static_cast<int>(loaded.color(0).b), 64);
}

void test_pcd_empty_cloud() {
  PointCloud empty;

  std::stringstream ss;
  io::savePCD(ss, empty, {.format = io::PCDFormat::ASCII});

  ss.seekg(0);
  io::PCDMetadata meta;
  PointCloud loaded = io::loadPCD(ss, meta);

  ASSERT_TRUE(loaded.empty());
  ASSERT_EQ(meta.num_points, 0u);
}

void test_pcd_exception_on_bad_stream() {
  std::ifstream bad_stream("nonexistent_file_12345.pcd");
  bool threw = false;

  try {
    io::PCDMetadata meta;
    io::loadPCD(bad_stream, meta);
  } catch (const io::IOException&) {
    threw = true;
  }

  ASSERT_TRUE(threw);
}

// =============================================================================
// BIN I/O Tests
// =============================================================================

void test_bin_roundtrip() {
  PointCloud cloud;
  cloud.useIntensity();

  cloud.add(1.0f, 2.0f, 3.0f);
  cloud.intensities().back() = 0.5f;
  cloud.add(4.0f, 5.0f, 6.0f);
  cloud.intensities().back() = 0.75f;

  std::stringstream ss(std::ios::binary | std::ios::in | std::ios::out);
  io::saveBIN(ss, cloud);

  ss.seekg(0);
  PointCloud loaded = io::loadBIN(ss);

  ASSERT_EQ(loaded.size(), 2u);
  ASSERT_NEAR(loaded.point(0).x(), 1.0f, EPS);
  ASSERT_NEAR(loaded.point(1).z(), 6.0f, EPS);

  ASSERT_TRUE(loaded.hasIntensity());
  ASSERT_NEAR(loaded.intensity(0), 0.5f, EPS);
  ASSERT_NEAR(loaded.intensity(1), 0.75f, EPS);
}

void test_bin_no_intensity_save() {
  // Cloud without intensity - should save 0.0 for intensity
  PointCloud cloud;
  cloud.add(1, 2, 3);

  std::stringstream ss(std::ios::binary | std::ios::in | std::ios::out);
  io::saveBIN(ss, cloud);

  ss.seekg(0);
  PointCloud loaded = io::loadBIN(ss);

  ASSERT_EQ(loaded.size(), 1u);
  ASSERT_TRUE(loaded.hasIntensity()); // BIN always loads with intensity
  ASSERT_NEAR(loaded.intensity(0), 0.0f, EPS);
}

// =============================================================================
// Trajectory I/O Tests
// =============================================================================

void test_trajectory_tum_roundtrip() {
  io::Trajectory traj;

  io::StampedPose sp1;
  sp1.timestamp = 1000.123456;
  sp1.pose.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  sp1.pose.rotate(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  traj.push_back(sp1);

  io::StampedPose sp2;
  sp2.timestamp = 1000.234567;
  sp2.pose.translation() = Eigen::Vector3d(4.0, 5.0, 6.0);
  traj.push_back(sp2);

  std::stringstream ss;
  io::saveTrajectoryTUM(ss, traj);

  ss.seekg(0);
  io::Trajectory loaded = io::loadTrajectoryTUM(ss);

  ASSERT_EQ(loaded.size(), 2u);
  ASSERT_NEAR(loaded[0].timestamp, 1000.123456, 1e-6);
  ASSERT_NEAR(loaded[0].pose.translation().x(), 1.0, EPSD);
  ASSERT_NEAR(loaded[1].pose.translation().z(), 6.0, EPSD);
}

void test_trajectory_kitti_roundtrip() {
  io::Trajectory traj;

  io::StampedPose sp;
  sp.timestamp = 0.0;
  sp.pose = Eigen::Isometry3d::Identity();
  sp.pose.translation() = Eigen::Vector3d(10.0, 20.0, 30.0);
  sp.pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));
  traj.push_back(sp);

  std::stringstream ss;
  io::saveTrajectoryKITTI(ss, traj);

  ss.seekg(0);
  io::Trajectory loaded = io::loadTrajectoryKITTI(ss);

  ASSERT_EQ(loaded.size(), 1u);
  ASSERT_NEAR(loaded[0].pose.translation().x(), 10.0, 1e-6);
  ASSERT_NEAR(loaded[0].pose.translation().y(), 20.0, 1e-6);
  ASSERT_NEAR(loaded[0].pose.translation().z(), 30.0, 1e-6);
}

void test_trajectory_tum_skip_comments() {
  std::string content = R"(
# This is a comment
1000.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0

# Another comment
1001.0 1.0 0.0 0.0 0.0 0.0 0.0 1.0
)";

  std::stringstream ss(content);
  io::Trajectory loaded = io::loadTrajectoryTUM(ss);

  ASSERT_EQ(loaded.size(), 2u);
  ASSERT_NEAR(loaded[0].timestamp, 1000.0, EPSD);
  ASSERT_NEAR(loaded[1].pose.translation().x(), 1.0, EPSD);
}

void test_trajectory_interpolation() {
  io::Trajectory traj;

  // Pose at t=0: origin
  traj.push_back(0.0, Eigen::Isometry3d::Identity());

  // Pose at t=1: translated by (2, 0, 0)
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  traj.push_back(1.0, pose1);

  // Interpolate at t=0.5 -> should be (1, 0, 0)
  auto interp = traj.poseAt(0.5);
  ASSERT_NEAR(interp.translation().x(), 1.0, EPSD);
  ASSERT_NEAR(interp.translation().y(), 0.0, EPSD);

  // Before first -> should return first pose
  auto before = traj.poseAt(-1.0);
  ASSERT_NEAR(before.translation().x(), 0.0, EPSD);

  // After last -> should return last pose
  auto after = traj.poseAt(2.0);
  ASSERT_NEAR(after.translation().x(), 2.0, EPSD);
}

void test_trajectory_transform() {
  io::Trajectory traj;

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  traj.push_back(0.0, pose);

  // Transform: shift by (10, 0, 0)
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);

  traj.transform(T);

  ASSERT_NEAR(traj[0].pose.translation().x(), 11.0, EPSD);
}

void test_trajectory_statistics() {
  io::Trajectory traj;

  // Create a simple path: origin -> (1,0,0) -> (1,1,0)
  traj.push_back(0.0, Eigen::Isometry3d::Identity());

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  traj.push_back(1.0, pose1);

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(1.0, 1.0, 0.0);
  traj.push_back(2.0, pose2);

  // Length: 1 + 1 = 2
  ASSERT_NEAR(traj.length(), 2.0, EPSD);

  // Duration: 2 - 0 = 2
  ASSERT_NEAR(traj.duration(), 2.0, EPSD);

  // Start/end time
  ASSERT_NEAR(traj.startTime(), 0.0, EPSD);
  ASSERT_NEAR(traj.endTime(), 2.0, EPSD);
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "=== test_io ===\n";

  std::cout << "PCD I/O:\n";
  TEST(pcd_ascii_roundtrip);
  TEST(pcd_binary_roundtrip);
  TEST(pcd_viewpoint);
  TEST(pcd_rgb_channel);
  TEST(pcd_empty_cloud);
  TEST(pcd_exception_on_bad_stream);

  std::cout << "BIN I/O:\n";
  TEST(bin_roundtrip);
  TEST(bin_no_intensity_save);

  std::cout << "Trajectory I/O:\n";
  TEST(trajectory_tum_roundtrip);
  TEST(trajectory_kitti_roundtrip);
  TEST(trajectory_tum_skip_comments);
  TEST(trajectory_interpolation);
  TEST(trajectory_transform);
  TEST(trajectory_statistics);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
