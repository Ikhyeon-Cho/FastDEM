// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_online_mode.cpp
 *
 * Integration tests for FastDEM online mode with mock transform systems.
 */

#include <gtest/gtest.h>

#include "fastdem/fastdem.hpp"

using namespace fastdem;

// ─── Mock Transform Systems ─────────────────────────────────────────────────

/// Mock calibration: returns a fixed extrinsic for known sensor frames.
class MockCalibration : public Calibration {
 public:
  explicit MockCalibration(
      Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity(),
      std::string base_frame = "base_link")
      : extrinsic_(extrinsic), base_frame_(std::move(base_frame)) {}

  std::optional<Eigen::Isometry3d> getExtrinsic(
      const std::string& sensor_frame) const override {
    if (sensor_frame == "unknown_sensor") return std::nullopt;
    return extrinsic_;
  }

  std::string getBaseFrame() const override { return base_frame_; }

 private:
  Eigen::Isometry3d extrinsic_;
  std::string base_frame_;
};

/// Mock odometry: returns a fixed or configurable pose for any timestamp.
class MockOdometry : public Odometry {
 public:
  explicit MockOdometry(
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(),
      std::string world_frame = "map")
      : pose_(pose), world_frame_(std::move(world_frame)) {}

  std::optional<Eigen::Isometry3d> getPoseAt(
      uint64_t /*timestamp_ns*/) const override {
    if (fail_) return std::nullopt;
    return pose_;
  }

  std::string getWorldFrame() const override { return world_frame_; }

  void setPose(const Eigen::Isometry3d& pose) { pose_ = pose; }
  void setFail(bool fail) { fail_ = fail; }

 private:
  Eigen::Isometry3d pose_;
  std::string world_frame_;
  bool fail_ = false;
};

// ─── Fixture ─────────────────────────────────────────────────────────────────

class OnlineModeTest : public ::testing::Test {
 protected:
  ElevationMap map;
  std::shared_ptr<MockCalibration> calibration;
  std::shared_ptr<MockOdometry> odometry;

  void SetUp() override {
    map.setGeometry(10.0f, 10.0f, 0.5f);
    calibration = std::make_shared<MockCalibration>();
    odometry = std::make_shared<MockOdometry>();
  }

  /// Create a shared_ptr cloud with frame_id and timestamp set.
  std::shared_ptr<PointCloud> makeCloud(float height = 1.0f,
                                        const std::string& frame_id = "lidar",
                                        int grid_half = 3,
                                        float spacing = 0.3f) {
    auto cloud = std::make_shared<PointCloud>();
    for (int i = -grid_half; i <= grid_half; ++i) {
      for (int j = -grid_half; j <= grid_half; ++j) {
        cloud->add(i * spacing, j * spacing, height);
      }
    }
    cloud->setFrameId(frame_id);
    cloud->setTimestamp(1000000000ULL);  // 1 second in ns
    return cloud;
  }
};

// ─── Tests ───────────────────────────────────────────────────────────────────

TEST_F(OnlineModeTest, IntegrateWithTransformSystems) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-2.0f, 5.0f)
      .setDistanceFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setCalibrationSystem(calibration)
      .setOdometrySystem(odometry);

  ASSERT_TRUE(mapper.hasTransformSystems());

  auto cloud = makeCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud));

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.1f);
}

TEST_F(OnlineModeTest, SetTransformSystemTemplate) {
  // MockCalibration + MockOdometry in a single object requires dual interface.
  // Test setCalibration + setOdometry separately, verify both set.
  FastDEM mapper(map);
  EXPECT_FALSE(mapper.hasTransformSystems());

  mapper.setCalibrationSystem(calibration);
  EXPECT_FALSE(mapper.hasTransformSystems());

  mapper.setOdometrySystem(odometry);
  EXPECT_TRUE(mapper.hasTransformSystems());
}

TEST_F(OnlineModeTest, IntegrateWithoutTransformsFails) {
  FastDEM mapper(map);

  auto cloud = makeCloud(1.0f);
  EXPECT_FALSE(mapper.integrate(cloud));
}

TEST_F(OnlineModeTest, NullCloudFails) {
  FastDEM mapper(map);
  mapper.setCalibrationSystem(calibration).setOdometrySystem(odometry);

  EXPECT_FALSE(mapper.integrate(nullptr));
}

TEST_F(OnlineModeTest, EmptyCloudFails) {
  FastDEM mapper(map);
  mapper.setCalibrationSystem(calibration).setOdometrySystem(odometry);

  auto empty = std::make_shared<PointCloud>();
  empty->setFrameId("lidar");
  EXPECT_FALSE(mapper.integrate(empty));
}

TEST_F(OnlineModeTest, MissingFrameIdFails) {
  FastDEM mapper(map);
  mapper.setCalibrationSystem(calibration).setOdometrySystem(odometry);

  auto cloud = makeCloud(1.0f, "");  // empty frame_id
  EXPECT_FALSE(mapper.integrate(cloud));
}

TEST_F(OnlineModeTest, UnknownSensorFrameFails) {
  FastDEM mapper(map);
  mapper.setCalibrationSystem(calibration).setOdometrySystem(odometry);

  auto cloud = makeCloud(1.0f, "unknown_sensor");
  EXPECT_FALSE(mapper.integrate(cloud));
}

TEST_F(OnlineModeTest, OdometryUnavailableFails) {
  FastDEM mapper(map);
  odometry->setFail(true);
  mapper.setCalibrationSystem(calibration).setOdometrySystem(odometry);

  auto cloud = makeCloud(1.0f);
  EXPECT_FALSE(mapper.integrate(cloud));
}

TEST_F(OnlineModeTest, MultipleIntegrationsOnline) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setDistanceFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setEstimatorType(EstimationType::Kalman)
      .setCalibrationSystem(calibration)
      .setOdometrySystem(odometry);

  auto cloud1 = makeCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud1));

  auto cloud2 = makeCloud(1.5f);
  EXPECT_TRUE(mapper.integrate(cloud2));

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  float elev = map.elevationAt(center);
  EXPECT_GT(elev, 0.9f);
  EXPECT_LT(elev, 1.6f);
}

TEST_F(OnlineModeTest, WithNonIdentityExtrinsic) {
  // Sensor mounted 1m above base_link
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.translation().z() = 1.0;
  calibration = std::make_shared<MockCalibration>(extrinsic);

  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setDistanceFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setCalibrationSystem(calibration)
      .setOdometrySystem(odometry);

  // Points at z=0 in sensor frame → z=1 in base frame → z=1 in world
  auto cloud = makeCloud(0.0f);
  EXPECT_TRUE(mapper.integrate(cloud));

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.1f);
}

TEST_F(OnlineModeTest, WithRobotPose) {
  // Robot at (2, 0, 0) in world
  Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
  robot_pose.translation().x() = 2.0;
  odometry = std::make_shared<MockOdometry>(robot_pose);

  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setDistanceFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setCalibrationSystem(calibration)
      .setOdometrySystem(odometry);

  auto cloud = makeCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud));

  // Points centered around sensor → shifted to (2, 0) in world
  grid_map::Position robot_pos(2.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(robot_pos));
  EXPECT_NEAR(map.elevationAt(robot_pos), 1.0f, 0.1f);
}

TEST_F(OnlineModeTest, RaycastingOnlineMode) {
  // Raise robot so raycasting has downward rays
  Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
  robot_pose.translation().z() = 3.0;
  odometry = std::make_shared<MockOdometry>(robot_pose);

  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setDistanceFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setEstimatorType(EstimationType::Kalman)
      .enableRaycasting(true)
      .enableUncertaintyFusion(true)
      .setCalibrationSystem(calibration)
      .setOdometrySystem(odometry);

  auto cloud = makeCloud(0.5f);
  EXPECT_TRUE(mapper.integrate(cloud));

  EXPECT_TRUE(map.exists(layer::elevation));
  EXPECT_TRUE(map.exists(layer::raycasting_upper_bound));
}
