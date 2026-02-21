// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_fastdem_integration.cpp
 *
 * End-to-end integration tests for the FastDEM class (explicit transforms).
 */

#include <gtest/gtest.h>

#include "fastdem/fastdem.hpp"
#include "fastdem/mapping/kalman_estimation.hpp"
#include "fastdem/postprocess/raycasting.hpp"

using namespace fastdem;

// ─── Fixture ────────────────────────────────────────────────────────────────

class FastDEMIntegrationTest : public ::testing::Test {
 protected:
  ElevationMap map;
  Eigen::Isometry3d T_base_sensor = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();

  void SetUp() override {
    // 10m x 10m, 0.5m resolution → 20x20 grid
    map.setGeometry(10.0f, 10.0f, 0.5f);
  }

  /// Create a flat grid of points at the given height.
  PointCloud makeGroundCloud(float height, int grid_half = 3,
                             float spacing = 0.3f) {
    PointCloud cloud;
    for (int i = -grid_half; i <= grid_half; ++i) {
      for (int j = -grid_half; j <= grid_half; ++j) {
        cloud.add(i * spacing, j * spacing, height);
      }
    }
    return cloud;
  }
};

// ─── Tests ──────────────────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, IntegrateUpdatesElevation) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-2.0f, 5.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setEstimatorType(EstimationType::Kalman);

  auto cloud = makeGroundCloud(1.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  // Center cell (0,0) should have elevation close to 1.0
  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.1f);
}

TEST_F(FastDEMIntegrationTest, EmptyCloudIsNoOp) {
  FastDEM mapper(map);

  PointCloud empty_cloud;
  mapper.integrate(empty_cloud, T_base_sensor, T_world_base);

  EXPECT_TRUE(map.isEmpty());
}

TEST_F(FastDEMIntegrationTest, HeightFilterRejectsOutOfRange) {
  FastDEM mapper(map);
  mapper.setHeightFilter(0.0f, 2.0f);  // Accept z in [0, 2]

  // All points at z=10.0 — above the filter range
  auto cloud = makeGroundCloud(10.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  EXPECT_TRUE(map.isEmpty());
}

TEST_F(FastDEMIntegrationTest, MultipleIntegrations) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setEstimatorType(EstimationType::Kalman);

  // First integration at z=1.0
  auto cloud1 = makeGroundCloud(1.0f);
  mapper.integrate(cloud1, T_base_sensor, T_world_base);

  // Second integration at z=1.5
  auto cloud2 = makeGroundCloud(1.5f);
  mapper.integrate(cloud2, T_base_sensor, T_world_base);

  // Kalman filter should blend: elevation between 1.0 and 1.5
  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  float elev = map.elevationAt(center);
  EXPECT_GT(elev, 0.9f);
  EXPECT_LT(elev, 1.6f);
}

TEST_F(FastDEMIntegrationTest, RaycastingRunsWithoutCrash) {
  // Raise the sensor so raycasting has downward rays
  T_world_base.translation().z() = 3.0;

  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setEstimatorType(EstimationType::Kalman)
      .enableRaycasting(true);

  auto cloud = makeGroundCloud(0.5f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  // Verify no crash and expected output layers exist
  EXPECT_TRUE(map.exists(layer::elevation));
  EXPECT_TRUE(map.exists(layer::raycasting));
  EXPECT_TRUE(map.exists(layer::variance));
  // Kalman-internal layers
  EXPECT_TRUE(map.exists(layer::kalman_p));
}
