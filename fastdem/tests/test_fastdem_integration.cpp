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

// ─── Sensor Model Variants ─────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, LiDARSensorModelPipeline) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::LiDAR);

  auto cloud = makeGroundCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud, T_base_sensor, T_world_base));

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.2f);
}

TEST_F(FastDEMIntegrationTest, RGBDSensorModelPipeline) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::RGBD);

  auto cloud = makeGroundCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud, T_base_sensor, T_world_base));

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.2f);
}

// ─── Estimator Variants ────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, P2QuantileEstimator) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant)
      .setEstimatorType(EstimationType::P2Quantile);

  // P2 needs ≥5 samples per cell to initialize
  for (int i = 0; i < 6; ++i) {
    auto cloud = makeGroundCloud(1.0f + i * 0.01f);
    mapper.integrate(cloud, T_base_sensor, T_world_base);
  }

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.2f);
}

// ─── Mapping Mode ──────────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, GlobalModeFixedOrigin) {
  FastDEM mapper(map);
  mapper.setMappingMode(MappingMode::GLOBAL)
      .setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  auto cloud = makeGroundCloud(1.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  // Move robot — map should NOT follow
  T_world_base.translation().x() = 3.0;
  auto cloud2 = makeGroundCloud(2.0f);
  mapper.integrate(cloud2, T_base_sensor, T_world_base);

  // Original position should still have data
  grid_map::Position origin(0.0, 0.0);
  EXPECT_TRUE(map.hasElevationAt(origin));
}

TEST_F(FastDEMIntegrationTest, LocalModeFollowsRobot) {
  FastDEM mapper(map);
  mapper.setMappingMode(MappingMode::LOCAL)
      .setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  auto cloud = makeGroundCloud(1.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  // Move robot far away — map should follow, old data discarded
  T_world_base.translation().x() = 100.0;
  auto cloud2 = makeGroundCloud(2.0f);
  mapper.integrate(cloud2, T_base_sensor, T_world_base);

  // Original origin should be outside the map now
  grid_map::Position origin(0.0, 0.0);
  EXPECT_FALSE(map.isInside(origin));
}

// ─── Config-based Construction ─────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, ConstructFromConfig) {
  Config cfg;
  cfg.mapping.estimation_type = EstimationType::Kalman;
  cfg.sensor_model.type = SensorType::Constant;
  cfg.point_filter.z_min = -2.0f;
  cfg.point_filter.z_max = 5.0f;
  cfg.raycasting.enabled = false;

  FastDEM mapper(map, cfg);

  auto cloud = makeGroundCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud, T_base_sensor, T_world_base));

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.1f);
}

TEST_F(FastDEMIntegrationTest, ConfigPointFilterApplied) {
  Config cfg;
  cfg.point_filter.z_min = 0.0f;
  cfg.point_filter.z_max = 2.0f;

  FastDEM mapper(map, cfg);

  // Points at z=5 — outside filter range
  auto cloud = makeGroundCloud(5.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  EXPECT_TRUE(map.isEmpty());
}

// ─── Transform Handling ────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, SensorOffsetApplied) {
  // Sensor mounted 1m above base
  T_base_sensor.translation().z() = 1.0;

  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  // Points at z=0 in sensor frame → z=1 in base frame → z=1 in world
  auto cloud = makeGroundCloud(0.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  grid_map::Position center(0.0, 0.0);
  ASSERT_TRUE(map.hasElevationAt(center));
  EXPECT_NEAR(map.elevationAt(center), 1.0f, 0.2f);
}

TEST_F(FastDEMIntegrationTest, RotatedTransform) {
  // Robot rotated 90° around Z — points should be remapped
  T_world_base.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  auto cloud = makeGroundCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud, T_base_sensor, T_world_base));

  // Should still produce elevation data
  EXPECT_FALSE(map.isEmpty());
}

// ─── Range Filter ──────────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, RangeFilterRejectsClosePoints) {
  FastDEM mapper(map);
  mapper.setRangeFilter(5.0f, 20.0f);  // Reject points closer than 5m

  // Points at ~0.3m spacing near origin — all within 1.5m of sensor
  auto cloud = makeGroundCloud(1.0f, /*grid_half=*/2, /*spacing=*/0.3f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  EXPECT_TRUE(map.isEmpty());
}

TEST_F(FastDEMIntegrationTest, CombinedHeightAndRangeFilter) {
  FastDEM mapper(map);
  mapper.setHeightFilter(0.0f, 3.0f)
      .setRangeFilter(0.0f, 20.0f)
      .setSensorModel(SensorType::Constant);

  // z=1.0 within height filter, range within limit → accepted
  auto cloud_in = makeGroundCloud(1.0f);
  mapper.integrate(cloud_in, T_base_sensor, T_world_base);
  EXPECT_FALSE(map.isEmpty());

  // Reset
  map.clearAll();

  // z=5.0 outside height filter → rejected
  auto cloud_out = makeGroundCloud(5.0f);
  mapper.integrate(cloud_out, T_base_sensor, T_world_base);
  EXPECT_TRUE(map.isEmpty());
}

// ─── Callbacks ─────────────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, PreprocessedCallbackFired) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  bool called = false;
  size_t received_size = 0;
  mapper.onPreprocessed([&](const PointCloud& cloud) {
    called = true;
    received_size = cloud.size();
  });

  auto cloud = makeGroundCloud(1.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  EXPECT_TRUE(called);
  EXPECT_GT(received_size, 0u);
}

TEST_F(FastDEMIntegrationTest, RasterizedCallbackFired) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  bool called = false;
  mapper.onRasterized([&](const PointCloud& /*cloud*/) {
    called = true;
  });

  auto cloud = makeGroundCloud(1.0f);
  mapper.integrate(cloud, T_base_sensor, T_world_base);

  EXPECT_TRUE(called);
}

// ─── Return Value ──────────────────────────────────────────────────────────

TEST_F(FastDEMIntegrationTest, IntegrateReturnsTrueOnSuccess) {
  FastDEM mapper(map);
  mapper.setHeightFilter(-5.0f, 15.0f)
      .setSensorModel(SensorType::Constant);

  auto cloud = makeGroundCloud(1.0f);
  EXPECT_TRUE(mapper.integrate(cloud, T_base_sensor, T_world_base));
}

TEST_F(FastDEMIntegrationTest, IntegrateReturnsFalseOnEmpty) {
  FastDEM mapper(map);
  PointCloud empty;
  EXPECT_FALSE(mapper.integrate(empty, T_base_sensor, T_world_base));
}

TEST_F(FastDEMIntegrationTest, IntegrateReturnsFalseWhenAllFiltered) {
  FastDEM mapper(map);
  mapper.setHeightFilter(100.0f, 200.0f);  // Accept only z in [100, 200]

  auto cloud = makeGroundCloud(1.0f);  // z=1 — filtered out
  EXPECT_FALSE(mapper.integrate(cloud, T_base_sensor, T_world_base));
}
