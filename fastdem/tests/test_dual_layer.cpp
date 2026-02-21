// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_dual_layer.cpp
 *
 * Tests for dual-layer (ground + obstacle) elevation mapping.
 * Dual-layer is always active in ElevationMapping.
 */

#include <gtest/gtest.h>

#include "fastdem/mapping/elevation_mapping.hpp"

using namespace fastdem;

// ─── Helpers ────────────────────────────────────────────────────────────────

namespace {

/// Build a PointCloud with given (x, y, z) triplets.
PointCloud makeCloud(std::initializer_list<std::array<float, 3>> points) {
  PointCloud cloud;
  for (const auto& p : points) {
    cloud.add(p[0], p[1], p[2]);
  }
  return cloud;
}

}  // namespace

// ─── Fixture ────────────────────────────────────────────────────────────────

class DualLayerTest : public ::testing::Test {
 protected:
  ElevationMap map;
  Eigen::Vector2d robot_pos{0.0, 0.0};

  void SetUp() override {
    // 10m x 10m, 0.5m resolution
    map.setGeometry(10.0f, 10.0f, 0.5f);
  }

  /// Create ElevationMapping with Kalman estimator.
  std::unique_ptr<ElevationMapping> makeKalmanMapping() {
    config::Mapping cfg;
    cfg.mode = MappingMode::GLOBAL;
    cfg.estimation_type = EstimationType::Kalman;
    cfg.kalman.min_variance = 0.0001f;
    cfg.kalman.max_variance = 1.0f;
    cfg.kalman.process_noise = 0.0f;
    return std::make_unique<ElevationMapping>(map, cfg);
  }

  /// Create ElevationMapping with Quantile estimator.
  std::unique_ptr<ElevationMapping> makeQuantileMapping() {
    config::Mapping cfg;
    cfg.mode = MappingMode::GLOBAL;
    cfg.estimation_type = EstimationType::P2Quantile;
    return std::make_unique<ElevationMapping>(map, cfg);
  }
};

// ─── Tests ──────────────────────────────────────────────────────────────────

TEST_F(DualLayerTest, GroundObstacleSeparation) {
  auto mapping = makeKalmanMapping();

  // Same cell (0,0): floor at z=0, wall at z=3
  auto cloud = makeCloud({{0, 0, 0.0f}, {0, 0, 3.0f}});
  mapping->update(cloud, robot_pos);

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  // Ground should be near 0
  float h_g = map.at(layer::elevation, idx);
  EXPECT_NEAR(h_g, 0.0f, 0.1f);

  // Obstacle should be near 3
  float h_o = map.at(layer::obstacle, idx);
  EXPECT_NEAR(h_o, 3.0f, 0.1f);
}

TEST_F(DualLayerTest, FlatSurfaceNoObstacle) {
  auto mapping = makeKalmanMapping();

  // Same cell: two points at nearly the same height (flat ground)
  auto cloud = makeCloud({{0, 0, 1.0f}, {0, 0, 1.02f}});
  mapping->update(cloud, robot_pos);

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  // Ground should be near 1.0
  EXPECT_NEAR(map.at(layer::elevation, idx), 1.0f, 0.1f);

  // Obstacle should remain NaN (max_z > min_z but difference is tiny noise)
  // The obstacle layer may get a value very close to ground — that's acceptable.
  // What matters is no false large obstacle.
  float h_o = map.at(layer::obstacle, idx);
  if (!std::isnan(h_o)) {
    EXPECT_NEAR(h_o, 1.02f, 0.1f);  // Obstacle = ground level (harmless)
  }
}

TEST_F(DualLayerTest, SinglePointOnlyGround) {
  auto mapping = makeKalmanMapping();

  // Single point per cell → min == max → only ground, no obstacle
  auto cloud = makeCloud({{0, 0, 2.0f}});
  mapping->update(cloud, robot_pos);

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  EXPECT_NEAR(map.at(layer::elevation, idx), 2.0f, 0.1f);
  EXPECT_TRUE(std::isnan(map.at(layer::obstacle, idx)));
}

TEST_F(DualLayerTest, KalmanWithDualLayer) {
  auto mapping = makeKalmanMapping();

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  // Frame 1: floor=0, wall=3
  auto cloud1 = makeCloud({{0, 0, 0.0f}, {0, 0, 3.0f}});
  mapping->update(cloud1, robot_pos);

  // Frame 2: floor=0.1, wall=3.1
  auto cloud2 = makeCloud({{0, 0, 0.1f}, {0, 0, 3.1f}});
  mapping->update(cloud2, robot_pos);

  // Ground: Kalman should blend between 0.0 and 0.1
  float h_g = map.at(layer::elevation, idx);
  EXPECT_GT(h_g, -0.05f);
  EXPECT_LT(h_g, 0.15f);

  // Obstacle: per-frame overwrite → last frame's max_z (3.1)
  float h_o = map.at(layer::obstacle, idx);
  EXPECT_FLOAT_EQ(h_o, 3.1f);
}

TEST_F(DualLayerTest, QuantileWithDualLayer) {
  auto mapping = makeQuantileMapping();

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  // Feed multiple frames to build up P2 statistics
  for (int i = 0; i < 10; ++i) {
    float noise = (i % 2 == 0) ? 0.05f : -0.05f;
    auto cloud = makeCloud({{0, 0, 0.0f + noise}, {0, 0, 5.0f + noise}});
    mapping->update(cloud, robot_pos);
  }

  // Ground estimator (Quantile) should track near 0
  float h_g = map.at(layer::elevation, idx);
  EXPECT_NEAR(h_g, 0.0f, 0.5f);

  // Obstacle: per-frame overwrite → last frame's max_z
  float h_o = map.at(layer::obstacle, idx);
  EXPECT_NEAR(h_o, 5.0f, 0.1f);
}

TEST_F(DualLayerTest, ElevationMaxReflectsTrueMax) {
  auto mapping = makeKalmanMapping();

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  // Frame 1: ground=0, wall=3
  auto cloud1 = makeCloud({{0, 0, 0.0f}, {0, 0, 3.0f}});
  mapping->update(cloud1, robot_pos);

  // elevation_max should be 3.0 (true max, not just what ground estimator saw)
  EXPECT_FLOAT_EQ(map.at(layer::elevation_max, idx), 3.0f);

  // Frame 2: ground=0, wall=5 (higher)
  auto cloud2 = makeCloud({{0, 0, 0.0f}, {0, 0, 5.0f}});
  mapping->update(cloud2, robot_pos);

  // elevation_max should update to 5.0
  EXPECT_FLOAT_EQ(map.at(layer::elevation_max, idx), 5.0f);
}

TEST_F(DualLayerTest, ObstacleClearsWhenFlat) {
  auto mapping = makeKalmanMapping();

  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0, 0), idx));

  // Frame 1: wall present → obstacle = 2.0
  auto cloud1 = makeCloud({{0, 0, 0.0f}, {0, 0, 2.0f}});
  mapping->update(cloud1, robot_pos);
  EXPECT_FLOAT_EQ(map.at(layer::obstacle, idx), 2.0f);

  // Frame 2: wall gone (flat ground only) → obstacle = NaN
  auto cloud2 = makeCloud({{0, 0, 0.0f}});
  mapping->update(cloud2, robot_pos);
  EXPECT_TRUE(std::isnan(map.at(layer::obstacle, idx)));
}
