// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_postprocess.cpp
 *
 * Unit tests for post-processing functions:
 * inpainting, raycasting, uncertainty_fusion, spatial_smoothing.
 */

#include <gtest/gtest.h>

#include "fastdem/postprocess/inpainting.hpp"
#include "fastdem/postprocess/raycasting.hpp"
#include "fastdem/postprocess/spatial_smoothing.hpp"
#include "fastdem/postprocess/uncertainty_fusion.hpp"

using namespace fastdem;

// ─── Fixture ────────────────────────────────────────────────────────────────

class PostprocessTest : public ::testing::Test {
 protected:
  ElevationMap map;

  void SetUp() override {
    // 10m x 10m, 0.5m resolution → 20x20 grid
    map.setGeometry(10.0f, 10.0f, 0.5f);
  }

  grid_map::Index centerIndex() const {
    grid_map::Index idx;
    map.getIndex(grid_map::Position(0.0, 0.0), idx);
    return idx;
  }
};

// ─── Inpainting ─────────────────────────────────────────────────────────────

TEST_F(PostprocessTest, InpaintingFillsSimpleHole) {
  // Set 8 neighbors around center to 1.0, leave center as NaN
  auto center = centerIndex();
  constexpr int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  constexpr int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  for (int i = 0; i < 8; ++i) {
    grid_map::Index neighbor(center(0) + dy[i], center(1) + dx[i]);
    map.at(layer::elevation, neighbor) = 1.0f;
  }
  ASSERT_TRUE(std::isnan(map.at(layer::elevation, center)));

  applyInpainting(map, 3, 2);

  ASSERT_TRUE(map.exists(layer::elevation_inpainted));
  float inpainted = map.at(layer::elevation_inpainted, center);
  EXPECT_TRUE(std::isfinite(inpainted));
  EXPECT_NEAR(inpainted, 1.0f, 0.01f);
}

TEST_F(PostprocessTest, InpaintingPreservesExistingValues) {
  // Fill all cells with 2.0
  map.get(layer::elevation).setConstant(2.0f);

  applyInpainting(map, 3, 2);

  ASSERT_TRUE(map.exists(layer::elevation_inpainted));
  auto center = centerIndex();
  EXPECT_FLOAT_EQ(map.at(layer::elevation_inpainted, center), 2.0f);
}

// ─── Raycasting ─────────────────────────────────────────────────────────────

TEST_F(PostprocessTest, RaycastingCreatesLayers) {
  // Add required layers
  map.add(layer::elevation_min, NAN);
  map.add(layer::elevation_max, NAN);

  // Populate a few cells with elevation data
  auto center = centerIndex();
  map.at(layer::elevation, center) = 1.0f;
  map.at(layer::elevation_min, center) = 0.5f;
  map.at(layer::elevation_max, center) = 1.5f;

  // Create a cloud with one downward point
  PointCloud cloud;
  cloud.add(1.0f, 0.0f, 0.5f);  // Below sensor

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = true;
  cfg.vote_threshold = 10;
  applyRaycasting(map, cloud, sensor_origin, cfg);

  EXPECT_TRUE(map.exists(layer::raycasting_upper_bound));
  EXPECT_TRUE(map.exists(layer::conflict_count));
}

TEST_F(PostprocessTest, RaycastingClearsGhostCell) {
  // Setup: ghost cell at position (2.0, 0.0) with high elevation
  map.add(layer::elevation_min, NAN);
  map.add(layer::elevation_max, NAN);

  grid_map::Index ghost_idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(2.0, 0.0), ghost_idx));

  // Ghost cell: high elevation with large variation (passes dynamic_height_threshold)
  map.at(layer::elevation, ghost_idx) = 10.0f;
  map.at(layer::elevation_min, ghost_idx) = 0.0f;
  map.at(layer::elevation_max, ghost_idx) = 10.0f;

  // Cloud target beyond the ghost cell — ray from sensor (0,0,5) to (4,0,0)
  // passes through the ghost cell at (2,0)
  PointCloud cloud;
  cloud.add(4.0f, 0.0f, 0.0f);

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = true;
  cfg.endpoint_margin = 0;
  cfg.ray_height_margin = 0.05f;
  cfg.dynamic_height_threshold = 0.5f;
  cfg.vote_threshold = 1;  // Clear after 1 conflict

  applyRaycasting(map, cloud, sensor_origin, cfg);

  // Ghost cell should have been cleared (elevation = NaN)
  EXPECT_TRUE(std::isnan(map.at(layer::elevation, ghost_idx)));
}

TEST_F(PostprocessTest, RaycastingDisabledIsNoOp) {
  PointCloud cloud;
  cloud.add(1.0f, 0.0f, 0.5f);
  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = false;
  applyRaycasting(map, cloud, sensor_origin, cfg);

  EXPECT_FALSE(map.exists(layer::raycasting_upper_bound));
  EXPECT_FALSE(map.exists(layer::conflict_count));
}

// ─── Uncertainty Fusion ─────────────────────────────────────────────────────

TEST_F(PostprocessTest, UncertaintyFusionComputesBounds) {
  // Add required layers and populate a block of cells
  map.add(layer::variance, NAN);

  auto center = centerIndex();
  // Fill a 3x3 block with valid elevation and variance
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      grid_map::Index idx(center(0) + dr, center(1) + dc);
      map.at(layer::elevation, idx) = 1.0f + 0.1f * dr;
      map.at(layer::variance, idx) = 0.01f;
    }
  }

  config::UncertaintyFusion cfg;
  cfg.enabled = true;
  cfg.search_radius = 0.6f;  // Slightly more than 1 cell (0.5m resolution)
  cfg.spatial_sigma = 0.3f;
  cfg.quantile_lower = 0.01f;
  cfg.quantile_upper = 0.99f;
  cfg.min_valid_neighbors = 1;
  applyUncertaintyFusion(map, cfg);

  ASSERT_TRUE(map.exists(layer::upper_bound));
  ASSERT_TRUE(map.exists(layer::lower_bound));
  ASSERT_TRUE(map.exists(layer::uncertainty_range));

  float upper = map.at(layer::upper_bound, center);
  float lower = map.at(layer::lower_bound, center);
  float range = map.at(layer::uncertainty_range, center);

  EXPECT_TRUE(std::isfinite(upper));
  EXPECT_TRUE(std::isfinite(lower));
  EXPECT_TRUE(std::isfinite(range));
  EXPECT_GT(upper, lower);
  EXPECT_NEAR(range, upper - lower, 1e-6f);
}

TEST_F(PostprocessTest, UncertaintyFusionSkipsMissingVariance) {
  // elevation exists (from SetUp) but no variance layer — should return
  config::UncertaintyFusion cfg;
  cfg.enabled = true;

  // Remove elevation data and ensure variance doesn't exist
  ElevationMap empty_map;
  empty_map.setGeometry(10.0f, 10.0f, 0.5f);
  applyUncertaintyFusion(empty_map, cfg);

  EXPECT_FALSE(empty_map.exists(layer::upper_bound));
}

TEST_F(PostprocessTest, UncertaintyFusionDisabledIsNoOp) {
  config::UncertaintyFusion cfg;
  cfg.enabled = false;
  applyUncertaintyFusion(map, cfg);

  EXPECT_FALSE(map.exists(layer::upper_bound));
}

// ─── Spatial Smoothing ──────────────────────────────────────────────────────

TEST_F(PostprocessTest, SpatialSmoothingRemovesSpike) {
  // Fill a 5x5 block with 1.0, set center to 100.0 (spike)
  auto center = centerIndex();
  for (int dr = -2; dr <= 2; ++dr) {
    for (int dc = -2; dc <= 2; ++dc) {
      grid_map::Index idx(center(0) + dr, center(1) + dc);
      map.at(layer::elevation, idx) = 1.0f;
    }
  }
  map.at(layer::elevation, center) = 100.0f;

  applySpatialSmoothing(map, layer::elevation, /*kernel_size=*/3,
                         /*min_valid_neighbors=*/5);

  // Center should now be median of neighbors (1.0), not 100.0
  float smoothed = map.at(layer::elevation, center);
  EXPECT_NEAR(smoothed, 1.0f, 0.01f);
}

TEST_F(PostprocessTest, SpatialSmoothingSkipsMissingLayer) {
  // Should not crash on non-existent layer
  applySpatialSmoothing(map, "nonexistent_layer");
}
