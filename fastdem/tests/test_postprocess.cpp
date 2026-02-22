// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_postprocess.cpp
 *
 * Unit tests for post-processing functions:
 * inpainting, raycasting, uncertainty_fusion, spatial_smoothing.
 */

#include <gtest/gtest.h>

#include "fastdem/postprocess/feature_extraction.hpp"
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
  // Populate a cell with elevation data
  auto center = centerIndex();
  map.at(layer::elevation, center) = 1.0f;

  PointCloud cloud;
  cloud.add(1.0f, 0.0f, 0.5f);  // Below sensor

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = true;
  applyRaycasting(map, cloud, sensor_origin, cfg);

  EXPECT_TRUE(map.exists(layer::ghost_removal));
  EXPECT_TRUE(map.exists(layer::raycasting));
  EXPECT_TRUE(map.exists(layer::visibility_logodds));
}

TEST_F(PostprocessTest, RaycastingClearsGhostCell) {
  // Ghost cell at (2,0) with high elevation — no measurement reaches it
  grid_map::Index ghost_idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(2.0, 0.0), ghost_idx));
  map.at(layer::elevation, ghost_idx) = 10.0f;

  // Ray from sensor (0,0,5) to target (4,0,0) passes through ghost cell
  PointCloud cloud;
  cloud.add(4.0f, 0.0f, 0.0f);

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = true;

  cfg.height_conflict_threshold = 0.05f;
  cfg.log_odds_ghost = 0.5f;
  cfg.clear_threshold = -0.4f;  // Clear after 1 conflict (0 - 0.5 = -0.5 < -0.4)

  applyRaycasting(map, cloud, sensor_origin, cfg);

  EXPECT_TRUE(std::isnan(map.at(layer::elevation, ghost_idx)));
  EXPECT_FLOAT_EQ(map.at(layer::ghost_removal, ghost_idx), 1.0f);
}

TEST_F(PostprocessTest, RaycastingObservedCellProtected) {
  // Cell at (2,0) has elevation AND is directly measured this frame
  grid_map::Index cell_idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(2.0, 0.0), cell_idx));
  map.at(layer::elevation, cell_idx) = 2.0f;

  // Target at (4,0,0): ray passes through (2,0) cell
  // Also add a measurement landing directly on (2,0)
  PointCloud cloud;
  cloud.add(4.0f, 0.0f, 0.0f);
  cloud.add(2.0f, 0.0f, 0.3f);  // Direct observation at cell (2,0)

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = true;

  cfg.height_conflict_threshold = 0.05f;
  cfg.log_odds_observed = 0.8f;
  cfg.log_odds_ghost = 0.5f;
  cfg.clear_threshold = -0.4f;

  applyRaycasting(map, cloud, sensor_origin, cfg);

  // Cell was observed (+0.8) then ghost evidence (-0.5) → logodds = +0.3 > -0.4
  // → NOT cleared
  EXPECT_FALSE(std::isnan(map.at(layer::elevation, cell_idx)));
}

TEST_F(PostprocessTest, RaycastingGhostRequiresAccumulation) {
  // Ghost cell — logodds should decrease gradually over multiple frames
  grid_map::Index ghost_idx;
  ASSERT_TRUE(map.getIndex(grid_map::Position(2.0, 0.0), ghost_idx));
  map.at(layer::elevation, ghost_idx) = 10.0f;

  PointCloud cloud;
  cloud.add(4.0f, 0.0f, 0.0f);

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = true;

  cfg.height_conflict_threshold = 0.05f;
  cfg.log_odds_ghost = 0.2f;
  cfg.clear_threshold = -0.9f;  // Needs 5+ conflicts to clear

  // Frame 1-4: logodds decreases but stays above threshold
  for (int i = 0; i < 4; ++i) {
    map.at(layer::elevation, ghost_idx) = 10.0f;  // Restore if cleared
    applyRaycasting(map, cloud, sensor_origin, cfg);
  }
  // logodds = 0 - 4*0.2 = -0.8 > -0.9 → not yet cleared
  EXPECT_FALSE(std::isnan(map.at(layer::elevation, ghost_idx)));

  // Frame 5: logodds = -0.8 - 0.2 = -1.0 < -0.9 → cleared
  applyRaycasting(map, cloud, sensor_origin, cfg);
  EXPECT_TRUE(std::isnan(map.at(layer::elevation, ghost_idx)));
}

TEST_F(PostprocessTest, RaycastingDisabledIsNoOp) {
  PointCloud cloud;
  cloud.add(1.0f, 0.0f, 0.5f);
  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f);

  config::Raycasting cfg;
  cfg.enabled = false;
  applyRaycasting(map, cloud, sensor_origin, cfg);

  EXPECT_FALSE(map.exists(layer::ghost_removal));
  EXPECT_FALSE(map.exists(layer::raycasting));
  EXPECT_FALSE(map.exists(layer::visibility_logodds));
}

// ─── Uncertainty Fusion ─────────────────────────────────────────────────────

TEST_F(PostprocessTest, UncertaintyFusionComputesBounds) {
  // Add required layers (estimator-computed bounds) and populate a block
  map.add(layer::upper_bound, NAN);
  map.add(layer::lower_bound, NAN);

  auto center = centerIndex();
  // Fill a 3x3 block with valid bounds
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      grid_map::Index idx(center(0) + dr, center(1) + dc);
      float h = 1.0f + 0.1f * dr;
      map.at(layer::elevation, idx) = h;
      map.at(layer::upper_bound, idx) = h + 0.2f;
      map.at(layer::lower_bound, idx) = h - 0.2f;
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

  float upper = map.at(layer::upper_bound, center);
  float lower = map.at(layer::lower_bound, center);

  EXPECT_TRUE(std::isfinite(upper));
  EXPECT_TRUE(std::isfinite(lower));
  EXPECT_GT(upper, lower);
}

TEST_F(PostprocessTest, UncertaintyFusionSkipsMissingBounds) {
  // No upper_bound/lower_bound layers — should return without crashing
  config::UncertaintyFusion cfg;
  cfg.enabled = true;

  ElevationMap empty_map;
  empty_map.setGeometry(10.0f, 10.0f, 0.5f);
  applyUncertaintyFusion(empty_map, cfg);

  // Should not crash — just return early
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

// ─── Feature Extraction ────────────────────────────────────────────────────

TEST_F(PostprocessTest, FeatureExtractionCreatesAllLayers) {
  // Fill a flat plane at z = 1.0
  map.get(layer::elevation).setConstant(1.0f);

  applyFeatureExtraction(map, /*analysis_radius=*/0.6f,
                         /*min_valid_neighbors=*/4);

  EXPECT_TRUE(map.exists(layer::step));
  EXPECT_TRUE(map.exists(layer::slope));
  EXPECT_TRUE(map.exists(layer::roughness));
  EXPECT_TRUE(map.exists(layer::curvature));
  EXPECT_TRUE(map.exists(layer::normal_x));
  EXPECT_TRUE(map.exists(layer::normal_y));
  EXPECT_TRUE(map.exists(layer::normal_z));
}

TEST_F(PostprocessTest, FeatureExtractionFlatPlane) {
  // Flat plane: slope ≈ 0, roughness ≈ 0, normal ≈ (0, 0, 1)
  map.get(layer::elevation).setConstant(1.0f);

  applyFeatureExtraction(map, 0.6f, 4);

  auto center = centerIndex();
  EXPECT_NEAR(map.at(layer::slope, center), 0.0f, 1.0f);      // < 1 degree
  EXPECT_NEAR(map.at(layer::roughness, center), 0.0f, 0.001f);
  EXPECT_NEAR(map.at(layer::step, center), 0.0f, 0.001f);
  EXPECT_NEAR(map.at(layer::normal_z, center), 1.0f, 0.01f);
}

TEST_F(PostprocessTest, FeatureExtractionTiltedPlane) {
  // Tilted plane: z increases with row → slope > 0
  const auto idx = map.indexer();
  for (int row = 0; row < idx.rows; ++row) {
    for (int col = 0; col < idx.cols; ++col) {
      auto [r, c] = idx(row, col);
      map.at(layer::elevation, grid_map::Index(r, c)) =
          static_cast<float>(row) * idx.resolution * 0.5f;  // 0.5 rise/run
    }
  }

  applyFeatureExtraction(map, 0.6f, 4);

  auto center = centerIndex();
  float slope = map.at(layer::slope, center);
  EXPECT_GT(slope, 10.0f);   // Clearly non-zero slope
  EXPECT_LT(slope, 45.0f);   // atan(0.5) ≈ 26.6°
}

TEST_F(PostprocessTest, FeatureExtractionStepDetection) {
  // Half the map at z=0, half at z=1 → step should be ~1.0 near boundary
  const auto idx = map.indexer();
  for (int row = 0; row < idx.rows; ++row) {
    for (int col = 0; col < idx.cols; ++col) {
      auto [r, c] = idx(row, col);
      map.at(layer::elevation, grid_map::Index(r, c)) =
          (col < idx.cols / 2) ? 0.0f : 1.0f;
    }
  }

  applyFeatureExtraction(map, 0.6f, 4);

  // Check a cell near the boundary (center column)
  auto center = centerIndex();
  float step_val = map.at(layer::step, center);
  EXPECT_GT(step_val, 0.5f);  // Should detect the step
}

TEST_F(PostprocessTest, FeatureExtractionHandlesUninitializedMap) {
  // Default-constructed map — should not crash
  ElevationMap empty_map;
  applyFeatureExtraction(empty_map);
}

TEST_F(PostprocessTest, FeatureExtractionSkipsNaNCells) {
  // All NaN — no features computed, but layers created
  applyFeatureExtraction(map, 0.6f, 4);

  EXPECT_TRUE(map.exists(layer::slope));
  auto center = centerIndex();
  EXPECT_FALSE(std::isfinite(map.at(layer::slope, center)));
}

TEST_F(PostprocessTest, FeatureExtractionInsufficientNeighbors) {
  // Single isolated cell — not enough neighbors
  auto center = centerIndex();
  map.at(layer::elevation, center) = 1.0f;

  applyFeatureExtraction(map, 0.6f, /*min_valid_neighbors=*/4);

  // Should remain NaN due to insufficient neighbors
  EXPECT_FALSE(std::isfinite(map.at(layer::slope, center)));
}

TEST_F(PostprocessTest, FeatureExtractionNormalPointsUp) {
  // For any surface, normal_z should be positive (flipped upward)
  const auto idx = map.indexer();
  for (int row = 0; row < idx.rows; ++row) {
    for (int col = 0; col < idx.cols; ++col) {
      auto [r, c] = idx(row, col);
      map.at(layer::elevation, grid_map::Index(r, c)) =
          static_cast<float>(row) * idx.resolution;
    }
  }

  applyFeatureExtraction(map, 0.6f, 4);

  auto center = centerIndex();
  if (std::isfinite(map.at(layer::normal_z, center))) {
    EXPECT_GT(map.at(layer::normal_z, center), 0.0f);
  }
}

TEST_F(PostprocessTest, FeatureExtractionCurvatureBounded) {
  // Curvature = |λ₀| / trace(cov), should be in [0, 1]
  map.get(layer::elevation).setConstant(1.0f);
  auto center = centerIndex();
  map.at(layer::elevation, center) = 2.0f;  // bump

  applyFeatureExtraction(map, 0.6f, 4);

  if (std::isfinite(map.at(layer::curvature, center))) {
    EXPECT_GE(map.at(layer::curvature, center), 0.0f);
    EXPECT_LE(map.at(layer::curvature, center), 1.0f);
  }
}
