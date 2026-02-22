// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_config.cpp
 *
 * Tests for YAML configuration loading and validation.
 */

#include <gtest/gtest.h>

#include <fstream>

#include "fastdem/config/fastdem.hpp"
#include "fastdem/config/postprocess.hpp"

using namespace fastdem;

// ─── Helpers ─────────────────────────────────────────────────────────────────

namespace {

/// Write a temporary YAML file and return its path.
std::string writeTempYaml(const std::string& content,
                          const std::string& name = "test_config.yaml") {
  std::string path = "/tmp/" + name;
  std::ofstream fs(path);
  fs << content;
  return path;
}

}  // namespace

// ─── Loading Tests ───────────────────────────────────────────────────────────

TEST(ConfigLoadTest, LoadDefaultYaml) {
  // The actual default.yaml should load without errors
  auto cfg = loadConfig(FASTDEM_CONFIG_DIR "/default.yaml");

  EXPECT_EQ(cfg.mapping.estimation_type, EstimationType::Kalman);
  EXPECT_EQ(cfg.sensor_model.type, SensorType::LiDAR);
  EXPECT_TRUE(cfg.raycasting.enabled);
}

TEST(ConfigLoadTest, NonexistentFileThrows) {
  EXPECT_THROW(loadConfig("/nonexistent/path.yaml"), std::runtime_error);
}

TEST(ConfigLoadTest, EmptyYamlUsesDefaults) {
  auto path = writeTempYaml("# empty config\n", "test_empty.yaml");
  auto cfg = loadConfig(path);

  // Should have all defaults
  Config defaults;
  EXPECT_EQ(cfg.mapping.mode, defaults.mapping.mode);
  EXPECT_EQ(cfg.mapping.estimation_type, defaults.mapping.estimation_type);
  EXPECT_EQ(cfg.sensor_model.type, defaults.sensor_model.type);
}

TEST(ConfigLoadTest, PartialYamlPreservesDefaults) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  type: p2_quantile\n",
      "test_partial.yaml");
  auto cfg = loadConfig(path);

  // Specified value loaded
  EXPECT_EQ(cfg.mapping.estimation_type, EstimationType::P2Quantile);

  // Unspecified values remain at defaults
  Config defaults;
  EXPECT_EQ(cfg.mapping.mode, defaults.mapping.mode);
  EXPECT_FLOAT_EQ(cfg.sensor_model.lidar.range_noise, defaults.sensor_model.lidar.range_noise);
}

TEST(ConfigLoadTest, AllEstimationTypes) {
  auto kalman = loadConfig(
      writeTempYaml("mapping:\n  type: kalman_filter\n", "test_kalman.yaml"));
  EXPECT_EQ(kalman.mapping.estimation_type, EstimationType::Kalman);

  auto p2 = loadConfig(
      writeTempYaml("mapping:\n  type: p2_quantile\n", "test_p2.yaml"));
  EXPECT_EQ(p2.mapping.estimation_type, EstimationType::P2Quantile);
}

TEST(ConfigLoadTest, AllSensorTypes) {
  auto lidar = loadConfig(
      writeTempYaml("sensor_model:\n  type: lidar\n", "test_lidar.yaml"));
  EXPECT_EQ(lidar.sensor_model.type, SensorType::LiDAR);

  auto rgbd = loadConfig(
      writeTempYaml("sensor_model:\n  type: rgbd\n", "test_rgbd.yaml"));
  EXPECT_EQ(rgbd.sensor_model.type, SensorType::RGBD);

  auto constant = loadConfig(
      writeTempYaml("sensor_model:\n  type: constant\n", "test_const.yaml"));
  EXPECT_EQ(constant.sensor_model.type, SensorType::Constant);
}

TEST(ConfigLoadTest, MappingModeParsed) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  mode: global\n",
      "test_mode_nested.yaml");
  auto cfg = loadConfig(path);
  EXPECT_EQ(cfg.mapping.mode, MappingMode::GLOBAL);
}

TEST(ConfigLoadTest, MappingModeDefaultIsLocal) {
  auto path = writeTempYaml("# empty\n", "test_mode_default.yaml");
  auto cfg = loadConfig(path);
  EXPECT_EQ(cfg.mapping.mode, MappingMode::LOCAL);
}

TEST(ConfigLoadTest, KalmanParameters) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  type: kalman_filter\n"
      "  kalman:\n"
      "    min_variance: 0.001\n"
      "    max_variance: 0.05\n"
      "    process_noise: 0.001\n",
      "test_kalman_params.yaml");
  auto cfg = loadConfig(path);

  EXPECT_FLOAT_EQ(cfg.mapping.kalman.min_variance, 0.001f);
  EXPECT_FLOAT_EQ(cfg.mapping.kalman.max_variance, 0.05f);
  EXPECT_FLOAT_EQ(cfg.mapping.kalman.process_noise, 0.001f);
}

// ─── Validation: Fatal Errors ────────────────────────────────────────────────

TEST(ConfigLoadTest, PointFilterParsed) {
  auto path = writeTempYaml(
      "point_filter:\n"
      "  z_min: -0.5\n"
      "  z_max: 2.0\n"
      "  range_min: 0.5\n"
      "  range_max: 20.0\n",
      "test_point_filter.yaml");
  auto cfg = loadConfig(path);

  EXPECT_FLOAT_EQ(cfg.point_filter.z_min, -0.5f);
  EXPECT_FLOAT_EQ(cfg.point_filter.z_max, 2.0f);
  EXPECT_FLOAT_EQ(cfg.point_filter.range_min, 0.5f);
  EXPECT_FLOAT_EQ(cfg.point_filter.range_max, 20.0f);
}

TEST(ConfigLoadTest, MissingPointFilterUsesDefaults) {
  auto path = writeTempYaml("# no point_filter\n", "test_no_pf.yaml");
  auto cfg = loadConfig(path);

  Config defaults;
  EXPECT_FLOAT_EQ(cfg.point_filter.z_min, defaults.point_filter.z_min);
  EXPECT_FLOAT_EQ(cfg.point_filter.z_max, defaults.point_filter.z_max);
  EXPECT_FLOAT_EQ(cfg.point_filter.range_min, defaults.point_filter.range_min);
  EXPECT_FLOAT_EQ(cfg.point_filter.range_max, defaults.point_filter.range_max);
}

TEST(ConfigValidationTest, KalmanMinVarGeMaxVarThrows) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  kalman:\n"
      "    min_variance: 0.1\n"
      "    max_variance: 0.001\n",
      "test_kalman_inv.yaml");
  EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST(ConfigValidationTest, P2MarkersNotSortedThrows) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  p2:\n"
      "    dn0: 0.0\n"
      "    dn1: 0.84\n"
      "    dn2: 0.50\n"
      "    dn3: 0.16\n"
      "    dn4: 1.0\n",
      "test_p2_unsorted.yaml");
  EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// ─── Validation: Non-Fatal Clamping ──────────────────────────────────────────

TEST(ConfigValidationTest, NegativeRangeNoiseClamped) {
  auto path = writeTempYaml(
      "sensor_model:\n"
      "  lidar:\n"
      "    range_noise: -0.5\n",
      "test_neg_noise.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GT(cfg.sensor_model.lidar.range_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeAngularNoiseClamped) {
  auto path = writeTempYaml(
      "sensor_model:\n"
      "  lidar:\n"
      "    angular_noise: -1.0\n",
      "test_neg_angular.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.sensor_model.lidar.angular_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeConstantUncertaintyClamped) {
  auto path = writeTempYaml(
      "sensor_model:\n"
      "  constant:\n"
      "    uncertainty: -0.1\n",
      "test_neg_const.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GT(cfg.sensor_model.constant.uncertainty, 0.0f);
}

TEST(ConfigValidationTest, NegativeProcessNoiseClamped) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  kalman:\n"
      "    process_noise: -0.01\n",
      "test_neg_pnoise.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.mapping.kalman.process_noise, 0.0f);
}

// ─── PostProcess Loading Tests ───────────────────────────────────────────────

TEST(PostProcessLoadTest, AllFieldsParsed) {
  auto path = writeTempYaml(
      "inpainting:\n"
      "  enabled: true\n"
      "  max_iterations: 5\n"
      "  min_valid_neighbors: 3\n"
      "uncertainty_fusion:\n"
      "  enabled: true\n"
      "  search_radius: 0.2\n"
      "  spatial_sigma: 0.1\n"
      "  quantile_lower: 0.05\n"
      "  quantile_upper: 0.95\n"
      "  min_valid_neighbors: 4\n"
      "feature_extraction:\n"
      "  enabled: true\n"
      "  analysis_radius: 0.5\n"
      "  min_valid_neighbors: 6\n",
      "test_postprocess_all.yaml");
  auto cfg = config::loadPostProcess(path);

  EXPECT_TRUE(cfg.inpainting.enabled);
  EXPECT_EQ(cfg.inpainting.max_iterations, 5);
  EXPECT_EQ(cfg.inpainting.min_valid_neighbors, 3);

  EXPECT_TRUE(cfg.uncertainty_fusion.enabled);
  EXPECT_FLOAT_EQ(cfg.uncertainty_fusion.search_radius, 0.2f);
  EXPECT_FLOAT_EQ(cfg.uncertainty_fusion.spatial_sigma, 0.1f);
  EXPECT_FLOAT_EQ(cfg.uncertainty_fusion.quantile_lower, 0.05f);
  EXPECT_FLOAT_EQ(cfg.uncertainty_fusion.quantile_upper, 0.95f);
  EXPECT_EQ(cfg.uncertainty_fusion.min_valid_neighbors, 4);

  EXPECT_TRUE(cfg.feature_extraction.enabled);
  EXPECT_FLOAT_EQ(cfg.feature_extraction.analysis_radius, 0.5f);
  EXPECT_EQ(cfg.feature_extraction.min_valid_neighbors, 6);
}

TEST(PostProcessLoadTest, EmptyYamlUsesDefaults) {
  auto path = writeTempYaml("# empty\n", "test_postprocess_empty.yaml");
  auto cfg = config::loadPostProcess(path);

  EXPECT_FALSE(cfg.inpainting.enabled);
  EXPECT_FALSE(cfg.uncertainty_fusion.enabled);
  EXPECT_FALSE(cfg.feature_extraction.enabled);
}

TEST(PostProcessLoadTest, LoadPostProcessYaml) {
  auto cfg = config::loadPostProcess(FASTDEM_CONFIG_DIR "/postprocess.yaml");

  EXPECT_TRUE(cfg.uncertainty_fusion.enabled);
  EXPECT_FALSE(cfg.inpainting.enabled);
  EXPECT_FALSE(cfg.feature_extraction.enabled);
}

// ─── Validation: Non-Fatal Clamping ──────────────────────────────────────────

// ─── PostProcess Validation Tests ─────────────────────────────────────────────

TEST(PostProcessValidationTest, NegativeSearchRadiusClamped) {
  auto path = writeTempYaml(
      "uncertainty_fusion:\n"
      "  search_radius: -0.5\n",
      "test_pp_neg_radius.yaml");
  auto cfg = config::loadPostProcess(path);
  EXPECT_GT(cfg.uncertainty_fusion.search_radius, 0.0f);
}

TEST(PostProcessValidationTest, ZeroSpatialSigmaClamped) {
  auto path = writeTempYaml(
      "uncertainty_fusion:\n"
      "  spatial_sigma: 0.0\n",
      "test_pp_zero_sigma.yaml");
  auto cfg = config::loadPostProcess(path);
  EXPECT_GT(cfg.uncertainty_fusion.spatial_sigma, 0.0f);
}

TEST(PostProcessValidationTest, InvertedQuantilesReset) {
  auto path = writeTempYaml(
      "uncertainty_fusion:\n"
      "  quantile_lower: 0.95\n"
      "  quantile_upper: 0.05\n",
      "test_pp_inv_quantile.yaml");
  auto cfg = config::loadPostProcess(path);
  EXPECT_LT(cfg.uncertainty_fusion.quantile_lower,
             cfg.uncertainty_fusion.quantile_upper);
}

TEST(PostProcessValidationTest, NegativeAnalysisRadiusClamped) {
  auto path = writeTempYaml(
      "feature_extraction:\n"
      "  analysis_radius: -1.0\n",
      "test_pp_neg_analysis.yaml");
  auto cfg = config::loadPostProcess(path);
  EXPECT_GT(cfg.feature_extraction.analysis_radius, 0.0f);
}

TEST(PostProcessValidationTest, NegativeMinNeighborsClamped) {
  auto path = writeTempYaml(
      "inpainting:\n"
      "  max_iterations: -2\n"
      "  min_valid_neighbors: -1\n",
      "test_pp_neg_neighbors.yaml");
  auto cfg = config::loadPostProcess(path);
  EXPECT_GE(cfg.inpainting.max_iterations, 1);
  EXPECT_GE(cfg.inpainting.min_valid_neighbors, 1);
}

// ─── Validation: Non-Fatal Clamping ──────────────────────────────────────────

TEST(ConfigValidationTest, ElevationMarkerOutOfRangeClamped) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  p2:\n"
      "    elevation_marker: 10\n",
      "test_marker_oob.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.mapping.p2.elevation_marker, 0);
  EXPECT_LE(cfg.mapping.p2.elevation_marker, 4);
}
