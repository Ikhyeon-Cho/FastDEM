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
  EXPECT_EQ(cfg.rasterization.method, RasterMethod::Max);
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
      "  type: welford\n",
      "test_partial.yaml");
  auto cfg = loadConfig(path);

  // Specified value loaded
  EXPECT_EQ(cfg.mapping.estimation_type, EstimationType::Welford);

  // Unspecified values remain at defaults
  Config defaults;
  EXPECT_EQ(cfg.mapping.mode, defaults.mapping.mode);
  EXPECT_FLOAT_EQ(cfg.sensor_model.range_noise, defaults.sensor_model.range_noise);
}

TEST(ConfigLoadTest, AllEstimationTypes) {
  auto kalman = loadConfig(
      writeTempYaml("mapping:\n  type: kalman_filter\n", "test_kalman.yaml"));
  EXPECT_EQ(kalman.mapping.estimation_type, EstimationType::Kalman);

  auto welford = loadConfig(
      writeTempYaml("mapping:\n  type: welford\n", "test_welford.yaml"));
  EXPECT_EQ(welford.mapping.estimation_type, EstimationType::Welford);

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

TEST(ConfigLoadTest, AllRasterMethods) {
  auto max_cfg = loadConfig(
      writeTempYaml("rasterization:\n  method: max\n", "test_rast_max.yaml"));
  EXPECT_EQ(max_cfg.rasterization.method, RasterMethod::Max);

  auto min_cfg = loadConfig(
      writeTempYaml("rasterization:\n  method: min\n", "test_rast_min.yaml"));
  EXPECT_EQ(min_cfg.rasterization.method, RasterMethod::Min);

  auto mean_cfg = loadConfig(
      writeTempYaml("rasterization:\n  method: mean\n", "test_rast_mean.yaml"));
  EXPECT_EQ(mean_cfg.rasterization.method, RasterMethod::Mean);
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

// point_filter is no longer parsed by loadConfig/parseConfig.
// Filter bounds are set by the application via setHeightFilter/setRangeFilter.

TEST(ConfigValidationTest, KalmanMinVarGeMaxVarThrows) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  kalman:\n"
      "    min_variance: 0.1\n"
      "    max_variance: 0.001\n",
      "test_kalman_inv.yaml");
  EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST(ConfigValidationTest, QuantileLowerGtUpperThrows) {
  auto path = writeTempYaml(
      "uncertainty_fusion:\n"
      "  enabled: true\n"
      "  quantile_lower: 0.99\n"
      "  quantile_upper: 0.01\n",
      "test_quant_inv.yaml");
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
      "  range_noise: -0.5\n",
      "test_neg_noise.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GT(cfg.sensor_model.range_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeAngularNoiseClamped) {
  auto path = writeTempYaml(
      "sensor_model:\n"
      "  angular_noise: -1.0\n",
      "test_neg_angular.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.sensor_model.angular_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeConstantUncertaintyClamped) {
  auto path = writeTempYaml(
      "sensor_model:\n"
      "  constant_uncertainty: -0.1\n",
      "test_neg_const.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GT(cfg.sensor_model.constant_uncertainty, 0.0f);
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
