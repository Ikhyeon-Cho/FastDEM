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

  EXPECT_EQ(cfg.core.mapping.mode, MappingMode::LOCAL);
  EXPECT_EQ(cfg.core.mapping.estimation_type, EstimationType::Kalman);
  EXPECT_EQ(cfg.core.sensor.type, SensorType::LiDAR);
  EXPECT_EQ(cfg.core.rasterization.method, RasterMethod::Max);
  EXPECT_TRUE(cfg.core.raycasting.enabled);
}

TEST(ConfigLoadTest, NonexistentFileThrows) {
  EXPECT_THROW(loadConfig("/nonexistent/path.yaml"), std::runtime_error);
}

TEST(ConfigLoadTest, EmptyYamlUsesDefaults) {
  auto path = writeTempYaml("# empty config\n", "test_empty.yaml");
  auto cfg = loadConfig(path);

  // Should have all defaults
  CoreConfig defaults;
  EXPECT_EQ(cfg.core.mapping.mode, defaults.mapping.mode);
  EXPECT_EQ(cfg.core.mapping.estimation_type, defaults.mapping.estimation_type);
  EXPECT_EQ(cfg.core.sensor.type, defaults.sensor.type);
  EXPECT_FLOAT_EQ(cfg.core.scan_filter.z_min, defaults.scan_filter.z_min);
  EXPECT_FLOAT_EQ(cfg.core.scan_filter.z_max, defaults.scan_filter.z_max);
}

TEST(ConfigLoadTest, PartialYamlPreservesDefaults) {
  auto path = writeTempYaml(
      "scan_filter:\n"
      "  z_min: -2.0\n"
      "  z_max: 5.0\n",
      "test_partial.yaml");
  auto cfg = loadConfig(path);

  // Specified values loaded
  EXPECT_FLOAT_EQ(cfg.core.scan_filter.z_min, -2.0f);
  EXPECT_FLOAT_EQ(cfg.core.scan_filter.z_max, 5.0f);

  // Unspecified values remain at defaults
  CoreConfig defaults;
  EXPECT_EQ(cfg.core.mapping.estimation_type, defaults.mapping.estimation_type);
  EXPECT_FLOAT_EQ(cfg.core.sensor.range_noise, defaults.sensor.range_noise);
}

TEST(ConfigLoadTest, AllEstimationTypes) {
  auto kalman = loadConfig(
      writeTempYaml("mapping:\n  type: kalman_filter\n", "test_kalman.yaml"));
  EXPECT_EQ(kalman.core.mapping.estimation_type, EstimationType::Kalman);

  auto welford = loadConfig(
      writeTempYaml("mapping:\n  type: welford\n", "test_welford.yaml"));
  EXPECT_EQ(welford.core.mapping.estimation_type, EstimationType::Welford);

  auto p2 = loadConfig(
      writeTempYaml("mapping:\n  type: p2_quantile\n", "test_p2.yaml"));
  EXPECT_EQ(p2.core.mapping.estimation_type, EstimationType::P2Quantile);
}

TEST(ConfigLoadTest, AllSensorTypes) {
  auto lidar = loadConfig(
      writeTempYaml("sensor:\n  type: lidar\n", "test_lidar.yaml"));
  EXPECT_EQ(lidar.core.sensor.type, SensorType::LiDAR);

  auto rgbd = loadConfig(
      writeTempYaml("sensor:\n  type: rgbd\n", "test_rgbd.yaml"));
  EXPECT_EQ(rgbd.core.sensor.type, SensorType::RGBD);

  auto constant = loadConfig(
      writeTempYaml("sensor:\n  type: constant\n", "test_const.yaml"));
  EXPECT_EQ(constant.core.sensor.type, SensorType::Constant);
}

TEST(ConfigLoadTest, AllRasterMethods) {
  auto max_cfg = loadConfig(
      writeTempYaml("rasterization:\n  method: max\n", "test_rast_max.yaml"));
  EXPECT_EQ(max_cfg.core.rasterization.method, RasterMethod::Max);

  auto min_cfg = loadConfig(
      writeTempYaml("rasterization:\n  method: min\n", "test_rast_min.yaml"));
  EXPECT_EQ(min_cfg.core.rasterization.method, RasterMethod::Min);

  auto mean_cfg = loadConfig(
      writeTempYaml("rasterization:\n  method: mean\n", "test_rast_mean.yaml"));
  EXPECT_EQ(mean_cfg.core.rasterization.method, RasterMethod::Mean);
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

  EXPECT_FLOAT_EQ(cfg.core.mapping.kalman.min_variance, 0.001f);
  EXPECT_FLOAT_EQ(cfg.core.mapping.kalman.max_variance, 0.05f);
  EXPECT_FLOAT_EQ(cfg.core.mapping.kalman.process_noise, 0.001f);
}

// ─── Validation: Fatal Errors ────────────────────────────────────────────────

TEST(ConfigValidationTest, ZMinGeZMaxThrows) {
  auto path = writeTempYaml(
      "scan_filter:\n"
      "  z_min: 5.0\n"
      "  z_max: 1.0\n",
      "test_zrange.yaml");
  EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST(ConfigValidationTest, RangeMinGeRangeMaxThrows) {
  auto path = writeTempYaml(
      "scan_filter:\n"
      "  range_min: 20.0\n"
      "  range_max: 5.0\n",
      "test_rrange.yaml");
  EXPECT_THROW(loadConfig(path), std::invalid_argument);
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
      "sensor:\n"
      "  range_noise: -0.5\n",
      "test_neg_noise.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GT(cfg.core.sensor.range_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeAngularNoiseClamped) {
  auto path = writeTempYaml(
      "sensor:\n"
      "  angular_noise: -1.0\n",
      "test_neg_angular.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.core.sensor.angular_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeConstantUncertaintyClamped) {
  auto path = writeTempYaml(
      "sensor:\n"
      "  constant_uncertainty: -0.1\n",
      "test_neg_const.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GT(cfg.core.sensor.constant_uncertainty, 0.0f);
}

TEST(ConfigValidationTest, NegativeProcessNoiseClamped) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  kalman:\n"
      "    process_noise: -0.01\n",
      "test_neg_pnoise.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.core.mapping.kalman.process_noise, 0.0f);
}

TEST(ConfigValidationTest, ElevationMarkerOutOfRangeClamped) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  p2:\n"
      "    elevation_marker: 10\n",
      "test_marker_oob.yaml");
  auto cfg = loadConfig(path);
  EXPECT_GE(cfg.core.mapping.p2.elevation_marker, 0);
  EXPECT_LE(cfg.core.mapping.p2.elevation_marker, 4);
}
