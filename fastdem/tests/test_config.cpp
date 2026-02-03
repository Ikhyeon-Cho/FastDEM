// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * test_config.cpp
 *
 * Tests for MappingConfig YAML loading and validation.
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
  auto cfg = MappingConfig::load(FASTDEM_CONFIG_DIR "/default.yaml");

  EXPECT_EQ(cfg.mapping.mode, MappingMode::LOCAL);
  EXPECT_EQ(cfg.mapping.estimation_type, EstimationType::Kalman);
  EXPECT_EQ(cfg.sensor.type, SensorType::LiDAR);
  EXPECT_EQ(cfg.rasterization.method, RasterMethod::Max);
  EXPECT_TRUE(cfg.raycasting.enabled);
  EXPECT_TRUE(cfg.inpainting.enabled);
}

TEST(ConfigLoadTest, NonexistentFileThrows) {
  EXPECT_THROW(MappingConfig::load("/nonexistent/path.yaml"),
               std::runtime_error);
}

TEST(ConfigLoadTest, EmptyYamlUsesDefaults) {
  auto path = writeTempYaml("# empty config\n", "test_empty.yaml");
  auto cfg = MappingConfig::load(path);

  // Should have all defaults
  MappingConfig defaults;
  EXPECT_EQ(cfg.mapping.mode, defaults.mapping.mode);
  EXPECT_EQ(cfg.mapping.estimation_type, defaults.mapping.estimation_type);
  EXPECT_EQ(cfg.sensor.type, defaults.sensor.type);
  EXPECT_FLOAT_EQ(cfg.scan_filter.z_min, defaults.scan_filter.z_min);
  EXPECT_FLOAT_EQ(cfg.scan_filter.z_max, defaults.scan_filter.z_max);
}

TEST(ConfigLoadTest, PartialYamlPreservesDefaults) {
  auto path = writeTempYaml(
      "scan_filter:\n"
      "  z_min: -2.0\n"
      "  z_max: 5.0\n",
      "test_partial.yaml");
  auto cfg = MappingConfig::load(path);

  // Specified values loaded
  EXPECT_FLOAT_EQ(cfg.scan_filter.z_min, -2.0f);
  EXPECT_FLOAT_EQ(cfg.scan_filter.z_max, 5.0f);

  // Unspecified values remain at defaults
  MappingConfig defaults;
  EXPECT_EQ(cfg.mapping.estimation_type, defaults.mapping.estimation_type);
  EXPECT_FLOAT_EQ(cfg.sensor.range_noise, defaults.sensor.range_noise);
}

TEST(ConfigLoadTest, AllEstimationTypes) {
  auto kalman = MappingConfig::load(writeTempYaml(
      "mapping:\n  type: kalman_filter\n", "test_kalman.yaml"));
  EXPECT_EQ(kalman.mapping.estimation_type, EstimationType::Kalman);

  auto welford = MappingConfig::load(
      writeTempYaml("mapping:\n  type: welford\n", "test_welford.yaml"));
  EXPECT_EQ(welford.mapping.estimation_type, EstimationType::Welford);

  auto p2 = MappingConfig::load(
      writeTempYaml("mapping:\n  type: p2_quantile\n", "test_p2.yaml"));
  EXPECT_EQ(p2.mapping.estimation_type, EstimationType::P2Quantile);
}

TEST(ConfigLoadTest, AllSensorTypes) {
  auto lidar = MappingConfig::load(
      writeTempYaml("sensor:\n  type: lidar\n", "test_lidar.yaml"));
  EXPECT_EQ(lidar.sensor.type, SensorType::LiDAR);

  auto rgbd = MappingConfig::load(
      writeTempYaml("sensor:\n  type: rgbd\n", "test_rgbd.yaml"));
  EXPECT_EQ(rgbd.sensor.type, SensorType::RGBD);

  auto constant = MappingConfig::load(
      writeTempYaml("sensor:\n  type: constant\n", "test_const.yaml"));
  EXPECT_EQ(constant.sensor.type, SensorType::Constant);
}

TEST(ConfigLoadTest, AllRasterMethods) {
  auto max_cfg = MappingConfig::load(
      writeTempYaml("rasterization:\n  method: max\n", "test_rast_max.yaml"));
  EXPECT_EQ(max_cfg.rasterization.method, RasterMethod::Max);

  auto min_cfg = MappingConfig::load(
      writeTempYaml("rasterization:\n  method: min\n", "test_rast_min.yaml"));
  EXPECT_EQ(min_cfg.rasterization.method, RasterMethod::Min);

  auto mean_cfg = MappingConfig::load(
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
  auto cfg = MappingConfig::load(path);

  EXPECT_FLOAT_EQ(cfg.mapping.kalman.min_variance, 0.001f);
  EXPECT_FLOAT_EQ(cfg.mapping.kalman.max_variance, 0.05f);
  EXPECT_FLOAT_EQ(cfg.mapping.kalman.process_noise, 0.001f);
}

// ─── Validation: Fatal Errors ────────────────────────────────────────────────

TEST(ConfigValidationTest, ZMinGeZMaxThrows) {
  auto path = writeTempYaml(
      "scan_filter:\n"
      "  z_min: 5.0\n"
      "  z_max: 1.0\n",
      "test_zrange.yaml");
  EXPECT_THROW(MappingConfig::load(path), std::invalid_argument);
}

TEST(ConfigValidationTest, RangeMinGeRangeMaxThrows) {
  auto path = writeTempYaml(
      "scan_filter:\n"
      "  range_min: 20.0\n"
      "  range_max: 5.0\n",
      "test_rrange.yaml");
  EXPECT_THROW(MappingConfig::load(path), std::invalid_argument);
}

TEST(ConfigValidationTest, KalmanMinVarGeMaxVarThrows) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  kalman:\n"
      "    min_variance: 0.1\n"
      "    max_variance: 0.001\n",
      "test_kalman_inv.yaml");
  EXPECT_THROW(MappingConfig::load(path), std::invalid_argument);
}

TEST(ConfigValidationTest, QuantileLowerGtUpperThrows) {
  auto path = writeTempYaml(
      "uncertainty_fusion:\n"
      "  enabled: true\n"
      "  quantile_lower: 0.99\n"
      "  quantile_upper: 0.01\n",
      "test_quant_inv.yaml");
  EXPECT_THROW(MappingConfig::load(path), std::invalid_argument);
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
  EXPECT_THROW(MappingConfig::load(path), std::invalid_argument);
}

// ─── Validation: Non-Fatal Clamping ──────────────────────────────────────────

TEST(ConfigValidationTest, NegativeRangeNoiseClamped) {
  auto path = writeTempYaml(
      "sensor:\n"
      "  range_noise: -0.5\n",
      "test_neg_noise.yaml");
  auto cfg = MappingConfig::load(path);
  EXPECT_GT(cfg.sensor.range_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeAngularNoiseClamped) {
  auto path = writeTempYaml(
      "sensor:\n"
      "  angular_noise: -1.0\n",
      "test_neg_angular.yaml");
  auto cfg = MappingConfig::load(path);
  EXPECT_GE(cfg.sensor.angular_noise, 0.0f);
}

TEST(ConfigValidationTest, NegativeConstantUncertaintyClamped) {
  auto path = writeTempYaml(
      "sensor:\n"
      "  constant_uncertainty: -0.1\n",
      "test_neg_const.yaml");
  auto cfg = MappingConfig::load(path);
  EXPECT_GT(cfg.sensor.constant_uncertainty, 0.0f);
}

TEST(ConfigValidationTest, NegativeProcessNoiseClamped) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  kalman:\n"
      "    process_noise: -0.01\n",
      "test_neg_pnoise.yaml");
  auto cfg = MappingConfig::load(path);
  EXPECT_GE(cfg.mapping.kalman.process_noise, 0.0f);
}

TEST(ConfigValidationTest, ElevationMarkerOutOfRangeClamped) {
  auto path = writeTempYaml(
      "mapping:\n"
      "  p2:\n"
      "    elevation_marker: 10\n",
      "test_marker_oob.yaml");
  auto cfg = MappingConfig::load(path);
  EXPECT_GE(cfg.mapping.p2.elevation_marker, 0);
  EXPECT_LE(cfg.mapping.p2.elevation_marker, 4);
}
