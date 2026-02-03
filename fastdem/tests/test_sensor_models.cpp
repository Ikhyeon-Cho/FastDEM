// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include "fastdem/sensors/lidar_model.hpp"
#include "fastdem/sensors/rgbd_model.hpp"
#include "fastdem/sensors/sensor_model.hpp"

using namespace fastdem;

// ── ConstantUncertaintyModel
// ──────────────────────────────────────────────────

TEST(ConstantUncertaintyModelTest, ReturnsScaledIdentity) {
  float sigma = 0.1f;
  ConstantUncertaintyModel model(sigma);

  auto cov = model.computeSensorCovariance(Eigen::Vector3f(1, 2, 3));
  Eigen::Matrix3f expected = Eigen::Matrix3f::Identity() * (sigma * sigma);

  EXPECT_TRUE(cov.isApprox(expected, 1e-6f));
}

TEST(ConstantUncertaintyModelTest, PositionIndependent) {
  ConstantUncertaintyModel model(0.05f);

  auto cov1 = model.computeSensorCovariance(Eigen::Vector3f(0, 0, 0));
  auto cov2 = model.computeSensorCovariance(Eigen::Vector3f(10, 20, 30));

  EXPECT_TRUE(cov1.isApprox(cov2, 1e-6f));
}

// ── LiDARSensorModel ─────────────────────────────────────────────────────────

class LiDARSensorModelTest : public ::testing::Test {
 protected:
  float range_noise = 0.02f;
  float angular_noise = 0.001f;
  LiDARSensorModel model{range_noise, angular_noise};
};

TEST_F(LiDARSensorModelTest, CovarianceIsSymmetric) {
  auto cov = model.computeSensorCovariance(Eigen::Vector3f(5, 3, 2));
  EXPECT_TRUE(cov.isApprox(cov.transpose(), 1e-6f));
}

TEST_F(LiDARSensorModelTest, CovarianceIsPositiveSemiDefinite) {
  auto cov = model.computeSensorCovariance(Eigen::Vector3f(5, 3, 2));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  auto eigenvalues = solver.eigenvalues();
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(eigenvalues(i), -1e-8f);
  }
}

TEST_F(LiDARSensorModelTest, ZeroDistanceFallback) {
  auto cov = model.computeSensorCovariance(Eigen::Vector3f(0, 0, 0));
  // Should return fallback_variance * I = 0.01 * I
  float expected = 0.01f;
  EXPECT_NEAR(cov(0, 0), expected, 1e-6f);
  EXPECT_NEAR(cov(1, 1), expected, 1e-6f);
  EXPECT_NEAR(cov(2, 2), expected, 1e-6f);
}

TEST_F(LiDARSensorModelTest, BeamDirectionVarianceIsRangeNoiseSq) {
  // Point along X axis: beam_dir = (1, 0, 0)
  float distance = 10.0f;
  Eigen::Vector3f point(distance, 0, 0);
  auto cov = model.computeSensorCovariance(point);

  // Beam direction variance = range_noise^2
  float var_radial = range_noise * range_noise;
  // Lateral variance = (d * angular_noise)^2
  float var_lateral = (distance * angular_noise) * (distance * angular_noise);

  // For point along X: cov(0,0) should be radial variance
  EXPECT_NEAR(cov(0, 0), var_radial, 1e-6f);
  // Perpendicular directions should have lateral variance
  EXPECT_NEAR(cov(1, 1), var_lateral, 1e-6f);
  EXPECT_NEAR(cov(2, 2), var_lateral, 1e-6f);
}

// ── RGBDSensorModel ──────────────────────────────────────────────────────────

class RGBDSensorModelTest : public ::testing::Test {
 protected:
  float a = 0.001f;
  float b = 0.002f;
  float c = 0.4f;
  float lateral_factor = 0.001f;
  RGBDSensorModel model{a, b, c, lateral_factor};
};

TEST_F(RGBDSensorModelTest, DiagonalStructure) {
  Eigen::Vector3f point(0.1f, 0.2f, 1.0f);
  auto cov = model.computeSensorCovariance(point);

  // RGBD model produces diagonal covariance
  EXPECT_NEAR(cov(0, 1), 0.0f, 1e-10f);
  EXPECT_NEAR(cov(0, 2), 0.0f, 1e-10f);
  EXPECT_NEAR(cov(1, 2), 0.0f, 1e-10f);
}

TEST_F(RGBDSensorModelTest, OptimalDepthMinimizesNormalNoise) {
  // At optimal depth c, sigma_norm = a (minimum)
  // At other depths, sigma_norm = a + b*(d-c)^2 > a
  Eigen::Vector3f at_optimal(0, 0, c);
  Eigen::Vector3f far_away(0, 0, c + 2.0f);

  auto cov_opt = model.computeSensorCovariance(at_optimal);
  auto cov_far = model.computeSensorCovariance(far_away);

  // Z variance at optimal should be less than at far distance
  EXPECT_LT(cov_opt(2, 2), cov_far(2, 2));
}

TEST_F(RGBDSensorModelTest, ZeroDepthFallback) {
  auto cov = model.computeSensorCovariance(Eigen::Vector3f(0, 0, 0));
  float expected = 0.01f;  // fallback_variance
  EXPECT_NEAR(cov(0, 0), expected, 1e-6f);
  EXPECT_NEAR(cov(1, 1), expected, 1e-6f);
  EXPECT_NEAR(cov(2, 2), expected, 1e-6f);
}

TEST_F(RGBDSensorModelTest, LateralVarianceScalesWithDepth) {
  float depth1 = 1.0f;
  float depth2 = 2.0f;
  auto cov1 = model.computeSensorCovariance(Eigen::Vector3f(0, 0, depth1));
  auto cov2 = model.computeSensorCovariance(Eigen::Vector3f(0, 0, depth2));

  // σ_lat = lateral_factor * depth, so var_lat ∝ depth²
  float ratio = cov2(0, 0) / cov1(0, 0);
  float expected_ratio = (depth2 * depth2) / (depth1 * depth1);
  EXPECT_NEAR(ratio, expected_ratio, 1e-4f);
}
