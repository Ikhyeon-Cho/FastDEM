// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include "fastdem/sensors/lidar_model.hpp"
#include "fastdem/sensors/rgbd_model.hpp"
#include "fastdem/sensors/sensor_model.hpp"

using namespace fastdem;

// ── createSensorModel (Factory)
// ─────────────────────────────────────────────

TEST(CreateSensorModelTest, CreateLiDAR) {
  config::SensorModel cfg;
  cfg.type = SensorType::LiDAR;
  cfg.range_noise = 0.03f;
  auto model = createSensorModel(cfg);

  // LiDAR model: beam along X at 10m → cov(0,0) = range_noise²
  auto cov = model->computeCovariance(Eigen::Vector3f(10, 0, 0));
  EXPECT_NEAR(cov(0, 0), 0.03f * 0.03f, 1e-6f);
}

TEST(CreateSensorModelTest, CreateRGBD) {
  config::SensorModel cfg;
  cfg.type = SensorType::RGBD;
  cfg.normal_a = 0.002f;
  cfg.normal_c = 0.5f;
  auto model = createSensorModel(cfg);

  // At optimal depth c: var_norm = a²
  auto cov = model->computeCovariance(Eigen::Vector3f(0, 0, cfg.normal_c));
  EXPECT_NEAR(cov(2, 2), cfg.normal_a * cfg.normal_a, 1e-8f);
}

TEST(CreateSensorModelTest, CreateConstant) {
  config::SensorModel cfg;
  cfg.type = SensorType::Constant;
  cfg.constant_uncertainty = 0.2f;
  auto model = createSensorModel(cfg);

  auto cov = model->computeCovariance(Eigen::Vector3f(1, 2, 3));
  float expected = 0.2f * 0.2f;
  EXPECT_NEAR(cov(0, 0), expected, 1e-6f);
  EXPECT_NEAR(cov(1, 1), expected, 1e-6f);
  EXPECT_NEAR(cov(2, 2), expected, 1e-6f);
}

// ── ConstantUncertaintyModel
// ──────────────────────────────────────────────────

TEST(ConstantUncertaintyModelTest, ReturnsScaledIdentity) {
  float sigma = 0.1f;
  ConstantUncertaintyModel model(sigma);

  auto cov = model.computeCovariance(Eigen::Vector3f(1, 2, 3));
  Eigen::Matrix3f expected = Eigen::Matrix3f::Identity() * (sigma * sigma);

  EXPECT_TRUE(cov.isApprox(expected, 1e-6f));
}

TEST(ConstantUncertaintyModelTest, PositionIndependent) {
  ConstantUncertaintyModel model(0.05f);

  auto cov1 = model.computeCovariance(Eigen::Vector3f(0, 0, 0));
  auto cov2 = model.computeCovariance(Eigen::Vector3f(10, 20, 30));

  EXPECT_TRUE(cov1.isApprox(cov2, 1e-6f));
}

TEST(ConstantUncertaintyModelTest, ZeroUncertaintyReturnsZeroMatrix) {
  ConstantUncertaintyModel model(0.0f);
  auto cov = model.computeCovariance(Eigen::Vector3f(1, 2, 3));
  EXPECT_TRUE(cov.isApprox(Eigen::Matrix3f::Zero(), 1e-10f));
}

// ── LiDARSensorModel ─────────────────────────────────────────────────────────

class LiDARSensorModelTest : public ::testing::Test {
 protected:
  float range_noise = 0.02f;
  float angular_noise = 0.001f;
  LiDARSensorModel model{range_noise, angular_noise};
};

TEST_F(LiDARSensorModelTest, CovarianceIsSymmetric) {
  auto cov = model.computeCovariance(Eigen::Vector3f(5, 3, 2));
  EXPECT_TRUE(cov.isApprox(cov.transpose(), 1e-6f));
}

TEST_F(LiDARSensorModelTest, CovarianceIsPositiveSemiDefinite) {
  auto cov = model.computeCovariance(Eigen::Vector3f(5, 3, 2));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  auto eigenvalues = solver.eigenvalues();
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(eigenvalues(i), -1e-8f);
  }
}

TEST_F(LiDARSensorModelTest, ZeroDistanceFallback) {
  auto cov = model.computeCovariance(Eigen::Vector3f(0, 0, 0));
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
  auto cov = model.computeCovariance(point);

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

TEST_F(LiDARSensorModelTest, DiagonalBeamDirections) {
  float distance = 10.0f;
  float var_radial = range_noise * range_noise;
  float var_lateral = (distance * angular_noise) * (distance * angular_noise);

  // Y axis: cov(1,1) = radial, cov(0,0) = cov(2,2) = lateral
  auto cov_y = model.computeCovariance(Eigen::Vector3f(0, distance, 0));
  EXPECT_NEAR(cov_y(1, 1), var_radial, 1e-6f);
  EXPECT_NEAR(cov_y(0, 0), var_lateral, 1e-6f);
  EXPECT_NEAR(cov_y(2, 2), var_lateral, 1e-6f);

  // Z axis: cov(2,2) = radial, cov(0,0) = cov(1,1) = lateral
  auto cov_z = model.computeCovariance(Eigen::Vector3f(0, 0, distance));
  EXPECT_NEAR(cov_z(2, 2), var_radial, 1e-6f);
  EXPECT_NEAR(cov_z(0, 0), var_lateral, 1e-6f);
  EXPECT_NEAR(cov_z(1, 1), var_lateral, 1e-6f);

  // Diagonal (1,1,1)/√3: all eigenvalues must be var_radial or var_lateral
  float d_diag = distance / std::sqrt(3.0f);
  auto cov_diag = model.computeCovariance(Eigen::Vector3f(d_diag, d_diag, d_diag));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov_diag);
  auto evals = solver.eigenvalues();
  // Distance = d_diag * √3 = distance, so same var_radial / var_lateral
  // One eigenvalue = var_radial, two = var_lateral
  std::vector<float> sorted = {evals(0), evals(1), evals(2)};
  std::sort(sorted.begin(), sorted.end());
  EXPECT_NEAR(sorted[0], var_lateral, 1e-5f);
  EXPECT_NEAR(sorted[1], var_lateral, 1e-5f);
  EXPECT_NEAR(sorted[2], var_radial, 1e-5f);
}

TEST_F(LiDARSensorModelTest, LongRangePSD) {
  // At d=50m, var_lateral = (50*0.001)² = 0.0025 > var_radial = 0.0004
  auto cov = model.computeCovariance(Eigen::Vector3f(50, 0, 0));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(solver.eigenvalues()(i), -1e-8f);
  }
}

TEST_F(LiDARSensorModelTest, BatchMatchesSingle) {
  PointCloud cloud(3);
  cloud.point(0) = Eigen::Vector3f(5, 0, 0);
  cloud.point(1) = Eigen::Vector3f(0, 3, 0);
  cloud.point(2) = Eigen::Vector3f(1, 2, 3);

  PointCloud result = model.computeCovariances(std::move(cloud));
  ASSERT_TRUE(result.hasCovariance());

  // Verify batch matches single
  Eigen::Vector3f pts[] = {{5, 0, 0}, {0, 3, 0}, {1, 2, 3}};
  for (size_t i = 0; i < 3; ++i) {
    auto expected = model.computeCovariance(pts[i]);
    EXPECT_TRUE(result.covariance(i).isApprox(expected, 1e-6f));
  }
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
  auto cov = model.computeCovariance(point);

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

  auto cov_opt = model.computeCovariance(at_optimal);
  auto cov_far = model.computeCovariance(far_away);

  // Z variance at optimal should be less than at far distance
  EXPECT_LT(cov_opt(2, 2), cov_far(2, 2));
}

TEST_F(RGBDSensorModelTest, ZeroDepthFallback) {
  auto cov = model.computeCovariance(Eigen::Vector3f(0, 0, 0));
  float expected = 0.01f;  // fallback_variance
  EXPECT_NEAR(cov(0, 0), expected, 1e-6f);
  EXPECT_NEAR(cov(1, 1), expected, 1e-6f);
  EXPECT_NEAR(cov(2, 2), expected, 1e-6f);
}

TEST_F(RGBDSensorModelTest, LateralVarianceScalesWithDepth) {
  float depth1 = 1.0f;
  float depth2 = 2.0f;
  auto cov1 = model.computeCovariance(Eigen::Vector3f(0, 0, depth1));
  auto cov2 = model.computeCovariance(Eigen::Vector3f(0, 0, depth2));

  // σ_lat = lateral_factor * depth, so var_lat ∝ depth²
  float ratio = cov2(0, 0) / cov1(0, 0);
  float expected_ratio = (depth2 * depth2) / (depth1 * depth1);
  EXPECT_NEAR(ratio, expected_ratio, 1e-4f);
}

TEST_F(RGBDSensorModelTest, NegativeDepthFallback) {
  auto cov = model.computeCovariance(Eigen::Vector3f(1, 2, -0.5f));
  float expected = 0.01f;  // fallback_variance
  EXPECT_NEAR(cov(0, 0), expected, 1e-6f);
  EXPECT_NEAR(cov(1, 1), expected, 1e-6f);
  EXPECT_NEAR(cov(2, 2), expected, 1e-6f);
}

TEST_F(RGBDSensorModelTest, OptimalDepthExactValue) {
  // At depth = c: σ_norm = a, so var_norm = a²
  auto cov = model.computeCovariance(Eigen::Vector3f(0, 0, c));
  EXPECT_NEAR(cov(2, 2), a * a, 1e-10f);
}

TEST_F(RGBDSensorModelTest, XYPositionIndependent) {
  // Same depth, different XY → same covariance (model uses only z)
  float depth = 1.5f;
  auto cov1 = model.computeCovariance(Eigen::Vector3f(0, 0, depth));
  auto cov2 = model.computeCovariance(Eigen::Vector3f(3, 4, depth));
  EXPECT_TRUE(cov1.isApprox(cov2, 1e-10f));
}
