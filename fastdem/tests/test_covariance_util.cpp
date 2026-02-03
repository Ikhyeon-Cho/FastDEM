// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/sensors/covariance_util.hpp"

using namespace fastdem::utils;

TEST(ProjectToHeightVarianceTest, IdentityRotationReturnsZZElement) {
  Eigen::Matrix3f cov;
  cov << 1, 0, 0,  //
      0, 2, 0,     //
      0, 0, 3;

  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

  float result = projectToHeightVariance(cov, R);
  EXPECT_NEAR(result, 3.0f, 1e-6f);
}

TEST(ProjectToHeightVarianceTest, ScalingInvariance) {
  Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

  float base = projectToHeightVariance(cov, R);

  float k = 3.0f;
  Eigen::Matrix3f scaled_cov = cov * (k * k);
  float scaled = projectToHeightVariance(scaled_cov, R);

  EXPECT_NEAR(scaled, k * k * base, 1e-5f);
}

TEST(ProjectToHeightVarianceTest, ZeroCovarianceReturnsZero) {
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

  float result = projectToHeightVariance(cov, R);
  EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST(ProjectToHeightVarianceTest, Rotation90AroundX) {
  // 90° rotation around X: maps sensor Z → map Y
  // So height (map Z) draws from sensor Y component
  Eigen::Matrix3f R;
  R << 1, 0, 0,  //
      0, 0, -1,  //
      0, 1, 0;

  Eigen::Matrix3f cov;
  cov << 1, 0, 0,  //
      0, 4, 0,     //
      0, 0, 9;

  // r_z = R^T * [0,0,1] = third row of R = [0, 1, 0]
  // σ_z² = r_z^T * cov * r_z = cov(1,1) = 4
  float result = projectToHeightVariance(cov, R);
  EXPECT_NEAR(result, 4.0f, 1e-5f);
}

TEST(ProjectToHeightVariancesTest, BatchMatchesSingle) {
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

  nanopcl::AlignedVector<nanopcl::Covariance> covs;
  covs.push_back(Eigen::Matrix3f::Identity() * 1.0f);
  covs.push_back(Eigen::Matrix3f::Identity() * 2.0f);
  covs.push_back(Eigen::Matrix3f::Identity() * 3.0f);

  auto variances = projectToHeightVariances(covs, R);

  ASSERT_EQ(variances.size(), 3u);
  for (size_t i = 0; i < covs.size(); ++i) {
    float expected = projectToHeightVariance(covs[i], R);
    EXPECT_FLOAT_EQ(variances[i], expected);
  }
}
