// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/mapping/kalman_estimation.hpp"

using namespace fastdem;

class KalmanTest : public ::testing::Test {
 protected:
  ElevationMap map;
  grid_map::Index idx{0, 0};

  void SetUp() override { map.setGeometry(10.0f, 10.0f, 0.5f); }
};

TEST_F(KalmanTest, FirstMeasurementInitializesElevation) {
  Kalman estimator(0.0001f, 0.01f, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  estimator.update(idx, 5.0f, 0.04f);

  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 5.0f);
  EXPECT_FLOAT_EQ(map.at(layer::kalman_p, idx), 0.04f);
  EXPECT_FLOAT_EQ(map.at(layer::n_points, idx), 1.0f);
}

TEST_F(KalmanTest, RepeatedLowVarianceMeasurementsReduceP) {
  Kalman estimator(0.0001f, 1.0f, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  estimator.update(idx, 5.0f, 0.5f);
  float initial_p = map.at(layer::kalman_p, idx);

  for (int i = 0; i < 20; ++i) {
    estimator.update(idx, 5.0f, 0.01f);
  }
  float final_p = map.at(layer::kalman_p, idx);

  EXPECT_LT(final_p, initial_p);
}

TEST_F(KalmanTest, KalmanPClamping) {
  float min_var = 0.001f;
  float max_var = 0.1f;
  Kalman estimator(min_var, max_var, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  estimator.update(idx, 5.0f, 0.05f);

  for (int i = 0; i < 100; ++i) {
    estimator.update(idx, 5.0f, 0.0001f);
  }

  float p = map.at(layer::kalman_p, idx);
  EXPECT_GE(p, min_var);
  EXPECT_LE(p, max_var);
}

TEST_F(KalmanTest, FinalizeComputesBoundsFromSampleVariance) {
  Kalman estimator(0.0001f, 1.0f, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  // Feed measurements with spread to build sample variance
  estimator.update(idx, 3.0f, 0.04f);
  estimator.update(idx, 7.0f, 0.04f);
  estimator.computeBounds();

  float elevation = map.at(layer::elevation, idx);
  float variance = map.at(layer::variance, idx);
  float sigma = std::sqrt(variance);

  EXPECT_NEAR(map.at(layer::upper_bound, idx), elevation + 2.0f * sigma, 1e-5f);
  EXPECT_NEAR(map.at(layer::lower_bound, idx), elevation - 2.0f * sigma, 1e-5f);
}

TEST_F(KalmanTest, ZeroMeasurementVarianceFallsBackToMaxVariance) {
  float max_var = 0.5f;
  Kalman estimator(0.0001f, max_var, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  estimator.update(idx, 5.0f, 0.0f);
  EXPECT_FLOAT_EQ(map.at(layer::kalman_p, idx), max_var);
}

TEST_F(KalmanTest, ElevationConvergesToTrueValue) {
  Kalman estimator(0.0001f, 1.0f, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  estimator.update(idx, 10.0f, 1.0f);

  for (int i = 0; i < 50; ++i) {
    estimator.update(idx, 5.0f, 0.01f);
  }

  EXPECT_NEAR(map.at(layer::elevation, idx), 5.0f, 0.1f);
}

TEST_F(KalmanTest, SampleVarianceTracked) {
  Kalman estimator(0.0001f, 1.0f, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  EXPECT_TRUE(map.exists(layer::sample_mean));
  EXPECT_TRUE(map.exists(layer::variance));

  estimator.update(idx, 3.0f, 0.01f);
  estimator.update(idx, 7.0f, 0.01f);

  // Sample variance of {3, 7} = 8
  EXPECT_FLOAT_EQ(map.at(layer::variance, idx), 8.0f);
}

TEST_F(KalmanTest, VarianceIsSampleVarianceNotKalmanP) {
  Kalman estimator(0.0001f, 1.0f, 0.0f);
  estimator.ensureLayers(map);
  estimator.bind(map);

  // After many identical measurements, Kalman P → min_variance (~0)
  // but sample_variance stays 0 (no spread in measurements)
  for (int i = 0; i < 50; ++i) {
    estimator.update(idx, 5.0f, 0.01f);
  }

  float kalman_p = map.at(layer::kalman_p, idx);
  float variance = map.at(layer::variance, idx);

  // kalman_p should have converged near min_variance
  EXPECT_NEAR(kalman_p, 0.0001f, 0.001f);
  // variance (sample variance) should be ~0 since all measurements are 5.0
  EXPECT_NEAR(variance, 0.0f, 1e-6f);
}
