// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/mapping/kalman_estimation.hpp"

using namespace fastdem;

class KalmanEstimationTest : public ::testing::Test {
 protected:
  ElevationMap map;
  grid_map::Index idx{0, 0};

  void SetUp() override { map.setGeometry(10.0f, 10.0f, 0.5f); }
};

TEST_F(KalmanEstimationTest, FirstMeasurementInitializesState) {
  KalmanEstimation estimator(0.0001f, 0.01f, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 5.0f, 0.04f);

  EXPECT_FLOAT_EQ(map.at(layer::state, idx), 5.0f);
  EXPECT_FLOAT_EQ(map.at(layer::variance, idx), 0.04f);
  EXPECT_FLOAT_EQ(map.at(layer::sample_count, idx), 1.0f);
}

TEST_F(KalmanEstimationTest, RepeatedLowVarianceMeasurementsReduceVariance) {
  KalmanEstimation estimator(0.0001f, 1.0f, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 5.0f, 0.5f);
  float initial_variance = map.at(layer::variance, idx);

  for (int i = 0; i < 20; ++i) {
    estimator.update(idx, 5.0f, 0.01f);
  }
  float final_variance = map.at(layer::variance, idx);

  EXPECT_LT(final_variance, initial_variance);
}

TEST_F(KalmanEstimationTest, VarianceClamping) {
  float min_var = 0.001f;
  float max_var = 0.1f;
  KalmanEstimation estimator(min_var, max_var, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 5.0f, 0.05f);

  for (int i = 0; i < 100; ++i) {
    estimator.update(idx, 5.0f, 0.0001f);
  }

  float variance = map.at(layer::variance, idx);
  EXPECT_GE(variance, min_var);
  EXPECT_LE(variance, max_var);
}

TEST_F(KalmanEstimationTest, FinalizeComputesBounds) {
  KalmanEstimation estimator(0.0001f, 1.0f, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 5.0f, 0.04f);
  estimator.finalize();

  float state = map.at(layer::state, idx);
  float variance = map.at(layer::variance, idx);
  float sigma = std::sqrt(variance);

  EXPECT_NEAR(map.at(layer::upper_bound, idx), state + 2.0f * sigma, 1e-5f);
  EXPECT_NEAR(map.at(layer::lower_bound, idx), state - 2.0f * sigma, 1e-5f);
  EXPECT_NEAR(map.at(layer::uncertainty_range, idx), 4.0f * sigma, 1e-5f);
}

TEST_F(KalmanEstimationTest, MinMaxTracking) {
  KalmanEstimation estimator(0.0001f, 0.01f, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 3.0f, 0.01f);
  estimator.update(idx, 1.0f, 0.01f);
  estimator.update(idx, 5.0f, 0.01f);

  EXPECT_FLOAT_EQ(map.at(layer::elevation_min, idx), 1.0f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation_max, idx), 5.0f);
}

TEST_F(KalmanEstimationTest, ZeroMeasurementVarianceFallsBackToMaxVariance) {
  float max_var = 0.5f;
  KalmanEstimation estimator(0.0001f, max_var, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 5.0f, 0.0f);
  EXPECT_FLOAT_EQ(map.at(layer::variance, idx), max_var);
}

TEST_F(KalmanEstimationTest, StateConvergesToTrueValue) {
  KalmanEstimation estimator(0.0001f, 1.0f, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 10.0f, 1.0f);

  for (int i = 0; i < 50; ++i) {
    estimator.update(idx, 5.0f, 0.01f);
  }

  EXPECT_NEAR(map.at(layer::state, idx), 5.0f, 0.1f);
}

TEST_F(KalmanEstimationTest, WelfordSampleVarianceTracked) {
  KalmanEstimation estimator(0.0001f, 1.0f, 0.0f);
  estimator.initialize(map);

  EXPECT_TRUE(map.exists(layer::mean));
  EXPECT_TRUE(map.exists(layer::sample_variance));

  estimator.update(idx, 3.0f, 0.01f);
  estimator.update(idx, 7.0f, 0.01f);

  // Welford running mean of raw measurements
  EXPECT_FLOAT_EQ(map.at(layer::mean, idx), 5.0f);
  // Sample variance of {3, 7} = 8
  EXPECT_FLOAT_EQ(map.at(layer::sample_variance, idx), 8.0f);
}

TEST_F(KalmanEstimationTest, ElevationWrittenInUpdate_KZero) {
  // k=0: elevation = state
  KalmanEstimation estimator(0.0001f, 1.0f, 0.0f, 0.0f);
  estimator.initialize(map);

  estimator.update(idx, 5.0f, 0.01f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), map.at(layer::state, idx));
}

TEST_F(KalmanEstimationTest, ElevationWithKOffset) {
  // k=1: elevation = state + σ_sample
  KalmanEstimation estimator(0.0001f, 1.0f, 0.0f, 1.0f);
  estimator.initialize(map);

  estimator.update(idx, 3.0f, 0.01f);
  // First measurement: sample_var=0, elevation = state
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), map.at(layer::state, idx));

  estimator.update(idx, 7.0f, 0.01f);
  float state = map.at(layer::state, idx);
  float sample_var = map.at(layer::sample_variance, idx);
  float expected = state + 1.0f * std::sqrt(sample_var);
  EXPECT_NEAR(map.at(layer::elevation, idx), expected, 1e-5f);
  EXPECT_GT(map.at(layer::elevation, idx), state);
}
