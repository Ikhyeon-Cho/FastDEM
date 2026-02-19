// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/mapping/mean_estimation.hpp"

using namespace fastdem;

class MeanEstimationTest : public ::testing::Test {
 protected:
  ElevationMap map;
  MeanEstimation estimator;
  grid_map::Index idx{0, 0};

  void SetUp() override {
    map.setGeometry(10.0f, 10.0f, 0.5f);
    estimator.initialize(map);
  }
};

TEST_F(MeanEstimationTest, InitializeCreatesLayers) {
  EXPECT_TRUE(map.exists(layer::elevation));
  EXPECT_TRUE(map.exists(layer::variance));
  EXPECT_TRUE(map.exists(layer::sample_count));
  EXPECT_TRUE(map.exists(layer::elevation_min));
  EXPECT_TRUE(map.exists(layer::elevation_max));
  EXPECT_TRUE(map.exists(layer::standard_error));
  EXPECT_TRUE(map.exists(layer::conf_interval_95));
  EXPECT_TRUE(map.exists(layer::upper_bound));
  EXPECT_TRUE(map.exists(layer::lower_bound));
  EXPECT_TRUE(map.exists(layer::uncertainty_range));
}

TEST_F(MeanEstimationTest, SingleMeasurement) {
  estimator.update(idx, 5.0f, 0.01f);

  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 5.0f);
  EXPECT_FLOAT_EQ(map.at(layer::sample_count, idx), 1.0f);
  EXPECT_FLOAT_EQ(map.at(layer::variance, idx), 0.0f);
}

TEST_F(MeanEstimationTest, IdenticalMeasurementsZeroVariance) {
  for (int i = 0; i < 10; ++i) {
    estimator.update(idx, 3.0f, 0.01f);
  }

  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 3.0f);
  EXPECT_NEAR(map.at(layer::variance, idx), 0.0f, 1e-10f);
  EXPECT_FLOAT_EQ(map.at(layer::sample_count, idx), 10.0f);
}

TEST_F(MeanEstimationTest, TwoMeasurementsVariance) {
  estimator.update(idx, 3.0f, 0.01f);
  estimator.update(idx, 7.0f, 0.01f);

  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 5.0f);
  EXPECT_FLOAT_EQ(map.at(layer::variance, idx), 8.0f);
  EXPECT_FLOAT_EQ(map.at(layer::sample_count, idx), 2.0f);
}

TEST_F(MeanEstimationTest, MultipleMeasurementsMean) {
  float values[] = {2.0f, 4.0f, 4.0f, 4.0f, 5.0f, 5.0f, 7.0f, 9.0f};
  for (float v : values) {
    estimator.update(idx, v, 0.01f);
  }

  EXPECT_NEAR(map.at(layer::elevation, idx), 5.0f, 1e-5f);
  EXPECT_FLOAT_EQ(map.at(layer::sample_count, idx), 8.0f);
  float variance = map.at(layer::variance, idx);
  EXPECT_GT(variance, 0.0f);
  EXPECT_TRUE(std::isfinite(variance));
}

TEST_F(MeanEstimationTest, MinMaxTracking) {
  estimator.update(idx, 3.0f, 0.01f);
  estimator.update(idx, 1.0f, 0.01f);
  estimator.update(idx, 5.0f, 0.01f);

  EXPECT_FLOAT_EQ(map.at(layer::elevation_min, idx), 1.0f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation_max, idx), 5.0f);
}

TEST_F(MeanEstimationTest, FinalizeComputesSEAndCI) {
  float values[] = {2.0f, 4.0f, 4.0f, 4.0f, 5.0f, 5.0f, 7.0f, 9.0f};
  for (float v : values) {
    estimator.update(idx, v, 0.01f);
  }

  estimator.finalize();

  float variance = map.at(layer::variance, idx);
  float count = map.at(layer::sample_count, idx);
  float expected_se = std::sqrt(variance / count);
  float expected_ci = 1.96f * expected_se;

  EXPECT_NEAR(map.at(layer::standard_error, idx), expected_se, 1e-5f);
  EXPECT_NEAR(map.at(layer::conf_interval_95, idx), expected_ci, 1e-5f);
}

TEST_F(MeanEstimationTest, FinalizeComputesBounds) {
  float values[] = {2.0f, 4.0f, 4.0f, 4.0f, 5.0f, 5.0f, 7.0f, 9.0f};
  for (float v : values) {
    estimator.update(idx, v, 0.01f);
  }

  estimator.finalize();

  float elevation = map.at(layer::elevation, idx);
  float ci = map.at(layer::conf_interval_95, idx);

  EXPECT_NEAR(map.at(layer::upper_bound, idx), elevation + ci, 1e-5f);
  EXPECT_NEAR(map.at(layer::lower_bound, idx), elevation - ci, 1e-5f);
  float range = map.at(layer::uncertainty_range, idx);
  EXPECT_NEAR(range, 2.0f * ci, 1e-5f);
}

TEST_F(MeanEstimationTest, ElevationIsMean) {
  estimator.update(idx, 5.0f, 0.01f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 5.0f);

  estimator.update(idx, 7.0f, 0.01f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 6.0f);
}
