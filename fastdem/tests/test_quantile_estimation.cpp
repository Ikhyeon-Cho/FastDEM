// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include <random>

#include "fastdem/mapping/quantile_estimation.hpp"

using namespace fastdem;

class P2QuantileTest : public ::testing::Test {
 protected:
  ElevationMap map;
  P2Quantile estimator;
  grid_map::Index idx{0, 0};

  void SetUp() override {
    map.setGeometry(10.0f, 10.0f, 0.5f);
    estimator.ensureLayers(map);
    estimator.bind(map);
  }
};

TEST_F(P2QuantileTest, InitializeCreatesLayers) {
  EXPECT_TRUE(map.exists(layer::p2_q0));
  EXPECT_TRUE(map.exists(layer::p2_q1));
  EXPECT_TRUE(map.exists(layer::p2_q2));
  EXPECT_TRUE(map.exists(layer::p2_q3));
  EXPECT_TRUE(map.exists(layer::p2_q4));
  EXPECT_TRUE(map.exists(layer::p2_n0));
  EXPECT_TRUE(map.exists(layer::p2_n1));
  EXPECT_TRUE(map.exists(layer::p2_n2));
  EXPECT_TRUE(map.exists(layer::p2_n3));
  EXPECT_TRUE(map.exists(layer::p2_n4));
  EXPECT_TRUE(map.exists(layer::upper_bound));
  EXPECT_TRUE(map.exists(layer::lower_bound));
}

TEST_F(P2QuantileTest, LessThanFiveStoredOnly) {
  estimator.update(idx, 3.0f, 0.0f);
  estimator.update(idx, 1.0f, 0.0f);
  estimator.update(idx, 4.0f, 0.0f);

  EXPECT_FLOAT_EQ(map.at(layer::n_points, idx), 3.0f);
}

TEST_F(P2QuantileTest, FiveObservationsActivatesP2) {
  estimator.update(idx, 5.0f, 0.0f);
  estimator.update(idx, 3.0f, 0.0f);
  estimator.update(idx, 1.0f, 0.0f);
  estimator.update(idx, 4.0f, 0.0f);
  estimator.update(idx, 2.0f, 0.0f);

  EXPECT_FLOAT_EQ(map.at(layer::n_points, idx), 5.0f);

  float q0 = map.at(layer::p2_q0, idx);
  float q1 = map.at(layer::p2_q1, idx);
  float q2 = map.at(layer::p2_q2, idx);
  float q3 = map.at(layer::p2_q3, idx);
  float q4 = map.at(layer::p2_q4, idx);

  EXPECT_LE(q0, q1);
  EXPECT_LE(q1, q2);
  EXPECT_LE(q2, q3);
  EXPECT_LE(q3, q4);
}

TEST_F(P2QuantileTest, MarkerMonotonicity) {
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);

  for (int i = 0; i < 100; ++i) {
    estimator.update(idx, dist(gen), 0.0f);
  }

  float q0 = map.at(layer::p2_q0, idx);
  float q1 = map.at(layer::p2_q1, idx);
  float q2 = map.at(layer::p2_q2, idx);
  float q3 = map.at(layer::p2_q3, idx);
  float q4 = map.at(layer::p2_q4, idx);

  EXPECT_LE(q0, q1);
  EXPECT_LE(q1, q2);
  EXPECT_LE(q2, q3);
  EXPECT_LE(q3, q4);
}

TEST_F(P2QuantileTest, NormalDistributionMedianApproximatesMean) {
  std::mt19937 gen(42);
  float true_mean = 5.0f;
  std::normal_distribution<float> dist(true_mean, 1.0f);

  for (int i = 0; i < 1000; ++i) {
    estimator.update(idx, dist(gen), 0.0f);
  }

  estimator.computeBounds(idx);

  // q[2] is median (50th percentile), should approximate true mean
  float median = map.at(layer::p2_q2, idx);
  EXPECT_NEAR(median, true_mean, 0.2f);
}

TEST_F(P2QuantileTest, FinalizeComputesBounds) {
  std::mt19937 gen(42);
  std::normal_distribution<float> dist(5.0f, 1.0f);

  for (int i = 0; i < 500; ++i) {
    estimator.update(idx, dist(gen), 0.0f);
  }

  estimator.computeBounds(idx);

  float lower = map.at(layer::lower_bound, idx);
  float upper = map.at(layer::upper_bound, idx);

  EXPECT_LT(lower, upper);
}

TEST_F(P2QuantileTest, ElevationWrittenInUpdate_BeforeP2) {
  // Before 5 observations: elevation = latest measurement
  estimator.update(idx, 3.0f, 0.0f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 3.0f);

  estimator.update(idx, 7.0f, 0.0f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 7.0f);
}

TEST_F(P2QuantileTest, ElevationWrittenInUpdate_AfterP2) {
  // Default elevation_marker=3 (84th percentile)
  estimator.update(idx, 1.0f, 0.0f);
  estimator.update(idx, 2.0f, 0.0f);
  estimator.update(idx, 3.0f, 0.0f);
  estimator.update(idx, 4.0f, 0.0f);
  estimator.update(idx, 5.0f, 0.0f);

  // After 5 observations: elevation = q[3] (84th percentile marker)
  float elevation = map.at(layer::elevation, idx);
  float q3 = map.at(layer::p2_q3, idx);
  EXPECT_FLOAT_EQ(elevation, q3);

  // After more updates, elevation should still track q[marker]
  estimator.update(idx, 6.0f, 0.0f);
  elevation = map.at(layer::elevation, idx);
  q3 = map.at(layer::p2_q3, idx);
  EXPECT_FLOAT_EQ(elevation, q3);
}
