// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * 03_estimator_comparison - Height estimation algorithm comparison
 *
 * Demonstrates:
 * - Using all three estimator types (Kalman, Welford, P2Quantile)
 * - Comparing estimation results on identical data
 * - Multiple integration passes with different noise seeds
 */

#include <fastdem/fastdem.hpp>

#include <iostream>
#include <memory>

#include "../common/data_loader.hpp"
#include "../common/timer.hpp"
#include "../common/visualization.hpp"

using namespace fastdem;

int main() {
  std::cout << "=== 03_estimator_comparison ===\n" << std::endl;

  // Estimator configurations
  struct EstimatorInfo {
    EstimationType type;
    const char* name;
  };
  EstimatorInfo estimators[] = {
      {EstimationType::Kalman, "Kalman"},
      {EstimationType::Welford, "Welford"},
      {EstimationType::P2Quantile, "P2Quantile"},
  };

  // Create maps and mappers
  constexpr int N = 3;
  ElevationMap maps[N];
  std::unique_ptr<FastDEM> mappers[N];

  for (int i = 0; i < N; ++i) {
    maps[i].setGeometry(10.0f, 10.0f, 0.1f);
    maps[i].setFrameId("map");

    mappers[i] = std::make_unique<FastDEM>(maps[i]);
    mappers[i]->setHeightRange(-1.0f, 2.0f)
        .setDistanceRange(0.5f, 10.0f)
        .setSensorModel(SensorType::Constant)
        .setEstimatorType(estimators[i].type);

    std::cout << "Created mapper with " << estimators[i].name << " estimator"
              << std::endl;
  }

  // Generate and integrate 3 passes with different noise seeds
  const auto I = Eigen::Isometry3d::Identity();
  examples::Timer timer;

  for (int pass = 0; pass < 10; ++pass) {
    auto cloud = examples::generateTerrainCloud(50000, 8.0f, 42 + pass);
    std::cout << "\nPass " << pass << ": " << cloud.size() << " points"
              << std::endl;

    for (int i = 0; i < N; ++i) {
      timer.start();
      mappers[i]->integrate(cloud, I, I);
      timer.printElapsed(std::string("  ") + estimators[i].name);
    }
  }

  // Print results and save images
  std::cout << std::endl;
  for (int i = 0; i < N; ++i) {
    examples::printMapStats(maps[i], estimators[i].name);
    examples::saveMapImage(
        maps[i], std::string(EXAMPLE_OUTPUT_DIR) + "/" + estimators[i].name +
                     ".png");
  }

  return 0;
}
