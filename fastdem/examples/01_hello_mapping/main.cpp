// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * 01_hello_mapping - FastDEM basic usage
 *
 * Demonstrates:
 * - Creating ElevationMap and FastDEM
 * - Configuring with setter API
 * - Integrating point clouds (offline mode)
 * - Accessing elevation data
 */

#include <fastdem/fastdem.hpp>

#include <iostream>

#include "../common/data_loader.hpp"
#include "../common/timer.hpp"
#include "../common/visualization.hpp"

using namespace fastdem;

int main() {
  std::cout << "=== 01_hello_mapping ===\n" << std::endl;

  // 1. Create map
  ElevationMap map;
  map.setGeometry(10.0f, 10.0f, 0.1f);
  map.setFrameId("map");

  // 2. Create FastDEM and configure
  FastDEM mapper(map);
  mapper.setHeightFilter(-1.0f, 2.0f)
      .setRangeFilter(0.5f, 10.0f)
      .setEstimatorType(EstimationType::Welford)
      .setSensorModel(SensorType::Constant);

  // 3. Generate and integrate
  auto cloud = examples::generateTerrainCloud(50000, 8.0f);
  std::cout << "Generated " << cloud.size() << " points" << std::endl;

  examples::Timer timer;
  timer.start();
  mapper.integrate(cloud, Eigen::Isometry3d::Identity(),
                   Eigen::Isometry3d::Identity());
  timer.printElapsed("Integration");

  // 4. Results
  examples::printMapStats(map);
  examples::printAsciiMap(map, 50);
  examples::saveMapImage(map, std::string(EXAMPLE_OUTPUT_DIR) + "/output.png");

  // 5. Direct access
  grid_map::Position origin(0.0, 0.0);
  if (map.hasElevationAt(origin)) {
    std::cout << "Elevation at origin: " << map.elevationAt(origin) << " m"
              << std::endl;
  }

  return 0;
}
