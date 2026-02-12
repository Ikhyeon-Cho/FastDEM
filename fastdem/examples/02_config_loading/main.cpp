// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * 02_config_loading - YAML configuration loading
 *
 * Demonstrates:
 * - Loading LOCAL and GLOBAL presets from YAML config files
 * - Creating FastDEM instances from loaded configs
 * - Comparing results of different mapping modes
 */

#include <fastdem/fastdem.hpp>

#include <iostream>

#include "../common/data_loader.hpp"
#include "../common/timer.hpp"
#include "../common/visualization.hpp"

using namespace fastdem;

int main() {
  std::cout << "=== 02_config_loading ===\n" << std::endl;

  // 1. Load configs from YAML presets
  auto config_local = loadConfig(EXAMPLE_CONFIG_DIR "/default.yaml");
  auto config_global = loadConfig(EXAMPLE_CONFIG_DIR "/global.yaml");

  std::cout << "Loaded LOCAL preset (default.yaml)" << std::endl;
  std::cout << "Loaded GLOBAL preset (global.yaml)\n" << std::endl;

  // 2. Create maps
  ElevationMap map_local;
  map_local.setGeometry(10.0f, 10.0f, 0.1f);
  map_local.setFrameId("map");

  ElevationMap map_global;
  map_global.setGeometry(20.0f, 20.0f, 0.1f);
  map_global.setFrameId("map");

  // 3. Create FastDEM instances with loaded configs
  FastDEM mapper_local(map_local, config_local.core);
  FastDEM mapper_global(map_global, config_global.core);

  // 4. Generate identical test data
  auto cloud = examples::generateTerrainCloud(50000, 8.0f);
  std::cout << "Generated " << cloud.size() << " points\n" << std::endl;

  // 5. Integrate into both mappers
  const auto I = Eigen::Isometry3d::Identity();

  examples::Timer timer;

  timer.start();
  mapper_local.integrate(cloud, I, I);
  timer.printElapsed("Local integration");

  timer.start();
  mapper_global.integrate(cloud, I, I);
  timer.printElapsed("Global integration");

  // 6. Compare results
  examples::printMapStats(map_local, "Local Mode");
  examples::printMapStats(map_global, "Global Mode");

  // 7. Save visualizations
  examples::saveMapImage(map_local,
                         std::string(EXAMPLE_OUTPUT_DIR) + "/local.png");
  examples::saveMapImage(map_global,
                         std::string(EXAMPLE_OUTPUT_DIR) + "/global.png");

  return 0;
}
