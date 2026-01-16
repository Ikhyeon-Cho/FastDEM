/*
 * 01_hello_mapping - Basic Mapper usage
 *
 * This example demonstrates the fundamental usage of height_mapping:
 * - Creating a Mapper with configuration
 * - Generating synthetic terrain data
 * - Integrating point clouds into the height map
 * - Accessing and visualizing the results
 */

#include <height_mapping/height_mapping.h>

#include <iostream>

#include "data_loader.h"
#include "timer.h"
#include "visualization.h"

int main() {
  std::cout << "=== 01_hello_mapping ===" << std::endl;
  std::cout << "Basic Mapper usage example\n" << std::endl;

  // -------------------------------------------------------------------------
  // 1. Create configuration
  // -------------------------------------------------------------------------
  height_mapping::MappingConfig config;

  // Map geometry
  config.map.width = 10.0f;      // 10 meters wide
  config.map.height = 10.0f;     // 10 meters tall
  config.map.resolution = 0.1f;  // 10 cm per cell
  config.map.frame_id = "map";
  config.map.mode = height_mapping::config::MappingMode::LOCAL;

  // Voxel filter (optional preprocessing)
  config.voxel_filter.voxel_size = 0.05f;

  // Height estimation algorithm
  config.estimation.type = "incremental_mean";

  std::cout << "Configuration:" << std::endl;
  std::cout << "  Map size:    " << config.map.width << " x "
            << config.map.height << " m" << std::endl;
  std::cout << "  Resolution:  " << config.map.resolution << " m/cell"
            << std::endl;
  std::cout << "  Estimator:   " << config.estimation.type << std::endl;
  std::cout << std::endl;

  // -------------------------------------------------------------------------
  // 2. Create Mapper
  // -------------------------------------------------------------------------
  height_mapping::Mapper session(config);
  std::cout << "Created Mapper" << std::endl;

  // -------------------------------------------------------------------------
  // 3. Generate synthetic terrain data
  // -------------------------------------------------------------------------
  std::cout << "Generating synthetic terrain..." << std::endl;
  auto cloud = examples::generateTerrainCloud(50000, 8.0f);
  cloud.setFrameId("sensor");  // Set sensor frame for transform validation
  std::cout << "Generated " << cloud.size() << " points" << std::endl;

  // -------------------------------------------------------------------------
  // 4. Integrate point cloud into height map
  // -------------------------------------------------------------------------
  // Define transforms (identity for this simple example)
  // T_base_sensor: Transform from sensor to base (parent=base_link,
  // child=sensor) T_map_base: Transform from base to map (parent=map,
  // child=base_link)
  npcl::Transformf T_base_sensor("base_link", "sensor");  // sensor -> base_link
  npcl::Transformf T_map_base("map", "base_link");        // base_link -> map

  examples::Timer timer;
  timer.start();

  session.integrate(cloud, T_base_sensor, T_map_base);

  timer.printElapsed("Integration");

  // -------------------------------------------------------------------------
  // 5. Access and visualize results
  // -------------------------------------------------------------------------
  const auto& map = session.map();

  // Print statistics
  examples::printMapStats(map);

  // Print ASCII preview
  examples::printAsciiMap(map, 50);

  // Save as PPM image
  std::string output_path = std::string(EXAMPLE_OUTPUT_DIR) + "/output.ppm";
  examples::saveMapImage(map, output_path);

  // -------------------------------------------------------------------------
  // 6. Direct cell access example
  // -------------------------------------------------------------------------
  std::cout << "--- Direct Cell Access ---" << std::endl;
  float elevation;
  if (map.getElevation(0.0f, 0.0f, elevation)) {
    std::cout << "Elevation at origin (0,0): " << elevation << " m"
              << std::endl;
  } else {
    std::cout << "Origin (0,0) is unmeasured" << std::endl;
  }

  if (map.getElevation(2.0f, 2.0f, elevation)) {
    std::cout << "Elevation at (2,2): " << elevation << " m" << std::endl;
  }

  std::cout << "\nDone!" << std::endl;
  return 0;
}
