// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * pcd2dem — Convert a PCD point cloud map to a clean elevation map.
 *
 * Pipeline: SOR → histogram filter → rasterize → inpaint
 *
 * Usage:
 *   ./pcd2dem input.pcd output.pcd [resolution]
 *
 * Example:
 *   ./pcd2dem campus_map.pcd campus_dem.pcd 0.1
 */

#include <fastdem/io/pcd_convert.hpp>
#include <iostream>
#include <nanopcl/io/pcd_io.hpp>
#include <string>

using namespace fastdem;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: pcd2dem <input.pcd> <output.pcd> [resolution]\n"
              << "  resolution: grid cell size in meters (default: 0.1)\n";
    return 1;
  }

  const std::string input_path = argv[1];
  const std::string output_path = argv[2];

  DEMConfig config;
  if (argc >= 4) config.resolution = std::stof(argv[3]);

  // Load
  std::cout << "Loading " << input_path << " ..." << std::endl;
  auto cloud = nanopcl::io::loadPCD(input_path);
  std::cout << "  " << cloud.size() << " points" << std::endl;

  // Build DEM (SOR → histogram filter → rasterize → inpaint)
  std::cout << "Building DEM (resolution=" << config.resolution << "m) ..."
            << std::endl;
  auto dem = buildDEM(cloud, config);

  const auto& size = dem.getSize();
  std::cout << "  Grid: " << size(0) << " x " << size(1) << " cells"
            << std::endl;

  // Export
  auto output = toPointCloud(dem);
  std::cout << "  " << output.size() << " elevation cells" << std::endl;

  nanopcl::io::savePCD(output_path, output);
  std::cout << "Saved to " << output_path << std::endl;

  return 0;
}
