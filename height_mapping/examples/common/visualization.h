/*
 * visualization.h
 *
 * Visualization utilities for examples.
 */

#ifndef EXAMPLES_COMMON_VISUALIZATION_H
#define EXAMPLES_COMMON_VISUALIZATION_H

#include <height_mapping/core/layers.h>
#include <height_mapping/core/map.h>
#include <height_mapping/io/ppm_io.h>

#include <cmath>
#include <iomanip>
#include <iostream>

namespace examples {

/**
 * @brief Print HeightMap statistics to console.
 *
 * @param map HeightMap to analyze
 * @param name Optional name for the map
 */
inline void printMapStats(const height_mapping::HeightMap& map,
                          const std::string& name = "HeightMap") {
  const auto& size = map.getSize();
  const auto& resolution = map.getResolution();
  const auto& length = map.getLength();

  std::cout << "\n=== " << name << " Statistics ===" << std::endl;
  std::cout << "Grid size:   " << size(0) << " x " << size(1) << " cells"
            << std::endl;
  std::cout << "Resolution:  " << resolution << " m/cell" << std::endl;
  std::cout << "Physical:    " << length(0) << " x " << length(1) << " m"
            << std::endl;
  std::cout << "Frame:       " << map.getFrameId() << std::endl;

  // Compute coverage and elevation statistics
  const auto& elevation = map.get(height_mapping::layer::elevation);
  int valid_count = 0;
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  double sum_z = 0.0;

  for (int i = 0; i < elevation.size(); ++i) {
    float val = elevation.data()[i];
    if (std::isfinite(val)) {
      valid_count++;
      min_z = std::min(min_z, val);
      max_z = std::max(max_z, val);
      sum_z += val;
    }
  }

  int total_cells = size(0) * size(1);
  float coverage = static_cast<float>(valid_count) / total_cells * 100.0f;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Coverage:    " << coverage << "% (" << valid_count << "/"
            << total_cells << " cells)" << std::endl;

  if (valid_count > 0) {
    float mean_z = static_cast<float>(sum_z / valid_count);
    std::cout << "Elevation:   [" << min_z << ", " << max_z << "] m"
              << std::endl;
    std::cout << "Mean elev:   " << mean_z << " m" << std::endl;
  }
  std::cout << std::endl;
}

/**
 * @brief Save HeightMap as PPM image.
 *
 * @param map HeightMap to export
 * @param filename Output filename (should end with .ppm)
 * @param layer Layer to export (default: elevation)
 * @return true if successful
 */
inline bool saveMapImage(const height_mapping::HeightMap& map,
                         const std::string& filename,
                         const std::string& layer = "elevation") {
  height_mapping::io::PpmExportConfig config;
  config.colormap = height_mapping::io::PpmExportConfig::Colormap::VIRIDIS;
  config.normalize =
      height_mapping::io::PpmExportConfig::Normalize::PERCENTILE_1_99;

  bool success = height_mapping::io::savePpm(filename, map, layer, config);
  if (success) {
    std::cout << "Saved: " << filename << std::endl;
  } else {
    std::cerr << "Failed to save: " << filename << std::endl;
  }
  return success;
}

/**
 * @brief Print a simple ASCII visualization of the map coverage.
 *
 * @param map HeightMap to visualize
 * @param max_width Maximum width in characters
 */
inline void printAsciiMap(const height_mapping::HeightMap& map,
                          int max_width = 40) {
  const auto& elevation = map.get(height_mapping::layer::elevation);
  int rows = static_cast<int>(elevation.rows());
  int cols = static_cast<int>(elevation.cols());

  // Compute step size for downsampling
  int step = std::max(1, std::max(rows, cols) / max_width);
  int display_rows = (rows + step - 1) / step;
  int display_cols = (cols + step - 1) / step;

  // Find min/max for normalization
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  for (int i = 0; i < elevation.size(); ++i) {
    float val = elevation.data()[i];
    if (std::isfinite(val)) {
      min_z = std::min(min_z, val);
      max_z = std::max(max_z, val);
    }
  }

  float range = max_z - min_z;
  if (range < 1e-6f) range = 1.0f;

  const char* levels = " .:-=+*#%@";
  int num_levels = 10;

  std::cout << "\n--- ASCII Preview ---" << std::endl;
  for (int r = 0; r < display_rows; ++r) {
    for (int c = 0; c < display_cols; ++c) {
      int src_r = r * step;
      int src_c = c * step;
      float val = elevation(src_r, src_c);

      if (!std::isfinite(val)) {
        std::cout << ' ';
      } else {
        float t = (val - min_z) / range;
        int idx = static_cast<int>(t * (num_levels - 1));
        idx = std::max(0, std::min(num_levels - 1, idx));
        std::cout << levels[idx];
      }
    }
    std::cout << std::endl;
  }
  std::cout << "---------------------\n" << std::endl;
}

}  // namespace examples

#endif  // EXAMPLES_COMMON_VISUALIZATION_H
