/*
 * ppm_io.h
 *
 * PPM image export for HeightMap visualization.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_IO_PPM_IO_H
#define HEIGHT_MAPPING_IO_PPM_IO_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "height_mapping/core/map.h"

namespace height_mapping {
namespace io {

/**
 * @brief Configuration for PPM image export
 */
struct PpmExportConfig {
  enum class Normalize { MIN_MAX, PERCENTILE_1_99, FIXED_RANGE };
  enum class Colormap { GRAYSCALE, VIRIDIS, JET };

  Normalize normalize = Normalize::PERCENTILE_1_99;
  Colormap colormap = Colormap::VIRIDIS;
  bool align_to_world = true;  // Unroll circular buffer
  float fixed_min = -2.0f;     // For FIXED_RANGE
  float fixed_max = 2.0f;
};

/**
 * @brief Export HeightMap layer as PPM image
 *
 * Features:
 * - NaN values rendered as white (255, 255, 255)
 * - Viridis/Jet colormap support
 * - Circular buffer unrolling for world-aligned output
 * - Percentile-based normalization for robust visualization
 *
 * @param filename Output file path (.ppm extension recommended)
 * @param map HeightMap containing the layer
 * @param layer_name Name of the layer to export
 * @param config Export configuration
 * @return true if successful
 */
bool savePpm(const std::string& filename, const HeightMap& map,
             const std::string& layer_name, const PpmExportConfig& config = {});

// =============================================================================
// Inline Implementation
// =============================================================================

namespace detail {

// Viridis colormap LUT (8-point with interpolation)
inline void viridisLUT(float t, uint8_t& r, uint8_t& g, uint8_t& b) {
  static const float lut[][3] = {
      {0.267f, 0.005f, 0.329f},  // 0.0
      {0.283f, 0.141f, 0.458f},  // 0.14
      {0.254f, 0.265f, 0.530f},  // 0.29
      {0.207f, 0.372f, 0.553f},  // 0.43
      {0.164f, 0.471f, 0.558f},  // 0.57
      {0.128f, 0.567f, 0.551f},  // 0.71
      {0.267f, 0.679f, 0.481f},  // 0.86
      {0.993f, 0.906f, 0.144f}   // 1.0
  };

  t = std::max(0.0f, std::min(1.0f, t));
  float idx = t * 7.0f;
  int i0 = static_cast<int>(idx);
  int i1 = std::min(i0 + 1, 7);
  float frac = idx - i0;

  r = static_cast<uint8_t>((lut[i0][0] * (1 - frac) + lut[i1][0] * frac) * 255);
  g = static_cast<uint8_t>((lut[i0][1] * (1 - frac) + lut[i1][1] * frac) * 255);
  b = static_cast<uint8_t>((lut[i0][2] * (1 - frac) + lut[i1][2] * frac) * 255);
}

inline void jetColor(float t, uint8_t& r, uint8_t& g, uint8_t& b) {
  t = std::max(0.0f, std::min(1.0f, t));

  if (t < 0.25f) {
    r = 0;
    g = static_cast<uint8_t>(4 * t * 255);
    b = 255;
  } else if (t < 0.5f) {
    r = 0;
    g = 255;
    b = static_cast<uint8_t>((1 - 4 * (t - 0.25f)) * 255);
  } else if (t < 0.75f) {
    r = static_cast<uint8_t>(4 * (t - 0.5f) * 255);
    g = 255;
    b = 0;
  } else {
    r = 255;
    g = static_cast<uint8_t>((1 - 4 * (t - 0.75f)) * 255);
    b = 0;
  }
}

inline std::pair<float, float> computeMinMax(const grid_map::Matrix& matrix,
                                             PpmExportConfig::Normalize mode,
                                             float fixed_min, float fixed_max) {
  if (mode == PpmExportConfig::Normalize::FIXED_RANGE) {
    return {fixed_min, fixed_max};
  }

  // Collect valid values
  std::vector<float> values;
  values.reserve(matrix.size());
  for (int i = 0; i < matrix.size(); ++i) {
    if (std::isfinite(matrix.data()[i])) {
      values.push_back(matrix.data()[i]);
    }
  }

  if (values.empty()) {
    return {0.0f, 1.0f};
  }

  if (mode == PpmExportConfig::Normalize::MIN_MAX) {
    auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
    return {*min_it, *max_it};
  }

  // PERCENTILE_1_99
  std::sort(values.begin(), values.end());
  size_t idx_1 = static_cast<size_t>(values.size() * 0.01);
  size_t idx_99 = static_cast<size_t>(values.size() * 0.99);
  idx_99 = std::min(idx_99, values.size() - 1);
  return {values[idx_1], values[idx_99]};
}

}  // namespace detail

inline bool savePpm(const std::string& filename, const HeightMap& map,
                    const std::string& layer_name,
                    const PpmExportConfig& config) {
  if (!map.exists(layer_name)) {
    std::cerr << "[ppm_io] Error: Layer '" << layer_name << "' does not exist"
              << std::endl;
    return false;
  }

  const auto& matrix = map.get(layer_name);
  int rows = static_cast<int>(matrix.rows());
  int cols = static_cast<int>(matrix.cols());

  // Compute normalization range
  auto [val_min, val_max] = detail::computeMinMax(
      matrix, config.normalize, config.fixed_min, config.fixed_max);
  float range = val_max - val_min;
  if (range < 1e-6f) range = 1.0f;

  // Get start index for circular buffer unrolling
  auto start_idx = map.getStartIndex();
  int start_row = config.align_to_world ? start_idx(0) : 0;
  int start_col = config.align_to_world ? start_idx(1) : 0;

  // Create PPM file
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    std::cerr << "[ppm_io] Error: Cannot create " << filename << std::endl;
    return false;
  }

  // PPM header (P6 = binary RGB)
  fs << "P6\n" << cols << " " << rows << "\n255\n";

  // Write pixel data (unrolled from circular buffer)
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      // Map from output image to circular buffer
      int buf_r = (r + start_row) % rows;
      int buf_c = (c + start_col) % cols;

      float val = matrix(buf_r, buf_c);
      uint8_t rgb[3];

      if (!std::isfinite(val)) {
        // NaN -> white
        rgb[0] = rgb[1] = rgb[2] = 255;
      } else {
        float t = (val - val_min) / range;
        t = std::max(0.0f, std::min(1.0f, t));

        switch (config.colormap) {
          case PpmExportConfig::Colormap::VIRIDIS:
            detail::viridisLUT(t, rgb[0], rgb[1], rgb[2]);
            break;
          case PpmExportConfig::Colormap::JET:
            detail::jetColor(t, rgb[0], rgb[1], rgb[2]);
            break;
          case PpmExportConfig::Colormap::GRAYSCALE:
          default:
            rgb[0] = rgb[1] = rgb[2] = static_cast<uint8_t>(t * 255);
            break;
        }
      }

      fs.write(reinterpret_cast<const char*>(rgb), 3);
    }
  }

  return true;
}

}  // namespace io
}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_IO_PPM_IO_H
