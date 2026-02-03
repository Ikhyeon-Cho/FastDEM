// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * png.hpp
 *
 * PNG image export for ElevationMap visualization.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_IO_PNG_HPP
#define FASTDEM_IO_PNG_HPP

#include <string>

#include "fastdem/elevation_map.hpp"

namespace fastdem {
namespace io {

/// Configuration for PNG image export.
struct PngExportConfig {
  enum class Normalize { MIN_MAX, PERCENTILE_1_99, FIXED_RANGE };
  enum class Colormap { GRAYSCALE, VIRIDIS, JET };

  Normalize normalize = Normalize::PERCENTILE_1_99;
  Colormap colormap = Colormap::VIRIDIS;
  bool align_to_world = true;  // Unroll circular buffer
  float fixed_min = -2.0f;     // For FIXED_RANGE
  float fixed_max = 2.0f;
};

/// Export ElevationMap layer as PNG image with colormap.
bool savePng(const std::string& filename, const ElevationMap& map,
             const std::string& layer_name, const PngExportConfig& config = {});

}  // namespace io
}  // namespace fastdem

#endif  // FASTDEM_IO_PNG_HPP
