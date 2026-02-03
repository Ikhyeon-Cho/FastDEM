// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * png_io.cpp
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#pragma GCC diagnostic pop

#include "fastdem/io/png.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace fastdem {
namespace io {

namespace detail {

std::pair<float, float> computeRange(const grid_map::Matrix& matrix,
                                     PngExportConfig::Normalize mode,
                                     float fixed_min, float fixed_max) {
  if (mode == PngExportConfig::Normalize::FIXED_RANGE) {
    return {fixed_min, fixed_max};
  }

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

  if (mode == PngExportConfig::Normalize::MIN_MAX) {
    auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
    return {*min_it, *max_it};
  }

  // PERCENTILE_1_99: use nth_element for O(n) instead of full sort
  size_t idx_1 = static_cast<size_t>(values.size() * 0.01);
  size_t idx_99 = std::min(static_cast<size_t>(values.size() * 0.99),
                           values.size() - 1);
  std::nth_element(values.begin(), values.begin() + idx_1, values.end());
  float p1 = values[idx_1];
  std::nth_element(values.begin(), values.begin() + idx_99, values.end());
  float p99 = values[idx_99];
  return {p1, p99};
}

void viridisLUT(float t, uint8_t& r, uint8_t& g, uint8_t& b) {
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

  r = static_cast<uint8_t>(
      (lut[i0][0] * (1 - frac) + lut[i1][0] * frac) * 255);
  g = static_cast<uint8_t>(
      (lut[i0][1] * (1 - frac) + lut[i1][1] * frac) * 255);
  b = static_cast<uint8_t>(
      (lut[i0][2] * (1 - frac) + lut[i1][2] * frac) * 255);
}

void jetColor(float t, uint8_t& r, uint8_t& g, uint8_t& b) {
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

}  // namespace detail

bool savePng(const std::string& filename, const ElevationMap& map,
             const std::string& layer_name, const PngExportConfig& config) {
  if (!map.exists(layer_name)) {
    spdlog::error("[png_io] Layer '{}' does not exist", layer_name);
    return false;
  }

  const auto& matrix = map.get(layer_name);
  int rows = static_cast<int>(matrix.rows());
  int cols = static_cast<int>(matrix.cols());

  auto [val_min, val_max] = detail::computeRange(
      matrix, config.normalize, config.fixed_min, config.fixed_max);
  float range = val_max - val_min;
  if (range < 1e-6f) range = 1.0f;

  auto start_idx = map.getStartIndex();
  int start_row = config.align_to_world ? start_idx(0) : 0;
  int start_col = config.align_to_world ? start_idx(1) : 0;

  // Build RGBA buffer (alpha=0 for NaN cells)
  constexpr int channels = 4;
  std::vector<uint8_t> pixels(rows * cols * channels);

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      int buf_r = (r + start_row) % rows;
      int buf_c = (c + start_col) % cols;

      float val = matrix(buf_r, buf_c);
      uint8_t* px = &pixels[(r * cols + c) * channels];

      if (!std::isfinite(val)) {
        px[0] = px[1] = px[2] = 0;
        px[3] = 0;
      } else {
        float t = (val - val_min) / range;
        t = std::max(0.0f, std::min(1.0f, t));

        switch (config.colormap) {
          case PngExportConfig::Colormap::VIRIDIS:
            detail::viridisLUT(t, px[0], px[1], px[2]);
            break;
          case PngExportConfig::Colormap::JET:
            detail::jetColor(t, px[0], px[1], px[2]);
            break;
          case PngExportConfig::Colormap::GRAYSCALE:
          default:
            px[0] = px[1] = px[2] = static_cast<uint8_t>(t * 255);
            break;
        }
        px[3] = 255;
      }
    }
  }

  int stride = cols * channels;
  if (!stbi_write_png(filename.c_str(), cols, rows, channels, pixels.data(),
                      stride)) {
    spdlog::error("[png_io] Write failed for {}", filename);
    return false;
  }

  return true;
}

}  // namespace io
}  // namespace fastdem
