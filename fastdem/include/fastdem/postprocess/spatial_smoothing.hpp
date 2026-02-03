// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * spatial_smoothing.hpp
 *
 * Spatial median smoothing for grid map layers.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_POSTPROCESS_SPATIAL_SMOOTHING_HPP
#define FASTDEM_POSTPROCESS_SPATIAL_SMOOTHING_HPP

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Apply spatial median smoothing to a map layer (in-place).
 *
 * Replaces each cell value with the median of its kernel neighborhood,
 * removing spike noise while preserving edges.
 *
 * @param map ElevationMap to process
 * @param layer_name Target layer name
 * @param kernel_size Median filter kernel size (default: 3)
 * @param min_valid_neighbors Minimum valid neighbors for filtering (default: 5)
 */
inline void applySpatialSmoothing(ElevationMap& map,
                                  const std::string& layer_name,
                                  int kernel_size = 3,
                                  int min_valid_neighbors = 5) {
  if (!map.exists(layer_name)) return;

  const Eigen::MatrixXf input = map.get(layer_name);  // Copy (double buffer)
  auto& output = map.get(layer_name);                  // Reference (in-place)

  const int half = kernel_size / 2;
  const int rows = input.rows();
  const int cols = input.cols();
  const auto& startIdx = map.getStartIndex();
  const int sr = startIdx(0);
  const int sc = startIdx(1);

  std::vector<float> window;
  window.reserve(kernel_size * kernel_size);

  // Iterate in logical (spatial) index space
  for (int lr = 0; lr < rows; ++lr) {
    for (int lc = 0; lc < cols; ++lc) {
      // Logical → buffer index
      int r = lr + sr;
      if (r >= rows) r -= rows;
      int c = lc + sc;
      if (c >= cols) c -= cols;

      if (!std::isfinite(input(r, c))) continue;

      window.clear();
      for (int dr = -half; dr <= half; ++dr) {
        for (int dc = -half; dc <= half; ++dc) {
          int nlr = lr + dr;
          int nlc = lc + dc;

          // Bounds check in logical space
          if (nlr < 0 || nlr >= rows || nlc < 0 || nlc >= cols) continue;

          // Logical → buffer index
          int nr = nlr + sr;
          if (nr >= rows) nr -= rows;
          int nc = nlc + sc;
          if (nc >= cols) nc -= cols;

          float val = input(nr, nc);
          if (std::isfinite(val)) {
            window.push_back(val);
          }
        }
      }

      if (static_cast<int>(window.size()) < min_valid_neighbors) continue;

      size_t mid = window.size() / 2;
      std::nth_element(window.begin(), window.begin() + mid, window.end());
      output(r, c) = window[mid];
    }
  }
}

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_SPATIAL_SMOOTHING_HPP
