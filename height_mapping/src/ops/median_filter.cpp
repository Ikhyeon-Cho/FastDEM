/*
 * median_filter.cpp
 *
 * Median filter for removing spike noise from height map layers.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ops/median_filter.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace height_mapping::ops {

namespace {

void filterLayer(HeightMap& map, const std::string& layer_name, int kernel_size,
                 int min_valid_neighbors) {
  if (!map.exists(layer_name)) return;

  const Eigen::MatrixXf input = map.get(layer_name);  // Copy (double buffer)
  auto& output = map.get(layer_name);                 // Reference (in-place)

  const int half = kernel_size / 2;
  const int rows = input.rows();
  const int cols = input.cols();

  std::vector<float> window;
  window.reserve(kernel_size * kernel_size);

  for (int r = half; r < rows - half; ++r) {
    for (int c = half; c < cols - half; ++c) {
      if (!std::isfinite(input(r, c))) continue;

      window.clear();
      for (int dr = -half; dr <= half; ++dr) {
        for (int dc = -half; dc <= half; ++dc) {
          float val = input(r + dr, c + dc);
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

}  // namespace

void applyMedianFilter(HeightMap& map, const config::MedianFilter& config) {
  if (!config.enabled) return;

  for (const auto& layer_name : config.target_layers) {
    filterLayer(map, layer_name, config.kernel_size,
                config.min_valid_neighbors);
  }
}

}  // namespace height_mapping::ops
