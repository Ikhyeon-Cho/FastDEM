/*
 * inpainting.cpp
 *
 * Iterative neighbor averaging for filling NaN holes in height maps.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <cmath>

#include "height_mapping/algorithms/algorithms.h"

namespace height_mapping::algorithms {

namespace {

// 8-connected neighbor offsets
constexpr int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
constexpr int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

float computeNeighborMean(const Eigen::MatrixXf& elevation,
                          const Eigen::MatrixXf& count, int row, int col,
                          int min_valid, int& valid_count) {
  const int rows = elevation.rows();
  const int cols = elevation.cols();

  float sum = 0.0f;
  valid_count = 0;

  for (int i = 0; i < 8; ++i) {
    int nr = row + dy[i];
    int nc = col + dx[i];

    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
      float val = elevation(nr, nc);
      // Only use cells that have been observed (count > 0)
      if (std::isfinite(val) && count(nr, nc) > 0) {
        sum += val;
        valid_count++;
      }
    }
  }

  if (valid_count >= min_valid) {
    return sum / static_cast<float>(valid_count);
  }
  return NAN;
}

}  // namespace

void applyInpainting(HeightMap& map, const InpaintingConfig& config) {
  if (!config.enabled) {
    return;
  }

  // Ensure inpainted layer exists
  if (!map.exists(layer::elevation_inpainted)) {
    map.add(layer::elevation_inpainted, NAN);
  }

  // Copy elevation to inpainted layer
  const auto& elevation = map.get(layer::elevation);
  const auto& count_mat = map.get(layer::count);
  auto& inpainted = map.get(layer::elevation_inpainted);
  inpainted = elevation;

  const int rows = inpainted.rows();
  const int cols = inpainted.cols();

  // Iterative inpainting
  for (int iter = 0; iter < config.max_iterations; ++iter) {
    bool changed = false;
    Eigen::MatrixXf buffer = inpainted;

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (!std::isnan(inpainted(r, c))) {
          continue;
        }

        int valid_count = 0;
        float mean = computeNeighborMean(
            inpainted, count_mat, r, c, config.min_valid_neighbors, valid_count);

        if (std::isfinite(mean)) {
          buffer(r, c) = mean;
          changed = true;
        }
      }
    }

    inpainted = buffer;

    if (!changed) {
      break;
    }
  }
}

}  // namespace height_mapping::algorithms
