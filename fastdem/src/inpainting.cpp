// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

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

#include "fastdem/postprocess/inpainting.hpp"

#include <cmath>

namespace fastdem {

namespace {

// 8-connected neighbor offsets
constexpr int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
constexpr int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

float computeNeighborMean(const Eigen::MatrixXf& elevation, int lr, int lc,
                          int rows, int cols, int sr, int sc, int min_valid,
                          int& valid_count) {
  float sum = 0.0f;
  valid_count = 0;

  for (int i = 0; i < 8; ++i) {
    int nlr = lr + dy[i];
    int nlc = lc + dx[i];

    // Bounds check in logical (spatial) space
    if (nlr < 0 || nlr >= rows || nlc < 0 || nlc >= cols) continue;

    // Logical → buffer index
    int nr = nlr + sr;
    if (nr >= rows) nr -= rows;
    int nc = nlc + sc;
    if (nc >= cols) nc -= cols;

    float val = elevation(nr, nc);
    if (std::isfinite(val)) {
      sum += val;
      valid_count++;
    }
  }

  if (valid_count >= min_valid) {
    return sum / static_cast<float>(valid_count);
  }
  return NAN;
}

}  // namespace

void applyInpainting(ElevationMap& map, const config::Inpainting& config) {
  if (!config.enabled) return;

  // Ensure inpainted layer exists
  if (!map.exists(layer::elevation_inpainted)) {
    map.add(layer::elevation_inpainted, NAN);
    map.setBasicLayers({layer::elevation_inpainted});
  }

  // Copy elevation to inpainted layer
  const auto& elevation = map.get(layer::elevation);
  auto& inpainted = map.get(layer::elevation_inpainted);
  inpainted = elevation;

  const int rows = inpainted.rows();
  const int cols = inpainted.cols();
  const auto& startIdx = map.getStartIndex();
  const int sr = startIdx(0);
  const int sc = startIdx(1);

  // Iterative inpainting
  Eigen::MatrixXf buffer(rows, cols);
  for (int iter = 0; iter < config.max_iterations; ++iter) {
    bool changed = false;
    buffer = inpainted;

    // Iterate in logical (spatial) index space
    for (int lr = 0; lr < rows; ++lr) {
      for (int lc = 0; lc < cols; ++lc) {
        // Logical → buffer index
        int r = lr + sr;
        if (r >= rows) r -= rows;
        int c = lc + sc;
        if (c >= cols) c -= cols;

        if (!std::isnan(inpainted(r, c))) {
          continue;
        }

        int valid_count = 0;
        float mean =
            computeNeighborMean(inpainted, lr, lc, rows, cols, sr, sc,
                                config.min_valid_neighbors, valid_count);

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

}  // namespace fastdem
