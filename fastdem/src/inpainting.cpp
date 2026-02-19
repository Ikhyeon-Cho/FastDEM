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

void applyInpainting(ElevationMap& map, int max_iterations,
                     int min_valid_neighbors, bool inplace) {
  const char* output = inplace ? layer::elevation : layer::elevation_inpainted;

  // Ensure output layer exists
  if (!map.exists(output)) {
    map.add(output, NAN);
  }

  // Copy elevation to output layer (no-op when inplace)
  auto& inpainted = map.get(output);
  if (!inplace) inpainted = map.get(layer::elevation);

  // 8-connected neighbor offsets
  constexpr int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  constexpr int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};

  const auto idx = map.indexer();
  Eigen::MatrixXf buffer(idx.rows, idx.cols);

  for (int iter = 0; iter < max_iterations; ++iter) {
    bool changed = false;
    buffer = inpainted;

    for (int row = 0; row < idx.rows; ++row) {
      for (int col = 0; col < idx.cols; ++col) {
        auto [r, c] = idx(row, col);
        if (!std::isnan(inpainted(r, c))) continue;

        float sum = 0.0f;
        int count = 0;
        for (int i = 0; i < 8; ++i) {
          if (!idx.contains(row + dr[i], col + dc[i])) continue;
          auto [nr, nc] = idx(row + dr[i], col + dc[i]);
          float val = inpainted(nr, nc);
          if (std::isfinite(val)) {
            sum += val;
            ++count;
          }
        }

        if (count >= min_valid_neighbors) {
          buffer(r, c) = sum / static_cast<float>(count);
          changed = true;
        }
      }
    }

    inpainted = buffer;
    if (!changed) break;
  }
}

}  // namespace fastdem
