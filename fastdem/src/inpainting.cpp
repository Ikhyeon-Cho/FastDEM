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

  // 8-connected neighborhood (3x3 excluding center)
  const auto reg8 = map.region(nanogrid::Size(3, 3));

  const auto size = map.getSize();
  Eigen::MatrixXf buffer(size(0), size(1));

  for (int iter = 0; iter < max_iterations; ++iter) {
    bool changed = false;
    buffer = inpainted;

    for (auto cell : map.cells()) {
      if (!std::isnan(inpainted(cell.index))) continue;

      float sum = 0.0f;
      int count = 0;
      for (auto n : map.neighbors(cell, reg8)) {
        if (n.row == cell.row && n.col == cell.col) continue;
        float val = inpainted(n.index);
        if (std::isfinite(val)) {
          sum += val;
          ++count;
        }
      }

      if (count >= min_valid_neighbors) {
        buffer(cell.index) = sum / static_cast<float>(count);
        changed = true;
      }
    }

    inpainted = buffer;
    if (!changed) break;
  }
}

}  // namespace fastdem
