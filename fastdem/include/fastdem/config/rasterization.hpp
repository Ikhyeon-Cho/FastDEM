// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_CONFIG_RASTERIZATION_HPP
#define FASTDEM_CONFIG_RASTERIZATION_HPP

namespace fastdem {

/// Per-cell elevation selection method.
enum class RasterMethod {
  Max,
  Min,
  Mean,
  MinMax,  ///< Emit both min and max (for dual-layer estimation)
};

namespace config {

struct Rasterization {
  RasterMethod method = RasterMethod::Max;
};

}  // namespace config
}  // namespace fastdem

#endif  // FASTDEM_CONFIG_RASTERIZATION_HPP
