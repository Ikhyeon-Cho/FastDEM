// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * rasterization.hpp
 *
 * Rasterization configuration.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_RASTERIZATION_HPP
#define FASTDEM_CONFIG_RASTERIZATION_HPP

namespace fastdem {

enum class RasterMethod {
  Max,
  Min,
  Mean,
};

namespace config {

struct Rasterization {
  RasterMethod method = RasterMethod::Max;
};

}  // namespace config
}  // namespace fastdem

#endif  // FASTDEM_CONFIG_RASTERIZATION_HPP
