// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * point_filter.hpp
 *
 * Point filtering configuration (z-range and distance bounds).
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_POINT_FILTER_HPP
#define FASTDEM_CONFIG_POINT_FILTER_HPP

#include <limits>

namespace fastdem::config {

/**
 * @brief Point filtering configuration (z-range and distance bounds).
 */
struct PointFilter {
  float z_min = -std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::max();
  float range_min = 0.0f;
  float range_max = std::numeric_limits<float>::max();
};

}  // namespace fastdem::config

#endif  // FASTDEM_CONFIG_POINT_FILTER_HPP
