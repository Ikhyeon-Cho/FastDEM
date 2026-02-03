// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * raycasting.hpp
 *
 * Raycasting configuration for ghost obstacle removal.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_RAYCASTING_HPP
#define FASTDEM_CONFIG_RAYCASTING_HPP

namespace fastdem::config {

/**
 * @brief Raycasting configuration for ghost removal.
 *
 * Uses temporal voting to remove ghost obstacles:
 * - Uses all downward rays (point.z < sensor.z) for maximum coverage
 * - Computes raycasting_upper_bound: maximum ray height at each cell
 * - Conflict: elevation > raycasting_upper_bound + ray_height_margin
 * - Clear cell when conflict_count >= vote_threshold
 *
 * Using maximum ray height ensures conservative ghost detection:
 * "All rays passed below the recorded elevation" = confirmed ghost
 */
struct Raycasting {
  bool enabled = false;

  // Endpoint handling
  int endpoint_margin = 2;  ///< Skip N cells before target (protects edges)

  // Conflict detection
  float ray_height_margin = 0.05f;  ///< elevation > upper_bound + margin = conflict

  // Dynamic obstacle filtering
  float dynamic_height_threshold = 0.5f;  ///< Only check cells where elevation_max - elevation_min > threshold

  // Temporal voting
  int vote_threshold = 10;  ///< Clear cell after N consecutive conflicts
};

}  // namespace fastdem::config

#endif  // FASTDEM_CONFIG_RAYCASTING_HPP
