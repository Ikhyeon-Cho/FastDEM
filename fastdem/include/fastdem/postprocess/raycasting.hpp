// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * raycasting.hpp
 *
 * Raycasting for ghost obstacle removal and persistence management.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_POSTPROCESS_RAYCASTING_HPP
#define FASTDEM_POSTPROCESS_RAYCASTING_HPP

#include <Eigen/Core>

#include "fastdem/config/raycasting.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

/**
 * @brief Raycasting for ghost obstacle removal and persistence management.
 *
 * Handles both Hit (observation) and Miss (ray pass-through) logic.
 * - Hit: Increment persistence at target cell
 * - Miss: Decrement persistence along ray path where map height > ray height
 *
 * @param map Height map to update
 * @param scan Point cloud in map frame
 * @param sensor_origin Sensor position in map frame
 * @param config Raycasting configuration
 */
void applyRaycasting(ElevationMap& map, const PointCloud& scan,
                     const Eigen::Vector3f& sensor_origin,
                     const config::Raycasting& config);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_RAYCASTING_HPP
