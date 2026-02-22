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

#include "fastdem/config/postprocess.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

namespace layer {
constexpr auto ghost_removal = "ghost_removal";
constexpr auto raycasting = "raycasting";
constexpr auto visibility_logodds = "_visibility_logodds";
}  // namespace layer

/**
 * @brief Raycasting for ghost obstacle removal via log-odds accumulation.
 *
 * Maintains per-cell log-odds representing confidence that recorded elevation
 * is valid. Observed cells gain confidence (+L_observed), cells where rays
 * pass below recorded elevation lose confidence (-L_ghost). Cells whose
 * log-odds fall below clear_threshold are cleared as ghosts.
 *
 * @param map Height map to update
 * @param scan Points inside map provide observed evidence; all points
 *             serve as ray targets.
 * @param sensor_origin Sensor position in map frame
 * @param config Raycasting configuration
 */
void applyRaycasting(ElevationMap& map, const PointCloud& scan,
                     const Eigen::Vector3f& sensor_origin,
                     const config::Raycasting& config);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_RAYCASTING_HPP
