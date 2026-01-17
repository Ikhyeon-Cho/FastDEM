/*
 * raycasting.h
 *
 * Raycasting for ghost obstacle removal and persistence management.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_OPS_RAYCASTING_H
#define HEIGHT_MAPPING_OPS_RAYCASTING_H

#include <Eigen/Core>

#include "height_mapping/config/types.h"
#include "height_mapping/core.h"

namespace height_mapping::ops {

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
void applyRaycasting(HeightMap& map, const PointCloud& scan,
                     const Eigen::Vector3f& sensor_origin,
                     const config::Raycasting& config);

}  // namespace height_mapping::ops

#endif  // HEIGHT_MAPPING_OPS_RAYCASTING_H
