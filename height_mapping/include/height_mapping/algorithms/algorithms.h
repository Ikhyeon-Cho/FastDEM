/*
 * algorithms.h
 *
 * Algorithm function declarations for height mapping.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ALGORITHMS_H
#define HEIGHT_MAPPING_ALGORITHMS_H

#include <Eigen/Core>

#include "height_mapping/algorithms/configs.h"
#include "height_mapping/estimators/height_estimator_base.h"
#include "height_mapping/types.h"

namespace height_mapping::algorithms {

/**
 * @brief Raycasting for ghost obstacle removal and persistence management.
 * Handles both Hit (observation) and Miss (ray pass-through) logic.
 */
void applyRaycasting(HeightMap& map, const PointCloud& scan,
                     const Eigen::Vector3f& sensor_origin,
                     const RaycastingConfig& config);

/**
 * @brief Updates the height map using point cloud data and a specific
 * estimator. Pure height estimation only.
 */
void updateHeightMap(HeightMap& map, const PointCloud& cloud,
                     estimators::HeightEstimatorBase& estimator);

/**
 * @brief Fills NaN holes in elevation using neighbor averaging.
 * Results stored in elevation_inpainted layer (original unchanged).
 */
void applyInpainting(HeightMap& map, const InpaintingConfig& config);

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_ALGORITHMS_H
