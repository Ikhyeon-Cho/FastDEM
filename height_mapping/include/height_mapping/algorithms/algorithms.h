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
#include "height_mapping/height_map.h"
#include "height_mapping/types.h"

namespace height_mapping::algorithms {

/**
 * @brief Clears obstacles in free space by raycasting from sensor to points.
 *
 * @param map Height map to update (modified in-place).
 * @param cloud Input point cloud (in map frame).
 * @param sensor_origin Sensor position (in map frame).
 * @param config Raycasting parameters.
 */
void applyRaycasting(HeightMap& map, const PointCloud& cloud,
                     const Eigen::Vector3f& sensor_origin,
                     const RaycastingConfig& config);

/**
 * @brief Updates the height map using point cloud data and a specific
 * estimator.
 *
 * @param map Height map to update (modified in-place).
 * @param cloud Input point cloud.
 * @param estimator The estimation algorithm to use for cell updates.
 */
void updateHeightMap(HeightMap& map, const PointCloud& cloud,
                     estimators::HeightEstimatorBase& estimator);

/**
 * @brief Segments ground points using a grid-based approach.
 *
 * @param cloud Input point cloud (modified in-place).
 * @param config Segmentation parameters.
 */
void segmentGroundGrid(PointCloud& cloud,
                       const GridGroundSegmentationConfig& config);

/**
 * @brief Segments ground points using statistical percentile analysis.
 *
 * @param cloud Input point cloud (modified in-place).
 * @param config Segmentation parameters.
 * @return Statistics from the segmentation operation.
 */
GroundSegmentationStats segmentGroundStatistical(
    PointCloud& cloud, const StatisticalGroundSegmentationConfig& config);

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_ALGORITHMS_H
