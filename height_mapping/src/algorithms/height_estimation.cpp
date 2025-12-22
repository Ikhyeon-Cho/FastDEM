/*
 * height_estimation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/algorithms/algorithms.h"

#include <cmath>

namespace height_mapping::algorithms {

void updateHeightMap(HeightMap& map, const PointCloud& cloud,
                     estimators::HeightEstimatorBase& estimator) {
  if (cloud.empty()) {
    return;
  }

  // Ensure core layers exist
  if (!map.exists(layer::elevation)) map.add(layer::elevation, NAN);
  if (!map.exists(layer::variance)) map.add(layer::variance, 0.0f);
  if (!map.exists(layer::count)) map.add(layer::count, 0.0f);

  auto& elevation_mat = map.get(layer::elevation);
  auto& variance_mat = map.get(layer::variance);
  auto& count_mat = map.get(layer::count);

  // Update each cell with new measurements
  for (const auto& point : cloud.points) {
    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), index)) {
      continue;
    }

    float& elevation = elevation_mat(index(0), index(1));
    float& variance = variance_mat(index(0), index(1));
    float& count = count_mat(index(0), index(1));

    estimator.update(elevation, variance, count, point.z());
  }

  // Batch compute derived statistics (Eigen vectorized)
  if (!map.exists(layer::standard_error)) map.add(layer::standard_error, NAN);
  if (!map.exists(layer::confidence_interval))
    map.add(layer::confidence_interval, NAN);

  auto& se_mat = map.get(layer::standard_error);
  auto& ci_mat = map.get(layer::confidence_interval);

  // standard_error = sqrt(variance / count)
  // confidence_interval_95 = 1.96 * standard_error
  se_mat = (variance_mat.array() / count_mat.array()).sqrt();
  ci_mat = 1.96f * se_mat.array();
}

}  // namespace height_mapping::algorithms