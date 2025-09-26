/*
 * height_estimation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/algorithms/height_estimation.h"
#include "height_mapping_core/estimators/incremental_mean.h"
#include "height_mapping_core/estimators/kalman_filter.h"
#include "height_mapping_core/estimators/moving_average.h"
#include <cmath>

namespace height_mapping::algorithms {

HeightEstimation::HeightEstimation() : config_{} {
  config_.validate();

  // Create default estimator
  switch (config_.estimator_type) {
    case Config::EstimatorType::KALMAN_FILTER: {
      estimators::KalmanFilter::Parameters params;
      params.process_noise = config_.kalman_process_noise;
      params.measurement_noise = config_.kalman_measurement_noise;
      params.initial_variance = config_.kalman_initial_variance;
      estimator_ = std::make_unique<estimators::KalmanFilter>(params);
      break;
    }
    case Config::EstimatorType::MOVING_AVERAGE: {
      estimators::MovingAverage::Parameters params;
      params.alpha = config_.moving_average_alpha;
      estimator_ = std::make_unique<estimators::MovingAverage>(params);
      break;
    }
    case Config::EstimatorType::INCREMENTAL_MEAN:
    default:
      estimator_ = std::make_unique<estimators::IncrementalMean>();
      break;
  }
}

HeightEstimation::HeightEstimation(const Config &config) : config_(config) {
  config_.validate();

  // Create estimator based on type
  switch (config_.estimator_type) {
    case Config::EstimatorType::KALMAN_FILTER: {
      estimators::KalmanFilter::Parameters params;
      params.process_noise = config_.kalman_process_noise;
      params.measurement_noise = config_.kalman_measurement_noise;
      params.initial_variance = config_.kalman_initial_variance;
      estimator_ = std::make_unique<estimators::KalmanFilter>(params);
      break;
    }
    case Config::EstimatorType::MOVING_AVERAGE: {
      estimators::MovingAverage::Parameters params;
      params.alpha = config_.moving_average_alpha;
      estimator_ = std::make_unique<estimators::MovingAverage>(params);
      break;
    }
    case Config::EstimatorType::INCREMENTAL_MEAN:
    default:
      estimator_ = std::make_unique<estimators::IncrementalMean>();
      break;
  }
}

void HeightEstimation::setConfig(const Config &config) {
  config_ = config;
  config_.validate();

  // Recreate estimator with new config
  switch (config_.estimator_type) {
    case Config::EstimatorType::KALMAN_FILTER: {
      estimators::KalmanFilter::Parameters params;
      params.process_noise = config_.kalman_process_noise;
      params.measurement_noise = config_.kalman_measurement_noise;
      params.initial_variance = config_.kalman_initial_variance;
      estimator_ = std::make_unique<estimators::KalmanFilter>(params);
      break;
    }
    case Config::EstimatorType::MOVING_AVERAGE: {
      estimators::MovingAverage::Parameters params;
      params.alpha = config_.moving_average_alpha;
      estimator_ = std::make_unique<estimators::MovingAverage>(params);
      break;
    }
    case Config::EstimatorType::INCREMENTAL_MEAN:
    default:
      estimator_ = std::make_unique<estimators::IncrementalMean>();
      break;
  }
}

HeightEstimation::Stats
HeightEstimation::updateMap(height_mapping::HeightMap &map,
                           const height_mapping::PointCloud &cloud) const {

  Stats stats;

  if (cloud.empty() || !estimator_) {
    return stats;
  }

  // Update map timestamp with cloud timestamp
  map.setTimestamp(cloud.timestamp());

  // Check for sensor data and create layers if needed
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  namespace layer = height_mapping::layer;

  if (has_intensity && !map.exists(layer::intensity)) {
    map.add(layer::intensity, 0.0f);
  }

  if (has_color) {
    if (!map.exists(layer::color_r)) {
      map.add(layer::color_r, 0.0f);
      map.add(layer::color_g, 0.0f);
      map.add(layer::color_b, 0.0f);
    }
  }

  // Pre-load layer matrices once for performance (critical optimization)
  auto &elevation = map[layer::elevation];
  auto &variance = map[layer::variance];
  auto &elevation_min = map[layer::elevation_min];
  auto &elevation_max = map[layer::elevation_max];
  auto &count = map[layer::count];

  // Get statistical quality layers
  auto &standard_error = map.getStandardError();
  auto &confidence_interval = map.getConfidenceInterval();

  grid_map::Index index;
  grid_map::Position position;

  for (const auto point : cloud) {
    stats.points_processed++;

    position << point.x(), point.y();

    // Skip if point is outside map bounds
    if (!map.getIndex(position, index)) {
      stats.points_out_of_bounds++;
      continue;
    }

    const auto &new_elevation = point.z();

    // Use helper function to access matrix with single index
    const auto i = index(0);
    const auto j = index(1);

    // Direct matrix access for performance
    float &cell_elevation = elevation(i, j);
    float &cell_variance = variance(i, j);
    float &cell_min = elevation_min(i, j);
    float &cell_max = elevation_max(i, j);
    float &cell_count = count(i, j);

    // Use the estimator to update elevation, variance, and count
    estimator_->update(cell_elevation, cell_variance, cell_count,
                       new_elevation);

    // Always update min/max regardless of algorithm
    cell_min = std::fmin(cell_min, new_elevation);
    cell_max = std::fmax(cell_max, new_elevation);

    // Compute statistical quality measures
    if (cell_count > 1) {
      // Standard Error = σ / √n
      standard_error(i, j) = std::sqrt(cell_variance / cell_count);

      // 95% Confidence Interval = 1.96 * SE
      confidence_interval(i, j) = 1.96f * standard_error(i, j);
    } else {
      standard_error(i, j) = NAN;
      confidence_interval(i, j) = NAN;
    }

    // Lambda for updating sensor data with running average
    auto updateSensorValue = [cell_count](float &current_value,
                                          float new_value) {
      if (std::isnan(current_value)) {
        current_value = new_value;
      } else {
        current_value =
            (current_value * (cell_count - 1) + new_value) / cell_count;
      }
    };

    // Update sensor data layers if present using PointView
    if (point.hasIntensity()) {
      auto &intensity_layer = map[layer::intensity];
      updateSensorValue(intensity_layer(i, j), point.intensity());
    }

    if (point.hasColor()) {
      const auto color = point.color();
      updateSensorValue(map[layer::color_r](i, j), color[0]);
      updateSensorValue(map[layer::color_g](i, j), color[1]);
      updateSensorValue(map[layer::color_b](i, j), color[2]);
    }

    stats.cells_updated++;
  }

  return stats;
}

} // namespace height_mapping::algorithms