/*
 * height_estimation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/height_estimation.h"
#include "height_mapping_core/estimators/estimator_factory.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <logger/logger.h>

namespace height_mapping::core::stages {

constexpr const char *STAGE_NAME = "HeightEstimation";

HeightEstimation::HeightEstimation() : Stage(STAGE_NAME) {
  // Create default estimator using factory
  std::map<std::string, std::string> empty_params;
  estimator_ =
      estimators::EstimatorFactory::create("incremental_mean", empty_params);
}

void HeightEstimation::configure(
    const std::map<std::string, std::string> &params) {
  // Extract estimator type, default to "incremental_mean" if not specified
  auto it = params.find("estimator_type");
  estimator_type_ = (it != params.end()) ? it->second : "incremental_mean";

  // Use factory to create the estimator with all relevant parameters
  estimator_ = estimators::EstimatorFactory::create(estimator_type_, params);

  if (!estimator_) {
    LOG_ERROR(STAGE_NAME, "Failed to create estimator, using default");
    estimator_ =
        estimators::EstimatorFactory::create("incremental_mean", params);
  }

  // Log configuration
  LOG_DEBUG(STAGE_NAME, "Configured with ", estimator_->name(), " estimator");
}

void HeightEstimation::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);

  // Access the height map from context
  auto &map = mapping_ctx.map();
  auto &cloud = mapping_ctx.cloud();

  if (cloud.empty()) {
    return;
  }
  if (!estimator_) {
    LOG_ERROR(STAGE_NAME, "No estimator configured");
    return;
  }

  // Pre-load layer matrices once for performance
  namespace layer = height_map::layer;
  auto &elevation = map[layer::elevation];
  auto &variance = map[layer::variance];
  auto &elevation_min = map[layer::elevation_min];
  auto &elevation_max = map[layer::elevation_max];
  auto &count = map[layer::count];

  // Get statistical quality layers
  auto &standard_error = map.getStandardError();
  auto &confidence_interval = map.getConfidenceInterval();

  // Update map timestamp with cloud timestamp
  map.setTimestamp(cloud.timestamp());

  // Check for sensor data and create layers if needed
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

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

  grid_map::Index index;
  grid_map::Position position;

  for (const auto point : cloud) { // PointView by value (not reference)
    position << point.x(), point.y();
    // Skip if point is outside map bounds
    if (!map.getIndex(position, index))
      continue;

    const auto &new_elevation = point.z();

    // Use helper function to access matrix with single index
    const auto i = index(0);
    const auto j = index(1);

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
      const auto new_color = point.color();
      updateSensorValue(map[layer::color_r](i, j), new_color[0]);
      updateSensorValue(map[layer::color_g](i, j), new_color[1]);
      updateSensorValue(map[layer::color_b](i, j), new_color[2]);
    }
  }
}

// Register this stage with the factory
REGISTER_STAGE(HeightEstimation)

} // namespace height_mapping::core::stages