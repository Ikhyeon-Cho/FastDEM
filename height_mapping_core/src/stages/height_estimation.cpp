/*
 * height_estimation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/height_estimation.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <iostream>

namespace height_mapping::core {

HeightEstimationStage::HeightEstimationStage()
    : Stage("HeightEstimation", "Height Estimator") {
}

void HeightEstimationStage::configure(const std::map<std::string, std::string>& params) {
  // Configure estimator type
  auto it = params.find("estimator_type");
  if (it != params.end()) {
    const std::string& type = it->second;
    if (type == "kalman") {
      estimator_type_ = EstimatorType::KALMAN;
    } else if (type == "mean") {
      estimator_type_ = EstimatorType::INCREMENTAL_MEAN;
    } else if (type == "moving_average") {
      estimator_type_ = EstimatorType::MOVING_AVERAGE;
    } else {
      std::cerr << "[HeightEstimation] Unknown estimator type: " << type
                << ", using default KALMAN" << std::endl;
    }
  }

  // Kalman filter parameters
  it = params.find("process_noise");
  if (it != params.end()) {
    process_noise_ = std::stof(it->second);
  }

  it = params.find("measurement_noise");
  if (it != params.end()) {
    measurement_noise_ = std::stof(it->second);
  }

  // Moving average parameters
  it = params.find("window_size");
  if (it != params.end()) {
    window_size_ = std::stoi(it->second);
    if (window_size_ <= 0) {
      std::cerr << "[HeightEstimation] Invalid window size: " << window_size_
                << ", using default 10" << std::endl;
      window_size_ = 10;
    }
  }
}

void HeightEstimationStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  // Access the height map from context
  auto& map = mapping_ctx.map();
  if (map.empty()) {
    std::cerr << "[HeightEstimation] No height map in context" << std::endl;
    return;
  }

  auto& cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  // TODO: Implement actual height estimation algorithms
  // For now, just store the estimator type in context
  switch (estimator_type_) {
    case EstimatorType::KALMAN:
      break;

    case EstimatorType::INCREMENTAL_MEAN:
      break;

    case EstimatorType::MOVING_AVERAGE:
      break;
  }

  // Here we would actually update the height map with estimated heights
  // This is a placeholder implementation
  size_t points_processed = cloud.size();
}

// Register this stage with the factory
REGISTER_STAGE(HeightEstimationStage)

} // namespace height_mapping::core