/*
 * statistical_ground_segmentation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/statistical_ground_segmentation.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registration.h"
#include <algorithm>
#include <logger/logger.h>

namespace height_mapping::core::stages {

constexpr const char *STAGE_NAME = "StatisticalGroundSegmentation";

StatisticalGroundSegmentation::StatisticalGroundSegmentation()
    : Stage(STAGE_NAME) {}

void StatisticalGroundSegmentation::configure(
    const std::map<std::string, std::string> &params) {

  auto getParam = [&params](const std::string &key, auto default_val) {
    auto it = params.find(key);
    if (it != params.end()) {
      if constexpr (std::is_same_v<decltype(default_val), float>) {
        return std::stof(it->second);
      } else if constexpr (std::is_same_v<decltype(default_val), size_t>) {
        return static_cast<size_t>(std::stoi(it->second));
      } else if constexpr (std::is_same_v<decltype(default_val), bool>) {
        return it->second == "true" || it->second == "1";
      }
    }
    return default_val;
  };

  // New simple parameters
  ground_percentile_ = getParam("ground_percentile", 0.1f); // for robust min
  ground_thickness_ = getParam("ground_thickness", 0.3f);   // 30cm ground layer
  noise_threshold_ =
      getParam("noise_threshold", 0.5f); // below percentile: noise
  keep_only_ground_ = getParam("keep_only_ground", true);

  LOG_DEBUG(STAGE_NAME,
            "Robust ground extraction: percentile=", ground_percentile_,
            ", thickness=", ground_thickness_, "m");
}

void StatisticalGroundSegmentation::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &cloud = mapping_ctx.cloud();

  if (cloud.empty()) {
    LOG_WARN(STAGE_NAME, "Input cloud is empty, skipping segmentation...");
    return;
  }

  // Collect all z values for percentile calculation
  std::vector<float> all_z_values;
  all_z_values.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    all_z_values.push_back(cloud[i].z());
  }

  // Sort for percentile calculation
  std::sort(all_z_values.begin(), all_z_values.end());

  // Use configured percentile as robust minimum (ignores outlier noise points)
  // This filters out deep noise spikes that occasionally appear below ground
  size_t percentile_idx =
      static_cast<size_t>(ground_percentile_ * all_z_values.size());
  percentile_idx =
      std::max(size_t(0), std::min(percentile_idx, all_z_values.size() - 1));
  float robust_min_z = all_z_values[percentile_idx];

  // Define ground layer thickness from the robust minimum
  // Points within this thickness from the robust minimum are considered ground
  float ground_threshold = robust_min_z + ground_thickness_;

  // Classify and filter points
  size_t ground_count = 0;
  size_t obstacle_count = 0;
  size_t noise_count = 0;

  geometry::PointCloud filtered_cloud;
  filtered_cloud.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    auto point = cloud[i];
    float z = point.z();

    if (z < robust_min_z - noise_threshold_) {
      // Deep outlier noise - ignore completely
      noise_count++;
    } else if (z <= ground_threshold) {
      // Ground point
      ground_count++;
      if (keep_only_ground_) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
    } else {
      // Obstacle point
      obstacle_count++;
      if (!keep_only_ground_) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
    }
  }
  std::string debug_msg =
      "Robust ground extraction: " + std::to_string(ground_count) +
      " ground, " + std::to_string(obstacle_count) + " obstacle, " +
      std::to_string(noise_count) + " noise points" + " (" +
      std::to_string(static_cast<int>(ground_percentile_ * 100)) +
      "th percentile z=" + std::to_string(robust_min_z) +
      "m, threshold=" + std::to_string(ground_threshold) + "m)";
  LOG_DEBUG_THROTTLE(1.0, STAGE_NAME, debug_msg);

  // Replace original cloud
  filtered_cloud.setTimestamp(cloud.timestamp());
  filtered_cloud.setFrameId(cloud.frameId());
  cloud = std::move(filtered_cloud);
}

// Register this stage with the factory
REGISTER_STAGE(StatisticalGroundSegmentation)

} // namespace height_mapping::core::stages