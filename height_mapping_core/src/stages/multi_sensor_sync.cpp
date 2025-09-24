/*
 * multi_sensor_sync.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/multi_sensor_sync.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registration.h"

#include <logger/logger.h>

namespace height_mapping::core::stages {

MultiSensorSync::MultiSensorSync() : Stage("MultiSensorSync") {}

void MultiSensorSync::configure(
    const std::map<std::string, std::string> &params) {
  auto it = params.find("buffer_size");
  if (it != params.end()) {
    buffer_size_ = std::stoull(it->second);
  }

  it = params.find("sync_tolerance_ms");
  if (it != params.end()) {
    sync_tolerance_ms_ = std::stod(it->second);
  }

  it = params.find("merge_policy");
  if (it != params.end()) {
    merge_policy_ = it->second;
    if (merge_policy_ != "latest" && merge_policy_ != "exact_sync" &&
        merge_policy_ != "nearest") {
      LOG_ERROR("MultiSensorSync", "Unknown merge policy: ", merge_policy_,
                ", using 'latest'");
      merge_policy_ = "latest";
    }
  }

  it = params.find("max_sensors");
  if (it != params.end()) {
    max_sensors_ = std::stoull(it->second);
  }
}

void MultiSensorSync::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);

  // Use default sensor ID (would be set by caller in real implementation)
  std::string sensor_id = "default";

  // Use current time as timestamp
  uint64_t timestamp = static_cast<uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());

  // Store current cloud in buffer
  auto &buffer = sensor_buffers_[sensor_id];
  {
    std::lock_guard<std::mutex> lock(buffer.mutex);

    // Add cloud to buffer
    auto cloud_ptr =
        std::make_shared<geometry::PointCloud>(mapping_ctx.cloud());
    buffer.clouds.push_back(cloud_ptr);
    buffer.timestamps.push_back(timestamp);

    // Maintain buffer size
    while (buffer.clouds.size() > buffer_size_) {
      buffer.clouds.pop_front();
      buffer.timestamps.pop_front();
    }
  }

  // Apply merge policy
  if (merge_policy_ == "latest") {
    // Simply use the current cloud (already in context)

  } else if (merge_policy_ == "exact_sync") {
    // Try to synchronize clouds from all sensors
    if (synchronizeClouds(ctx)) {
    } else {
      // Sync failed, use current cloud
    }

  } else if (merge_policy_ == "nearest") {
    // Merge with nearest timestamp clouds
    mergeLatest(ctx);
  }

  // Store statistics
}

bool MultiSensorSync::synchronizeClouds(pipeline::Context &ctx) {
  // Check if all sensors have data
  if (sensor_buffers_.size() < 2) {
    return false; // Need at least 2 sensors for sync
  }

  // Find common timestamp within tolerance
  // This is a simplified implementation
  // Real implementation would find overlapping timestamps across all sensors

  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  geometry::PointCloud merged_cloud;

  for (const auto &[sensor_id, buffer] : sensor_buffers_) {
    std::lock_guard<std::mutex> lock(buffer.mutex);
    if (!buffer.clouds.empty()) {
      // Merge latest cloud from each sensor
      merged_cloud += *(buffer.clouds.back());
    }
  }

  if (!merged_cloud.empty()) {
    mapping_ctx.cloud() = std::move(merged_cloud);
    return true;
  }

  return false;
}

void MultiSensorSync::mergeLatest(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  geometry::PointCloud merged_cloud = mapping_ctx.cloud();

  // Merge with latest clouds from other sensors
  for (const auto &[sensor_id, buffer] : sensor_buffers_) {
    std::lock_guard<std::mutex> lock(buffer.mutex);
    if (!buffer.clouds.empty()) {
      // Skip if this is the current sensor
      if (sensor_id == "default") {
        continue;
      }
      merged_cloud += *(buffer.clouds.back());
    }
  }

  mapping_ctx.cloud() = std::move(merged_cloud);
}

// Register this stage with the factory
REGISTER_STAGE(MultiSensorSync)

} // namespace height_mapping::core::stages