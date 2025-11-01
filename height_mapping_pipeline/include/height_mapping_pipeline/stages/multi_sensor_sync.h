/*
 * multi_sensor_sync.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_MULTI_SENSOR_SYNC_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_MULTI_SENSOR_SYNC_H

#include "height_mapping_core/data/point_cloud.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "flowpipe/stage.h"
#include "flowpipe/stage_registration.h"
#include <chrono>
#include <deque>
#include <logger/logger.h>
#include <map>
#include <memory>
#include <mutex>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage for multi-sensor synchronization (header-only)
 *
 * This stage buffers and synchronizes point clouds from multiple sensors.
 * Note: This is a pipeline-specific feature, not a core algorithm.
 */
class MultiSensorSync : public flowpipe::Stage {
  struct SensorBuffer {
    std::deque<std::shared_ptr<height_mapping::PointCloud>> clouds;
    std::deque<uint64_t> timestamps;
    std::mutex mutex;
  };

public:
  MultiSensorSync() : Stage("MultiSensorSync") {}

  void configure(const std::map<std::string, std::string> &params) override {
    loadParam(params, "buffer_size", buffer_size_);

    double sync_tolerance;
    if (loadParam(params, "sync_tolerance_ms", sync_tolerance)) {
      sync_tolerance_ms_ = sync_tolerance;
    }

    std::string policy;
    if (loadParam(params, "merge_policy", policy)) {
      merge_policy_ = policy;
      if (merge_policy_ != "latest" && merge_policy_ != "exact_sync" &&
          merge_policy_ != "nearest") {
        LOG_ERROR(getName(), "Unknown merge policy: ", merge_policy_,
                  ", using 'latest'");
        merge_policy_ = "latest";
      }
    }

    loadParam(params, "max_sensors", max_sensors_);

    LOG_DEBUG(getName(), "Configured with buffer_size=", buffer_size_,
              ", sync_tolerance=", sync_tolerance_ms_,
              "ms, policy=", merge_policy_);
  }

protected:
  void processImpl(flowpipe::Context &ctx) override {
    auto &mapping_ctx = static_cast<MappingContext &>(ctx);

    // Get sensor ID from context metadata (if available)
    std::string sensor_id = "default";
    // In real implementation, sensor_id would come from context metadata

    // Use cloud timestamp
    uint64_t timestamp = mapping_ctx.cloud().timestamp();

    // Store current cloud in buffer
    auto &buffer = sensor_buffers_[sensor_id];
    {
      std::lock_guard<std::mutex> lock(buffer.mutex);

      // Add cloud to buffer
      auto cloud_ptr =
          std::make_shared<height_mapping::PointCloud>(mapping_ctx.cloud());
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
      processed_count_++;

    } else if (merge_policy_ == "exact_sync") {
      // Try to synchronize clouds from all sensors
      if (synchronizeClouds(mapping_ctx, timestamp)) {
        synchronized_count_++;
      } else {
        // Sync failed, use current cloud
        sync_failures_++;
      }

    } else if (merge_policy_ == "nearest") {
      // Merge with nearest timestamp clouds
      mergeLatest(mapping_ctx);
      merged_count_++;
    }

    LOG_DEBUG_THROTTLE(
        1.0, getName(), "Multi-sensor sync: ", sensor_buffers_.size(),
        " sensors, ", processed_count_, " processed, ", synchronized_count_,
        " synchronized, ", merged_count_, " merged");
  }

private:
  bool synchronizeClouds(MappingContext &ctx, uint64_t target_timestamp) {
    if (sensor_buffers_.size() < 2) {
      return false; // Need at least 2 sensors for sync
    }

    height_mapping::PointCloud merged_cloud;
    uint64_t tolerance_ns = static_cast<uint64_t>(sync_tolerance_ms_ * 1e6);

    for (auto &[sensor_id, buffer] : sensor_buffers_) {
      std::lock_guard<std::mutex> lock(buffer.mutex);

      // Find cloud with closest timestamp within tolerance
      bool found = false;
      for (size_t i = 0; i < buffer.timestamps.size(); ++i) {
        uint64_t diff = (buffer.timestamps[i] > target_timestamp)
                            ? buffer.timestamps[i] - target_timestamp
                            : target_timestamp - buffer.timestamps[i];

        if (diff <= tolerance_ns) {
          merged_cloud += *buffer.clouds[i];
          found = true;
          break;
        }
      }

      if (!found) {
        return false; // Couldn't find synchronized cloud for this sensor
      }
    }

    // Replace context cloud with merged result
    ctx.cloud() = std::move(merged_cloud);
    return true;
  }

  void mergeLatest(MappingContext &ctx) {
    height_mapping::PointCloud merged_cloud = ctx.cloud();

    for (auto &[sensor_id, buffer] : sensor_buffers_) {
      if (sensor_id == "default")
        continue; // Skip current sensor

      std::lock_guard<std::mutex> lock(buffer.mutex);
      if (!buffer.clouds.empty()) {
        merged_cloud += *buffer.clouds.back();
      }
    }

    ctx.cloud() = std::move(merged_cloud);
  }

  // Configuration
  size_t buffer_size_ = 10;
  double sync_tolerance_ms_ = 50.0;
  std::string merge_policy_ = "latest";
  size_t max_sensors_ = 5;

  // State
  std::map<std::string, SensorBuffer> sensor_buffers_;

  // Statistics
  size_t processed_count_ = 0;
  size_t synchronized_count_ = 0;
  size_t merged_count_ = 0;
  size_t sync_failures_ = 0;
};

REGISTER_STAGE(MultiSensorSync)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_MULTI_SENSOR_SYNC_H