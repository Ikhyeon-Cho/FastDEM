/*
 * multi_sensor_sync.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_MULTI_SENSOR_SYNC_H
#define HEIGHT_MAPPING_LAB_STAGES_MULTI_SENSOR_SYNC_H

#include <spdlog/spdlog.h>

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>

#include "height_mapping/core.h"
#include "height_mapping/lab/frame.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage for multi-sensor synchronization (header-only)
 *
 * This stage buffers and synchronizes point clouds from multiple sensors.
 * Note: This is a pipeline-specific feature, not a core algorithm.
 */
class MultiSensorSync : public ::ppl::Stage<MappingFrame> {
  struct SensorBuffer {
    std::deque<std::shared_ptr<PointCloud>> clouds;
    std::deque<uint64_t> timestamps;
    std::mutex mutex;
  };

 public:
  MultiSensorSync() = default;

  void configure(const YAML::Node& config) override {
    if (config["buffer_size"])
      buffer_size_ = config["buffer_size"].as<size_t>();

    if (config["sync_tolerance_ms"])
      sync_tolerance_ms_ = config["sync_tolerance_ms"].as<double>();

    if (config["merge_policy"]) {
      merge_policy_ = config["merge_policy"].as<std::string>();
      if (merge_policy_ != "latest" && merge_policy_ != "exact_sync" &&
          merge_policy_ != "nearest") {
        spdlog::error(
            "[MultiSensorSync] Unknown merge policy: {}, using 'latest'",
            merge_policy_);
        merge_policy_ = "latest";
      }
    }

    if (config["max_sensors"])
      max_sensors_ = config["max_sensors"].as<size_t>();

    spdlog::debug(
        "[MultiSensorSync] buffer_size={}, sync_tolerance={}ms, policy={}",
        buffer_size_, sync_tolerance_ms_, merge_policy_);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    // Get sensor ID from frame metadata (if available)
    std::string sensor_id = "default";
    // In real implementation, sensor_id would come from frame metadata

    // Use cloud timestamp (already in nanoseconds)
    uint64_t timestamp = frame->cloud->timestamp();

    // Store current cloud in buffer
    auto& buffer = sensor_buffers_[sensor_id];
    {
      std::lock_guard<std::mutex> lock(buffer.mutex);

      // Add cloud to buffer
      auto cloud_ptr = std::make_shared<PointCloud>(*frame->cloud);
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
      // Simply use the current cloud (already in frame)
      processed_count_++;

    } else if (merge_policy_ == "exact_sync") {
      // Try to synchronize clouds from all sensors
      if (synchronizeClouds(frame, timestamp)) {
        synchronized_count_++;
      } else {
        // Sync failed, use current cloud
        sync_failures_++;
      }

    } else if (merge_policy_ == "nearest") {
      // Merge with nearest timestamp clouds
      mergeLatest(frame);
      merged_count_++;
    }

    spdlog::debug(
        "[MultiSensorSync] {} sensors, {} processed, {} synchronized, {} "
        "merged",
        sensor_buffers_.size(), processed_count_, synchronized_count_,
        merged_count_);
    return true;
  }

 private:
  bool synchronizeClouds(const std::shared_ptr<MappingFrame>& frame,
                         uint64_t target_timestamp) {
    if (sensor_buffers_.size() < 2) {
      return false;  // Need at least 2 sensors for sync
    }

    PointCloud merged_cloud;
    uint64_t tolerance_ns = static_cast<uint64_t>(sync_tolerance_ms_ * 1e6);

    for (auto& [sensor_id, buffer] : sensor_buffers_) {
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
        return false;  // Couldn't find synchronized cloud for this sensor
      }
    }

    // Replace frame cloud with merged result
    *frame->cloud = std::move(merged_cloud);
    return true;
  }

  void mergeLatest(const std::shared_ptr<MappingFrame>& frame) {
    PointCloud merged_cloud = *frame->cloud;

    for (auto& [sensor_id, buffer] : sensor_buffers_) {
      if (sensor_id == "default") continue;  // Skip current sensor

      std::lock_guard<std::mutex> lock(buffer.mutex);
      if (!buffer.clouds.empty()) {
        merged_cloud += *buffer.clouds.back();
      }
    }

    *frame->cloud = std::move(merged_cloud);
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

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_MULTI_SENSOR_SYNC_H
