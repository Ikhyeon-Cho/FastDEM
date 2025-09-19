/*
 * multi_sensor_sync_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_MULTI_SENSOR_SYNC_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_MULTI_SENSOR_SYNC_STAGE_H

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <chrono>
#include <deque>
#include <mutex>

namespace height_mapping::core {

class MultiSensorSyncStage : public PipelineStage<MultiSensorSyncStage> {
public:
  static constexpr const char *STAGE_NAME = "MultiSensorSync";
  static constexpr const char *STAGE_TYPE = "Synchronization";

  struct SensorData {
    std::string sensor_id;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::chrono::steady_clock::time_point timestamp;
  };

  explicit MultiSensorSyncStage(size_t num_sensors = 2,
                                double sync_tolerance_ms = 50.0)
      : num_sensors_(num_sensors), sync_tolerance_ms_(sync_tolerance_ms) {}

  void initialize() override {
    sensor_buffers_.resize(num_sensors_);
    initialized_ = true;
  }

  void processImpl(ProcessingContext &ctx) {
    // Get sensor ID from ctx
    auto sensor_id = ctx.getOrDefault<std::string>("sensor_id", "sensor_0");

    // Find sensor index
    size_t sensor_idx = getSensorIndex(sensor_id);
    if (sensor_idx >= num_sensors_) {
      // Unknown sensor, skip
      return;
    }

    // Add to buffer
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);

      SensorData data;
      data.sensor_id = sensor_id;
      data.cloud = ctx.cloud();
      data.timestamp = ctx.stats().timestamp;

      sensor_buffers_[sensor_idx].push_back(data);

      // Keep buffer size limited
      while (sensor_buffers_[sensor_idx].size() > max_buffer_size_) {
        sensor_buffers_[sensor_idx].pop_front();
      }
    }

    // Try to synchronize
    if (trySynchronize(ctx)) {
      ctx.set("synchronized", true);
      ctx.set("num_sensors_synchronized", static_cast<int>(num_sensors_));
    } else {
      ctx.set("synchronized", false);
      ctx.set("waiting_for_sync", true);
    }
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    return initialized_;
  }

  void setNumSensors(size_t num) {
    num_sensors_ = num;
    sensor_buffers_.resize(num_sensors_);
  }

  void setSyncTolerance(double tolerance_ms) {
    sync_tolerance_ms_ = tolerance_ms;
  }

private:
  size_t getSensorIndex(const std::string &sensor_id) {
    // Simple mapping: sensor_0 -> 0, sensor_1 -> 1, etc.
    if (sensor_id.size() > 7 && sensor_id.substr(0, 7) == "sensor_") {
      try {
        return std::stoul(sensor_id.substr(7));
      } catch (...) {
        return num_sensors_; // Invalid index
      }
    }

    // Try to find in registered sensors
    auto it = std::find(registered_sensors_.begin(), registered_sensors_.end(),
                        sensor_id);
    if (it != registered_sensors_.end()) {
      return std::distance(registered_sensors_.begin(), it);
    }

    // Register new sensor
    if (registered_sensors_.size() < num_sensors_) {
      registered_sensors_.push_back(sensor_id);
      return registered_sensors_.size() - 1;
    }

    return num_sensors_; // Invalid
  }

  bool trySynchronize(ProcessingContext &ctx) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // Check if all sensors have data
    for (const auto &buffer : sensor_buffers_) {
      if (buffer.empty()) {
        return false;
      }
    }

    // Find closest timestamps
    auto ref_time = sensor_buffers_[0].front().timestamp;
    std::vector<size_t> sync_indices(num_sensors_, 0);

    for (size_t i = 1; i < num_sensors_; ++i) {
      size_t best_idx = 0;
      auto best_diff =
          std::chrono::milliseconds(static_cast<long>(sync_tolerance_ms_) + 1);

      for (size_t j = 0; j < sensor_buffers_[i].size(); ++j) {
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::abs((sensor_buffers_[i][j].timestamp - ref_time).count()));

        if (diff < best_diff) {
          best_diff = diff;
          best_idx = j;
        }
      }

      if (best_diff.count() > sync_tolerance_ms_) {
        return false; // No sync possible
      }

      sync_indices[i] = best_idx;
    }

    // Merge synchronized clouds
    pcl::PointCloud<pcl::PointXYZ> merged_cloud;

    for (size_t i = 0; i < num_sensors_; ++i) {
      const auto &sensor_cloud = sensor_buffers_[i][sync_indices[i]].cloud;
      merged_cloud += sensor_cloud;
    }

    // Update ctx with merged cloud
    ctx.cloud() = merged_cloud;

    // Remove used data from buffers
    for (size_t i = 0; i < num_sensors_; ++i) {
      for (size_t j = 0; j <= sync_indices[i]; ++j) {
        if (!sensor_buffers_[i].empty()) {
          sensor_buffers_[i].pop_front();
        }
      }
    }

    return true;
  }

  size_t num_sensors_;
  double sync_tolerance_ms_;
  std::vector<std::deque<SensorData>> sensor_buffers_;
  std::vector<std::string> registered_sensors_;
  std::mutex buffer_mutex_;
  bool initialized_ = false;
  static constexpr size_t max_buffer_size_ = 10;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_MULTI_SENSOR_SYNC_STAGE_H