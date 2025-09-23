/*
 * multi_sensor_sync.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_MULTI_SENSOR_SYNC_H
#define HEIGHT_MAPPING_CORE_STAGES_MULTI_SENSOR_SYNC_H

#include "height_mapping_core/geometry/point_cloud.h"
#include "pipeline_core/stage.h"
#include <deque>
#include <memory>
#include <mutex>

namespace height_mapping::core::stages {

/**
 * @brief Stage that synchronizes and merges data from multiple sensors
 *
 * Buffers incoming point clouds from different sensors and merges them
 * based on timestamps and synchronization policy.
 */
class MultiSensorSync : public pipeline::Stage {
public:
  MultiSensorSync();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "buffer_size": Maximum number of clouds to buffer per sensor
   *        - "sync_tolerance_ms": Time tolerance for synchronization in
   * milliseconds
   *        - "merge_policy": how, exact_sync, or nearest
   *        - "max_sensors": Maximum number of sensors to handle
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  using PointCloud = geometry::PointCloud;

  struct CloudBuffer {
    std::deque<std::shared_ptr<PointCloud>> clouds;
    std::deque<uint64_t> timestamps;
    mutable std::mutex mutex;
  };

  // Configuration
  size_t buffer_size_ = 10;
  double sync_tolerance_ms_ = 100.0;    // 100ms tolerance
  std::string merge_policy_ = "latest"; // latest, exact_sync, nearest
  size_t max_sensors_ = 3;

  // Buffers for each sensor
  std::map<std::string, CloudBuffer> sensor_buffers_;

  // Helper functions
  bool synchronizeClouds(pipeline::Context &ctx);
  void mergeLatest(pipeline::Context &ctx);
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_MULTI_SENSOR_SYNC_H