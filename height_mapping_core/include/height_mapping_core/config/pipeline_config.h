/*
 * pipeline_config.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_CONFIG_PIPELINE_CONFIG_H
#define HEIGHT_MAPPING_CORE_CONFIG_PIPELINE_CONFIG_H

#include <string>

namespace height_mapping::core {

// Pure pipeline configuration - no ROS dependencies
struct PipelineConfig {

  // Which stages to enable
  struct Stages {
    bool transform = true;
    bool voxel_filter = true;
    bool passthrough_filter = true;
    bool height_estimation = true;
    bool map_update = true;
  } stages;

  // Filter parameters
  struct Filters {
    // Voxel filter
    double voxel_size = 0.05;

    // Passthrough filter bounds (in sensor frame)
    double x_min = -100.0;
    double x_max = 100.0;
    double y_min = -100.0;
    double y_max = 100.0;
    double z_min = -2.0;
    double z_max = 5.0;
  } filters;

  // Transform stage
  struct Transform {
    std::string target_frame = "map";
  } transform;

  // Height estimator parameters
  struct Estimator {
    enum Type {
      MEAN,
      KALMAN,
      MOVING_AVERAGE,
      INCREMENTAL_MEAN,
      ADAPTIVE_KALMAN
    };

    Type type = INCREMENTAL_MEAN;
    double kalman_process_noise = 0.01;
    double kalman_measurement_noise = 0.1;
    int moving_average_window = 5;
  } estimator;

  // Map update parameters
  struct MapUpdate {
    enum Mode { CONTINUOUS, THRESHOLD, CENTERED };

    Mode mode = CONTINUOUS;
    double threshold = 1.0; // For threshold mode
    std::string robot_frame = "base_link";
    // Note: map_frame is derived from transform.target_frame
  } map_update;

  // Pipeline behavior
  struct Behavior {
    bool stop_on_error = true;
    bool enable_profiling = false;
  } behavior;

  // Validate configuration
  bool validate() const;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_CONFIG_PIPELINE_CONFIG_H