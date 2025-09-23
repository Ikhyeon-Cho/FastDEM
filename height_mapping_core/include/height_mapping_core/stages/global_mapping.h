/*
 * global_mapping.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_GLOBAL_MAPPING_H
#define HEIGHT_MAPPING_CORE_STAGES_GLOBAL_MAPPING_H

#include "pipeline_core/stage.h"
#include <memory>

namespace height_mapping::core::stages {

/**
 * @brief Stage that aggregates local maps into a global map
 *
 * Maintains a larger global map by stitching together local map updates,
 * handling overlaps and managing memory for large-scale mapping.
 */
class GlobalMapping : public pipeline::Stage {
public:
  GlobalMapping();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "global_map_size_x": Global map width in meters
   *        - "global_map_size_y": Global map height in meters
   *        - "global_map_resolution": Resolution in meters per cell
   *        - "merge_method": averaging, max_height, or latest
   *        - "memory_limit_mb": Maximum memory usage in megabytes
   *        - "enable_compression": Enable map compression
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  // Global map configuration
  float global_map_size_x_ = 100.0f;   // meters
  float global_map_size_y_ = 100.0f;   // meters
  float global_map_resolution_ = 0.1f; // meters per cell
  std::string merge_method_ = "averaging";

  // Memory management
  size_t memory_limit_mb_ = 1024; // 1GB default
  bool enable_compression_ = false;

  // Statistics
  size_t total_local_maps_integrated_ = 0;
  size_t global_map_cells_updated_ = 0;
  float global_map_coverage_percent_ = 0.0f;

  // Helper functions
  void mergeLocalToGlobal(pipeline::Context &ctx);
  float calculateCoverage() const;
  bool checkMemoryUsage() const;
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_GLOBAL_MAPPING_H