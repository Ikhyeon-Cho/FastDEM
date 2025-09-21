/*
 * voxel_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_VOXEL_FILTER_H
#define HEIGHT_MAPPING_CORE_STAGES_VOXEL_FILTER_H

#include "pipeline_core/stage.h"

namespace height_mapping::core {

/**
 * @brief Stage that performs voxel grid filtering on point clouds
 *
 * Reduces point cloud density by dividing space into voxels and
 * keeping only one point per voxel (centroid).
 */
class VoxelFilterStage : public pipeline::Stage {
public:
  VoxelFilterStage();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "voxel_size": Size of voxels in meters (default: 0.05)
   */
  void configure(const std::map<std::string, std::string>& params) override;

  // Getters
  float getVoxelSize() const { return voxel_size_; }

protected:
  void processImpl(pipeline::Context& ctx) override;

private:
  float voxel_size_ = 0.05f;  // Default 5cm voxels

  // Statistics
  mutable size_t total_points_processed_ = 0;
  mutable size_t total_points_filtered_ = 0;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_STAGES_VOXEL_FILTER_H