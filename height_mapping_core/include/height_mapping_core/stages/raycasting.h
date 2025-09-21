/*
 * raycasting.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_RAYCASTING_H
#define HEIGHT_MAPPING_CORE_STAGES_RAYCASTING_H

#include "pipeline_core/stage.h"

namespace height_mapping::core {

/**
 * @brief Stage that performs raycasting from 3D points to 2.5D height map
 *
 * Projects 3D point cloud data onto a 2D grid map, handling occlusions
 * and ray tracing for accurate height estimation.
 */
class RaycastingStage : public pipeline::Stage {
public:
  RaycastingStage();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "ray_length_max": Maximum ray length in meters
   *        - "ray_length_min": Minimum ray length in meters
   *        - "enable_clearing": Enable clearing of cells along ray
   *        - "clearing_height_threshold": Height threshold for clearing
   */
  void configure(const std::map<std::string, std::string>& params) override;

protected:
  void processImpl(pipeline::Context& ctx) override;

private:
  float ray_length_max_ = 10.0f;
  float ray_length_min_ = 0.1f;
  bool enable_clearing_ = true;
  float clearing_height_threshold_ = 0.5f;

  // Statistics
  mutable size_t total_rays_cast_ = 0;
  mutable size_t cells_updated_ = 0;
  mutable size_t cells_cleared_ = 0;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_STAGES_RAYCASTING_H