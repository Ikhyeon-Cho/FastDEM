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
#include <cmath>

namespace height_mapping::core::stages {

/**
 * @brief Stage that performs raycasting for height correction
 *
 * Uses ray constraints to correct overestimated heights. If a ray passes
 * through a cell at height h, the ground cannot be higher than h.
 * Ray angle is used to identify likely ground measurements.
 */
class Raycasting : public pipeline::Stage {
public:
  Raycasting();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "max_ground_angle": Max angle (degrees) for ground rays (default: -5)
   *        - "correction_threshold": Buffer height for corrections (default: 0.02m)
   *        - "enable_correction": Whether to enable height correction (default: true)
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  // Configuration parameters
  float max_ground_angle_ = -5.0f * M_PI / 180.0f; // -5 degrees in radians
  float correction_threshold_ = 0.02f;             // 2cm buffer
  bool enable_correction_ = true;                  // Enable height correction

  // Statistics
  mutable size_t total_rays_cast_ = 0;
  mutable size_t cells_corrected_ = 0;
  mutable size_t ground_points_detected_ = 0;
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_RAYCASTING_H