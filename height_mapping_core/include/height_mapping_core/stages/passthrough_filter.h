/*
 * passthrough_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_PASSTHROUGH_FILTER_H
#define HEIGHT_MAPPING_CORE_STAGES_PASSTHROUGH_FILTER_H

#include "pipeline_core/stage.h"
#include <limits>

namespace height_mapping::core {

/**
 * @brief Stage that filters points outside specified bounds
 *
 * Removes points that fall outside the specified x, y, z limits
 * or distance from origin.
 */
class PassthroughFilterStage : public pipeline::Stage {
public:
  PassthroughFilterStage();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "x_min", "x_max": X-axis limits in meters
   *        - "y_min", "y_max": Y-axis limits in meters
   *        - "z_min", "z_max": Z-axis limits in meters
   *        - "distance_min", "distance_max": Distance from origin limits
   */
  void configure(const std::map<std::string, std::string>& params) override;

  // Setters for programmatic configuration
  void setXLimits(float min, float max) { x_min_ = min; x_max_ = max; }
  void setYLimits(float min, float max) { y_min_ = min; y_max_ = max; }
  void setZLimits(float min, float max) { z_min_ = min; z_max_ = max; }
  void setDistanceLimits(float min, float max) {
    distance_min_ = min;
    distance_max_ = max;
    use_distance_filter_ = true;
  }

protected:
  void processImpl(pipeline::Context& ctx) override;

private:
  // Axis-aligned bounds
  float x_min_ = -std::numeric_limits<float>::max();
  float x_max_ = std::numeric_limits<float>::max();
  float y_min_ = -std::numeric_limits<float>::max();
  float y_max_ = std::numeric_limits<float>::max();
  float z_min_ = -std::numeric_limits<float>::max();
  float z_max_ = std::numeric_limits<float>::max();

  // Distance filter
  float distance_min_ = 0.0f;
  float distance_max_ = std::numeric_limits<float>::max();
  bool use_distance_filter_ = false;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_STAGES_PASSTHROUGH_FILTER_H