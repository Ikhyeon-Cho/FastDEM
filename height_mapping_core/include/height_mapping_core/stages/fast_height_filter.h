/*
 * fast_height_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_FAST_HEIGHT_FILTER_H
#define HEIGHT_MAPPING_CORE_STAGES_FAST_HEIGHT_FILTER_H

#include "pipeline_core/stage.h"

namespace height_mapping::core {

/**
 * @brief Stage that quickly filters points based on height thresholds
 *
 * Efficiently removes points that are too high or too low relative to
 * expected terrain height, reducing processing load for later stages.
 */
class FastHeightFilterStage : public pipeline::Stage {
public:
  FastHeightFilterStage();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "min_height": Minimum height relative to ground
   *        - "max_height": Maximum height relative to ground
   *        - "reference_height": Reference ground height
   *        - "adaptive": Enable adaptive height filtering
   */
  void configure(const std::map<std::string, std::string>& params) override;

protected:
  void processImpl(pipeline::Context& ctx) override;

private:
  float min_height_ = -2.0f;  // meters below reference
  float max_height_ = 3.0f;   // meters above reference
  float reference_height_ = 0.0f;
  bool adaptive_filtering_ = false;

  // Statistics
  size_t total_points_filtered_ = 0;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_STAGES_FAST_HEIGHT_FILTER_H