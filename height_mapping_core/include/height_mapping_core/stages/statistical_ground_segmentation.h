/*
 * statistical_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_STATISTICAL_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_CORE_STAGES_STATISTICAL_GROUND_SEGMENTATION_H

#include "pipeline_core/stage.h"

namespace height_mapping::core::stages {

/**
 * @brief Stage that segments ground from obstacles using robust percentile-based approach
 *
 * This stage uses a simple and robust method to extract ground points:
 * 1. Calculates a percentile (e.g., 10th) to find robust minimum z-value, ignoring noise
 * 2. Defines ground as points within a thickness range from this minimum
 * 3. Filters out deep noise points that appear below the percentile threshold
 */
class StatisticalGroundSegmentation : public pipeline::Stage {
public:
  StatisticalGroundSegmentation();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *  - "ground_percentile": Percentile for robust minimum (default: 0.1 = 10th percentile)
   *  - "ground_thickness": Thickness of ground layer from percentile (default: 0.3m)
   *  - "noise_threshold": Distance below percentile to consider as noise (default: 0.5m)
   *  - "keep_only_ground": If true, keeps only ground points (default: true)
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  // Configuration parameters
  float ground_percentile_ = 0.1f;  // Percentile for robust minimum (ignores noise)
  float ground_thickness_ = 0.3f;   // Thickness of ground layer from robust min
  float noise_threshold_ = 0.5f;    // Points this far below percentile are noise
  bool keep_only_ground_ = true;    // Filter to ground points only
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_STATISTICAL_GROUND_SEGMENTATION_H