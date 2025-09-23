/*
 * transform.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_POINT_CLOUD_TRANSFORM_H
#define HEIGHT_MAPPING_CORE_STAGES_POINT_CLOUD_TRANSFORM_H

#include "pipeline_core/stage.h"
#include <string>

namespace height_mapping::core::stages {

/**
 * @brief Stage that transforms point clouds between coordinate frames
 *
 * Uses the transform provider service to lookup and apply transforms.
 * Requires a transform provider to be injected into the context.
 */
class PointCloudTransform : public pipeline::Stage {
public:
  PointCloudTransform();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "target_frame": Target coordinate frame (default: "map")
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  std::string target_frame_{"map"};
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_POINT_CLOUD_TRANSFORM_H