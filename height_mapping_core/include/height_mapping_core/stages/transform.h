/*
 * transform.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_TRANSFORM_H
#define HEIGHT_MAPPING_CORE_STAGES_TRANSFORM_H

#include "pipeline_core/stage.h"
#include <Eigen/Geometry>

namespace height_mapping::core {

/**
 * @brief Stage that transforms point clouds between coordinate frames
 */
class TransformStage : public pipeline::Stage {
public:
  TransformStage();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "target_frame": Target coordinate frame
   *        - "static_transform": 4x4 transformation matrix (16 values)
   */
  void configure(const std::map<std::string, std::string>& params) override;

  // Setters for programmatic configuration
  void setTargetFrame(const std::string& frame) { target_frame_ = frame; }
  void setStaticTransform(const Eigen::Matrix4f& transform) {
    static_transform_ = transform;
    use_static_transform_ = true;
  }

protected:
  void processImpl(pipeline::Context& ctx) override;

private:
  std::string target_frame_ = "map";
  Eigen::Matrix4f static_transform_ = Eigen::Matrix4f::Identity();
  bool use_static_transform_ = false;

  // Helper function to parse transform matrix from string
  Eigen::Matrix4f parseTransformMatrix(const std::string& str);
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_STAGES_TRANSFORM_H