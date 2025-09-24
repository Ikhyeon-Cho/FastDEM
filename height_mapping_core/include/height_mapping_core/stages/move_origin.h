/*
 * move_origin.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_MOVE_ORIGIN_H
#define HEIGHT_MAPPING_CORE_STAGES_MOVE_ORIGIN_H

#include "pipeline_core/stage.h"
#include <Eigen/Geometry>

namespace height_mapping::core::stages {

/**
 * @brief Stage that updates the map origin to follow the robot
 *
 * Keeps the map centered around the robot or updates based on movement
 * threshold.
 */
class MoveOrigin : public pipeline::Stage {
public:
  enum class UpdateMode {
    CONTINUOUS, // Always center on robot
    THRESHOLD,  // Update when robot moves beyond threshold
    CENTERED,   // Keep robot in center region
    FIXED       // Never update origin
  };

  MoveOrigin();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "update_mode": continuous, threshold, centered, or fixed
   *        - "update_threshold": Movement threshold in meters (for threshold
   * mode)
   *        - "center_region_size": Size of center region (for centered mode)
   *        - "robot_frame": Robot coordinate frame name
   *        - "map_frame": Map coordinate frame name
   */
  void configure(const std::map<std::string, std::string> &params) override;

  // Setters
  void setUpdateMode(UpdateMode mode) { update_mode_ = mode; }
  void setUpdateThreshold(float threshold) { update_threshold_ = threshold; }

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  UpdateMode update_mode_ = UpdateMode::CONTINUOUS;
  float update_threshold_ = 1.0f;   // meters
  float center_region_size_ = 5.0f; // meters
  std::string robot_frame_ = "base_link";
  std::string map_frame_ = "map";

  // Track last update position
  Eigen::Vector3f last_update_position_ = Eigen::Vector3f::Zero();
  bool first_update_ = true;

  // Statistics
  size_t total_updates_ = 0;
  float total_distance_moved_ = 0.0f;
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_MOVE_ORIGIN_H