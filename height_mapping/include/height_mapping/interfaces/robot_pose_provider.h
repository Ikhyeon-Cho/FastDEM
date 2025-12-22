/*
 * robot_pose_provider.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_INTERFACES_ROBOT_POSE_PROVIDER_H
#define HEIGHT_MAPPING_CORE_INTERFACES_ROBOT_POSE_PROVIDER_H

#include <memory>
#include <optional>

#include "height_mapping/types.h"

namespace height_mapping {

/**
 * @brief Interface for providing robot pose estimation
 *
 * This interface provides the dynamic transformation from robot body frame
 * to the global/map frame. This is typically the result of localization
 * (SLAM, odometry, etc.) and changes over time.
 *
 * Semantic meaning: "Where is the robot in the world?"
 */
class IRobotPoseProvider {
 public:
  using Ptr = std::shared_ptr<IRobotPoseProvider>;
  using ConstPtr = std::shared_ptr<const IRobotPoseProvider>;

  virtual ~IRobotPoseProvider() = default;

  /**
   * @brief Get the robot pose in the global/map frame at a specific time
   * @param timestamp_ns Timestamp in nanoseconds
   * @return Transform T_map_base (Base → Map) if available
   *
   * The returned transform represents the robot's position and orientation
   * in the global coordinate frame. This is dynamic and changes as the
   * robot moves.
   */
  virtual std::optional<Transformf> getRobotPoseAt(uint64_t timestamp_ns) = 0;

  /**
   * @brief Get the global/map frame name (e.g., "map", "world", "odom")
   */
  virtual std::string getMapFrame() const = 0;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_INTERFACES_ROBOT_POSE_PROVIDER_H
