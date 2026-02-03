// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * transform_interface.hpp
 *
 * Transform system interfaces for FastDEM.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_TRANSFORM_INTERFACE_HPP
#define FASTDEM_TRANSFORM_INTERFACE_HPP

#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>

namespace fastdem {

/**
 * @brief Sensor calibration interface (static transform).
 *
 * Provides the static transformation from sensor frame to robot body frame.
 * Semantic: "Where is the sensor mounted on the robot?"
 */
class Calibration {
 public:
  using Ptr = std::shared_ptr<Calibration>;

  virtual ~Calibration() = default;

  /// Get T_base_sensor (Sensor → Base) for the given sensor frame
  virtual std::optional<Eigen::Isometry3d> getExtrinsic(
      const std::string& sensor_frame) const = 0;

  /// Get the robot body frame name (e.g., "base_link")
  virtual std::string getBaseFrame() const = 0;
};

/**
 * @brief Robot odometry interface (dynamic transform).
 *
 * Provides the dynamic transformation from robot body frame to world frame.
 * Semantic: "Where is the robot?"
 */
class Odometry {
 public:
  using Ptr = std::shared_ptr<Odometry>;

  virtual ~Odometry() = default;

  /// Get T_world_base (Base → World) at the given timestamp
  virtual std::optional<Eigen::Isometry3d> getPoseAt(uint64_t timestamp_ns) const = 0;

  /// Get the world frame name (e.g., "odom", "map")
  virtual std::string getWorldFrame() const = 0;
};

}  // namespace fastdem

#endif  // FASTDEM_TRANSFORM_INTERFACE_HPP
