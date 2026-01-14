/*
 * tf2.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_ADAPTERS_TF2_H
#define HEIGHT_MAPPING_ROS_ADAPTERS_TF2_H

#include <ros/ros.h>
#include <spdlog/spdlog.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

#include "height_mapping/interfaces/extrinsics_provider.h"
#include "height_mapping/interfaces/robot_pose_provider.h"
#include "height_mapping/types.h"

namespace height_mapping_ros::adapters {

// Import core types
using height_mapping::IExtrinsicsProvider;
using height_mapping::IRobotPoseProvider;
using height_mapping::Transformf;

/**
 * @brief ROS TF2-based implementation of IExtrinsicsProvider and
 * IRobotPoseProvider
 *
 * This class implements both interfaces using ROS TF2 as the backend.
 * It separates the semantic concepts of:
 * - Extrinsics: Static sensor-to-robot calibration (Sensor → Base)
 * - Robot Pose: Dynamic robot localization (Base → Map)
 *
 * While TF2 handles both as transforms, this class provides a clean
 * semantic separation at the interface level.
 */
class TF2 : public IExtrinsicsProvider, public IRobotPoseProvider {
 public:
  using Ptr = std::shared_ptr<TF2>;
  using ConstPtr = std::shared_ptr<const TF2>;

  struct Config {
    std::string base_frame = "base_link";
    std::string map_frame = "map";
    double max_wait_time = 0.1;
    double max_stale_time = 0.1;
    bool use_latest_transform_fallback = false;
  };

  explicit TF2(const Config& config)
      : config_(config),
        buffer_(::ros::Duration(10.0)),  // 10 sec cache
        listener_(buffer_) {
    ::ros::Duration(0.5).sleep();  // Wait for TF buffer to fill
  }

  ~TF2() override = default;

  // ========== IExtrinsicsProvider Implementation ==========

  std::string getBaseFrame() const override { return config_.base_frame; }

  /**
   * @brief Get sensor extrinsic calibration (Sensor → Base)
   *
   * This returns the static transform from sensor frame to base_link.
   * Uses time(0) to get the latest available transform, as extrinsics
   * are typically static.
   */
  std::optional<Transformf> getExtrinsic(
      const std::string& sensor_frame) override {
    if (sensor_frame.empty()) {
      spdlog::warn("Empty sensor_frame in getExtrinsic()");
      return std::nullopt;
    }

    // Use time(0) for static extrinsics
    auto tf_msg_opt =
        lookupTransformMsg(config_.base_frame, sensor_frame, ::ros::Time(0));

    if (!tf_msg_opt) {
      return std::nullopt;
    }

    return convertToTransform(tf_msg_opt.value(), sensor_frame,
                              config_.base_frame);
  }

  // ========== IRobotPoseProvider Implementation ==========

  std::string getMapFrame() const override { return config_.map_frame; }

  /**
   * @brief Get robot pose in global frame (Base → Map)
   *
   * This returns the dynamic transform from base_link to map frame.
   * Uses the provided timestamp for temporal consistency with sensor data.
   */
  std::optional<Transformf> getRobotPoseAt(uint64_t timestamp_ns) override {
    ::ros::Time lookup_time;
    lookup_time.fromNSec(timestamp_ns);

    // Try exact time lookup first
    auto tf_msg_opt =
        lookupTransformMsg(config_.map_frame, config_.base_frame, lookup_time);

    if (!tf_msg_opt) {
      // Fallback to latest if enabled
      if (config_.use_latest_transform_fallback) {
        spdlog::warn("Using latest transform as fallback for robot pose");
        tf_msg_opt = lookupTransformMsg(config_.map_frame, config_.base_frame,
                                        ::ros::Time(0));
      }
      if (!tf_msg_opt) {
        return std::nullopt;
      }
    }

    // Check time tolerance
    const auto& tf_msg = tf_msg_opt.value();
    double time_diff = std::abs((lookup_time - tf_msg.header.stamp).toSec());
    if (time_diff > config_.max_stale_time) {
      spdlog::warn("Robot pose time difference too large: {} sec (max: {} sec)",
                   time_diff, config_.max_stale_time);

      if (config_.use_latest_transform_fallback) {
        tf_msg_opt = lookupTransformMsg(config_.map_frame, config_.base_frame,
                                        ::ros::Time(0));
        if (!tf_msg_opt) {
          return std::nullopt;
        }
      } else {
        return std::nullopt;
      }
    }

    return convertToTransform(tf_msg_opt.value(), config_.base_frame,
                              config_.map_frame);
  }

 private:
  // Helper: Lookup transform message with exception handling
  std::optional<geometry_msgs::TransformStamped> lookupTransformMsg(
      const std::string& target_frame, const std::string& source_frame,
      const ::ros::Time& time) {
    try {
      return buffer_.lookupTransform(target_frame, source_frame, time,
                                     ::ros::Duration(config_.max_wait_time));
    } catch (const tf2::TransformException& ex) {
      spdlog::warn("TF lookup failed: {}", ex.what());
      return std::nullopt;
    }
  }

  // Helper: Convert ROS transform to Transformf with frame info
  // Note: Transformf constructor takes (parent, child, T) order
  Transformf convertToTransform(const geometry_msgs::TransformStamped& tf_msg,
                                const std::string& child_frame,
                                const std::string& parent_frame) const {
    Eigen::Isometry3f tf = tf2::transformToEigen(tf_msg).cast<float>();
    return Transformf(parent_frame, child_frame, tf);
  }

 private:
  Config config_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace height_mapping_ros::adapters

#endif  // HEIGHT_MAPPING_ROS_ADAPTERS_TF2_H
