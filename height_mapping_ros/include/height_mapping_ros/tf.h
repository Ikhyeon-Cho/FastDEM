/*
 * tf.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_TF_H
#define HEIGHT_MAPPING_ROS_TF_H

#include <ros/time.h>
#include <spdlog/spdlog.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <optional>

#include "height_mapping/api/transform_providers.h"
#include "height_mapping/core.h"

namespace height_mapping_ros {

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
 * - Extrinsics: Static sensor-to-robot calibration (Sensor -> Base)
 * - Robot Pose: Dynamic robot localization (Base -> Map)
 *
 * While TF2 handles both as transforms, this class provides a clean
 * semantic separation at the interface level.
 */
class TFLookup : public IExtrinsicsProvider, public IRobotPoseProvider {
 public:
  TFLookup(std::string base_frame, std::string map_frame,
           double max_wait_time = 0.1, double max_stale_time = 0.1,
           bool use_latest_transform_fallback = false)
      : base_frame_(std::move(base_frame)),
        map_frame_(std::move(map_frame)),
        max_wait_time_(max_wait_time),
        max_stale_time_(max_stale_time),
        use_latest_transform_fallback_(use_latest_transform_fallback),
        buffer_(ros::Duration(10.0)),
        listener_(buffer_) {
    ros::Duration(0.5).sleep();  // Wait for TF buffer to fill
  }

  ~TFLookup() override = default;

  // ========== IExtrinsicsProvider Implementation ==========

  std::string getBaseFrame() const override { return base_frame_; }

  /**
   * @brief Get sensor extrinsic calibration (Sensor -> Base)
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
        lookupTransformMsg(base_frame_, sensor_frame, ros::Time(0));

    if (!tf_msg_opt) {
      return std::nullopt;
    }

    return convertToTransform(tf_msg_opt.value(), sensor_frame, base_frame_);
  }

  // ========== IRobotPoseProvider Implementation ==========

  std::string getMapFrame() const override { return map_frame_; }

  /**
   * @brief Get robot pose in global frame (Base -> Map)
   *
   * This returns the dynamic transform from base_link to map frame.
   * Uses the provided timestamp for temporal consistency with sensor data.
   */
  std::optional<Transformf> getRobotPoseAt(uint64_t timestamp_ns) override {
    ros::Time lookup_time;
    lookup_time.fromNSec(timestamp_ns);

    // Try exact time lookup first
    auto tf_msg_opt = lookupTransformMsg(map_frame_, base_frame_, lookup_time);

    if (!tf_msg_opt) {
      // Fallback to latest if enabled
      if (use_latest_transform_fallback_) {
        spdlog::warn("Using latest transform as fallback for robot pose");
        tf_msg_opt = lookupTransformMsg(map_frame_, base_frame_, ros::Time(0));
      }
      if (!tf_msg_opt) {
        return std::nullopt;
      }
    }

    // Check time tolerance
    const auto& tf_msg = tf_msg_opt.value();
    double time_diff = std::abs((lookup_time - tf_msg.header.stamp).toSec());
    if (time_diff > max_stale_time_) {
      spdlog::warn("Robot pose time difference too large: {} sec (max: {} sec)",
                   time_diff, max_stale_time_);

      if (use_latest_transform_fallback_) {
        tf_msg_opt = lookupTransformMsg(map_frame_, base_frame_, ros::Time(0));
        if (!tf_msg_opt) {
          return std::nullopt;
        }
      } else {
        return std::nullopt;
      }
    }

    return convertToTransform(tf_msg_opt.value(), base_frame_, map_frame_);
  }

 private:
  // Helper: Lookup transform message with exception handling
  std::optional<geometry_msgs::TransformStamped> lookupTransformMsg(
      const std::string& target_frame, const std::string& source_frame,
      const ros::Time& time) {
    try {
      return buffer_.lookupTransform(target_frame, source_frame, time,
                                     ros::Duration(max_wait_time_));
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
  std::string base_frame_;
  std::string map_frame_;
  double max_wait_time_;
  double max_stale_time_;
  bool use_latest_transform_fallback_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace height_mapping_ros

#endif  // HEIGHT_MAPPING_ROS_TF_H
