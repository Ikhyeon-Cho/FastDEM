// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_ROS_TF_BRIDGE_HPP
#define FASTDEM_ROS_TF_BRIDGE_HPP

#include <ros/time.h>
#include <spdlog/spdlog.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <unordered_map>

#include "fastdem/transform_interface.hpp"

namespace fastdem::ros1 {

/**
 * @brief ROS TF2-based implementation of Calibration and Odometry.
 *
 * Implements both interfaces using ROS TF2 as the backend:
 * - Calibration: Static sensor-to-robot transform (Sensor -> Base)
 * - Odometry: Dynamic robot localization (Base -> World)
 *
 * Register with FastDEM via setTransformSystem(bridge).
 */
class TFBridge : public Calibration, public Odometry {
 public:
  TFBridge(std::string base_frame, std::string world_frame)
      : base_frame_(std::move(base_frame)),
        world_frame_(std::move(world_frame)),
        buffer_(ros::Duration(10.0)),
        listener_(buffer_) {
    ros::Duration(0.5).sleep();  // Wait for TF buffer to fill
  }

  void setMaxWaitTime(double seconds) { max_wait_time_ = seconds; }
  void setMaxStaleTime(double seconds) { max_stale_time_ = seconds; }
  void setLatestTransformFallback(bool enable) {
    use_latest_transform_fallback_ = enable;
  }

  ~TFBridge() override = default;

  // ========== Calibration Implementation ==========

  std::string getBaseFrame() const override { return base_frame_; }

  std::optional<Eigen::Isometry3d> getExtrinsic(
      const std::string& sensor_frame) const override {
    if (sensor_frame.empty()) {
      spdlog::warn("Empty sensor_frame in getExtrinsic()");
      return std::nullopt;
    }

    // Return cached extrinsic if available (static TF doesn't change)
    if (auto it = extrinsic_cache_.find(sensor_frame);
        it != extrinsic_cache_.end()) {
      return it->second;
    }

    auto tf_msg_opt =
        lookupTransformMsg(base_frame_, sensor_frame, ros::Time(0));
    if (!tf_msg_opt) {
      return std::nullopt;
    }

    auto T = tf2::transformToEigen(tf_msg_opt.value());
    extrinsic_cache_[sensor_frame] = T;
    return T;
  }

  // ========== Odometry Implementation ==========

  std::string getWorldFrame() const override { return world_frame_; }

  std::optional<Eigen::Isometry3d> getPoseAt(
      uint64_t timestamp_ns) const override {
    ros::Time lookup_time;
    lookup_time.fromNSec(timestamp_ns);

    auto tf_msg_opt =
        lookupTransformMsg(world_frame_, base_frame_, lookup_time);

    // Validate: lookup succeeded and within time tolerance
    if (tf_msg_opt) {
      double time_diff =
          std::abs((lookup_time - tf_msg_opt->header.stamp).toSec());
      if (time_diff > max_stale_time_) {
        spdlog::warn(
            "Robot pose time difference too large: {} sec (max: {} sec)",
            time_diff, max_stale_time_);
        tf_msg_opt.reset();
      }
    }

    // Fallback to latest transform if enabled
    if (!tf_msg_opt && use_latest_transform_fallback_) {
      spdlog::warn("Using latest transform as fallback for robot pose");
      tf_msg_opt = lookupTransformMsg(world_frame_, base_frame_, ros::Time(0));
    }

    if (!tf_msg_opt) {
      return std::nullopt;
    }

    return tf2::transformToEigen(tf_msg_opt.value());
  }

 private:
  std::optional<geometry_msgs::TransformStamped> lookupTransformMsg(
      const std::string& target_frame, const std::string& source_frame,
      const ros::Time& time) const {
    try {
      return buffer_.lookupTransform(target_frame, source_frame, time,
                                     ros::Duration(max_wait_time_));
    } catch (const tf2::TransformException& ex) {
      spdlog::warn("TF lookup failed: {}", ex.what());
      return std::nullopt;
    }
  }

  std::string base_frame_;
  std::string world_frame_;
  double max_wait_time_{0.1};
  double max_stale_time_{0.1};
  bool use_latest_transform_fallback_{false};

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  mutable std::unordered_map<std::string, Eigen::Isometry3d> extrinsic_cache_;
};

}  // namespace fastdem::ros1

#endif  // FASTDEM_ROS_TF_BRIDGE_HPP
