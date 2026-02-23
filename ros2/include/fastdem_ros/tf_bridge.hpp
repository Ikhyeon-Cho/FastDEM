// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_ROS_TF_BRIDGE_HPP
#define FASTDEM_ROS_TF_BRIDGE_HPP

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <spdlog/spdlog.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <unordered_map>

#include "fastdem/transform_interface.hpp"

namespace fastdem::ros2 {

/**
 * @brief ROS2 TF2-based implementation of Calibration and Odometry.
 *
 * Implements both interfaces using ROS2 TF2 as the backend:
 * - Calibration: Static sensor-to-robot transform (Sensor -> Base)
 * - Odometry: Dynamic robot localization (Base -> World)
 *
 * Register with FastDEM via setTransformSystem(bridge).
 */
class TFBridge : public Calibration, public Odometry {
 public:
  TFBridge(rclcpp::Clock::SharedPtr clock, std::string base_frame,
           std::string world_frame, double max_wait_time = 0.1,
           double max_stale_time = 0.1,
           bool use_latest_transform_fallback = false)
      : base_frame_(std::move(base_frame)),
        world_frame_(std::move(world_frame)),
        max_wait_time_(max_wait_time),
        max_stale_time_(max_stale_time),
        use_latest_transform_fallback_(use_latest_transform_fallback),
        buffer_(clock),
        listener_(buffer_) {}

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
        lookupTransformMsg(base_frame_, sensor_frame, tf2::TimePointZero);
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
    auto lookup_time =
        tf2::TimePoint(std::chrono::nanoseconds(timestamp_ns));

    auto tf_msg_opt =
        lookupTransformMsg(world_frame_, base_frame_, lookup_time);

    // Validate: lookup succeeded and within time tolerance
    // Skip staleness check when timestamp_ns == 0 (latest transform request)
    if (tf_msg_opt && timestamp_ns != 0) {
      auto stamp_ns =
          static_cast<uint64_t>(tf_msg_opt->header.stamp.sec) * 1000000000ULL +
          static_cast<uint64_t>(tf_msg_opt->header.stamp.nanosec);
      double time_diff =
          std::abs(static_cast<double>(timestamp_ns) -
                   static_cast<double>(stamp_ns)) /
          1e9;
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
      tf_msg_opt =
          lookupTransformMsg(world_frame_, base_frame_, tf2::TimePointZero);
    }

    if (!tf_msg_opt) {
      return std::nullopt;
    }

    return tf2::transformToEigen(tf_msg_opt.value());
  }

 private:
  std::optional<geometry_msgs::msg::TransformStamped> lookupTransformMsg(
      const std::string& target_frame, const std::string& source_frame,
      const tf2::TimePoint& time) const {
    try {
      return buffer_.lookupTransform(
          target_frame, source_frame, time,
          tf2::durationFromSec(max_wait_time_));
    } catch (const tf2::TransformException& ex) {
      spdlog::warn("TF lookup failed: {}", ex.what());
      return std::nullopt;
    }
  }

  std::string base_frame_;
  std::string world_frame_;
  double max_wait_time_;
  double max_stale_time_;
  bool use_latest_transform_fallback_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  mutable std::unordered_map<std::string, Eigen::Isometry3d>
      extrinsic_cache_;
};

}  // namespace fastdem::ros2

#endif  // FASTDEM_ROS_TF_BRIDGE_HPP
