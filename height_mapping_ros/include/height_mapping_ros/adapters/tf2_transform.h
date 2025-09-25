/*
 * tf2_transform.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_ADAPTERS_TF2_TRANSFORM_PROVIDER_H
#define HEIGHT_MAPPING_ROS_ADAPTERS_TF2_TRANSFORM_PROVIDER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

#include "height_mapping_core/data/transform.h"
#include "height_mapping_pipeline/interfaces/transform_provider.h"
#include <logger/logger.h>

namespace height_mapping::ros {

static constexpr const char *ADAPTER_NAME = "TF2Transform";

class TF2Transform : public mapping::ITransformProvider {
public:
  // Constructor that creates its own buffer and listener
  TF2Transform()
      : owned_buffer_(std::make_unique<tf2_ros::Buffer>(
            ::ros::Duration(10.0))), // 10 sec cache
        buffer_(*owned_buffer_),
        listener_(std::make_unique<tf2_ros::TransformListener>(buffer_)) {

    // Wait a bit for TF buffer to fill
    ::ros::Duration(0.5).sleep();
  }

  // Constructor that uses an external buffer (for sharing between nodes)
  explicit TF2Transform(tf2_ros::Buffer &buffer) : buffer_(buffer) {}

  ~TF2Transform() override = default;

  // ITransformProvider interface
  std::optional<Transform3D> lookupTransform(const std::string &target_frame,
                                             const std::string &source_frame,
                                             uint64_t timestamp_ns) override {

    // Validate frames
    if (target_frame.empty() || source_frame.empty()) {
      LOG_WARN_ONCE(ADAPTER_NAME,
                    "Empty frame_id in transform lookup (target: '",
                    target_frame, "', source: '", source_frame, "')");
      return std::nullopt;
    }

    try {
      ::ros::Time stamp;
      stamp.fromNSec(timestamp_ns);

      // First try to get transform at exact timestamp
      // This will interpolate if necessary when transforms bracket the
      // timestamp
      geometry_msgs::TransformStamped tf_msg = buffer_.lookupTransform(
          target_frame, source_frame, stamp, ::ros::Duration(timeout_));

      // Check if the transform is too old (more than max_extrapolation_time)
      double time_diff = std::abs((stamp - tf_msg.header.stamp).toSec());
      if (time_diff > max_extrapolation_time_) {
        LOG_WARN_THROTTLE(1.0, ADAPTER_NAME,
                          "Transform time difference too large: ", time_diff,
                          " sec (max: ", max_extrapolation_time_, " sec) for ",
                          source_frame, "->", target_frame);

        // Try to use latest transform if enabled
        if (use_latest_transform_fallback_) {
          LOG_WARN_THROTTLE(1.0, ADAPTER_NAME,
                            "Using latest transform as fallback");
          return lookupLatestTransform(target_frame, source_frame);
        }
        return std::nullopt;
      }

      return convertToCore(tf_msg);

    } catch (const tf2::TransformException &ex) {
      LOG_WARN_THROTTLE(1.0, ADAPTER_NAME, "TF lookup failed: ", ex.what());

      // Try to use latest transform if enabled and within tolerance
      if (use_latest_transform_fallback_) {
        LOG_WARN_THROTTLE(1.0, ADAPTER_NAME,
                          "Using latest transform as fallback");
        return lookupLatestTransform(target_frame, source_frame);
      }
      return std::nullopt;
    }
  }

  std::optional<Transform3D>
  lookupLatestTransform(const std::string &target_frame,
                        const std::string &source_frame) override {

    // Validate frames
    if (target_frame.empty() || source_frame.empty()) {
      LOG_WARN_ONCE(ADAPTER_NAME,
                    "Empty frame_id in transform lookup (target: '",
                    target_frame, "', source: '", source_frame, "')");
      return std::nullopt;
    }

    try {
      geometry_msgs::TransformStamped tf_msg =
          buffer_.lookupTransform(target_frame, source_frame, ::ros::Time(0),
                                  ::ros::Duration(timeout_));

      return convertToCore(tf_msg);

    } catch (const tf2::TransformException &ex) {
      LOG_WARN_THROTTLE(1.0, ADAPTER_NAME, "TF lookup failed: ", ex.what());
      return std::nullopt;
    }
  }

  // Additional ROS-specific methods
  bool canTransform(const std::string &target_frame,
                    const std::string &source_frame,
                    const ::ros::Time &time = ::ros::Time(0)) const {

    return buffer_.canTransform(target_frame, source_frame, time);
  }

  void setLookupTimeout(double timeout_seconds) { timeout_ = timeout_seconds; }

  void setMaxExtrapolationTime(double max_time_seconds) {
    max_extrapolation_time_ = max_time_seconds;
  }

  void setUseLatestTransformFallback(bool use_fallback) {
    use_latest_transform_fallback_ = use_fallback;
  }

  // Wait for transform to become available
  bool waitForTransform(const std::string &target_frame,
                        const std::string &source_frame,
                        const ::ros::Time &time,
                        const ::ros::Duration &timeout) {

    try {
      buffer_.lookupTransform(target_frame, source_frame, time, timeout);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

private:
  Transform3D
  convertToCore(const geometry_msgs::TransformStamped &tf_msg) const {
    // Convert to Eigen
    Eigen::Isometry3d eigen_tf = tf2::transformToEigen(tf_msg);

    // Convert to our Transform3D
    Transform3D result(eigen_tf.matrix().cast<float>());

    return result;
  }

private:
  std::unique_ptr<tf2_ros::Buffer>
      owned_buffer_;        // Only used if we create our own
  tf2_ros::Buffer &buffer_; // Reference to buffer (owned or external)
  std::unique_ptr<tf2_ros::TransformListener>
      listener_;         // Only used if we create our own
  double timeout_ = 0.1; // Default lookup timeout in seconds
  double max_extrapolation_time_ =
      0.1; // Maximum time difference for transform (seconds)
  bool use_latest_transform_fallback_ =
      false; // Whether to use latest transform as fallback
};

// Simplified transform provider for static transforms
class StaticTransformProvider : public mapping::ITransformProvider {
public:
  void addStaticTransform(const std::string &target_frame,
                          const std::string &source_frame,
                          const Transform3D &transform) {

    std::string key = target_frame + "_" + source_frame;
    static_transforms_[key] = transform;

    // Also add inverse
    std::string inverse_key = source_frame + "_" + target_frame;
    static_transforms_[inverse_key] = transform.inverse();
  }

  std::optional<Transform3D>
  lookupTransform(const std::string &target_frame,
                  const std::string &source_frame,
                  uint64_t /*timestamp_ns*/) override {

    return lookupLatestTransform(target_frame, source_frame);
  }

  std::optional<Transform3D>
  lookupLatestTransform(const std::string &target_frame,
                        const std::string &source_frame) override {

    std::string key = target_frame + "_" + source_frame;
    auto it = static_transforms_.find(key);

    if (it != static_transforms_.end()) {
      return it->second;
    }

    return std::nullopt;
  }

private:
  std::unordered_map<std::string, Transform3D> static_transforms_;
};

} // namespace height_mapping::ros

#endif // HEIGHT_MAPPING_ROS_ADAPTERS_TF2_TRANSFORM_PROVIDER_H