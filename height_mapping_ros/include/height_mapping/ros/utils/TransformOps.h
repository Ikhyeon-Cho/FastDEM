#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TransformOps {
public:
  explicit TransformOps() : nh_(), tf_listener_(tf_buffer_) {}
  explicit TransformOps(const ros::NodeHandle &nh) : nh_(nh), tf_listener_(tf_buffer_) {}

  /**
   * @brief Attempts to lookup the transform between two frames.
   * @param target_frame The target frame.
   * @param source_frame The source frame.
   * @param transform Output variable for the transform.
   * @param time Time at which to lookup transform.
   * @return True if successful, false otherwise.
   */
  bool lookupTransform(const std::string &target_frame,
                       const std::string &source_frame,
                       geometry_msgs::TransformStamped &transform,
                       const ros::Time &time = ros::Time(0),
                       const ros::Duration &timeout = ros::Duration(0.1)) {

    try {
      transform = tf_buffer_.lookupTransform(target_frame, source_frame, time, timeout);
      return true;
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM_DELAYED_THROTTLE(1.0, "TF lookup failed: " << ex.what());
      return false;
    }
  }

  static geometry_msgs::TransformStamped
  multiplyTransforms(const geometry_msgs::TransformStamped &transform1,
                     const geometry_msgs::TransformStamped &transform2) {
    tf2::Transform t1, t2;
    tf2::fromMsg(transform1.transform, t1);
    tf2::fromMsg(transform2.transform, t2);

    tf2::Transform t_multiplied = t1 * t2;
    geometry_msgs::TransformStamped transform_multiplied;
    transform_multiplied.transform = tf2::toMsg(t_multiplied);
    transform_multiplied.header.frame_id = transform2.header.frame_id;
    transform_multiplied.header.stamp = transform2.header.stamp;
    transform_multiplied.child_frame_id = transform1.child_frame_id;
    return transform_multiplied;
  }

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
