/*
 * pointcloud_converter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/utils/pointcloud_converter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace height_mapping::ros::utils {

void PointCloudConverter::rosToCore(const sensor_msgs::PointCloud2 &ros_msg,
                                    PointCloudXYZ &core_cloud) {
  // Set metadata
  core_cloud.timestamp = ros_msg.header.stamp.toNSec();
  core_cloud.frame_id = ros_msg.header.frame_id;

  // Clear and reserve space
  const size_t num_points = ros_msg.width * ros_msg.height;
  core_cloud.points.clear();
  core_cloud.points.reserve(num_points);

  // Direct access to point data (avoiding PCL conversion overhead)
  const uint8_t *data_ptr = ros_msg.data.data();
  const size_t point_step = ros_msg.point_step;

  // Find field offsets
  int x_offset = -1, y_offset = -1, z_offset = -1;
  for (const auto &field : ros_msg.fields) {
    if (field.name == "x")
      x_offset = field.offset;
    else if (field.name == "y")
      y_offset = field.offset;
    else if (field.name == "z")
      z_offset = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    ROS_ERROR_THROTTLE(1.0, "Point cloud missing x, y, or z fields");
    return;
  }

  // Extract points
  for (size_t i = 0; i < num_points; ++i) {
    const float *x =
        reinterpret_cast<const float *>(data_ptr + i * point_step + x_offset);
    const float *y =
        reinterpret_cast<const float *>(data_ptr + i * point_step + y_offset);
    const float *z =
        reinterpret_cast<const float *>(data_ptr + i * point_step + z_offset);

    if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
      core_cloud.points.emplace_back(*x, *y, *z);
    }
  }
}

sensor_msgs::PointCloud2
PointCloudConverter::coreToROS(const PointCloudXYZ &core_cloud) {
  // Use PCL for convenience in creating the ROS message structure
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  coreToPCL(core_cloud, pcl_cloud);

  sensor_msgs::PointCloud2 ros_msg;
  pcl::toROSMsg(pcl_cloud, ros_msg);

  return ros_msg;
}

bool PointCloudConverter::validate(const sensor_msgs::PointCloud2 &msg) {
  // Check frame ID
  if (msg.header.frame_id.empty()) {
    ROS_WARN_THROTTLE(1.0, "Point cloud has empty frame_id");
    return false;
  }

  // Check if empty
  if (msg.width * msg.height == 0) {
    ROS_WARN_THROTTLE(1.0, "Point cloud is empty");
    return false;
  }

  // Check for required fields
  bool has_x = false, has_y = false, has_z = false;
  for (const auto &field : msg.fields) {
    if (field.name == "x")
      has_x = true;
    else if (field.name == "y")
      has_y = true;
    else if (field.name == "z")
      has_z = true;
  }

  if (!has_x || !has_y || !has_z) {
    ROS_WARN_THROTTLE(1.0, "Point cloud missing required x, y, or z fields");
    return false;
  }

  return true;
}

} // namespace height_mapping::ros::utils