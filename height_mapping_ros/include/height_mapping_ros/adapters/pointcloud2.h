/*
 * pointcloud2.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_ADAPTERS_POINTCLOUD2_H
#define HEIGHT_MAPPING_ROS_ADAPTERS_POINTCLOUD2_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>

#include <cmath>

#include "height_mapping/types.h"

namespace height_mapping_ros::adapters {

using height_mapping::Color;
using height_mapping::Point;
using height_mapping::PointCloud;

// Get point count from ROS message without conversion
inline size_t getPointCount(const sensor_msgs::PointCloud2 &msg) {
  return msg.width * msg.height;
}

// Convert ROS message to core point cloud (returns shared_ptr for zero-copy pipeline)
inline std::shared_ptr<PointCloud> fromPointCloud2(
    const sensor_msgs::PointCloud2 &ros_msg) {
  // Reserve space
  const size_t num_points = ros_msg.width * ros_msg.height;
  auto cloud = std::make_shared<PointCloud>(num_points);

  // Set metadata
  cloud->setTimestamp(ros_msg.header.stamp.toNSec());
  cloud->setFrameId(ros_msg.header.frame_id);

  // Direct access to point data for efficiency
  const uint8_t *data_ptr = ros_msg.data.data();
  const size_t point_step = ros_msg.point_step;

  // Find field offsets
  int x_offset = -1, y_offset = -1, z_offset = -1;
  int intensity_offset = -1;
  int r_offset = -1, g_offset = -1, b_offset = -1;
  int rgb_offset = -1;

  for (const auto &field : ros_msg.fields) {
    if (field.name == "x")
      x_offset = field.offset;
    else if (field.name == "y")
      y_offset = field.offset;
    else if (field.name == "z")
      z_offset = field.offset;
    else if (field.name == "intensity")
      intensity_offset = field.offset;
    else if (field.name == "r")
      r_offset = field.offset;
    else if (field.name == "g")
      g_offset = field.offset;
    else if (field.name == "b")
      b_offset = field.offset;
    else if (field.name == "rgb")
      rgb_offset = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    spdlog::error("Point cloud missing x, y, or z fields");
    return cloud;  // Return empty cloud
  }

  // Check which channels are available
  const bool has_intensity = (intensity_offset >= 0);
  const bool has_rgb_separate =
      (r_offset >= 0 && g_offset >= 0 && b_offset >= 0);
  const bool has_rgb_packed = (rgb_offset >= 0);
  const bool has_color = has_rgb_separate || has_rgb_packed;

  // Enable channels if present
  if (has_intensity) cloud->enableIntensity();
  if (has_color) cloud->enableColor();

  // Extract points and sensor data
  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t *point_ptr = data_ptr + i * point_step;

    const float *x = reinterpret_cast<const float *>(point_ptr + x_offset);
    const float *y = reinterpret_cast<const float *>(point_ptr + y_offset);
    const float *z = reinterpret_cast<const float *>(point_ptr + z_offset);

    // Skip invalid points
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }

    // Add point
    cloud->points.push_back(Point(*x, *y, *z));

    // Extract intensity if available
    if (has_intensity) {
      const float *intensity =
          reinterpret_cast<const float *>(point_ptr + intensity_offset);
      cloud->intensity().push_back(std::isfinite(*intensity) ? *intensity
                                                             : 0.0f);
    }

    // Extract color if available
    if (has_rgb_separate) {
      const uint8_t *r = point_ptr + r_offset;
      const uint8_t *g = point_ptr + g_offset;
      const uint8_t *b = point_ptr + b_offset;
      cloud->color().push_back(Color{*r, *g, *b});
    } else if (has_rgb_packed) {
      const uint32_t *rgb =
          reinterpret_cast<const uint32_t *>(point_ptr + rgb_offset);
      uint8_t r = (*rgb >> 16) & 0xFF;
      uint8_t g = (*rgb >> 8) & 0xFF;
      uint8_t b = (*rgb) & 0xFF;
      cloud->color().push_back(Color{r, g, b});
    }
  }

  return cloud;
}

// Convert core point cloud to ROS message
inline sensor_msgs::PointCloud2 toPointCloud2(const PointCloud &cloud) {
  sensor_msgs::PointCloud2 ros_msg;

  // Set metadata
  ros_msg.header.stamp.fromNSec(cloud.timestamp());
  ros_msg.header.frame_id = cloud.frameId();

  // Set dimensions
  ros_msg.height = 1;  // Unorganized cloud
  ros_msg.width = cloud.size();
  ros_msg.is_bigendian = false;
  ros_msg.is_dense = false;  // May contain NaN/Inf

  // Determine point structure based on available sensor data
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  // Helper lambda to add fields
  auto addField = [&ros_msg](const std::string &name, size_t &offset,
                             uint8_t datatype, uint8_t count = 1) {
    sensor_msgs::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    ros_msg.fields.push_back(field);
    offset += (datatype == sensor_msgs::PointField::FLOAT32) ? 4 : count;
  };

  // Build fields based on available data
  size_t offset = 0;
  ros_msg.fields.clear();

  // XYZ fields (always present)
  addField("x", offset, sensor_msgs::PointField::FLOAT32);
  addField("y", offset, sensor_msgs::PointField::FLOAT32);
  addField("z", offset, sensor_msgs::PointField::FLOAT32);

  // Add intensity field if present
  if (has_intensity) {
    addField("intensity", offset, sensor_msgs::PointField::FLOAT32);
  }

  // Add RGB fields if present
  if (has_color) {
    addField("r", offset, sensor_msgs::PointField::UINT8);
    addField("g", offset, sensor_msgs::PointField::UINT8);
    addField("b", offset, sensor_msgs::PointField::UINT8);
    offset += 1;  // Padding for alignment
  }

  // Set strides
  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.point_step * ros_msg.width;

  // Allocate and fill data
  ros_msg.data.resize(ros_msg.row_step);
  uint8_t *data_ptr = ros_msg.data.data();

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &point = cloud.points[i];
    uint8_t *point_data = data_ptr + i * ros_msg.point_step;

    // Write XYZ
    *reinterpret_cast<float *>(point_data) = point.x();
    *reinterpret_cast<float *>(point_data + 4) = point.y();
    *reinterpret_cast<float *>(point_data + 8) = point.z();

    size_t current_offset = 12;

    // Write intensity if present
    if (has_intensity) {
      *reinterpret_cast<float *>(point_data + current_offset) =
          cloud.intensity()[i];
      current_offset += 4;
    }

    // Write color if present
    if (has_color) {
      const auto &color = cloud.color()[i];
      point_data[current_offset] = color[0];      // R
      point_data[current_offset + 1] = color[1];  // G
      point_data[current_offset + 2] = color[2];  // B
    }
  }

  return ros_msg;
}

// Validate ROS message
inline bool validate(const sensor_msgs::PointCloud2 &msg) {
  // Check frame ID
  if (msg.header.frame_id.empty()) {
    spdlog::warn("Point cloud has empty frame_id");
    return false;
  }

  // Check if empty
  if (msg.width * msg.height == 0) {
    spdlog::warn("Point cloud is empty");
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
    spdlog::warn("Point cloud missing required x, y, or z fields");
    return false;
  }

  return true;
}

}  // namespace height_mapping_ros::adapters

#endif  // HEIGHT_MAPPING_ROS_ADAPTERS_POINTCLOUD2_H
