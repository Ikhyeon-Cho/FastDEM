/*
 * pointcloud_converter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/adapters/pointcloud_converter.h"
#include <ros/ros.h>

namespace height_mapping::ros::adapters {

std::shared_ptr<PointCloud> fromROS(const sensor_msgs::PointCloud2 &ros_msg) {

  // Reserve space
  const size_t num_points = ros_msg.width * ros_msg.height;
  auto core_cloud = std::make_shared<PointCloud>(num_points);

  // Set metadata
  core_cloud->setTimestamp(ros_msg.header.stamp.toNSec());
  core_cloud->setFrameId(ros_msg.header.frame_id);

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
    ROS_ERROR_THROTTLE(1.0, "Point cloud missing x, y, or z fields");
    return core_cloud; // Return empty cloud
  }

  // Prepare sensor data containers if fields exist
  std::vector<float> intensity_data;
  std::vector<std::array<uint8_t, 3>> color_data;

  const bool has_intensity = (intensity_offset >= 0);
  const bool has_rgb_separate =
      (r_offset >= 0 && g_offset >= 0 && b_offset >= 0);
  const bool has_rgb_packed = (rgb_offset >= 0);
  const bool has_color = has_rgb_separate || has_rgb_packed;

  if (has_intensity) {
    intensity_data.reserve(num_points);
  }
  if (has_color) {
    color_data.reserve(num_points);
  }

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
    core_cloud->emplace_back(*x, *y, *z);

    // Extract intensity if available
    if (has_intensity) {
      const float *intensity =
          reinterpret_cast<const float *>(point_ptr + intensity_offset);
      intensity_data.push_back(std::isfinite(*intensity) ? *intensity : 0.0f);
    }

    // Extract color if available
    if (has_rgb_separate) {
      const uint8_t *r = point_ptr + r_offset;
      const uint8_t *g = point_ptr + g_offset;
      const uint8_t *b = point_ptr + b_offset;
      color_data.push_back({*r, *g, *b});
    } else if (has_rgb_packed) {
      const uint32_t *rgb =
          reinterpret_cast<const uint32_t *>(point_ptr + rgb_offset);
      uint8_t r = (*rgb >> 16) & 0xFF;
      uint8_t g = (*rgb >> 8) & 0xFF;
      uint8_t b = (*rgb) & 0xFF;
      color_data.push_back({r, g, b});
    }
  }

  // Attach sensor data if present
  if (has_intensity && !intensity_data.empty()) {
    core_cloud->attachIntensity(std::move(intensity_data));
  }
  if (has_color && !color_data.empty()) {
    core_cloud->attachColor(std::move(color_data));
  }

  return core_cloud;
}

sensor_msgs::PointCloud2 toROS(const PointCloud &core_cloud) {
  sensor_msgs::PointCloud2 ros_msg;

  // Set metadata
  ros_msg.header.stamp.fromNSec(core_cloud.timestamp());
  ros_msg.header.frame_id = core_cloud.frameId();

  // Set dimensions
  ros_msg.height = 1; // Unorganized cloud
  ros_msg.width = core_cloud.size();
  ros_msg.is_bigendian = false;
  ros_msg.is_dense = false; // May contain NaN/Inf

  // Determine point structure based on available sensor data
  const bool has_intensity = core_cloud.hasIntensity();
  const bool has_color = core_cloud.hasColor();

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
    offset += 1; // Padding for alignment
  }

  // Set strides
  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.point_step * ros_msg.width;

  // Allocate and fill data
  ros_msg.data.resize(ros_msg.row_step);
  uint8_t *data_ptr = ros_msg.data.data();

  for (size_t i = 0; i < core_cloud.size(); ++i) {
    auto point = core_cloud[i]; // PointView for unified access
    uint8_t *point_data = data_ptr + i * ros_msg.point_step;

    // Write XYZ
    *reinterpret_cast<float *>(point_data) = point.x();
    *reinterpret_cast<float *>(point_data + 4) = point.y();
    *reinterpret_cast<float *>(point_data + 8) = point.z();

    size_t current_offset = 12;

    // Write intensity if present
    if (has_intensity) {
      *reinterpret_cast<float *>(point_data + current_offset) =
          point.intensity();
      current_offset += 4;
    }

    // Write color if present
    if (has_color) {
      const auto color = point.color();
      point_data[current_offset] = color[0];     // R
      point_data[current_offset + 1] = color[1]; // G
      point_data[current_offset + 2] = color[2]; // B
    }
  }

  return ros_msg;
}

bool validate(const sensor_msgs::PointCloud2 &msg) {
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

} // namespace height_mapping::ros::adapters