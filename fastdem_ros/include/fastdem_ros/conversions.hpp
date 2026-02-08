// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_ROS_CONVERSIONS_HPP
#define FASTDEM_ROS_CONVERSIONS_HPP

#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <fastdem/elevation_map.hpp>
#include <nanopcl/bridge/ros1.hpp>
#include <std_msgs/Float32MultiArray.h>

namespace fastdem::ros1 {

/**
 * @brief Convert ElevationMap to ROS message
 * @param map Source height map
 * @param msg Output ROS message
 */
inline void toMessage(const fastdem::ElevationMap& map,
                      grid_map_msgs::GridMap& msg) {
  // Header
  msg.info.header.stamp.fromNSec(map.getTimestamp());
  msg.info.header.frame_id = map.getFrameId();

  // Geometry
  msg.info.resolution = map.getResolution();
  msg.info.length_x = map.getLength().x();
  msg.info.length_y = map.getLength().y();
  msg.info.pose.position.x = map.getPosition().x();
  msg.info.pose.position.y = map.getPosition().y();
  msg.info.pose.position.z = 0.0;
  msg.info.pose.orientation.w = 1.0;
  msg.info.pose.orientation.x = 0.0;
  msg.info.pose.orientation.y = 0.0;
  msg.info.pose.orientation.z = 0.0;

  // Layers
  msg.layers = map.getLayers();
  msg.basic_layers = map.getBasicLayers();

  // Data
  msg.data.clear();
  for (const auto& layer : map.getLayers()) {
    std_msgs::Float32MultiArray data_array;

    // Layout
    data_array.layout.dim.resize(2);
    data_array.layout.dim[0].label = "column_index";
    data_array.layout.dim[0].size = map.getSize()(0);
    data_array.layout.dim[0].stride = map.getSize()(0) * map.getSize()(1);
    data_array.layout.dim[1].label = "row_index";
    data_array.layout.dim[1].size = map.getSize()(1);
    data_array.layout.dim[1].stride = map.getSize()(1);

    // Copy data (Eigen column-major to row-major)
    const auto& layer_data = map.get(layer);
    data_array.data.resize(layer_data.size());

    // grid_map uses column-major (Eigen default), ROS message expects row-major
    size_t idx = 0;
    for (Eigen::Index col = 0; col < layer_data.cols(); ++col) {
      for (Eigen::Index row = 0; row < layer_data.rows(); ++row) {
        data_array.data[idx++] = layer_data(row, col);
      }
    }

    msg.data.push_back(data_array);
  }

  // Outer/start index for circular buffer
  msg.outer_start_index = map.getStartIndex()(0);
  msg.inner_start_index = map.getStartIndex()(1);
}

/**
 * @brief Convert ElevationMap to PointCloud2 for RViz visualization
 *
 * Iterates the elevation layer in column-major order (Eigen-native) with
 * precomputed per-row/col world coordinates to avoid per-cell getPosition().
 */
inline sensor_msgs::PointCloud2 toPointCloud2(
    const fastdem::ElevationMap& map) {
  const auto& elev = map.get(fastdem::layer::elevation);
  const bool has_intensity = map.exists(fastdem::layer::intensity);
  const bool has_color = map.exists(fastdem::layer::color);
  const auto size = map.getSize();
  const Eigen::Index rows = size(0);
  const Eigen::Index cols = size(1);
  const auto startIdx = map.getStartIndex();
  const double res = map.getResolution();

  // Precompute world coordinates per row (x) and per col (y)
  const double origin_x =
      map.getPosition().x() + map.getLength().x() / 2.0 - res / 2.0;
  const double origin_y =
      map.getPosition().y() + map.getLength().y() / 2.0 - res / 2.0;

  std::vector<float> row_x(rows);
  for (Eigen::Index r = 0; r < rows; ++r) {
    int unwrapped = (r - startIdx(0) + rows) % rows;
    row_x[r] = static_cast<float>(origin_x - unwrapped * res);
  }

  std::vector<float> col_y(cols);
  for (Eigen::Index c = 0; c < cols; ++c) {
    int unwrapped = (c - startIdx(1) + cols) % cols;
    col_y[c] = static_cast<float>(origin_y - unwrapped * res);
  }

  // Build point cloud (single pass, column-major for cache efficiency)
  nanopcl::PointCloud cloud;
  if (has_intensity) cloud.useIntensity();
  if (has_color) cloud.useColor();
  cloud.reserve(static_cast<size_t>(rows * cols));

  const float* elev_data = elev.data();
  const float* intensity_data =
      has_intensity ? map.get(fastdem::layer::intensity).data() : nullptr;
  const float* color_data =
      has_color ? map.get(fastdem::layer::color).data() : nullptr;

  for (Eigen::Index col = 0; col < cols; ++col) {
    const float y = col_y[col];
    const Eigen::Index col_offset = col * rows;
    for (Eigen::Index row = 0; row < rows; ++row) {
      const float z = elev_data[col_offset + row];
      if (!std::isfinite(z)) continue;

      cloud.add(row_x[row], y, z);

      const size_t pidx = cloud.size() - 1;
      if (intensity_data) {
        cloud.intensity(pidx) = intensity_data[col_offset + row];
      }
      if (color_data) {
        uint32_t rgba;
        std::memcpy(&rgba, &color_data[col_offset + row], sizeof(uint32_t));
        cloud.color(pidx) = {static_cast<uint8_t>((rgba >> 16) & 0xFF),
                             static_cast<uint8_t>((rgba >> 8) & 0xFF),
                             static_cast<uint8_t>(rgba & 0xFF)};
      }
    }
  }

  return nanopcl::to(cloud, map.getFrameId(),
                     ros::Time().fromNSec(map.getTimestamp()));
}

/**
 * @brief Create map boundary marker for RViz visualization
 */
inline visualization_msgs::Marker toRegionMarker(
    const fastdem::ElevationMap& map) {
  visualization_msgs::Marker marker;
  marker.header.stamp.fromNSec(map.getTimestamp());
  marker.header.frame_id = map.getFrameId();
  marker.ns = "fastdem";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.pose.orientation.w = 1.0;

  const double cx = map.getPosition().x();
  const double cy = map.getPosition().y();
  const double hx = map.getLength().x() / 2.0;
  const double hy = map.getLength().y() / 2.0;

  geometry_msgs::Point p;
  p.z = 0.0;
  const double corners[][2] = {{cx - hx, cy - hy},
                                {cx + hx, cy - hy},
                                {cx + hx, cy + hy},
                                {cx - hx, cy + hy},
                                {cx - hx, cy - hy}};
  for (const auto& c : corners) {
    p.x = c[0];
    p.y = c[1];
    marker.points.push_back(p);
  }

  return marker;
}

}  // namespace fastdem::ros1

#endif  // FASTDEM_ROS_CONVERSIONS_HPP
