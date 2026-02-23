// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_BRIDGE_ROS2_HPP
#define FASTDEM_BRIDGE_ROS2_HPP

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/time.hpp>

#include <fastdem/bridge/ros/impl.hpp>
#include <fastdem/bridge/ros/impl_visualization.hpp>

namespace fastdem::ros2 {

namespace detail {
inline rclcpp::Time toStamp(uint64_t ns) {
  return rclcpp::Time(static_cast<int64_t>(ns));
}
}  // namespace detail

/// Convert ElevationMap to PointCloud2 with all non-internal layers as fields.
inline sensor_msgs::msg::PointCloud2 toPointCloud2(
    const ElevationMap& map,
    const char* elevation_layer = layer::elevation) {
  return fastdem::detail::toPointCloud2Impl<sensor_msgs::msg::PointCloud2,
                                   sensor_msgs::msg::PointField>(
      map, detail::toStamp(map.getTimestamp()), elevation_layer);
}

/// Convert a submap region to PointCloud2 (zero-copy from parent map).
inline sensor_msgs::msg::PointCloud2 toPointCloud2(
    const ElevationMap& map, const grid_map::Position& center,
    const grid_map::Length& length,
    const char* elevation_layer = layer::elevation) {
  bool ok = false;
  grid_map::SubmapGeometry geom(map, center, length, ok);
  if (!ok) return {};
  return fastdem::detail::toPointCloud2Impl<sensor_msgs::msg::PointCloud2,
                                   sensor_msgs::msg::PointField>(
      map, detail::toStamp(map.getTimestamp()), elevation_layer,
      geom.getStartIndex(), geom.getSize());
}

/// Convert ElevationMap to grid_map_msgs::msg::GridMap.
inline grid_map_msgs::msg::GridMap toGridMap(const ElevationMap& map) {
  grid_map_msgs::msg::GridMap msg;
  msg.header.stamp = detail::toStamp(map.getTimestamp());
  msg.header.frame_id = map.getFrameId();
  fastdem::detail::fillGridMapMsg<grid_map_msgs::msg::GridMap,
                         std_msgs::msg::Float32MultiArray>(msg, map);
  return msg;
}

/// Create map boundary marker for RViz visualization.
inline visualization_msgs::msg::Marker toMapBoundary(const ElevationMap& map) {
  return fastdem::detail::toMarkerImpl<visualization_msgs::msg::Marker,
                              geometry_msgs::msg::Point>(
      map, detail::toStamp(map.getTimestamp()));
}

/// Convert ElevationMap normal vectors to MarkerArray for RViz visualization.
inline visualization_msgs::msg::MarkerArray toNormalMarkers(
    const ElevationMap& map, float arrow_length = 0.15f, int stride = 1) {
  return fastdem::detail::toNormalMarkersImpl<
      visualization_msgs::msg::MarkerArray, visualization_msgs::msg::Marker,
      geometry_msgs::msg::Point, std_msgs::msg::ColorRGBA>(
      map, detail::toStamp(map.getTimestamp()), arrow_length, stride);
}

}  // namespace fastdem::ros2

#endif  // FASTDEM_BRIDGE_ROS2_HPP
