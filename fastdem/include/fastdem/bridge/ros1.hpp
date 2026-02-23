// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_BRIDGE_ROS1_HPP
#define FASTDEM_BRIDGE_ROS1_HPP

#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fastdem/bridge/ros/impl.hpp>
#include <fastdem/bridge/ros/impl_visualization.hpp>

namespace fastdem::ros1 {

namespace detail {
inline ros::Time toStamp(uint64_t ns) {
  ros::Time t;
  t.fromNSec(ns);
  return t;
}
}  // namespace detail

/// Convert ElevationMap to PointCloud2 with all non-internal layers as fields.
inline sensor_msgs::PointCloud2 toPointCloud2(
    const ElevationMap& map,
    const char* elevation_layer = layer::elevation) {
  return fastdem::detail::toPointCloud2Impl<sensor_msgs::PointCloud2,
                                   sensor_msgs::PointField>(
      map, detail::toStamp(map.getTimestamp()), elevation_layer);
}

/// Convert a submap region to PointCloud2 (zero-copy from parent map).
inline sensor_msgs::PointCloud2 toPointCloud2(
    const ElevationMap& map, const grid_map::Position& center,
    const grid_map::Length& length,
    const char* elevation_layer = layer::elevation) {
  bool ok = false;
  grid_map::SubmapGeometry geom(map, center, length, ok);
  if (!ok) return {};
  return fastdem::detail::toPointCloud2Impl<sensor_msgs::PointCloud2,
                                   sensor_msgs::PointField>(
      map, detail::toStamp(map.getTimestamp()), elevation_layer,
      geom.getStartIndex(), geom.getSize());
}

/// Convert ElevationMap to grid_map_msgs::GridMap.
inline grid_map_msgs::GridMap toGridMap(const ElevationMap& map) {
  grid_map_msgs::GridMap msg;
  msg.info.header.stamp = detail::toStamp(map.getTimestamp());
  msg.info.header.frame_id = map.getFrameId();
  fastdem::detail::fillGridMapMsg<grid_map_msgs::GridMap, std_msgs::Float32MultiArray>(
      msg, map);
  return msg;
}

/// Create map boundary marker for RViz visualization.
inline visualization_msgs::Marker toMapBoundary(const ElevationMap& map) {
  return fastdem::detail::toMarkerImpl<visualization_msgs::Marker,
                              geometry_msgs::Point>(
      map, detail::toStamp(map.getTimestamp()));
}

/// Convert ElevationMap normal vectors to MarkerArray for RViz visualization.
inline visualization_msgs::MarkerArray toNormalMarkers(
    const ElevationMap& map, float arrow_length = 0.15f, int stride = 1) {
  return fastdem::detail::toNormalMarkersImpl<
      visualization_msgs::MarkerArray, visualization_msgs::Marker,
      geometry_msgs::Point, std_msgs::ColorRGBA>(
      map, detail::toStamp(map.getTimestamp()), arrow_length, stride);
}

}  // namespace fastdem::ros1

#endif  // FASTDEM_BRIDGE_ROS1_HPP
