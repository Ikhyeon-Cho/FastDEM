/*
 * conversions.h
 *
 * ROS message conversion utilities.
 */

#ifndef HEIGHT_MAPPING_ROS_CONVERSIONS_H
#define HEIGHT_MAPPING_ROS_CONVERSIONS_H

#include <grid_map_msgs/GridMap.h>
#include <height_mapping/core.h>
#include <std_msgs/Float32MultiArray.h>

namespace height_mapping_ros {

/**
 * @brief Convert HeightMap to ROS message
 * @param map Source height map
 * @param msg Output ROS message
 */
inline void toMessage(const height_mapping::HeightMap& map,
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

}  // namespace height_mapping_ros

#endif  // HEIGHT_MAPPING_ROS_CONVERSIONS_H
