/*
 * pointcloud_converter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_POINTCLOUD_CONVERTER_H
#define HEIGHT_MAPPING_POINTCLOUD_CONVERTER_H

#include "height_mapping_core/geometry/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>

namespace height_mapping::ros::adapters {

// Import types from geometry namespace
using PointCloud = geometry::PointCloud;

// Convert ROS message to core point cloud (returns shared_ptr)
std::shared_ptr<PointCloud> fromROS(const sensor_msgs::PointCloud2 &ros_msg);

// Convert core point cloud to ROS message
sensor_msgs::PointCloud2 toROS(const PointCloud &core_cloud);

// Get point count from ROS message without conversion
inline size_t getPointCount(const sensor_msgs::PointCloud2 &msg) {
  return msg.width * msg.height;
}

// Validate ROS message
bool validate(const sensor_msgs::PointCloud2 &msg);

} // namespace height_mapping::ros::adapters

#endif // HEIGHT_MAPPING_POINTCLOUD_CONVERTER_H