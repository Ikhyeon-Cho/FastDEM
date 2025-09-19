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

#include "height_mapping_core/data/point_cloud.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace height_mapping {
namespace utils {

class PointCloudConverter {
public:
  // Convert ROS message to core point cloud (optimized, no PCL intermediate)
  static void rosToCore(const sensor_msgs::PointCloud2 &ros_msg,
                        core::PointCloudXYZ &core_cloud);

  // Convert core point cloud to ROS message
  static sensor_msgs::PointCloud2
  coreToROS(const core::PointCloudXYZ &core_cloud);

  // Convert PCL to core point cloud
  template <typename PointT>
  static void pclToCore(const pcl::PointCloud<PointT> &pcl_cloud,
                        core::PointCloudXYZ &core_cloud) {
    core_cloud.timestamp = pcl_cloud.header.stamp * 1000; // Convert us to ns
    core_cloud.frame_id = pcl_cloud.header.frame_id;
    core_cloud.points.clear();
    core_cloud.points.reserve(pcl_cloud.points.size());

    for (const auto &pt : pcl_cloud.points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        core_cloud.points.emplace_back(pt.x, pt.y, pt.z);
      }
    }
  }

  // Convert core to PCL point cloud
  template <typename PointT>
  static void coreToPCL(const core::PointCloudXYZ &core_cloud,
                        pcl::PointCloud<PointT> &pcl_cloud) {
    pcl_cloud.header.frame_id = core_cloud.frame_id;
    pcl_cloud.header.stamp = core_cloud.timestamp / 1000; // Convert ns to us
    pcl_cloud.points.clear();
    pcl_cloud.points.reserve(core_cloud.points.size());

    for (const auto &pt : core_cloud.points) {
      PointT pcl_pt;
      pcl_pt.x = pt.x;
      pcl_pt.y = pt.y;
      pcl_pt.z = pt.z;
      pcl_cloud.points.push_back(pcl_pt);
    }

    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
  }

  // Get point count from ROS message without conversion
  static size_t getPointCount(const sensor_msgs::PointCloud2 &msg) {
    return msg.width * msg.height;
  }

  // Validate ROS message
  static bool validate(const sensor_msgs::PointCloud2 &msg);
};

} // namespace utils
} // namespace height_mapping

#endif // HEIGHT_MAPPING_POINTCLOUD_CONVERTER_H