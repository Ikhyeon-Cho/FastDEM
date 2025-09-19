/*
 * gridmap_publisher.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_ADAPTERS_GRIDMAP_PUBLISHER_H
#define HEIGHT_MAPPING_ROS_ADAPTERS_GRIDMAP_PUBLISHER_H

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <string>

#include "height_map_core/height_map.h"
#include "height_mapping_core/data/point_cloud.h"

namespace height_mapping::ros {

class GridMapPublisher {
public:
  GridMapPublisher(::ros::NodeHandle &nh,
                   const std::string &map_topic = "height_map",
                   const std::string &frame_id = "map")
      : frame_id_(frame_id) {

    map_publisher_ = nh.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);
    marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(
        map_topic + "_markers", 1, true);
    cloud_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>(map_topic + "_cloud", 1, true);

    ROS_INFO("Publishing height map to: %s", map_topic.c_str());
  }

  ~GridMapPublisher() = default;

  // Publish the height map
  void publish(const height_map::HeightMap &map) {
    if (!shouldPublish())
      return;

    publishGridMap(map);

    if (publish_markers_) {
      publishMarkers(map);
    }

    if (publish_cloud_) {
      publishPointCloud(map);
    }
  }

  // Configuration
  void setFrameId(const std::string &frame_id) { frame_id_ = frame_id; }

  void enableMarkers(bool enable) { publish_markers_ = enable; }

  void enablePointCloud(bool enable) { publish_cloud_ = enable; }

  void setPublishRate(double rate_hz) {
    if (rate_hz > 0) {
      min_publish_interval_ = ::ros::Duration(1.0 / rate_hz);
    }
  }

  // Check if anyone is listening
  bool hasSubscribers() const {
    return map_publisher_.getNumSubscribers() > 0 ||
           marker_publisher_.getNumSubscribers() > 0 ||
           cloud_publisher_.getNumSubscribers() > 0;
  }

private:
  bool shouldPublish() const {
    // Only check if there are subscribers
    // Rate limiting removed - we control rate at the source
    return hasSubscribers();
  }

  void publishGridMap(const height_map::HeightMap &map) {
    // Convert to ROS message using grid_map_ros
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map.getGridMap(), msg);

    // Update header
    msg.info.header.stamp = ::ros::Time::now();
    msg.info.header.frame_id = frame_id_;

    map_publisher_.publish(msg);
    // last_publish_time_ removed - no longer needed without rate limiting
  }

  void publishMarkers(const height_map::HeightMap &map) {
    visualization_msgs::MarkerArray markers;

    // Create height visualization as colored cubes
    visualization_msgs::Marker height_marker;
    height_marker.header.frame_id = frame_id_;
    height_marker.header.stamp = ::ros::Time::now();
    height_marker.ns = "height_map";
    height_marker.id = 0;
    height_marker.type = visualization_msgs::Marker::CUBE_LIST;
    height_marker.action = visualization_msgs::Marker::ADD;

    float resolution = map.getResolution();
    height_marker.scale.x = resolution * 0.9; // Slightly smaller to see gaps
    height_marker.scale.y = resolution * 0.9;
    height_marker.scale.z = 0.01; // Thin layer

    // Get elevation data
    const auto &grid = map.getGridMap();
    if (!grid.exists("elevation"))
      return;

    const auto &elevation = grid["elevation"];
    float min_height = elevation.minCoeffOfFinites();
    float max_height = elevation.maxCoeffOfFinites();
    float height_range = max_height - min_height;

    for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      grid.getPosition(*it, position);

      float height = elevation((*it)(0), (*it)(1));
      if (!std::isfinite(height))
        continue;

      // Add point
      geometry_msgs::Point p;
      p.x = position.x();
      p.y = position.y();
      p.z = height;
      height_marker.points.push_back(p);

      // Color based on height (blue to red)
      std_msgs::ColorRGBA color;
      float normalized =
          (height_range > 0) ? (height - min_height) / height_range : 0.5f;

      color.r = normalized;
      color.g = 1.0f - std::abs(normalized - 0.5f) * 2.0f;
      color.b = 1.0f - normalized;
      color.a = 0.8f;
      height_marker.colors.push_back(color);
    }

    markers.markers.push_back(height_marker);

    // Add variance visualization if available
    if (grid.exists("variance")) {
      visualization_msgs::Marker variance_marker;
      variance_marker.header = height_marker.header;
      variance_marker.ns = "variance";
      variance_marker.id = 1;
      variance_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      variance_marker.action = visualization_msgs::Marker::ADD;
      variance_marker.scale.x = resolution * 0.3;
      variance_marker.scale.y = resolution * 0.3;
      variance_marker.scale.z = resolution * 0.3;

      const auto &variance = grid["variance"];
      for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
        float var = variance((*it)(0), (*it)(1));
        if (!std::isfinite(var) || var <= 0)
          continue;

        grid_map::Position position;
        grid.getPosition(*it, position);
        float height = elevation((*it)(0), (*it)(1));

        geometry_msgs::Point p;
        p.x = position.x();
        p.y = position.y();
        p.z = height + 0.1f; // Slightly above surface
        variance_marker.points.push_back(p);

        // Color based on variance (green = low, red = high)
        std_msgs::ColorRGBA color;
        float normalized_var =
            std::min(var / 0.1f, 1.0f); // Normalize to 0.1m variance
        color.r = normalized_var;
        color.g = 1.0f - normalized_var;
        color.b = 0.0f;
        color.a = 0.5f;
        variance_marker.colors.push_back(color);
      }

      if (!variance_marker.points.empty()) {
        markers.markers.push_back(variance_marker);
      }
    }

    marker_publisher_.publish(markers);
  }

  void publishPointCloud(const height_map::HeightMap &map) {
    // Convert height map to point cloud for compatibility with other tools
    sensor_msgs::PointCloud2 cloud_msg;

    const auto &grid = map.getGridMap();
    if (!grid.exists("elevation"))
      return;

    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    const auto &elevation = grid["elevation"];
    const auto *variance_data =
        grid.exists("variance") ? &grid["variance"] : nullptr;

    for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      float height = elevation((*it)(0), (*it)(1));
      if (!std::isfinite(height))
        continue;

      grid_map::Position position;
      grid.getPosition(*it, position);

      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = height;

      // Use variance as intensity if available
      if (variance_data) {
        float var = (*variance_data)((*it)(0), (*it)(1));
        point.intensity = std::isfinite(var) ? var : 0.0f;
      } else {
        point.intensity = height;
      }

      pcl_cloud.push_back(point);
    }

    // Convert to ROS message
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp = ::ros::Time::now();

    cloud_publisher_.publish(cloud_msg);
  }

private:
  ::ros::Publisher map_publisher_;
  ::ros::Publisher marker_publisher_;
  ::ros::Publisher cloud_publisher_;

  std::string frame_id_;
  bool publish_markers_ = true;
  bool publish_cloud_ = false;

  mutable ::ros::Time last_publish_time_;
  ::ros::Duration min_publish_interval_{0.1}; // Default 10Hz max
};

// Simplified publisher for processed point clouds
class ProcessedCloudPublisher {
public:
  ProcessedCloudPublisher(::ros::NodeHandle &nh,
                          const std::string &topic = "processed_cloud")
      : publisher_(nh.advertise<sensor_msgs::PointCloud2>(topic, 1)) {}

  void publish(const core::PointCloudXYZ &cloud) {
    if (publisher_.getNumSubscribers() == 0)
      return;

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.reserve(cloud.points.size());

    for (const auto &point : cloud.points) {
      pcl_cloud.emplace_back(point.x, point.y, point.z);
    }

    // Convert to ROS message
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.frame_id = cloud.frame_id;
    msg.header.stamp.fromNSec(cloud.timestamp);

    publisher_.publish(msg);
  }

  bool hasSubscribers() const { return publisher_.getNumSubscribers() > 0; }

private:
  ::ros::Publisher publisher_;
};

} // namespace height_mapping::ros

#endif // HEIGHT_MAPPING_ROS_ADAPTERS_GRIDMAP_PUBLISHER_H