/*
 * height_mapping_node.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping_core/mapping_engine.h"
#include "height_mapping_ros/adapters/pointcloud_converter.h"
#include "height_mapping_ros/adapters/tf2_transform.h"
#include "height_mapping_ros/params.h"

namespace height_mapping::ros {

class MappingNode {
public:
  MappingNode() : private_nh_("~"), params_(private_nh_) { initialize(); }

private:
  void initialize() {
    ROS_INFO("Initializing Height Mapping Node...");

    setupTransformProvider();
    setupROSInterfaces();
    createMappingEngine();

    printNodeInfo();
    ROS_INFO("Height Mapping Node successfully initialized!");
  }

  void setupTransformProvider() {
    tf_ = std::make_shared<TF2Transform>();
    tf_->setLookupTimeout(params_.tf_timeout_s);
    tf_->setMaxExtrapolationTime(params_.tf_extrapolation_s);

    ROS_INFO("Tf2 Transform ready [OK]");
  }

  void setupROSInterfaces() {
    auto &nh = nodeHandle_;

    // Input
    point_cloud_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(
        params_.input_cloud_topic, 10, &MappingNode::pointCloudCallback, this);

    // Outputs
    gridmap_publisher_ =
        nh.advertise<grid_map_msgs::GridMap>(params_.output_map_topic, 1, true);

    if (params_.publish_processed_cloud) {
      processed_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(
          params_.processed_cloud_topic, 1);
    }

    // Timer for periodic publishing
    if (params_.publish_rate_hz > 0) {
      publish_timer_ =
          nh.createTimer(::ros::Duration(1.0 / params_.publish_rate_hz),
                         &MappingNode::publishTimerCallback, this);
    }

    ROS_INFO("ROS interfaces ready [OK]");
  }

  void createMappingEngine() {
    try {
      mapper_ =
          std::make_unique<core::MappingEngine>(tf_, params_.mapping_config);
    } catch (const std::exception &e) {
      ROS_ERROR("Failed to create mapping engine from config: %s", e.what());
      throw;
    }

    ROS_INFO("Mapping engine ready [OK]");
  }

  void printNodeInfo() const {
    // Extract filename from path
    size_t last_slash = params_.mapping_config.find_last_of("/");
    std::string config_filename =
        (last_slash != std::string::npos)
            ? params_.mapping_config.substr(last_slash + 1)
            : params_.mapping_config;

    ROS_INFO(" ");
    ROS_INFO("===== Configuration Summary =====");
    ROS_INFO("Configuration:");
    ROS_INFO("  Mapping config: %s", config_filename.c_str());
    ROS_INFO("Topics:");
    ROS_INFO("  Input:  %s", point_cloud_subscriber_.getTopic().c_str());
    ROS_INFO("  Output: %s", gridmap_publisher_.getTopic().c_str());
    ROS_INFO("Processing:");
    ROS_INFO("  Publish rate: %.1f Hz", params_.publish_rate_hz);
    ROS_INFO("=================================");
    ROS_INFO(" ");
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    try {
      auto cloud = adapters::fromROS(*msg);
      mapper_->registerCloud(cloud);

      publishProcessedCloudIfRequested(msg);
    } catch (const std::exception &e) {
      ROS_ERROR_THROTTLE(1.0, "Error processing point cloud: %s", e.what());
    }
  }

  void publishProcessedCloudIfRequested(
      const sensor_msgs::PointCloud2::ConstPtr &original_msg) {
    // Check if publication is enabled and has subscribers
    if (!params_.publish_processed_cloud) {
      return;
    }
    if (processed_cloud_publisher_.getNumSubscribers() == 0) {
      return;
    }
    auto processed_cloud = mapper_->getProcessedCloud();
    if (!processed_cloud) {
      return;
    }

    sensor_msgs::PointCloud2 cloud_msg = adapters::toROS(*processed_cloud);
    processed_cloud_publisher_.publish(cloud_msg);
  }

  void publishTimerCallback(const ::ros::TimerEvent &event) {
    publishHeightMap();
  }

  void publishHeightMap() {
    if (gridmap_publisher_.getNumSubscribers() == 0) {
      return;
    }
    auto height_map = mapper_->getHeightMap();
    if (!height_map) {
      return;
    }

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(height_map->getGridMap(), msg);
    gridmap_publisher_.publish(msg);
  }

private:
  // ROS handles
  ::ros::NodeHandle nodeHandle_;
  ::ros::NodeHandle private_nh_;

  // Configuration
  Parameters params_;

  // Core components
  std::shared_ptr<TF2Transform> tf_;
  std::unique_ptr<core::MappingEngine> mapper_;

  // ROS interfaces
  ::ros::Subscriber point_cloud_subscriber_;
  ::ros::Publisher processed_cloud_publisher_;
  ::ros::Publisher gridmap_publisher_;
  ::ros::Timer publish_timer_;
};

} // namespace height_mapping::ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "height_mapping_node");

  try {
    height_mapping::ros::MappingNode node;
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("Fatal error: %s", e.what());
    return 1;
  }

  return 0;
}