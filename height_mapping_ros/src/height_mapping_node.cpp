/*
 * height_mapping_node.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
*/

#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping_core/config/config_loader.h" // For YAML-based Core config
#include "height_mapping_core/engine/height_mapping_engine.h"
#include "height_mapping_ros/adapters/gridmap_publisher.h"
#include "height_mapping_ros/adapters/tf2_transform_provider.h"
#include "height_mapping_ros/node_config.h"
#include "height_mapping_ros/utils/pointcloud_converter.h"

namespace height_mapping {
namespace ros {

class MappingNode {
public:
  MappingNode() : private_nh_("~") { initialize(); }

private:
  void initialize() {

    ROS_INFO("Initializing Height Mapping Node...");

    // 1. Load ROS-specific configuration from parameter server
    ros_config_ = MappingNodeConfig::loadFromROS(private_nh_);
    if (!ros_config_.validate()) {
      throw std::runtime_error("Invalid ROS configuration");
    }
    ROS_INFO("Loaded ROS config from parameter server");

    // 2. Create and configure transform provider via ROS TF2
    auto transform_provider = std::make_shared<TF2TransformProvider>();
    transform_provider->setLookupTimeout(ros_config_.transform.lookup_timeout);
    transform_provider->setMaxExtrapolationTime(
        ros_config_.transform.extrapolation_time);

    // 3. Create mapping engine - it loads its own configuration
    std::string config_file = resolveConfigPath(ros_config_.core_config_file);
    mapping_engine_ =
        core::MappingEngine::createFromFile(config_file, transform_provider);
    ROS_INFO("Created mapping engine with config: %s", config_file.c_str());

    // 4. Setup ROS interfaces (pub/sub)
    setupROS();

    printConfigSummary();

    ROS_INFO("Height Mapping Node initialized");
  }

  std::string resolveConfigPath(const std::string &path) {
    std::string resolved_path = path;

    // Handle package:// URLs
    if (path.find("package://") == 0) {
      size_t pos = path.find('/', 10);
      std::string package = path.substr(10, pos - 10);
      std::string relative_path = path.substr(pos);

      try {
        resolved_path = ::ros::package::getPath(package) + relative_path;
      } catch (const std::exception &e) {
        ROS_ERROR("Failed to resolve package path: %s", e.what());
        throw;
      }
    }

    return resolved_path;
  }

  void printConfigSummary() const {
    ROS_INFO("===== Configuration Summary =====");
    ROS_INFO("ROS Parameters (from parameter server):");
    ROS_INFO("  - Topics: %s -> %s", ros_config_.topics.point_cloud.c_str(),
             ros_config_.topics.height_map.c_str());
    ROS_INFO("  - Publishing rate: %.1f Hz", ros_config_.publishing.rate);
    ROS_INFO("  - Queue size: %d", ros_config_.processing.queue_size);
    ROS_INFO("  - TF timeout: %.2fs", ros_config_.transform.lookup_timeout);
    ROS_INFO("=================================");
  }

  void setupROS() {
    // Subscribe to point cloud
    point_cloud_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>(
        ros_config_.topics.point_cloud, ros_config_.processing.queue_size,
        &MappingNode::pointCloudCallback, this);

    // Setup publishers
    gridmap_publisher_ = std::make_unique<GridMapPublisher>(
        node_handle_, ros_config_.topics.height_map);
    gridmap_publisher_->enableMarkers(ros_config_.publishing.publish_markers);

    if (ros_config_.publishing.publish_processed_cloud) {
      processed_cloud_publisher_ =
          node_handle_.advertise<sensor_msgs::PointCloud2>(
              ros_config_.topics.processed_cloud, 1);
    }

    // Setup publish timer
    if (ros_config_.publishing.rate > 0) {
      publish_timer_ = node_handle_.createTimer(
          ::ros::Duration(1.0 / ros_config_.publishing.rate),
          &MappingNode::publishTimerCallback, this);
    }

    ROS_INFO("ROS interfaces setup complete");
    ROS_INFO("  Subscribed to: %s", ros_config_.topics.point_cloud.c_str());
    ROS_INFO("  Publishing map to: %s", ros_config_.topics.height_map.c_str());
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    try {
      // 1. Convert ROS message to Core data type
      core::PointCloudXYZ cloud;
      utils::PointCloudConverter::rosToCore(*msg, cloud);

      // 2. Integrate cloud into the height map (move since cloud not used after)
      mapping_engine_->integrateCloud(std::move(cloud));

      // 3. Publish processed cloud if enabled and available
      if (ros_config_.publishing.publish_processed_cloud &&
          processed_cloud_publisher_.getNumSubscribers() > 0) {
        auto processed_cloud = mapping_engine_->getProcessedCloud();
        if (processed_cloud) {
          sensor_msgs::PointCloud2 cloud_msg =
              utils::PointCloudConverter::coreToROS(*processed_cloud);
          cloud_msg.header.stamp = msg->header.stamp;
          cloud_msg.header.frame_id = processed_cloud->frame_id;
          processed_cloud_publisher_.publish(cloud_msg);
        }
      }

      // Log statistics
      auto statistics = mapping_engine_->getStatistics();
      if (statistics.clouds_processed > 0) {
        ROS_DEBUG_THROTTLE(5.0, "Processed %zu clouds, avg time: %.2f ms",
                           statistics.clouds_processed,
                           statistics.avg_processing_time_ms);
      }

    } catch (const std::exception &e) {
      ROS_ERROR_THROTTLE(1.0, "Error processing point cloud: %s", e.what());
    }
  }

  void publishTimerCallback(const ::ros::TimerEvent &event) {
    // Get the current map from engine
    auto height_map = mapping_engine_->getHeightMap();

    if (height_map && gridmap_publisher_) {
      gridmap_publisher_->publish(*height_map);
    }
  }

private:
  // ROS handles
  ::ros::NodeHandle node_handle_;
  ::ros::NodeHandle private_nh_;

  // ROS interfaces
  ::ros::Subscriber point_cloud_subscriber_;
  ::ros::Publisher processed_cloud_publisher_;
  std::unique_ptr<GridMapPublisher> gridmap_publisher_;
  ::ros::Timer publish_timer_;

  // Core engine
  std::unique_ptr<core::MappingEngine> mapping_engine_;

  // ROS Node Configuration
  MappingNodeConfig ros_config_;
};

} // namespace ros
} // namespace height_mapping

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