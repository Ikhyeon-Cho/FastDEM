/*
 * height_mapping_node.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>

#include "height_mapping/height_mapping.h"
#include "height_mapping_ros/adapters/pointcloud2.h"
#include "height_mapping_ros/adapters/tf2.h"
#include "height_mapping_ros/node_parameters.h"

namespace fs = std::filesystem;
using namespace height_mapping_ros;

namespace height_mapping {

class MappingNode {
 public:
  MappingNode() : nh_{"~"} {}

  bool initialize() {
    if (!loadParameters()) return false;
    if (!setupTF2()) return false;
    if (!setupMapper()) return false;

    setupPublishersAndSubscribers();
    printNodeSummary();

    spdlog::info("Height mapping node initialized successfully!");
    return true;
  }

 private:
  // Type Aliases
  using MapperPtr = std::unique_ptr<IMapper>;
  using TF2Ptr = std::shared_ptr<adapters::TF2>;

  bool loadParameters() {
    params_ = NodeParameters::load(nh_);
    if (!params_.isValid()) {
      spdlog::error("Invalid node parameters. Check your launch/config file.");
      return false;
    }
    spdlog::set_level(spdlog::level::from_str(params_.logger_level));
    return true;
  }

  bool setupTF2() {
    tf2_ = std::make_shared<adapters::TF2>(params_.tf);
    spdlog::info("TF2 adapter initialized [OK]");
    return true;
  }

  bool setupMapper() {
    // ISP: Single TF system viewed through two semantic interfaces
    // ROS TF address both extrinsics and robot pose in same system
    IExtrinsicsProvider::Ptr extrinsics = tf2_;
    IRobotPoseProvider::Ptr pose_provider = tf2_;

    try {
      if (params_.use_configurable_mapper) {
        // ppl::HeightMapper (experimental)
        auto config = ppl::HeightMapper::Config::load(params_.config_path);
        mapper_ = std::make_unique<ppl::HeightMapper>(config, extrinsics,
                                                      pose_provider);
        spdlog::info("ppl::HeightMapper initialized [OK]");
      } else {
        // HeightMapper (Standard)
        auto config = HeightMapper::Config::load(params_.config_path);
        mapper_ =
            std::make_unique<HeightMapper>(config, extrinsics, pose_provider);
        spdlog::info("HeightMapper initialized [OK]");
      }
    } catch (const std::exception& e) {
      spdlog::error("Mapper initialization failed: {}", e.what());
      return false;
    }
    return true;
  }

  void setupPublishersAndSubscribers() {
    scan_sub_ = nh_.subscribe(params_.topics.input_scan, 10,
                              &MappingNode::onScanReceived, this);

    processed_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("scan/processed", 1);

    height_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map/gridmap", 1);

    map_publish_timer_ =
        nh_.createTimer(ros::Duration(1.0 / params_.topics.publish_rate),
                        &MappingNode::publishMap, this);
  }

  // ==================== Callbacks ====================

  void onScanReceived(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      first_scan = false;
    }

    // Zero-copy: adapter returns shared_ptr, integrate accepts shared_ptr
    auto scan_cloud = adapters::fromPointCloud2(*msg);
    mapper_->integrate(scan_cloud);
    publishDebugData();
  }

  void publishMap(const ros::TimerEvent&) {
    // Skip if no subscribers
    if (height_map_pub_.getNumSubscribers() == 0) return;

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(mapper_->getHeightMap(), msg);
    height_map_pub_.publish(msg);
  }

  // ==================== Helpers ====================

  void publishDebugData() {
    // TODO: Debug output functionality removed during ppl migration
    // Re-implement if needed using new pipeline architecture
  }

  void printNodeSummary() const {
    // C++17 filesystem for clean path parsing
    std::string config_name = fs::path(params_.config_path).filename();
    std::string mapper_type = mapper_->name();

    spdlog::info("");
    spdlog::info("===== Height Mapping Node =====");
    spdlog::info("  Mapper   : {}", mapper_type);
    spdlog::info("  Config   : {}", config_name);
    spdlog::info("  Input    : {}", scan_sub_.getTopic());
    spdlog::info("  Output   : {}", height_map_pub_.getTopic());
    spdlog::info("  Pub rate : {} Hz", params_.topics.publish_rate);
    spdlog::info("===============================");
    spdlog::info("");
  }

  ros::NodeHandle nh_;

  // Core objects
  NodeParameters params_;
  TF2Ptr tf2_;
  MapperPtr mapper_;

  // ROS handles
  ros::Subscriber scan_sub_;
  ros::Publisher processed_cloud_pub_;
  ros::Publisher height_map_pub_;
  ros::Timer map_publish_timer_;
};

}  // namespace height_mapping

int main(int argc, char** argv) {
  ros::init(argc, argv, "height_mapping_node");
  height_mapping::MappingNode node;
  if (!node.initialize()) {
    return 1;
  }

  // Multi-threaded spinner for parallel processing:
  // Thread 1: Point cloud processing (mapping)
  // Thread 2: Map publishing (visualization)
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
