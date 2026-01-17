/*
 * ppl_mapping_node.cpp
 *
 * Example node using lab::Mapper (pipeline-based, experimental).
 * For researchers and advanced users who want YAML-configurable pipelines.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <height_mapping/height_mapping.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <memory>
#include <nanopcl/bridge/ros1.hpp>

#include "height_mapping_ros/conversions.h"
#include "height_mapping_ros/parameters.h"
#include "height_mapping_ros/tf.h"

namespace fs = std::filesystem;
using namespace height_mapping;
using PplMapper = height_mapping::lab::Mapper;

namespace height_mapping_ros {

class PplMappingNode {
 public:
  PplMappingNode() : nh_{"~"} {}

  bool initialize() {
    if (!loadParameters()) return false;
    if (!setupTFLookup()) return false;
    if (!setupMapper()) return false;

    setupPublishersAndSubscribers();
    printNodeSummary();

    spdlog::info("PPL mapping node initialized successfully!");
    return true;
  }

 private:
  using MapperPtr = std::unique_ptr<PplMapper>;
  using TFLookupPtr = std::shared_ptr<TFLookup>;

  bool loadParameters() {
    params_ = NodeParameters::load(nh_);
    if (!params_.isValid()) {
      spdlog::error("Invalid node parameters. Check your launch/config file.");
      return false;
    }
    spdlog::set_level(spdlog::level::from_str(params_.logger_level));
    return true;
  }

  bool setupTFLookup() {
    tf_lookup_ = std::make_shared<TFLookup>(params_.tf.base_frame,  //
                                            params_.tf.map_frame,
                                            params_.tf.max_wait_time,
                                            params_.tf.max_stale_time);
    spdlog::info("TF Lookup ready [OK]");
    return true;
  }

  bool setupMapper() {
    IExtrinsicsProvider::Ptr extrinsics = tf_lookup_;
    IRobotPoseProvider::Ptr pose_provider = tf_lookup_;

    try {
      auto config = PplMapper::Config::load(params_.config_path);
      mapper_ = std::make_unique<PplMapper>(config, extrinsics, pose_provider);
      spdlog::info("lab::Mapper initialized [OK]");
    } catch (const std::exception& e) {
      spdlog::error("Mapper initialization failed: {}", e.what());
      return false;
    }
    return true;
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    scan_sub_ = nh_.subscribe(params_.topics.input_scan, 10, &PplMappingNode::scanCallback, this);
    scan_processed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan/processed", 1);
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map/gridmap", 1);
    map_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / params_.topics.publish_rate),
                                          &PplMappingNode::publishMap, this);
    // clang-format on
  }

  void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      first_scan = false;
    }

    auto cloud = std::make_shared<PointCloud>(npcl::from(*msg));
    mapper_->update(cloud);
    publishProcessedScan(*cloud);
  }

  void publishMap(const ros::TimerEvent&) {
    if (map_pub_.getNumSubscribers() == 0) return;

    grid_map_msgs::GridMap msg;
    toMessage(mapper_->map(), msg);
    map_pub_.publish(msg);
  }

  void publishProcessedScan(const PointCloud& cloud) {
    if (scan_processed_pub_.getNumSubscribers() == 0) return;
    scan_processed_pub_.publish(npcl::to<sensor_msgs::PointCloud2>(cloud));
  }

  void printNodeSummary() const {
    std::string config_name = fs::path(params_.config_path).filename();

    spdlog::info("");
    spdlog::info("===== PPL Mapping Node (Experimental) =====");
    spdlog::info("  Config   : {}", config_name);
    spdlog::info("  Input    : {}", scan_sub_.getTopic());
    spdlog::info("  Output   : {}", map_pub_.getTopic());
    spdlog::info("  Pub rate : {} Hz", params_.topics.publish_rate);
    spdlog::info("============================================");
    spdlog::info("");
  }

  ros::NodeHandle nh_;

  NodeParameters params_;
  TFLookupPtr tf_lookup_;
  MapperPtr mapper_;

  ros::Subscriber scan_sub_;
  ros::Publisher scan_processed_pub_;
  ros::Publisher map_pub_;
  ros::Timer map_pub_timer_;
};

}  // namespace height_mapping_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "ppl_mapping_node");
  height_mapping_ros::PplMappingNode node;
  if (!node.initialize()) {
    return 1;
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
