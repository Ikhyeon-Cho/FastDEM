// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>
#include <visualization_msgs/Marker.h>

#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <filesystem>
#include <memory>
#include <nanopcl/bridge/ros1.hpp>
#include <shared_mutex>

#include "fastdem_ros/conversions.hpp"
#include "fastdem_ros/parameters.hpp"
#include "fastdem_ros/tf_bridge.hpp"

namespace fastdem::ros1 {

namespace fs = std::filesystem;
using fastdem::ElevationMap;
using fastdem::FastDEM;
using fastdem::PointCloud;

class MappingNode {
 public:
  MappingNode() : nh_{"~"} {}

  bool initialize() {
    if (!loadParameters()) return false;
    if (!setupMapping()) return false;

    setupPublishersAndSubscribers();
    printNodeSummary();

    spdlog::info("FastDEM node initialized successfully!");
    return true;
  }

 private:
  bool loadParameters() {
    params_ = NodeParameters::load(nh_);
    if (!params_.isValid()) {
      spdlog::error("Invalid node parameters. Check your launch/config file.");
      return false;
    }
    spdlog::set_level(spdlog::level::from_str(params_.logger_level));
    return true;
  }

  bool setupMapping() {
    try {
      // Prepare ElevationMap
      map_.setGeometry(params_.map.width, params_.map.height,
                       params_.map.resolution);
      map_.setFrameId(params_.tf.map_frame);

      // Load configuration
      auto cfg = fastdem::loadConfig(params_.mapping_config);
      post_cfg_ = cfg.postprocess;

      // Initialize FastDEM
      mapper_ = std::make_unique<FastDEM>(map_, cfg.core);

      // Override mapping mode if specified in ROS params
      if (params_.mapping_mode == "global")
        mapper_->setMappingMode(fastdem::MappingMode::GLOBAL);
      else if (params_.mapping_mode == "local")
        mapper_->setMappingMode(fastdem::MappingMode::LOCAL);

      // TF bridge implements both Calibration and Odometry
      auto tf_bridge = std::make_shared<TFBridge>(params_.tf.base_frame,  //
                                                  params_.tf.map_frame,
                                                  params_.tf.max_wait_time,
                                                  params_.tf.max_stale_time);
      mapper_->setTransformSystem(tf_bridge);

      // Register callback to publish processed cloud
      mapper_->setProcessedCloudCallback(
          [this](const PointCloud& cloud) { publishProcessedScan(cloud); });

      spdlog::info("FastDEM mapper initialized [OK]");
    } catch (const std::exception& e) {
      spdlog::error("FastDEM initialization failed: {}", e.what());
      return false;
    }
    return true;
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    scan_sub_ = nh_.subscribe(params_.topics.input_scan, 10, &MappingNode::scanCallback, this);
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan/filtered", 1);
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map/gridmap", 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map/cloud", 1);
    region_pub_ = nh_.advertise<visualization_msgs::Marker>("map/region", 1);
    map_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / params_.topics.publish_rate),
                                          &MappingNode::publishMap, this);
    // clang-format on
  }

  // ==================== Callbacks ====================

  void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      first_scan = false;
    }

    auto cloud = std::make_shared<PointCloud>(nanopcl::from(*msg));
    std::unique_lock lock(map_mutex_);
    mapper_->integrate(cloud);

    // Post-processing (user-side, map-only operations)
    const auto& inp = post_cfg_.inpainting;
    if (inp.enabled)
      applyInpainting(map_, inp.max_iterations, inp.min_valid_neighbors);
    const auto& fe = post_cfg_.feature_extraction;
    if (fe.enabled)
      applyFeatureExtraction(map_, fe.analysis_radius, fe.min_valid_neighbors);
  }

  // ==================== Publishers ====================

  void publishMap(const ros::TimerEvent&) {
    const bool want_gridmap = map_pub_.getNumSubscribers() > 0;
    const bool want_cloud = cloud_pub_.getNumSubscribers() > 0;
    const bool want_region = region_pub_.getNumSubscribers() > 0;
    if (!want_gridmap && !want_cloud && !want_region) return;

    std::shared_lock lock(map_mutex_);
    if (want_gridmap) {
      grid_map_msgs::GridMap msg;
      toMessage(map_, msg);
      map_pub_.publish(msg);
    }
    if (want_cloud) {
      cloud_pub_.publish(toPointCloud2(map_));
    }
    if (want_region) {
      region_pub_.publish(toRegionMarker(map_));
    }
  }

  void publishProcessedScan(const PointCloud& cloud) {
    if (scan_filtered_pub_.getNumSubscribers() == 0) return;
    scan_filtered_pub_.publish(nanopcl::to(cloud));
  }

  void printNodeSummary() const {
    std::string config_name = fs::path(params_.mapping_config).filename();

    spdlog::info("");
    spdlog::info("===== FastDEM Node =====");
    spdlog::info("  Config   : {}", config_name);
    spdlog::info("  Input    : {}", scan_sub_.getTopic());
    spdlog::info("  Output   : {}", map_pub_.getTopic());
    spdlog::info("  Pub rate : {} Hz", params_.topics.publish_rate);
    spdlog::info("===============================");
    spdlog::info("");
  }

  ros::NodeHandle nh_;
  NodeParameters params_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  fastdem::PostProcessConfig post_cfg_;
  mutable std::shared_mutex map_mutex_;

  // ROS handles
  ros::Subscriber scan_sub_;
  ros::Publisher scan_filtered_pub_;
  ros::Publisher map_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher region_pub_;
  ros::Timer map_pub_timer_;
};

}  // namespace fastdem::ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastdem_ros_node");
  fastdem::ros1::MappingNode node;
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
