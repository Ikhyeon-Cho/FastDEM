// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>
#include <visualization_msgs/Marker.h>

#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <memory>
#include <shared_mutex>

#include "fastdem_ros/conversions.hpp"
#include "fastdem_ros/parameters.hpp"
#include "fastdem_ros/tf_bridge.hpp"

namespace fastdem::ros1 {

using fastdem::ElevationMap;
using fastdem::FastDEM;
using fastdem::PointCloud;

class MappingNode {
 public:
  MappingNode() : nh_{"~"} {}

  bool initialize() {
    if (!loadParameters()) return false;

    setupCore();
    setupPublishersAndSubscribers();
    printNodeSummary();

    spdlog::info("FastDEM node initialized successfully!");
    return true;
  }

 private:
  bool loadParameters() {
    std::string config_path;
    nh_.param<std::string>("config_file", config_path, std::string{});
    try {
      cfg_ = NodeConfig::load(config_path);
    } catch (const std::exception& e) {
      spdlog::error("Failed to load config: {}", e.what());
      return false;
    }

    // Launch arg override: input_scan
    std::string input_scan;
    if (nh_.getParam("input_scan", input_scan)) {
      cfg_.topics.input_scan = input_scan;
    }

    spdlog::set_level(spdlog::level::from_str(cfg_.logger_level));
    return true;
  }

  void setupCore() {
    // Map geometry
    map_.setGeometry(cfg_.map.width, cfg_.map.height, cfg_.map.resolution);
    map_.setFrameId(cfg_.tf.map_frame);

    // Mapper
    mapper_ = std::make_unique<FastDEM>(map_, cfg_.pipeline);
    mapper_->setMappingMode(cfg_.mapping_mode);

    // Set scan filter
    mapper_->setHeightFilter(cfg_.point_filter.z_min, cfg_.point_filter.z_max);
    mapper_->setRangeFilter(cfg_.point_filter.range_min,
                            cfg_.point_filter.range_max);

    // TF bridge implements both Calibration and Odometry
    auto ros_tf = std::make_shared<TFBridge>(
        cfg_.tf.base_frame, cfg_.tf.map_frame,  //
        cfg_.tf.max_wait_time, cfg_.tf.max_stale_time);
    mapper_->setTransformSystem(ros_tf);

    // Register callback to publish processed cloud
    mapper_->setProcessedCloudCallback(
        [this](const PointCloud& cloud) { publishProcessedScan(cloud); });
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    scan_sub_ = nh_.subscribe(cfg_.topics.input_scan, 10, &MappingNode::scanCallback, this);
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan", 1);
    if (cfg_.mapping_mode != fastdem::MappingMode::GLOBAL)
      gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("gridmap", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
    boundary_pub_ = nh_.advertise<visualization_msgs::Marker>("map_boundary", 1);
    map_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.topics.publish_rate),
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
    if (cfg_.inpainting.enabled)
      applyInpainting(map_, cfg_.inpainting.max_iterations,
                      cfg_.inpainting.min_valid_neighbors);
    if (cfg_.feature_extraction.enabled)
      applyFeatureExtraction(map_, cfg_.feature_extraction.analysis_radius,
                             cfg_.feature_extraction.min_valid_neighbors);
  }

  // ==================== Publishers ====================

  void publishMap(const ros::TimerEvent&) {
    const bool want_map = map_pub_.getNumSubscribers() > 0;
    const bool want_gridmap = gridmap_pub_.getNumSubscribers() > 0;
    const bool want_boundary = boundary_pub_.getNumSubscribers() > 0;
    if (!want_map && !want_gridmap && !want_boundary) return;

    std::shared_lock lock(map_mutex_);
    if (want_map) {
      map_pub_.publish(ros1::toPointCloud2(map_));
    }
    if (want_gridmap) {
      gridmap_pub_.publish(ros1::toGridMap(map_));
    }
    if (want_boundary) {
      boundary_pub_.publish(ros1::toMapBoundary(map_));
    }
  }

  void publishProcessedScan(const PointCloud& cloud) {
    if (scan_pub_.getNumSubscribers() == 0) return;
    scan_pub_.publish(nanopcl::to(cloud));
  }

  void printNodeSummary() const {
    spdlog::info("");
    spdlog::info("===== FastDEM Mapping Node =====");
    spdlog::info("  Input    : {}", scan_sub_.getTopic());
    spdlog::info("  Output   : {}", map_pub_.getTopic());
    spdlog::info("  Pub rate : {} Hz", cfg_.topics.publish_rate);
    spdlog::info("================================");
    spdlog::info("");
  }

  ros::NodeHandle nh_;
  NodeConfig cfg_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  mutable std::shared_mutex map_mutex_;

  // ROS handles
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher map_pub_;
  ros::Publisher gridmap_pub_;
  ros::Publisher boundary_pub_;
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
